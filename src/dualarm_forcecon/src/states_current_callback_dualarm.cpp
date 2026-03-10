#include "DualArmForceControl.h"

#include <cstdio>
#include <cmath>
#include <limits>
#include <map>
#include <array>
#include <algorithm>
#include <cctype>
#include <mutex>

namespace {
// hand force debug cache (canonical row order: THMB, INDX, MIDL, RING, BABY)
std::mutex g_hand_force_dbg_mtx;

// raw scalar from /isaac_contact_states (per finger)
Eigen::Matrix<double,5,1> g_l_hand_raw_scalar = Eigen::Matrix<double,5,1>::Zero();
Eigen::Matrix<double,5,1> g_r_hand_raw_scalar = Eigen::Matrix<double,5,1>::Zero();

// reconstructed force in sensor frame
Eigen::Matrix<double,5,3> g_l_hand_force_sensor = Eigen::Matrix<double,5,3>::Zero();
Eigen::Matrix<double,5,3> g_r_hand_force_sensor = Eigen::Matrix<double,5,3>::Zero();

// converted force in wrist(hand-base) frame (same convention as monitor CUR_F)
Eigen::Matrix<double,5,3> g_l_hand_force_wrist = Eigen::Matrix<double,5,3>::Zero();
Eigen::Matrix<double,5,3> g_r_hand_force_wrist = Eigen::Matrix<double,5,3>::Zero();
} // namespace

// --------------------
// JointsCallback
// --------------------
void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (joint_names_.empty()) { joint_names_ = msg->name; is_initialized_ = true; }

    for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& n = msg->name[i];
        const double p = msg->position[i];

        // arms
        if      (n=="left_joint_1")  q_l_c_(0)=p;
        else if (n=="left_joint_2")  q_l_c_(1)=p;
        else if (n=="left_joint_3")  q_l_c_(2)=p;
        else if (n=="left_joint_4")  q_l_c_(3)=p;
        else if (n=="left_joint_5")  q_l_c_(4)=p;
        else if (n=="left_joint_6")  q_l_c_(5)=p;

        else if (n=="right_joint_1") q_r_c_(0)=p;
        else if (n=="right_joint_2") q_r_c_(1)=p;
        else if (n=="right_joint_3") q_r_c_(2)=p;
        else if (n=="right_joint_4") q_r_c_(3)=p;
        else if (n=="right_joint_5") q_r_c_(4)=p;
        else if (n=="right_joint_6") q_r_c_(5)=p;

        else {
            auto hj = dualarm_forcecon::kin::parseHandJointName(n);
            if (!hj.ok) continue;

            const int idx = hj.finger_id * 4 + hj.joint_id;  // 0..19
            if (idx < 0 || idx >= 20) continue;

            if (hj.is_left)  q_l_h_c_(idx) = p;
            else             q_r_h_c_(idx) = p;
        }
    }
}

// --------------------
// PositionCallback (WRAPPER)
// --------------------
void DualArmForceControl::PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    (void)msg;
    if (!is_initialized_) return;

    ArmPositionCallback(msg);
    HandPositionCallback(msg);
}

// --------------------
// ArmPositionCallback
// --------------------
void DualArmForceControl::ArmPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    (void)msg;
    if (!is_initialized_) return;
    if (!arm_fk_) return;

    constexpr double kPosDeadbandM = 1e-6;
    constexpr double kQuatDeadband = 1e-9;

    auto normalize_quat_msg = [](geometry_msgs::msg::Pose& p) {
        Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
        if (!std::isfinite(q.w()) || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z())) {
            p.orientation.x = 0.0;
            p.orientation.y = 0.0;
            p.orientation.z = 0.0;
            p.orientation.w = 1.0;
            return;
        }
        const double n = q.norm();
        if (n < 1e-12) {
            p.orientation.x = 0.0;
            p.orientation.y = 0.0;
            p.orientation.z = 0.0;
            p.orientation.w = 1.0;
            return;
        }
        q.normalize();
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();
    };

    auto quat_align_to_ref = [](geometry_msgs::msg::Pose& p, const geometry_msgs::msg::Pose& ref) {
        Eigen::Quaterniond q (p.orientation.w,   p.orientation.x,   p.orientation.y,   p.orientation.z);
        Eigen::Quaterniond qr(ref.orientation.w, ref.orientation.x, ref.orientation.y, ref.orientation.z);
        if (q.norm() < 1e-12 || qr.norm() < 1e-12) return;
        q.normalize();
        qr.normalize();
        if (q.dot(qr) < 0.0) {
            q.coeffs() *= -1.0;
        }
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();
    };

    auto apply_pose_deadband = [&](geometry_msgs::msg::Pose& dst,
                                   const geometry_msgs::msg::Pose& src,
                                   bool& is_init_cache)
    {
        if (!is_init_cache) {
            dst = src;
            normalize_quat_msg(dst);
            is_init_cache = true;
            return;
        }

        geometry_msgs::msg::Pose cand = src;
        normalize_quat_msg(cand);
        quat_align_to_ref(cand, dst);

        if (std::fabs(cand.position.x - dst.position.x) >= kPosDeadbandM) dst.position.x = cand.position.x;
        if (std::fabs(cand.position.y - dst.position.y) >= kPosDeadbandM) dst.position.y = cand.position.y;
        if (std::fabs(cand.position.z - dst.position.z) >= kPosDeadbandM) dst.position.z = cand.position.z;

        const double dx = std::fabs(cand.orientation.x - dst.orientation.x);
        const double dy = std::fabs(cand.orientation.y - dst.orientation.y);
        const double dz = std::fabs(cand.orientation.z - dst.orientation.z);
        const double dw = std::fabs(cand.orientation.w - dst.orientation.w);
        const double dmax = std::max(std::max(dx, dy), std::max(dz, dw));

        if (dmax >= kQuatDeadband) {
            dst.orientation = cand.orientation;
            normalize_quat_msg(dst);
        }
    };

    static bool s_cur_l_init = false;
    static bool s_cur_r_init = false;
    static bool s_tar_l_init = false;
    static bool s_tar_r_init = false;

    std::vector<double> jl(6), jr(6);
    for (int i = 0; i < 6; ++i) {
        jl[i] = q_l_c_(i);
        jr[i] = q_r_c_(i);
    }

    geometry_msgs::msg::Pose cur_l_fk = arm_fk_->getLeftFK(jl);
    geometry_msgs::msg::Pose cur_r_fk = arm_fk_->getRightFK(jr);

    apply_pose_deadband(current_pose_l_, cur_l_fk, s_cur_l_init);
    apply_pose_deadband(current_pose_r_, cur_r_fk, s_cur_r_init);

    if (!delta_arm_base_pose_initialized_ && s_cur_l_init && s_cur_r_init) {
        delta_arm_base_pose_l_ = current_pose_l_;
        delta_arm_base_pose_r_ = current_pose_r_;
        delta_arm_base_pose_initialized_ = true;

        RCLCPP_INFO(
            node_->get_logger(),
            "[DeltaArmBase] latched initial base pose | "
            "L=(%.4f %.4f %.4f) R=(%.4f %.4f %.4f)",
            delta_arm_base_pose_l_.position.x,
            delta_arm_base_pose_l_.position.y,
            delta_arm_base_pose_l_.position.z,
            delta_arm_base_pose_r_.position.x,
            delta_arm_base_pose_r_.position.y,
            delta_arm_base_pose_r_.position.z
        );
    }

    if (current_arm_control_mode_ == "idle") {
        target_pose_l_ = current_pose_l_;
        target_pose_r_ = current_pose_r_;
        s_tar_l_init = true;
        s_tar_r_init = true;
    }
    else if (current_arm_control_mode_ == "forward") {
        std::vector<double> jl_t(6), jr_t(6);
        for (int i = 0; i < 6; ++i) {
            jl_t[i] = q_l_t_(i);
            jr_t[i] = q_r_t_(i);
        }

        geometry_msgs::msg::Pose tar_l_fk = arm_fk_->getLeftFK(jl_t);
        geometry_msgs::msg::Pose tar_r_fk = arm_fk_->getRightFK(jr_t);

        apply_pose_deadband(target_pose_l_, tar_l_fk, s_tar_l_init);
        apply_pose_deadband(target_pose_r_, tar_r_fk, s_tar_r_init);
    }
    // arm inverse 모드에서는 target_pose_* 를 TargetArmPositionCallback / DeltaArmPositionCallback 이 관리
}

// --------------------
// HandPositionCallback
// --------------------
void DualArmForceControl::HandPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    (void)msg;
    if (!is_initialized_) return;
    if (!hand_fk_l_ || !hand_fk_r_) return;

    constexpr double kFingerPosDeadbandM = 1e-6;

    auto safe_get = [&](const std::vector<Eigen::Vector3d>& v, int idx) -> Eigen::Vector3d {
        if (idx < 0 || idx >= static_cast<int>(v.size())) return Eigen::Vector3d::Zero();
        const Eigen::Vector3d& p = v[idx];
        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
            return Eigen::Vector3d::Zero();
        }
        return p;
    };

    auto set_point_from_vec = [&](geometry_msgs::msg::Point& p, const Eigen::Vector3d& v) {
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
    };

    auto apply_point_deadband = [&](geometry_msgs::msg::Point& dst,
                                    const Eigen::Vector3d& src,
                                    bool& is_init_cache)
    {
        if (!is_init_cache) {
            set_point_from_vec(dst, src);
            is_init_cache = true;
            return;
        }

        if (std::fabs(src.x() - dst.x) >= kFingerPosDeadbandM) dst.x = src.x();
        if (std::fabs(src.y() - dst.y) >= kFingerPosDeadbandM) dst.y = src.y();
        if (std::fabs(src.z() - dst.z) >= kFingerPosDeadbandM) dst.z = src.z();
    };

    auto compress20to15 = [&](const Eigen::VectorXd& qh) -> std::vector<double> {
        std::vector<double> h15(15, 0.0);

        if (qh.size() >= 20) {
            for (int f = 0; f < 5; ++f) {
                const int b20 = f * 4;
                const int b15 = f * 3;
                h15[b15 + 0] = qh(b20 + 0);
                h15[b15 + 1] = qh(b20 + 1);
                h15[b15 + 2] = qh(b20 + 2);
            }
            return h15;
        }

        const int n = std::min<int>(qh.size(), 15);
        for (int i = 0; i < n; ++i) h15[i] = qh(i);
        return h15;
    };

    static bool s_l_cur_init[5] = {false, false, false, false, false};
    static bool s_r_cur_init[5] = {false, false, false, false, false};
    static bool s_l_tar_init[5] = {false, false, false, false, false};
    static bool s_r_tar_init[5] = {false, false, false, false, false};

    const std::vector<double> hl15   = compress20to15(q_l_h_c_);
    const std::vector<double> hr15   = compress20to15(q_r_h_c_);
    const std::vector<double> hl_t15 = compress20to15(q_l_h_t_);
    const std::vector<double> hr_t15 = compress20to15(q_r_h_t_);

    const std::vector<Eigen::Vector3d> tl   = hand_fk_l_->computeFingertips(hl15);
    const std::vector<Eigen::Vector3d> tr   = hand_fk_r_->computeFingertips(hr15);
    const std::vector<Eigen::Vector3d> tl_t = hand_fk_l_->computeFingertips(hl_t15);
    const std::vector<Eigen::Vector3d> tr_t = hand_fk_r_->computeFingertips(hr_t15);

    apply_point_deadband(f_l_thumb_,  safe_get(tl, 0), s_l_cur_init[0]);
    apply_point_deadband(f_l_index_,  safe_get(tl, 1), s_l_cur_init[1]);
    apply_point_deadband(f_l_middle_, safe_get(tl, 2), s_l_cur_init[2]);
    apply_point_deadband(f_l_ring_,   safe_get(tl, 3), s_l_cur_init[3]);
    apply_point_deadband(f_l_baby_,   safe_get(tl, 4), s_l_cur_init[4]);

    apply_point_deadband(f_r_thumb_,  safe_get(tr, 0), s_r_cur_init[0]);
    apply_point_deadband(f_r_index_,  safe_get(tr, 1), s_r_cur_init[1]);
    apply_point_deadband(f_r_middle_, safe_get(tr, 2), s_r_cur_init[2]);
    apply_point_deadband(f_r_ring_,   safe_get(tr, 3), s_r_cur_init[3]);
    apply_point_deadband(f_r_baby_,   safe_get(tr, 4), s_r_cur_init[4]);

    if (current_hand_control_mode_ == "idle") {
        t_f_l_thumb_  = f_l_thumb_;
        t_f_l_index_  = f_l_index_;
        t_f_l_middle_ = f_l_middle_;
        t_f_l_ring_   = f_l_ring_;
        t_f_l_baby_   = f_l_baby_;

        t_f_r_thumb_  = f_r_thumb_;
        t_f_r_index_  = f_r_index_;
        t_f_r_middle_ = f_r_middle_;
        t_f_r_ring_   = f_r_ring_;
        t_f_r_baby_   = f_r_baby_;

        for (int i = 0; i < 5; ++i) {
            s_l_tar_init[i] = true;
            s_r_tar_init[i] = true;
        }
    } else {
        apply_point_deadband(t_f_l_thumb_,  safe_get(tl_t, 0), s_l_tar_init[0]);
        apply_point_deadband(t_f_l_index_,  safe_get(tl_t, 1), s_l_tar_init[1]);
        apply_point_deadband(t_f_l_middle_, safe_get(tl_t, 2), s_l_tar_init[2]);
        apply_point_deadband(t_f_l_ring_,   safe_get(tl_t, 3), s_l_tar_init[3]);
        apply_point_deadband(t_f_l_baby_,   safe_get(tl_t, 4), s_l_tar_init[4]);

        apply_point_deadband(t_f_r_thumb_,  safe_get(tr_t, 0), s_r_tar_init[0]);
        apply_point_deadband(t_f_r_index_,  safe_get(tr_t, 1), s_r_tar_init[1]);
        apply_point_deadband(t_f_r_middle_, safe_get(tr_t, 2), s_r_tar_init[2]);
        apply_point_deadband(t_f_r_ring_,   safe_get(tr_t, 3), s_r_tar_init[3]);
        apply_point_deadband(t_f_r_baby_,   safe_get(tr_t, 4), s_r_tar_init[4]);
    }
}

void DualArmForceControl::ControlModeCallback(
    const std::shared_ptr<dualarm_forcecon_interfaces::srv::SetControlMode::Request> req,
    std::shared_ptr<dualarm_forcecon_interfaces::srv::SetControlMode::Response> res)
{
    if (!req) {
        res->success = false;
        res->message = "Null request";
        return;
    }

    auto to_lower = [](std::string s) -> std::string {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
        return s;
    };

    const std::string prev_arm_mode  = current_arm_control_mode_;
    const std::string prev_hand_mode = current_hand_control_mode_;

    const std::string new_arm_mode  = to_lower(req->arm_mode);
    const std::string new_hand_mode = to_lower(req->hand_mode);

    auto valid_arm_mode = [](const std::string& m) -> bool {
        return (m == "idle" || m == "forward" || m == "inverse");
    };

    auto valid_hand_mode = [](const std::string& m) -> bool {
        return (m == "idle" || m == "forward" || m == "inverse" || m == "forcecon");
    };

    if (!valid_arm_mode(new_arm_mode)) {
        res->success = false;
        res->message = "Invalid arm_mode: " + req->arm_mode +
                       " (allowed: idle, forward, inverse)";
        return;
    }

    if (!valid_hand_mode(new_hand_mode)) {
        res->success = false;
        res->message = "Invalid hand_mode: " + req->hand_mode +
                       " (allowed: idle, forward, inverse, forcecon)";
        return;
    }

    current_arm_control_mode_  = new_arm_mode;
    current_hand_control_mode_ = new_hand_mode;

    const bool entering_hand_forcecon =
        (prev_hand_mode != "forcecon" && current_hand_control_mode_ == "forcecon");
    const bool leaving_hand_forcecon =
        (prev_hand_mode == "forcecon" && current_hand_control_mode_ != "forcecon");

    auto reset_all_hand_adm = [&]() {
        for (int f = 0; f < 5; ++f) {
            if (hand_adm_l_[f]) hand_adm_l_[f]->resetState();
            if (hand_adm_r_[f]) hand_adm_r_[f]->resetState();
        }
    };

    auto clear_forcecon_latch = [&]() {
        hand_force_cmd_valid_ = false;
        hand_force_cmd_hand_id_ = 0;
        hand_force_cmd_finger_id_ = 3;
        hand_force_cmd_p_des_base_.setZero();
        hand_force_cmd_f_des_base_.setZero();
        hand_force_cmd_stamp_ns_ = 0;
    };

    // ------------------------------------------------------------------------
    // ARM mode handling
    // ------------------------------------------------------------------------
    arm_idle_synced_ = false;

    if (current_arm_control_mode_ == "inverse") {
        for (int i = 0; i < 6; ++i) {
            q_l_t_(i) = q_l_c_(i);
            q_r_t_(i) = q_r_c_(i);
        }

        target_pose_l_ = current_pose_l_;
        target_pose_r_ = current_pose_r_;
    }

    f_l_t_.setZero();
    f_r_t_.setZero();

    // ------------------------------------------------------------------------
    // HAND mode handling
    // ------------------------------------------------------------------------
    hand_idle_synced_ = false;

    if (current_hand_control_mode_ == "inverse" ||
        current_hand_control_mode_ == "forcecon") {
        for (int i = 0; i < 20; ++i) {
            q_l_h_t_(i) = q_l_h_c_(i);
            q_r_h_t_(i) = q_r_h_c_(i);
        }

        t_f_l_thumb_  = f_l_thumb_;
        t_f_l_index_  = f_l_index_;
        t_f_l_middle_ = f_l_middle_;
        t_f_l_ring_   = f_l_ring_;
        t_f_l_baby_   = f_l_baby_;

        t_f_r_thumb_  = f_r_thumb_;
        t_f_r_index_  = f_r_index_;
        t_f_r_middle_ = f_r_middle_;
        t_f_r_ring_   = f_r_ring_;
        t_f_r_baby_   = f_r_baby_;
    }

    if (current_hand_control_mode_ != "forcecon") {
        f_l_hand_t_.setZero();
        f_r_hand_t_.setZero();
    }

    if (entering_hand_forcecon) {
        clear_forcecon_latch();
        f_l_hand_t_.setZero();
        f_r_hand_t_.setZero();
        reset_all_hand_adm();
    }

    if (leaving_hand_forcecon) {
        clear_forcecon_latch();
        reset_all_hand_adm();
        forcecon_hold_snapshot_valid_ = false;
        forcecon_prev_cycle_ = false;
    }

    res->success = true;
    res->message =
        "Arm mode: " + current_arm_control_mode_ +
        ", Hand mode: " + current_hand_control_mode_;
}

void DualArmForceControl::HandContactForceCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!msg) return;

    auto reset_current_force_buffers_only = [&]() {
        f_l_c_.setZero();
        f_r_c_.setZero();

        f_l_hand_c_.setZero();
        f_r_hand_c_.setZero();

        raw_l_hand_contact_.setZero();
        raw_r_hand_contact_.setZero();
        f_l_hand_sensor_c_.setZero();
        f_r_hand_sensor_c_.setZero();
        f_l_hand_wrist_c_.setZero();
        f_r_hand_wrist_c_.setZero();
    };
    reset_current_force_buffers_only();

    if (msg->data.size() < 10) {
        return;
    }

    if (!hand_fk_l_ || !hand_fk_r_) {
        return;
    }

    auto compress20to15 = [](const Eigen::VectorXd& qh) -> std::vector<double> {
        std::vector<double> h15(15, 0.0);

        if (qh.size() >= 20) {
            for (int f = 0; f < 5; ++f) {
                const int b20 = f * 4;
                const int b15 = f * 3;
                h15[b15 + 0] = qh(b20 + 0);
                h15[b15 + 1] = qh(b20 + 1);
                h15[b15 + 2] = qh(b20 + 2);
            }
            return h15;
        }

        const int n = std::min<int>(qh.size(), 15);
        for (int i = 0; i < n; ++i) h15[i] = qh(i);
        return h15;
    };

    const std::vector<double> hl15 = compress20to15(q_l_h_c_);
    const std::vector<double> hr15 = compress20to15(q_r_h_c_);

    const std::vector<Eigen::Matrix3d> Rl_base_tip = hand_fk_l_->computeTipRotationsBase(hl15);
    const std::vector<Eigen::Matrix3d> Rr_base_tip = hand_fk_r_->computeTipRotationsBase(hr15);

    auto safeR = [](const std::vector<Eigen::Matrix3d>& Rs, int idx) -> Eigen::Matrix3d {
        if (idx < 0 || idx >= static_cast<int>(Rs.size())) return Eigen::Matrix3d::Identity();
        return Rs[idx];
    };

    const Eigen::Matrix3d R_tip_sensor = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_wrist_output_calib;
    R_wrist_output_calib <<  0.0, 0.0, 1.0,
                             0.0, 1.0, 0.0,
                            -1.0, 0.0, 0.0;

    const std::array<int,5> msg_to_row = {4, 3, 2, 1, 0};
    const std::array<const char*,5> finger_name = {{"THMB","INDX","MIDL","RING","BABY"}};

    auto assign_one_hand = [&](Eigen::Matrix<double,5,1>& raw_contact_mat,
                               Eigen::Matrix<double,5,3>& F_sensor_mat,
                               Eigen::Matrix<double,5,3>& F_wrist_mat,
                               Eigen::Matrix<double,5,3>& F_hand_cur,
                               const std::vector<Eigen::Matrix3d>& R_base_tip_all,
                               int msg_offset,
                               const char* hand_tag)
    {
        static int dbg_decim = 0;
        const bool do_dbg = ((dbg_decim++ % 20) == 0);

        for (int k = 0; k < 5; ++k) {
            const int row = msg_to_row[k];
            const double s = static_cast<double>(msg->data[msg_offset + k]);

            raw_contact_mat(row, 0) = s;

            const Eigen::Vector3d f_sensor(s, 0.0, 0.0);
            F_sensor_mat.row(row) = f_sensor.transpose();

            const Eigen::Vector3d f_tip = R_tip_sensor * f_sensor;
            const Eigen::Matrix3d R_base_tip = safeR(R_base_tip_all, row);
            const Eigen::Vector3d f_wrist_raw = R_base_tip * f_tip;
            Eigen::Vector3d f_wrist = R_wrist_output_calib * f_wrist_raw;

            for (int i = 0; i < 3; ++i) {
                if (std::fabs(f_wrist(i)) < 1e-9) f_wrist(i) = 0.0;
            }

            F_wrist_mat.row(row) = f_wrist.transpose();
            F_hand_cur.row(row)  = f_wrist.transpose();

            (void)do_dbg;
            (void)finger_name;
            (void)hand_tag;
        }
    };

    assign_one_hand(raw_l_hand_contact_,
                    f_l_hand_sensor_c_,
                    f_l_hand_wrist_c_,
                    f_l_hand_c_,
                    Rl_base_tip,
                    0,
                    "L");

    assign_one_hand(raw_r_hand_contact_,
                    f_r_hand_sensor_c_,
                    f_r_hand_wrist_c_,
                    f_r_hand_c_,
                    Rr_base_tip,
                    5,
                    "R");
}

void DualArmForceControl::PublishHandForceMonitor()
{
    if (!node_) return;

    std_msgs::msg::Float32MultiArray cur_msg;
    std_msgs::msg::Float32MultiArray tar_msg;

    cur_msg.data.resize(30, 0.0f);
    tar_msg.data.resize(30, 0.0f);

    auto pack5x3 = [](const Eigen::Matrix<double,5,3>& M,
                      std::vector<float>& out,
                      int offset)
    {
        for (int f = 0; f < 5; ++f) {
            for (int a = 0; a < 3; ++a) {
                out[offset + f * 3 + a] = static_cast<float>(M(f, a));
            }
        }
    };

    pack5x3(f_l_hand_c_, cur_msg.data, 0);
    pack5x3(f_l_hand_t_, tar_msg.data, 0);

    pack5x3(f_r_hand_c_, cur_msg.data, 15);
    pack5x3(f_r_hand_t_, tar_msg.data, 15);

    hand_force_current_monitor_pub_->publish(cur_msg);
    hand_force_target_monitor_pub_->publish(tar_msg);
}

// ============================================================================
// PrintDualArmStates
// ============================================================================
void DualArmForceControl::PrintDualArmStates() {
    if (!is_initialized_) return;

    constexpr const char* C_RESET   = "\033[0m";
    constexpr const char* C_TITLE   = "\033[1;38;5;252m";
    constexpr const char* C_DIM     = "\033[38;5;245m";

    constexpr const char* C_CUR_POS = "\033[1;38;5;81m";
    constexpr const char* C_TAR_POS = "\033[1;38;5;220m";

    constexpr const char* C_CUR_F   = "\033[1;31m";
    constexpr const char* C_TAR_F   = "\033[1;34m";
    constexpr const char* C_MODE    = "\033[1;92m";

    auto fmtArmLine = [&](const char* tag,
                          const geometry_msgs::msg::Pose& pose,
                          const Eigen::Vector3d& f,
                          const char* c_pos,
                          const char* c_f)
    {
        double r_deg, p_deg, y_deg;
        ArmForwardKinematics::quatToEulerXYZDeg_Isaac(pose.orientation, r_deg, p_deg, y_deg);

        printf("  %-4s %sP[m,deg]=(%7.4f %7.4f %7.4f | %7.2f %7.2f %7.2f)%s   "
               "%sF[N]=(%7.3f %7.3f %7.3f)%s\n",
               tag,
               c_pos,
               pose.position.x, pose.position.y, pose.position.z,
               r_deg, p_deg, y_deg,
               C_RESET,
               c_f,
               f(0), f(1), f(2),
               C_RESET);
    };

    auto fmtFingerLine = [&](const char* tag,
                             const geometry_msgs::msg::Point& p,
                             const Eigen::RowVector3d& f,
                             const char* c_pos,
                             const char* c_f)
    {
        printf("  %-4s %sP[m]=(%7.4f %7.4f %7.4f)%s   "
               "%sF[N]=(%7.3f %7.3f %7.3f)%s\n",
               tag,
               c_pos,
               p.x, p.y, p.z,
               C_RESET,
               c_f,
               f(0), f(1), f(2),
               C_RESET);
    };

    auto printFingerBlock = [&](const char* name4,
                                const geometry_msgs::msg::Point& cur_p,
                                const geometry_msgs::msg::Point& tar_p,
                                const Eigen::RowVector3d& cur_f,
                                const Eigen::RowVector3d& tar_f)
    {
        fmtFingerLine(name4, cur_p, cur_f, C_CUR_POS, C_CUR_F);
        fmtFingerLine(name4, tar_p, tar_f, C_TAR_POS, C_TAR_F);
        printf("\n");
    };

    printf("\033[2J\033[H");

    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);
    printf("%s   Dual Arm & Hand Monitor v22 | Arm:[%s%s%s] Hand:[%s%s%s] | %sCUR_POS%s %sTAR_POS%s %sCUR_F%s %sTAR_F%s%s\n",
           C_TITLE,
           C_MODE, current_arm_control_mode_.c_str(), C_RESET,
           C_MODE, current_hand_control_mode_.c_str(), C_RESET,
           C_CUR_POS, C_RESET,
           C_TAR_POS, C_RESET,
           C_CUR_F,   C_RESET,
           C_TAR_F,   C_RESET,
           C_RESET);
    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);

    printf("%s[L ARM]%s\n", C_TITLE, C_RESET);
    fmtArmLine("CUR", current_pose_l_, f_l_c_, C_CUR_POS, C_CUR_F);
    fmtArmLine("TAR", target_pose_l_,  f_l_t_, C_TAR_POS, C_TAR_F);

    printf("%s------------------------------------------------------------------------------------------------------------%s\n", C_DIM, C_RESET);

    printf("%s[R ARM]%s\n", C_TITLE, C_RESET);
    fmtArmLine("CUR", current_pose_r_, f_r_c_, C_CUR_POS, C_CUR_F);
    fmtArmLine("TAR", target_pose_r_,  f_r_t_, C_TAR_POS, C_TAR_F);

    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);

    printf("%s[L HAND] (positions are expressed in LEFT_HAND_BASE frame)%s\n\n", C_TITLE, C_RESET);

    printFingerBlock("BABY",
                     f_l_baby_, t_f_l_baby_,
                     f_l_hand_c_.row(4), f_l_hand_t_.row(4));

    printFingerBlock("RING",
                     f_l_ring_, t_f_l_ring_,
                     f_l_hand_c_.row(3), f_l_hand_t_.row(3));

    printFingerBlock("MIDL",
                     f_l_middle_, t_f_l_middle_,
                     f_l_hand_c_.row(2), f_l_hand_t_.row(2));

    printFingerBlock("INDX",
                     f_l_index_, t_f_l_index_,
                     f_l_hand_c_.row(1), f_l_hand_t_.row(1));

    printFingerBlock("THMB",
                     f_l_thumb_, t_f_l_thumb_,
                     f_l_hand_c_.row(0), f_l_hand_t_.row(0));

    printf("%s------------------------------------------------------------------------------------------------------------%s\n", C_DIM, C_RESET);

    printf("%s[R HAND] (positions are expressed in RIGHT_HAND_BASE frame)%s\n\n", C_TITLE, C_RESET);

    printFingerBlock("BABY",
                     f_r_baby_, t_f_r_baby_,
                     f_r_hand_c_.row(4), f_r_hand_t_.row(4));

    printFingerBlock("RING",
                     f_r_ring_, t_f_r_ring_,
                     f_r_hand_c_.row(3), f_r_hand_t_.row(3));

    printFingerBlock("MIDL",
                     f_r_middle_, t_f_r_middle_,
                     f_r_hand_c_.row(2), f_r_hand_t_.row(2));

    printFingerBlock("INDX",
                     f_r_index_, t_f_r_index_,
                     f_r_hand_c_.row(1), f_r_hand_t_.row(1));

    printFingerBlock("THMB",
                     f_r_thumb_, t_f_r_thumb_,
                     f_r_hand_c_.row(0), f_r_hand_t_.row(0));

    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);
}