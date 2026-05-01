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

inline bool isFiniteVec3(const Eigen::Vector3d& v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline Eigen::Vector3d pointToVec(const geometry_msgs::msg::Point& p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}

inline void setPointFromVec(geometry_msgs::msg::Point& p, const Eigen::Vector3d& v)
{
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
}

inline Eigen::Vector3d safeGetTip(const std::vector<Eigen::Vector3d>& v, int idx)
{
    if (idx < 0 || idx >= static_cast<int>(v.size())) return Eigen::Vector3d::Zero();
    const Eigen::Vector3d& p = v[static_cast<std::size_t>(idx)];
    if (!isFiniteVec3(p)) return Eigen::Vector3d::Zero();
    return p;
}

inline Eigen::Matrix3d safeGetRot(const std::vector<Eigen::Matrix3d>& v, int idx)
{
    if (idx < 0 || idx >= static_cast<int>(v.size())) return Eigen::Matrix3d::Identity();
    return v[static_cast<std::size_t>(idx)];
}

inline void writeMatrixRow(Eigen::Matrix<double,5,3>& M, int row, const Eigen::Vector3d& v)
{
    if (row < 0 || row >= 5) return;
    M(row, 0) = v.x();
    M(row, 1) = v.y();
    M(row, 2) = v.z();
}

inline Eigen::Vector3d readMatrixRow(const Eigen::Matrix<double,5,3>& M, int row)
{
    if (row < 0 || row >= 5) return Eigen::Vector3d::Zero();
    return Eigen::Vector3d(M(row, 0), M(row, 1), M(row, 2));
}

inline void matrixRowToPoint(const Eigen::Matrix<double,5,3>& M, int row, geometry_msgs::msg::Point& p)
{
    if (row < 0 || row >= 5) {
        p.x = 0.0; p.y = 0.0; p.z = 0.0;
        return;
    }
    p.x = M(row, 0);
    p.y = M(row, 1);
    p.z = M(row, 2);
}

inline void updatePointSetFromMatrix(const Eigen::Matrix<double,5,3>& M,
                                     geometry_msgs::msg::Point& thumb,
                                     geometry_msgs::msg::Point& index,
                                     geometry_msgs::msg::Point& middle,
                                     geometry_msgs::msg::Point& ring,
                                     geometry_msgs::msg::Point& baby)
{
    matrixRowToPoint(M, 0, thumb);
    matrixRowToPoint(M, 1, index);
    matrixRowToPoint(M, 2, middle);
    matrixRowToPoint(M, 3, ring);
    matrixRowToPoint(M, 4, baby);
}

inline std::vector<double> compress20to15(const Eigen::VectorXd& qh)
{
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
}

inline std::vector<double> eigenToStdVec(const Eigen::VectorXd& q)
{
    std::vector<double> out(static_cast<std::size_t>(q.size()), 0.0);
    for (int i = 0; i < q.size(); ++i) out[static_cast<std::size_t>(i)] = q(i);
    return out;
}

} // namespace

// --------------------
// JointsCallback
// --------------------
void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!msg) return;
    if (joint_names_.empty()) {
        joint_names_ = msg->name;
        is_initialized_ = true;
    }

    auto starts_with = [](const std::string& s, const std::string& prefix) -> bool {
        return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
    };

    auto is_rby1_fixed_joint_name = [&](const std::string& n) -> bool {
        if (n == "left_wheel" || n == "right_wheel") return true;
        if (starts_with(n, "torso_")) return true;
        if (starts_with(n, "head_")) return true;
        auto hj = dualarm_forcecon::kin::parseHandJointName(n);
        return hj.ok;
    };

    const size_t n_pos = std::min(msg->name.size(), msg->position.size());
    for (size_t i = 0; i < n_pos; ++i) {
        const std::string& n = msg->name[i];
        const double p = msg->position[i];
        last_joint_position_[n] = p;

        if (startup_fixed_joint_hold_enabled_ && !startup_fixed_joint_hold_latched_) {
            if (is_rby1_fixed_joint_name(n)) {
                startup_fixed_joint_position_[n] = p;
            }
        }

        const int li = kin_cfg_.findLeftArmJointIndex(n);
        const int ri = kin_cfg_.findRightArmJointIndex(n);

        if (li >= 0 && li < q_l_c_.size()) {
            q_l_c_(li) = p;
            continue;
        }
        if (ri >= 0 && ri < q_r_c_.size()) {
            q_r_c_(ri) = p;
            continue;
        }

        const bool should_track_hand_state = kin_cfg_.hand_enabled || startup_fixed_hand_hold_enabled_;
        if (!should_track_hand_state) continue;

        auto hj = dualarm_forcecon::kin::parseHandJointName(n);
        if (!hj.ok) continue;

        const int idx = hj.finger_id * 4 + hj.joint_id;  // 0..19
        if (idx < 0 || idx >= 20) continue;

        if (hj.is_left) q_l_h_c_(idx) = p;
        else            q_r_h_c_(idx) = p;
    }

    if (startup_fixed_hand_hold_enabled_ && !startup_fixed_hand_hold_latched_) {
        q_l_h_fixed_ = q_l_h_c_;
        q_r_h_fixed_ = q_r_h_c_;
        startup_fixed_hand_hold_latched_ = true;
    }

    if (startup_fixed_joint_hold_enabled_ && !startup_fixed_joint_hold_latched_) {
        startup_fixed_joint_hold_latched_ = true;
        RCLCPP_INFO(
            node_->get_logger(),
            "[StartupHold] latched %zu startup fixed joints (wheel/torso/head/fingers)",
            startup_fixed_joint_position_.size());
    }
}

// --------------------
// PositionCallback (WRAPPER)
// --------------------
void DualArmForceControl::PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    (void)msg;
    if (!is_initialized_) return;

    ArmPositionCallback(msg);
    HandPositionCallback(msg);
}

// --------------------
// ArmPositionCallback
// --------------------
void DualArmForceControl::ArmPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
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

    const std::vector<double> jl = eigenToStdVec(q_l_c_);
    const std::vector<double> jr = eigenToStdVec(q_r_c_);

    geometry_msgs::msg::Pose cur_l_fk = arm_fk_->getLeftFK(jl);
    geometry_msgs::msg::Pose cur_r_fk = arm_fk_->getRightFK(jr);

    apply_pose_deadband(current_pose_l_, cur_l_fk, s_cur_l_init);
    apply_pose_deadband(current_pose_r_, cur_r_fk, s_cur_r_init);

    if (!delta_arm_base_pose_initialized_ && s_cur_l_init && s_cur_r_init) {
        delta_arm_base_pose_l_ = current_pose_l_;
        delta_arm_base_pose_r_ = current_pose_r_;
        delta_arm_base_pose_initialized_ = true;

        // RCLCPP_INFO(
        //     node_->get_logger(),
        //     "[DeltaArmBase] latched initial base pose | "
        //     "L=(%.4f %.4f %.4f) R=(%.4f %.4f %.4f)",
        //     delta_arm_base_pose_l_.position.x,
        //     delta_arm_base_pose_l_.position.y,
        //     delta_arm_base_pose_l_.position.z,
        //     delta_arm_base_pose_r_.position.x,
        //     delta_arm_base_pose_r_.position.y,
        //     delta_arm_base_pose_r_.position.z
        // );
    }

    if (current_arm_control_mode_ == "idle") {
        target_pose_l_ = current_pose_l_;
        target_pose_r_ = current_pose_r_;
        s_tar_l_init = true;
        s_tar_r_init = true;
    }
    else if (current_arm_control_mode_ == "forward") {
        const std::vector<double> jl_t = eigenToStdVec(q_l_t_);
        const std::vector<double> jr_t = eigenToStdVec(q_r_t_);

        geometry_msgs::msg::Pose tar_l_fk = arm_fk_->getLeftFK(jl_t);
        geometry_msgs::msg::Pose tar_r_fk = arm_fk_->getRightFK(jr_t);

        apply_pose_deadband(target_pose_l_, tar_l_fk, s_tar_l_init);
        apply_pose_deadband(target_pose_r_, tar_r_fk, s_tar_r_init);
    }
    else if (current_arm_control_mode_ == "inverse") {
        const bool is_rby1 = (kin_cfg_.profile.find("rby1") != std::string::npos);
        if (is_rby1 && !rby1_arm_target_active_) {
            target_pose_l_ = current_pose_l_;
            target_pose_r_ = current_pose_r_;
        }
        // Doosan inverse keeps target_pose_* managed by callbacks as before.
    }
}

// --------------------
// HandPositionCallback
//   - current fingertip position x_0 source
//   - current fingertip rotation cache source
//   - updates target-point monitor depending on mode
// --------------------
void DualArmForceControl::HandPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    (void)msg;
    if (!is_initialized_) return;
    if (!kin_cfg_.hand_enabled) return;
    if (!hand_fk_l_ || !hand_fk_r_) return;

    constexpr double kFingerPosDeadbandM = 1e-6;

    auto apply_point_deadband = [&](geometry_msgs::msg::Point& dst,
                                    const Eigen::Vector3d& src,
                                    bool& is_init_cache)
    {
        if (!is_init_cache) {
            setPointFromVec(dst, src);
            is_init_cache = true;
            return;
        }

        if (std::fabs(src.x() - dst.x) >= kFingerPosDeadbandM) dst.x = src.x();
        if (std::fabs(src.y() - dst.y) >= kFingerPosDeadbandM) dst.y = src.y();
        if (std::fabs(src.z() - dst.z) >= kFingerPosDeadbandM) dst.z = src.z();
    };

    static bool s_l_cur_init[5] = {false, false, false, false, false};
    static bool s_r_cur_init[5] = {false, false, false, false, false};
    static bool s_l_tar_init[5] = {false, false, false, false, false};
    static bool s_r_tar_init[5] = {false, false, false, false, false};
    static bool s_l_cmd_init[5] = {false, false, false, false, false};
    static bool s_r_cmd_init[5] = {false, false, false, false, false};

    const std::vector<double> hl15 = compress20to15(q_l_h_c_);
    const std::vector<double> hr15 = compress20to15(q_r_h_c_);

    const std::vector<Eigen::Vector3d> tl = hand_fk_l_->computeFingertips(hl15);
    const std::vector<Eigen::Vector3d> tr = hand_fk_r_->computeFingertips(hr15);

    const std::vector<Eigen::Matrix3d> Rl = hand_fk_l_->computeTipRotationsBase(hl15);
    const std::vector<Eigen::Matrix3d> Rr = hand_fk_r_->computeTipRotationsBase(hr15);

    // current fingertip position cache
    for (int i = 0; i < 5; ++i) {
        writeMatrixRow(x_l_hand_c_, i, safeGetTip(tl, i));
        writeMatrixRow(x_r_hand_c_, i, safeGetTip(tr, i));

        R_l_base_tip_c_[static_cast<std::size_t>(i)] = safeGetRot(Rl, i);
        R_r_base_tip_c_[static_cast<std::size_t>(i)] = safeGetRot(Rr, i);
    }

    // current point monitor
    apply_point_deadband(f_l_thumb_,  readMatrixRow(x_l_hand_c_, 0), s_l_cur_init[0]);
    apply_point_deadband(f_l_index_,  readMatrixRow(x_l_hand_c_, 1), s_l_cur_init[1]);
    apply_point_deadband(f_l_middle_, readMatrixRow(x_l_hand_c_, 2), s_l_cur_init[2]);
    apply_point_deadband(f_l_ring_,   readMatrixRow(x_l_hand_c_, 3), s_l_cur_init[3]);
    apply_point_deadband(f_l_baby_,   readMatrixRow(x_l_hand_c_, 4), s_l_cur_init[4]);

    apply_point_deadband(f_r_thumb_,  readMatrixRow(x_r_hand_c_, 0), s_r_cur_init[0]);
    apply_point_deadband(f_r_index_,  readMatrixRow(x_r_hand_c_, 1), s_r_cur_init[1]);
    apply_point_deadband(f_r_middle_, readMatrixRow(x_r_hand_c_, 2), s_r_cur_init[2]);
    apply_point_deadband(f_r_ring_,   readMatrixRow(x_r_hand_c_, 3), s_r_cur_init[3]);
    apply_point_deadband(f_r_baby_,   readMatrixRow(x_r_hand_c_, 4), s_r_cur_init[4]);

    // If command Cartesian cache is still zero-ish, initialize with current
    for (int i = 0; i < 5; ++i) {
        const Eigen::Vector3d xl_cmd = readMatrixRow(x_l_hand_cmd_, i);
        const Eigen::Vector3d xr_cmd = readMatrixRow(x_r_hand_cmd_, i);

        if (xl_cmd.norm() < 1e-12) writeMatrixRow(x_l_hand_cmd_, i, readMatrixRow(x_l_hand_c_, i));
        if (xr_cmd.norm() < 1e-12) writeMatrixRow(x_r_hand_cmd_, i, readMatrixRow(x_r_hand_c_, i));
    }

    // command point monitor (optional future extension)
    apply_point_deadband(c_f_l_thumb_,  readMatrixRow(x_l_hand_cmd_, 0), s_l_cmd_init[0]);
    apply_point_deadband(c_f_l_index_,  readMatrixRow(x_l_hand_cmd_, 1), s_l_cmd_init[1]);
    apply_point_deadband(c_f_l_middle_, readMatrixRow(x_l_hand_cmd_, 2), s_l_cmd_init[2]);
    apply_point_deadband(c_f_l_ring_,   readMatrixRow(x_l_hand_cmd_, 3), s_l_cmd_init[3]);
    apply_point_deadband(c_f_l_baby_,   readMatrixRow(x_l_hand_cmd_, 4), s_l_cmd_init[4]);

    apply_point_deadband(c_f_r_thumb_,  readMatrixRow(x_r_hand_cmd_, 0), s_r_cmd_init[0]);
    apply_point_deadband(c_f_r_index_,  readMatrixRow(x_r_hand_cmd_, 1), s_r_cmd_init[1]);
    apply_point_deadband(c_f_r_middle_, readMatrixRow(x_r_hand_cmd_, 2), s_r_cmd_init[2]);
    apply_point_deadband(c_f_r_ring_,   readMatrixRow(x_r_hand_cmd_, 3), s_r_cmd_init[3]);
    apply_point_deadband(c_f_r_baby_,   readMatrixRow(x_r_hand_cmd_, 4), s_r_cmd_init[4]);

    if (current_hand_control_mode_ == "idle") {
        // idle: desired == current
        x_l_hand_d_ = x_l_hand_c_;
        x_r_hand_d_ = x_r_hand_c_;
        x_l_hand_ref_ = x_l_hand_c_;
        x_r_hand_ref_ = x_r_hand_c_;
        x_l_hand_cmd_ = x_l_hand_c_;
        x_r_hand_cmd_ = x_r_hand_c_;

        updatePointSetFromMatrix(
            x_l_hand_d_,
            t_f_l_thumb_, t_f_l_index_, t_f_l_middle_, t_f_l_ring_, t_f_l_baby_);

        updatePointSetFromMatrix(
            x_r_hand_d_,
            t_f_r_thumb_, t_f_r_index_, t_f_r_middle_, t_f_r_ring_, t_f_r_baby_);

        hand_cartesian_target_l_initialized_ = true;
        hand_cartesian_target_r_initialized_ = true;

        for (int i = 0; i < 5; ++i) {
            s_l_tar_init[i] = true;
            s_r_tar_init[i] = true;
        }
    }
    else if (current_hand_control_mode_ == "forward") {
        // forward: x_d comes from motion joint target via FK
        const std::vector<double> hl_ref15 = compress20to15(q_l_h_motion_t_);
        const std::vector<double> hr_ref15 = compress20to15(q_r_h_motion_t_);

        const std::vector<Eigen::Vector3d> tl_ref = hand_fk_l_->computeFingertips(hl_ref15);
        const std::vector<Eigen::Vector3d> tr_ref = hand_fk_r_->computeFingertips(hr_ref15);

        for (int i = 0; i < 5; ++i) {
            writeMatrixRow(x_l_hand_d_, i, safeGetTip(tl_ref, i));
            writeMatrixRow(x_r_hand_d_, i, safeGetTip(tr_ref, i));
        }

        hand_cartesian_target_l_initialized_ = true;
        hand_cartesian_target_r_initialized_ = true;

        apply_point_deadband(t_f_l_thumb_,  readMatrixRow(x_l_hand_d_, 0), s_l_tar_init[0]);
        apply_point_deadband(t_f_l_index_,  readMatrixRow(x_l_hand_d_, 1), s_l_tar_init[1]);
        apply_point_deadband(t_f_l_middle_, readMatrixRow(x_l_hand_d_, 2), s_l_tar_init[2]);
        apply_point_deadband(t_f_l_ring_,   readMatrixRow(x_l_hand_d_, 3), s_l_tar_init[3]);
        apply_point_deadband(t_f_l_baby_,   readMatrixRow(x_l_hand_d_, 4), s_l_tar_init[4]);

        apply_point_deadband(t_f_r_thumb_,  readMatrixRow(x_r_hand_d_, 0), s_r_tar_init[0]);
        apply_point_deadband(t_f_r_index_,  readMatrixRow(x_r_hand_d_, 1), s_r_tar_init[1]);
        apply_point_deadband(t_f_r_middle_, readMatrixRow(x_r_hand_d_, 2), s_r_tar_init[2]);
        apply_point_deadband(t_f_r_ring_,   readMatrixRow(x_r_hand_d_, 3), s_r_tar_init[3]);
        apply_point_deadband(t_f_r_baby_,   readMatrixRow(x_r_hand_d_, 4), s_r_tar_init[4]);
    }
    else if (current_hand_control_mode_ == "inverse") {
        // inverse: x_d is authoritative Cartesian target set by callback
        // if not initialized yet, keep current to avoid jumps
        if (!hand_cartesian_target_l_initialized_) {
            x_l_hand_d_ = x_l_hand_c_;
            hand_cartesian_target_l_initialized_ = true;
        }
        if (!hand_cartesian_target_r_initialized_) {
            x_r_hand_d_ = x_r_hand_c_;
            hand_cartesian_target_r_initialized_ = true;
        }

        apply_point_deadband(t_f_l_thumb_,  readMatrixRow(x_l_hand_d_, 0), s_l_tar_init[0]);
        apply_point_deadband(t_f_l_index_,  readMatrixRow(x_l_hand_d_, 1), s_l_tar_init[1]);
        apply_point_deadband(t_f_l_middle_, readMatrixRow(x_l_hand_d_, 2), s_l_tar_init[2]);
        apply_point_deadband(t_f_l_ring_,   readMatrixRow(x_l_hand_d_, 3), s_l_tar_init[3]);
        apply_point_deadband(t_f_l_baby_,   readMatrixRow(x_l_hand_d_, 4), s_l_tar_init[4]);

        apply_point_deadband(t_f_r_thumb_,  readMatrixRow(x_r_hand_d_, 0), s_r_tar_init[0]);
        apply_point_deadband(t_f_r_index_,  readMatrixRow(x_r_hand_d_, 1), s_r_tar_init[1]);
        apply_point_deadband(t_f_r_middle_, readMatrixRow(x_r_hand_d_, 2), s_r_tar_init[2]);
        apply_point_deadband(t_f_r_ring_,   readMatrixRow(x_r_hand_d_, 3), s_r_tar_init[3]);
        apply_point_deadband(t_f_r_baby_,   readMatrixRow(x_r_hand_d_, 4), s_r_tar_init[4]);
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
    std::string new_hand_mode = to_lower(req->hand_mode);
    if (!kin_cfg_.hand_enabled && new_hand_mode != "idle") {
        RCLCPP_WARN(node_->get_logger(),
                    "[ControlMode] hand.enabled=false in YAML. Forcing requested hand_mode='%s' to 'idle'.",
                    req->hand_mode.c_str());
        new_hand_mode = "idle";
    }

    auto valid_arm_mode = [](const std::string& m) -> bool {
        return (m == "idle" || m == "forward" || m == "inverse");
    };

    auto valid_hand_mode = [](const std::string& m) -> bool {
        return (m == "idle" || m == "forward" || m == "inverse");
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
                       " (allowed: idle, forward, inverse)";
        return;
    }

    current_arm_control_mode_  = new_arm_mode;
    current_hand_control_mode_ = new_hand_mode;

    auto reset_all_hand_adm = [&]() {
        for (int f = 0; f < 5; ++f) {
            if (hand_adm_l_[f]) hand_adm_l_[f]->resetState();
            if (hand_adm_r_[f]) hand_adm_r_[f]->resetState();
        }
    };

    auto clear_hand_force_cmd = [&]() {
        hand_force_cmd_valid_ = false;
        hand_force_cmd_hand_id_ = 0;
        hand_force_cmd_finger_id_ = 3;
        hand_force_cmd_f_des_base_.setZero();
        hand_force_cmd_stamp_ns_ = 0;
        f_l_hand_t_.setZero();
        f_r_hand_t_.setZero();
    };

    auto sync_hand_targets_to_current = [&]() {
        for (int i = 0; i < 20; ++i) {
            q_l_h_motion_t_(i) = q_l_h_c_(i);
            q_r_h_motion_t_(i) = q_r_h_c_(i);
            q_l_h_t_(i) = q_l_h_c_(i);
            q_r_h_t_(i) = q_r_h_c_(i);
        }

        x_l_hand_d_   = x_l_hand_c_;
        x_r_hand_d_   = x_r_hand_c_;
        x_l_hand_ref_ = x_l_hand_c_;
        x_r_hand_ref_ = x_r_hand_c_;
        x_l_hand_cmd_ = x_l_hand_c_;
        x_r_hand_cmd_ = x_r_hand_c_;

        k_l_hand_eff_.setZero();
        k_r_hand_eff_.setZero();

        updatePointSetFromMatrix(
            x_l_hand_d_,
            t_f_l_thumb_, t_f_l_index_, t_f_l_middle_, t_f_l_ring_, t_f_l_baby_);

        updatePointSetFromMatrix(
            x_r_hand_d_,
            t_f_r_thumb_, t_f_r_index_, t_f_r_middle_, t_f_r_ring_, t_f_r_baby_);

        updatePointSetFromMatrix(
            x_l_hand_cmd_,
            c_f_l_thumb_, c_f_l_index_, c_f_l_middle_, c_f_l_ring_, c_f_l_baby_);

        updatePointSetFromMatrix(
            x_r_hand_cmd_,
            c_f_r_thumb_, c_f_r_index_, c_f_r_middle_, c_f_r_ring_, c_f_r_baby_);

        hand_cartesian_target_l_initialized_ = true;
        hand_cartesian_target_r_initialized_ = true;
    };

    // ------------------------------------------------------------------------
    // ARM mode handling
    // ------------------------------------------------------------------------
    arm_idle_synced_ = false;

    if (current_arm_control_mode_ == "inverse") {
        q_l_t_ = q_l_c_;
        q_r_t_ = q_r_c_;

        target_pose_l_ = current_pose_l_;
        target_pose_r_ = current_pose_r_;
        rby1_arm_target_active_ = false;  // armed only when an explicit arm target arrives
    } else {
        rby1_arm_target_active_ = false;
    }

    f_l_t_.setZero();
    f_r_t_.setZero();

    // ------------------------------------------------------------------------
    // HAND mode handling
    // ------------------------------------------------------------------------
    hand_idle_synced_ = false;

    if (prev_hand_mode != current_hand_control_mode_) {
        clear_hand_force_cmd();
        reset_all_hand_adm();
        sync_hand_targets_to_current();
    } else if (current_hand_control_mode_ == "idle") {
        // keep idle tightly synced to current
        sync_hand_targets_to_current();
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

    const std::vector<double> hl15 = compress20to15(q_l_h_c_);
    const std::vector<double> hr15 = compress20to15(q_r_h_c_);

    const std::vector<Eigen::Matrix3d> Rl_base_tip = hand_fk_l_->computeTipRotationsBase(hl15);
    const std::vector<Eigen::Matrix3d> Rr_base_tip = hand_fk_r_->computeTipRotationsBase(hr15);

    for (int i = 0; i < 5; ++i) {
        R_l_base_tip_c_[static_cast<std::size_t>(i)] = safeGetRot(Rl_base_tip, i);
        R_r_base_tip_c_[static_cast<std::size_t>(i)] = safeGetRot(Rr_base_tip, i);
    }

    const Eigen::Matrix3d R_tip_sensor = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d R_wrist_output_calib;
    R_wrist_output_calib <<  0.0, 0.0, 1.0,
                             0.0, 1.0, 0.0,
                            -1.0, 0.0, 0.0;

    // incoming observed order was [BABY, RING, MIDL, INDX, THMB]
    // internal canonical row order is [THMB, INDX, MIDL, RING, BABY]
    const std::array<int,5> msg_to_row = {4, 3, 2, 1, 0};

    auto assign_one_hand = [&](Eigen::Matrix<double,5,1>& raw_contact_mat,
                               Eigen::Matrix<double,5,3>& F_sensor_mat,
                               Eigen::Matrix<double,5,3>& F_wrist_mat,
                               Eigen::Matrix<double,5,3>& F_hand_cur,
                               const std::vector<Eigen::Matrix3d>& R_base_tip_all,
                               int msg_offset)
    {
        for (int k = 0; k < 5; ++k) {
            const int row = msg_to_row[k];
            const double s = static_cast<double>(msg->data[msg_offset + k]);

            raw_contact_mat(row, 0) = s;

            const Eigen::Vector3d f_sensor(s, 0.0, 0.0);
            F_sensor_mat.row(row) = f_sensor.transpose();

            const Eigen::Vector3d f_tip = R_tip_sensor * f_sensor;
            const Eigen::Matrix3d R_base_tip = safeGetRot(R_base_tip_all, row);
            const Eigen::Vector3d f_wrist_raw = R_base_tip * f_tip;
            Eigen::Vector3d f_wrist = R_wrist_output_calib * f_wrist_raw;

            for (int i = 0; i < 3; ++i) {
                if (std::fabs(f_wrist(i)) < 1e-9) f_wrist(i) = 0.0;
            }

            F_wrist_mat.row(row) = f_wrist.transpose();
            F_hand_cur.row(row)  = f_wrist.transpose();
        }
    };

    assign_one_hand(raw_l_hand_contact_,
                    f_l_hand_sensor_c_,
                    f_l_hand_wrist_c_,
                    f_l_hand_c_,
                    Rl_base_tip,
                    0);

    assign_one_hand(raw_r_hand_contact_,
                    f_r_hand_sensor_c_,
                    f_r_hand_wrist_c_,
                    f_r_hand_c_,
                    Rr_base_tip,
                    5);
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
void DualArmForceControl::PrintDualArmStates()
{
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
    printf("%s   Dual Arm & Hand Monitor v25 | Arm:[%s%s%s] Hand:[%s%s%s] | %sCUR_POS%s %sTAR_POS%s %sCUR_F%s %sTAR_F%s%s\n",
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