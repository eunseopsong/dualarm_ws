#include "DualArmForceControl.h"

#include <cstdio>
#include <cmath>
#include <limits>
#include <map>
#include <array>
#include <algorithm>
#include <cctype>

// ============================================================================
// v13 patched: TargetArmPositionCallback (inverse arm IK)
// msg: 12 = [L x y z r p y, R x y z r p y]
// - /target_arm_cartesian_pose 입력 xyz는 "base frame" 기준이라고 가정 (현재 패치)
// - ik_targets_frame_ == "base" 인 경우에도 추가 z offset 보정하지 않음
// - raw / ik-input / IK 성공여부 디버그 로그 추가
// ============================================================================
void DualArmForceControl::TargetArmPositionCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_arm_control_mode_ != "inverse") return;
    if (!msg || msg->data.size() < 12) return;
    if (!arm_ik_l_ || !arm_ik_r_) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    std::array<double,3> l_xyz_raw{msg->data[0], msg->data[1], msg->data[2]};
    std::array<double,3> l_eul    {msg->data[3], msg->data[4], msg->data[5]};
    std::array<double,3> r_xyz_raw{msg->data[6], msg->data[7], msg->data[8]};
    std::array<double,3> r_eul    {msg->data[9], msg->data[10], msg->data[11]};

    auto finite3 = [](const std::array<double,3>& v) {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    };

    if (!finite3(l_xyz_raw) || !finite3(l_eul) || !finite3(r_xyz_raw) || !finite3(r_eul)) {
        RCLCPP_WARN(logger, "[TargetArmPositionCallback] Non-finite input detected. Ignore.");
        return;
    }

    constexpr bool kInputPoseIsWorldFrame = false;
    constexpr double z_offset = 0.306;

    auto toLower = [](std::string s) {
        for (auto &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    };

    const std::string ik_frame = toLower(ik_targets_frame_);
    const bool ik_expects_base =
        (ik_frame == "base" || ik_frame == "robot_base" || ik_frame == "local");

    std::array<double,3> l_xyz_ik = l_xyz_raw;
    std::array<double,3> r_xyz_ik = r_xyz_raw;

    bool l_offset_applied = false;
    bool r_offset_applied = false;

    if (kInputPoseIsWorldFrame && ik_expects_base) {
        l_xyz_ik[2] -= z_offset;
        r_xyz_ik[2] -= z_offset;
        l_offset_applied = true;
        r_offset_applied = true;
    }

    std::vector<double> ql(6), qr(6);
    for (int i = 0; i < 6; ++i) {
        ql[i] = q_l_c_(i);
        qr[i] = q_r_c_(i);
    }

    std::vector<double> rl, rr;
    bool l_ok = arm_ik_l_->solveIK(
        ql, l_xyz_ik, l_eul, ik_targets_frame_, ik_euler_conv_, ik_angle_unit_, rl);

    bool r_ok = arm_ik_r_->solveIK(
        qr, r_xyz_ik, r_eul, ik_targets_frame_, ik_euler_conv_, ik_angle_unit_, rr);

    if (l_ok && rl.size() >= 6) {
        for (int i = 0; i < 6; ++i) q_l_t_(i) = rl[i];
    } else {
        RCLCPP_WARN(logger, "[IK][L] solveIK failed or invalid output size. ok=%d size=%zu",
                    static_cast<int>(l_ok), rl.size());
    }

    if (r_ok && rr.size() >= 6) {
        for (int i = 0; i < 6; ++i) q_r_t_(i) = rr[i];
    } else {
        RCLCPP_WARN(logger, "[IK][R] solveIK failed or invalid output size. ok=%d size=%zu",
                    static_cast<int>(r_ok), rr.size());
    }

    target_pose_l_.position.x = l_xyz_raw[0];
    target_pose_l_.position.y = l_xyz_raw[1];
    target_pose_l_.position.z = l_xyz_raw[2];

    target_pose_r_.position.x = r_xyz_raw[0];
    target_pose_r_.position.y = r_xyz_raw[1];
    target_pose_r_.position.z = r_xyz_raw[2];

    auto eulToQuat = [&](double ex, double ey, double ez) {
        double a0 = ex, a1 = ey, a2 = ez;
        if (ik_angle_unit_ == "deg") {
            a0 *= M_PI / 180.0;
            a1 *= M_PI / 180.0;
            a2 *= M_PI / 180.0;
        }

        Eigen::AngleAxisd Rx(a0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(a1, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(a2, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q;
        const std::string conv = toLower(ik_euler_conv_);

        if (conv == "zyx" || conv == "rzyx") q = Rz * Ry * Rx;
        else                                 q = Rx * Ry * Rz;

        geometry_msgs::msg::Quaternion qq;
        qq.x = q.x();
        qq.y = q.y();
        qq.z = q.z();
        qq.w = q.w();
        return qq;
    };

    target_pose_l_.orientation = eulToQuat(l_eul[0], l_eul[1], l_eul[2]);
    target_pose_r_.orientation = eulToQuat(r_eul[0], r_eul[1], r_eul[2]);

    static int dbg_decim = 0;
    const bool do_dbg = ((dbg_decim++ % 20) == 0);

    if (do_dbg) {
        RCLCPP_INFO(logger,
            "[TargetArmPosCb] arm_mode=inverse frame=%s euler_conv=%s angle_unit=%s z_offset=%.4f input_is_world=%d",
            ik_targets_frame_.c_str(), ik_euler_conv_.c_str(), ik_angle_unit_.c_str(),
            z_offset, static_cast<int>(kInputPoseIsWorldFrame));

        RCLCPP_INFO(logger,
            "[L] raw xyz=(%.4f %.4f %.4f) -> ik xyz=(%.4f %.4f %.4f) offset=%d | eul=(%.3f %.3f %.3f) | ok=%d",
            l_xyz_raw[0], l_xyz_raw[1], l_xyz_raw[2],
            l_xyz_ik[0],  l_xyz_ik[1],  l_xyz_ik[2],
            static_cast<int>(l_offset_applied),
            l_eul[0], l_eul[1], l_eul[2],
            static_cast<int>(l_ok && rl.size() >= 6));

        RCLCPP_INFO(logger,
            "[R] raw xyz=(%.4f %.4f %.4f) -> ik xyz=(%.4f %.4f %.4f) offset=%d | eul=(%.3f %.3f %.3f) | ok=%d",
            r_xyz_raw[0], r_xyz_raw[1], r_xyz_raw[2],
            r_xyz_ik[0],  r_xyz_ik[1],  r_xyz_ik[2],
            static_cast<int>(r_offset_applied),
            r_eul[0], r_eul[1], r_eul[2],
            static_cast<int>(r_ok && rr.size() >= 6));

        if (l_ok && rl.size() >= 6) {
            RCLCPP_INFO(logger,
                "[IK Q L] q_t=[%.3f %.3f %.3f %.3f %.3f %.3f]",
                q_l_t_(0), q_l_t_(1), q_l_t_(2), q_l_t_(3), q_l_t_(4), q_l_t_(5));
        }
        if (r_ok && rr.size() >= 6) {
            RCLCPP_INFO(logger,
                "[IK Q R] q_t=[%.3f %.3f %.3f %.3f %.3f %.3f]",
                q_r_t_(0), q_r_t_(1), q_r_t_(2), q_r_t_(3), q_r_t_(4), q_r_t_(5));
        }
    }
}

// ============================================================================
// TargetHandPositionCallback (inverse hand IK, pos-only)
// ============================================================================
void DualArmForceControl::TargetHandPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_hand_control_mode_ != "inverse") return;
    if (!msg) return;
    if (msg->data.size() < 30) return;

    if (!hand_ik_l_ || !hand_ik_r_) return;
    if (!hand_fk_l_ || !hand_fk_r_) return;

    std::array<Eigen::Vector3d,5> tgt_l;
    std::array<Eigen::Vector3d,5> tgt_r;

    for (int i = 0; i < 5; ++i) {
        const int b = i * 3;
        tgt_l[i] = Eigen::Vector3d(msg->data[b + 0], msg->data[b + 1], msg->data[b + 2]);
    }
    for (int i = 0; i < 5; ++i) {
        const int b = 15 + i * 3;
        tgt_r[i] = Eigen::Vector3d(msg->data[b + 0], msg->data[b + 1], msg->data[b + 2]);
    }

    auto eigen20_to_stdvec20 = [&](const Eigen::VectorXd& qh) -> std::vector<double> {
        std::vector<double> out(20, 0.0);
        const int n = std::min<int>(qh.size(), 20);
        for (int i = 0; i < n; ++i) out[i] = qh(i);
        return out;
    };

    auto near_zero_norm = [](const Eigen::VectorXd& v)->bool {
        if (v.size() == 0) return true;
        return (v.norm() < 1e-12);
    };

    std::vector<double> ql_init20 = near_zero_norm(q_l_h_t_) ? eigen20_to_stdvec20(q_l_h_c_)
                                                             : eigen20_to_stdvec20(q_l_h_t_);
    std::vector<double> qr_init20 = near_zero_norm(q_r_h_t_) ? eigen20_to_stdvec20(q_r_h_c_)
                                                             : eigen20_to_stdvec20(q_r_h_t_);

    dualarm_forcecon::HandInverseKinematics::Options opt;
    opt.max_iters       = 80;
    opt.tol_pos_m       = 5e-4;
    opt.lambda          = 1e-2;
    opt.lambda_min      = 1e-5;
    opt.lambda_max      = 1.0;
    opt.alpha           = 0.8;
    opt.alpha_min       = 0.05;
    opt.max_step        = 0.12;
    opt.mu_posture      = 1e-4;
    opt.use_urdf_like_limits = true;
    opt.verbose         = false;

    opt.mask    = {{true, true, true, true, true}};
    opt.weights = {{1.0, 1.0, 1.0, 1.0, 1.0}};

    std::vector<double> ql_sol20, qr_sol20;
    const bool ok_l = hand_ik_l_->solveIKFingertips(ql_init20, tgt_l, ql_sol20, opt);
    const bool ok_r = hand_ik_r_->solveIKFingertips(qr_init20, tgt_r, qr_sol20, opt);

    if (ok_l && ql_sol20.size() >= 20) {
        for (int i = 0; i < 20; ++i) q_l_h_t_(i) = ql_sol20[i];
    }
    if (ok_r && qr_sol20.size() >= 20) {
        for (int i = 0; i < 20; ++i) q_r_h_t_(i) = qr_sol20[i];
    }

    auto assign_point = [&](geometry_msgs::msg::Point& p, const Eigen::Vector3d& v) {
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
    };

    auto safe_get = [&](const std::vector<Eigen::Vector3d>& v, int idx) -> Eigen::Vector3d {
        if (idx < 0 || idx >= static_cast<int>(v.size())) return Eigen::Vector3d::Zero();
        return v[idx];
    };

    if (ok_l && ql_sol20.size() >= 20) {
        const auto tl_cmd = hand_fk_l_->computeFingertips(ql_sol20);
        assign_point(t_f_l_thumb_,  safe_get(tl_cmd, 0));
        assign_point(t_f_l_index_,  safe_get(tl_cmd, 1));
        assign_point(t_f_l_middle_, safe_get(tl_cmd, 2));
        assign_point(t_f_l_ring_,   safe_get(tl_cmd, 3));
        assign_point(t_f_l_baby_,   safe_get(tl_cmd, 4));
    } else {
        assign_point(t_f_l_thumb_,  tgt_l[0]);
        assign_point(t_f_l_index_,  tgt_l[1]);
        assign_point(t_f_l_middle_, tgt_l[2]);
        assign_point(t_f_l_ring_,   tgt_l[3]);
        assign_point(t_f_l_baby_,   tgt_l[4]);
    }

    if (ok_r && qr_sol20.size() >= 20) {
        const auto tr_cmd = hand_fk_r_->computeFingertips(qr_sol20);
        assign_point(t_f_r_thumb_,  safe_get(tr_cmd, 0));
        assign_point(t_f_r_index_,  safe_get(tr_cmd, 1));
        assign_point(t_f_r_middle_, safe_get(tr_cmd, 2));
        assign_point(t_f_r_ring_,   safe_get(tr_cmd, 3));
        assign_point(t_f_r_baby_,   safe_get(tr_cmd, 4));
    } else {
        assign_point(t_f_r_thumb_,  tgt_r[0]);
        assign_point(t_f_r_index_,  tgt_r[1]);
        assign_point(t_f_r_middle_, tgt_r[2]);
        assign_point(t_f_r_ring_,   tgt_r[3]);
        assign_point(t_f_r_baby_,   tgt_r[4]);
    }

    if (!ok_l || !ok_r) {
        RCLCPP_WARN(node_->get_logger(),
                    "[HandIK] solve result: left=%d right=%d (partial update may be applied)",
                    static_cast<int>(ok_l), static_cast<int>(ok_r));
    }
}

// ============================================================================
// DeltaArmPositionCallback (inverse arm IK, delta command w.r.t latched initial pose)
// ============================================================================
void DualArmForceControl::DeltaArmPositionCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_arm_control_mode_ != "inverse") return;
    if (!msg || msg->data.size() < 12) return;
    if (!arm_ik_l_ || !arm_ik_r_) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    auto finite3 = [](const std::array<double,3>& v) {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    };

    std::array<double,3> l_dxyz{msg->data[0],  msg->data[1],  msg->data[2]};
    std::array<double,3> l_deul{msg->data[3],  msg->data[4],  msg->data[5]};
    std::array<double,3> r_dxyz{msg->data[6],  msg->data[7],  msg->data[8]};
    std::array<double,3> r_deul{msg->data[9],  msg->data[10], msg->data[11]};

    if (!finite3(l_dxyz) || !finite3(l_deul) || !finite3(r_dxyz) || !finite3(r_deul)) {
        RCLCPP_WARN(logger, "[DeltaArmPositionCallback] Non-finite delta input detected. Ignore.");
        return;
    }

    if (!delta_arm_base_pose_initialized_) {
        RCLCPP_WARN(logger, "[DeltaArmPositionCallback] base pose is not initialized yet.");
        return;
    }

    std::array<double,3> l_xyz_base{
        delta_arm_base_pose_l_.position.x,
        delta_arm_base_pose_l_.position.y,
        delta_arm_base_pose_l_.position.z
    };
    std::array<double,3> r_xyz_base{
        delta_arm_base_pose_r_.position.x,
        delta_arm_base_pose_r_.position.y,
        delta_arm_base_pose_r_.position.z
    };

    double l_ex_deg = 0.0, l_ey_deg = 0.0, l_ez_deg = 0.0;
    double r_ex_deg = 0.0, r_ey_deg = 0.0, r_ez_deg = 0.0;

    ArmForwardKinematics::quatToEulerXYZDeg_Isaac(
        delta_arm_base_pose_l_.orientation, l_ex_deg, l_ey_deg, l_ez_deg);
    ArmForwardKinematics::quatToEulerXYZDeg_Isaac(
        delta_arm_base_pose_r_.orientation, r_ex_deg, r_ey_deg, r_ez_deg);

    auto deg_to_current_unit = [&](double deg_val)->double {
        if (ik_angle_unit_ == "deg") return deg_val;
        return deg_val * M_PI / 180.0;
    };

    std::array<double,3> l_eul_base{
        deg_to_current_unit(l_ex_deg),
        deg_to_current_unit(l_ey_deg),
        deg_to_current_unit(l_ez_deg)
    };
    std::array<double,3> r_eul_base{
        deg_to_current_unit(r_ex_deg),
        deg_to_current_unit(r_ey_deg),
        deg_to_current_unit(r_ez_deg)
    };

    std_msgs::msg::Float64MultiArray abs_msg;
    abs_msg.data.resize(12);

    abs_msg.data[0] = l_xyz_base[0] + l_dxyz[0];
    abs_msg.data[1] = l_xyz_base[1] + l_dxyz[1];
    abs_msg.data[2] = l_xyz_base[2] + l_dxyz[2];
    abs_msg.data[3] = l_eul_base[0] + l_deul[0];
    abs_msg.data[4] = l_eul_base[1] + l_deul[1];
    abs_msg.data[5] = l_eul_base[2] + l_deul[2];

    abs_msg.data[6]  = r_xyz_base[0] + r_dxyz[0];
    abs_msg.data[7]  = r_xyz_base[1] + r_dxyz[1];
    abs_msg.data[8]  = r_xyz_base[2] + r_dxyz[2];
    abs_msg.data[9]  = r_eul_base[0] + r_deul[0];
    abs_msg.data[10] = r_eul_base[1] + r_deul[1];
    abs_msg.data[11] = r_eul_base[2] + r_deul[2];

    static int dbg_decim = 0;
    const bool do_dbg = ((dbg_decim++ % 20) == 0);

    if (do_dbg) {
        RCLCPP_INFO(logger,
            "[DeltaArmPosCb] arm_mode=inverse angle_unit=%s euler_conv=%s | delta interpreted w.r.t latched initial pose",
            ik_angle_unit_.c_str(), ik_euler_conv_.c_str());

        RCLCPP_INFO(logger,
            "[L] base xyz=(%.4f %.4f %.4f) + dxyz=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
            l_xyz_base[0], l_xyz_base[1], l_xyz_base[2],
            l_dxyz[0], l_dxyz[1], l_dxyz[2],
            abs_msg.data[0], abs_msg.data[1], abs_msg.data[2]);

        RCLCPP_INFO(logger,
            "[L] base eul=(%.4f %.4f %.4f) + deul=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
            l_eul_base[0], l_eul_base[1], l_eul_base[2],
            l_deul[0], l_deul[1], l_deul[2],
            abs_msg.data[3], abs_msg.data[4], abs_msg.data[5]);

        RCLCPP_INFO(logger,
            "[R] base xyz=(%.4f %.4f %.4f) + dxyz=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
            r_xyz_base[0], r_xyz_base[1], r_xyz_base[2],
            r_dxyz[0], r_dxyz[1], r_dxyz[2],
            abs_msg.data[6], abs_msg.data[7], abs_msg.data[8]);

        RCLCPP_INFO(logger,
            "[R] base eul=(%.4f %.4f %.4f) + deul=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
            r_eul_base[0], r_eul_base[1], r_eul_base[2],
            r_deul[0], r_deul[1], r_deul[2],
            abs_msg.data[9], abs_msg.data[10], abs_msg.data[11]);
    }

    auto abs_msg_ptr = std::make_shared<std_msgs::msg::Float64MultiArray>(abs_msg);
    TargetArmPositionCallback(abs_msg_ptr);
}

// ============================================================================
// DeltaHandPositionCallback (inverse hand IK, delta command for one fingertip)
// ============================================================================
void DualArmForceControl::DeltaHandPositionCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_hand_control_mode_ != "inverse") return;
    if (!msg || msg->data.size() < 5) return;
    if (!hand_ik_l_ || !hand_ik_r_) return;
    if (!hand_fk_l_ || !hand_fk_r_) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    const double side_d   = msg->data[0];
    const double finger_d = msg->data[1];

    if (!std::isfinite(side_d) || !std::isfinite(finger_d)) {
        RCLCPP_WARN(logger, "[DeltaHandPositionCallback] Non-finite side/finger input. Ignore.");
        return;
    }

    const int side   = static_cast<int>(std::llround(side_d));
    const int finger = static_cast<int>(std::llround(finger_d));

    if (!(side == 0 || side == 1)) {
        RCLCPP_WARN(logger, "[DeltaHandPositionCallback] invalid side=%d (use 0:left, 1:right)", side);
        return;
    }
    if (finger < 0 || finger > 4) {
        RCLCPP_WARN(logger, "[DeltaHandPositionCallback] invalid finger=%d (use 0..4)", finger);
        return;
    }

    const Eigen::Vector3d dxyz(msg->data[2], msg->data[3], msg->data[4]);
    if (!std::isfinite(dxyz.x()) || !std::isfinite(dxyz.y()) || !std::isfinite(dxyz.z())) {
        RCLCPP_WARN(logger, "[DeltaHandPositionCallback] Non-finite delta xyz input. Ignore.");
        return;
    }

    auto point_to_vec3 = [](const geometry_msgs::msg::Point& p) -> Eigen::Vector3d {
        return Eigen::Vector3d(p.x, p.y, p.z);
    };

    std::array<Eigen::Vector3d,5> cur_l;
    std::array<Eigen::Vector3d,5> cur_r;

    cur_l[0] = point_to_vec3(f_l_thumb_);
    cur_l[1] = point_to_vec3(f_l_index_);
    cur_l[2] = point_to_vec3(f_l_middle_);
    cur_l[3] = point_to_vec3(f_l_ring_);
    cur_l[4] = point_to_vec3(f_l_baby_);

    cur_r[0] = point_to_vec3(f_r_thumb_);
    cur_r[1] = point_to_vec3(f_r_index_);
    cur_r[2] = point_to_vec3(f_r_middle_);
    cur_r[3] = point_to_vec3(f_r_ring_);
    cur_r[4] = point_to_vec3(f_r_baby_);

    std::array<Eigen::Vector3d,5> tgt_l = cur_l;
    std::array<Eigen::Vector3d,5> tgt_r = cur_r;

    if (side == 0) tgt_l[static_cast<size_t>(finger)] += dxyz;
    else           tgt_r[static_cast<size_t>(finger)] += dxyz;

    std_msgs::msg::Float64MultiArray abs_msg;
    abs_msg.data.resize(30, 0.0);

    for (int i = 0; i < 5; ++i) {
        const int b = i * 3;
        abs_msg.data[b + 0] = tgt_l[i].x();
        abs_msg.data[b + 1] = tgt_l[i].y();
        abs_msg.data[b + 2] = tgt_l[i].z();
    }

    for (int i = 0; i < 5; ++i) {
        const int b = 15 + i * 3;
        abs_msg.data[b + 0] = tgt_r[i].x();
        abs_msg.data[b + 1] = tgt_r[i].y();
        abs_msg.data[b + 2] = tgt_r[i].z();
    }

    static int dbg_decim = 0;
    const bool do_dbg = ((dbg_decim++ % 20) == 0);

    if (do_dbg) {
        const Eigen::Vector3d p_cur = (side == 0) ? cur_l[static_cast<size_t>(finger)]
                                                  : cur_r[static_cast<size_t>(finger)];
        const Eigen::Vector3d p_tgt = (side == 0) ? tgt_l[static_cast<size_t>(finger)]
                                                  : tgt_r[static_cast<size_t>(finger)];

        RCLCPP_INFO(logger,
            "[DeltaHandPosCb] hand_mode=inverse side=%s finger=%d | cur=(%.4f %.4f %.4f) + d=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
            (side == 0 ? "L" : "R"),
            finger,
            p_cur.x(), p_cur.y(), p_cur.z(),
            dxyz.x(), dxyz.y(), dxyz.z(),
            p_tgt.x(), p_tgt.y(), p_tgt.z());
    }

    auto abs_msg_ptr = std::make_shared<std_msgs::msg::Float64MultiArray>(abs_msg);
    TargetHandPositionCallback(abs_msg_ptr);
}

// --------------------
// TargetArmJointsCallback (forward)
// --------------------
void DualArmForceControl::TargetArmJointsCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_arm_control_mode_ != "forward") return;
    if (!msg) return;
    if (msg->data.size() < 12) return;

    for (int i = 0; i < 6; ++i) {
        q_l_t_(i) = msg->data[i + 0];
        q_r_t_(i) = msg->data[i + 6];
    }
}

// --------------------
// TargetHandJointsCallback (forward)
// --------------------
void DualArmForceControl::TargetHandJointsCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_hand_control_mode_ != "forward") return;
    if (!msg) return;

    const size_t n = msg->data.size();

    auto enforce_mimic_q4_eq_q3 = [&](Eigen::VectorXd& qh20) {
        if (qh20.size() < 20) return;
        for (int f = 0; f < 5; ++f) {
            const int b = f * 4;
            qh20(b + 3) = qh20(b + 2);
        }
    };

    auto assign_hand15_to_qh20 = [&](Eigen::VectorXd& qh20, const size_t offset) {
        if (qh20.size() < 20) return;
        for (int f = 0; f < 5; ++f) {
            const int b15 = f * 3;
            const int b20 = f * 4;

            qh20(b20 + 0) = msg->data[offset + b15 + 0];
            qh20(b20 + 1) = msg->data[offset + b15 + 1];
            qh20(b20 + 2) = msg->data[offset + b15 + 2];
            qh20(b20 + 3) = msg->data[offset + b15 + 2];
        }
    };

    auto assign_hand20_to_qh20 = [&](Eigen::VectorXd& qh20, const size_t offset) {
        if (qh20.size() < 20) return;
        for (int i = 0; i < 20; ++i) qh20(i) = msg->data[offset + i];
        enforce_mimic_q4_eq_q3(qh20);
    };

    if (n >= 40) {
        assign_hand20_to_qh20(q_l_h_t_, 0);
        assign_hand20_to_qh20(q_r_h_t_, 20);
        return;
    }

    if (n >= 30) {
        assign_hand15_to_qh20(q_l_h_t_, 0);
        assign_hand15_to_qh20(q_r_h_t_, 15);
        return;
    }

    if (n >= 52) {
        assign_hand20_to_qh20(q_l_h_t_, 12);
        assign_hand20_to_qh20(q_r_h_t_, 32);
        return;
    }

    if (n >= 42) {
        assign_hand15_to_qh20(q_l_h_t_, 12);
        assign_hand15_to_qh20(q_r_h_t_, 27);
        return;
    }
}

void DualArmForceControl::TargetHandForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_hand_control_mode_ != "forcecon") return;
    if (!msg) return;
    if (msg->data.size() < 8) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    const double hand_id_d   = msg->data[0];
    const double finger_id_d = msg->data[1];

    if (!std::isfinite(hand_id_d) || !std::isfinite(finger_id_d)) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] non-finite hand_id/finger_id");
        return;
    }

    const int hand_id   = static_cast<int>(std::llround(hand_id_d));
    const int finger_id = static_cast<int>(std::llround(finger_id_d));

    if (!(hand_id == 0 || hand_id == 1)) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] invalid hand_id=%d (use 0:left, 1:right)", hand_id);
        return;
    }
    if (finger_id < 0 || finger_id > 4) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] invalid finger_id=%d (use 0..4)", finger_id);
        return;
    }

    Eigen::Vector3d p_des_base(msg->data[2], msg->data[3], msg->data[4]);
    Eigen::Vector3d f_des_base(msg->data[5], msg->data[6], msg->data[7]);

    if (!(std::isfinite(p_des_base.x()) && std::isfinite(p_des_base.y()) && std::isfinite(p_des_base.z()) &&
          std::isfinite(f_des_base.x()) && std::isfinite(f_des_base.y()) && std::isfinite(f_des_base.z()))) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] non-finite desired pose/force");
        return;
    }

    const bool is_left = (hand_id == 0);
    const bool first_force_cmd_in_session = (!hand_force_cmd_valid_);

    if (first_force_cmd_in_session) {
        const int n_la = std::min<int>(q_l_t_.size(), q_l_c_.size());
        const int n_ra = std::min<int>(q_r_t_.size(), q_r_c_.size());
        for (int i = 0; i < n_la; ++i) q_l_t_(i) = q_l_c_(i);
        for (int i = 0; i < n_ra; ++i) q_r_t_(i) = q_r_c_(i);

        const int n_lh = std::min<int>(q_l_h_t_.size(), q_l_h_c_.size());
        const int n_rh = std::min<int>(q_r_h_t_.size(), q_r_h_c_.size());
        for (int i = 0; i < n_lh; ++i) q_l_h_t_(i) = q_l_h_c_(i);
        for (int i = 0; i < n_rh; ++i) q_r_h_t_(i) = q_r_h_c_(i);

        target_pose_l_ = current_pose_l_;
        target_pose_r_ = current_pose_r_;

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

        RCLCPP_INFO(logger,
            "[TargetHandForceCb] forcecon hold target latched (first cmd in session). arm/hand non-target joints fixed.");
    }

    f_l_hand_t_.setZero();
    f_r_hand_t_.setZero();

    if (is_left) f_l_hand_t_.row(finger_id) = f_des_base.transpose();
    else         f_r_hand_t_.row(finger_id) = f_des_base.transpose();

    auto setFingerTargetPoint = [&](bool left, int fid, const Eigen::Vector3d& p) {
        geometry_msgs::msg::Point pt;
        pt.x = p.x(); pt.y = p.y(); pt.z = p.z();

        if (left) {
            switch (fid) {
                case 0: t_f_l_thumb_  = pt; break;
                case 1: t_f_l_index_  = pt; break;
                case 2: t_f_l_middle_ = pt; break;
                case 3: t_f_l_ring_   = pt; break;
                case 4: t_f_l_baby_   = pt; break;
                default: break;
            }
        } else {
            switch (fid) {
                case 0: t_f_r_thumb_  = pt; break;
                case 1: t_f_r_index_  = pt; break;
                case 2: t_f_r_middle_ = pt; break;
                case 3: t_f_r_ring_   = pt; break;
                case 4: t_f_r_baby_   = pt; break;
                default: break;
            }
        }
    };
    setFingerTargetPoint(is_left, finger_id, p_des_base);

    hand_force_cmd_valid_      = true;
    hand_force_cmd_hand_id_    = hand_id;
    hand_force_cmd_finger_id_  = finger_id;
    hand_force_cmd_p_des_base_ = p_des_base;
    hand_force_cmd_f_des_base_ = f_des_base;
    hand_force_cmd_stamp_ns_   = node_ ? node_->get_clock()->now().nanoseconds() : 0;

    static int prev_key = -1;
    const int key = (is_left ? 0 : 5) + finger_id;

    if (first_force_cmd_in_session || key != prev_key) {
        auto& adm_arr = is_left ? hand_adm_l_ : hand_adm_r_;
        if (adm_arr[static_cast<size_t>(finger_id)]) {
            Eigen::Vector3d p_init = p_des_base;
            if (is_left) {
                switch (finger_id) {
                    case 0: p_init = Eigen::Vector3d(f_l_thumb_.x,  f_l_thumb_.y,  f_l_thumb_.z);  break;
                    case 1: p_init = Eigen::Vector3d(f_l_index_.x,  f_l_index_.y,  f_l_index_.z);  break;
                    case 2: p_init = Eigen::Vector3d(f_l_middle_.x, f_l_middle_.y, f_l_middle_.z); break;
                    case 3: p_init = Eigen::Vector3d(f_l_ring_.x,   f_l_ring_.y,   f_l_ring_.z);   break;
                    case 4: p_init = Eigen::Vector3d(f_l_baby_.x,   f_l_baby_.y,   f_l_baby_.z);   break;
                }
            } else {
                switch (finger_id) {
                    case 0: p_init = Eigen::Vector3d(f_r_thumb_.x,  f_r_thumb_.y,  f_r_thumb_.z);  break;
                    case 1: p_init = Eigen::Vector3d(f_r_index_.x,  f_r_index_.y,  f_r_index_.z);  break;
                    case 2: p_init = Eigen::Vector3d(f_r_middle_.x, f_r_middle_.y, f_r_middle_.z); break;
                    case 3: p_init = Eigen::Vector3d(f_r_ring_.x,   f_r_ring_.y,   f_r_ring_.z);   break;
                    case 4: p_init = Eigen::Vector3d(f_r_baby_.x,   f_r_baby_.y,   f_r_baby_.z);   break;
                }
            }

            adm_arr[static_cast<size_t>(finger_id)]->resetState(p_init);
        }
        prev_key = key;
    }

    static int dbg_decim = 0;
    if ((dbg_decim++ % 20) == 0) {
        RCLCPP_INFO(logger,
            "[TargetHandForceCb][LATCH] hand_mode=forcecon side=%s finger=%d | first=%d | "
            "p_des=(%.4f %.4f %.4f) | f_des=(%.3f %.3f %.3f) | execution=ControlLoop | arm_target=HOLD_LATCHED",
            is_left ? "L" : "R",
            finger_id,
            static_cast<int>(first_force_cmd_in_session),
            p_des_base.x(), p_des_base.y(), p_des_base.z(),
            f_des_base.x(), f_des_base.y(), f_des_base.z());
    }
}