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
    if (current_control_mode_ != "inverse") return;
    if (!msg || msg->data.size() < 12) return;
    if (!arm_ik_l_ || !arm_ik_r_) return;

    // ROS logger (DualArmForceControl는 Node 상속이 아닐 수 있으므로 node_ 사용)
    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    // -----------------------------
    // [A] 입력 파싱 (RAW command)
    // -----------------------------
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

    // -----------------------------
    // [B] frame / z-offset 처리
    // -----------------------------
    // 현재 패치:
    // /target_arm_cartesian_pose 입력은 base frame 기준으로 사용한다.
    // 따라서 world->base z offset 보정을 적용하지 않는다.
    //
    // (나중에 world frame 입력으로 다시 바꾸고 싶으면 true로 변경)
    constexpr bool kInputPoseIsWorldFrame = false;

    // TODO: 네 프로젝트에서 실제 offset 멤버명이 있다면 아래 상수 대신 그 멤버로 교체
    // 예) const double z_offset = world_base_z_offset_; / base_z_offset_ / ...
    constexpr double z_offset = 0.306;  // [m] dualarm_forcecon baseline default (memory 기준)

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

    // 입력이 world이고 IK가 base를 기대할 때만 z offset 1회 적용
    if (kInputPoseIsWorldFrame && ik_expects_base) {
        l_xyz_ik[2] -= z_offset;
        r_xyz_ik[2] -= z_offset;
        l_offset_applied = true;
        r_offset_applied = true;
    }

    // -----------------------------
    // [C] 현재 joint seed 준비
    // -----------------------------
    std::vector<double> ql(6), qr(6);
    for (int i = 0; i < 6; ++i) {
        ql[i] = q_l_c_(i);
        qr[i] = q_r_c_(i);
    }

    // -----------------------------
    // [D] IK solve
    // -----------------------------
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

    // -----------------------------
    // [E] 모니터용 target pose 저장 (RAW 입력값 기준)
    //     => UI에서 사용자가 보낸 값 그대로 표시되게 유지
    // -----------------------------
    target_pose_l_.position.x = l_xyz_raw[0];
    target_pose_l_.position.y = l_xyz_raw[1];
    target_pose_l_.position.z = l_xyz_raw[2];

    target_pose_r_.position.x = r_xyz_raw[0];
    target_pose_r_.position.y = r_xyz_raw[1];
    target_pose_r_.position.z = r_xyz_raw[2];

    // Euler -> Quaternion (표시용; IK 규칙과 최대한 맞춤)
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
        else                                 q = Rx * Ry * Rz;  // default

        geometry_msgs::msg::Quaternion qq;
        qq.x = q.x();
        qq.y = q.y();
        qq.z = q.z();
        qq.w = q.w();
        return qq;
    };

    target_pose_l_.orientation = eulToQuat(l_eul[0], l_eul[1], l_eul[2]);
    target_pose_r_.orientation = eulToQuat(r_eul[0], r_eul[1], r_eul[2]);

    // -----------------------------
    // [F] 디버그 로그 (과도한 스팸 방지: 20회당 1회)
    // -----------------------------
    static int dbg_decim = 0;
    const bool do_dbg = ((dbg_decim++ % 20) == 0);

    if (do_dbg) {
        RCLCPP_INFO(logger,
            "[TargetArmPosCb] mode=inverse frame=%s euler_conv=%s angle_unit=%s z_offset=%.4f input_is_world=%d",
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
// v11: TargetHandPositionCallback (inverse hand IK, pos-only)
// msg size:
//   - 15: LEFT only  (thumb..baby each xyz)  -> right는 hold
//   - 30: LEFT(0..14) + RIGHT(15..29)
// order per hand = [thumb xyz, index xyz, middle xyz, ring xyz, baby xyz]
// all positions expressed in each HAND_BASE frame (left_joint_6/right_joint_6)
// ============================================================================
void DualArmForceControl::TargetHandPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_control_mode_ != "inverse") return;
    if (!msg) return;
    if (msg->data.size() < 30) return;

    if (!hand_ik_l_ || !hand_ik_r_) return;
    if (!hand_fk_l_ || !hand_fk_r_) return;

    // ------------------------------------------------------------
    // 1) Parse target fingertip positions (HAND_BASE frame)
    //    order: thumb,index,middle,ring,baby
    //    data: left(15) + right(15)
    // ------------------------------------------------------------
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

    // ------------------------------------------------------------
    // 2) Initial guess (prefer current target joints for continuity)
    //    q_*_h_t_ / q_*_h_c_ are stored as 20DoF representation
    // ------------------------------------------------------------
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

    // ------------------------------------------------------------
    // 3) IK options (v13: independent finger IK)
    // ------------------------------------------------------------
    dualarm_forcecon::HandInverseKinematics::Options opt;
    opt.max_iters       = 80;
    opt.tol_pos_m       = 5e-4;   // 0.5 mm 수준
    opt.lambda          = 1e-2;
    opt.lambda_min      = 1e-5;
    opt.lambda_max      = 1.0;
    opt.alpha           = 0.8;
    opt.alpha_min       = 0.05;
    opt.max_step        = 0.12;
    opt.mu_posture      = 1e-4;
    opt.use_urdf_like_limits = true;
    opt.verbose         = false;

    // 기본은 모두 활성
    opt.mask    = {{true, true, true, true, true}};
    opt.weights = {{1.0, 1.0, 1.0, 1.0, 1.0}};

    // ------------------------------------------------------------
    // 4) Solve hand IK (returns 20DoF representation, with q4=q3 mimic)
    // ------------------------------------------------------------
    std::vector<double> ql_sol20, qr_sol20;
    const bool ok_l = hand_ik_l_->solveIKFingertips(ql_init20, tgt_l, ql_sol20, opt);
    const bool ok_r = hand_ik_r_->solveIKFingertips(qr_init20, tgt_r, qr_sol20, opt);

    if (ok_l && ql_sol20.size() >= 20) {
        for (int i = 0; i < 20; ++i) q_l_h_t_(i) = ql_sol20[i];
    }
    if (ok_r && qr_sol20.size() >= 20) {
        for (int i = 0; i < 20; ++i) q_r_h_t_(i) = qr_sol20[i];
    }

    // ------------------------------------------------------------
    // 5) Update target fingertip display
    //    - 성공 시: solved joints로 FK한 실제 command tip 표시
    //    - 실패 시: requested target 그대로 표시
    // ------------------------------------------------------------
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

    // (선택) 로그
    if (!ok_l || !ok_r) {
        // 필요하면 throttle 로그로 바꿔도 됨
        RCLCPP_WARN(node_->get_logger(),
                    "[HandIK v13] solve result: left=%d right=%d (partial update may be applied)",
                    static_cast<int>(ok_l), static_cast<int>(ok_r));
    }
}

// ============================================================================
// v15: DeltaArmPositionCallback (inverse arm IK, delta command)
// msg: 12 = [L dx dy dz d(rx) d(ry) d(rz), R dx dy dz d(rx) d(ry) d(rz)]
//
// 동작:
//   absolute_target = current_pose(world monitor pose) + delta
//   -> TargetArmPositionCallback() 재사용 (frame/z-offset/IK 로직 일원화)
//
// 주의:
// - position delta는 "world 표시 좌표계 기준"으로 더해짐
// - orientation delta는 "Euler component-wise add" 방식 (현재 설정 unit 기준)
//   (SO(3) composition이 아니라 사용자가 요청한 단순 delta 방식)
// ============================================================================
void DualArmForceControl::DeltaArmPositionCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_control_mode_ != "inverse") return;
    if (!msg || msg->data.size() < 12) return;
    if (!arm_ik_l_ || !arm_ik_r_) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    auto finite3 = [](const std::array<double,3>& v) {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    };

    // -----------------------------
    // [A] delta 입력 파싱
    // -----------------------------
    std::array<double,3> l_dxyz{msg->data[0],  msg->data[1],  msg->data[2]};
    std::array<double,3> l_deul{msg->data[3],  msg->data[4],  msg->data[5]};
    std::array<double,3> r_dxyz{msg->data[6],  msg->data[7],  msg->data[8]};
    std::array<double,3> r_deul{msg->data[9],  msg->data[10], msg->data[11]};

    if (!finite3(l_dxyz) || !finite3(l_deul) || !finite3(r_dxyz) || !finite3(r_deul)) {
        RCLCPP_WARN(logger, "[DeltaArmPositionCallback] Non-finite delta input detected. Ignore.");
        return;
    }

    // -----------------------------
    // [B] 현재 pose(world monitor pose) 읽기
    // -----------------------------
    // current_pose_l_/r_ 는 PositionCallback/FK에서 업데이트되는 world pose라고 가정
    std::array<double,3> l_xyz_cur{
        current_pose_l_.position.x,
        current_pose_l_.position.y,
        current_pose_l_.position.z
    };
    std::array<double,3> r_xyz_cur{
        current_pose_r_.position.x,
        current_pose_r_.position.y,
        current_pose_r_.position.z
    };

    // 현재 orientation quaternion -> Euler(XYZ, deg) (Isaac UI 매칭용 helper 재사용)
    double l_ex_deg = 0.0, l_ey_deg = 0.0, l_ez_deg = 0.0;
    double r_ex_deg = 0.0, r_ey_deg = 0.0, r_ez_deg = 0.0;

    ArmForwardKinematics::quatToEulerXYZDeg_Isaac(current_pose_l_.orientation, l_ex_deg, l_ey_deg, l_ez_deg);
    ArmForwardKinematics::quatToEulerXYZDeg_Isaac(current_pose_r_.orientation, r_ex_deg, r_ey_deg, r_ez_deg);

    auto deg_to_current_unit = [&](double deg_val)->double {
        // TargetArmPositionCallback에서 angle_unit_ 해석을 다시 하므로
        // 여기서는 "현재 Euler 표현값"을 delta와 같은 단위로 맞춰서 합산만 수행
        if (ik_angle_unit_ == "deg") return deg_val;
        // rad / auto 는 rad 기준으로 합산 (auto에서도 rad로 넣어도 solveIK에서 허용)
        return deg_val * M_PI / 180.0;
    };

    std::array<double,3> l_eul_cur{
        deg_to_current_unit(l_ex_deg),
        deg_to_current_unit(l_ey_deg),
        deg_to_current_unit(l_ez_deg)
    };
    std::array<double,3> r_eul_cur{
        deg_to_current_unit(r_ex_deg),
        deg_to_current_unit(r_ey_deg),
        deg_to_current_unit(r_ez_deg)
    };

    // -----------------------------
    // [C] 절대 target 생성 = current + delta
    // -----------------------------
    std_msgs::msg::Float64MultiArray abs_msg;
    abs_msg.data.resize(12);

    // Left XYZ
    abs_msg.data[0] = l_xyz_cur[0] + l_dxyz[0];
    abs_msg.data[1] = l_xyz_cur[1] + l_dxyz[1];
    abs_msg.data[2] = l_xyz_cur[2] + l_dxyz[2];
    // Left Euler
    abs_msg.data[3] = l_eul_cur[0] + l_deul[0];
    abs_msg.data[4] = l_eul_cur[1] + l_deul[1];
    abs_msg.data[5] = l_eul_cur[2] + l_deul[2];

    // Right XYZ
    abs_msg.data[6]  = r_xyz_cur[0] + r_dxyz[0];
    abs_msg.data[7]  = r_xyz_cur[1] + r_dxyz[1];
    abs_msg.data[8]  = r_xyz_cur[2] + r_dxyz[2];
    // Right Euler
    abs_msg.data[9]  = r_eul_cur[0] + r_deul[0];
    abs_msg.data[10] = r_eul_cur[1] + r_deul[1];
    abs_msg.data[11] = r_eul_cur[2] + r_deul[2];

    // -----------------------------
    // [D] 디버그 로그 (스팸 방지)
    // -----------------------------
    static int dbg_decim = 0;
    const bool do_dbg = ((dbg_decim++ % 20) == 0);

    if (do_dbg) {
        RCLCPP_INFO(logger,
            "[DeltaArmPosCb] angle_unit=%s euler_conv=%s | delta interpreted as current-display-frame increments",
            ik_angle_unit_.c_str(), ik_euler_conv_.c_str());

        RCLCPP_INFO(logger,
            "[L] cur xyz=(%.4f %.4f %.4f) + dxyz=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
            l_xyz_cur[0], l_xyz_cur[1], l_xyz_cur[2],
            l_dxyz[0], l_dxyz[1], l_dxyz[2],
            abs_msg.data[0], abs_msg.data[1], abs_msg.data[2]);

        RCLCPP_INFO(logger,
            "[R] cur xyz=(%.4f %.4f %.4f) + dxyz=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
            r_xyz_cur[0], r_xyz_cur[1], r_xyz_cur[2],
            r_dxyz[0], r_dxyz[1], r_dxyz[2],
            abs_msg.data[6], abs_msg.data[7], abs_msg.data[8]);
    }

    // -----------------------------
    // [E] 기존 절대 target callback 재사용 (v13/v14 z-offset/frame 처리 일관성 유지)
    // -----------------------------
    auto abs_msg_ptr = std::make_shared<std_msgs::msg::Float64MultiArray>(abs_msg);
    TargetArmPositionCallback(abs_msg_ptr);
}


// ============================================================================
// DeltaHandPositionCallback (inverse hand IK, delta command for one fingertip)
// msg: 5 = [side, finger, dx, dy, dz]
//
// side   : 0=left, 1=right
// finger : 0=thumb, 1=index, 2=middle, 3=ring, 4=baby
// dxyz   : fingertip delta in each HAND_BASE frame [m]
//
// 동작:
//   absolute_target_fingertips = current_fingertips + delta(one selected finger)
//   -> TargetHandPositionCallback() 재사용 (IK/모니터 업데이트 로직 일원화)
//
// 주의:
// - delta는 각 손의 HAND_BASE frame 기준
// - 현재 fingertip 위치는 f_* (CUR monitor) 값을 사용
// - inverse 모드에서만 동작
// ============================================================================
void DualArmForceControl::DeltaHandPositionCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_control_mode_ != "inverse") return;
    if (!msg || msg->data.size() < 5) return;
    if (!hand_ik_l_ || !hand_ik_r_) return;
    if (!hand_fk_l_ || !hand_fk_r_) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    // -----------------------------
    // [A] 입력 파싱
    // -----------------------------
    const double side_d   = msg->data[0];
    const double finger_d = msg->data[1];

    if (!std::isfinite(side_d) || !std::isfinite(finger_d)) {
        RCLCPP_WARN(logger, "[DeltaHandPositionCallback] Non-finite side/finger input. Ignore.");
        return;
    }

    const int side   = static_cast<int>(std::llround(side_d));    // 0=left, 1=right
    const int finger = static_cast<int>(std::llround(finger_d));  // 0..4 canonical

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

    // -----------------------------
    // [B] 현재 fingertip 위치(CUR monitor) 읽기
    // order per hand = [thumb, index, middle, ring, baby]
    // -----------------------------
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

    // -----------------------------
    // [C] 절대 target 생성 = current + delta (선택 finger만)
    //     TargetHandPositionCallback 포맷(30) 구성:
    //     left(15) + right(15)
    // -----------------------------
    std::array<Eigen::Vector3d,5> tgt_l = cur_l;
    std::array<Eigen::Vector3d,5> tgt_r = cur_r;

    if (side == 0) tgt_l[static_cast<size_t>(finger)] += dxyz;
    else           tgt_r[static_cast<size_t>(finger)] += dxyz;

    std_msgs::msg::Float64MultiArray abs_msg;
    abs_msg.data.resize(30, 0.0);

    // left hand (0..14)
    for (int i = 0; i < 5; ++i) {
        const int b = i * 3;
        abs_msg.data[b + 0] = tgt_l[i].x();
        abs_msg.data[b + 1] = tgt_l[i].y();
        abs_msg.data[b + 2] = tgt_l[i].z();
    }

    // right hand (15..29)
    for (int i = 0; i < 5; ++i) {
        const int b = 15 + i * 3;
        abs_msg.data[b + 0] = tgt_r[i].x();
        abs_msg.data[b + 1] = tgt_r[i].y();
        abs_msg.data[b + 2] = tgt_r[i].z();
    }

    // -----------------------------
    // [D] 디버그 로그 (스팸 방지)
    // -----------------------------
    static int dbg_decim = 0;
    const bool do_dbg = ((dbg_decim++ % 20) == 0);

    if (do_dbg) {
        const Eigen::Vector3d p_cur = (side == 0) ? cur_l[static_cast<size_t>(finger)]
                                                  : cur_r[static_cast<size_t>(finger)];
        const Eigen::Vector3d p_tgt = (side == 0) ? tgt_l[static_cast<size_t>(finger)]
                                                  : tgt_r[static_cast<size_t>(finger)];

        RCLCPP_INFO(logger,
            "[DeltaHandPosCb] side=%s finger=%d | cur=(%.4f %.4f %.4f) + d=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
            (side == 0 ? "L" : "R"),
            finger,
            p_cur.x(), p_cur.y(), p_cur.z(),
            dxyz.x(), dxyz.y(), dxyz.z(),
            p_tgt.x(), p_tgt.y(), p_tgt.z());
    }

    // -----------------------------
    // [E] 기존 절대 target callback 재사용 (IK/표시 일관성 유지)
    // -----------------------------
    auto abs_msg_ptr = std::make_shared<std_msgs::msg::Float64MultiArray>(abs_msg);
    TargetHandPositionCallback(abs_msg_ptr);
}

// --------------------
// TargetArmJointsCallback (forward)
// supports:
//   - 12 : [L arm 6, R arm 6]
// --------------------
void DualArmForceControl::TargetArmJointsCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_control_mode_ != "forward") return;
    if (!msg) return;
    if (msg->data.size() < 12) return;

    for (int i = 0; i < 6; ++i) {
        q_l_t_(i) = msg->data[i + 0];
        q_r_t_(i) = msg->data[i + 6];
    }
}

// --------------------
// TargetHandJointsCallback (forward)
// supports:
//   - 30 : hand15(left) + hand15(right)
//   - 40 : hand20(left) + hand20(right)  (joint4 canonicalized to joint3)
// legacy compatibility (optional):
//   - 42 : arm12 + hand15 + hand15
//   - 52 : arm12 + hand20 + hand20
// --------------------
void DualArmForceControl::TargetHandJointsCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_control_mode_ != "forward") return;
    if (!msg) return;

    const size_t n = msg->data.size();

    // Helper: finger-wise mimic enforce on 20DoF hand vector
    // layout = [thumb1..4, index1..4, middle1..4, ring1..4, baby1..4]
    auto enforce_mimic_q4_eq_q3 = [&](Eigen::VectorXd& qh20) {
        if (qh20.size() < 20) return;
        for (int f = 0; f < 5; ++f) {
            const int b = f * 4;
            qh20(b + 3) = qh20(b + 2);  // joint4 = joint3
        }
    };

    // Helper: 15DoF -> 20DoF expand (joint4 = joint3)
    // input order (15): [thumb1,2,3, index1,2,3, middle1,2,3, ring1,2,3, baby1,2,3]
    auto assign_hand15_to_qh20 = [&](Eigen::VectorXd& qh20, const size_t offset) {
        if (qh20.size() < 20) return;
        for (int f = 0; f < 5; ++f) {
            const int b15 = f * 3;
            const int b20 = f * 4;

            qh20(b20 + 0) = msg->data[offset + b15 + 0];
            qh20(b20 + 1) = msg->data[offset + b15 + 1];
            qh20(b20 + 2) = msg->data[offset + b15 + 2];
            qh20(b20 + 3) = msg->data[offset + b15 + 2]; // mimic
        }
    };

    // Helper: 20DoF direct copy, then canonicalize q4=q3
    auto assign_hand20_to_qh20 = [&](Eigen::VectorXd& qh20, const size_t offset) {
        if (qh20.size() < 20) return;
        for (int i = 0; i < 20; ++i) qh20(i) = msg->data[offset + i];
        enforce_mimic_q4_eq_q3(qh20);
    };

    // ----------------------------
    // Preferred split formats
    // ----------------------------
    if (n >= 40) {
        // 40 = left20 + right20
        assign_hand20_to_qh20(q_l_h_t_, 0);
        assign_hand20_to_qh20(q_r_h_t_, 20);
        return;
    }

    if (n >= 30) {
        // 30 = left15 + right15
        assign_hand15_to_qh20(q_l_h_t_, 0);
        assign_hand15_to_qh20(q_r_h_t_, 15);
        return;
    }

    // ----------------------------
    // Legacy compatibility (optional)
    // ----------------------------
    if (n >= 52) {
        // 12 + 20 + 20
        assign_hand20_to_qh20(q_l_h_t_, 12);
        assign_hand20_to_qh20(q_r_h_t_, 32);
        return;
    }

    if (n >= 42) {
        // 12 + 15 + 15
        assign_hand15_to_qh20(q_l_h_t_, 12);
        assign_hand15_to_qh20(q_r_h_t_, 27);
        return;
    }

    // else: invalid hand message size -> ignore
}

// --------------------
// TargetHandForceCallback (v18, forcecon mode)
// /target_hand_force : Float64MultiArray
//   [hand_id, finger_id, px, py, pz, fx, fy, fz]
//    hand_id   : 0=left, 1=right
//    finger_id : 0=thumb,1=index,2=middle,3=ring,4=baby (canonical order)
//    p*        : desired fingertip position in HAND BASE frame [m]
//    f*        : desired force in HAND BASE frame [N]
//
// 동작:
// - forcecon 모드에서만 동작
// - 선택된 손가락만 hand_admittance_control + HandIK로 q1,q2,q3(q4=q3) 갱신
// - 나머지 arm/hand joint는 현재/기존 target 유지 (mode 진입 시 sync된 값 유지)
// - 실제 publish는 ControlLoop() 타이머가 담당 (/isaac_joint_command)
//
// [중요 패치]
// - TAR_F 표시값은 "파싱 직후" 먼저 갱신한다.
//   => controller/IK 실패해도 print에서 target force는 보이게 함
// --------------------
void DualArmForceControl::TargetHandForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_control_mode_ != "forcecon") return;
    if (!msg) return;
    if (msg->data.size() < 8) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    // ---- parse ----
    const double hand_id_d   = msg->data[0];
    const double finger_id_d = msg->data[1];

    if (!std::isfinite(hand_id_d) || !std::isfinite(finger_id_d)) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] non-finite hand_id/finger_id");
        return;
    }

    const int hand_id   = static_cast<int>(std::llround(hand_id_d));     // 0=left, 1=right
    const int finger_id = static_cast<int>(std::llround(finger_id_d));   // canonical order

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

    // ---- select side resources / state refs ----
    auto& adm_arr = is_left ? hand_adm_l_ : hand_adm_r_;
    auto  adm_ptr = adm_arr[static_cast<size_t>(finger_id)];

    const Eigen::Matrix<double,5,3>& f_hand_c_mat = is_left ? f_l_hand_c_ : f_r_hand_c_;
    Eigen::Matrix<double,5,3>&       f_hand_t_mat = is_left ? f_l_hand_t_ : f_r_hand_t_;
    Eigen::VectorXd&                 q_h_c        = is_left ? q_l_h_c_    : q_r_h_c_;
    Eigen::VectorXd&                 q_h_t        = is_left ? q_l_h_t_    : q_r_h_t_;

    // ---- monitor target force 표시 업데이트 (먼저 수행) ----
    // controller/IK 성공 여부와 무관하게 TAR_F는 사용자가 보낸 명령을 반영
    {
        f_hand_t_mat.setZero();
        f_hand_t_mat.row(finger_id) = f_des_base.transpose();
    }

    // ---- monitor target fingertip position 표시 업데이트 (선택 finger만 먼저 반영) ----
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

    // ---- admittance controller readiness check ----
    if (!adm_ptr || !adm_ptr->isOk()) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] admittance controller not ready. side=%s finger=%d (TAR_F monitor updated)",
                    is_left ? "L" : "R", finger_id);
        return;
    }

    // ---- measured force in HAND BASE frame (already transformed in HandContactForceCallback) ----
    // row order = canonical: thumb,index,middle,ring,baby
    Eigen::Vector3d f_meas_base = f_hand_c_mat.row(finger_id).transpose();

    // ---- current q20 ----
    std::vector<double> q_cur20(20, 0.0);
    for (int i = 0; i < 20 && i < q_h_c.size(); ++i) q_cur20[i] = q_h_c(i);

    // ---- dt estimate (per hand/finger static timer) ----
    // callback 기반 입력 주기를 그대로 사용. 첫 호출은 10ms fallback.
    double dt_s = 0.01;
    if (node_) {
        static std::array<bool, 10>   s_valid = {false,false,false,false,false,false,false,false,false,false};
        static std::array<int64_t,10> s_last_ns{};

        const int key = (is_left ? 0 : 5) + finger_id;
        const auto now = node_->get_clock()->now();
        const int64_t now_ns = now.nanoseconds();

        if (s_valid[key]) {
            dt_s = static_cast<double>(now_ns - s_last_ns[key]) * 1e-9;
        }
        s_last_ns[key] = now_ns;
        s_valid[key] = true;

        if (!std::isfinite(dt_s) || dt_s <= 0.0) dt_s = 0.01;
        dt_s = std::max(1e-4, std::min(dt_s, 5e-2));
    }

    // ---- run hand admittance (Strategy A, Method 1) ----
    dualarm_forcecon::HandAdmittanceControl::StepInput in;
    in.p_des_base = p_des_base;
    in.f_des_base = f_des_base;
    in.f_meas_base = f_meas_base;
    in.q_hand_current20 = q_cur20;
    in.dt_s = dt_s;

    auto out = adm_ptr->step(in);

    if (!out.controller_ok) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] controller step failed (controller_ok=0, TAR_F monitor kept)");
        return;
    }

    // ---- forcecon policy: non-target joints stay idle/hold ----
    // arms는 항상 현재값 유지 (forcecon은 hand만 제어)
    for (int i = 0; i < 6; ++i) {
        q_l_t_(i) = q_l_c_(i);
        q_r_t_(i) = q_r_c_(i);
    }

    // selected hand finger만 q target 업데이트 (q4 = mimic(q3))
    const int b = finger_id * 4;
    if (b + 3 < q_h_t.size()) {
        q_h_t(b + 0) = out.q_cmd_123[0];
        q_h_t(b + 1) = out.q_cmd_123[1];
        q_h_t(b + 2) = out.q_cmd_123[2];
        q_h_t(b + 3) = out.q_cmd_4_mimic;
    }

    // 참고:
    // TAR_F / target fingertip display는 이미 위에서 갱신 완료.
    // 여기서는 q target (실제 제어 입력)만 반영하면 됨.

    // ---- debug (decimated) ----
    static int dbg_decim = 0;
    if ((dbg_decim++ % 20) == 0) {
        const Eigen::Vector3d f_tar_row = f_hand_t_mat.row(finger_id).transpose();
        RCLCPP_INFO(logger,
            "[TargetHandForceCb] side=%s finger=%d dt=%.4f ik_ok=%d | p_des=(%.4f %.4f %.4f) p_cmd=(%.4f %.4f %.4f) | f_des=(%.3f %.3f %.3f) f_meas=(%.3f %.3f %.3f) f_tar_row=(%.3f %.3f %.3f)",
            is_left ? "L" : "R",
            finger_id,
            dt_s,
            static_cast<int>(out.ik_ok),
            p_des_base.x(), p_des_base.y(), p_des_base.z(),
            out.p_cmd_base.x(), out.p_cmd_base.y(), out.p_cmd_base.z(),
            f_des_base.x(), f_des_base.y(), f_des_base.z(),
            f_meas_base.x(), f_meas_base.y(), f_meas_base.z(),
            f_tar_row.x(), f_tar_row.y(), f_tar_row.z());
    }

    if (!out.ik_ok) {
        RCLCPP_WARN(logger,
            "[TargetHandForceCb] IK fallback used. side=%s finger=%d",
            is_left ? "L" : "R", finger_id);
    }

    // 참고:
    // 실제 /isaac_joint_command publish는 ControlLoop() 타이머가 수행함.
}