#include "DualArmForceControl.h"

#include <cstdio>
#include <cmath>
#include <limits>
#include <map>
#include <array>
#include <algorithm>
#include <cctype>


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
// ArmPositionCallback (v16 minor patch: output stabilization / deadband)
// --------------------
void DualArmForceControl::ArmPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    (void)msg;
    if (!is_initialized_) return;
    if (!arm_fk_) return;

    // ---- tuning (display stability) ----
    // position deadband: 1 um
    constexpr double kPosDeadbandM = 1e-6;
    // quaternion "same" threshold (component-wise max diff after hemisphere alignment)
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
            q.coeffs() *= -1.0; // keep hemisphere consistent
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

        // position (axis-wise sticky deadband)
        if (std::fabs(cand.position.x - dst.position.x) >= kPosDeadbandM) dst.position.x = cand.position.x;
        if (std::fabs(cand.position.y - dst.position.y) >= kPosDeadbandM) dst.position.y = cand.position.y;
        if (std::fabs(cand.position.z - dst.position.z) >= kPosDeadbandM) dst.position.z = cand.position.z;

        // orientation (update whole quaternion only if change is meaningful)
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

    // static caches (callback-local; no header change required)
    static bool s_cur_l_init = false;
    static bool s_cur_r_init = false;
    static bool s_tar_l_init = false;
    static bool s_tar_r_init = false;

    // ---------------- current pose (from q_c_) ----------------
    std::vector<double> jl(6), jr(6);
    for (int i = 0; i < 6; ++i) {
        jl[i] = q_l_c_(i);
        jr[i] = q_r_c_(i);
    }

    geometry_msgs::msg::Pose cur_l_fk = arm_fk_->getLeftFK(jl);
    geometry_msgs::msg::Pose cur_r_fk = arm_fk_->getRightFK(jr);

    apply_pose_deadband(current_pose_l_, cur_l_fk, s_cur_l_init);
    apply_pose_deadband(current_pose_r_, cur_r_fk, s_cur_r_init);

    // ---------------- target pose (mode-dependent) ----------------
    if (current_control_mode_ == "idle") {
        // idle에서는 target을 current에 hard sync (표시 흔들림 최소화)
        target_pose_l_ = current_pose_l_;
        target_pose_r_ = current_pose_r_;

        s_tar_l_init = true;
        s_tar_r_init = true;
    }
    else if (current_control_mode_ == "forward") {
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
    // inverse 모드에서는 target_pose_* 는 TargetArmPositionCallback / DeltaArmPositionCallback 이 관리
}

// --------------------
// HandPositionCallback (v16 minor patch: fingertip output stabilization / deadband)
// --------------------
void DualArmForceControl::HandPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    (void)msg;
    if (!is_initialized_) return;
    if (!hand_fk_l_ || !hand_fk_r_) return;

    // ---- tuning (display stability) ----
    // fingertip position deadband: 1 um
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

    // v12: 20DoF(표현) -> 15DoF(독립) 압축
    auto compress20to15 = [&](const Eigen::VectorXd& qh) -> std::vector<double> {
        std::vector<double> h15(15, 0.0);

        if (qh.size() >= 20) {
            for (int f = 0; f < 5; ++f) {
                const int b20 = f * 4;
                const int b15 = f * 3;
                h15[b15 + 0] = qh(b20 + 0);
                h15[b15 + 1] = qh(b20 + 1);
                h15[b15 + 2] = qh(b20 + 2);
                // qh(b20+3)는 mimic으로 간주하여 무시
            }
            return h15;
        }

        // 혹시 이미 15DoF 저장 구조로 바뀐 경우도 호환
        const int n = std::min<int>(qh.size(), 15);
        for (int i = 0; i < n; ++i) h15[i] = qh(i);
        return h15;
    };

    // static caches (callback-local; no header change required)
    static bool s_l_cur_init[5] = {false, false, false, false, false};
    static bool s_r_cur_init[5] = {false, false, false, false, false};
    static bool s_l_tar_init[5] = {false, false, false, false, false};
    static bool s_r_tar_init[5] = {false, false, false, false, false};

    // 현재/타겟 손 관절을 15DoF independent로 변환
    const std::vector<double> hl15   = compress20to15(q_l_h_c_);
    const std::vector<double> hr15   = compress20to15(q_r_h_c_);
    const std::vector<double> hl_t15 = compress20to15(q_l_h_t_);
    const std::vector<double> hr_t15 = compress20to15(q_r_h_t_);

    // Fingertip FK (order: thumb,index,middle,ring,baby)
    const std::vector<Eigen::Vector3d> tl   = hand_fk_l_->computeFingertips(hl15);
    const std::vector<Eigen::Vector3d> tr   = hand_fk_r_->computeFingertips(hr15);
    const std::vector<Eigen::Vector3d> tl_t = hand_fk_l_->computeFingertips(hl_t15);
    const std::vector<Eigen::Vector3d> tr_t = hand_fk_r_->computeFingertips(hr_t15);

    // ---------------- current fingertip (thumb,index,middle,ring,baby) ----------------
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

    // ---------------- target fingertip (monitor TAR line) ----------------
    if (current_control_mode_ == "idle") {
        // idle에서는 target = current (표시 안정화)
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
// ControlModeCallback (v18: forcecon mode 추가, target-force zero issue fix)
// mode cycle:
//   idle -> forward -> inverse -> forcecon -> idle -> ...
//
// forcecon 진입 시:
// - arm/hand target을 현재값으로 동기화 (비제어 joint idle 유지)
// - hand target force는 여기서 setZero() 하지 않음 (TargetHandForceCallback 값 유지)
// --------------------
void DualArmForceControl::ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;

    // mode cycle
    if (current_control_mode_ == "idle") {
        current_control_mode_ = "forward";
    } else if (current_control_mode_ == "forward") {
        current_control_mode_ = "inverse";
    } else if (current_control_mode_ == "inverse") {
        current_control_mode_ = "forcecon";
    } else {
        current_control_mode_ = "idle";
    }

    // idle 진입 시 다음 ControlLoop에서 1회 sync 하도록
    if (current_control_mode_ == "idle") {
        idle_synced_ = false;
    }

    // inverse / forcecon 진입 시 즉시 현재값으로 target sync
    // - hand만 테스트할 때 arm이 갑자기 움직이지 않게
    // - forcecon에서 비대상 joint는 idle 유지
    if (current_control_mode_ == "inverse" || current_control_mode_ == "forcecon") {
        for (int i = 0; i < 6; ++i) {
            q_l_t_(i) = q_l_c_(i);
            q_r_t_(i) = q_r_c_(i);
        }
        for (int i = 0; i < 20; ++i) {
            q_l_h_t_(i) = q_l_h_c_(i);
            q_r_h_t_(i) = q_r_h_c_(i);
        }

        // monitor pose/fingertip target도 현재 상태로 시작 (표시 안정화)
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
    }

    // arm force monitor target은 현재 미사용이므로 항상 0 유지
    // (센서 콜백에서 target force를 건드리지 않도록 정책 분리)
    f_l_t_.setZero();
    f_r_t_.setZero();

    // ✅ hand target force는 forcecon에서 지우지 않음
    //    (TargetHandForceCallback에서 설정한 값을 print에서 볼 수 있게 유지)
    //    forcecon 이외의 모드에서만 clear
    if (current_control_mode_ == "idle" ||
        current_control_mode_ == "forward" ||
        current_control_mode_ == "inverse") {
        f_l_hand_t_.setZero();
        f_r_hand_t_.setZero();
    }

    res->success = true;
    res->message = "Mode: " + current_control_mode_;
}

// ============================================================================
// HandContactForceCallback (v17/v18 patched + refactor)
// ----------------------------------------------------------------------------
// 역할:
// - /isaac_contact_states (Float32MultiArray, 10 scalars) 를 읽어
//   각 손가락의 "현재 측정 힘" f_*_hand_c_ 를 갱신
// - target force (f_*_hand_t_) 는 여기서 절대 setZero 하지 않음
//   -> TargetHandForceCallback 에서 설정한 TAR_F 표시값 유지
//
// /isaac_contact_states empirical order per hand (observed):
//   [BABY, RING, MIDL, INDX, THMB]
//
// Internal hand force row order:
//   row0=THMB, row1=INDX, row2=MIDL, row3=RING, row4=BABY
//
// Scalar-contact assumption (temporary):
//   Isaac sensor scalar = force along fingertip sensor local +X axis
//
// Frame conversion:
//   sensor(+X scalar) -> tip frame -> hand base frame -> empirical base correction
// ============================================================================
void DualArmForceControl::HandContactForceCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!msg) return;

    // ------------------------------------------------------------------------
    // Reset CURRENT force buffers only
    // IMPORTANT:
    // - Do NOT touch target force buffers here (f_*_t_, f_*_hand_t_)
    // - Sensor callback updates ONLY current measurement buffers (*_c_)
    // ------------------------------------------------------------------------
    auto reset_current_force_buffers_only = [&]() {
        // arm force is not measured yet -> keep current monitor as zero
        f_l_c_.setZero();
        f_r_c_.setZero();

        // hand current measured forces
        f_l_hand_c_.setZero();
        f_r_hand_c_.setZero();
    };
    reset_current_force_buffers_only();

    // Need 10 scalar values (5 left + 5 right)
    if (msg->data.size() < 10) {
        return;
    }

    if (!hand_fk_l_ || !hand_fk_r_) {
        return;
    }

    // ------------------------------------------------------------------------
    // Helper: 20DoF (internal hand joint vector) -> 15DoF independent vector
    // order(20): [thumb1..4, index1..4, middle1..4, ring1..4, baby1..4]
    // order(15): [thumb1..3, index1..3, middle1..3, ring1..3, baby1..3]
    // ------------------------------------------------------------------------
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

    // Current hand joints -> fingertip rotation in hand-base frame
    const std::vector<double> hl15 = compress20to15(q_l_h_c_);
    const std::vector<double> hr15 = compress20to15(q_r_h_c_);

    // Returned order: [thumb, index, middle, ring, baby]
    const std::vector<Eigen::Matrix3d> Rl_base_tip = hand_fk_l_->computeTipRotationsBase(hl15);
    const std::vector<Eigen::Matrix3d> Rr_base_tip = hand_fk_r_->computeTipRotationsBase(hr15);

    auto safeR = [](const std::vector<Eigen::Matrix3d>& Rs, int idx) -> Eigen::Matrix3d {
        if (idx < 0 || idx >= static_cast<int>(Rs.size())) return Eigen::Matrix3d::Identity();
        return Rs[idx];
    };

    // ------------------------------------------------------------------------
    // [Calibration #1] sensor frame -> tip frame
    // Temporary assumption: sensor +X == tip +X
    // ------------------------------------------------------------------------
    const Eigen::Matrix3d R_tip_sensor = Eigen::Matrix3d::Identity();

    // ------------------------------------------------------------------------
    // [Calibration #2] empirical hand-base correction
    // Maps raw base vector -> desired display convention
    //
    // [x', y', z']^T = [ -z, x, -y ]^T
    // ------------------------------------------------------------------------
    Eigen::Matrix3d R_base_corr;
    R_base_corr <<  0.0,  0.0, -1.0,
                    1.0,  0.0,  0.0,
                    0.0, -1.0,  0.0;

    // ------------------------------------------------------------------------
    // Message-index -> internal row mapping
    // msg order per hand: [BABY, RING, MIDL, INDX, THMB]
    // internal row order : [THMB, INDX, MIDL, RING, BABY]
    // => BABY->4, RING->3, MIDL->2, INDX->1, THMB->0
    // ------------------------------------------------------------------------
    const std::array<int,5> msg_to_row = {4, 3, 2, 1, 0};

    // ------------------------------------------------------------------------
    // Convert one hand (5 scalars) into hand-base-frame force vectors
    // ------------------------------------------------------------------------
    auto assign_one_hand = [&](Eigen::Matrix<double,5,3>& F_hand_cur,
                               const std::vector<Eigen::Matrix3d>& R_base_tip_all,
                               int msg_offset)
    {
        for (int k = 0; k < 5; ++k) {
            const int row = msg_to_row[k];   // internal row index
            const double s = static_cast<double>(msg->data[msg_offset + k]);

            // Scalar contact -> sensor local force vector (temporary assumption)
            // sensor local +X direction only
            const Eigen::Vector3d f_sensor(s, 0.0, 0.0);

            // sensor -> tip
            const Eigen::Vector3d f_tip = R_tip_sensor * f_sensor;

            // tip -> hand base
            const Eigen::Matrix3d R_base_tip = safeR(R_base_tip_all, row);
            const Eigen::Vector3d f_base_raw = R_base_tip * f_tip;

            // empirical base-frame correction (display / convention alignment)
            Eigen::Vector3d f_base = R_base_corr * f_base_raw;

            // tiny numerical cleanup
            for (int i = 0; i < 3; ++i) {
                if (std::fabs(f_base(i)) < 1e-9) f_base(i) = 0.0;
            }

            F_hand_cur.row(row) = f_base.transpose();
        }
    };

    // Left hand: msg[0..4]
    assign_one_hand(f_l_hand_c_, Rl_base_tip, 0);

    // Right hand: msg[5..9]
    assign_one_hand(f_r_hand_c_, Rr_base_tip, 5);

    // NOTE:
    // - f_l_hand_t_ / f_r_hand_t_ are intentionally NOT touched here.
    // - They should be updated only by TargetHandForceCallback (desired force command),
    //   or explicitly cleared in ControlModeCallback when leaving forcecon.
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


// ============================================================================
// PrintDualArmStates (v17+ target force visible)
// - Finger print order: BABY -> RING -> MIDL -> INDX -> THMB
// - Hand force row mapping (canonical): THMB=0, INDX=1, MIDL=2, RING=3, BABY=4
// - ARM/HAND 모두 저장된 CUR_F / TAR_F를 그대로 출력
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
    printf("%s   Dual Arm & Hand Monitor v17 | Mode: [%s%s%s] | %sCUR_POS%s %sTAR_POS%s %sCUR_F%s %sTAR_F%s%s\n",
           C_TITLE,
           C_MODE, current_control_mode_.c_str(), C_RESET,
           C_CUR_POS, C_RESET,
           C_TAR_POS, C_RESET,
           C_CUR_F,   C_RESET,
           C_TAR_F,   C_RESET,
           C_RESET);
    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);

    // ------------------------------------------------------------------------
    // ARM (stored CUR_F / TAR_F)
    // ------------------------------------------------------------------------
    printf("%s[L ARM]%s\n", C_TITLE, C_RESET);
    fmtArmLine("CUR", current_pose_l_, f_l_c_, C_CUR_POS, C_CUR_F);
    fmtArmLine("TAR", target_pose_l_,  f_l_t_, C_TAR_POS, C_TAR_F);

    printf("%s------------------------------------------------------------------------------------------------------------%s\n", C_DIM, C_RESET);

    printf("%s[R ARM]%s\n", C_TITLE, C_RESET);
    fmtArmLine("CUR", current_pose_r_, f_r_c_, C_CUR_POS, C_CUR_F);
    fmtArmLine("TAR", target_pose_r_,  f_r_t_, C_TAR_POS, C_TAR_F);

    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);

    // ------------------------------------------------------------------------
    // LEFT HAND (display order: BABY -> RING -> MIDL -> INDX -> THMB)
    // Canonical row map: THMB=0, INDX=1, MIDL=2, RING=3, BABY=4
    // ------------------------------------------------------------------------
    printf("%s[L HAND] (positions are expressed in LEFT_HAND_BASE frame)%s\n\n", C_TITLE, C_RESET);

    printFingerBlock("BABY", f_l_baby_,   t_f_l_baby_,   f_l_hand_c_.row(4), f_l_hand_t_.row(4));
    printFingerBlock("RING", f_l_ring_,   t_f_l_ring_,   f_l_hand_c_.row(3), f_l_hand_t_.row(3));
    printFingerBlock("MIDL", f_l_middle_, t_f_l_middle_, f_l_hand_c_.row(2), f_l_hand_t_.row(2));
    printFingerBlock("INDX", f_l_index_,  t_f_l_index_,  f_l_hand_c_.row(1), f_l_hand_t_.row(1));
    printFingerBlock("THMB", f_l_thumb_,  t_f_l_thumb_,  f_l_hand_c_.row(0), f_l_hand_t_.row(0));

    printf("%s------------------------------------------------------------------------------------------------------------%s\n", C_DIM, C_RESET);

    // ------------------------------------------------------------------------
    // RIGHT HAND (display order: BABY -> RING -> MIDL -> INDX -> THMB)
    // Canonical row map: THMB=0, INDX=1, MIDL=2, RING=3, BABY=4
    // ------------------------------------------------------------------------
    printf("%s[R HAND] (positions are expressed in RIGHT_HAND_BASE frame)%s\n\n", C_TITLE, C_RESET);

    printFingerBlock("BABY", f_r_baby_,   t_f_r_baby_,   f_r_hand_c_.row(4), f_r_hand_t_.row(4));
    printFingerBlock("RING", f_r_ring_,   t_f_r_ring_,   f_r_hand_c_.row(3), f_r_hand_t_.row(3));
    printFingerBlock("MIDL", f_r_middle_, t_f_r_middle_, f_r_hand_c_.row(2), f_r_hand_t_.row(2));
    printFingerBlock("INDX", f_r_index_,  t_f_r_index_,  f_r_hand_c_.row(1), f_r_hand_t_.row(1));
    printFingerBlock("THMB", f_r_thumb_,  t_f_r_thumb_,  f_r_hand_c_.row(0), f_r_hand_t_.row(0));

    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);
}