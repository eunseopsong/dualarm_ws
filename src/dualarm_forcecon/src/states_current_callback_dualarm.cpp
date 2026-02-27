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
// HandContactForceCallback (v18 patched after HandFK axis remap)
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
//   sensor(+X scalar) -> tip frame -> hand base frame (already axis-remapped in HandFK)
//   -> empirical base correction (updated for v18 HandFK axis patch)
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
    // NOTE: v18에서 hand_fk_->computeTipRotationsBase()는 이미 "user/display HAND_BASE axis" 기준으로 반환됨
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
    // [Calibration #2] empirical hand-base correction (UPDATED for v18 HandFK axis patch)
    //
    // 이전 v17/v18-pre-FK-axis-patch 에서는:
    //   [x', y', z'] = [ -z, x, -y ]
    //
    // 하지만 v18에서 HandForwardKinematics::computeTipRotationsBase()가
    // base frame 축을 user/display 기준으로 remap 하도록 수정되었으므로,
    // 여기서는 과거 보정을 그대로 쓰면 축이 다시 꼬인다.
    //
    // 현재 권장 보정 (재보정):
    //   [x', y', z'] = [ -x, +y, -z ]
    // 즉, diag(-1, +1, -1)
    //
    // (참고) y축 부호는 접촉 방향 실험에서 필요하면 +1/-1로 한 줄 튜닝 가능
    // ------------------------------------------------------------------------
    Eigen::Matrix3d R_base_corr;
    R_base_corr << -1.0,  0.0,  0.0,
                    0.0,  1.0,  0.0,
                    0.0,  0.0, -1.0;

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

            // tip -> hand base (already in v18 user/display HAND_BASE axis convention)
            const Eigen::Matrix3d R_base_tip = safeR(R_base_tip_all, row);
            const Eigen::Vector3d f_base_raw = R_base_tip * f_tip;

            // empirical base-frame correction (updated after HandFK axis patch)
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
    printf("%s   Dual Arm & Hand Monitor v18 | Mode: [%s%s%s] | %sCUR_POS%s %sTAR_POS%s %sCUR_F%s %sTAR_F%s%s\n",
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