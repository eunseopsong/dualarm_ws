#include "DualArmForceControl.h"

#include <cstdio>
#include <cmath>
#include <limits>
#include <map>

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
            // v10: parseHandJointName가 joint_1까지 robust하게 잡음
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

    // current FK (arms)
    std::vector<double> jl(6), jr(6);
    for (int i=0;i<6;i++){ jl[i]=q_l_c_(i); jr[i]=q_r_c_(i); }
    current_pose_l_ = arm_fk_->getLeftFK(jl);
    current_pose_r_ = arm_fk_->getRightFK(jr);

    if (current_control_mode_ == "idle") {
        target_pose_l_ = current_pose_l_;
        target_pose_r_ = current_pose_r_;
    } else if (current_control_mode_ == "forward") {
        std::vector<double> jl_t(6), jr_t(6);
        for (int i=0;i<6;i++){ jl_t[i]=q_l_t_(i); jr_t[i]=q_r_t_(i); }
        target_pose_l_ = arm_fk_->getLeftFK(jl_t);
        target_pose_r_ = arm_fk_->getRightFK(jr_t);
    }
}

void DualArmForceControl::HandPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!msg) return;

    // ------------------------------------------------------------
    // 0) FK 객체 준비 확인
    // ------------------------------------------------------------
    if (!hand_fk_l_ || !hand_fk_r_) {
        // FK가 아직 생성되지 않았으면 계산 불가
        return;
    }

    // ------------------------------------------------------------
    // 1) 유틸: Eigen -> geometry_msgs::msg::Point 변환
    // ------------------------------------------------------------
    auto assign_point = [&](geometry_msgs::msg::Point& p, const Eigen::Vector3d& v) {
        p.x = v.x();
        p.y = v.y();
        p.z = v.z();
    };

    // ------------------------------------------------------------
    // 2) 유틸: vector<Eigen::Vector3d> 안전 인덱싱
    // ------------------------------------------------------------
    auto safe_get = [&](const std::vector<Eigen::Vector3d>& v, int idx) -> Eigen::Vector3d {
        if (idx < 0 || idx >= static_cast<int>(v.size())) return Eigen::Vector3d::Zero();
        return v[idx];
    };

    // ------------------------------------------------------------
    // 3) msg -> (현재/타겟) 손 관절 벡터 생성
    //    - 아래는 "이름 기반 매핑" 템플릿.
    //    - 네 코드에 이미 확정된 매핑/인덱스가 있으면 그걸로 교체해도 됨.
    // ------------------------------------------------------------
    // 예: 각 손 20DOF라고 가정 (프로젝트 설정에 맞게 수정 가능)
    constexpr int HAND_DOF = 20;
    Eigen::Matrix<double, HAND_DOF, 1> hl;   hl.setZero();
    Eigen::Matrix<double, HAND_DOF, 1> hr;   hr.setZero();
    Eigen::Matrix<double, HAND_DOF, 1> hl_t; hl_t.setZero();   // 타겟(있으면)
    Eigen::Matrix<double, HAND_DOF, 1> hr_t; hr_t.setZero();

    // ------------------------------------------------------------
    // 3-1) 파싱 함수(내부 람다)
    //  - msg->name / msg->position 를 읽어 hl/hr 채움
    //  - ⚠️ 여기의 조인트명-인덱스 매핑만 네 프로젝트에 맞게 채워라.
    // ------------------------------------------------------------
    auto parseHandJointsFromMsg = [&](const sensor_msgs::msg::JointState::SharedPtr& m,
                                      Eigen::Matrix<double, HAND_DOF, 1>& out_l,
                                      Eigen::Matrix<double, HAND_DOF, 1>& out_r)
    {
        const size_t N = std::min(m->name.size(), m->position.size());
        for (size_t i = 0; i < N; ++i) {
            const std::string& n = m->name[i];
            const double q = m->position[i];

            // -----------------------------
            // ✅ TODO: 아래는 예시 템플릿
            // 네 손 조인트 네이밍에 맞게 "정확히" 수정해야 함.
            //
            // out_l(k) / out_r(k)에서 k는 0..HAND_DOF-1
            // -----------------------------

            // ===== Left hand examples =====
            // if (n == "left_hand_joint_0") out_l(0) = q;
            // else if (n == "left_hand_joint_1") out_l(1) = q;
            // ...
            // ===== Right hand examples =====
            // else if (n == "right_hand_joint_0") out_r(0) = q;
            // else if (n == "right_hand_joint_1") out_r(1) = q;
            // ...

            // ----------------------------------------------------
            // 만약 이미 네 코드에 "확정된 매핑"이 있다면
            // 위 TODO 구간 전체를 네 기존 매핑 코드로 교체하면 끝.
            // ----------------------------------------------------
        }
    };

    // 현재 손 관절 파싱
    parseHandJointsFromMsg(msg, hl, hr);

    // ------------------------------------------------------------
    // 3-2) 타겟 관절(있을 때만)
    //  - 네 코드가 타겟을 따로 안 쓰면, 아래 블록은 그대로 둬도 됨(0 유지)
    //  - 타겟을 별도 토픽으로 받는 구조면, 여기서 msg가 아니라
    //    "저장해둔 타겟 벡터"를 hl_t/hr_t에 넣으면 됨.
    // ------------------------------------------------------------
    // 예시:
    // hl_t = hand_q_l_target_;
    // hr_t = hand_q_r_target_;

    // ------------------------------------------------------------
    // 4) Fingertip FK 계산: vector<Eigen::Vector3d> 반환 가정
    //    order: [thumb, index, middle, ring, baby]
    // ------------------------------------------------------------
    const std::vector<Eigen::Vector3d> tl = hand_fk_l_->computeFingertips(hl);
    const std::vector<Eigen::Vector3d> tr = hand_fk_r_->computeFingertips(hr);

    // ------------------------------------------------------------
    // 5) 현재 fingertip -> Point 멤버에 저장
    // ------------------------------------------------------------
    assign_point(f_l_thumb_,  safe_get(tl, 0));
    assign_point(f_l_index_,  safe_get(tl, 1));
    assign_point(f_l_middle_, safe_get(tl, 2));
    assign_point(f_l_ring_,   safe_get(tl, 3));
    assign_point(f_l_baby_,   safe_get(tl, 4));

    assign_point(f_r_thumb_,  safe_get(tr, 0));
    assign_point(f_r_index_,  safe_get(tr, 1));
    assign_point(f_r_middle_, safe_get(tr, 2));
    assign_point(f_r_ring_,   safe_get(tr, 3));
    assign_point(f_r_baby_,   safe_get(tr, 4));

    // ------------------------------------------------------------
    // 6) 타겟 fingertip도 계산해서 저장(원하면 유지)
    // ------------------------------------------------------------
    {
        const std::vector<Eigen::Vector3d> tl_t = hand_fk_l_->computeFingertips(hl_t);
        const std::vector<Eigen::Vector3d> tr_t = hand_fk_r_->computeFingertips(hr_t);

        assign_point(t_f_l_thumb_,  safe_get(tl_t, 0));
        assign_point(t_f_l_index_,  safe_get(tl_t, 1));
        assign_point(t_f_l_middle_, safe_get(tl_t, 2));
        assign_point(t_f_l_ring_,   safe_get(tl_t, 3));
        assign_point(t_f_l_baby_,   safe_get(tl_t, 4));

        assign_point(t_f_r_thumb_,  safe_get(tr_t, 0));
        assign_point(t_f_r_index_,  safe_get(tr_t, 1));
        assign_point(t_f_r_middle_, safe_get(tr_t, 2));
        assign_point(t_f_r_ring_,   safe_get(tr_t, 3));
        assign_point(t_f_r_baby_,   safe_get(tr_t, 4));
    }

    // ------------------------------------------------------------
    // 7) (선택) 디버그/프린트/후속 로직은 네 기존 코드 그대로
    // ------------------------------------------------------------
}

// --------------------
// TargetPositionCallback (inverse)
// --------------------
void DualArmForceControl::TargetPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (current_control_mode_ != "inverse") return;
    if (msg->data.size() < 12) return;
    if (!arm_ik_l_ || !arm_ik_r_) return;

    std::array<double,3> l_xyz{msg->data[0], msg->data[1], msg->data[2]};
    std::array<double,3> l_eul{msg->data[3], msg->data[4], msg->data[5]};
    std::array<double,3> r_xyz{msg->data[6], msg->data[7], msg->data[8]};
    std::array<double,3> r_eul{msg->data[9], msg->data[10], msg->data[11]};

    std::vector<double> ql(6), qr(6);
    for (int i=0;i<6;i++){ ql[i]=q_l_c_(i); qr[i]=q_r_c_(i); }

    std::vector<double> rl, rr;
    if (arm_ik_l_->solveIK(ql, l_xyz, l_eul, ik_targets_frame_, ik_euler_conv_, ik_angle_unit_, rl) && rl.size()>=6) {
        for (int i=0;i<6;i++) q_l_t_(i)=rl[i];
    }
    if (arm_ik_r_->solveIK(qr, r_xyz, r_eul, ik_targets_frame_, ik_euler_conv_, ik_angle_unit_, rr) && rr.size()>=6) {
        for (int i=0;i<6;i++) q_r_t_(i)=rr[i];
    }

    target_pose_l_.position.x = l_xyz[0];
    target_pose_l_.position.y = l_xyz[1];
    target_pose_l_.position.z = l_xyz[2];

    target_pose_r_.position.x = r_xyz[0];
    target_pose_r_.position.y = r_xyz[1];
    target_pose_r_.position.z = r_xyz[2];

    auto eulToQuat = [&](double ex, double ey, double ez){
        double a0 = ex, a1 = ey, a2 = ez;
        if (ik_angle_unit_ == "deg") { a0*=M_PI/180.0; a1*=M_PI/180.0; a2*=M_PI/180.0; }
        Eigen::AngleAxisd Rx(a0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(a1, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(a2, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = Rx*Ry*Rz;
        geometry_msgs::msg::Quaternion qq;
        qq.x=q.x(); qq.y=q.y(); qq.z=q.z(); qq.w=q.w();
        return qq;
    };
    target_pose_l_.orientation = eulToQuat(l_eul[0], l_eul[1], l_eul[2]);
    target_pose_r_.orientation = eulToQuat(r_eul[0], r_eul[1], r_eul[2]);
}

// --------------------
// TargetJointCallback (forward)
// --------------------
void DualArmForceControl::TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_control_mode_ != "forward") return;

    const size_t n = msg->data.size();

    if (n >= 12) {
        for (int i = 0; i < 6; ++i) {
            q_l_t_(i) = msg->data[i + 0];
            q_r_t_(i) = msg->data[i + 6];
        }
    } else {
        return;
    }

    if (n >= 52) {
        for (int i = 0; i < 20; ++i) q_l_h_t_(i) = msg->data[12 + i];
        for (int i = 0; i < 20; ++i) q_r_h_t_(i) = msg->data[32 + i];
    }
}

// --------------------
// ControlModeCallback
// --------------------
void DualArmForceControl::ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                              std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    current_control_mode_ =
        (current_control_mode_=="idle") ? "forward" :
        (current_control_mode_=="forward") ? "inverse" : "idle";

    if (current_control_mode_=="idle") idle_synced_=false;

    res->success=true;
    res->message="Mode: "+current_control_mode_;
}

// --------------------
// ContactForceCallback (arm)
// --------------------
void DualArmForceControl::ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 6) return;
    f_l_c_ << msg->data[0], msg->data[1], msg->data[2];
    f_r_c_ << msg->data[3], msg->data[4], msg->data[5];

    f_l_t_ = f_l_c_;
    f_r_t_ = f_r_c_;
}

// ============================================================================
// PrintDualArmStates (hand points are in hand-base frames now)
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
    printf("%s   Dual Arm & Hand Monitor v10 | Mode: [%s%s%s] | %sCUR_POS%s %sTAR_POS%s %sCUR_F%s %sTAR_F%s%s\n",
           C_TITLE,
           C_MODE, current_control_mode_.c_str(), C_RESET,
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
    printFingerBlock("THMB", f_l_thumb_,   t_f_l_thumb_,   f_l_hand_c_.row(0), f_l_hand_t_.row(0));
    printFingerBlock("INDX", f_l_index_,   t_f_l_index_,   f_l_hand_c_.row(1), f_l_hand_t_.row(1));
    printFingerBlock("MIDL", f_l_middle_,  t_f_l_middle_,  f_l_hand_c_.row(2), f_l_hand_t_.row(2));
    printFingerBlock("RING", f_l_ring_,    t_f_l_ring_,    f_l_hand_c_.row(3), f_l_hand_t_.row(3));
    printFingerBlock("BABY", f_l_baby_,    t_f_l_baby_,    f_l_hand_c_.row(4), f_l_hand_t_.row(4));

    printf("%s------------------------------------------------------------------------------------------------------------%s\n", C_DIM, C_RESET);

    printf("%s[R HAND] (positions are expressed in RIGHT_HAND_BASE frame)%s\n\n", C_TITLE, C_RESET);
    printFingerBlock("THMB", f_r_thumb_,   t_f_r_thumb_,   f_r_hand_c_.row(0), f_r_hand_t_.row(0));
    printFingerBlock("INDX", f_r_index_,   t_f_r_index_,   f_r_hand_c_.row(1), f_r_hand_t_.row(1));
    printFingerBlock("MIDL", f_r_middle_,  t_f_r_middle_,  f_r_hand_c_.row(2), f_r_hand_t_.row(2));
    printFingerBlock("RING", f_r_ring_,    t_f_r_ring_,    f_r_hand_c_.row(3), f_r_hand_t_.row(3));
    printFingerBlock("BABY", f_r_baby_,    t_f_r_baby_,    f_r_hand_c_.row(4), f_r_hand_t_.row(4));

    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);
}