#include "DualArmForceControl.h"

#include <cstdio>
#include <cmath>

// --------------------
// JointsCallback
// --------------------
void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (joint_names_.empty()) { joint_names_ = msg->name; is_initialized_ = true; }

    for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& n = msg->name[i];
        const double p = msg->position[i];

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
            int f_idx = -1;
            if      (n.find("thumb")  != std::string::npos) f_idx=0;
            else if (n.find("index")  != std::string::npos) f_idx=4;
            else if (n.find("middle") != std::string::npos) f_idx=8;
            else if (n.find("ring")   != std::string::npos) f_idx=12;
            else if (n.find("baby")   != std::string::npos) f_idx=16;

            if (f_idx!=-1) {
                int j_idx = -1;
                if      (n.find("1")!=std::string::npos) j_idx=0;
                else if (n.find("2")!=std::string::npos) j_idx=1;
                else if (n.find("3")!=std::string::npos) j_idx=2;
                else if (n.find("4")!=std::string::npos) j_idx=3;

                if (j_idx!=-1) {
                    if (n.find("left")!=std::string::npos)  q_l_h_c_(f_idx+j_idx)=p;
                    else                                     q_r_h_c_(f_idx+j_idx)=p;
                }
            }
        }
    }
}

// --------------------
// PositionCallback
// --------------------
void DualArmForceControl::PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    (void)msg;
    if (!is_initialized_) return;
    if (!arm_fk_ || !hand_fk_l_ || !hand_fk_r_) return;

    // current FK
    std::vector<double> jl(6), jr(6);
    for (int i=0;i<6;i++){ jl[i]=q_l_c_(i); jr[i]=q_r_c_(i); }
    current_pose_l_ = arm_fk_->getLeftFK(jl);
    current_pose_r_ = arm_fk_->getRightFK(jr);

    // target pose handling:
    // - idle: target=current
    // - forward: target는 target joints로 FK 해서 "가고 있는 위치" 표시
    // - inverse: TargetPositionCallback에서 저장된 target_pose 유지
    if (current_control_mode_ == "idle") {
        target_pose_l_ = current_pose_l_;
        target_pose_r_ = current_pose_r_;
    } else if (current_control_mode_ == "forward") {
        std::vector<double> jl_t(6), jr_t(6);
        for (int i=0;i<6;i++){ jl_t[i]=q_l_t_(i); jr_t[i]=q_r_t_(i); }
        target_pose_l_ = arm_fk_->getLeftFK(jl_t);
        target_pose_r_ = arm_fk_->getRightFK(jr_t);
    }

    // fingertips (curr)
    std::vector<double> hl(20), hr(20);
    for (int i=0;i<20;i++){ hl[i]=q_l_h_c_(i); hr[i]=q_r_h_c_(i); }
    auto tl = hand_fk_l_->computeFingertips(hl);
    auto tr = hand_fk_r_->computeFingertips(hr);

    if(!tl.empty()){
        f_l_thumb_  = HandForwardKinematics::combinePosePoint(current_pose_l_, tl["link4_thumb"]);
        f_l_index_  = HandForwardKinematics::combinePosePoint(current_pose_l_, tl["link4_index"]);
        f_l_middle_ = HandForwardKinematics::combinePosePoint(current_pose_l_, tl["link4_middle"]);
        f_l_ring_   = HandForwardKinematics::combinePosePoint(current_pose_l_, tl["link4_ring"]);
        f_l_baby_   = HandForwardKinematics::combinePosePoint(current_pose_l_, tl["link4_baby"]);
    }
    if(!tr.empty()){
        f_r_thumb_  = HandForwardKinematics::combinePosePoint(current_pose_r_, tr["link4_thumb"]);
        f_r_index_  = HandForwardKinematics::combinePosePoint(current_pose_r_, tr["link4_index"]);
        f_r_middle_ = HandForwardKinematics::combinePosePoint(current_pose_r_, tr["link4_middle"]);
        f_r_ring_   = HandForwardKinematics::combinePosePoint(current_pose_r_, tr["link4_ring"]);
        f_r_baby_   = HandForwardKinematics::combinePosePoint(current_pose_r_, tr["link4_baby"]);
    }

    // hand target = current (핸드 고정 정책)
    t_f_l_thumb_  = f_l_thumb_;  t_f_l_index_  = f_l_index_;  t_f_l_middle_ = f_l_middle_; t_f_l_ring_ = f_l_ring_; t_f_l_baby_ = f_l_baby_;
    t_f_r_thumb_  = f_r_thumb_;  t_f_r_index_  = f_r_index_;  t_f_r_middle_ = f_r_middle_; t_f_r_ring_ = f_r_ring_; t_f_r_baby_ = f_r_baby_;

    // hand force도 아직은 0 유지 (나중에 토픽 연결되면 callback에서 채우면 됨)
    f_l_hand_t_ = f_l_hand_c_;
    f_r_hand_t_ = f_r_hand_c_;
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

    // Print용: inverse에서는 "입력 목표"를 그대로 저장해서 보여줌
    target_pose_l_.position.x = l_xyz[0];
    target_pose_l_.position.y = l_xyz[1];
    target_pose_l_.position.z = l_xyz[2];

    target_pose_r_.position.x = r_xyz[0];
    target_pose_r_.position.y = r_xyz[1];
    target_pose_r_.position.z = r_xyz[2];

    // orientation은 굳이 quaternion으로 저장 안 해도 되지만, Print가 Euler 출력하므로 placeholder로 생성
    // (Euler 출력은 IK 입력 euler를 그대로 쓰게 만들 수도 있는데, 여기서는 단순화를 위해 quaternion 생성)
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

    // -------------------------
    // 1) Arms (always if >=12)
    // -------------------------
    if (n >= 12) {
        for (int i = 0; i < 6; ++i) {
            q_l_t_(i) = msg->data[i + 0];
            q_r_t_(i) = msg->data[i + 6];
        }
    } else {
        return; // 12 미만이면 무시
    }

    // -------------------------
    // 2) Hands (only if >=52)
    //   [12~31]  : Left hand 20
    //   [32~51]  : Right hand 20
    // -------------------------
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

    // 현재는 target force를 별도 입력 안 받으므로 targ = curr
    f_l_t_ = f_l_c_;
    f_r_t_ = f_r_c_;
}

// ============================================================================
// PrintDualArmStates (최하단)
// ============================================================================
void DualArmForceControl::PrintDualArmStates() {
    if (!is_initialized_) return;

    // ===== ANSI Colors =====
    // Position 색깔은 유지 (256-color 파스텔)
    // Curr Force: Red
    // Targ Force: Blue
    constexpr const char* C_RESET   = "\033[0m";
    constexpr const char* C_TITLE   = "\033[1;38;5;252m";
    constexpr const char* C_DIM     = "\033[38;5;245m";

    constexpr const char* C_CUR_POS = "\033[1;38;5;81m";   // teal-ish (keep)
    constexpr const char* C_TAR_POS = "\033[1;38;5;220m";  // gold-ish (keep)

    constexpr const char* C_CUR_F   = "\033[1;31m";        // RED
    constexpr const char* C_TAR_F   = "\033[1;34m";        // BLUE
    constexpr const char* C_MODE    = "\033[1;92m";        // neon green (mode)

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
        printf("\n"); // finger block gap
    };

    // clear screen
    printf("\033[2J\033[H");

    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);
    printf("%s   Dual Arm & Hand Monitor v6 | Mode: [%s%s%s] | %sCUR_POS%s %sTAR_POS%s %sCUR_F%s %sTAR_F%s%s\n",
           C_TITLE,
           C_MODE, current_control_mode_.c_str(), C_RESET,
           C_CUR_POS, C_RESET,
           C_TAR_POS, C_RESET,
           C_CUR_F,   C_RESET,
           C_TAR_F,   C_RESET,
           C_RESET);
    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);

    // ---------------- ARM ----------------
    printf("%s[L ARM]%s\n", C_TITLE, C_RESET);
    fmtArmLine("CUR", current_pose_l_, f_l_c_, C_CUR_POS, C_CUR_F);
    fmtArmLine("TAR", target_pose_l_,  f_l_t_, C_TAR_POS, C_TAR_F);

    printf("%s------------------------------------------------------------------------------------------------------------%s\n", C_DIM, C_RESET);

    printf("%s[R ARM]%s\n", C_TITLE, C_RESET);
    fmtArmLine("CUR", current_pose_r_, f_r_c_, C_CUR_POS, C_CUR_F);
    fmtArmLine("TAR", target_pose_r_,  f_r_t_, C_TAR_POS, C_TAR_F);

    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);

    // ---------------- HAND ----------------
    printf("%s[L HAND]%s\n\n", C_TITLE, C_RESET);
    printFingerBlock("THMB", f_l_thumb_,   t_f_l_thumb_,   f_l_hand_c_.row(0), f_l_hand_t_.row(0));
    printFingerBlock("INDX", f_l_index_,   t_f_l_index_,   f_l_hand_c_.row(1), f_l_hand_t_.row(1));
    printFingerBlock("MIDL", f_l_middle_,  t_f_l_middle_,  f_l_hand_c_.row(2), f_l_hand_t_.row(2));
    printFingerBlock("RING", f_l_ring_,    t_f_l_ring_,    f_l_hand_c_.row(3), f_l_hand_t_.row(3));
    printFingerBlock("BABY", f_l_baby_,    t_f_l_baby_,    f_l_hand_c_.row(4), f_l_hand_t_.row(4));

    printf("%s------------------------------------------------------------------------------------------------------------%s\n", C_DIM, C_RESET);

    printf("%s[R HAND]%s\n\n", C_TITLE, C_RESET);
    printFingerBlock("THMB", f_r_thumb_,   t_f_r_thumb_,   f_r_hand_c_.row(0), f_r_hand_t_.row(0));
    printFingerBlock("INDX", f_r_index_,   t_f_r_index_,   f_r_hand_c_.row(1), f_r_hand_t_.row(1));
    printFingerBlock("MIDL", f_r_middle_,  t_f_r_middle_,  f_r_hand_c_.row(2), f_r_hand_t_.row(2));
    printFingerBlock("RING", f_r_ring_,    t_f_r_ring_,    f_r_hand_c_.row(3), f_r_hand_t_.row(3));
    printFingerBlock("BABY", f_r_baby_,    t_f_r_baby_,    f_r_hand_c_.row(4), f_r_hand_t_.row(4));

    printf("%s============================================================================================================%s\n", C_DIM, C_RESET);
}
