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
            // v8: robust hand parsing (suffix-based)
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
// PositionCallback
// --------------------
void DualArmForceControl::PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    (void)msg;
    if (!is_initialized_) return;
    if (!arm_fk_ || !hand_fk_l_ || !hand_fk_r_) return;

    // current FK (arms)
    std::vector<double> jl(6), jr(6);
    for (int i=0;i<6;i++){ jl[i]=q_l_c_(i); jr[i]=q_r_c_(i); }
    current_pose_l_ = arm_fk_->getLeftFK(jl);
    current_pose_r_ = arm_fk_->getRightFK(jr);

    // target pose handling:
    // - idle: target=current
    // - forward: target=FK(q_t)
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

    // ---------- helper: default point = wrist position ----------
    auto wristPoint = [&](const geometry_msgs::msg::Pose& p)->geometry_msgs::msg::Point{
        geometry_msgs::msg::Point out;
        out.x = p.position.x; out.y = p.position.y; out.z = p.position.z;
        return out;
    };

    // =======================
    // fingertips (CURRENT)
    // =======================
    f_l_thumb_  = wristPoint(current_pose_l_);
    f_l_index_  = wristPoint(current_pose_l_);
    f_l_middle_ = wristPoint(current_pose_l_);
    f_l_ring_   = wristPoint(current_pose_l_);
    f_l_baby_   = wristPoint(current_pose_l_);

    f_r_thumb_  = wristPoint(current_pose_r_);
    f_r_index_  = wristPoint(current_pose_r_);
    f_r_middle_ = wristPoint(current_pose_r_);
    f_r_ring_   = wristPoint(current_pose_r_);
    f_r_baby_   = wristPoint(current_pose_r_);

    std::vector<double> hl(20), hr(20);
    for (int i=0;i<20;i++){ hl[i]=q_l_h_c_(i); hr[i]=q_r_h_c_(i); }
    auto tl = hand_fk_l_->computeFingertips(hl);
    auto tr = hand_fk_r_->computeFingertips(hr);

    geometry_msgs::msg::Pose rel;
    if (dualarm_forcecon::kin::findPoseByKeywordCI(tl, "thumb", rel))  f_l_thumb_  = HandForwardKinematics::combinePosePoint(current_pose_l_, rel);
    if (dualarm_forcecon::kin::findPoseByKeywordCI(tl, "index", rel))  f_l_index_  = HandForwardKinematics::combinePosePoint(current_pose_l_, rel);
    if (dualarm_forcecon::kin::findPoseByKeywordCI(tl, "middle", rel)) f_l_middle_ = HandForwardKinematics::combinePosePoint(current_pose_l_, rel);
    if (dualarm_forcecon::kin::findPoseByKeywordCI(tl, "ring", rel))   f_l_ring_   = HandForwardKinematics::combinePosePoint(current_pose_l_, rel);
    if (dualarm_forcecon::kin::findPoseByKeywordCI(tl, "baby", rel))   f_l_baby_   = HandForwardKinematics::combinePosePoint(current_pose_l_, rel);

    if (dualarm_forcecon::kin::findPoseByKeywordCI(tr, "thumb", rel))  f_r_thumb_  = HandForwardKinematics::combinePosePoint(current_pose_r_, rel);
    if (dualarm_forcecon::kin::findPoseByKeywordCI(tr, "index", rel))  f_r_index_  = HandForwardKinematics::combinePosePoint(current_pose_r_, rel);
    if (dualarm_forcecon::kin::findPoseByKeywordCI(tr, "middle", rel)) f_r_middle_ = HandForwardKinematics::combinePosePoint(current_pose_r_, rel);
    if (dualarm_forcecon::kin::findPoseByKeywordCI(tr, "ring", rel))   f_r_ring_   = HandForwardKinematics::combinePosePoint(current_pose_r_, rel);
    if (dualarm_forcecon::kin::findPoseByKeywordCI(tr, "baby", rel))   f_r_baby_   = HandForwardKinematics::combinePosePoint(current_pose_r_, rel);

    // =======================
    // fingertips (TARGET)
    //  - idle : target = current
    //  - forward : FK(q_h_t_) + combine with target_pose (arm target)
    //  - inverse : keep as current (hand fixed policy)
    // =======================
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

    if (current_control_mode_ == "forward") {
        // default = wrist target
        t_f_l_thumb_  = wristPoint(target_pose_l_);
        t_f_l_index_  = wristPoint(target_pose_l_);
        t_f_l_middle_ = wristPoint(target_pose_l_);
        t_f_l_ring_   = wristPoint(target_pose_l_);
        t_f_l_baby_   = wristPoint(target_pose_l_);

        t_f_r_thumb_  = wristPoint(target_pose_r_);
        t_f_r_index_  = wristPoint(target_pose_r_);
        t_f_r_middle_ = wristPoint(target_pose_r_);
        t_f_r_ring_   = wristPoint(target_pose_r_);
        t_f_r_baby_   = wristPoint(target_pose_r_);

        std::vector<double> hl_t(20), hr_t(20);
        for (int i=0;i<20;i++){ hl_t[i]=q_l_h_t_(i); hr_t[i]=q_r_h_t_(i); }
        auto tl_t = hand_fk_l_->computeFingertips(hl_t);
        auto tr_t = hand_fk_r_->computeFingertips(hr_t);

        if (dualarm_forcecon::kin::findPoseByKeywordCI(tl_t, "thumb", rel))  t_f_l_thumb_  = HandForwardKinematics::combinePosePoint(target_pose_l_, rel);
        if (dualarm_forcecon::kin::findPoseByKeywordCI(tl_t, "index", rel))  t_f_l_index_  = HandForwardKinematics::combinePosePoint(target_pose_l_, rel);
        if (dualarm_forcecon::kin::findPoseByKeywordCI(tl_t, "middle", rel)) t_f_l_middle_ = HandForwardKinematics::combinePosePoint(target_pose_l_, rel);
        if (dualarm_forcecon::kin::findPoseByKeywordCI(tl_t, "ring", rel))   t_f_l_ring_   = HandForwardKinematics::combinePosePoint(target_pose_l_, rel);
        if (dualarm_forcecon::kin::findPoseByKeywordCI(tl_t, "baby", rel))   t_f_l_baby_   = HandForwardKinematics::combinePosePoint(target_pose_l_, rel);

        if (dualarm_forcecon::kin::findPoseByKeywordCI(tr_t, "thumb", rel))  t_f_r_thumb_  = HandForwardKinematics::combinePosePoint(target_pose_r_, rel);
        if (dualarm_forcecon::kin::findPoseByKeywordCI(tr_t, "index", rel))  t_f_r_index_  = HandForwardKinematics::combinePosePoint(target_pose_r_, rel);
        if (dualarm_forcecon::kin::findPoseByKeywordCI(tr_t, "middle", rel)) t_f_r_middle_ = HandForwardKinematics::combinePosePoint(target_pose_r_, rel);
        if (dualarm_forcecon::kin::findPoseByKeywordCI(tr_t, "ring", rel))   t_f_r_ring_   = HandForwardKinematics::combinePosePoint(target_pose_r_, rel);
        if (dualarm_forcecon::kin::findPoseByKeywordCI(tr_t, "baby", rel))   t_f_r_baby_   = HandForwardKinematics::combinePosePoint(target_pose_r_, rel);
    }

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

    // 1) Arms (>=12)
    if (n >= 12) {
        for (int i = 0; i < 6; ++i) {
            q_l_t_(i) = msg->data[i + 0];
            q_r_t_(i) = msg->data[i + 6];
        }
    } else {
        return;
    }

    // 2) Hands (>=52)
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
// PrintDualArmStates (최하단)
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
    printf("%s   Dual Arm & Hand Monitor v8 | Mode: [%s%s%s] | %sCUR_POS%s %sTAR_POS%s %sCUR_F%s %sTAR_F%s%s\n",
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
