#include "DualArmForceControl.h"
#include <cstdio>
#include <cmath>

void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (joint_names_.empty()) {
        joint_names_ = msg->name;
        is_initialized_ = true;
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& n = msg->name[i];
        double p = msg->position[i];

        if      (n == "left_joint_1")  q_l_c_(0) = p;
        else if (n == "left_joint_2")  q_l_c_(1) = p;
        else if (n == "left_joint_3")  q_l_c_(2) = p;
        else if (n == "left_joint_4")  q_l_c_(3) = p;
        else if (n == "left_joint_5")  q_l_c_(4) = p;
        else if (n == "left_joint_6")  q_l_c_(5) = p;
        else if (n == "right_joint_1") q_r_c_(0) = p;
        else if (n == "right_joint_2") q_r_c_(1) = p;
        else if (n == "right_joint_3") q_r_c_(2) = p;
        else if (n == "right_joint_4") q_r_c_(3) = p;
        else if (n == "right_joint_5") q_r_c_(4) = p;
        else if (n == "right_joint_6") q_r_c_(5) = p;
        else {
            int f_idx =
                (n.find("thumb")  != std::string::npos) ? 0  :
                (n.find("index")  != std::string::npos) ? 4  :
                (n.find("middle") != std::string::npos) ? 8  :
                (n.find("ring")   != std::string::npos) ? 12 :
                (n.find("baby")   != std::string::npos) ? 16 : -1;

            if (f_idx != -1) {
                int j_idx =
                    (n.find("1") != std::string::npos) ? 0 :
                    (n.find("2") != std::string::npos) ? 1 :
                    (n.find("3") != std::string::npos) ? 2 :
                    (n.find("4") != std::string::npos) ? 3 : -1;

                if (j_idx != -1) {
                    if (n.find("left") != std::string::npos) q_l_h_c_(f_idx + j_idx) = p;
                    else                                       q_r_h_c_(f_idx + j_idx) = p;
                }
            }
        }
    }
}

void DualArmForceControl::PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    (void)msg;
    if (!arm_fk_ || !arm_fk_->isOk() || !is_initialized_) return;

    // 1) Arm FK (World 기준으로 저장)
    std::vector<double> jl(q_l_c_.data(), q_l_c_.data() + 6);
    std::vector<double> jr(q_r_c_.data(), q_r_c_.data() + 6);

    current_pose_l_ = arm_fk_->getLeftFKWorld(jl);
    current_pose_r_ = arm_fk_->getRightFKWorld(jr);

    // 2) Hand FK (기존 유지)
    std::vector<double> hl(q_l_h_c_.data(), q_l_h_c_.data() + 20);
    std::vector<double> hr(q_r_h_c_.data(), q_r_h_c_.data() + 20);

    auto tl = hand_fk_l_->computeFingertips(hl);
    auto tr = hand_fk_r_->computeFingertips(hr);

    if (!tl.empty()) {
        f_l_thumb_  = combinePose(current_pose_l_, tl["link4_thumb"]);
        f_l_index_  = combinePose(current_pose_l_, tl["link4_index"]);
        f_l_middle_ = combinePose(current_pose_l_, tl["link4_middle"]);
        f_l_ring_   = combinePose(current_pose_l_, tl["link4_ring"]);
        f_l_baby_   = combinePose(current_pose_l_, tl["link4_baby"]);
    }
    if (!tr.empty()) {
        f_r_thumb_  = combinePose(current_pose_r_, tr["link4_thumb"]);
        f_r_index_  = combinePose(current_pose_r_, tr["link4_index"]);
        f_r_middle_ = combinePose(current_pose_r_, tr["link4_middle"]);
        f_r_ring_   = combinePose(current_pose_r_, tr["link4_ring"]);
        f_r_baby_   = combinePose(current_pose_r_, tr["link4_baby"]);
    }
}

void DualArmForceControl::PrintDualArmStates()
{
    if (!is_initialized_) return;

    printf("\033[2J\033[H");
    printf("============================================================================================================\n");
    printf("   Dual Arm & Hand Monitor v5.1 | Mode: [\033[1;32m%-7s\033[0m] | Cyan: Curr, Yel: Targ\n",
           current_control_mode_.c_str());
    printf("============================================================================================================\n");

    auto print_arm = [&](const char* side, geometry_msgs::msg::Pose& p,
                         Eigen::VectorXd& cur_q, Eigen::VectorXd& tar_q)
    {
        double ox, oy, oz;
        quatToEulerXYZDeg(p.orientation, ox, oy, oz);

        printf("[%s ARM] Joints: \033[1;36m%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f\033[0m\n",
               side, cur_q(0), cur_q(1), cur_q(2), cur_q(3), cur_q(4), cur_q(5));
        printf("        Target: \033[1;33m%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f\033[0m\n",
               tar_q(0), tar_q(1), tar_q(2), tar_q(3), tar_q(4), tar_q(5));

        // Isaac UI처럼: Translate(m) + Orient XYZ(deg)
        printf("        Translate: \033[1;35m[%0.5f, %0.5f, %0.5f]\033[0m | "
               "Orient(XYZ deg): \033[1;33m[%7.3f, %7.3f, %7.3f]\033[0m\n",
               p.position.x, p.position.y, p.position.z, ox, oy, oz);
    };

    print_arm("L", current_pose_l_, q_l_c_, q_l_t_);
    printf("------------------------------------------------------------------------------------------------------------\n");
    print_arm("R", current_pose_r_, q_r_c_, q_r_t_);
    printf("============================================================================================================\n");

    printf("[FINGERTIP POSITIONS (World Frame)]\n");
    auto print_fingers = [&](const char* side,
                             geometry_msgs::msg::Point& t, geometry_msgs::msg::Point& i,
                             geometry_msgs::msg::Point& m, geometry_msgs::msg::Point& r,
                             geometry_msgs::msg::Point& b)
    {
        printf("%s: THUMB[%0.3f, %0.3f, %0.3f] INDEX[%0.3f, %0.3f, %0.3f] MID[%0.3f, %0.3f, %0.3f]\n",
               side, t.x, t.y, t.z, i.x, i.y, i.z, m.x, m.y, m.z);
        printf("   RING [%0.3f, %0.3f, %0.3f] BABY [%0.3f, %0.3f, %0.3f]\n",
               r.x, r.y, r.z, b.x, b.y, b.z);
    };

    print_fingers("L", f_l_thumb_, f_l_index_, f_l_middle_, f_l_ring_, f_l_baby_);
    printf("-\n");
    print_fingers("R", f_r_thumb_, f_r_index_, f_r_middle_, f_r_ring_, f_r_baby_);
    printf("============================================================================================================\n");
}

// ----------- existing callbacks 유지 (단, IK 옵션 확장) -----------

void DualArmForceControl::TargetPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_control_mode_ != "inverse" || msg->data.size() < 12) return;

    double l_xyz[3] = {msg->data[0], msg->data[1], msg->data[2]};
    double l_eul[3] = {msg->data[3], msg->data[4], msg->data[5]};
    double r_xyz[3] = {msg->data[6], msg->data[7], msg->data[8]};
    double r_eul[3] = {msg->data[9], msg->data[10], msg->data[11]};

    std::vector<double> ql(q_l_c_.data(), q_l_c_.data() + 6);
    std::vector<double> qr(q_r_c_.data(), q_r_c_.data() + 6);
    std::vector<double> rl, rr;

    // IK 옵션 파싱
    ArmInverseKinematics::EulerConv conv =
        (ik_euler_conv_ == "xyz") ? ArmInverseKinematics::EulerConv::XYZ : ArmInverseKinematics::EulerConv::RPY_ZYX;

    ArmInverseKinematics::AngleUnit unit =
        (ik_angle_unit_ == "deg")  ? ArmInverseKinematics::AngleUnit::DEG :
        (ik_angle_unit_ == "auto") ? ArmInverseKinematics::AngleUnit::AUTO :
                                     ArmInverseKinematics::AngleUnit::RAD;

    bool okL = false, okR = false;

    if (ik_targets_frame_ == "world") {
        okL = arm_ik_l_->solveIKWorld(ql, T_world_base_, l_xyz, l_eul, rl, conv, unit);
        okR = arm_ik_r_->solveIKWorld(qr, T_world_base_, r_xyz, r_eul, rr, conv, unit);
    } else {
        // base 입력 (기본 호환)
        if (ik_euler_conv_ == "rpy" && ik_angle_unit_ == "rad") {
            okL = arm_ik_l_->solveIK(ql, l_xyz, l_eul, rl);
            okR = arm_ik_r_->solveIK(qr, r_xyz, r_eul, rr);
        } else {
            okL = arm_ik_l_->solveIKWithEuler(ql, l_xyz, l_eul, rl, conv, unit);
            okR = arm_ik_r_->solveIKWithEuler(qr, r_xyz, r_eul, rr, conv, unit);
        }
    }

    if (okL) for (int i = 0; i < 6; i++) q_l_t_(i) = rl[i];
    if (okR) for (int i = 0; i < 6; i++) q_r_t_(i) = rr[i];
}

void DualArmForceControl::TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_control_mode_ == "forward" && msg->data.size() >= 12) {
        for (int i = 0; i < 6; i++) {
            q_l_t_(i) = msg->data[i];
            q_r_t_(i) = msg->data[i + 6];
        }
    }
}

void DualArmForceControl::ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    current_control_mode_ =
        (current_control_mode_ == "idle")    ? "forward" :
        (current_control_mode_ == "forward") ? "inverse" : "idle";

    if (current_control_mode_ == "idle") idle_synced_ = false;

    res->success = true;
    res->message = "Mode: " + current_control_mode_;
}

void DualArmForceControl::ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 6) {
        f_l_c_ << msg->data[0], msg->data[1], msg->data[2];
        f_r_c_ << msg->data[3], msg->data[4], msg->data[5];
    }
}
