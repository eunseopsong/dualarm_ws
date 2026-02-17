#include "DualArmForceControl.h"
#include <cstdio>

void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 최초 실행 시 조인트 이름 리스트 동기화
    if (joint_names_.empty()) {
        joint_names_ = msg->name;
        is_initialized_ = true;
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::string n = msg->name[i];
        double p = msg->position[i];

        if (n == "left_joint_1") q_l_c_(0) = p;
        else if (n == "left_joint_2") q_l_c_(1) = p;
        else if (n == "left_joint_3") q_l_c_(2) = p;
        else if (n == "left_joint_4") q_l_c_(3) = p;
        else if (n == "left_joint_5") q_l_c_(4) = p;
        else if (n == "left_joint_6") q_l_c_(5) = p;
        else if (n == "right_joint_1") q_r_c_(0) = p;
        else if (n == "right_joint_2") q_r_c_(1) = p;
        else if (n == "right_joint_3") q_r_c_(2) = p;
        else if (n == "right_joint_4") q_r_c_(3) = p;
        else if (n == "right_joint_5") q_r_c_(4) = p;
        else if (n == "right_joint_6") q_r_c_(5) = p;
    }
}

void DualArmForceControl::TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 12) return;
    for(int i=0; i<6; i++) { q_l_t_(i) = msg->data[i]; q_r_t_(i) = msg->data[i+6]; }
    if (msg->data.size() >= 52) {
        for(int i=0; i<20; i++) { q_l_h_t_(i) = msg->data[i+12]; q_r_h_t_(i) = msg->data[i+32]; }
    }
}

void DualArmForceControl::ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    current_control_mode_ = (current_control_mode_ == "idle") ? "forward" : "idle";
    res->success = true;
    res->message = "Mode: " + current_control_mode_;
}

void DualArmForceControl::ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 6) return;
    f_l_c_ << msg->data[0], msg->data[1], msg->data[2];
    f_r_c_ << msg->data[3], msg->data[4], msg->data[5];
}

void DualArmForceControl::PrintDualArmStates() {
    if (!is_initialized_) return;
    printf("\033[H");
    printf("================================================================================\n");
    printf("   Dual Arm & Hand Monitor | Mode: [\033[1;32m%-7s\033[0m] | [Cyan: Curr, Yellow: Targ]\n", current_control_mode_.c_str());
    printf("================================================================================\n");

    auto print_arm = [&](const char* side, Eigen::VectorXd& c, Eigen::VectorXd& t, Eigen::Vector3d& fc, Eigen::Vector3d& ft) {
        printf("[%s ARM]  Joints: \033[1;36m", side);
        for(int i=0; i<6; i++) printf("%6.2f ", c(i));
        printf("\033[0m\n         Target: \033[1;33m");
        for(int i=0; i<6; i++) printf("%6.2f ", t(i));
        printf("\033[0m\n         Force : \033[1;36m%6.2f %6.2f %6.2f\033[0m\n", fc(0), fc(1), fc(2));
        printf("         F_Targ: \033[1;33m%6.2f %6.2f %6.2f\033[0m\n", ft(0), ft(1), ft(2));
    };

    print_arm("L", q_l_c_, q_l_t_, f_l_c_, f_l_t_);
    printf("--------------------------------------------------------------------------------\n");
    print_arm("R", q_r_c_, q_r_t_, f_r_c_, f_r_t_);
    printf("================================================================================\n");

    const char* f_n[] = {"Thumb", "Index", "Middle", "Ring", "Baby"};
    printf("[HAND] Finger: Joint (1, 2, 3, 4)                        | Force (Fx, Fy, Fz)\n");
    printf("--------------------------------------------------------------------------------\n");
    for(int i=0; i<5; i++) {
        printf("L_%-6s \033[1;36m%5.1f %5.1f %5.1f %5.1f\033[0m \033[1;33m%5.1f %5.1f %5.1f %5.1f\033[0m | \033[1;36m%5.1f %5.1f %5.1f\033[0m \033[1;33m%5.1f %5.1f %5.1f\033[0m\n", 
               f_n[i], q_l_h_c_(i*4), q_l_h_c_(i*4+1), q_l_h_c_(i*4+2), q_l_h_c_(i*4+3), q_l_h_t_(i*4), q_l_h_t_(i*4+1), q_l_h_t_(i*4+2), q_l_h_t_(i*4+3),
               f_l_h_c_(i*3), f_l_h_c_(i*3+1), f_l_h_c_(i*3+2), f_l_h_t_(i*3), f_l_h_t_(i*3+1), f_l_h_t_(i*3+2));
    }
    printf("--------------------------------------------------------------------------------\n");
    for(int i=0; i<5; i++) {
        printf("R_%-6s \033[1;36m%5.1f %5.1f %5.1f %5.1f\033[0m \033[1;33m%5.1f %5.1f %5.1f %5.1f\033[0m | \033[1;36m%5.1f %5.1f %5.1f\033[0m \033[1;33m%5.1f %5.1f %5.1f\033[0m\n", 
               f_n[i], q_r_h_c_(i*4), q_r_h_c_(i*4+1), q_r_h_c_(i*4+2), q_r_h_c_(i*4+3), q_r_h_t_(i*4), q_r_h_t_(i*4+1), q_r_h_t_(i*4+2), q_r_h_t_(i*4+3),
               f_r_h_c_(i*3), f_r_h_c_(i*3+1), f_r_h_c_(i*3+2), f_r_h_t_(i*3), f_r_h_t_(i*3+1), f_r_h_t_(i*3+2));
    }
    printf("================================================================================\n");
}