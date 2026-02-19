#include "DualArmForceControl.h"
#include <cstdio>
#include <string>
#include <cmath> // RPY 변환용 수학 함수 추가

void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
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
        else {
            int f_idx = -1;
            if (n.find("thumb") != std::string::npos) f_idx = 0;
            else if (n.find("index") != std::string::npos) f_idx = 4;
            else if (n.find("middle") != std::string::npos) f_idx = 8;
            else if (n.find("ring") != std::string::npos) f_idx = 12;
            else if (n.find("baby") != std::string::npos) f_idx = 16;

            if (f_idx != -1) {
                int j_idx = -1;
                if (n.find("1") != std::string::npos) j_idx = 0;
                else if (n.find("2") != std::string::npos) j_idx = 1;
                else if (n.find("3") != std::string::npos) j_idx = 2;
                else if (n.find("4") != std::string::npos) j_idx = 3;

                if (j_idx != -1) {
                    if (n.find("left") != std::string::npos) q_l_h_c_(f_idx + j_idx) = p;
                    else if (n.find("right") != std::string::npos) q_r_h_c_(f_idx + j_idx) = p;
                }
            }
        }
    }
}

void DualArmForceControl::PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    (void)msg; // 경고 방지
    if (!arm_fk_ || !is_initialized_) return;

    std::vector<double> jl(q_l_c_.data(), q_l_c_.data() + q_l_c_.size());
    std::vector<double> jr(q_r_c_.data(), q_r_c_.data() + q_r_c_.size());
    
    current_pose_l_ = arm_fk_->getLeftFK(jl);
    current_pose_r_ = arm_fk_->getRightFK(jr);
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
    if (current_control_mode_ == "idle") idle_synced_ = false;
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
    
    printf("\033[2J\033[H");
    printf("========================================================================================\n");
    printf("   Dual Arm & Hand Monitor | Mode: [\033[1;32m%-7s\033[0m] | Cyan: Curr, Yel: Targ\n", current_control_mode_.c_str());
    printf("========================================================================================\n");

    auto print_arm = [&](const char* side, Eigen::VectorXd& c, Eigen::VectorXd& t, Eigen::Vector3d& fc, Eigen::Vector3d& ft, geometry_msgs::msg::Pose& pose) {
        
        // 🌟 쿼터니언을 RPY(Roll, Pitch, Yaw)로 변환하는 로직
        double qx = pose.orientation.x;
        double qy = pose.orientation.y;
        double qz = pose.orientation.z;
        double qw = pose.orientation.w;

        // Roll (x-axis)
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        double roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis)
        double sinp = 2.0 * (qw * qy - qz * qx);
        double pitch;
        if (std::abs(sinp) >= 1.0)
            pitch = std::copysign(M_PI / 2.0, sinp); // 범위를 벗어나면 90도로 고정
        else
            pitch = std::asin(sinp);

        // Yaw (z-axis)
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        // 출력 형식 맞추기
        printf("[%s ARM] Joints: \033[1;36m%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\033[0m | F_C: \033[1;36m%5.1f %5.1f %5.1f\033[0m\n", 
               side, c(0),c(1),c(2),c(3),c(4),c(5), fc(0),fc(1),fc(2));
        printf("        Target: \033[1;33m%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\033[0m | F_T: \033[1;33m%5.1f %5.1f %5.1f\033[0m\n", 
               t(0),t(1),t(2),t(3),t(4),t(5), ft(0),ft(1),ft(2));
               
        // 🌟 XYZ 와 RPY 함께 출력 (RPY 단위는 라디안)
        printf("        \033[1;35m-> Curr Pose (XYZ, RPY): [%5.3f, %5.3f, %5.3f] / [%5.3f, %5.3f, %5.3f]\033[0m\n", 
               pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw);
    };

    print_arm("L", q_l_c_, q_l_t_, f_l_c_, f_l_t_, current_pose_l_);
    print_arm("R", q_r_c_, q_r_t_, f_r_c_, f_r_t_, current_pose_r_);
    
    printf("----------------------------------------------------------------------------------------\n");
    printf("[HAND]    | Joints: Curr(C) / Targ(Y)             | Forces: Curr(C) / Targ(Y)\n");
    printf("----------------------------------------------------------------------------------------\n");

    const char* f_names[] = {"Thumb", "Index", "Middle", "Ring", "Baby"};
    
    for(int i=0; i<5; i++) {
        int q_idx = i * 4;
        int f_idx = i * 3;
        printf("L_%-7s | \033[1;36m%5.1f %5.1f %5.1f %5.1f\033[0m / \033[1;33m%5.1f %5.1f %5.1f %5.1f\033[0m | \033[1;36m%5.1f %5.1f %5.1f\033[0m / \033[1;33m%5.1f %5.1f %5.1f\033[0m\n", 
               f_names[i], 
               q_l_h_c_(q_idx), q_l_h_c_(q_idx+1), q_l_h_c_(q_idx+2), q_l_h_c_(q_idx+3),
               q_l_h_t_(q_idx), q_l_h_t_(q_idx+1), q_l_h_t_(q_idx+2), q_l_h_t_(q_idx+3),
               f_l_h_c_(f_idx), f_l_h_c_(f_idx+1), f_l_h_c_(f_idx+2),
               f_l_h_t_(f_idx), f_l_h_t_(f_idx+1), f_l_h_t_(f_idx+2));
    }
    
    printf("          |                                       |\n");
    
    for(int i=0; i<5; i++) {
        int q_idx = i * 4;
        int f_idx = i * 3;
        printf("R_%-7s | \033[1;36m%5.1f %5.1f %5.1f %5.1f\033[0m / \033[1;33m%5.1f %5.1f %5.1f %5.1f\033[0m | \033[1;36m%5.1f %5.1f %5.1f\033[0m / \033[1;33m%5.1f %5.1f %5.1f\033[0m\n", 
               f_names[i], 
               q_r_h_c_(q_idx), q_r_h_c_(q_idx+1), q_r_h_c_(q_idx+2), q_r_h_c_(q_idx+3),
               q_r_h_t_(q_idx), q_r_h_t_(q_idx+1), q_r_h_t_(q_idx+2), q_r_h_t_(q_idx+3),
               f_r_h_c_(f_idx), f_r_h_c_(f_idx+1), f_r_h_c_(f_idx+2),
               f_r_h_t_(f_idx), f_r_h_t_(f_idx+1), f_r_h_t_(f_idx+2));
    }
    printf("========================================================================================\n");
}