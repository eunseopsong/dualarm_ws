#include "DualArmForceControl.h"
#include <cstdio>
#include <string>
#include <cmath>

// JointsCallback, PositionCallback 등은 이전과 동일 (생략 가능하나 전체 코드 요청하셨으므로 포함)
void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (joint_names_.empty()) { joint_names_ = msg->name; is_initialized_ = true; }
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::string n = msg->name[i]; double p = msg->position[i];
        if (n == "left_joint_1") q_l_c_(0) = p; else if (n == "left_joint_2") q_l_c_(1) = p;
        else if (n == "left_joint_3") q_l_c_(2) = p; else if (n == "left_joint_4") q_l_c_(3) = p;
        else if (n == "left_joint_5") q_l_c_(4) = p; else if (n == "left_joint_6") q_l_c_(5) = p;
        else if (n == "right_joint_1") q_r_c_(0) = p; else if (n == "right_joint_2") q_r_c_(1) = p;
        else if (n == "right_joint_3") q_r_c_(2) = p; else if (n == "right_joint_4") q_r_c_(3) = p;
        else if (n == "right_joint_5") q_r_c_(4) = p; else if (n == "right_joint_6") q_r_c_(5) = p;
        else {
            int f_idx = -1;
            if (n.find("thumb") != std::string::npos) f_idx = 0; else if (n.find("index") != std::string::npos) f_idx = 4;
            else if (n.find("middle") != std::string::npos) f_idx = 8; else if (n.find("ring") != std::string::npos) f_idx = 12;
            else if (n.find("baby") != std::string::npos) f_idx = 16;
            if (f_idx != -1) {
                int j_idx = -1;
                if (n.find("1") != std::string::npos) j_idx = 0; else if (n.find("2") != std::string::npos) j_idx = 1;
                else if (n.find("3") != std::string::npos) j_idx = 2; else if (n.find("4") != std::string::npos) j_idx = 3;
                if (j_idx != -1) {
                    if (n.find("left") != std::string::npos) q_l_h_c_(f_idx + j_idx) = p;
                    else if (n.find("right") != std::string::npos) q_r_h_c_(f_idx + j_idx) = p;
                }
            }
        }
    }
}

void DualArmForceControl::PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    (void)msg; if (!arm_fk_ || !is_initialized_) return;
    std::vector<double> jl(q_l_c_.data(), q_l_c_.data() + q_l_c_.size());
    std::vector<double> jr(q_r_c_.data(), q_r_c_.data() + q_r_c_.size());
    current_pose_l_ = arm_fk_->getLeftFK(jl);
    current_pose_r_ = arm_fk_->getRightFK(jr);
}

// ✅ [추가] Inverse Kinematics 타겟 콜백
void DualArmForceControl::TargetPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (current_control_mode_ != "inverse" || msg->data.size() < 12) return;

    double target_l_xyz[3] = {msg->data[0], msg->data[1], msg->data[2]};
    double target_l_rpy[3] = {msg->data[3], msg->data[4], msg->data[5]};
    double target_r_xyz[3] = {msg->data[6], msg->data[7], msg->data[8]};
    double target_r_rpy[3] = {msg->data[9], msg->data[10], msg->data[11]};

    std::vector<double> cur_ql(q_l_c_.data(), q_l_c_.data() + 6);
    std::vector<double> cur_qr(q_r_c_.data(), q_r_c_.data() + 6);
    std::vector<double> res_ql, res_qr;

    if (arm_ik_l_->solveIK(cur_ql, target_l_xyz, target_l_rpy, res_ql)) {
        for(int i=0; i<6; i++) q_l_t_(i) = res_ql[i];
    }
    if (arm_ik_r_->solveIK(cur_qr, target_r_xyz, target_r_rpy, res_qr)) {
        for(int i=0; i<6; i++) q_r_t_(i) = res_qr[i];
    }
}

void DualArmForceControl::TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (current_control_mode_ != "forward" || msg->data.size() < 12) return;
    for(int i=0; i<6; i++) { q_l_t_(i) = msg->data[i]; q_r_t_(i) = msg->data[i+6]; }
}

void DualArmForceControl::ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req;
    // ✅ 모드 순환: idle -> forward -> inverse -> idle
    if (current_control_mode_ == "idle") current_control_mode_ = "forward";
    else if (current_control_mode_ == "forward") current_control_mode_ = "inverse";
    else current_control_mode_ = "idle";

    if (current_control_mode_ == "idle") idle_synced_ = false;
    res->success = true;
    res->message = "Mode changed to: " + current_control_mode_;
}

// ContactForceCallback 및 PrintDualArmStates는 이전과 동일 (생략)
void DualArmForceControl::ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 6) return;
    f_l_c_ << msg->data[0], msg->data[1], msg->data[2];
    f_r_c_ << msg->data[3], msg->data[4], msg->data[5];
}

void DualArmForceControl::PrintDualArmStates() {
    if (!is_initialized_) return;
    printf("\033[2J\033[H");
    printf("========================================================================================\n");
    printf("   Dual Arm Monitor | Mode: [\033[1;32m%-7s\033[0m] | Cyan: Curr, Yel: Targ\n", current_control_mode_.c_str());
    printf("========================================================================================\n");
    auto print_arm = [&](const char* side, Eigen::VectorXd& c, Eigen::VectorXd& t, Eigen::Vector3d& fc, geometry_msgs::msg::Pose& pose) {
        double qx = pose.orientation.x; double qy = pose.orientation.y; double qz = pose.orientation.z; double qw = pose.orientation.w;
        double roll = std::atan2(2.0*(qw*qx+qy*qz), 1.0-2.0*(qx*qx+qy*qy));
        double sinp = 2.0*(qw*qy-qz*qx);
        double pitch = (std::abs(sinp)>=1.0) ? std::copysign(M_PI/2.0, sinp) : std::asin(sinp);
        double yaw = std::atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));
        printf("[%s ARM] Joints: \033[1;36m%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\033[0m | F_C: \033[1;36m%5.1f %5.1f %5.1f\033[0m\n", side, c(0),c(1),c(2),c(3),c(4),c(5), fc(0),fc(1),fc(2));
        printf("        Target: \033[1;33m%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\033[0m\n", t(0),t(1),t(2),t(3),t(4),t(5));
        printf("        \033[1;35m-> Curr Pose (XYZ, RPY): [%5.3f, %5.3f, %5.3f] / [%5.3f, %5.3f, %5.3f]\033[0m\n", pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw);
    };
    print_arm("L", q_l_c_, q_l_t_, f_l_c_, current_pose_l_);
    print_arm("R", q_r_c_, q_r_t_, f_r_c_, current_pose_r_);
    printf("========================================================================================\n");
}