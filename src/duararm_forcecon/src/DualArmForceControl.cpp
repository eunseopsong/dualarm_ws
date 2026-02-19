#include "DualArmForceControl.h"
#include <chrono>

using namespace std::chrono_literals;

DualArmForceControl::DualArmForceControl(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/isaac_joint_states", qos, std::bind(&DualArmForceControl::JointsCallback, this, std::placeholders::_1));
    contact_force_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/isaac_contact_states", qos, std::bind(&DualArmForceControl::ContactForceCallback, this, std::placeholders::_1));
    target_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/forward_joint_targets", qos, std::bind(&DualArmForceControl::TargetJointCallback, this, std::placeholders::_1));

    joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_command", 10);
    mode_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/change_control_mode", std::bind(&DualArmForceControl::ControlModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Vector Initialization
    q_l_c_.setZero(6); q_r_c_.setZero(6); q_l_t_.setZero(6); q_r_t_.setZero(6);
    q_l_h_c_.setZero(20); q_r_h_c_.setZero(20); q_l_h_t_.setZero(20); q_r_h_t_.setZero(20);
    f_l_c_.setZero(); f_r_c_.setZero(); f_l_t_.setZero(); f_r_t_.setZero();
    f_l_h_c_.setZero(15); f_r_h_c_.setZero(15); f_l_h_t_.setZero(15); f_r_h_t_.setZero(15);

    print_timer_ = node_->create_wall_timer(500ms, std::bind(&DualArmForceControl::PrintDualArmStates, this));
    control_timer_ = node_->create_wall_timer(10ms, std::bind(&DualArmForceControl::ControlLoop, this));
}

void DualArmForceControl::ControlLoop() {
    if (!is_initialized_ || joint_names_.empty()) return;

    if (current_control_mode_ == "idle") {
        q_l_t_ = q_l_c_; q_r_t_ = q_r_c_;
        q_l_h_t_ = q_l_h_c_; q_r_h_t_ = q_r_h_c_;
    }

    auto cmd = sensor_msgs::msg::JointState();
    cmd.header.stamp = node_->now();
    cmd.name = joint_names_; // Isaac Sim의 순서 그대로 사용

    for (const auto& name : joint_names_) {
        // [안전 구역 1] 팔(Arm) 매핑 - 기존 작동 코드 100% 유지
        if (name == "left_joint_1") cmd.position.push_back(q_l_t_(0));
        else if (name == "left_joint_2") cmd.position.push_back(q_l_t_(1));
        else if (name == "left_joint_3") cmd.position.push_back(q_l_t_(2));
        else if (name == "left_joint_4") cmd.position.push_back(q_l_t_(3));
        else if (name == "left_joint_5") cmd.position.push_back(q_l_t_(4));
        else if (name == "left_joint_6") cmd.position.push_back(q_l_t_(5));
        else if (name == "right_joint_1") cmd.position.push_back(q_r_t_(0));
        else if (name == "right_joint_2") cmd.position.push_back(q_r_t_(1));
        else if (name == "right_joint_3") cmd.position.push_back(q_r_t_(2));
        else if (name == "right_joint_4") cmd.position.push_back(q_r_t_(3));
        else if (name == "right_joint_5") cmd.position.push_back(q_r_t_(4));
        else if (name == "right_joint_6") cmd.position.push_back(q_r_t_(5));
        else if (name == "yaw_joint" || name == "pitch_joint") cmd.position.push_back(0.0);
        
        // [추가 구역] 손가락(Hand) 매핑 - stoi 에러 없이 키워드만 검색
        else {
            int f_idx = -1;
            if (name.find("thumb") != std::string::npos) f_idx = 0;
            else if (name.find("index") != std::string::npos) f_idx = 4;
            else if (name.find("middle") != std::string::npos) f_idx = 8;
            else if (name.find("ring") != std::string::npos) f_idx = 12;
            else if (name.find("baby") != std::string::npos) f_idx = 16;

            if (f_idx != -1) {
                int j_idx = -1;
                if (name.find("1") != std::string::npos) j_idx = 0;
                else if (name.find("2") != std::string::npos) j_idx = 1;
                else if (name.find("3") != std::string::npos) j_idx = 2;
                else if (name.find("4") != std::string::npos) j_idx = 3;

                if (j_idx != -1) {
                    if (name.find("left") != std::string::npos) cmd.position.push_back(q_l_h_t_(f_idx + j_idx));
                    else if (name.find("right") != std::string::npos) cmd.position.push_back(q_r_h_t_(f_idx + j_idx));
                    else cmd.position.push_back(0.0);
                } else {
                    cmd.position.push_back(0.0);
                }
            } else {
                cmd.position.push_back(0.0); 
            }
        }
    }

    joint_command_pub_->publish(cmd);
}

DualArmForceControl::~DualArmForceControl() {}