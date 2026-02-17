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
        // Arm 매핑
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
        // 몸체 및 기타 조인트는 0.0 고정
        else if (name == "yaw_joint" || name == "pitch_joint") cmd.position.push_back(0.0);
        // 핸드 조인트 (이름 패턴에 따라 추가 가능)
        else cmd.position.push_back(0.0); 
    }

    joint_command_pub_->publish(cmd);
}

DualArmForceControl::~DualArmForceControl() {}