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

    // Trigger 서비스 등록
    mode_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/change_control_mode", std::bind(&DualArmForceControl::ControlModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    q_l_c_.setZero(6); q_r_c_.setZero(6); q_l_t_.setZero(6); q_r_t_.setZero(6);
    q_l_h_c_.setZero(20); q_r_h_c_.setZero(20); q_l_h_t_.setZero(20); q_r_h_t_.setZero(20);
    f_l_c_.setZero(); f_r_c_.setZero(); f_l_t_.setZero(); f_r_t_.setZero();
    f_l_h_c_.setZero(15); f_r_h_c_.setZero(15); f_l_h_t_.setZero(15); f_r_h_t_.setZero(15);

    print_timer_ = node_->create_wall_timer(500ms, std::bind(&DualArmForceControl::PrintDualArmStates, this));
    control_timer_ = node_->create_wall_timer(2ms, std::bind(&DualArmForceControl::ControlLoop, this));

    is_initialized_ = true;
}

void DualArmForceControl::ControlLoop() {
    if (!is_initialized_) return;
    if (current_control_mode_ == "idle") {
        q_l_t_ = q_l_c_; q_r_t_ = q_r_c_;
        q_l_h_t_ = q_l_h_c_; q_r_h_t_ = q_r_h_c_;
    }
}

DualArmForceControl::~DualArmForceControl() {}