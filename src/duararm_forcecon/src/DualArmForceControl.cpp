#include "DualArmForceControl.h"
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

DualArmForceControl::DualArmForceControl(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "DualArmForceControl Class Created.");

    // QoS 설정
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    sensor_qos.best_effort();
    sensor_qos.durability_volatile();

    // ========================================================================
    // 1. Subscribers
    // ========================================================================
    
    // [Joints] Topic: /isaac_joint_states
    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/isaac_joint_states", sensor_qos, 
        std::bind(&DualArmForceControl::JointsCallback, this, std::placeholders::_1));

    // [Force] Topic: /isaac_contact_states (Float64MultiArray)
    ftsensor_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/isaac_contact_states", sensor_qos, 
        std::bind(&DualArmForceControl::ContactForceCallback, this, std::placeholders::_1));

    // ========================================================================
    // 2. Initialization
    // ========================================================================
    q_left_.setZero(ARM_DOF);
    q_right_.setZero(ARM_DOF);
    q_left_hand_.setZero(HAND_DOF);
    q_right_hand_.setZero(HAND_DOF);
    
    force_left_.setZero();
    torque_left_.setZero();
    force_right_.setZero();
    torque_right_.setZero();
    
    is_initialized_ = true;
}

DualArmForceControl::~DualArmForceControl()
{
    RCLCPP_INFO(node_->get_logger(), "DualArmForceControl Class Destroyed.");
}