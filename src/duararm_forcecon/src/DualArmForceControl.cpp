#include "DualArmForceControl.h"
#include <chrono>
#include <functional>
#include <memory>

using namespace std::chrono_literals;

DualArmForceControl::DualArmForceControl(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "DualArmForceControl Class Created.");

    // QoS 설정 (Sensor Data용 Best Effort)
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    sensor_qos.best_effort();
    sensor_qos.durability_volatile();

    // 일반 QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability_volatile();

    // ========================================================================
    // 1. Subscriber 생성
    // ========================================================================
    
    // [수정] Joint States Subscriber (Callback 이름 변경: JointsCallback)
    // Topic: /isaac_joint_states
    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/isaac_joint_states", sensor_qos, 
        std::bind(&DualArmForceControl::JointsCallback, this, std::placeholders::_1));

    // FT Sensor Subscriber
    // Topic: /isaac_contact_states
    // ftsensor_sub_ = node_->create_subscription<geometry_msgs::msg::Wrench>(
    //     "/isaac_contact_states", qos, 
    //     std::bind(&DualArmForceControl::ftSensorCallback, this, std::placeholders::_1));

    // ========================================================================
    // 2. 변수 초기화
    // ========================================================================
    q_left_.setZero(ARM_DOF);
    q_right_.setZero(ARM_DOF);
    q_left_hand_.setZero(HAND_DOF);
    q_right_hand_.setZero(HAND_DOF);
    
    teleop_force_.setZero();
    contact_force = 0.0;
    
    is_initialized_ = true;
}

DualArmForceControl::~DualArmForceControl()
{
    RCLCPP_INFO(node_->get_logger(), "DualArmForceControl Class Destroyed.");
}

// 참고: 
// JointsCallback()과 ftSensorCallback()의 구현 내용은 
// src/states_callback_dualarm.cpp 파일에 있습니다.