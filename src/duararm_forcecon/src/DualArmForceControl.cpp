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
    // 2. Initialization (Current States)
    // ========================================================================
    // 현재값 초기화
    q_left_.setZero(ARM_DOF);
    q_right_.setZero(ARM_DOF);
    q_left_hand_.setZero(HAND_DOF);
    q_right_hand_.setZero(HAND_DOF);
    
    force_left_.setZero();
    torque_left_.setZero();
    force_right_.setZero();
    torque_right_.setZero();
    
    // ========================================================================
    // 3. Initialization (Target States)
    // ========================================================================
    // [추가] 목표값 변수 초기화 (일단 0으로 설정)
    qd_left_.setZero(ARM_DOF);
    qd_right_.setZero(ARM_DOF);
    qd_left_hand_.setZero(HAND_DOF);
    qd_right_hand_.setZero(HAND_DOF);

    // ========================================================================
    // 4. Timers
    // ========================================================================
    // [추가] 0.5초(500ms) 마다 상태 출력 함수 실행
    print_timer_ = node_->create_wall_timer(
        500ms, std::bind(&DualArmForceControl::PrintDualArmStates, this));

    is_initialized_ = true;
}

DualArmForceControl::~DualArmForceControl()
{
    RCLCPP_INFO(node_->get_logger(), "DualArmForceControl Class Destroyed.");
}