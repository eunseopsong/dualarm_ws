#include "DualArmForceControl.h"
#include <chrono>

using namespace std::chrono_literals;

DualArmForceControl::DualArmForceControl(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/isaac_joint_states", qos, std::bind(&DualArmForceControl::JointsCallback, this, std::placeholders::_1));
    ftsensor_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/isaac_contact_states", qos, std::bind(&DualArmForceControl::ContactForceCallback, this, std::placeholders::_1));

    // 메모리 할당 및 초기화
    q_l_c_.setZero(ARM_DOF); q_r_c_.setZero(ARM_DOF);
    q_l_t_.setZero(ARM_DOF); q_r_t_.setZero(ARM_DOF);
    
    q_l_h_c_.setZero(HAND_DOF); q_r_h_c_.setZero(HAND_DOF);
    q_l_h_t_.setZero(HAND_DOF); q_r_h_t_.setZero(HAND_DOF);

    f_l_c_.setZero(); f_r_c_.setZero();
    f_l_t_.setZero(); f_r_t_.setZero();

    f_l_h_c_.setZero(FINGER_FORCE_DOF); f_r_h_c_.setZero(FINGER_FORCE_DOF);
    f_l_h_t_.setZero(FINGER_FORCE_DOF); f_r_h_t_.setZero(FINGER_FORCE_DOF);

    print_timer_ = node_->create_wall_timer(500ms, std::bind(&DualArmForceControl::PrintDualArmStates, this));
    control_timer_ = node_->create_wall_timer(2ms, std::bind(&DualArmForceControl::ControlLoop, this));

    is_initialized_ = true;
}
DualArmForceControl::~DualArmForceControl() {}

void DualArmForceControl::ControlLoop() { if (!is_initialized_) return; }
