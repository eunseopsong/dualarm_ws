#include "DualArmForceControl.h"

// 생성자
DualArmForceControl::DualArmForceControl(const rclcpp::Node::SharedPtr& node)
: node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "DualArmForceControl Class Created.");
    
    // 나중에 여기에 Subscriber/Publisher 생성 코드가 들어갑니다.
}

// 소멸자
DualArmForceControl::~DualArmForceControl()
{
    RCLCPP_INFO(node_->get_logger(), "DualArmForceControl Class Destroyed.");
}

// 메인 루프 (껍데기)
void DualArmForceControl::CalculateAndPublishJoint()
{
    if (!running_) return;

    // 나중에 여기에 제어 로직이 들어갑니다.
    // RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Control Loop Running...");
}