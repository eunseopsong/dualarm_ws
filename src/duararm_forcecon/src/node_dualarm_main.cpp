#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

// 방금 만든 헤더 include
#include "DualArmForceControl.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("dualarm_control_node");

    // 클래스 객체 생성
    auto dualarm_ctrl = std::make_shared<DualArmForceControl>(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // RCLCPP_INFO(node->get_logger(), "DualArm Control Node Started.");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}