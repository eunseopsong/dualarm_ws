#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

// ROS2 core
#include <rclcpp/rclcpp.hpp>

// ROS msg (나중에 쓸 것들 미리 include)
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench.hpp>

// std
#include <memory>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Dense>

class DualArmForceControl
{
public:
    // 생성자에서 Node 포인터를 받습니다.
    explicit DualArmForceControl(const rclcpp::Node::SharedPtr& node);
    ~DualArmForceControl();

    // 메인 루프 함수 (껍데기)
    void CalculateAndPublishJoint();

private:
    // ROS Node 핸들
    rclcpp::Node::SharedPtr node_;

    // 상태 변수 예시
    bool running_ = true;
};

#endif // DUALARM_FORCE_CONTROL_H