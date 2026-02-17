#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp" // [수정] Wrench 대신 사용
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

class DualArmForceControl : public std::enable_shared_from_this<DualArmForceControl>
{
public:
    DualArmForceControl(std::shared_ptr<rclcpp::Node> node);
    ~DualArmForceControl();

    // ========================================================================
    // Callbacks (states_callback_dualarm.cpp에 구현)
    // ========================================================================
    // Arm + Hand 전체 관절 콜백
    void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // [수정] Contact Force 콜백 (Float64MultiArray)
    void ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // ========================================================================
    // Member Variables
    // ========================================================================
public:
    // 로봇 자유도
    static const int ARM_DOF = 7;
    static const int HAND_DOF = 15;

private:
    std::shared_ptr<rclcpp::Node> node_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ftsensor_sub_; // [수정] 타입 변경

    // ------------------------------------------------------------------------
    // Robot States
    // ------------------------------------------------------------------------
    // Arm Joint Positions
    Eigen::VectorXd q_left_;
    Eigen::VectorXd q_right_;

    // Hand Joint Positions
    Eigen::VectorXd q_left_hand_;
    Eigen::VectorXd q_right_hand_;

    // Contact Forces (Left)
    Eigen::Vector3d force_left_;   // Fx, Fy, Fz
    Eigen::Vector3d torque_left_;  // Tx, Ty, Tz

    // Contact Forces (Right)
    Eigen::Vector3d force_right_;  // Fx, Fy, Fz
    Eigen::Vector3d torque_right_; // Tx, Ty, Tz

    bool is_initialized_ = false;
};

#endif // DUALARM_FORCE_CONTROL_H