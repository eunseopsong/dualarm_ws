#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <array>

class DualArmForceControl : public std::enable_shared_from_this<DualArmForceControl>
{
public:
    DualArmForceControl(std::shared_ptr<rclcpp::Node> node);
    ~DualArmForceControl();

    // ========================================================================
    // Callbacks (states_callback_dualarm.cpp에 구현됨)
    // ========================================================================
    // [수정] Arm + Hand 전체 관절 콜백
    void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // Force/Torque Sensor 콜백
    // void ftSensorCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);

    // ========================================================================
    // Member Variables
    // ========================================================================
public:
    // 로봇 자유도 설정
    static const int ARM_DOF = 7;    // Arm: 7축
    static const int HAND_DOF = 15;  // Hand: 15축 (Thumb 3 + Index 3 + Middle 3 + Ring 3 + Pinky 3)

private:
    std::shared_ptr<rclcpp::Node> node_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr ftsensor_sub_;

    // ------------------------------------------------------------------------
    // Robot States
    // ------------------------------------------------------------------------
    // Arm Joint Positions
    Eigen::VectorXd q_left_;   // Size: ARM_DOF
    Eigen::VectorXd q_right_;  // Size: ARM_DOF

    // Hand Joint Positions
    Eigen::VectorXd q_left_hand_;  // Size: HAND_DOF
    Eigen::VectorXd q_right_hand_; // Size: HAND_DOF

    // Contact Forces (Wrench)
    Eigen::Vector3d teleop_force_; // Fx, Fy, Fz
    double contact_force;          // Force Magnitude or Z-force
    bool teleop_force_valid_ = false;

    // 기타 상태 변수들 (필요시 추가)
    bool is_initialized_ = false;
};

#endif // DUALARM_FORCE_CONTROL_H