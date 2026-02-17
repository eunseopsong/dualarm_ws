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
    // Callbacks (Implemented in src/states_callback_dualarm.cpp)
    // ========================================================================
    // 1. Arm + Hand Joint States Callback
    void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // 2. Contact Force Callback (Float64MultiArray)
    void ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // ========================================================================
    // Debugging Functions
    // ========================================================================
    // Print current and target states to terminal
    void PrintDualArmStates();

    // ========================================================================
    // Member Variables
    // ========================================================================
public:
    // Robot Degrees of Freedom
    static const int ARM_DOF = 7;    // Arm: 7 DOF
    static const int HAND_DOF = 15;  // Hand: 15 DOF (Aidin Hand)

private:
    std::shared_ptr<rclcpp::Node> node_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ftsensor_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr print_timer_; // For debug printing (0.5s)

    // ------------------------------------------------------------------------
    // Robot States (Current - Measured from sensors)
    // ------------------------------------------------------------------------
    // Arm Joint Positions
    Eigen::VectorXd q_left_;   // Size: ARM_DOF
    Eigen::VectorXd q_right_;  // Size: ARM_DOF

    // Hand Joint Positions
    Eigen::VectorXd q_left_hand_;  // Size: HAND_DOF
    Eigen::VectorXd q_right_hand_; // Size: HAND_DOF

    // Contact Forces (Measured)
    Eigen::Vector3d force_left_;   // Fx, Fy, Fz
    Eigen::Vector3d torque_left_;  // Tx, Ty, Tz
    
    Eigen::Vector3d force_right_;  // Fx, Fy, Fz
    Eigen::Vector3d torque_right_; // Tx, Ty, Tz

    // ------------------------------------------------------------------------
    // Robot States (Target - Desired / Command)
    // ------------------------------------------------------------------------
    Eigen::VectorXd qd_left_;       // Target Arm Left
    Eigen::VectorXd qd_right_;      // Target Arm Right
    Eigen::VectorXd qd_left_hand_;  // Target Hand Left
    Eigen::VectorXd qd_right_hand_; // Target Hand Right

    // System Status
    bool is_initialized_ = false;
};

#endif // DUALARM_FORCE_CONTROL_H