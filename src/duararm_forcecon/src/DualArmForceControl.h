#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

class DualArmForceControl : public std::enable_shared_from_this<DualArmForceControl>
{
public:
    DualArmForceControl(std::shared_ptr<rclcpp::Node> node);
    ~DualArmForceControl();

    void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void ControlLoop();         // 500Hz
    void PrintDualArmStates();  // 2Hz

public:
    // Isaac Sim 데이터 규격 반영
    static const int ARM_DOF = 6;           // J1~J6
    static const int HAND_DOF = 20;          // 5 fingers * 4 joints
    static const int FORCE_DIM = 3;          // Fx, Fy, Fz (Moment 제외)
    static const int FINGER_FORCE_DOF = 15;  // 5 fingers * 3-axis Force

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ftsensor_sub_;
    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Current Values (Cyan)
    Eigen::VectorXd q_l_c_, q_r_c_;
    Eigen::VectorXd q_l_h_c_, q_r_h_c_;
    Eigen::Vector3d f_l_c_, f_r_c_;
    Eigen::VectorXd f_l_h_c_, f_r_h_c_;

    // Target Values (Yellow)
    Eigen::VectorXd q_l_t_, q_r_t_;
    Eigen::VectorXd q_l_h_t_, q_r_h_t_;
    Eigen::Vector3d f_l_t_, f_r_t_;
    Eigen::VectorXd f_l_h_t_, f_r_h_t_;

    bool is_initialized_ = false;
};

#endif