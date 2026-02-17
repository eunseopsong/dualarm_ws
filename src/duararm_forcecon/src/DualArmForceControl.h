#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>

class DualArmForceControl : public std::enable_shared_from_this<DualArmForceControl> {
public:
    DualArmForceControl(std::shared_ptr<rclcpp::Node> node);
    ~DualArmForceControl();

    void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void ControlLoop();
    void PrintDualArmStates();

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr contact_force_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_joint_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mode_service_;
    
    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::string current_control_mode_ = "idle";
    std::vector<std::string> joint_names_; // Isaac Sim 조인트 이름 저장용

    // Arm & Hand Vectors
    Eigen::VectorXd q_l_c_, q_r_c_, q_l_t_, q_r_t_;
    Eigen::VectorXd q_l_h_c_, q_r_h_c_, q_l_h_t_, q_r_h_t_;
    Eigen::Vector3d f_l_c_, f_r_c_, f_l_t_, f_r_t_;
    Eigen::VectorXd f_l_h_c_, f_r_h_c_, f_l_h_t_, f_r_h_t_;

    bool is_initialized_ = false;
};
#endif