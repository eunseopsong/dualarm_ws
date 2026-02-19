#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

#include <memory>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>

// Kinematics
#include "dualarm_forcecon/Kinematics/kinematics_utils.hpp"
#include "dualarm_forcecon/Kinematics/arm_forward_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/arm_inverse_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp"

class DualArmForceControl : public std::enable_shared_from_this<DualArmForceControl> {
public:
    DualArmForceControl(std::shared_ptr<rclcpp::Node> node);
    ~DualArmForceControl();

    // Callbacks
    void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void TargetPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // Main
    void ControlLoop();

    // Print
    void PrintDualArmStates();

private:
    // Node
    std::shared_ptr<rclcpp::Node> node_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr contact_force_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mode_service_;

    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Params
    std::string urdf_path_;
    std::array<double,3> world_base_xyz_{0.0, 0.0, 0.306};
    std::array<double,3> world_base_euler_xyz_deg_{0.0, 0.0, 0.0};

    std::string ik_targets_frame_{"base"};   // "base" or "world"
    std::string ik_euler_conv_{"rpy"};       // "rpy" or "xyz"
    std::string ik_angle_unit_{"rad"};       // "rad" or "deg" or "auto"

    // Mode
    std::string current_control_mode_{"idle"};
    bool is_initialized_{false};
    bool idle_synced_{false};

    // Joint names
    std::vector<std::string> joint_names_;

    // Arm joints
    Eigen::VectorXd q_l_c_, q_r_c_, q_l_t_, q_r_t_;

    // Hand joints (20)
    Eigen::VectorXd q_l_h_c_, q_r_h_c_, q_l_h_t_, q_r_h_t_;

    // Forces
    Eigen::Vector3d f_l_c_{0,0,0}, f_r_c_{0,0,0};

    // Kinematics objects
    std::shared_ptr<ArmForwardKinematics> arm_fk_;
    std::shared_ptr<ArmInverseKinematics> arm_ik_l_, arm_ik_r_;
    std::shared_ptr<HandForwardKinematics> hand_fk_l_, hand_fk_r_;

    // Current wrist pose (World frame) - FK 결과
    geometry_msgs::msg::Pose current_pose_l_, current_pose_r_;

    // Fingertip positions (World frame)
    geometry_msgs::msg::Point f_l_thumb_, f_l_index_, f_l_middle_, f_l_ring_, f_l_baby_;
    geometry_msgs::msg::Point f_r_thumb_, f_r_index_, f_r_middle_, f_r_ring_, f_r_baby_;
};

#endif
