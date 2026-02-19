#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>
#include <vector>
#include <memory>

// Kinematics Headers
#include "dualarm_forcecon/Kinematics/arm_forward_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/arm_inverse_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp"

class DualArmForceControl : public std::enable_shared_from_this<DualArmForceControl> {
public:
    DualArmForceControl(std::shared_ptr<rclcpp::Node> node);
    ~DualArmForceControl();

    void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void TargetPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    void ControlLoop();
    void PrintDualArmStates();

private:
    // ROS2 Node & Interface
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr contact_force_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mode_service_;

    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // System States
    std::string current_control_mode_ = "idle";
    std::vector<std::string> joint_names_;
    bool is_initialized_ = false;
    bool idle_synced_ = false;

    // Joint Data (Arm & Hand)
    Eigen::VectorXd q_l_c_, q_r_c_, q_l_t_, q_r_t_;
    Eigen::VectorXd q_l_h_c_, q_r_h_c_, q_l_h_t_, q_r_h_t_;

    // Force Data
    Eigen::Vector3d f_l_c_, f_r_c_;

    // Kinematics Objects
    std::shared_ptr<ArmForwardKinematics> arm_fk_;
    std::shared_ptr<ArmInverseKinematics> arm_ik_l_, arm_ik_r_;
    std::shared_ptr<HandForwardKinematics> hand_fk_l_, hand_fk_r_;

    // Pose (우리는 "World 기준"으로 저장해서 Isaac UI와 동일 출력)
    geometry_msgs::msg::Pose current_pose_l_, current_pose_r_;

    // Fingertip Positions (World Frame)
    geometry_msgs::msg::Point f_l_thumb_, f_l_index_, f_l_middle_, f_l_ring_, f_l_baby_;
    geometry_msgs::msg::Point f_r_thumb_, f_r_index_, f_r_middle_, f_r_ring_, f_r_baby_;

    // World_T_base (Isaac UI world 기준 맞추기)
    Eigen::Affine3d T_world_base_ = Eigen::Affine3d::Identity();

    // IK 입력 옵션 (기본은 기존 호환: base + rpy(rad))
    std::string ik_targets_frame_ = "base";   // "base" or "world"
    std::string ik_euler_conv_    = "rpy";    // "rpy" or "xyz"
    std::string ik_angle_unit_    = "rad";    // "rad" or "deg" or "auto"

    // URDF path
    std::string urdf_path_;

    // Utility: base pose + relative pose => world point
    geometry_msgs::msg::Point combinePose(const geometry_msgs::msg::Pose& base,
                                          const geometry_msgs::msg::Pose& relative);

    // Utility: Isaac UI와 동일한 Euler XYZ(deg) 추출
    void quatToEulerXYZDeg(const geometry_msgs::msg::Quaternion& q,
                           double& ex_deg, double& ey_deg, double& ez_deg) const;

    // Utility: wrap deg to (-180, 180]
    static double wrapDeg(double a);
};

#endif // DUALARM_FORCE_CONTROL_H
