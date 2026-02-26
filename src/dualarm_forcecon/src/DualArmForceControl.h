#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"   // /isaac_contact_states 용
#include "std_msgs/msg/float64_multi_array.hpp"   // arm/hand target 토픽들 용
#include "std_srvs/srv/trigger.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>
#include <memory>
#include <array>

// include/ kinematics
#include "dualarm_forcecon/Kinematics/arm_forward_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/arm_inverse_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_inverse_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/kinematics_utils.hpp"

class DualArmForceControl : public std::enable_shared_from_this<DualArmForceControl> {
public:
    DualArmForceControl(std::shared_ptr<rclcpp::Node> node);
    ~DualArmForceControl();

    // Callbacks
    void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    void ArmPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void HandPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Cartesian target callbacks (inverse mode)
    void TargetArmPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TargetHandPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // v15: Delta arm Cartesian target callback (inverse mode)
    // msg: 12 = [L dx dy dz droll dpitch dyaw, R dx dy dz droll dpitch dyaw]
    void DeltaArmPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Forward joint target callbacks (split)
    void TargetArmJointsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TargetHandJointsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Contact force callback uses Float32MultiArray (Isaac ActionGraph)
    void HandContactForceCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // Main
    void ControlLoop();
    void PrintDualArmStates();

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_sub_;

    // Cartesian targets (inverse mode)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_arm_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_hand_pos_sub_;

    // v15: Delta Cartesian target (inverse mode)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr delta_arm_pos_sub_;

    // Forward joint targets (split)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_arm_joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_hand_joint_sub_;

    // Contact states topic from Isaac ActionGraph (Float32MultiArray)
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr contact_force_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mode_service_;

    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Params
    std::string urdf_path_;
    std::array<double,3> world_base_xyz_{0.0, 0.0, 0.306};
    std::array<double,3> world_base_euler_xyz_deg_{0.0, 0.0, 0.0};

    std::string ik_targets_frame_ = "base";
    std::string ik_euler_conv_    = "rpy";
    std::string ik_angle_unit_    = "rad";

    // mode
    std::string current_control_mode_ = "idle";
    std::vector<std::string> joint_names_;
    bool is_initialized_ = false;
    bool idle_synced_ = false;

    // joint states
    Eigen::VectorXd q_l_c_, q_r_c_, q_l_t_, q_r_t_;          // 6
    Eigen::VectorXd q_l_h_c_, q_r_h_c_, q_l_h_t_, q_r_h_t_;  // 20

    // forces (arm)
    Eigen::Vector3d f_l_c_{0,0,0}, f_r_c_{0,0,0};
    Eigen::Vector3d f_l_t_{0,0,0}, f_r_t_{0,0,0};

    // forces (hand) : 5 fingers x 3
    Eigen::Matrix<double,5,3> f_l_hand_c_;
    Eigen::Matrix<double,5,3> f_r_hand_c_;
    Eigen::Matrix<double,5,3> f_l_hand_t_;
    Eigen::Matrix<double,5,3> f_r_hand_t_;

    // kinematics
    std::shared_ptr<ArmForwardKinematics> arm_fk_;
    std::shared_ptr<ArmInverseKinematics> arm_ik_l_, arm_ik_r_;

    std::shared_ptr<dualarm_forcecon::HandForwardKinematics> hand_fk_l_, hand_fk_r_;
    std::shared_ptr<dualarm_forcecon::HandInverseKinematics> hand_ik_l_, hand_ik_r_;

    // poses (arm world/base pose as configured by arm_fk_ default frame)
    geometry_msgs::msg::Pose current_pose_l_, current_pose_r_;
    geometry_msgs::msg::Pose target_pose_l_,  target_pose_r_;

    // fingertip position (HAND BASE FRAME)
    geometry_msgs::msg::Point f_l_thumb_, f_l_index_, f_l_middle_, f_l_ring_, f_l_baby_;
    geometry_msgs::msg::Point f_r_thumb_, f_r_index_, f_r_middle_, f_r_ring_, f_r_baby_;

    geometry_msgs::msg::Point t_f_l_thumb_, t_f_l_index_, t_f_l_middle_, t_f_l_ring_, t_f_l_baby_;
    geometry_msgs::msg::Point t_f_r_thumb_, t_f_r_index_, t_f_r_middle_, t_f_r_ring_, t_f_r_baby_;
};

#endif