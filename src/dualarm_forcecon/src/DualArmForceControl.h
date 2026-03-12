#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "dualarm_forcecon_interfaces/srv/set_control_mode.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>

#include <string>
#include <vector>
#include <memory>
#include <array>
#include <cstdint>

// include/ kinematics
#include "dualarm_forcecon/Kinematics/arm_forward_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/arm_inverse_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_inverse_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_admittance_control.hpp"
#include "dualarm_forcecon/Kinematics/kinematics_utils.hpp"

class DualArmForceControl : public std::enable_shared_from_this<DualArmForceControl> {
public:
    DualArmForceControl(std::shared_ptr<rclcpp::Node> node);
    ~DualArmForceControl();

    // ------------------------------------------------------------------------
    // Current-state callbacks
    // ------------------------------------------------------------------------
    void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ArmPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void HandPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // ------------------------------------------------------------------------
    // Inverse targets
    // ------------------------------------------------------------------------
    void TargetArmPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TargetHandPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void DeltaArmPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void DeltaHandPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // ------------------------------------------------------------------------
    // Forward targets
    // ------------------------------------------------------------------------
    void TargetArmJointsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TargetHandJointsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // ------------------------------------------------------------------------
    // Measured contact force callback (Isaac)
    // ------------------------------------------------------------------------
    void HandContactForceCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // ------------------------------------------------------------------------
    // Desired-force reference callback for unified hand admittance
    //
    // Supported:
    //   [hand_id, finger_id, fx, fy, fz]
    //   [hand_id, finger_id, px, py, pz, fx, fy, fz]  // legacy, p ignored
    //
    // hand_id : 0=left, 1=right
    // finger_id canonical order:
    //   0 thumb, 1 index, 2 middle, 3 ring, 4 baby
    // ------------------------------------------------------------------------
    void TargetHandForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void ControlModeCallback(
        const std::shared_ptr<dualarm_forcecon_interfaces::srv::SetControlMode::Request> req,
        std::shared_ptr<dualarm_forcecon_interfaces::srv::SetControlMode::Response> res);

    // ------------------------------------------------------------------------
    // Main loop / monitor
    // ------------------------------------------------------------------------
    void ControlLoop();
    void PrintDualArmStates();

    // Publish current/target hand-force monitor topics for rqt_plot
    void PublishHandForceMonitor();

private:
    std::shared_ptr<rclcpp::Node> node_;

    // ------------------------------------------------------------------------
    // ROS interfaces
    // ------------------------------------------------------------------------
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_sub_;

    // Cartesian targets
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_arm_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_hand_pos_sub_;

    // Delta Cartesian targets
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr delta_arm_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr delta_hand_pos_sub_;

    // Forward joint targets
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_arm_joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_hand_joint_sub_;

    // Desired-force reference topic
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_hand_force_sub_;

    // Contact states topic from Isaac
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr contact_force_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;

    // Monitor publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_force_current_monitor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_force_target_monitor_pub_;

    rclcpp::Service<dualarm_forcecon_interfaces::srv::SetControlMode>::SharedPtr mode_service_;

    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ------------------------------------------------------------------------
    // Params
    // ------------------------------------------------------------------------
    std::string urdf_path_;
    std::array<double,3> world_base_xyz_{0.0, 0.0, 0.306};
    std::array<double,3> world_base_euler_xyz_deg_{0.0, 0.0, 0.0};

    std::string ik_targets_frame_ = "base";
    std::string ik_euler_conv_    = "rpy";
    std::string ik_angle_unit_    = "rad";

    // ------------------------------------------------------------------------
    // Mode / init
    // ------------------------------------------------------------------------
    std::string current_arm_control_mode_  = "idle";   // idle / forward / inverse
    std::string current_hand_control_mode_ = "idle";   // idle / forward / inverse

    std::vector<std::string> joint_names_;
    bool is_initialized_ = false;

    bool arm_idle_synced_  = false;
    bool hand_idle_synced_ = false;

    // ------------------------------------------------------------------------
    // Joint states
    // ------------------------------------------------------------------------
    // arm 6DoF current / final cmd
    Eigen::VectorXd q_l_c_, q_r_c_, q_l_t_, q_r_t_;

    // hand 20DoF current / final cmd
    Eigen::VectorXd q_l_h_c_, q_r_h_c_, q_l_h_t_, q_r_h_t_;

    // hand motion-reference targets (before unified admittance)
    // - forward mode authoritative source
    // - inverse mode에서도 seed / reference로 사용 가능
    Eigen::VectorXd q_l_h_motion_t_, q_r_h_motion_t_;

    // ------------------------------------------------------------------------
    // Arm forces monitor (currently unused measured source => zero)
    // ------------------------------------------------------------------------
    Eigen::Vector3d f_l_c_{0,0,0}, f_r_c_{0,0,0};
    Eigen::Vector3d f_l_t_{0,0,0}, f_r_t_{0,0,0};

    // ------------------------------------------------------------------------
    // Hand force states (canonical row order: THMB, INDX, MIDL, RING, BABY)
    // ------------------------------------------------------------------------
    // current measured/contact force in HAND BASE frame
    Eigen::Matrix<double,5,3> f_l_hand_c_;
    Eigen::Matrix<double,5,3> f_r_hand_c_;

    // desired target force in HAND BASE frame
    Eigen::Matrix<double,5,3> f_l_hand_t_;
    Eigen::Matrix<double,5,3> f_r_hand_t_;

    // ------------------------------------------------------------------------
    // Hand Cartesian states (canonical row order)
    //
    // x_* naming:
    //   c   : current measured fingertip position
    //   d   : desired target position
    //   ref : reference position used by admittance after tangent-anchor logic
    //   cmd : final Cartesian command output from admittance
    // ------------------------------------------------------------------------
    Eigen::Matrix<double,5,3> x_l_hand_c_;
    Eigen::Matrix<double,5,3> x_r_hand_c_;

    Eigen::Matrix<double,5,3> x_l_hand_d_;
    Eigen::Matrix<double,5,3> x_r_hand_d_;

    Eigen::Matrix<double,5,3> x_l_hand_ref_;
    Eigen::Matrix<double,5,3> x_r_hand_ref_;

    Eigen::Matrix<double,5,3> x_l_hand_cmd_;
    Eigen::Matrix<double,5,3> x_r_hand_cmd_;

    // Effective stiffness used by unified hand admittance (for debug/monitor)
    Eigen::Matrix<double,5,3> k_l_hand_eff_;
    Eigen::Matrix<double,5,3> k_r_hand_eff_;

    // Cartesian target cache initialization flags (primarily for inverse mode)
    bool hand_cartesian_target_l_initialized_{false};
    bool hand_cartesian_target_r_initialized_{false};

    // ------------------------------------------------------------------------
    // Optional current fingertip rotation cache
    // R_base_tip for each fingertip (canonical order)
    // ------------------------------------------------------------------------
    std::array<Eigen::Matrix3d,5> R_l_base_tip_c_;
    std::array<Eigen::Matrix3d,5> R_r_base_tip_c_;

    // ------------------------------------------------------------------------
    // Legacy single-active-finger desired-force latch
    //
    // NOTE:
    //   v25-style unified pipeline should treat f_l_hand_t_ / f_r_hand_t_ as the
    //   authoritative desired-force state. These legacy members are kept only
    //   temporarily so intermediate src patches can compile during migration.
    // ------------------------------------------------------------------------
    bool hand_force_cmd_valid_{false};
    int  hand_force_cmd_hand_id_{0};     // 0=left, 1=right
    int  hand_force_cmd_finger_id_{3};   // canonical: 0..4
    Eigen::Vector3d hand_force_cmd_f_des_base_{Eigen::Vector3d::Zero()};
    int64_t hand_force_cmd_stamp_ns_{0};

    // ------------------------------------------------------------------------
    // delta-arm command base pose latch
    // - latched once from current_pose_l_/r_ after node start
    // - DeltaArmPositionCallback uses:
    //     target = latched_base_pose + delta
    // ------------------------------------------------------------------------
    bool delta_arm_base_pose_initialized_{false};
    geometry_msgs::msg::Pose delta_arm_base_pose_l_;
    geometry_msgs::msg::Pose delta_arm_base_pose_r_;

    // ------------------------------------------------------------------------
    // Kinematics
    // ------------------------------------------------------------------------
    std::shared_ptr<ArmForwardKinematics> arm_fk_;
    std::shared_ptr<ArmInverseKinematics> arm_ik_l_, arm_ik_r_;

    std::shared_ptr<dualarm_forcecon::HandForwardKinematics> hand_fk_l_, hand_fk_r_;
    std::shared_ptr<dualarm_forcecon::HandInverseKinematics> hand_ik_l_, hand_ik_r_;

    // Per-finger hand admittance controllers
    // canonical order: 0 thumb,1 index,2 middle,3 ring,4 baby
    std::array<std::shared_ptr<dualarm_forcecon::HandAdmittanceControl>,5> hand_adm_l_;
    std::array<std::shared_ptr<dualarm_forcecon::HandAdmittanceControl>,5> hand_adm_r_;

    // ------------------------------------------------------------------------
    // Arm poses (world/base as configured by arm_fk_)
    // ------------------------------------------------------------------------
    geometry_msgs::msg::Pose current_pose_l_, current_pose_r_;
    geometry_msgs::msg::Pose target_pose_l_,  target_pose_r_;

    // ------------------------------------------------------------------------
    // Fingertip monitor points (HAND BASE FRAME)
    //
    // current fingertip points
    // ------------------------------------------------------------------------
    geometry_msgs::msg::Point f_l_thumb_, f_l_index_, f_l_middle_, f_l_ring_, f_l_baby_;
    geometry_msgs::msg::Point f_r_thumb_, f_r_index_, f_r_middle_, f_r_ring_, f_r_baby_;

    // target fingertip points
    geometry_msgs::msg::Point t_f_l_thumb_, t_f_l_index_, t_f_l_middle_, t_f_l_ring_, t_f_l_baby_;
    geometry_msgs::msg::Point t_f_r_thumb_, t_f_r_index_, t_f_r_middle_, t_f_r_ring_, t_f_r_baby_;

    // command fingertip points (after admittance, optional for future monitor extension)
    geometry_msgs::msg::Point c_f_l_thumb_, c_f_l_index_, c_f_l_middle_, c_f_l_ring_, c_f_l_baby_;
    geometry_msgs::msg::Point c_f_r_thumb_, c_f_r_index_, c_f_r_middle_, c_f_r_ring_, c_f_r_baby_;

    // ------------------------------------------------------------------------
    // Hand contact debug buffers (CURRENT)
    // row order = canonical (thumb,index,middle,ring,baby)
    // ------------------------------------------------------------------------
    Eigen::Matrix<double,5,1> raw_l_hand_contact_;
    Eigen::Matrix<double,5,1> raw_r_hand_contact_;

    // reconstructed force in raw sensor frame
    Eigen::Matrix<double,5,3> f_l_hand_sensor_c_;
    Eigen::Matrix<double,5,3> f_r_hand_sensor_c_;

    // converted force in wrist / hand-base-aligned frame
    Eigen::Matrix<double,5,3> f_l_hand_wrist_c_;
    Eigen::Matrix<double,5,3> f_r_hand_wrist_c_;
};

#endif