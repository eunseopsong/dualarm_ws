#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "dualarm_forcecon_interfaces/srv/set_control_mode.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>


#include <Eigen/Dense>

#include <string>
#include <vector>
#include <memory>
#include <array>
#include <cstdint>
#include <unordered_map>

// include/ kinematics
#include "dualarm_kinematics/arm/arm_forward_kinematics.hpp"
#include "dualarm_kinematics/arm/arm_inverse_kinematics.hpp"
#include "dualarm_kinematics/hand/hand_forward_kinematics.hpp"
#include "dualarm_kinematics/hand/hand_inverse_kinematics.hpp"
#include "dualarm_forcecon/control/hand_admittance_control.hpp"
#include "dualarm_kinematics/core/kinematics_utils.hpp"
#include "dualarm_kinematics/core/kinematics_config.hpp"

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
    void TargetAuxJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void TargetBaseVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // ------------------------------------------------------------------------
    // Measured hand force callback (Isaac)
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
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_aux_joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_base_velocity_sub_;

    // Desired-force reference topic
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_hand_force_sub_;

    // AIDIN hand force topic from Isaac: left 5 fingers then right 5 fingers,
    // each finger ordered Fx, Fy, Fz.
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hand_force_xyz_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;

    // Monitor publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_force_current_monitor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_force_target_monitor_pub_;

    rclcpp::Service<dualarm_forcecon_interfaces::srv::SetControlMode>::SharedPtr mode_service_;

    rclcpp::TimerBase::SharedPtr print_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    double control_loop_period_s_ = 0.005;

    // ------------------------------------------------------------------------
    // Params
    // ------------------------------------------------------------------------
    std::string urdf_path_;
    std::array<double,3> world_base_xyz_{0.0, 0.0, 0.306};
    std::array<double,3> world_base_euler_xyz_deg_{0.0, 0.0, 0.0};

    std::string ik_targets_frame_ = "base";
    std::string ik_euler_conv_    = "rpy";
    std::string ik_angle_unit_    = "rad";

    // YAML-selectable robot kinematics profile/link/joint mapping
    dualarm_forcecon::DualArmKinematicsConfig kin_cfg_;

    // Latest observed JointState values. Unknown/non-controlled joints are
    // republished as hold-current instead of being forced to zero.
    std::unordered_map<std::string, double> last_joint_position_;

    // RBY1 startup hold:
    // lock wheel / torso / head / finger joints to the first observed JointState.
    bool startup_fixed_joint_hold_enabled_ = false;

    // Hand runtime can be enabled even when the selected robot YAML has
    // hand.enabled=false. This is needed for RBY1 + external hand teleop, where
    // the arm kinematics profile is arm-only but the old v25 hand admittance
    // pipeline should still run for hand forward/inverse modes.
    bool hand_runtime_enabled_ = false;
    bool startup_fixed_joint_hold_latched_ = false;
    std::unordered_map<std::string, double> startup_fixed_joint_position_;

    bool startup_fixed_hand_hold_enabled_ = false;
    bool startup_fixed_hand_hold_latched_ = false;
    Eigen::VectorXd q_l_h_fixed_, q_r_h_fixed_;

    // RBY1 startup arm home hold:
    // latch the first observed arm JointState at node startup and keep publishing
    // this home joint target while arm inverse mode is active but no explicit
    // Cartesian target/delta command has arrived yet.
    bool startup_arm_home_hold_enabled_ = false;
    bool startup_arm_home_hold_latched_ = false;
    Eigen::VectorXd q_l_arm_home_hold_, q_r_arm_home_hold_;

    // RBY1 arm command safety. The RBY1 arm is redundant, so pose IK can
    // occasionally return a far-away but mathematically valid joint solution.
    // Isaac receives absolute joint-position commands, therefore final arm
    // commands are slew-limited before publishing.
    bool rby1_arm_cmd_slew_initialized_ = false;
    Eigen::VectorXd q_l_arm_cmd_prev_, q_r_arm_cmd_prev_;
    double rby1_arm_max_cmd_step_rad_ = 0.006;
    double rby1_arm_servo_max_cart_step_m_ = 0.0008;

    // RBY1 non-arm command extension.
    // /forward_aux_joint_targets accepts JointState commands:
    // - wheel joints use velocity
    // - torso joints use position, optional velocity as max position speed
    std::unordered_map<std::string, double> aux_joint_position_command_;
    std::unordered_map<std::string, double> aux_joint_position_max_speed_command_;
    std::unordered_map<std::string, double> aux_joint_velocity_command_;
    rclcpp::Time aux_joint_velocity_stamp_;
    double aux_joint_velocity_timeout_sec_ = 0.5;

    // RBY1 mobile-base command extension.
    // /cmd_vel is converted to left_wheel/right_wheel velocity commands while
    // the normal consolidated command continues holding torso/head/arms/hands.
    double cmd_vel_linear_x_ = 0.0;
    double cmd_vel_angular_z_ = 0.0;
    rclcpp::Time cmd_vel_stamp_;
    double cmd_vel_timeout_sec_ = 0.3;
    double wheel_radius_m_ = 0.1;
    double wheel_base_m_ = 0.53;
    double max_wheel_speed_rad_s_ = 10.0;
    double torso_position_max_speed_rad_s_ = 2.0;
    bool invert_wheel_velocity_command_ = true;
    bool torso_upright_hold_enabled_ = true;
    bool torso_upright_hold_during_wheel_only_ = true;
    std::unordered_map<std::string, double> torso_upright_position_command_;

    // ------------------------------------------------------------------------
    // Mode / init
    // ------------------------------------------------------------------------
    std::string current_arm_control_mode_  = "idle";   // idle / forward / inverse
    std::string current_hand_control_mode_ = "idle";   // idle / forward / inverse

    std::vector<std::string> joint_names_;
    bool is_initialized_ = false;

    bool arm_idle_synced_  = false;
    bool hand_idle_synced_ = false;

    // RBY1 arm inverse servo: target pose is stored by callbacks and
    // incrementally tracked in ControlLoop() using pose-constrained DLS.
    // false means no explicit /target_arm_cartesian_pose or
    // /delta_arm_cartesian_pose command has arrived yet. In that state,
    // RBY1 keeps publishing the latched startup home joint command.
    bool rby1_arm_target_active_ = false;

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
