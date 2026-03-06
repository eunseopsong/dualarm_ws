#ifndef DUALARM_FORCE_CONTROL_H
#define DUALARM_FORCE_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"   // /isaac_contact_states + monitor topics
#include "std_msgs/msg/float64_multi_array.hpp"   // target topics
#include "std_srvs/srv/trigger.hpp"

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

    // Current-state callbacks
    void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void ArmPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void HandPositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Inverse targets
    void TargetArmPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TargetHandPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void DeltaArmPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void DeltaHandPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Forward targets
    void TargetArmJointsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void TargetHandJointsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Measured contact force callback (Isaac)
    void HandContactForceCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // Force-control command callback (forcecon mode)
    // /target_hand_force : [hand_id, finger_id, px, py, pz, fx, fy, fz]
    void TargetHandForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    void ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // Main loop / monitor
    void ControlLoop();
    void PrintDualArmStates();

    // NEW: publish current/target hand-force monitor topics for rqt_plot
    void PublishHandForceMonitor();

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr position_sub_;

    // Cartesian targets (inverse mode)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_arm_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_hand_pos_sub_;

    // Delta Cartesian targets (inverse mode)
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr delta_arm_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr delta_hand_pos_sub_;

    // Forward joint targets
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_arm_joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_hand_joint_sub_;

    // Force-control target topic
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_hand_force_sub_;

    // Contact states topic from Isaac
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr contact_force_sub_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;

    // NEW: monitor publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_force_current_monitor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_force_target_monitor_pub_;

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

    // Mode / init
    std::string current_control_mode_ = "idle";
    std::vector<std::string> joint_names_;
    bool is_initialized_ = false;
    bool idle_synced_ = false;

    // Joint states
    Eigen::VectorXd q_l_c_, q_r_c_, q_l_t_, q_r_t_;          // arm 6DoF
    Eigen::VectorXd q_l_h_c_, q_r_h_c_, q_l_h_t_, q_r_h_t_;  // hand 20DoF canonical (q4 mimic slot included)

    // Arm forces monitor (currently unused measured source => zero)
    Eigen::Vector3d f_l_c_{0,0,0}, f_r_c_{0,0,0};
    Eigen::Vector3d f_l_t_{0,0,0}, f_r_t_{0,0,0};

    // Hand forces (5x3, canonical rows: THMB,INDX,MIDL,RING,BABY)
    Eigen::Matrix<double,5,3> f_l_hand_c_;
    Eigen::Matrix<double,5,3> f_r_hand_c_;
    Eigen::Matrix<double,5,3> f_l_hand_t_;
    Eigen::Matrix<double,5,3> f_r_hand_t_;

    // ------------------------------------------------------------------------
    // v19 patch: latched forcecon command (callback stores, ControlLoop executes)
    // ------------------------------------------------------------------------
    bool hand_force_cmd_valid_{false};
    int  hand_force_cmd_hand_id_{0};     // 0=left, 1=right
    int  hand_force_cmd_finger_id_{3};   // canonical: 0..4
    Eigen::Vector3d hand_force_cmd_p_des_base_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d hand_force_cmd_f_des_base_{Eigen::Vector3d::Zero()};
    int64_t hand_force_cmd_stamp_ns_{0};

    // ------------------------------------------------------------------------
    // v19.1 patch: forcecon hold snapshot (freeze arm/wrist at forcecon entry)
    // ------------------------------------------------------------------------
    Eigen::VectorXd q_l_arm_forcecon_hold_;   // size 6
    Eigen::VectorXd q_r_arm_forcecon_hold_;   // size 6
    Eigen::VectorXd q_l_hand_forcecon_hold_;  // size 20
    Eigen::VectorXd q_r_hand_forcecon_hold_;  // size 20

    bool forcecon_hold_snapshot_valid_{false};
    bool forcecon_prev_cycle_{false};

    // Kinematics
    std::shared_ptr<ArmForwardKinematics> arm_fk_;
    std::shared_ptr<ArmInverseKinematics> arm_ik_l_, arm_ik_r_;

    std::shared_ptr<dualarm_forcecon::HandForwardKinematics> hand_fk_l_, hand_fk_r_;
    std::shared_ptr<dualarm_forcecon::HandInverseKinematics> hand_ik_l_, hand_ik_r_;

    // Per-finger hand admittance controllers
    // canonical order: 0 thumb,1 index,2 middle,3 ring,4 baby
    std::array<std::shared_ptr<dualarm_forcecon::HandAdmittanceControl>,5> hand_adm_l_;
    std::array<std::shared_ptr<dualarm_forcecon::HandAdmittanceControl>,5> hand_adm_r_;

    // Arm poses (world/base as configured by arm_fk_)
    geometry_msgs::msg::Pose current_pose_l_, current_pose_r_;
    geometry_msgs::msg::Pose target_pose_l_,  target_pose_r_;

    // Fingertip positions (HAND BASE FRAME)
    geometry_msgs::msg::Point f_l_thumb_, f_l_index_, f_l_middle_, f_l_ring_, f_l_baby_;
    geometry_msgs::msg::Point f_r_thumb_, f_r_index_, f_r_middle_, f_r_ring_, f_r_baby_;

    geometry_msgs::msg::Point t_f_l_thumb_, t_f_l_index_, t_f_l_middle_, t_f_l_ring_, t_f_l_baby_;
    geometry_msgs::msg::Point t_f_r_thumb_, t_f_r_index_, t_f_r_middle_, t_f_r_ring_, t_f_r_baby_;

    // hand contact debug buffers (CURRENT) : 5 fingers x (raw scalar / sensor vec / wrist vec)
    // row order = canonical (thumb,index,middle,ring,baby)
    Eigen::Matrix<double,5,1> raw_l_hand_contact_;
    Eigen::Matrix<double,5,1> raw_r_hand_contact_;

    Eigen::Matrix<double,5,3> f_l_hand_sensor_c_;
    Eigen::Matrix<double,5,3> f_r_hand_sensor_c_;

    Eigen::Matrix<double,5,3> f_l_hand_wrist_c_;
    Eigen::Matrix<double,5,3> f_r_hand_wrist_c_;
};

#endif