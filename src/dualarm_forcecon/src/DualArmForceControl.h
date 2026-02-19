#ifndef DUALARM_FORCECON__DUALARM_FORCE_CONTROL_HPP_
#define DUALARM_FORCECON__DUALARM_FORCE_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "dualarm_forcecon/Kinematics/arm_forward_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/arm_inverse_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp"

class DualArmForceControl : public std::enable_shared_from_this<DualArmForceControl>
{
public:
  explicit DualArmForceControl(const std::shared_ptr<rclcpp::Node>& node);
  ~DualArmForceControl();

  void ControlLoop();
  void PrintDualArmStates();

  // Callbacks
  void JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void TargetPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res);

private:
  enum class ControlMode { IDLE=0, FORWARD=1, INVERSE=2 };

  struct PubMapEntry {
    enum class Src { L_ARM, R_ARM, L_HAND, R_HAND, CONST0 };
    Src src{Src::CONST0};
    int idx{-1};
  };

  void build_publish_map_locked();  // joint_names_ 기반으로 pub_map_ 구축
  void update_fk_locked();          // 최신 q로 FK(팔/손) 계산해서 pose/point 갱신

  static int extract_trailing_int(const std::string& s);  // 문자열 끝 숫자 파싱
  static geometry_msgs::msg::Point combinePose(const geometry_msgs::msg::Pose& base,
                                               const geometry_msgs::msg::Pose& relative);

  static const char* mode_to_cstr(ControlMode m) {
    switch(m){
      case ControlMode::IDLE: return "idle";
      case ControlMode::FORWARD: return "forward";
      case ControlMode::INVERSE: return "inverse";
    }
    return "unknown";
  }

private:
  std::shared_ptr<rclcpp::Node> node_;

  // Params
  std::string urdf_path_;
  std::string base_link_;
  std::string left_tip_link_;
  std::string right_tip_link_;
  std::string left_hand_base_link_;
  std::string right_hand_base_link_;
  std::vector<std::string> fingertip_links_;

  std::string joint_states_topic_;
  std::string joint_cmd_topic_;
  std::string contact_topic_;
  std::string target_pose_topic_;
  std::string forward_joint_topic_;
  std::string mode_service_name_;

  double control_hz_{100.0};
  double fk_hz_{50.0};
  double print_hz_{2.0};

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr contact_force_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr mode_service_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr fk_timer_;
  rclcpp::TimerBase::SharedPtr print_timer_;

  // Thread-safe shared state
  std::mutex mtx_;
  bool is_initialized_{false};
  bool idle_synced_{false};
  ControlMode mode_{ControlMode::IDLE};

  std::vector<std::string> joint_names_;
  std::vector<PubMapEntry> pub_map_;

  // Arm/Hand q (curr/target)
  Eigen::VectorXd q_l_c_, q_r_c_, q_l_t_, q_r_t_;
  Eigen::VectorXd q_l_h_c_, q_r_h_c_, q_l_h_t_, q_r_h_t_;

  // Forces
  Eigen::Vector3d f_l_c_{0,0,0}, f_r_c_{0,0,0};

  // Kinematics
  std::shared_ptr<ArmForwardKinematics> arm_fk_;
  std::shared_ptr<ArmInverseKinematics> arm_ik_l_, arm_ik_r_;
  std::shared_ptr<HandForwardKinematics> hand_fk_l_, hand_fk_r_;

  // FK results
  geometry_msgs::msg::Pose current_pose_l_, current_pose_r_;
  geometry_msgs::msg::Point f_l_thumb_, f_l_index_, f_l_middle_, f_l_ring_, f_l_baby_;
  geometry_msgs::msg::Point f_r_thumb_, f_r_index_, f_r_middle_, f_r_ring_, f_r_baby_;
};

#endif
