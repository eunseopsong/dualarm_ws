#include "DualArmForceControl.h"

using namespace std::chrono_literals;

DualArmForceControl::DualArmForceControl(const std::shared_ptr<rclcpp::Node>& node)
: node_(node)
{
  // -------------------------
  // Parameters
  // -------------------------
  urdf_path_ = node_->declare_parameter<std::string>(
    "urdf_path",
    "/home/eunseop/isaac/isaac_save/dualarm/dualarm_description/urdf/aidin_dsr_dualarm.urdf"
  );
  base_link_ = node_->declare_parameter<std::string>("base_link", "base_link");
  left_tip_link_  = node_->declare_parameter<std::string>("left_tip_link",  "left_link_6");
  right_tip_link_ = node_->declare_parameter<std::string>("right_tip_link", "right_link_6");
  left_hand_base_link_  = node_->declare_parameter<std::string>("left_hand_base_link",  "left_hand_base_link");
  right_hand_base_link_ = node_->declare_parameter<std::string>("right_hand_base_link", "right_hand_base_link");

  fingertip_links_ = node_->declare_parameter<std::vector<std::string>>(
    "fingertip_links",
    std::vector<std::string>{"link4_thumb","link4_index","link4_middle","link4_ring","link4_baby"}
  );

  joint_states_topic_   = node_->declare_parameter<std::string>("joint_states_topic",   "/isaac_joint_states");
  joint_cmd_topic_      = node_->declare_parameter<std::string>("joint_cmd_topic",      "/isaac_joint_command");
  contact_topic_        = node_->declare_parameter<std::string>("contact_topic",        "/isaac_contact_states");
  target_pose_topic_    = node_->declare_parameter<std::string>("target_pose_topic",    "/target_cartesian_pose");
  forward_joint_topic_  = node_->declare_parameter<std::string>("forward_joint_topic",  "/forward_joint_targets");
  mode_service_name_    = node_->declare_parameter<std::string>("mode_service_name",    "/change_control_mode");

  control_hz_ = node_->declare_parameter<double>("control_hz", 100.0);
  fk_hz_      = node_->declare_parameter<double>("fk_hz", 50.0);
  print_hz_   = node_->declare_parameter<double>("print_hz", 2.0);

  // -------------------------
  // QoS
  // -------------------------
  auto qos_state = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  auto qos_cmd   = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // -------------------------
  // ROS Interfaces
  // -------------------------
  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    joint_states_topic_, qos_state,
    std::bind(&DualArmForceControl::JointsCallback, this, std::placeholders::_1)
  );

  target_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    target_pose_topic_, qos_cmd,
    std::bind(&DualArmForceControl::TargetPositionCallback, this, std::placeholders::_1)
  );

  target_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    forward_joint_topic_, qos_cmd,
    std::bind(&DualArmForceControl::TargetJointCallback, this, std::placeholders::_1)
  );

  contact_force_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    contact_topic_, qos_state,
    std::bind(&DualArmForceControl::ContactForceCallback, this, std::placeholders::_1)
  );

  joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(joint_cmd_topic_, qos_cmd);

  mode_service_ = node_->create_service<std_srvs::srv::Trigger>(
    mode_service_name_,
    std::bind(&DualArmForceControl::ControlModeCallback, this, std::placeholders::_1, std::placeholders::_2)
  );

  // -------------------------
  // State init
  // -------------------------
  q_l_c_.setZero(6); q_r_c_.setZero(6); q_l_t_.setZero(6); q_r_t_.setZero(6);
  q_l_h_c_.setZero(20); q_r_h_c_.setZero(20); q_l_h_t_.setZero(20); q_r_h_t_.setZero(20);

  // -------------------------
  // Kinematics init
  // -------------------------
  arm_fk_  = std::make_shared<ArmForwardKinematics>(urdf_path_, base_link_, left_tip_link_, right_tip_link_);
  arm_ik_l_ = std::make_shared<ArmInverseKinematics>(urdf_path_, base_link_, left_tip_link_);
  arm_ik_r_ = std::make_shared<ArmInverseKinematics>(urdf_path_, base_link_, right_tip_link_);

  hand_fk_l_ = std::make_shared<HandForwardKinematics>(urdf_path_, left_hand_base_link_, fingertip_links_);
  hand_fk_r_ = std::make_shared<HandForwardKinematics>(urdf_path_, right_hand_base_link_, fingertip_links_);

  // -------------------------
  // Timers
  // -------------------------
  const auto control_period = std::chrono::duration<double>(1.0 / std::max(1.0, control_hz_));
  const auto fk_period      = std::chrono::duration<double>(1.0 / std::max(1.0, fk_hz_));
  const auto print_period   = std::chrono::duration<double>(1.0 / std::max(0.1, print_hz_));

  control_timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(control_period),
    std::bind(&DualArmForceControl::ControlLoop, this)
  );

  fk_timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(fk_period),
    [this](){
      std::lock_guard<std::mutex> lk(mtx_);
      if (!is_initialized_) return;
      update_fk_locked();
    }
  );

  print_timer_ = node_->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(print_period),
    std::bind(&DualArmForceControl::PrintDualArmStates, this)
  );

  RCLCPP_INFO(node_->get_logger(), "DualArmForceControl v5.1 initialized. mode=idle");
}

DualArmForceControl::~DualArmForceControl() = default;

int DualArmForceControl::extract_trailing_int(const std::string& s)
{
  int i = static_cast<int>(s.size()) - 1;
  while (i >= 0 && std::isdigit(static_cast<unsigned char>(s[i]))) i--;
  if (i == static_cast<int>(s.size()) - 1) return -1; // no trailing digits
  try {
    return std::stoi(s.substr(i + 1));
  } catch (...) {
    return -1;
  }
}

geometry_msgs::msg::Point DualArmForceControl::combinePose(const geometry_msgs::msg::Pose& base,
                                                           const geometry_msgs::msg::Pose& relative)
{
  Eigen::Translation3d t_b(base.position.x, base.position.y, base.position.z);
  Eigen::Quaterniond q_b(base.orientation.w, base.orientation.x, base.orientation.y, base.orientation.z);
  Eigen::Affine3d T_world_base = t_b * q_b;

  Eigen::Translation3d t_r(relative.position.x, relative.position.y, relative.position.z);
  Eigen::Quaterniond q_r(relative.orientation.w, relative.orientation.x, relative.orientation.y, relative.orientation.z);
  Eigen::Affine3d T_base_rel = t_r * q_r;

  Eigen::Affine3d T_world_rel = T_world_base * T_base_rel;

  geometry_msgs::msg::Point p;
  p.x = T_world_rel.translation().x();
  p.y = T_world_rel.translation().y();
  p.z = T_world_rel.translation().z();
  return p;
}

void DualArmForceControl::build_publish_map_locked()
{
  pub_map_.clear();
  pub_map_.resize(joint_names_.size());

  for (size_t i=0; i<joint_names_.size(); ++i) {
    const auto& n = joint_names_[i];
    PubMapEntry e;

    // Arm
    if      (n=="left_joint_1")  { e.src=PubMapEntry::Src::L_ARM; e.idx=0; }
    else if (n=="left_joint_2")  { e.src=PubMapEntry::Src::L_ARM; e.idx=1; }
    else if (n=="left_joint_3")  { e.src=PubMapEntry::Src::L_ARM; e.idx=2; }
    else if (n=="left_joint_4")  { e.src=PubMapEntry::Src::L_ARM; e.idx=3; }
    else if (n=="left_joint_5")  { e.src=PubMapEntry::Src::L_ARM; e.idx=4; }
    else if (n=="left_joint_6")  { e.src=PubMapEntry::Src::L_ARM; e.idx=5; }
    else if (n=="right_joint_1") { e.src=PubMapEntry::Src::R_ARM; e.idx=0; }
    else if (n=="right_joint_2") { e.src=PubMapEntry::Src::R_ARM; e.idx=1; }
    else if (n=="right_joint_3") { e.src=PubMapEntry::Src::R_ARM; e.idx=2; }
    else if (n=="right_joint_4") { e.src=PubMapEntry::Src::R_ARM; e.idx=3; }
    else if (n=="right_joint_5") { e.src=PubMapEntry::Src::R_ARM; e.idx=4; }
    else if (n=="right_joint_6") { e.src=PubMapEntry::Src::R_ARM; e.idx=5; }
    else {
      // Hand (keyword 기반)
      int f_base = -1;
      if      (n.find("thumb")  != std::string::npos) f_base = 0;
      else if (n.find("index")  != std::string::npos) f_base = 4;
      else if (n.find("middle") != std::string::npos) f_base = 8;
      else if (n.find("ring")   != std::string::npos) f_base = 12;
      else if (n.find("baby")   != std::string::npos) f_base = 16;

      const int trailing = extract_trailing_int(n); // 1..4 기대
      const int j_idx = (trailing>=1 && trailing<=4) ? (trailing-1) : -1;

      if (f_base != -1 && j_idx != -1) {
        if (n.find("left") != std::string::npos) {
          e.src = PubMapEntry::Src::L_HAND; e.idx = f_base + j_idx;
        } else if (n.find("right") != std::string::npos) {
          e.src = PubMapEntry::Src::R_HAND; e.idx = f_base + j_idx;
        } else {
          e.src = PubMapEntry::Src::CONST0; e.idx = -1;
        }
      } else {
        e.src = PubMapEntry::Src::CONST0; e.idx = -1;
      }
    }

    pub_map_[i] = e;
  }
}

void DualArmForceControl::update_fk_locked()
{
  if (!arm_fk_ || !hand_fk_l_ || !hand_fk_r_) return;

  std::vector<double> jl(q_l_c_.data(), q_l_c_.data()+6);
  std::vector<double> jr(q_r_c_.data(), q_r_c_.data()+6);
  current_pose_l_ = arm_fk_->getLeftFK(jl);
  current_pose_r_ = arm_fk_->getRightFK(jr);

  std::vector<double> hl(q_l_h_c_.data(), q_l_h_c_.data()+20);
  std::vector<double> hr(q_r_h_c_.data(), q_r_h_c_.data()+20);

  auto tl = hand_fk_l_->computeFingertips(hl);
  auto tr = hand_fk_r_->computeFingertips(hr);

  if(!tl.empty()){
    f_l_thumb_  = combinePose(current_pose_l_, tl["link4_thumb"]);
    f_l_index_  = combinePose(current_pose_l_, tl["link4_index"]);
    f_l_middle_ = combinePose(current_pose_l_, tl["link4_middle"]);
    f_l_ring_   = combinePose(current_pose_l_, tl["link4_ring"]);
    f_l_baby_   = combinePose(current_pose_l_, tl["link4_baby"]);
  }
  if(!tr.empty()){
    f_r_thumb_  = combinePose(current_pose_r_, tr["link4_thumb"]);
    f_r_index_  = combinePose(current_pose_r_, tr["link4_index"]);
    f_r_middle_ = combinePose(current_pose_r_, tr["link4_middle"]);
    f_r_ring_   = combinePose(current_pose_r_, tr["link4_ring"]);
    f_r_baby_   = combinePose(current_pose_r_, tr["link4_baby"]);
  }
}

void DualArmForceControl::ControlLoop()
{
  sensor_msgs::msg::JointState cmd;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!is_initialized_ || joint_names_.empty()) return;

    // idle 진입 시 1회 sync (현재 상태 유지 = hold)
    if (mode_ == ControlMode::IDLE && !idle_synced_) {
      q_l_t_ = q_l_c_;
      q_r_t_ = q_r_c_;
      q_l_h_t_ = q_l_h_c_;
      q_r_h_t_ = q_r_h_c_;
      idle_synced_ = true;
    }
    if (mode_ != ControlMode::IDLE) idle_synced_ = false;

    cmd.header.stamp = node_->now();
    cmd.name = joint_names_;
    cmd.position.resize(joint_names_.size(), 0.0);

    // pub_map_ 기반으로 빠르게 채우기
    for (size_t i=0; i<pub_map_.size(); ++i) {
      const auto& e = pub_map_[i];
      switch(e.src){
        case PubMapEntry::Src::L_ARM:  cmd.position[i] = q_l_t_(e.idx); break;
        case PubMapEntry::Src::R_ARM:  cmd.position[i] = q_r_t_(e.idx); break;
        case PubMapEntry::Src::L_HAND: cmd.position[i] = q_l_h_t_(e.idx); break;
        case PubMapEntry::Src::R_HAND: cmd.position[i] = q_r_h_t_(e.idx); break;
        default:                       cmd.position[i] = 0.0; break;
      }
    }
  }
  joint_command_pub_->publish(cmd);
}
