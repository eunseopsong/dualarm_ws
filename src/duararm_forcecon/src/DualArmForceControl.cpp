#include "DualArmForceControl.h"
#include <chrono>

using namespace std::chrono_literals;

DualArmForceControl::DualArmForceControl(std::shared_ptr<rclcpp::Node> node) : node_(node) {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/isaac_joint_states", qos, std::bind(&DualArmForceControl::JointsCallback, this, std::placeholders::_1));
    position_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/isaac_joint_states", qos, std::bind(&DualArmForceControl::PositionCallback, this, std::placeholders::_1));
    target_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("/target_cartesian_pose", qos, std::bind(&DualArmForceControl::TargetPositionCallback, this, std::placeholders::_1));
    contact_force_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("/isaac_contact_states", qos, std::bind(&DualArmForceControl::ContactForceCallback, this, std::placeholders::_1));
    target_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>("/forward_joint_targets", qos, std::bind(&DualArmForceControl::TargetJointCallback, this, std::placeholders::_1));

    joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_command", 10);
    mode_service_ = node_->create_service<std_srvs::srv::Trigger>("/change_control_mode", std::bind(&DualArmForceControl::ControlModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    q_l_c_.setZero(6); q_r_c_.setZero(6); q_l_t_.setZero(6); q_r_t_.setZero(6);
    q_l_h_c_.setZero(20); q_r_h_c_.setZero(20); q_l_h_t_.setZero(20); q_r_h_t_.setZero(20);
    f_l_c_.setZero(); f_r_c_.setZero();

    std::string urdf = "/home/eunseop/isaac/isaac_save/dualarm/dualarm_description/urdf/aidin_dsr_dualarm.urdf";
    arm_fk_ = std::make_shared<ArmForwardKinematics>(urdf, "base_link", "left_link_6", "right_link_6");
    arm_ik_l_ = std::make_shared<ArmInverseKinematics>(urdf, "base_link", "left_link_6");
    arm_ik_r_ = std::make_shared<ArmInverseKinematics>(urdf, "base_link", "right_link_6");

    // ✅ Hand FK 초기화
    std::vector<std::string> tips = {"link4_thumb", "link4_index", "link4_middle", "link4_ring", "link4_baby"};
    hand_fk_l_ = std::make_shared<HandForwardKinematics>(urdf, "left_hand_base_link", tips);
    hand_fk_r_ = std::make_shared<HandForwardKinematics>(urdf, "right_hand_base_link", tips);

    print_timer_ = node_->create_wall_timer(500ms, std::bind(&DualArmForceControl::PrintDualArmStates, this));
    control_timer_ = node_->create_wall_timer(10ms, std::bind(&DualArmForceControl::ControlLoop, this));
}

geometry_msgs::msg::Point DualArmForceControl::combinePose(const geometry_msgs::msg::Pose& base, const geometry_msgs::msg::Pose& relative) {
    Eigen::Translation3d t_b(base.position.x, base.position.y, base.position.z);
    Eigen::Quaterniond q_b(base.orientation.w, base.orientation.x, base.orientation.y, base.orientation.z);
    Eigen::Affine3d T_world_base = t_b * q_b;

    Eigen::Translation3d t_r(relative.position.x, relative.position.y, relative.position.z);
    Eigen::Quaterniond q_r(relative.orientation.w, relative.orientation.x, relative.orientation.y, relative.orientation.z);
    Eigen::Affine3d T_base_rel = t_r * q_r;

    Eigen::Affine3d T_world_rel = T_world_base * T_base_rel;
    
    geometry_msgs::msg::Point p;
    p.x = T_world_rel.translation().x(); p.y = T_world_rel.translation().y(); p.z = T_world_rel.translation().z();
    return p;
}

// ControlLoop, JointsCallback 등 기존 로직 유지...
void DualArmForceControl::ControlLoop() {
    if (!is_initialized_ || joint_names_.empty()) return;
    if (current_control_mode_ == "idle" && !idle_synced_) {
        q_l_t_=q_l_c_; q_r_t_=q_r_c_; q_l_h_t_=q_l_h_c_; q_r_h_t_=q_r_h_c_; idle_synced_=true;
    } else if (current_control_mode_ != "idle") { idle_synced_=false; }

    auto cmd = sensor_msgs::msg::JointState();
    cmd.header.stamp = node_->now(); cmd.name = joint_names_;
    for (const auto& n : joint_names_) {
        if (n=="left_joint_1") cmd.position.push_back(q_l_t_(0));
        else if (n=="left_joint_2") cmd.position.push_back(q_l_t_(1));
        else if (n=="left_joint_3") cmd.position.push_back(q_l_t_(2));
        else if (n=="left_joint_4") cmd.position.push_back(q_l_t_(3));
        else if (n=="left_joint_5") cmd.position.push_back(q_l_t_(4));
        else if (n=="left_joint_6") cmd.position.push_back(q_l_t_(5));
        else if (n=="right_joint_1") cmd.position.push_back(q_r_t_(0));
        else if (n=="right_joint_2") cmd.position.push_back(q_r_t_(1));
        else if (n=="right_joint_3") cmd.position.push_back(q_r_t_(2));
        else if (n=="right_joint_4") cmd.position.push_back(q_r_t_(3));
        else if (n=="right_joint_5") cmd.position.push_back(q_r_t_(4));
        else if (n=="right_joint_6") cmd.position.push_back(q_r_t_(5));
        else {
            int f_idx = -1;
            if (n.find("thumb")!=std::string::npos) f_idx=0;
            else if (n.find("index")!=std::string::npos) f_idx=4;
            else if (n.find("middle")!=std::string::npos) f_idx=8;
            else if (n.find("ring")!=std::string::npos) f_idx=12;
            else if (n.find("baby")!=std::string::npos) f_idx=16;
            if (f_idx != -1) {
                int j_idx = -1;
                if (n.find("1")!=std::string::npos) j_idx=0; else if (n.find("2")!=std::string::npos) j_idx=1;
                else if (n.find("3")!=std::string::npos) j_idx=2; else if (n.find("4")!=std::string::npos) j_idx=3;
                if (j_idx != -1) {
                    if (n.find("left")!=std::string::npos) cmd.position.push_back(q_l_h_t_(f_idx+j_idx));
                    else cmd.position.push_back(q_r_h_t_(f_idx+j_idx));
                } else cmd.position.push_back(0.0);
            } else cmd.position.push_back(0.0);
        }
    }
    joint_command_pub_->publish(cmd);
}
DualArmForceControl::~DualArmForceControl() {}