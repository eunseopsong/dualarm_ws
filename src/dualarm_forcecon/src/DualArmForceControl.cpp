#include "DualArmForceControl.h"
#include <chrono>

using namespace std::chrono_literals;

DualArmForceControl::DualArmForceControl(std::shared_ptr<rclcpp::Node> node)
: node_(node)
{
    // -------------------------
    // Params (Isaac UI match)
    // -------------------------
    urdf_path_ = node_->declare_parameter<std::string>(
        "urdf_path",
        "/home/eunseop/isaac/isaac_save/dualarm/dualarm_description/urdf/aidin_dsr_dualarm.urdf"
    );

    auto world_base_xyz_vec = node_->declare_parameter<std::vector<double>>(
        "world_base_xyz", std::vector<double>{0.0, 0.0, 0.306}
    );
    auto world_base_euler_xyz_deg_vec = node_->declare_parameter<std::vector<double>>(
        "world_base_euler_xyz_deg", std::vector<double>{0.0, 0.0, 0.0}
    );

    auto to_arr3 = [&](const std::vector<double>& v, const std::array<double,3>& def)->std::array<double,3>{
        if (v.size() >= 3) return {v[0], v[1], v[2]};
        return def;
    };

    world_base_xyz_ = to_arr3(world_base_xyz_vec, {0.0, 0.0, 0.306});
    world_base_euler_xyz_deg_ = to_arr3(world_base_euler_xyz_deg_vec, {0.0, 0.0, 0.0});

    ik_targets_frame_ = node_->declare_parameter<std::string>("ik_targets_frame", "base"); // base/world
    ik_euler_conv_    = node_->declare_parameter<std::string>("ik_euler_conv", "rpy");     // rpy/xyz
    ik_angle_unit_    = node_->declare_parameter<std::string>("ik_angle_unit", "rad");     // rad/deg/auto

    // -------------------------
    // ROS IF
    // -------------------------
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/isaac_joint_states", qos, std::bind(&DualArmForceControl::JointsCallback, this, std::placeholders::_1));

    position_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/isaac_joint_states", qos, std::bind(&DualArmForceControl::PositionCallback, this, std::placeholders::_1));

    // ✅ v11: target_arm / target_hand 분리
    target_arm_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_arm_cartesian_pose", qos,
        std::bind(&DualArmForceControl::TargetArmPositionCallback, this, std::placeholders::_1));

    target_hand_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_hand_fingertips", qos,
        std::bind(&DualArmForceControl::TargetHandPositionCallback, this, std::placeholders::_1));

    contact_force_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/isaac_contact_states", qos, std::bind(&DualArmForceControl::ContactForceCallback, this, std::placeholders::_1));

    target_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/forward_joint_targets", qos, std::bind(&DualArmForceControl::TargetJointCallback, this, std::placeholders::_1));

    joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_command", 10);

    mode_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/change_control_mode",
        std::bind(&DualArmForceControl::ControlModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    // -------------------------
    // State init
    // -------------------------
    q_l_c_.setZero(6); q_r_c_.setZero(6); q_l_t_.setZero(6); q_r_t_.setZero(6);
    q_l_h_c_.setZero(20); q_r_h_c_.setZero(20); q_l_h_t_.setZero(20); q_r_h_t_.setZero(20);

    f_l_hand_c_.setZero(); f_r_hand_c_.setZero();
    f_l_hand_t_.setZero(); f_r_hand_t_.setZero();

    // -------------------------
    // Kinematics
    // -------------------------
    arm_fk_   = std::make_shared<ArmForwardKinematics>(urdf_path_, "base_link", "left_link_6", "right_link_6");
    arm_ik_l_ = std::make_shared<ArmInverseKinematics>(urdf_path_, "base_link", "left_link_6");
    arm_ik_r_ = std::make_shared<ArmInverseKinematics>(urdf_path_, "base_link", "right_link_6");

    if (arm_fk_ && arm_fk_->isOk()) {
        arm_fk_->setWorldBaseTransformXYZEulerDeg(world_base_xyz_, world_base_euler_xyz_deg_);
    }
    if (arm_ik_l_ && arm_ik_l_->isOk()) {
        arm_ik_l_->setWorldBaseTransformXYZEulerDeg(world_base_xyz_, world_base_euler_xyz_deg_);
    }
    if (arm_ik_r_ && arm_ik_r_->isOk()) {
        arm_ik_r_->setWorldBaseTransformXYZEulerDeg(world_base_xyz_, world_base_euler_xyz_deg_);
    }

    // NOTE: canonical tip keys 유지
    std::vector<std::string> tips = {"link4_thumb", "link4_index", "link4_middle", "link4_ring", "link4_baby"};

    hand_fk_l_ = std::make_shared<dualarm_forcecon::HandForwardKinematics>(urdf_path_, "left_hand_base_link", tips);
    hand_fk_r_ = std::make_shared<dualarm_forcecon::HandForwardKinematics>(urdf_path_, "right_hand_base_link", tips);

    // ✅ v11: Hand IK (pos-only)
    hand_ik_l_ = std::make_shared<dualarm_forcecon::HandInverseKinematics>(urdf_path_, "left_hand_base_link", tips);
    hand_ik_r_ = std::make_shared<dualarm_forcecon::HandInverseKinematics>(urdf_path_, "right_hand_base_link", tips);

    // -------------------------
    // Timers
    // -------------------------
    print_timer_   = node_->create_wall_timer(500ms, std::bind(&DualArmForceControl::PrintDualArmStates, this));
    control_timer_ = node_->create_wall_timer(10ms,  std::bind(&DualArmForceControl::ControlLoop, this));
}

DualArmForceControl::~DualArmForceControl() {}

void DualArmForceControl::ControlLoop() {
    if (!is_initialized_ || joint_names_.empty()) return;

    if (current_control_mode_ == "idle" && !idle_synced_) {
        q_l_t_   = q_l_c_;
        q_r_t_   = q_r_c_;
        q_l_h_t_ = q_l_h_c_;
        q_r_h_t_ = q_r_h_c_;
        idle_synced_ = true;
    } else if (current_control_mode_ != "idle") {
        idle_synced_ = false;
    }

    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = joint_names_;
    cmd.position.reserve(joint_names_.size());

    for (const auto& n : joint_names_) {
        // ----------------
        // Arms
        // ----------------
        if      (n == "left_joint_1")  cmd.position.push_back(q_l_t_(0));
        else if (n == "left_joint_2")  cmd.position.push_back(q_l_t_(1));
        else if (n == "left_joint_3")  cmd.position.push_back(q_l_t_(2));
        else if (n == "left_joint_4")  cmd.position.push_back(q_l_t_(3));
        else if (n == "left_joint_5")  cmd.position.push_back(q_l_t_(4));
        else if (n == "left_joint_6")  cmd.position.push_back(q_l_t_(5));
        else if (n == "right_joint_1") cmd.position.push_back(q_r_t_(0));
        else if (n == "right_joint_2") cmd.position.push_back(q_r_t_(1));
        else if (n == "right_joint_3") cmd.position.push_back(q_r_t_(2));
        else if (n == "right_joint_4") cmd.position.push_back(q_r_t_(3));
        else if (n == "right_joint_5") cmd.position.push_back(q_r_t_(4));
        else if (n == "right_joint_6") cmd.position.push_back(q_r_t_(5));

        // ----------------
        // Hands (robust parsing)
        // ----------------
        else {
            auto hj = dualarm_forcecon::kin::parseHandJointName(n);
            if (!hj.ok) {
                cmd.position.push_back(0.0);
                continue;
            }

            const int idx = hj.finger_id * 4 + hj.joint_id; // 0..19
            if (idx < 0 || idx >= 20) {
                cmd.position.push_back(0.0);
                continue;
            }

            if (hj.is_left) cmd.position.push_back(q_l_h_t_(idx));
            else            cmd.position.push_back(q_r_h_t_(idx));
        }
    }

    joint_command_pub_->publish(cmd);
}