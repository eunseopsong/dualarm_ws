#include "DualArmForceControl.h"
#include <chrono>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace {

// ----- YAML helpers -----
template <std::size_t N>
bool readArrayDouble(const YAML::Node& n, const char* key, std::array<double,N>& out) {
    if (!n || !n[key]) return false;
    const YAML::Node v = n[key];
    if (!v.IsSequence() || v.size() != N) return false;
    for (std::size_t i = 0; i < N; ++i) out[i] = v[i].as<double>();
    return true;
}

template <std::size_t N>
bool readArrayBool(const YAML::Node& n, const char* key, std::array<bool,N>& out) {
    if (!n || !n[key]) return false;
    const YAML::Node v = n[key];
    if (!v.IsSequence() || v.size() != N) return false;
    for (std::size_t i = 0; i < N; ++i) out[i] = v[i].as<bool>();
    return true;
}

template <typename T>
bool readScalar(const YAML::Node& n, const char* key, T& out) {
    if (!n || !n[key]) return false;
    out = n[key].as<T>();
    return true;
}

inline Eigen::Vector3d rowToVec3(const Eigen::Matrix<double,5,3>& M, int row)
{
    if (row < 0 || row >= 5) return Eigen::Vector3d::Zero();
    return Eigen::Vector3d(M(row, 0), M(row, 1), M(row, 2));
}

inline void vec3ToRow(Eigen::Matrix<double,5,3>& M, int row, const Eigen::Vector3d& v)
{
    if (row < 0 || row >= 5) return;
    M(row, 0) = v.x();
    M(row, 1) = v.y();
    M(row, 2) = v.z();
}

inline std::vector<double> eigen20ToStdVec20(const Eigen::VectorXd& qh)
{
    std::vector<double> out(20, 0.0);
    const int n = std::min<int>(qh.size(), 20);
    for (int i = 0; i < n; ++i) out[i] = qh(i);
    return out;
}

inline std::vector<double> compress20to15(const Eigen::VectorXd& qh)
{
    std::vector<double> h15(15, 0.0);

    if (qh.size() >= 20) {
        for (int f = 0; f < 5; ++f) {
            const int b20 = f * 4;
            const int b15 = f * 3;
            h15[b15 + 0] = qh(b20 + 0);
            h15[b15 + 1] = qh(b20 + 1);
            h15[b15 + 2] = qh(b20 + 2);
        }
        return h15;
    }

    const int n = std::min<int>(qh.size(), 15);
    for (int i = 0; i < n; ++i) h15[i] = qh(i);
    return h15;
}

inline Eigen::Vector3d safeGetTip(const std::vector<Eigen::Vector3d>& tips, int idx)
{
    if (idx < 0 || idx >= static_cast<int>(tips.size())) return Eigen::Vector3d::Zero();
    const Eigen::Vector3d& p = tips[static_cast<std::size_t>(idx)];
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) {
        return Eigen::Vector3d::Zero();
    }
    return p;
}

inline void applyFingerCmd(Eigen::VectorXd& qh20,
                           int finger_id,
                           const dualarm_forcecon::HandAdmittanceControl::StepOutput& out_step)
{
    if (finger_id < 0 || finger_id > 4) return;
    if (qh20.size() < 20) return;

    const int b = finger_id * 4;
    qh20(b + 0) = out_step.q_cmd_123[0];
    qh20(b + 1) = out_step.q_cmd_123[1];
    qh20(b + 2) = out_step.q_cmd_123[2];
    qh20(b + 3) = out_step.q_cmd_4_mimic;
}

void applyYamlToHandCfg(const YAML::Node& n, dualarm_forcecon::HandAdmittanceControl::Config& cfg) {
    readArrayDouble<3>(n, "mass", cfg.mass);
    readArrayDouble<3>(n, "damping", cfg.damping);
    readArrayDouble<3>(n, "stiffness", cfg.stiffness);

    readArrayBool<3>(n, "force_ctrl_enable", cfg.force_ctrl_enable);

    readArrayDouble<3>(n, "force_error_axis_sign", cfg.force_error_axis_sign);
    readArrayDouble<3>(n, "force_deadband_N", cfg.force_deadband_N);

    readArrayDouble<3>(n, "max_offset_m", cfg.max_offset_m);
    readArrayDouble<3>(n, "max_step_m", cfg.max_step_m);
    readArrayDouble<3>(n, "max_adm_velocity_mps", cfg.max_adm_velocity_mps);

    readArrayDouble<3>(n, "force_target_ramp_rate_Nps", cfg.force_target_ramp_rate_Nps);
    readArrayDouble<3>(n, "force_target_release_rate_Nps", cfg.force_target_release_rate_Nps);

    readScalar<bool>(n, "use_hybrid_force_position_mode", cfg.use_hybrid_force_position_mode);
    readScalar<int>(n, "hybrid_force_axis", cfg.hybrid_force_axis);

    readScalar<bool>(n, "hold_tangent_anchor_on_contact", cfg.hold_tangent_anchor_on_contact);
    readScalar<bool>(n, "tangent_anchor_use_measured_pose", cfg.tangent_anchor_use_measured_pose);

    readScalar<bool>(n, "use_force_lpf", cfg.use_force_lpf);
    readScalar<double>(n, "force_lpf_tau_s", cfg.force_lpf_tau_s);

    readScalar<double>(n, "dt_min_s", cfg.dt_min_s);
    readScalar<double>(n, "dt_max_s", cfg.dt_max_s);

    readScalar<bool>(n, "force_error_des_minus_meas", cfg.force_error_des_minus_meas);

    readScalar<double>(n, "contact_force_threshold_N", cfg.contact_force_threshold_N);
    readScalar<bool>(n, "use_contact_hysteresis", cfg.use_contact_hysteresis);
    readScalar<double>(n, "contact_on_threshold_N", cfg.contact_on_threshold_N);
    readScalar<double>(n, "contact_off_threshold_N", cfg.contact_off_threshold_N);
    readScalar<bool>(n, "contact_gate_use_enabled_axes_only", cfg.contact_gate_use_enabled_axes_only);

    readScalar<bool>(n, "decay_when_no_contact", cfg.decay_when_no_contact);
    readScalar<double>(n, "no_contact_decay_ratio", cfg.no_contact_decay_ratio);

    readScalar<bool>(n, "antiwindup_on_offset_clamp", cfg.antiwindup_on_offset_clamp);
    readScalar<bool>(n, "zero_velocity_on_offset_clamp", cfg.zero_velocity_on_offset_clamp);
    readScalar<double>(n, "offset_clamp_velocity_damping", cfg.offset_clamp_velocity_damping);
    readScalar<bool>(n, "sync_adm_state_to_final_cmd", cfg.sync_adm_state_to_final_cmd);

    readScalar<bool>(n, "use_slip_detection", cfg.use_slip_detection);
    readScalar<double>(n, "tangent_slip_threshold_m", cfg.tangent_slip_threshold_m);

    readScalar<bool>(n, "use_slip_guard", cfg.use_slip_guard);
    readScalar<double>(n, "slip_guard_force_scale", cfg.slip_guard_force_scale);
    readScalar<bool>(n, "slip_guard_reanchor_tangent", cfg.slip_guard_reanchor_tangent);
    readScalar<double>(n, "slip_guard_velocity_damping", cfg.slip_guard_velocity_damping);

    readScalar<int>(n, "ik_max_iters", cfg.ik_max_iters);
    readScalar<double>(n, "ik_tol_pos_m", cfg.ik_tol_pos_m);
    readScalar<double>(n, "ik_lambda", cfg.ik_lambda);
    readScalar<double>(n, "ik_lambda_min", cfg.ik_lambda_min);
    readScalar<double>(n, "ik_lambda_max", cfg.ik_lambda_max);
    readScalar<double>(n, "ik_alpha", cfg.ik_alpha);
    readScalar<double>(n, "ik_alpha_min", cfg.ik_alpha_min);
    readScalar<double>(n, "ik_max_step", cfg.ik_max_step);
    readScalar<double>(n, "ik_mu_posture", cfg.ik_mu_posture);

    readScalar<bool>(n, "prefer_last_success_q_seed", cfg.prefer_last_success_q_seed);
    readScalar<bool>(n, "keep_last_success_on_ik_fail", cfg.keep_last_success_on_ik_fail);
    readScalar<bool>(n, "damp_velocity_on_ik_fail", cfg.damp_velocity_on_ik_fail);
    readScalar<double>(n, "ik_fail_velocity_damping", cfg.ik_fail_velocity_damping);

    readArrayDouble<9>(n, "R_tip_sensor_rowmajor", cfg.R_tip_sensor_rowmajor);
    readArrayDouble<9>(n, "R_base_corr_rowmajor", cfg.R_base_corr_rowmajor);
    readScalar<bool>(n, "fallback_to_f_meas_base_if_sensor_transform_fails",
                     cfg.fallback_to_f_meas_base_if_sensor_transform_fails);
    readScalar<int>(n, "debug_decimation", cfg.debug_decimation);
}


inline std::string resolveKinematicsConfigYaml(
    const YAML::Node& root,
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& default_yaml)
{
    auto logger = node ? node->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    // ----------------------------------------------------------------------
    // Priority 1: explicit kinematics YAML override from ROS parameter
    //
    // Example:
    //   -p kinematics_cfg_yaml:=/absolute/path/to/rby1_kinematics.yaml
    // ----------------------------------------------------------------------
    const std::string override_yaml =
        node->declare_parameter<std::string>("kinematics_cfg_yaml", "");

    if (!override_yaml.empty()) {
        RCLCPP_INFO(
            logger,
            "[KinematicsConfig] using ROS parameter override kinematics_cfg_yaml=%s",
            override_yaml.c_str());
        return override_yaml;
    }

    const YAML::Node kin_node = root ? root["kinematics"] : YAML::Node();

    if (!kin_node) {
        RCLCPP_WARN(
            logger,
            "[KinematicsConfig] forcecon yaml has no 'kinematics' node. Use default yaml=%s",
            default_yaml.c_str());
        return default_yaml;
    }

    // ----------------------------------------------------------------------
    // Priority 2: robot_profile_id from ROS parameter
    //
    // Example:
    //   -p robot_profile_id:=1
    // ----------------------------------------------------------------------
    const int robot_profile_id_param =
        node->declare_parameter<int>("robot_profile_id", -1);

    int selected_id = robot_profile_id_param;

    // ----------------------------------------------------------------------
    // Priority 3: active_robot_id from forcecon_cfg.yaml
    // ----------------------------------------------------------------------
    try {
        if (selected_id < 0 && kin_node["active_robot_id"]) {
            selected_id = kin_node["active_robot_id"].as<int>();
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(
            logger,
            "[KinematicsConfig] failed to read kinematics.active_robot_id (%s).",
            e.what());
    }

    // ----------------------------------------------------------------------
    // New v27+ profile-list style:
    //
    // kinematics:
    //   active_robot_id: 0
    //   profiles:
    //     - id: 0
    //       name: doosan_dualarm
    //       config_yaml: /path/to/doosan_dualarm_kinematics.yaml
    //     - id: 1
    //       name: rby1
    //       config_yaml: /path/to/rby1_kinematics.yaml
    // ----------------------------------------------------------------------
    if (selected_id >= 0 && kin_node["profiles"] && kin_node["profiles"].IsSequence()) {
        try {
            for (const auto& profile : kin_node["profiles"]) {
                if (!profile["id"] || !profile["config_yaml"]) {
                    continue;
                }

                const int id = profile["id"].as<int>();
                if (id != selected_id) {
                    continue;
                }

                const std::string name =
                    profile["name"] ? profile["name"].as<std::string>() : "unnamed";

                const std::string config_yaml =
                    profile["config_yaml"].as<std::string>();

                RCLCPP_INFO(
                    logger,
                    "[KinematicsConfig] selected robot id=%d name=%s yaml=%s",
                    id,
                    name.c_str(),
                    config_yaml.c_str());

                return config_yaml;
            }

            RCLCPP_WARN(
                logger,
                "[KinematicsConfig] active robot id=%d was not found in kinematics.profiles.",
                selected_id);
        } catch (const std::exception& e) {
            RCLCPP_WARN(
                logger,
                "[KinematicsConfig] failed while parsing kinematics.profiles (%s).",
                e.what());
        }
    }

    // ----------------------------------------------------------------------
    // Backward-compatible fallback:
    //
    // kinematics:
    //   config_yaml: /path/to/doosan_dualarm_kinematics.yaml
    // ----------------------------------------------------------------------
    try {
        if (kin_node["config_yaml"]) {
            const std::string config_yaml = kin_node["config_yaml"].as<std::string>();
            RCLCPP_INFO(
                logger,
                "[KinematicsConfig] fallback to kinematics.config_yaml=%s",
                config_yaml.c_str());
            return config_yaml;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(
            logger,
            "[KinematicsConfig] failed to read kinematics.config_yaml (%s).",
            e.what());
    }

    RCLCPP_WARN(
        logger,
        "[KinematicsConfig] no valid profile selected. Use default yaml=%s",
        default_yaml.c_str());

    return default_yaml;
}


} // namespace

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

    ik_targets_frame_ = node_->declare_parameter<std::string>("ik_targets_frame", "base");
    ik_euler_conv_    = node_->declare_parameter<std::string>("ik_euler_conv", "rpy");
    ik_angle_unit_    = node_->declare_parameter<std::string>("ik_angle_unit", "rad");

    const std::string cfg_yaml_path = node_->declare_parameter<std::string>(
        "forcecon_cfg_yaml",
        "/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml"
    );

    const std::string default_kinematics_cfg_yaml_path =
        "/home/eunseop/dualarm_ws/src/dualarm_kinematics/config/doosan_dualarm_kinematics.yaml";

    // -------------------------
    // Load YAML cfg early
    //   - forcecon_cfg_yaml : force/admittance/control parameters
    //   - selected robot kinematics profile is resolved from:
    //
    //       1) ROS parameter:
    //            -p kinematics_cfg_yaml:=/absolute/path/to/robot_kinematics.yaml
    //
    //       2) ROS parameter:
    //            -p robot_profile_id:=1
    //
    //       3) forcecon_cfg.yaml:
    //            kinematics.active_robot_id
    //
    //       4) backward-compatible:
    //            kinematics.config_yaml
    // -------------------------
    YAML::Node root;
    try {
        root = YAML::LoadFile(cfg_yaml_path);
        RCLCPP_INFO(node_->get_logger(), "[forcecon_cfg] loaded: %s", cfg_yaml_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(),
                    "[forcecon_cfg] failed to load %s (%s). Use defaults.",
                    cfg_yaml_path.c_str(), e.what());
        root = YAML::Node();
    }

    const std::string kinematics_cfg_yaml_path =
        resolveKinematicsConfigYaml(root, node_, default_kinematics_cfg_yaml_path);

    YAML::Node kin_root;
    bool kin_yaml_loaded = false;
    try {
        kin_root = YAML::LoadFile(kinematics_cfg_yaml_path);
        kin_yaml_loaded = true;
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(),
                    "[kinematics_cfg] failed to load %s (%s). Fall back to forcecon_cfg_yaml robot_kinematics block.",
                    kinematics_cfg_yaml_path.c_str(), e.what());
        kin_root = root;
    }

    kin_cfg_ = dualarm_forcecon::DualArmKinematicsConfig::fromYaml(kin_root, urdf_path_);
    urdf_path_ = kin_cfg_.urdf_path;

    RCLCPP_INFO(node_->get_logger(),
                "[KinematicsConfig] source=%s",
                kin_yaml_loaded ? kinematics_cfg_yaml_path.c_str() : "forcecon_cfg_yaml fallback");

    RCLCPP_INFO(node_->get_logger(),
                "[KinematicsConfig] profile=%s urdf=%s base=%s Ltip=%s Rtip=%s Ldof=%d Rdof=%d hand_enabled=%d",
                kin_cfg_.profile.c_str(), urdf_path_.c_str(), kin_cfg_.arm_base_link.c_str(),
                kin_cfg_.left_arm_tip_link.c_str(), kin_cfg_.right_arm_tip_link.c_str(),
                kin_cfg_.leftArmDof(), kin_cfg_.rightArmDof(), static_cast<int>(kin_cfg_.hand_enabled));

    // -------------------------
    // ROS IF
    // -------------------------
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/isaac_joint_states", qos,
        std::bind(&DualArmForceControl::JointsCallback, this, std::placeholders::_1));

    position_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/isaac_joint_states", qos,
        std::bind(&DualArmForceControl::PositionCallback, this, std::placeholders::_1));

    target_arm_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_arm_cartesian_pose", qos,
        std::bind(&DualArmForceControl::TargetArmPositionCallback, this, std::placeholders::_1));

    target_hand_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_hand_fingertips", qos,
        std::bind(&DualArmForceControl::TargetHandPositionCallback, this, std::placeholders::_1));

    delta_arm_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/delta_arm_cartesian_pose", qos,
        std::bind(&DualArmForceControl::DeltaArmPositionCallback, this, std::placeholders::_1));

    delta_hand_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/delta_hand_fingertips", qos,
        std::bind(&DualArmForceControl::DeltaHandPositionCallback, this, std::placeholders::_1));

    target_arm_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/forward_arm_joint_targets", qos,
        std::bind(&DualArmForceControl::TargetArmJointsCallback, this, std::placeholders::_1));

    target_hand_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/forward_hand_joint_targets", qos,
        std::bind(&DualArmForceControl::TargetHandJointsCallback, this, std::placeholders::_1));

    target_hand_force_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_hand_force", qos,
        std::bind(&DualArmForceControl::TargetHandForceCallback, this, std::placeholders::_1));

    contact_force_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/isaac_contact_states", qos,
        std::bind(&DualArmForceControl::HandContactForceCallback, this, std::placeholders::_1));

    joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/isaac_joint_command", 10);

    hand_force_current_monitor_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/hand_force_current_monitor", 10);

    hand_force_target_monitor_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/hand_force_target_monitor", 10);

    mode_service_ = node_->create_service<dualarm_forcecon_interfaces::srv::SetControlMode>(
        "/change_control_mode",
        std::bind(&DualArmForceControl::ControlModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    // -------------------------
    // State init
    // -------------------------
    q_l_c_.setZero(kin_cfg_.leftArmDof());
    q_r_c_.setZero(kin_cfg_.rightArmDof());
    q_l_t_.setZero(kin_cfg_.leftArmDof());
    q_r_t_.setZero(kin_cfg_.rightArmDof());

    q_l_h_c_.setZero(20); q_r_h_c_.setZero(20);
    q_l_h_t_.setZero(20); q_r_h_t_.setZero(20);
    q_l_h_motion_t_.setZero(20); q_r_h_motion_t_.setZero(20);

    q_l_h_fixed_.setZero(20); q_r_h_fixed_.setZero(20);

    f_l_c_.setZero(); f_r_c_.setZero();
    f_l_t_.setZero(); f_r_t_.setZero();

    f_l_hand_c_.setZero(); f_r_hand_c_.setZero();
    f_l_hand_t_.setZero(); f_r_hand_t_.setZero();

    x_l_hand_c_.setZero();   x_r_hand_c_.setZero();
    x_l_hand_d_.setZero();   x_r_hand_d_.setZero();
    x_l_hand_ref_.setZero(); x_r_hand_ref_.setZero();
    x_l_hand_cmd_.setZero(); x_r_hand_cmd_.setZero();

    k_l_hand_eff_.setZero();
    k_r_hand_eff_.setZero();

    raw_l_hand_contact_.setZero();
    raw_r_hand_contact_.setZero();

    f_l_hand_sensor_c_.setZero();
    f_r_hand_sensor_c_.setZero();

    f_l_hand_wrist_c_.setZero();
    f_r_hand_wrist_c_.setZero();

    for (int i = 0; i < 5; ++i) {
        R_l_base_tip_c_[static_cast<std::size_t>(i)] = Eigen::Matrix3d::Identity();
        R_r_base_tip_c_[static_cast<std::size_t>(i)] = Eigen::Matrix3d::Identity();
    }

    hand_force_cmd_valid_ = false;
    hand_force_cmd_hand_id_ = 0;
    hand_force_cmd_finger_id_ = 3;
    hand_force_cmd_f_des_base_.setZero();
    hand_force_cmd_stamp_ns_ = 0;

    hand_cartesian_target_l_initialized_ = false;
    hand_cartesian_target_r_initialized_ = false;

    arm_idle_synced_ = false;
    hand_idle_synced_ = false;

    delta_arm_base_pose_initialized_ = false;

    startup_fixed_joint_hold_enabled_ = (kin_cfg_.profile.find("rby1") != std::string::npos);
    startup_fixed_hand_hold_enabled_  = startup_fixed_joint_hold_enabled_;
    startup_fixed_joint_hold_latched_ = false;
    startup_fixed_hand_hold_latched_  = false;

    // -------------------------
    // Kinematics
    // -------------------------
    arm_fk_   = std::make_shared<ArmForwardKinematics>(
        urdf_path_, kin_cfg_.arm_base_link, kin_cfg_.left_arm_tip_link, kin_cfg_.right_arm_tip_link);
    arm_ik_l_ = std::make_shared<ArmInverseKinematics>(
        urdf_path_, kin_cfg_.arm_base_link, kin_cfg_.left_arm_tip_link);
    arm_ik_r_ = std::make_shared<ArmInverseKinematics>(
        urdf_path_, kin_cfg_.arm_base_link, kin_cfg_.right_arm_tip_link);

    if (arm_fk_ && arm_fk_->isOk()) {
        arm_fk_->setWorldBaseTransformXYZEulerDeg(world_base_xyz_, world_base_euler_xyz_deg_);
    }
    if (arm_ik_l_ && arm_ik_l_->isOk()) {
        arm_ik_l_->setWorldBaseTransformXYZEulerDeg(world_base_xyz_, world_base_euler_xyz_deg_);
    }
    if (arm_ik_r_ && arm_ik_r_->isOk()) {
        arm_ik_r_->setWorldBaseTransformXYZEulerDeg(world_base_xyz_, world_base_euler_xyz_deg_);
    }

    if (kin_cfg_.hand_enabled) {
        hand_fk_l_ = std::make_shared<dualarm_forcecon::HandForwardKinematics>(
            urdf_path_, kin_cfg_.left_hand_base_link, kin_cfg_.hand_tip_suffixes);
        hand_fk_r_ = std::make_shared<dualarm_forcecon::HandForwardKinematics>(
            urdf_path_, kin_cfg_.right_hand_base_link, kin_cfg_.hand_tip_suffixes);

        hand_ik_l_ = std::make_shared<dualarm_forcecon::HandInverseKinematics>(
            urdf_path_, kin_cfg_.left_hand_base_link, kin_cfg_.hand_tip_suffixes);
        hand_ik_r_ = std::make_shared<dualarm_forcecon::HandInverseKinematics>(
            urdf_path_, kin_cfg_.right_hand_base_link, kin_cfg_.hand_tip_suffixes);
    } else {
        hand_fk_l_.reset();
        hand_fk_r_.reset();
        hand_ik_l_.reset();
        hand_ik_r_.reset();
    }

    const YAML::Node hand_node = root["hand_admittance"];
    const YAML::Node def_node  = hand_node ? hand_node["defaults"] : YAML::Node();
    const YAML::Node per_node  = hand_node ? hand_node["per_finger"] : YAML::Node();

    const std::array<std::string,5> finger_key = {{"thumb","index","middle","ring","baby"}};

    for (int f = 0; f < 5; ++f) {
        dualarm_forcecon::HandAdmittanceControl::Config cfg{};

        if (def_node) {
            applyYamlToHandCfg(def_node, cfg);
        }

        if (per_node && per_node[finger_key[static_cast<std::size_t>(f)]]) {
            applyYamlToHandCfg(per_node[finger_key[static_cast<std::size_t>(f)]], cfg);
        }

        hand_adm_l_[f] = std::make_shared<dualarm_forcecon::HandAdmittanceControl>(hand_fk_l_, hand_ik_l_, f, cfg);
        hand_adm_r_[f] = std::make_shared<dualarm_forcecon::HandAdmittanceControl>(hand_fk_r_, hand_ik_r_, f, cfg);

        if (hand_adm_l_[f] && node_) hand_adm_l_[f]->setDebugLogger(node_->get_logger());
        if (hand_adm_r_[f] && node_) hand_adm_r_[f]->setDebugLogger(node_->get_logger());
    }

    // -------------------------
    // Timers
    // -------------------------
    print_timer_   = node_->create_wall_timer(500ms, std::bind(&DualArmForceControl::PrintDualArmStates, this));
    control_timer_ = node_->create_wall_timer(10ms,  std::bind(&DualArmForceControl::ControlLoop, this));

    RCLCPP_INFO(
        node_->get_logger(),
        "[StartupHold] fixed_joint_hold_enabled=%d fixed_hand_hold_enabled=%d profile=%s",
        static_cast<int>(startup_fixed_joint_hold_enabled_),
        static_cast<int>(startup_fixed_hand_hold_enabled_),
        kin_cfg_.profile.c_str());
}

DualArmForceControl::~DualArmForceControl() {}

void DualArmForceControl::ControlLoop()
{
    if (!is_initialized_ || joint_names_.empty()) return;

    // ------------------------------------------------------------------------
    // ARM idle sync
    // ------------------------------------------------------------------------
    if (current_arm_control_mode_ == "idle" && !arm_idle_synced_) {
        q_l_t_ = q_l_c_;
        q_r_t_ = q_r_c_;
        arm_idle_synced_ = true;
    } else if (current_arm_control_mode_ != "idle") {
        arm_idle_synced_ = false;
    }

    // ------------------------------------------------------------------------
    // HAND idle sync
    // ------------------------------------------------------------------------
    if (current_hand_control_mode_ == "idle" && !hand_idle_synced_) {
        if (startup_fixed_hand_hold_enabled_ && startup_fixed_hand_hold_latched_) {
            q_l_h_motion_t_ = q_l_h_fixed_;
            q_r_h_motion_t_ = q_r_h_fixed_;
            q_l_h_t_ = q_l_h_fixed_;
            q_r_h_t_ = q_r_h_fixed_;
        } else {
            q_l_h_motion_t_ = q_l_h_c_;
            q_r_h_motion_t_ = q_r_h_c_;
            q_l_h_t_ = q_l_h_c_;
            q_r_h_t_ = q_r_h_c_;
        }

        x_l_hand_d_   = x_l_hand_c_;
        x_r_hand_d_   = x_r_hand_c_;
        x_l_hand_ref_ = x_l_hand_c_;
        x_r_hand_ref_ = x_r_hand_c_;
        x_l_hand_cmd_ = x_l_hand_c_;
        x_r_hand_cmd_ = x_r_hand_c_;

        k_l_hand_eff_.setZero();
        k_r_hand_eff_.setZero();

        hand_idle_synced_ = true;
    } else if (current_hand_control_mode_ != "idle") {
        hand_idle_synced_ = false;
    }

    // ------------------------------------------------------------------------
    // HAND unified forward / inverse admittance pipeline
    // ------------------------------------------------------------------------
    if (!kin_cfg_.hand_enabled || current_hand_control_mode_ == "idle") {
        f_l_hand_t_.setZero();
        f_r_hand_t_.setZero();
    } else {
        // forward mode: q_motion_target -> FK -> x_d
        if (current_hand_control_mode_ == "forward" && hand_fk_l_ && hand_fk_r_) {
            const std::vector<double> ql_ref15 = compress20to15(q_l_h_motion_t_);
            const std::vector<double> qr_ref15 = compress20to15(q_r_h_motion_t_);

            const std::vector<Eigen::Vector3d> tips_l = hand_fk_l_->computeFingertips(ql_ref15);
            const std::vector<Eigen::Vector3d> tips_r = hand_fk_r_->computeFingertips(qr_ref15);

            for (int f = 0; f < 5; ++f) {
                vec3ToRow(x_l_hand_d_, f, safeGetTip(tips_l, f));
                vec3ToRow(x_r_hand_d_, f, safeGetTip(tips_r, f));
            }

            hand_cartesian_target_l_initialized_ = true;
            hand_cartesian_target_r_initialized_ = true;
        }

        if (!hand_cartesian_target_l_initialized_) x_l_hand_d_ = x_l_hand_c_;
        if (!hand_cartesian_target_r_initialized_) x_r_hand_d_ = x_r_hand_c_;

        Eigen::VectorXd q_l_work = q_l_h_c_;
        Eigen::VectorXd q_r_work = q_r_h_c_;

        auto process_one_hand = [&](bool is_left)
        {
            auto& q_work   = is_left ? q_l_work : q_r_work;
            auto& q_cmd    = is_left ? q_l_h_t_ : q_r_h_t_;

            auto& x_cur_M  = is_left ? x_l_hand_c_   : x_r_hand_c_;
            auto& x_des_M  = is_left ? x_l_hand_d_   : x_r_hand_d_;
            auto& x_ref_M  = is_left ? x_l_hand_ref_ : x_r_hand_ref_;
            auto& x_cmd_M  = is_left ? x_l_hand_cmd_ : x_r_hand_cmd_;
            auto& k_eff_M  = is_left ? k_l_hand_eff_ : k_r_hand_eff_;

            auto& f_cur_M  = is_left ? f_l_hand_c_ : f_r_hand_c_;
            auto& f_des_M  = is_left ? f_l_hand_t_ : f_r_hand_t_;

            auto& adm_arr  = is_left ? hand_adm_l_ : hand_adm_r_;

            for (int finger_id = 0; finger_id < 5; ++finger_id) {
                auto adm_ptr = adm_arr[static_cast<std::size_t>(finger_id)];
                if (!adm_ptr || !adm_ptr->isOk()) {
                    vec3ToRow(x_ref_M, finger_id, rowToVec3(x_des_M, finger_id));
                    vec3ToRow(x_cmd_M, finger_id, rowToVec3(x_des_M, finger_id));
                    k_eff_M.row(finger_id).setZero();
                    continue;
                }

                double dt_s = 0.01;
                if (node_) {
                    static std::array<bool,10> s_valid = {false,false,false,false,false,false,false,false,false,false};
                    static std::array<int64_t,10> s_last_ns = {0,0,0,0,0,0,0,0,0,0};

                    const int key = (is_left ? 0 : 5) + finger_id;
                    const int64_t now_ns = node_->get_clock()->now().nanoseconds();

                    if (s_valid[static_cast<std::size_t>(key)]) {
                        dt_s = static_cast<double>(now_ns - s_last_ns[static_cast<std::size_t>(key)]) * 1e-9;
                    }
                    s_last_ns[static_cast<std::size_t>(key)] = now_ns;
                    s_valid[static_cast<std::size_t>(key)] = true;

                    if (!std::isfinite(dt_s) || dt_s <= 0.0) dt_s = 0.01;
                    dt_s = std::max(1e-4, std::min(dt_s, 5e-2));
                }

                dualarm_forcecon::HandAdmittanceControl::StepInput in;
                in.p_des_base       = rowToVec3(x_des_M, finger_id);
                in.f_des_base       = rowToVec3(f_des_M, finger_id);
                in.f_meas_base      = rowToVec3(f_cur_M, finger_id);
                in.q_hand_current20 = eigen20ToStdVec20(q_work);
                in.dt_s             = dt_s;

                auto out = adm_ptr->step(in);

                if (out.controller_ok) {
                    vec3ToRow(x_ref_M, finger_id, out.p_ref_base);
                    vec3ToRow(x_cmd_M, finger_id, out.p_cmd_base);

                    if (in.f_des_base.norm() > 1e-9) {
                        vec3ToRow(k_eff_M, finger_id, Eigen::Vector3d::Zero());
                    } else {
                        const auto& cfg = adm_ptr->config();
                        vec3ToRow(k_eff_M, finger_id,
                                  Eigen::Vector3d(cfg.stiffness[0], cfg.stiffness[1], cfg.stiffness[2]));
                    }

                    applyFingerCmd(q_work, finger_id, out);
                } else {
                    vec3ToRow(x_ref_M, finger_id, rowToVec3(x_des_M, finger_id));
                    vec3ToRow(x_cmd_M, finger_id, rowToVec3(x_des_M, finger_id));
                    vec3ToRow(k_eff_M, finger_id, Eigen::Vector3d::Zero());
                }
            }

            q_cmd = q_work;
        };

        process_one_hand(true);
        process_one_hand(false);
    }

    // ------------------------------------------------------------------------
    // Publish consolidated joint command
    // ------------------------------------------------------------------------
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = joint_names_;
    cmd.position.reserve(joint_names_.size());

    for (const auto& n : joint_names_) {
        const int li = kin_cfg_.findLeftArmJointIndex(n);
        const int ri = kin_cfg_.findRightArmJointIndex(n);

        if (li >= 0 && li < q_l_t_.size()) {
            cmd.position.push_back(q_l_t_(li));
            continue;
        }
        if (ri >= 0 && ri < q_r_t_.size()) {
            cmd.position.push_back(q_r_t_(ri));
            continue;
        }

        if (startup_fixed_joint_hold_enabled_ && startup_fixed_joint_hold_latched_) {
            const auto it_fixed = startup_fixed_joint_position_.find(n);
            if (it_fixed != startup_fixed_joint_position_.end()) {
                cmd.position.push_back(it_fixed->second);
                continue;
            }
        }

        if (kin_cfg_.hand_enabled) {
            auto hj = dualarm_forcecon::kin::parseHandJointName(n);
            if (hj.ok) {
                const int idx = hj.finger_id * 4 + hj.joint_id;
                if (idx >= 0 && idx < 20) {
                    if (hj.is_left) cmd.position.push_back(q_l_h_t_(idx));
                    else            cmd.position.push_back(q_r_h_t_(idx));
                    continue;
                }
            }
        } else if (startup_fixed_hand_hold_enabled_ && startup_fixed_hand_hold_latched_) {
            auto hj = dualarm_forcecon::kin::parseHandJointName(n);
            if (hj.ok) {
                const int idx = hj.finger_id * 4 + hj.joint_id;
                if (idx >= 0 && idx < 20) {
                    if (hj.is_left) cmd.position.push_back(q_l_h_fixed_(idx));
                    else            cmd.position.push_back(q_r_h_fixed_(idx));
                    continue;
                }
            }
        }

        const auto it = last_joint_position_.find(n);
        cmd.position.push_back(it != last_joint_position_.end() ? it->second : 0.0);
    }

    PublishHandForceMonitor();
    joint_command_pub_->publish(cmd);
}