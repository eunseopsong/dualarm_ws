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

void applyYamlToHandCfg(const YAML::Node& n, dualarm_forcecon::HandAdmittanceControl::Config& cfg) {
    // arrays
    readArrayDouble<3>(n, "mass", cfg.mass);
    readArrayDouble<3>(n, "damping", cfg.damping);
    readArrayDouble<3>(n, "stiffness", cfg.stiffness);

    readArrayBool<3>(n, "force_ctrl_enable", cfg.force_ctrl_enable);

    readArrayDouble<3>(n, "force_error_axis_sign", cfg.force_error_axis_sign);
    readArrayDouble<3>(n, "force_deadband_N", cfg.force_deadband_N);

    readArrayDouble<3>(n, "max_offset_m", cfg.max_offset_m);
    readArrayDouble<3>(n, "max_step_m", cfg.max_step_m);
    readArrayDouble<3>(n, "max_adm_velocity_mps", cfg.max_adm_velocity_mps);

    // scalars / bools
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

    // IK options
    readScalar<int>(n, "ik_max_iters", cfg.ik_max_iters);
    readScalar<double>(n, "ik_tol_pos_m", cfg.ik_tol_pos_m);
    readScalar<double>(n, "ik_lambda", cfg.ik_lambda);
    readScalar<double>(n, "ik_lambda_min", cfg.ik_lambda_min);
    readScalar<double>(n, "ik_lambda_max", cfg.ik_lambda_max);
    readScalar<double>(n, "ik_alpha", cfg.ik_alpha);
    readScalar<double>(n, "ik_alpha_min", cfg.ik_alpha_min);
    readScalar<double>(n, "ik_max_step", cfg.ik_max_step);
    readScalar<double>(n, "ik_mu_posture", cfg.ik_mu_posture);

    // fallback
    readScalar<bool>(n, "prefer_last_success_q_seed", cfg.prefer_last_success_q_seed);
    readScalar<bool>(n, "keep_last_success_on_ik_fail", cfg.keep_last_success_on_ik_fail);
    readScalar<bool>(n, "damp_velocity_on_ik_fail", cfg.damp_velocity_on_ik_fail);
    readScalar<double>(n, "ik_fail_velocity_damping", cfg.ik_fail_velocity_damping);
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

    // cfg yaml path
    const std::string cfg_yaml_path = node_->declare_parameter<std::string>(
        "forcecon_cfg_yaml",
        "/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml"
    );

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

    raw_l_hand_contact_.setZero();
    raw_r_hand_contact_.setZero();

    f_l_hand_sensor_c_.setZero();
    f_r_hand_sensor_c_.setZero();

    f_l_hand_wrist_c_.setZero();
    f_r_hand_wrist_c_.setZero();

    hand_force_cmd_valid_ = false;
    hand_force_cmd_hand_id_ = 0;
    hand_force_cmd_finger_id_ = 3;
    hand_force_cmd_p_des_base_.setZero();
    hand_force_cmd_f_des_base_.setZero();
    hand_force_cmd_stamp_ns_ = 0;

    // forcecon hold snapshot init
    q_l_arm_forcecon_hold_.setZero(6);
    q_r_arm_forcecon_hold_.setZero(6);
    q_l_hand_forcecon_hold_.setZero(20);
    q_r_hand_forcecon_hold_.setZero(20);
    forcecon_hold_snapshot_valid_ = false;
    forcecon_prev_cycle_ = false;

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

    std::vector<std::string> tips = {"link4_thumb", "link4_index", "link4_middle", "link4_ring", "link4_baby"};

    hand_fk_l_ = std::make_shared<dualarm_forcecon::HandForwardKinematics>(urdf_path_, "left_hand_base_link", tips);
    hand_fk_r_ = std::make_shared<dualarm_forcecon::HandForwardKinematics>(urdf_path_, "right_hand_base_link", tips);

    hand_ik_l_ = std::make_shared<dualarm_forcecon::HandInverseKinematics>(urdf_path_, "left_hand_base_link", tips);
    hand_ik_r_ = std::make_shared<dualarm_forcecon::HandInverseKinematics>(urdf_path_, "right_hand_base_link", tips);

    // -------------------------
    // Load YAML cfg (once) and build per-finger controllers
    // -------------------------
    YAML::Node root;
    try {
        root = YAML::LoadFile(cfg_yaml_path);
        RCLCPP_INFO(node_->get_logger(), "[forcecon_cfg] loaded: %s", cfg_yaml_path.c_str());
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(),
                    "[forcecon_cfg] failed to load %s (%s). Use empty cfg.",
                    cfg_yaml_path.c_str(), e.what());
        root = YAML::Node();
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

    const bool in_forcecon = (current_control_mode_ == "forcecon");
    const bool entering_forcecon = ( in_forcecon && !forcecon_prev_cycle_);
    const bool leaving_forcecon  = (!in_forcecon &&  forcecon_prev_cycle_);

    if (entering_forcecon) {
        q_l_arm_forcecon_hold_  = q_l_c_;
        q_r_arm_forcecon_hold_  = q_r_c_;
        q_l_hand_forcecon_hold_ = q_l_h_c_;
        q_r_hand_forcecon_hold_ = q_r_h_c_;

        forcecon_hold_snapshot_valid_ = true;

        q_l_t_   = q_l_arm_forcecon_hold_;
        q_r_t_   = q_r_arm_forcecon_hold_;
        q_l_h_t_ = q_l_hand_forcecon_hold_;
        q_r_h_t_ = q_r_hand_forcecon_hold_;
    }

    if (leaving_forcecon) {
        forcecon_hold_snapshot_valid_ = false;
    }

    forcecon_prev_cycle_ = in_forcecon;

    if (in_forcecon) {
        if (forcecon_hold_snapshot_valid_) {
            q_l_t_   = q_l_arm_forcecon_hold_;
            q_r_t_   = q_r_arm_forcecon_hold_;
            q_l_h_t_ = q_l_hand_forcecon_hold_;
            q_r_h_t_ = q_r_hand_forcecon_hold_;
        } else {
            q_l_t_   = q_l_c_;
            q_r_t_   = q_r_c_;
            q_l_h_t_ = q_l_h_c_;
            q_r_h_t_ = q_r_h_c_;
        }
    }

    if (current_control_mode_ == "forcecon" && hand_force_cmd_valid_) {
        const bool is_left = (hand_force_cmd_hand_id_ == 0);
        const int finger_id = hand_force_cmd_finger_id_;

        if (finger_id >= 0 && finger_id < 5) {
            auto& adm_arr = is_left ? hand_adm_l_ : hand_adm_r_;
            auto adm_ptr = adm_arr[static_cast<size_t>(finger_id)];

            if (adm_ptr && adm_ptr->isOk()) {
                const Eigen::Matrix<double,5,3>& f_hand_cur = is_left ? f_l_hand_c_ : f_r_hand_c_;
                Eigen::VectorXd& q_h_c = is_left ? q_l_h_c_ : q_r_h_c_;
                Eigen::VectorXd& q_h_t = is_left ? q_l_h_t_ : q_r_h_t_;

                std::vector<double> q_cur20(20, 0.0);
                for (int i = 0; i < 20 && i < q_h_c.size(); ++i) q_cur20[i] = q_h_c(i);

                double dt_s = 0.01;
                if (node_) {
                    static std::array<bool,10> s_valid = {false,false,false,false,false,false,false,false,false,false};
                    static std::array<int64_t,10> s_last_ns = {0,0,0,0,0,0,0,0,0,0};

                    const int key = (is_left ? 0 : 5) + finger_id;
                    const int64_t now_ns = node_->get_clock()->now().nanoseconds();

                    if (s_valid[key]) dt_s = static_cast<double>(now_ns - s_last_ns[key]) * 1e-9;
                    s_last_ns[key] = now_ns;
                    s_valid[key] = true;

                    if (!std::isfinite(dt_s) || dt_s <= 0.0) dt_s = 0.01;
                    dt_s = std::max(1e-4, std::min(dt_s, 5e-2));
                }

                dualarm_forcecon::HandAdmittanceControl::StepInput in;
                in.p_des_base = hand_force_cmd_p_des_base_;
                in.f_des_base = hand_force_cmd_f_des_base_;
                in.f_meas_base = f_hand_cur.row(finger_id).transpose();
                in.q_hand_current20 = q_cur20;
                in.dt_s = dt_s;

                auto out = adm_ptr->step(in);

                if (out.controller_ok) {
                    const int b = finger_id * 4;
                    if (b + 3 < q_h_t.size()) {
                        q_h_t(b + 0) = out.q_cmd_123[0];
                        q_h_t(b + 1) = out.q_cmd_123[1];
                        q_h_t(b + 2) = out.q_cmd_123[2];
                        q_h_t(b + 3) = out.q_cmd_4_mimic;
                    }
                }
            }
        }
    }

    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = joint_names_;
    cmd.position.reserve(joint_names_.size());

    for (const auto& n : joint_names_) {
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
        else {
            auto hj = dualarm_forcecon::kin::parseHandJointName(n);
            if (!hj.ok) { cmd.position.push_back(0.0); continue; }

            const int idx = hj.finger_id * 4 + hj.joint_id;
            if (idx < 0 || idx >= 20) { cmd.position.push_back(0.0); continue; }

            if (hj.is_left) cmd.position.push_back(q_l_h_t_(idx));
            else            cmd.position.push_back(q_r_h_t_(idx));
        }
    }

    PublishHandForceMonitor();
    joint_command_pub_->publish(cmd);
}