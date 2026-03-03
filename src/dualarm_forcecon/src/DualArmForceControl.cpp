#include "DualArmForceControl.h"
#include <chrono>
#include <algorithm>
#include <cmath>

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
        "/isaac_joint_states", qos,
        std::bind(&DualArmForceControl::JointsCallback, this, std::placeholders::_1));

    position_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/isaac_joint_states", qos,
        std::bind(&DualArmForceControl::PositionCallback, this, std::placeholders::_1));

    // Cartesian targets (inverse mode)
    target_arm_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_arm_cartesian_pose", qos,
        std::bind(&DualArmForceControl::TargetArmPositionCallback, this, std::placeholders::_1));

    target_hand_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_hand_fingertips", qos,
        std::bind(&DualArmForceControl::TargetHandPositionCallback, this, std::placeholders::_1));

    // Delta Cartesian targets (inverse mode)
    delta_arm_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/delta_arm_cartesian_pose", qos,
        std::bind(&DualArmForceControl::DeltaArmPositionCallback, this, std::placeholders::_1));

    delta_hand_pos_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/delta_hand_fingertips", qos,
        std::bind(&DualArmForceControl::DeltaHandPositionCallback, this, std::placeholders::_1));

    // Forward joint targets (split)
    target_arm_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/forward_arm_joint_targets", qos,
        std::bind(&DualArmForceControl::TargetArmJointsCallback, this, std::placeholders::_1));

    target_hand_joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/forward_hand_joint_targets", qos,
        std::bind(&DualArmForceControl::TargetHandJointsCallback, this, std::placeholders::_1));

    // Force-control target callback (forcecon mode) - now latches command only
    target_hand_force_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_hand_force", qos,
        std::bind(&DualArmForceControl::TargetHandForceCallback, this, std::placeholders::_1));

    contact_force_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/isaac_contact_states", qos,
        std::bind(&DualArmForceControl::HandContactForceCallback, this, std::placeholders::_1));

    joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/isaac_joint_command", 10);

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
    hand_force_cmd_finger_id_ = 3; // ring default
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

    // canonical tip keys
    std::vector<std::string> tips = {"link4_thumb", "link4_index", "link4_middle", "link4_ring", "link4_baby"};

    hand_fk_l_ = std::make_shared<dualarm_forcecon::HandForwardKinematics>(urdf_path_, "left_hand_base_link", tips);
    hand_fk_r_ = std::make_shared<dualarm_forcecon::HandForwardKinematics>(urdf_path_, "right_hand_base_link", tips);

    hand_ik_l_ = std::make_shared<dualarm_forcecon::HandInverseKinematics>(urdf_path_, "left_hand_base_link", tips);
    hand_ik_r_ = std::make_shared<dualarm_forcecon::HandInverseKinematics>(urdf_path_, "right_hand_base_link", tips);

    // Per-finger hand admittance controllers (x-axis force + tangent hold default)
    for (int f = 0; f < 5; ++f) {
        dualarm_forcecon::HandAdmittanceControl::Config cfg;
        cfg.verbose = false;
        cfg.ik_verbose = false;

        // 사용자 요구사항 기본정책: x축 힘제어 + 나머지 축 접촉 후 고정
        cfg.use_hybrid_force_position_mode = true;
        cfg.hybrid_force_axis = 0; // x-axis in HAND_BASE frame
        cfg.force_ctrl_enable = {{true, false, false}}; // hybrid 모드가 강제하긴 하지만 명시
        cfg.hold_tangent_anchor_on_contact = true;
        cfg.tangent_anchor_use_measured_pose = true;
        cfg.release_tangent_anchor_on_contact_off = false;
        cfg.contact_gate_use_enabled_axes_only = true;

        // 접촉 후 미끄러짐 완화 (조금 보수적으로)
        cfg.use_slip_detection = true;
        cfg.use_slip_guard = true;
        cfg.tangent_slip_threshold_m = 0.0010; // 1mm
        cfg.slip_guard_force_scale = 0.20;
        cfg.slip_guard_reanchor_tangent = true;
        cfg.slip_guard_velocity_damping = 0.15;

        // 현재 v18 측정값은 HandContactForceCallback에서 이미 HAND_BASE 기준으로 변환됨
        // (raw 3축 센서 토픽 연결 전까지는 base 입력 경로 사용)
        cfg.enable_sensor_raw_force_transform = false;

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

    // ----------------------------------------------------------------------
    // idle mode: one-shot sync target <- current
    // ----------------------------------------------------------------------
    if (current_control_mode_ == "idle" && !idle_synced_) {
        q_l_t_   = q_l_c_;
        q_r_t_   = q_r_c_;
        q_l_h_t_ = q_l_h_c_;
        q_r_h_t_ = q_r_h_c_;
        idle_synced_ = true;
    } else if (current_control_mode_ != "idle") {
        idle_synced_ = false;
    }

    // ----------------------------------------------------------------------
    // v19.1 patch: detect forcecon entry/exit INSIDE ControlLoop and latch hold snapshot
    //   -> no dependency on ControlModeCallback patch timing
    // ----------------------------------------------------------------------
    const bool in_forcecon = (current_control_mode_ == "forcecon");
    const bool entering_forcecon = ( in_forcecon && !forcecon_prev_cycle_);
    const bool leaving_forcecon  = (!in_forcecon &&  forcecon_prev_cycle_);

    if (entering_forcecon) {
        // capture current arm/hand as fixed hold reference for forcecon
        if (q_l_arm_forcecon_hold_.size() != 6)   q_l_arm_forcecon_hold_.setZero(6);
        if (q_r_arm_forcecon_hold_.size() != 6)   q_r_arm_forcecon_hold_.setZero(6);
        if (q_l_hand_forcecon_hold_.size() != 20) q_l_hand_forcecon_hold_.setZero(20);
        if (q_r_hand_forcecon_hold_.size() != 20) q_r_hand_forcecon_hold_.setZero(20);

        q_l_arm_forcecon_hold_  = q_l_c_;
        q_r_arm_forcecon_hold_  = q_r_c_;
        q_l_hand_forcecon_hold_ = q_l_h_c_;
        q_r_hand_forcecon_hold_ = q_r_h_c_;

        forcecon_hold_snapshot_valid_ = true;

        // immediately apply hold to targets (first forcecon tick부터 고정)
        q_l_t_   = q_l_arm_forcecon_hold_;
        q_r_t_   = q_r_arm_forcecon_hold_;
        q_l_h_t_ = q_l_hand_forcecon_hold_;
        q_r_h_t_ = q_r_hand_forcecon_hold_;

        if (node_) {
            RCLCPP_INFO(node_->get_logger(),
                        "[ControlLoop] Enter forcecon -> hold snapshot latched (arm/wrist fixed at entry state)");
        }
    }

    if (leaving_forcecon) {
        forcecon_hold_snapshot_valid_ = false;

        if (node_) {
            RCLCPP_INFO(node_->get_logger(),
                        "[ControlLoop] Leave forcecon -> hold snapshot invalidated");
        }
    }

    // update edge detector state
    forcecon_prev_cycle_ = in_forcecon;

    // ----------------------------------------------------------------------
    // v19.1 patch: while in forcecon, always start from hold snapshot (NOT q_*_c_)
    //   - arm stays completely fixed at forcecon-entry snapshot
    //   - hand starts from hold snapshot; selected finger only is overwritten below
    // ----------------------------------------------------------------------
    if (in_forcecon) {
        if (forcecon_hold_snapshot_valid_ &&
            q_l_arm_forcecon_hold_.size() == 6 &&
            q_r_arm_forcecon_hold_.size() == 6 &&
            q_l_hand_forcecon_hold_.size() == 20 &&
            q_r_hand_forcecon_hold_.size() == 20)
        {
            q_l_t_   = q_l_arm_forcecon_hold_;
            q_r_t_   = q_r_arm_forcecon_hold_;
            q_l_h_t_ = q_l_hand_forcecon_hold_;
            q_r_h_t_ = q_r_hand_forcecon_hold_;
        } else {
            // safe fallback (should rarely happen)
            q_l_t_   = q_l_c_;
            q_r_t_   = q_r_c_;
            q_l_h_t_ = q_l_h_c_;
            q_r_h_t_ = q_r_h_c_;
        }
    }

    // ----------------------------------------------------------------------
    // v19 patch: forcecon executes HERE at timer rate (100Hz-ish), not in callback
    // ----------------------------------------------------------------------
    if (current_control_mode_ == "forcecon" && hand_force_cmd_valid_) {
        const bool is_left = (hand_force_cmd_hand_id_ == 0);
        const int finger_id = hand_force_cmd_finger_id_;

        if (finger_id >= 0 && finger_id < 5) {
            auto& adm_arr = is_left ? hand_adm_l_ : hand_adm_r_;
            auto adm_ptr = adm_arr[static_cast<size_t>(finger_id)];

            if (adm_ptr && adm_ptr->isOk()) {
                // NOTE:
                // - arm/hand target base hold는 위(in_forcecon block)에서 이미 snapshot으로 적용됨
                // - 여기서는 selected finger만 q_h_t_를 갱신한다.
                // - current state q_h_c_는 측정값(실제 로봇 상태)로 유지 -> admittance/IK 입력으로 사용

                const Eigen::Matrix<double,5,3>& f_hand_cur = is_left ? f_l_hand_c_ : f_r_hand_c_;
                Eigen::VectorXd& q_h_c = is_left ? q_l_h_c_ : q_r_h_c_;
                Eigen::VectorXd& q_h_t = is_left ? q_l_h_t_ : q_r_h_t_;

                // q current (20DoF canonical)
                std::vector<double> q_cur20(20, 0.0);
                for (int i = 0; i < 20 && i < q_h_c.size(); ++i) q_cur20[i] = q_h_c(i);

                // dt estimate per (hand,finger)
                double dt_s = 0.01;
                if (node_) {
                    static std::array<bool,10> s_valid = {false,false,false,false,false,false,false,false,false,false};
                    static std::array<int64_t,10> s_last_ns = {0,0,0,0,0,0,0,0,0,0};

                    const int key = (is_left ? 0 : 5) + finger_id;
                    const int64_t now_ns = node_->get_clock()->now().nanoseconds();

                    if (s_valid[key]) {
                        dt_s = static_cast<double>(now_ns - s_last_ns[key]) * 1e-9;
                    }
                    s_last_ns[key] = now_ns;
                    s_valid[key] = true;

                    if (!std::isfinite(dt_s) || dt_s <= 0.0) dt_s = 0.01;
                    dt_s = std::max(1e-4, std::min(dt_s, 5e-2));
                }

                dualarm_forcecon::HandAdmittanceControl::StepInput in;
                in.p_des_base = hand_force_cmd_p_des_base_;
                in.f_des_base = hand_force_cmd_f_des_base_;

                // 현재 v18 측정 경로: HandContactForceCallback에서 이미 HAND_BASE로 변환된 값 사용
                in.f_meas_base = f_hand_cur.row(finger_id).transpose();

                // raw sensor 3축 경로는 추후 연결 가능 (hand_admittance_control.hpp에서 지원)
                in.use_f_meas_sensor_raw = false;
                in.f_meas_sensor_raw.setZero();

                in.q_hand_current20 = q_cur20;
                in.dt_s = dt_s;

                auto out = adm_ptr->step(in);

                if (out.controller_ok) {
                    // selected finger target joints only (q4 mimic)
                    const int b = finger_id * 4;
                    if (b + 3 < q_h_t.size()) {
                        q_h_t(b + 0) = out.q_cmd_123[0];
                        q_h_t(b + 1) = out.q_cmd_123[1];
                        q_h_t(b + 2) = out.q_cmd_123[2];
                        q_h_t(b + 3) = out.q_cmd_4_mimic;
                    }

                    // monitor target fingertip position: controller's actual p_cmd (not just requested p_des)
                    geometry_msgs::msg::Point pt;
                    pt.x = out.p_cmd_base.x();
                    pt.y = out.p_cmd_base.y();
                    pt.z = out.p_cmd_base.z();

                    if (is_left) {
                        switch (finger_id) {
                            case 0: t_f_l_thumb_  = pt; break;
                            case 1: t_f_l_index_  = pt; break;
                            case 2: t_f_l_middle_ = pt; break;
                            case 3: t_f_l_ring_   = pt; break;
                            case 4: t_f_l_baby_   = pt; break;
                            default: break;
                        }
                    } else {
                        switch (finger_id) {
                            case 0: t_f_r_thumb_  = pt; break;
                            case 1: t_f_r_index_  = pt; break;
                            case 2: t_f_r_middle_ = pt; break;
                            case 3: t_f_r_ring_   = pt; break;
                            case 4: t_f_r_baby_   = pt; break;
                            default: break;
                        }
                    }

                    // optional debug (decimated)
                    if (node_) {
                        static int dbg_decim = 0;
                        if ((dbg_decim++ % 50) == 0) {
                            RCLCPP_INFO(
                                node_->get_logger(),
                                "[ForceConLoop] side=%s finger=%d dt=%.4f ik_ok=%d contact=%d | p_cmd=(%.4f %.4f %.4f) | f_des=(%.3f %.3f %.3f) f_meas=(%.3f %.3f %.3f) | arm_hold=%d",
                                is_left ? "L" : "R",
                                finger_id,
                                dt_s,
                                static_cast<int>(out.ik_ok),
                                static_cast<int>(out.contact_on),
                                out.p_cmd_base.x(), out.p_cmd_base.y(), out.p_cmd_base.z(),
                                in.f_des_base.x(), in.f_des_base.y(), in.f_des_base.z(),
                                in.f_meas_base.x(), in.f_meas_base.y(), in.f_meas_base.z(),
                                static_cast<int>(forcecon_hold_snapshot_valid_));
                        }
                    }
                } else {
                    if (node_) {
                        static int warn_decim = 0;
                        if ((warn_decim++ % 100) == 0) {
                            RCLCPP_WARN(node_->get_logger(),
                                        "[ForceConLoop] hand admittance controller step failed (side=%s finger=%d)",
                                        is_left ? "L" : "R", finger_id);
                        }
                    }
                }
            }
        }
    }

    // ----------------------------------------------------------------------
    // Publish /isaac_joint_command from q_*_t_ (existing behavior)
    // ----------------------------------------------------------------------
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = node_->now();
    cmd.name = joint_names_;
    cmd.position.reserve(joint_names_.size());

    for (const auto& n : joint_names_) {
        // Arms
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

        // Hands (robust parsing)
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