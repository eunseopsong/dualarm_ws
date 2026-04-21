#ifndef DUALARM_FORCECON_CONTROL_HAND_ADMITTANCE_CONTROL_HPP_
#define DUALARM_FORCECON_CONTROL_HAND_ADMITTANCE_CONTROL_HPP_

#pragma once

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "dualarm_kinematics/hand/hand_forward_kinematics.hpp"
#include "dualarm_kinematics/hand/hand_inverse_kinematics.hpp"

namespace dualarm_forcecon {

class HandAdmittanceControl {
public:
    enum FingerId : int {
        THUMB  = 0,
        INDEX  = 1,
        MIDDLE = 2,
        RING   = 3,
        BABY   = 4
    };

    struct Config {
        // --------------------------------------------------------------------
        // Core MDK parameters (HAND BASE frame)
        // --------------------------------------------------------------------
        std::array<double,3> mass;
        std::array<double,3> damping;
        std::array<double,3> stiffness;

        // Base per-axis enable (hybrid mode may override to 1-axis)
        std::array<bool,3> force_ctrl_enable;

        // Hybrid force/position mode
        bool use_hybrid_force_position_mode;
        int  hybrid_force_axis;   // 0=x,1=y,2=z (HAND BASE)

        // Tangent anchor
        bool hold_tangent_anchor_on_contact;
        bool tangent_anchor_use_measured_pose;

        // Force signal preprocessing
        std::array<double,3> force_error_axis_sign;
        std::array<double,3> force_deadband_N;
        bool use_force_lpf;
        double force_lpf_tau_s;

        // Target force ramp
        std::array<double,3> force_target_ramp_rate_Nps;
        std::array<double,3> force_target_release_rate_Nps;

        // Safety / limits
        std::array<double,3> max_offset_m;
        std::array<double,3> max_step_m;
        std::array<double,3> max_adm_velocity_mps;

        double dt_min_s;
        double dt_max_s;

        // f_err definition
        bool force_error_des_minus_meas;

        // Contact detection / hysteresis
        double contact_force_threshold_N;
        bool use_contact_hysteresis;
        double contact_on_threshold_N;
        double contact_off_threshold_N;
        bool contact_gate_use_enabled_axes_only;

        // No-contact decay
        bool decay_when_no_contact;
        double no_contact_decay_ratio;

        // Slip detection / guard
        bool use_slip_detection;
        double tangent_slip_threshold_m;
        bool use_slip_guard;
        double slip_guard_force_scale;
        bool slip_guard_reanchor_tangent;
        double slip_guard_velocity_damping;

        // Anti-windup / state sync
        bool antiwindup_on_offset_clamp;
        bool zero_velocity_on_offset_clamp;
        double offset_clamp_velocity_damping;
        bool sync_adm_state_to_final_cmd;

        // IK solver options (kept for compatibility / staged migration)
        int    ik_max_iters;
        double ik_tol_pos_m;
        double ik_lambda;
        double ik_lambda_min;
        double ik_lambda_max;
        double ik_alpha;
        double ik_alpha_min;
        double ik_max_step;
        double ik_mu_posture;

        // IK fallback
        bool prefer_last_success_q_seed;
        bool keep_last_success_on_ik_fail;
        bool damp_velocity_on_ik_fail;
        double ik_fail_velocity_damping;

        // Raw sensor transform
        std::array<double,9> R_tip_sensor_rowmajor;
        std::array<double,9> R_base_corr_rowmajor;
        bool fallback_to_f_meas_base_if_sensor_transform_fails;

        // Debug
        int debug_decimation;

        // New: target force ~= 0 판정 threshold
        double force_target_zero_eps;

        // POD-style config loaded from YAML
        Config()
        : mass{{3.0, 3.0, 3.0}},
          damping{{500.0, 500.0, 500.0}},
          stiffness{{0.0, 0.0, 0.0}},
          force_ctrl_enable{{false, false, true}},
          use_hybrid_force_position_mode(true),
          hybrid_force_axis(2),
          hold_tangent_anchor_on_contact(true),
          tangent_anchor_use_measured_pose(true),
          force_error_axis_sign{{1.0, 1.0, 1.0}},
          force_deadband_N{{0.0, 0.0, 0.0}},
          use_force_lpf(true),
          force_lpf_tau_s(0.04),
          force_target_ramp_rate_Nps{{20.0, 20.0, 20.0}},
          force_target_release_rate_Nps{{40.0, 40.0, 40.0}},
          max_offset_m{{0.005, 0.005, 0.010}},
          max_step_m{{0.0003, 0.0003, 0.0003}},
          max_adm_velocity_mps{{0.01, 0.01, 0.01}},
          dt_min_s(1e-4),
          dt_max_s(5e-2),
          force_error_des_minus_meas(true),
          contact_force_threshold_N(0.5),
          use_contact_hysteresis(true),
          contact_on_threshold_N(0.7),
          contact_off_threshold_N(0.3),
          contact_gate_use_enabled_axes_only(true),
          decay_when_no_contact(true),
          no_contact_decay_ratio(0.90),
          use_slip_detection(true),
          tangent_slip_threshold_m(0.0015),
          use_slip_guard(true),
          slip_guard_force_scale(0.25),
          slip_guard_reanchor_tangent(true),
          slip_guard_velocity_damping(0.2),
          antiwindup_on_offset_clamp(true),
          zero_velocity_on_offset_clamp(true),
          offset_clamp_velocity_damping(0.2),
          sync_adm_state_to_final_cmd(true),
          ik_max_iters(80),
          ik_tol_pos_m(5e-4),
          ik_lambda(1e-2),
          ik_lambda_min(1e-5),
          ik_lambda_max(1.0),
          ik_alpha(0.8),
          ik_alpha_min(0.05),
          ik_max_step(0.15),
          ik_mu_posture(1e-4),
          prefer_last_success_q_seed(true),
          keep_last_success_on_ik_fail(true),
          damp_velocity_on_ik_fail(true),
          ik_fail_velocity_damping(0.2),
          R_tip_sensor_rowmajor{{1,0,0, 0,1,0, 0,0,1}},
          R_base_corr_rowmajor{{1,0,0, 0,1,0, 0,0,1}},
          fallback_to_f_meas_base_if_sensor_transform_fails(true),
          debug_decimation(50),
          force_target_zero_eps(1e-9)
        {}
    };

    struct StepInput {
        // --------------------------------------------------------------------
        // Preferred new inputs (for v25-style unified pipeline)
        // --------------------------------------------------------------------
        Eigen::Vector3d x_d_base{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())}; // target fingertip pos
        Eigen::Vector3d x_0_base{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())}; // current fingertip pos
        Eigen::Vector3d f_d_base{Eigen::Vector3d::Zero()};   // desired force in hand base frame
        Eigen::Vector3d f_ext_base{Eigen::Vector3d::Zero()}; // measured external/contact force in hand base frame

        // Optional raw sensor route (if provided, and transform info exists)
        bool has_f_sensor{false};
        Eigen::Vector3d f_sensor{Eigen::Vector3d::Zero()};   // raw sensor-frame force
        bool has_R_base_tip{false};
        Eigen::Matrix3d R_base_tip{Eigen::Matrix3d::Identity()}; // tip->base rotation

        // Current hand joints (20DoF preferred)
        std::vector<double> q_hand_current20;

        // dt
        double dt_s{0.001};

        // --------------------------------------------------------------------
        // Backward-compatible aliases (kept so old src still compiles)
        // --------------------------------------------------------------------
        Eigen::Vector3d p_des_base{Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};  // old alias of x_d_base
        Eigen::Vector3d f_des_base{Eigen::Vector3d::Zero()};   // old alias of f_d_base
        Eigen::Vector3d f_meas_base{Eigen::Vector3d::Zero()};  // old alias of f_ext_base
    };

    struct StepOutput {
        bool controller_ok{false};
        bool ik_ok{false};

        bool contact_on{false};
        bool contact_rising_edge{false};
        bool contact_falling_edge{false};
        double used_dt_s{0.0};

        // --------------------------------------------------------------------
        // Preferred new fields
        // --------------------------------------------------------------------
        Eigen::Vector3d x_0_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d x_d_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d x_ref_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d x_cmd_base{Eigen::Vector3d::Zero()};

        Eigen::Vector3d f_ext_base_raw{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_ext_base_filt{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_ext_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_d_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_d_ramped{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_d_used{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_err{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_drive{Eigen::Vector3d::Zero()};

        Eigen::Vector3d adm_offset{Eigen::Vector3d::Zero()};
        Eigen::Vector3d adm_velocity{Eigen::Vector3d::Zero()};
        Eigen::Vector3d adm_acceleration{Eigen::Vector3d::Zero()};
        Eigen::Vector3d k_eff{Eigen::Vector3d::Zero()};

        std::array<bool,3> axis_force_enable_eff{{false,false,false}};
        int hybrid_force_axis{-1};

        std::array<bool,3> offset_clamped{{false,false,false}};
        std::array<bool,3> step_limited{{false,false,false}};

        bool tangent_anchor_valid{false};
        Eigen::Vector3d tangent_anchor_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d tangent_drift_vec{Eigen::Vector3d::Zero()};
        double tangent_drift_norm{0.0};
        bool slip_detected{false};

        // --------------------------------------------------------------------
        // Backward-compatible aliases
        // --------------------------------------------------------------------
        Eigen::Vector3d p_meas_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_des_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_ref_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_cmd_base{Eigen::Vector3d::Zero()};

        Eigen::Vector3d f_meas_base_raw{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base_filt{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_des_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_des_used{Eigen::Vector3d::Zero()};

        // --------------------------------------------------------------------
        // IK compatibility output (kept for staged src migration)
        // --------------------------------------------------------------------
        std::array<double,3> q_cmd_123 {{0.0, 0.0, 0.0}};
        double q_cmd_4_mimic{0.0};
        std::vector<double> q_cmd20;
    };

public:
    HandAdmittanceControl() = default;

    HandAdmittanceControl(std::shared_ptr<HandForwardKinematics> hand_fk,
                          std::shared_ptr<HandInverseKinematics> hand_ik,
                          int finger_id)
    {
        initialize(hand_fk, hand_ik, finger_id, Config{});
    }

    HandAdmittanceControl(std::shared_ptr<HandForwardKinematics> hand_fk,
                          std::shared_ptr<HandInverseKinematics> hand_ik,
                          int finger_id,
                          const Config& cfg)
    {
        initialize(hand_fk, hand_ik, finger_id, cfg);
    }

    bool initialize(std::shared_ptr<HandForwardKinematics> hand_fk,
                    std::shared_ptr<HandInverseKinematics> hand_ik,
                    int finger_id)
    {
        return initialize(hand_fk, hand_ik, finger_id, Config{});
    }

    bool initialize(std::shared_ptr<HandForwardKinematics> hand_fk,
                    std::shared_ptr<HandInverseKinematics> hand_ik,
                    int finger_id,
                    const Config& cfg)
    {
        hand_fk_ = hand_fk;
        hand_ik_ = hand_ik;
        cfg_ = cfg;

        finger_id_ = clampFingerId_(finger_id);
        initialized_ = false;
        last_success_valid_ = false;

        adm_x_.setZero();
        adm_v_.setZero();
        adm_a_.setZero();
        p_cmd_prev_.setZero();

        f_meas_filt_.setZero();
        f_d_ramped_.setZero();
        f_lpf_initialized_ = false;

        contact_state_ = false;
        prev_contact_state_ = false;

        tangent_anchor_valid_ = false;
        tangent_anchor_p_.setZero();

        last_q_cmd_123_ = {{0.0, 0.0, 0.0}};
        last_q_cmd20_.clear();

        debug_counter_ = 0;

        const bool ok_fk = (hand_fk_ != nullptr && hand_fk_->ok());
        const bool ok_ik = (hand_ik_ != nullptr && hand_ik_->ok());
        ok_ = ok_fk && ok_ik;

        return ok_;
    }

    bool isOk() const { return ok_; }

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }

    int fingerId() const { return finger_id_; }

    void setDebugLogger(const rclcpp::Logger& logger) {
        debug_logger_ = logger;
        has_debug_logger_ = true;
    }

    void resetState()
    {
        initialized_ = false;
        adm_x_.setZero();
        adm_v_.setZero();
        adm_a_.setZero();
        p_cmd_prev_.setZero();

        f_meas_filt_.setZero();
        f_d_ramped_.setZero();
        f_lpf_initialized_ = false;

        contact_state_ = false;
        prev_contact_state_ = false;

        tangent_anchor_valid_ = false;
        tangent_anchor_p_.setZero();
    }

    void resetState(const Eigen::Vector3d& p_ref_base)
    {
        initialized_ = true;
        adm_x_.setZero();
        adm_v_.setZero();
        adm_a_.setZero();
        p_cmd_prev_ = p_ref_base;

        f_meas_filt_.setZero();
        f_d_ramped_.setZero();
        f_lpf_initialized_ = false;

        contact_state_ = false;
        prev_contact_state_ = false;

        tangent_anchor_valid_ = false;
        tangent_anchor_p_.setZero();
    }

    static std::string fingerName(int fid)
    {
        switch (fid) {
            case THUMB:  return "thumb";
            case INDEX:  return "index";
            case MIDDLE: return "middle";
            case RING:   return "ring";
            case BABY:   return "baby";
            default:     return "unknown";
        }
    }

    StepOutput step(const StepInput& in)
    {
        StepOutput out;
        out.controller_ok = false;
        out.ik_ok = false;

        if (!ok_ || !hand_fk_ || !hand_ik_) return out;
        if (in.q_hand_current20.size() < 15) return out;

        out.controller_ok = true;

        // --------------------------------------------------------------------
        // STEP 0) dt stabilize
        // --------------------------------------------------------------------
        const double dt = clampScalar_(in.dt_s, cfg_.dt_min_s, cfg_.dt_max_s);
        out.used_dt_s = dt;

        // --------------------------------------------------------------------
        // STEP 1) current fingertip (x0) and desired fingertip (xd)
        // --------------------------------------------------------------------
        const std::vector<Eigen::Vector3d> tips_cur = hand_fk_->computeFingertips(in.q_hand_current20);
        const Eigen::Vector3d p_meas_fk = safeTip_(tips_cur, finger_id_);

        const Eigen::Vector3d x0 = chooseCurrentPose_(in, p_meas_fk);
        const Eigen::Vector3d xd = chooseDesiredPose_(in);

        out.x_0_base = x0;
        out.x_d_base = xd;

        out.p_meas_base = x0;
        out.p_des_base  = xd;

        // --------------------------------------------------------------------
        // STEP 2) measured force input (preferred: raw sensor -> transform, else base)
        // --------------------------------------------------------------------
        const Eigen::Vector3d f_meas_input_base = resolveMeasuredForceBase_(in);
        out.f_ext_base_raw  = f_meas_input_base;
        out.f_meas_base_raw = f_meas_input_base;

        // --------------------------------------------------------------------
        // STEP 2.1) desired force input
        // --------------------------------------------------------------------
        const Eigen::Vector3d f_des_in = chooseDesiredForce_(in);
        out.f_d_base   = f_des_in;
        out.f_des_base = f_des_in;

        // --------------------------------------------------------------------
        // STEP 2.2) LPF on measured force
        // --------------------------------------------------------------------
        Eigen::Vector3d f_meas_used = f_meas_input_base;
        if (cfg_.use_force_lpf) {
            const double tau = std::max(1e-6, cfg_.force_lpf_tau_s);
            const double alpha = clampScalar_(dt / (tau + dt), 0.0, 1.0);

            if (!f_lpf_initialized_) {
                f_meas_filt_ = f_meas_input_base;
                f_lpf_initialized_ = true;
            } else {
                f_meas_filt_ = (1.0 - alpha) * f_meas_filt_ + alpha * f_meas_input_base;
            }
            f_meas_used = f_meas_filt_;
        } else {
            f_meas_filt_ = f_meas_input_base;
            f_lpf_initialized_ = true;
            f_meas_used = f_meas_input_base;
        }

        out.f_ext_base_filt = f_meas_filt_;
        out.f_ext_base      = f_meas_used;

        out.f_meas_base_filt = f_meas_filt_;
        out.f_meas_base      = f_meas_used;

        // --------------------------------------------------------------------
        // STEP 2.3) Hybrid axis
        // --------------------------------------------------------------------
        std::array<bool,3> axis_force_enable_eff = cfg_.force_ctrl_enable;
        int hybrid_force_axis = -1;

        if (cfg_.use_hybrid_force_position_mode) {
            hybrid_force_axis = clampAxis_(cfg_.hybrid_force_axis);
            axis_force_enable_eff = {{false, false, false}};
            axis_force_enable_eff[static_cast<std::size_t>(hybrid_force_axis)] = true;
        }
        out.axis_force_enable_eff = axis_force_enable_eff;
        out.hybrid_force_axis     = hybrid_force_axis;

        // --------------------------------------------------------------------
        // STEP 2.4) Force target ramp / release
        // --------------------------------------------------------------------
        const Eigen::Vector3d f_des_ramped = rampDesiredForce_(f_des_in, dt);
        out.f_d_ramped = f_des_ramped;

        // --------------------------------------------------------------------
        // STEP 3) Contact detection
        // --------------------------------------------------------------------
        const double f_metric = contactMetricNorm_(f_meas_used, axis_force_enable_eff);
        prev_contact_state_ = contact_state_;

        if (cfg_.use_contact_hysteresis) {
            const double on_thr  = std::max(0.0, cfg_.contact_on_threshold_N);
            const double off_thr = std::max(0.0, std::min(on_thr, cfg_.contact_off_threshold_N));
            if (contact_state_) {
                if (f_metric <= off_thr) contact_state_ = false;
            } else {
                if (f_metric >= on_thr) contact_state_ = true;
            }
            out.contact_on = contact_state_;
        } else {
            out.contact_on = (f_metric >= std::max(0.0, cfg_.contact_force_threshold_N));
            contact_state_ = out.contact_on;
        }

        out.contact_rising_edge  = (!prev_contact_state_ && out.contact_on);
        out.contact_falling_edge = ( prev_contact_state_ && !out.contact_on);

        // --------------------------------------------------------------------
        // STEP 3.5) Tangent anchor
        //   - x_ref is the position reference used on position-controlled axes.
        //   - for force-enabled axis, dynamics uses x_d/x_0 directly.
        // --------------------------------------------------------------------
        Eigen::Vector3d x_ref = xd;

        if (cfg_.hold_tangent_anchor_on_contact) {
            if (out.contact_rising_edge) {
                tangent_anchor_valid_ = true;
                tangent_anchor_p_ = cfg_.tangent_anchor_use_measured_pose ? x0 : xd;
            }

            if (out.contact_on && tangent_anchor_valid_) {
                for (int ax = 0; ax < 3; ++ax) {
                    if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                        x_ref(ax) = tangent_anchor_p_(ax);
                    }
                }
            }
        }

        out.x_ref_base = x_ref;
        out.p_ref_base = x_ref;

        out.tangent_anchor_valid = tangent_anchor_valid_;
        out.tangent_anchor_base  = tangent_anchor_p_;

        // --------------------------------------------------------------------
        // STEP 3.7) Slip detection / guard
        // --------------------------------------------------------------------
        Eigen::Vector3d tangent_drift = Eigen::Vector3d::Zero();
        double tangent_drift_norm = 0.0;
        bool slip_detected = false;

        if (cfg_.use_slip_detection && out.contact_on && tangent_anchor_valid_) {
            for (int ax = 0; ax < 3; ++ax) {
                if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    tangent_drift(ax) = x0(ax) - tangent_anchor_p_(ax);
                }
            }
            tangent_drift_norm = tangent_drift.norm();
            slip_detected = (tangent_drift_norm > std::max(0.0, cfg_.tangent_slip_threshold_m));
        }

        Eigen::Vector3d f_des_used = f_des_ramped;
        if (cfg_.use_slip_guard && slip_detected) {
            const double s = clampScalar_(cfg_.slip_guard_force_scale, 0.0, 1.0);
            for (int ax = 0; ax < 3; ++ax) {
                if (axis_force_enable_eff[static_cast<std::size_t>(ax)]) f_des_used(ax) *= s;
                else f_des_used(ax) = 0.0;
            }
            const double rv = clampScalar_(cfg_.slip_guard_velocity_damping, 0.0, 1.0);
            adm_v_ *= rv;

            if (cfg_.slip_guard_reanchor_tangent) {
                tangent_anchor_valid_ = true;
                tangent_anchor_p_ = x0;
                out.tangent_anchor_valid = true;
                out.tangent_anchor_base = tangent_anchor_p_;
            }
        }

        out.tangent_drift_vec  = tangent_drift;
        out.tangent_drift_norm = tangent_drift_norm;
        out.slip_detected      = slip_detected;
        out.f_d_used           = f_des_used;
        out.f_des_used         = f_des_used;

        // --------------------------------------------------------------------
        // STEP 4) Force error / drive
        //   keep v24 sign/deadband/hybrid logic as much as possible
        // --------------------------------------------------------------------
        Eigen::Vector3d f_err;
        if (cfg_.force_error_des_minus_meas) f_err = (f_des_used - f_meas_used);
        else                                 f_err = (f_meas_used - f_des_used);
        out.f_err = f_err;

        Eigen::Vector3d f_drive = f_err;
        for (int ax = 0; ax < 3; ++ax) {
            if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                f_drive(ax) = 0.0;
                continue;
            }

            const double db = std::max(0.0, cfg_.force_deadband_N[ax]);
            if (std::fabs(f_drive(ax)) <= db) f_drive(ax) = 0.0;
            else f_drive(ax) = (f_drive(ax) > 0.0) ? (f_drive(ax) - db) : (f_drive(ax) + db);

            const double sgn = (cfg_.force_error_axis_sign[ax] >= 0.0) ? 1.0 : -1.0;
            f_drive(ax) *= sgn;
        }
        out.f_drive = f_drive;

        // --------------------------------------------------------------------
        // STEP 5) init
        //   state is defined as displacement around current measured pose x0:
        //   x_cmd = x0 + adm_x  (force axis)
        // --------------------------------------------------------------------
        if (!initialized_) {
            initialized_ = true;
            adm_x_.setZero();
            adm_v_.setZero();
            adm_a_.setZero();
            p_cmd_prev_ = x0;
        }

        // --------------------------------------------------------------------
        // STEP 6) unified admittance dynamics
        //
        //   M * x_ddot + D * x_dot = f_drive + K_eff * (x_d - x_0)
        //
        //   where
        //     - x_d : target position
        //     - x_0 : current measured position
        //     - K_eff = stiffness (if |F_d| ~ 0), else 0
        //
        //   This realizes:
        //     - target_force == 0  -> position-control-like behavior
        //     - target_force != 0  -> force-control-like behavior
        // --------------------------------------------------------------------
        Eigen::Vector3d x_cmd = x_ref;
        Eigen::Vector3d k_eff = Eigen::Vector3d::Zero();

        for (int ax = 0; ax < 3; ++ax) {
            const bool axis_force_on = axis_force_enable_eff[static_cast<std::size_t>(ax)];

            if (!axis_force_on) {
                adm_x_(ax) = 0.0;
                adm_v_(ax) = 0.0;
                adm_a_(ax) = 0.0;
                x_cmd(ax) = x_ref(ax);
                continue;
            }

            const double M = std::max(1e-6, cfg_.mass[ax]);
            const double D = std::max(0.0,  cfg_.damping[ax]);

            const bool force_target_is_zero = (std::fabs(f_des_used(ax)) <= std::max(0.0, cfg_.force_target_zero_eps));
            const double K_eff = force_target_is_zero ? std::max(0.0, cfg_.stiffness[ax]) : 0.0;
            k_eff(ax) = K_eff;

            // no-contact decay (keep active cfg behavior)
            if (!out.contact_on && cfg_.decay_when_no_contact && !force_target_is_zero) {
                const double r = clampScalar_(cfg_.no_contact_decay_ratio, 0.0, 1.0);
                adm_x_(ax) *= r;
                adm_v_(ax) *= r;
            }

            const double pos_err = xd(ax) - x0(ax);
            const double xdd = (f_drive(ax) + K_eff * pos_err - D * adm_v_(ax)) / M;

            double xd_new = adm_v_(ax) + xdd * dt;

            const double vmax = std::fabs(cfg_.max_adm_velocity_mps[ax]);
            if (vmax > 0.0) xd_new = clampScalar_(xd_new, -vmax, vmax);

            double x_new = adm_x_(ax) + xd_new * dt;

            const double x_lim = std::fabs(cfg_.max_offset_m[ax]);
            bool offset_clamped = false;
            if (x_lim > 0.0) {
                const double x_clamped = clampScalar_(x_new, -x_lim, x_lim);
                offset_clamped = (std::fabs(x_clamped - x_new) > 1e-15);
                x_new = x_clamped;
            }

            if (cfg_.antiwindup_on_offset_clamp && offset_clamped) {
                if (cfg_.zero_velocity_on_offset_clamp) xd_new = 0.0;
                else {
                    const double r = clampScalar_(cfg_.offset_clamp_velocity_damping, 0.0, 1.0);
                    xd_new *= r;
                }
            }

            adm_a_(ax) = xdd;
            adm_v_(ax) = xd_new;
            adm_x_(ax) = x_new;
            out.offset_clamped[static_cast<std::size_t>(ax)] = offset_clamped;

            x_cmd(ax) = x0(ax) + adm_x_(ax);
        }

        // --------------------------------------------------------------------
        // STEP 6.5) step clamp on final Cartesian command
        // --------------------------------------------------------------------
        for (int ax = 0; ax < 3; ++ax) {
            const double dmax = std::fabs(cfg_.max_step_m[ax]);
            const double d = x_cmd(ax) - p_cmd_prev_(ax);
            double d_applied = d;

            if (dmax > 0.0) {
                d_applied = clampScalar_(d, -dmax, dmax);
                out.step_limited[static_cast<std::size_t>(ax)] = (std::fabs(d_applied - d) > 1e-15);
            }
            x_cmd(ax) = p_cmd_prev_(ax) + d_applied;
        }

        if (cfg_.sync_adm_state_to_final_cmd) {
            for (int ax = 0; ax < 3; ++ax) {
                if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    adm_x_(ax) = 0.0;
                    adm_v_(ax) = 0.0;
                    adm_a_(ax) = 0.0;
                    continue;
                }
                adm_x_(ax) = x_cmd(ax) - x0(ax);
                if (out.step_limited[static_cast<std::size_t>(ax)]) adm_v_(ax) *= 0.5;
            }
        }

        out.k_eff = k_eff;
        out.adm_offset = adm_x_;
        out.adm_velocity = adm_v_;
        out.adm_acceleration = adm_a_;

        out.x_cmd_base = x_cmd;
        out.p_cmd_base = x_cmd;

        // --------------------------------------------------------------------
        // STEP 7) IK compatibility path
        //   kept so current src can continue using q_cmd20 / q_cmd_123
        //   later src patch can ignore this and use x_cmd directly.
        // --------------------------------------------------------------------
        std::array<Eigen::Vector3d,5> tips_target;
        for (int i = 0; i < 5; ++i) tips_target[i] = safeTip_(tips_cur, i);
        tips_target[static_cast<std::size_t>(finger_id_)] = x_cmd;

        HandInverseKinematics::Options ik_opt;
        ik_opt.max_iters   = cfg_.ik_max_iters;
        ik_opt.tol_pos_m   = cfg_.ik_tol_pos_m;
        ik_opt.lambda      = cfg_.ik_lambda;
        ik_opt.lambda_min  = cfg_.ik_lambda_min;
        ik_opt.lambda_max  = cfg_.ik_lambda_max;
        ik_opt.alpha       = cfg_.ik_alpha;
        ik_opt.alpha_min   = cfg_.ik_alpha_min;
        ik_opt.max_step    = cfg_.ik_max_step;
        ik_opt.mu_posture  = cfg_.ik_mu_posture;
        ik_opt.verbose     = false;

        ik_opt.mask = {{false,false,false,false,false}};
        ik_opt.weights = {{0.0,0.0,0.0,0.0,0.0}};
        ik_opt.mask[static_cast<std::size_t>(finger_id_)] = true;
        ik_opt.weights[static_cast<std::size_t>(finger_id_)] = 1.0;

        std::vector<double> q_seed = in.q_hand_current20;
        if (cfg_.prefer_last_success_q_seed && last_success_valid_ && last_q_cmd20_.size() >= 20) {
            q_seed = last_q_cmd20_;
        }

        std::vector<double> q_sol20;
        const bool ik_ok = hand_ik_->solveIKFingertips(q_seed, tips_target, q_sol20, ik_opt);
        out.ik_ok = ik_ok;

        if (ik_ok && q_sol20.size() >= 20) {
            out.q_cmd20 = q_sol20;
            const int qb = finger_id_ * 4;
            out.q_cmd_123[0] = q_sol20[qb + 0];
            out.q_cmd_123[1] = q_sol20[qb + 1];
            out.q_cmd_123[2] = q_sol20[qb + 2];
            out.q_cmd_4_mimic = q_sol20[qb + 2];

            last_success_valid_ = true;
            last_q_cmd_123_ = out.q_cmd_123;
            last_q_cmd20_ = q_sol20;

            p_cmd_prev_ = x_cmd;
            maybeDebugPrint_(out);
            return out;
        }

        // --------------------------------------------------------------------
        // STEP 7.5) IK fail fallback
        // --------------------------------------------------------------------
        if (cfg_.damp_velocity_on_ik_fail) {
            const double r = clampScalar_(cfg_.ik_fail_velocity_damping, 0.0, 1.0);
            adm_v_ *= r;
        }

        out.q_cmd20 = in.q_hand_current20;
        if (out.q_cmd20.size() < 20) out.q_cmd20.resize(20, 0.0);

        const int qb = finger_id_ * 4;
        if (cfg_.keep_last_success_on_ik_fail && last_success_valid_) {
            out.q_cmd_123 = last_q_cmd_123_;
            out.q_cmd_4_mimic = last_q_cmd_123_[2];

            out.q_cmd20[qb + 0] = out.q_cmd_123[0];
            out.q_cmd20[qb + 1] = out.q_cmd_123[1];
            out.q_cmd20[qb + 2] = out.q_cmd_123[2];
            out.q_cmd20[qb + 3] = out.q_cmd_4_mimic;
        } else {
            if (static_cast<int>(in.q_hand_current20.size()) >= qb + 3) {
                out.q_cmd_123[0] = in.q_hand_current20[qb + 0];
                out.q_cmd_123[1] = in.q_hand_current20[qb + 1];
                out.q_cmd_123[2] = in.q_hand_current20[qb + 2];
                out.q_cmd_4_mimic = in.q_hand_current20[qb + 2];
            }
        }

        p_cmd_prev_ = x_cmd;
        maybeDebugPrint_(out);
        return out;
    }

private:
    static int clampFingerId_(int fid) { return std::max(0, std::min(4, fid)); }
    static int clampAxis_(int ax) { return std::max(0, std::min(2, ax)); }
    static double clampScalar_(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

    static Eigen::Vector3d safeTip_(const std::vector<Eigen::Vector3d>& tips, int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(tips.size())) return Eigen::Vector3d::Zero();
        return tips[static_cast<std::size_t>(idx)];
    }

    static bool isFiniteVec3_(const Eigen::Vector3d& v)
    {
        return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
    }

    static bool isFiniteMat3_(const Eigen::Matrix3d& M)
    {
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                if (!std::isfinite(M(r,c))) return false;
            }
        }
        return true;
    }

    static Eigen::Matrix3d rowMajorArrayToMat3_(const std::array<double,9>& a)
    {
        Eigen::Matrix3d R;
        R << a[0], a[1], a[2],
             a[3], a[4], a[5],
             a[6], a[7], a[8];
        return R;
    }

    Eigen::Vector3d chooseDesiredPose_(const StepInput& in) const
    {
        if (isFiniteVec3_(in.x_d_base)) return in.x_d_base;
        if (isFiniteVec3_(in.p_des_base)) return in.p_des_base;
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d chooseCurrentPose_(const StepInput& in, const Eigen::Vector3d& fk_pose) const
    {
        if (isFiniteVec3_(in.x_0_base)) return in.x_0_base;
        return fk_pose;
    }

    Eigen::Vector3d chooseDesiredForce_(const StepInput& in) const
    {
        // prefer new field if non-zero, otherwise fallback to old alias
        if (in.f_d_base.norm() > 0.0) return in.f_d_base;
        return in.f_des_base;
    }

    Eigen::Vector3d resolveMeasuredForceBase_(const StepInput& in) const
    {
        // If explicit sensor-frame info is provided, use configured transforms:
        // f_base = R_base_corr * R_base_tip * R_tip_sensor * f_sensor
        if (in.has_f_sensor && isFiniteVec3_(in.f_sensor) && in.has_R_base_tip && isFiniteMat3_(in.R_base_tip)) {
            const Eigen::Matrix3d R_tip_sensor = rowMajorArrayToMat3_(cfg_.R_tip_sensor_rowmajor);
            const Eigen::Matrix3d R_base_corr  = rowMajorArrayToMat3_(cfg_.R_base_corr_rowmajor);

            if (isFiniteMat3_(R_tip_sensor) && isFiniteMat3_(R_base_corr)) {
                return R_base_corr * in.R_base_tip * R_tip_sensor * in.f_sensor;
            }

            if (!cfg_.fallback_to_f_meas_base_if_sensor_transform_fails) {
                return Eigen::Vector3d::Zero();
            }
        }

        // Fallback to already-base-frame measurement
        if (isFiniteVec3_(in.f_ext_base)) return in.f_ext_base;
        if (isFiniteVec3_(in.f_meas_base)) return in.f_meas_base;
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d rampDesiredForce_(const Eigen::Vector3d& target, double dt)
    {
        Eigen::Vector3d out = f_d_ramped_;

        for (int ax = 0; ax < 3; ++ax) {
            const double cur = f_d_ramped_(ax);
            const double tgt = target(ax);

            const double abs_cur = std::fabs(cur);
            const double abs_tgt = std::fabs(tgt);

            const bool increasing = (abs_tgt > abs_cur + 1e-15);
            const double rate = increasing
                ? std::max(0.0, cfg_.force_target_ramp_rate_Nps[ax])
                : std::max(0.0, cfg_.force_target_release_rate_Nps[ax]);

            if (rate <= 0.0) {
                out(ax) = tgt;
                continue;
            }

            const double step = rate * dt;
            const double diff = tgt - cur;
            out(ax) = cur + clampScalar_(diff, -step, step);
        }

        f_d_ramped_ = out;
        return out;
    }

    double contactMetricNorm_(const Eigen::Vector3d& f,
                              const std::array<bool,3>& axis_force_enable_eff) const
    {
        if (!cfg_.contact_gate_use_enabled_axes_only) return f.norm();

        double s2 = 0.0;
        bool any = false;
        for (int ax = 0; ax < 3; ++ax) {
            if (axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                s2 += f(ax) * f(ax);
                any = true;
            }
        }
        return any ? std::sqrt(s2) : f.norm();
    }

    void maybeDebugPrint_(const StepOutput& out)
    {
        if (!has_debug_logger_) return;

        debug_counter_++;
        const int decim = std::max(1, cfg_.debug_decimation);
        if ((debug_counter_ % decim) != 0) return;

        // RCLCPP_INFO(
        //     debug_logger_,
        //     "[HandAdm][%s] x0=(%.4f %.4f %.4f) xd=(%.4f %.4f %.4f) xcmd=(%.4f %.4f %.4f) "
        //     "Fext=(%.3f %.3f %.3f) Fd=(%.3f %.3f %.3f) K=(%.3f %.3f %.3f) contact=%d slip=%d",
        //     fingerName(finger_id_).c_str(),
        //     out.x_0_base.x(), out.x_0_base.y(), out.x_0_base.z(),
        //     out.x_d_base.x(), out.x_d_base.y(), out.x_d_base.z(),
        //     out.x_cmd_base.x(), out.x_cmd_base.y(), out.x_cmd_base.z(),
        //     out.f_ext_base.x(), out.f_ext_base.y(), out.f_ext_base.z(),
        //     out.f_d_used.x(), out.f_d_used.y(), out.f_d_used.z(),
        //     out.k_eff.x(), out.k_eff.y(), out.k_eff.z(),
        //     static_cast<int>(out.contact_on),
        //     static_cast<int>(out.slip_detected)
        // );
    }

private:
    bool ok_{false};
    bool initialized_{false};

    std::shared_ptr<HandForwardKinematics> hand_fk_;
    std::shared_ptr<HandInverseKinematics> hand_ik_;

    int finger_id_{RING};
    Config cfg_;

    // Internal admittance state:
    // x_cmd(force-axis) = x0 + adm_x
    Eigen::Vector3d adm_x_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d adm_v_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d adm_a_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d p_cmd_prev_{Eigen::Vector3d::Zero()};

    Eigen::Vector3d f_meas_filt_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d f_d_ramped_{Eigen::Vector3d::Zero()};
    bool f_lpf_initialized_{false};

    bool contact_state_{false};
    bool prev_contact_state_{false};

    bool tangent_anchor_valid_{false};
    Eigen::Vector3d tangent_anchor_p_{Eigen::Vector3d::Zero()};

    // IK compatibility cache
    bool last_success_valid_{false};
    std::array<double,3> last_q_cmd_123_ {{0.0, 0.0, 0.0}};
    std::vector<double> last_q_cmd20_;

    rclcpp::Logger debug_logger_{rclcpp::get_logger("HandAdmittanceControl")};
    bool has_debug_logger_{false};
    int debug_counter_{0};
};

} // namespace dualarm_forcecon

#endif // DUALARM_FORCECON_CONTROL_HAND_ADMITTANCE_CONTROL_HPP_