#ifndef DUALARM_FORCECON_KINEMATICS_HAND_ADMITTANCE_CONTROL_HPP_
#define DUALARM_FORCECON_KINEMATICS_HAND_ADMITTANCE_CONTROL_HPP_

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

#include "dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_inverse_kinematics.hpp"

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
        // MDK parameters (x,y,z in HAND BASE frame)
        std::array<double,3> mass;
        std::array<double,3> damping;
        std::array<double,3> stiffness;

        // Base per-axis enable (hybrid mode may override to 1-axis)
        std::array<bool,3> force_ctrl_enable;

        // Hybrid force/position mode
        bool use_hybrid_force_position_mode;
        int  hybrid_force_axis;   // 0=x,1=y,2=z (HAND BASE)
        bool hold_tangent_anchor_on_contact;
        bool tangent_anchor_use_measured_pose;
        bool release_tangent_anchor_on_contact_off;

        // Force signal preprocessing
        std::array<double,3> force_error_axis_sign;
        std::array<double,3> force_deadband_N;
        bool use_force_lpf;
        double force_lpf_tau_s;

        bool use_force_target_ramp;
        bool force_ramp_only_when_contact;
        std::array<double,3> force_target_ramp_rate_Nps;
        std::array<double,3> force_target_release_rate_Nps;

        // Optional raw-sensor-force -> HAND_BASE transform (future)
        bool enable_sensor_raw_force_transform;
        std::array<double,9> R_tip_sensor_rowmajor; // row-major 3x3
        std::array<double,9> R_base_corr_rowmajor;  // row-major 3x3
        bool fallback_to_f_meas_base_if_sensor_transform_fails;

        // Safety / limits
        std::array<double,3> max_offset_m;
        std::array<double,3> max_step_m;
        std::array<double,3> max_adm_velocity_mps;

        double dt_min_s;
        double dt_max_s;

        // f_err definition
        bool force_error_des_minus_meas;

        // Contact gate
        bool use_contact_gate;
        double contact_force_threshold_N;

        bool use_contact_hysteresis;
        double contact_on_threshold_N;
        double contact_off_threshold_N;

        bool contact_gate_use_enabled_axes_only;

        bool decay_when_no_contact;
        double no_contact_decay_ratio;

        bool antiwindup_on_offset_clamp;
        bool zero_velocity_on_offset_clamp;
        double offset_clamp_velocity_damping;

        bool sync_adm_state_to_final_cmd;

        // Slip detection / guard
        bool use_slip_detection;
        double tangent_slip_threshold_m;
        bool use_slip_guard;
        double slip_guard_force_scale;
        bool slip_guard_reanchor_tangent;
        double slip_guard_velocity_damping;

        // IK solver options
        int    ik_max_iters;
        double ik_tol_pos_m;
        double ik_lambda;
        double ik_lambda_min;
        double ik_lambda_max;
        double ik_alpha;
        double ik_alpha_min;
        double ik_max_step;
        double ik_mu_posture;
        bool   ik_verbose;

        bool prefer_last_success_q_seed;

        bool keep_last_success_on_ik_fail;
        bool damp_velocity_on_ik_fail;
        double ik_fail_velocity_damping;

        // Debug
        bool debug_enable_rclcpp;
        int  debug_decimation;
        bool debug_print_all_steps;
        bool verbose;

        Config()
        : mass{{0.05, 0.05, 0.03}},
          damping{{8.0, 8.0, 10.0}},
          stiffness{{0.0, 0.0, 0.0}},
          force_ctrl_enable{{false, false, true}},
          use_hybrid_force_position_mode(true),
          hybrid_force_axis(2),
          hold_tangent_anchor_on_contact(true),
          tangent_anchor_use_measured_pose(true),
          release_tangent_anchor_on_contact_off(false),
          force_error_axis_sign{{1.0, 1.0, 1.0}},
          force_deadband_N{{0.0, 0.0, 0.0}},
          use_force_lpf(true),
          force_lpf_tau_s(0.04),
          use_force_target_ramp(true),
          force_ramp_only_when_contact(true),
          force_target_ramp_rate_Nps{{20.0, 20.0, 20.0}},
          force_target_release_rate_Nps{{40.0, 40.0, 40.0}},
          enable_sensor_raw_force_transform(false),
          R_tip_sensor_rowmajor{{1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0}},
          R_base_corr_rowmajor{{1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0}},
          fallback_to_f_meas_base_if_sensor_transform_fails(true),
          max_offset_m{{0.005, 0.005, 0.010}},
          max_step_m{{0.0003, 0.0003, 0.0003}},
          max_adm_velocity_mps{{0.01, 0.01, 0.01}},
          dt_min_s(1e-4),
          dt_max_s(5e-2),
          force_error_des_minus_meas(true),
          use_contact_gate(true),
          contact_force_threshold_N(0.5),
          use_contact_hysteresis(true),
          contact_on_threshold_N(0.7),
          contact_off_threshold_N(0.3),
          contact_gate_use_enabled_axes_only(true),
          decay_when_no_contact(true),
          no_contact_decay_ratio(0.90),
          antiwindup_on_offset_clamp(true),
          zero_velocity_on_offset_clamp(true),
          offset_clamp_velocity_damping(0.2),
          sync_adm_state_to_final_cmd(true),
          use_slip_detection(true),
          tangent_slip_threshold_m(0.0015),
          use_slip_guard(true),
          slip_guard_force_scale(0.25),
          slip_guard_reanchor_tangent(true),
          slip_guard_velocity_damping(0.2),
          ik_max_iters(80),
          ik_tol_pos_m(5e-4),
          ik_lambda(1e-2),
          ik_lambda_min(1e-5),
          ik_lambda_max(1.0),
          ik_alpha(0.8),
          ik_alpha_min(0.05),
          ik_max_step(0.15),
          ik_mu_posture(1e-4),
          ik_verbose(false),
          prefer_last_success_q_seed(true),
          keep_last_success_on_ik_fail(true),
          damp_velocity_on_ik_fail(true),
          ik_fail_velocity_damping(0.2),
          debug_enable_rclcpp(false),
          debug_decimation(50),
          debug_print_all_steps(false),
          verbose(false)
        {}
    };

    struct StepInput {
        Eigen::Vector3d p_des_base{Eigen::Vector3d::Zero()};   // desired fingertip pos in hand base frame
        Eigen::Vector3d f_des_base{Eigen::Vector3d::Zero()};   // desired force in hand base frame

        // Base-frame measured force (current v18 path: HandContactForceCallback output)
        Eigen::Vector3d f_meas_base{Eigen::Vector3d::Zero()};

        // Optional raw sensor-frame measured force (future)
        bool use_f_meas_sensor_raw{false};
        Eigen::Vector3d f_meas_sensor_raw{Eigen::Vector3d::Zero()};

        std::vector<double> q_hand_current20;                  // current hand joints (20DoF preferred)
        double dt_s{0.001};
    };

    struct StepOutput {
        bool controller_ok{false};
        bool ik_ok{false};

        bool contact_on{false};
        bool contact_rising_edge{false};
        bool contact_falling_edge{false};
        double used_dt_s{0.0};

        Eigen::Vector3d p_meas_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_des_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_ref_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_cmd_base{Eigen::Vector3d::Zero()};

        bool used_sensor_raw_transform{false};
        Eigen::Vector3d f_meas_sensor_raw{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base_from_sensor{Eigen::Vector3d::Zero()};

        Eigen::Vector3d f_meas_base_raw{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base_filt{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_des_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_des_ramped{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_des_used{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_err{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_drive{Eigen::Vector3d::Zero()};

        Eigen::Vector3d adm_offset{Eigen::Vector3d::Zero()};
        Eigen::Vector3d adm_velocity{Eigen::Vector3d::Zero()};

        std::array<bool,3> axis_force_enable_eff{{false,false,false}};
        int hybrid_force_axis{-1};

        std::array<bool,3> offset_clamped{{false,false,false}};
        std::array<bool,3> step_limited{{false,false,false}};

        bool tangent_anchor_valid{false};
        Eigen::Vector3d tangent_anchor_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d tangent_drift_vec{Eigen::Vector3d::Zero()};
        double tangent_drift_norm{0.0};
        bool slip_detected{false};

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
        initialize(hand_fk, hand_ik, finger_id, Config());
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
        return initialize(hand_fk, hand_ik, finger_id, Config());
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
        p_cmd_prev_.setZero();

        f_meas_filt_.setZero();
        f_lpf_initialized_ = false;

        contact_state_ = false;
        prev_contact_state_ = false;

        tangent_anchor_valid_ = false;
        tangent_anchor_p_.setZero();

        f_des_ramped_.setZero();
        f_des_ramp_initialized_ = false;

        last_q_cmd_123_ = {{0.0, 0.0, 0.0}};
        last_q_cmd20_.clear();

        debug_counter_ = 0;

        const bool ok_fk = (hand_fk_ != nullptr && hand_fk_->ok());
        const bool ok_ik = (hand_ik_ != nullptr && hand_ik_->ok());
        ok_ = ok_fk && ok_ik;

        if (cfg_.verbose) {
            std::printf("[HandAdm] initialize: ok=%d finger=%s(%d)\n",
                        ok_ ? 1 : 0, fingerName(finger_id_).c_str(), finger_id_);
        }
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
        p_cmd_prev_.setZero();

        f_meas_filt_.setZero();
        f_lpf_initialized_ = false;

        contact_state_ = false;
        prev_contact_state_ = false;

        tangent_anchor_valid_ = false;
        tangent_anchor_p_.setZero();

        f_des_ramped_.setZero();
        f_des_ramp_initialized_ = false;
    }

    void resetState(const Eigen::Vector3d& p_ref_base)
    {
        initialized_ = true;
        adm_x_.setZero();
        adm_v_.setZero();
        p_cmd_prev_ = p_ref_base;

        f_meas_filt_.setZero();
        f_lpf_initialized_ = false;

        contact_state_ = false;
        prev_contact_state_ = false;

        tangent_anchor_valid_ = false;
        tangent_anchor_p_.setZero();

        f_des_ramped_.setZero();
        f_des_ramp_initialized_ = false;
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

        if (!ok_ || !hand_fk_ || !hand_ik_) return out;
        if (in.q_hand_current20.size() < 15) return out;

        out.controller_ok = true;

        // STEP 0) dt stabilize
        const double dt = clampScalar_(in.dt_s, cfg_.dt_min_s, cfg_.dt_max_s);
        out.used_dt_s = dt;

        // STEP 1) FK
        const std::vector<Eigen::Vector3d> tips_cur = hand_fk_->computeFingertips(in.q_hand_current20);
        const Eigen::Vector3d p_meas = safeTip_(tips_cur, finger_id_);
        out.p_meas_base = p_meas;
        out.p_des_base  = in.p_des_base;

        // STEP 2) measured force input select
        Eigen::Vector3d f_meas_input_base = in.f_meas_base;
        out.f_meas_sensor_raw = in.f_meas_sensor_raw;
        out.f_meas_base_from_sensor.setZero();
        out.used_sensor_raw_transform = false;

        if (cfg_.enable_sensor_raw_force_transform && in.use_f_meas_sensor_raw) {
            bool tf_ok = false;

            const std::vector<Eigen::Matrix3d> R_base_tip_all = hand_fk_->computeTipRotationsBase(in.q_hand_current20);
            const Eigen::Matrix3d R_base_tip = safeRot_(R_base_tip_all, finger_id_);

            const Eigen::Matrix3d R_tip_sensor = mat3FromRowMajor_(cfg_.R_tip_sensor_rowmajor);
            const Eigen::Matrix3d R_base_corr  = mat3FromRowMajor_(cfg_.R_base_corr_rowmajor);

            const Eigen::Vector3d f_tip      = R_tip_sensor * in.f_meas_sensor_raw;
            const Eigen::Vector3d f_base_raw = R_base_tip * f_tip;
            const Eigen::Vector3d f_base     = R_base_corr * f_base_raw;

            if (isFiniteVec3_(f_base)) {
                f_meas_input_base = f_base;
                out.f_meas_base_from_sensor = f_base;
                out.used_sensor_raw_transform = true;
                tf_ok = true;
            }

            if (!tf_ok && !cfg_.fallback_to_f_meas_base_if_sensor_transform_fails) {
                out.controller_ok = false;
                return out;
            }
        }

        out.f_meas_base_raw = f_meas_input_base;
        out.f_des_base      = in.f_des_base;

        // LPF
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

        out.f_meas_base_filt = f_meas_filt_;
        out.f_meas_base      = f_meas_used;

        // STEP 2.5) Hybrid axis
        std::array<bool,3> axis_force_enable_eff = cfg_.force_ctrl_enable;
        int hybrid_force_axis = -1;

        if (cfg_.use_hybrid_force_position_mode) {
            hybrid_force_axis = clampAxis_(cfg_.hybrid_force_axis);
            axis_force_enable_eff = {{false, false, false}};
            axis_force_enable_eff[static_cast<std::size_t>(hybrid_force_axis)] = true;
        }
        out.axis_force_enable_eff = axis_force_enable_eff;
        out.hybrid_force_axis     = hybrid_force_axis;

        // STEP 3) Contact detection
        auto contact_metric_norm = [&](const Eigen::Vector3d& f)->double {
            if (!cfg_.contact_gate_use_enabled_axes_only) return f.norm();
            double s2 = 0.0; bool any=false;
            for (int ax=0; ax<3; ++ax) {
                if (axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    s2 += f(ax)*f(ax);
                    any=true;
                }
            }
            return any ? std::sqrt(s2) : f.norm();
        };

        const double f_metric = contact_metric_norm(f_meas_used);
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

        // STEP 3.5) Tangent anchor
        Eigen::Vector3d p_ref = in.p_des_base;

        if (cfg_.hold_tangent_anchor_on_contact) {
            if (out.contact_rising_edge) {
                tangent_anchor_valid_ = true;
                tangent_anchor_p_ = cfg_.tangent_anchor_use_measured_pose ? p_meas : in.p_des_base;
            }
            if (out.contact_falling_edge && cfg_.release_tangent_anchor_on_contact_off) {
                tangent_anchor_valid_ = false;
            }
            if (out.contact_on && tangent_anchor_valid_) {
                for (int ax=0; ax<3; ++ax) {
                    if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                        p_ref(ax) = tangent_anchor_p_(ax);
                    }
                }
            }
        }

        out.p_ref_base = p_ref;
        out.tangent_anchor_valid = tangent_anchor_valid_;
        out.tangent_anchor_base  = tangent_anchor_p_;

        // STEP 3.6) Force target ramp
        Eigen::Vector3d f_des_ramped = in.f_des_base;
        if (!f_des_ramp_initialized_) {
            f_des_ramped_ = Eigen::Vector3d::Zero();
            f_des_ramp_initialized_ = true;
        }

        if (cfg_.use_force_target_ramp) {
            for (int ax=0; ax<3; ++ax) {
                const bool axis_force_on = axis_force_enable_eff[static_cast<std::size_t>(ax)];
                if (!axis_force_on) { f_des_ramped_(ax)=0.0; continue; }

                const double target_contact = in.f_des_base(ax);
                const double target_release = 0.0;
                const bool allow_ramp_to_target =
                    (!cfg_.force_ramp_only_when_contact) || out.contact_on;
                const double target = allow_ramp_to_target ? target_contact : target_release;

                const double rate_up   = std::max(1e-9, cfg_.force_target_ramp_rate_Nps[ax]);
                const double rate_down = std::max(1e-9, cfg_.force_target_release_rate_Nps[ax]);

                const double diff = target - f_des_ramped_(ax);
                const double rate = (std::fabs(target) >= std::fabs(f_des_ramped_(ax))) ? rate_up : rate_down;
                const double step = rate * dt;

                if (std::fabs(diff) <= step) f_des_ramped_(ax) = target;
                else f_des_ramped_(ax) += ((diff > 0.0) ? step : -step);
            }
            f_des_ramped = f_des_ramped_;
        } else {
            for (int ax=0; ax<3; ++ax) {
                if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) f_des_ramped(ax)=0.0;
            }
            f_des_ramped_ = f_des_ramped;
            f_des_ramp_initialized_ = true;
        }
        out.f_des_ramped = f_des_ramped;

        // STEP 3.7) Slip detection / guard
        Eigen::Vector3d tangent_drift = Eigen::Vector3d::Zero();
        double tangent_drift_norm = 0.0;
        bool slip_detected = false;

        if (cfg_.use_slip_detection && out.contact_on && tangent_anchor_valid_) {
            for (int ax=0; ax<3; ++ax) {
                if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    tangent_drift(ax) = p_meas(ax) - tangent_anchor_p_(ax);
                }
            }
            tangent_drift_norm = tangent_drift.norm();
            slip_detected = (tangent_drift_norm > std::max(0.0, cfg_.tangent_slip_threshold_m));
        }

        Eigen::Vector3d f_des_used = f_des_ramped;
        if (cfg_.use_slip_guard && slip_detected) {
            const double s = clampScalar_(cfg_.slip_guard_force_scale, 0.0, 1.0);
            for (int ax=0; ax<3; ++ax) {
                if (axis_force_enable_eff[static_cast<std::size_t>(ax)]) f_des_used(ax) *= s;
                else f_des_used(ax) = 0.0;
            }
            const double rv = clampScalar_(cfg_.slip_guard_velocity_damping, 0.0, 1.0);
            adm_v_ *= rv;

            if (cfg_.slip_guard_reanchor_tangent) {
                tangent_anchor_valid_ = true;
                tangent_anchor_p_ = p_meas;
                out.tangent_anchor_valid = true;
                out.tangent_anchor_base = tangent_anchor_p_;
            }
        }

        out.tangent_drift_vec  = tangent_drift;
        out.tangent_drift_norm = tangent_drift_norm;
        out.slip_detected      = slip_detected;
        out.f_des_used         = f_des_used;

        // STEP 4) Force error / drive
        Eigen::Vector3d f_err;
        if (cfg_.force_error_des_minus_meas) f_err = (f_des_used - f_meas_used);
        else                                 f_err = (f_meas_used - f_des_used);
        out.f_err = f_err;

        Eigen::Vector3d f_drive = f_err;
        for (int ax=0; ax<3; ++ax) {
            if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                f_drive(ax) = 0.0;
                continue;
            }

            const double db = std::max(0.0, cfg_.force_deadband_N[ax]);
            if (std::fabs(f_drive(ax)) <= db) f_drive(ax)=0.0;
            else f_drive(ax) = (f_drive(ax) > 0.0) ? (f_drive(ax)-db) : (f_drive(ax)+db);

            const double sgn = (cfg_.force_error_axis_sign[ax] >= 0.0) ? 1.0 : -1.0;
            f_drive(ax) *= sgn;
        }
        out.f_drive = f_drive;

        // STEP 5) init
        if (!initialized_) {
            initialized_ = true;
            adm_x_.setZero();
            adm_v_.setZero();
            p_cmd_prev_ = p_ref;
        }

        // STEP 6) 1D admittance per axis
        Eigen::Vector3d p_cmd = p_ref;

        for (int ax=0; ax<3; ++ax) {
            const bool axis_force_on = axis_force_enable_eff[static_cast<std::size_t>(ax)];

            if (!axis_force_on) {
                adm_x_(ax)=0.0; adm_v_(ax)=0.0; p_cmd(ax)=p_ref(ax);
                continue;
            }

            const bool apply_adm = (!cfg_.use_contact_gate) || out.contact_on;

            if (apply_adm) {
                const double M = std::max(1e-6, cfg_.mass[ax]);
                const double D = std::max(0.0,  cfg_.damping[ax]);
                const double K = std::max(0.0,  cfg_.stiffness[ax]);

                const double x   = adm_x_(ax);
                const double xd  = adm_v_(ax);
                const double xdd = (f_drive(ax) - D*xd - K*x) / M;

                double xd_new = xd + xdd * dt;

                const double vmax = std::fabs(cfg_.max_adm_velocity_mps[ax]);
                if (vmax > 0.0) xd_new = clampScalar_(xd_new, -vmax, vmax);

                double x_new = x + xd_new * dt;

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

                adm_v_(ax) = xd_new;
                adm_x_(ax) = x_new;
                out.offset_clamped[static_cast<std::size_t>(ax)] = offset_clamped;
            } else {
                if (cfg_.decay_when_no_contact) {
                    const double r = clampScalar_(cfg_.no_contact_decay_ratio, 0.0, 1.0);
                    adm_x_(ax) *= r;
                    adm_v_(ax) *= r;
                } else {
                    adm_x_(ax) = 0.0;
                    adm_v_(ax) = 0.0;
                }
            }

            p_cmd(ax) = p_ref(ax) + adm_x_(ax);
        }

        // STEP 6.5) step clamp
        for (int ax=0; ax<3; ++ax) {
            const double dmax = std::fabs(cfg_.max_step_m[ax]);
            const double d = p_cmd(ax) - p_cmd_prev_(ax);
            double d_applied = d;
            if (dmax > 0.0) {
                d_applied = clampScalar_(d, -dmax, dmax);
                out.step_limited[static_cast<std::size_t>(ax)] = (std::fabs(d_applied - d) > 1e-15);
            }
            p_cmd(ax) = p_cmd_prev_(ax) + d_applied;
        }

        if (cfg_.sync_adm_state_to_final_cmd) {
            for (int ax=0; ax<3; ++ax) {
                if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    adm_x_(ax)=0.0; adm_v_(ax)=0.0;
                    continue;
                }
                adm_x_(ax) = p_cmd(ax) - p_ref(ax);
                if (out.step_limited[static_cast<std::size_t>(ax)]) adm_v_(ax) *= 0.5;
            }
        }

        out.p_cmd_base = p_cmd;
        out.adm_offset = adm_x_;
        out.adm_velocity = adm_v_;

        // STEP 7) IK
        std::array<Eigen::Vector3d,5> tips_target;
        for (int i=0; i<5; ++i) tips_target[i] = safeTip_(tips_cur, i);
        tips_target[finger_id_] = p_cmd;

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
        ik_opt.verbose     = cfg_.ik_verbose;

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

            p_cmd_prev_ = p_cmd;
            debugStep_(out);
            return out;
        }

        // STEP 7.5) IK fail fallback
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

        debugStep_(out);
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

    static Eigen::Matrix3d safeRot_(const std::vector<Eigen::Matrix3d>& Rs, int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(Rs.size())) return Eigen::Matrix3d::Identity();
        return Rs[static_cast<std::size_t>(idx)];
    }

    static bool isFiniteVec3_(const Eigen::Vector3d& v)
    {
        return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
    }

    static Eigen::Matrix3d mat3FromRowMajor_(const std::array<double,9>& a)
    {
        Eigen::Matrix3d M;
        M << a[0], a[1], a[2],
             a[3], a[4], a[5],
             a[6], a[7], a[8];
        return M;
    }

    void debugStep_(const StepOutput& out)
    {
        if (!cfg_.debug_enable_rclcpp) return;

        const int decim = std::max(1, cfg_.debug_decimation);
        const bool print_now = ((debug_counter_++ % decim) == 0);
        if (!print_now) return;

        rclcpp::Logger logger = has_debug_logger_ ? debug_logger_
                                                  : rclcpp::get_logger("HandAdmittanceControl");

        RCLCPP_INFO(
            logger,
            "[HandAdm][%s] dt=%.4f contact=%d rise=%d fall=%d slip=%d | "
            "hax=%d | p_ref=(%.4f %.4f %.4f) p_cmd=(%.4f %.4f %.4f) | "
            "f_meas=(%.3f %.3f %.3f) f_des=(%.3f %.3f %.3f) f_used=(%.3f %.3f %.3f) | "
            "f_err=(%.3f %.3f %.3f) | ik_ok=%d",
            fingerName(finger_id_).c_str(),
            out.used_dt_s,
            static_cast<int>(out.contact_on),
            static_cast<int>(out.contact_rising_edge),
            static_cast<int>(out.contact_falling_edge),
            static_cast<int>(out.slip_detected),
            out.hybrid_force_axis,
            out.p_ref_base.x(), out.p_ref_base.y(), out.p_ref_base.z(),
            out.p_cmd_base.x(), out.p_cmd_base.y(), out.p_cmd_base.z(),
            out.f_meas_base.x(), out.f_meas_base.y(), out.f_meas_base.z(),
            out.f_des_base.x(), out.f_des_base.y(), out.f_des_base.z(),
            out.f_des_used.x(), out.f_des_used.y(), out.f_des_used.z(),
            out.f_err.x(), out.f_err.y(), out.f_err.z(),
            static_cast<int>(out.ik_ok));

        if (!cfg_.debug_print_all_steps) return;

        RCLCPP_INFO(
            logger,
            "  [adm] x=(%.5f %.5f %.5f) v=(%.5f %.5f %.5f) | step_lim=[%d%d%d] clamp=[%d%d%d]",
            out.adm_offset.x(), out.adm_offset.y(), out.adm_offset.z(),
            out.adm_velocity.x(), out.adm_velocity.y(), out.adm_velocity.z(),
            static_cast<int>(out.step_limited[0]), static_cast<int>(out.step_limited[1]), static_cast<int>(out.step_limited[2]),
            static_cast<int>(out.offset_clamped[0]), static_cast<int>(out.offset_clamped[1]), static_cast<int>(out.offset_clamped[2]));
    }

private:
    bool ok_{false};
    bool initialized_{false};

    std::shared_ptr<HandForwardKinematics> hand_fk_;
    std::shared_ptr<HandInverseKinematics> hand_ik_;

    int finger_id_{RING};
    Config cfg_;

    Eigen::Vector3d adm_x_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d adm_v_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d p_cmd_prev_{Eigen::Vector3d::Zero()};

    Eigen::Vector3d f_meas_filt_{Eigen::Vector3d::Zero()};
    bool f_lpf_initialized_{false};

    bool contact_state_{false};
    bool prev_contact_state_{false};

    bool tangent_anchor_valid_{false};
    Eigen::Vector3d tangent_anchor_p_{Eigen::Vector3d::Zero()};

    Eigen::Vector3d f_des_ramped_{Eigen::Vector3d::Zero()};
    bool f_des_ramp_initialized_{false};

    bool last_success_valid_{false};
    std::array<double,3> last_q_cmd_123_ {{0.0, 0.0, 0.0}};
    std::vector<double> last_q_cmd20_;

    rclcpp::Logger debug_logger_{rclcpp::get_logger("HandAdmittanceControl")};
    bool has_debug_logger_{false};
    int debug_counter_{0};
};

} // namespace dualarm_forcecon

#endif // DUALARM_FORCECON_KINEMATICS_HAND_ADMITTANCE_CONTROL_HPP_