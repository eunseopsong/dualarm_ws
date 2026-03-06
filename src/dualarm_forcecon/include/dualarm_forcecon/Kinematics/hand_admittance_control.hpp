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

    // =========================================================================
    // Minimal Config (basic admittance + IK only)
    // =========================================================================
    struct Config {
        // Admittance MDK per axis in HAND_BASE frame
        std::array<double,3> mass;
        std::array<double,3> damping;
        std::array<double,3> stiffness;

        // Force-control axis selection
        // - if use_hybrid_force_position_mode == true:
        //     only hybrid_force_axis is force-controlled, others follow Xd directly
        // - else:
        //     force_ctrl_enable[] is used directly
        bool use_hybrid_force_position_mode;
        int  hybrid_force_axis;                 // 0=x, 1=y, 2=z
        std::array<bool,3> force_ctrl_enable;   // used only when hybrid=false

        // Force error definition & sign tuning
        // f_err = (Fd - Fext) if true, else (Fext - Fd)
        bool force_error_des_minus_meas;
        std::array<double,3> force_error_axis_sign;

        // Optional measured-force LPF
        bool use_force_lpf;
        double force_lpf_tau_s;

        // dt clamp
        double dt_min_s;
        double dt_max_s;

        // Contact flag threshold (diagnostics only; no gate)
        double contact_detect_threshold_N;

        // IK options
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

        // Stability on IK fail (minimal)
        bool   damp_velocity_on_ik_fail;
        double ik_fail_velocity_damping;

        // Debug
        bool debug_enable_rclcpp;
        int  debug_decimation;
        bool verbose;

        Config()
        : mass{{3.0, 3.0, 3.0}},
          damping{{1000.0, 1000.0, 1000.0}},
          stiffness{{0.0, 0.0, 0.0}},

          // default: z-axis force control (hand-base Z), others position hold
          use_hybrid_force_position_mode(true),
          hybrid_force_axis(2),
          force_ctrl_enable{{false, false, true}},

          force_error_des_minus_meas(true),
          force_error_axis_sign{{1.0, 1.0, 1.0}},

          use_force_lpf(true),
          force_lpf_tau_s(0.03),

          dt_min_s(1e-4),
          dt_max_s(5e-2),

          contact_detect_threshold_N(0.5),

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
          debug_decimation(20),
          verbose(false)
        {}
    };

    // =========================================================================
    // Input / Output
    // =========================================================================
    struct StepInput {
        // Xd, Fd, Fext in HAND_BASE frame
        Eigen::Vector3d p_des_base{Eigen::Vector3d::Zero()};   // Xd
        Eigen::Vector3d f_des_base{Eigen::Vector3d::Zero()};   // Fd
        Eigen::Vector3d f_meas_base{Eigen::Vector3d::Zero()};  // Fext (measured force)

        // current finger/hand state for FK/IK
        std::vector<double> q_hand_current20;                  // canonical 20DoF preferred
        double dt_s{0.001};
    };

    struct StepOutput {
        bool controller_ok{false};
        bool ik_ok{false};

        // diagnostics
        bool contact_on{false};
        double used_dt_s{0.0};

        Eigen::Vector3d p_meas_base{Eigen::Vector3d::Zero()};  // current fingertip pos
        Eigen::Vector3d p_des_base{Eigen::Vector3d::Zero()};   // Xd
        Eigen::Vector3d p_cmd_base{Eigen::Vector3d::Zero()};   // Xcmd

        Eigen::Vector3d f_meas_base_raw{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base_filt{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base{Eigen::Vector3d::Zero()};  // control-used Fext
        Eigen::Vector3d f_des_base{Eigen::Vector3d::Zero()};   // Fd
        Eigen::Vector3d f_err{Eigen::Vector3d::Zero()};        // Fd - Fext (or reversed)

        // admittance internal state (x_cmd - x_des), and velocity
        Eigen::Vector3d adm_offset{Eigen::Vector3d::Zero()};
        Eigen::Vector3d adm_velocity{Eigen::Vector3d::Zero()};

        std::array<bool,3> axis_force_enable_eff{{false,false,false}};
        int hybrid_force_axis{-1};

        // selected finger outputs
        std::array<double,3> q_cmd_123 {{0.0, 0.0, 0.0}};
        double q_cmd_4_mimic{0.0}; // = q3
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

        // Admittance state
        x_cmd_state_.setZero();
        xdot_state_.setZero();

        // LPF state
        f_meas_filt_.setZero();
        f_lpf_initialized_ = false;

        // Contact flag (diagnostics only)
        contact_state_ = false;

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
        x_cmd_state_.setZero();
        xdot_state_.setZero();

        f_meas_filt_.setZero();
        f_lpf_initialized_ = false;

        contact_state_ = false;
    }

    void resetState(const Eigen::Vector3d& p_ref_base)
    {
        initialized_ = true;
        x_cmd_state_ = p_ref_base;      // directly set Xcmd state
        xdot_state_.setZero();

        f_meas_filt_.setZero();
        f_lpf_initialized_ = false;

        contact_state_ = false;
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

    // =========================================================================
    // Core Step
    //   Xd, Fd, Fext -> basic admittance -> Xcmd -> IK -> q_cmd
    // =========================================================================
    StepOutput step(const StepInput& in)
    {
        StepOutput out;
        out.controller_ok = false;

        if (!ok_ || !hand_fk_ || !hand_ik_) return out;
        if (in.q_hand_current20.size() < 15) return out;

        out.controller_ok = true;

        // ---------------------------------------------------------------------
        // STEP 0) dt guard
        // ---------------------------------------------------------------------
        const double dt = clampScalar_(in.dt_s, cfg_.dt_min_s, cfg_.dt_max_s);
        out.used_dt_s = dt;

        // ---------------------------------------------------------------------
        // STEP 1) Current fingertip pose (FK)
        // ---------------------------------------------------------------------
        const std::vector<Eigen::Vector3d> tips_cur = hand_fk_->computeFingertips(in.q_hand_current20);
        const Eigen::Vector3d p_meas = safeTip_(tips_cur, finger_id_);

        out.p_meas_base = p_meas;
        out.p_des_base  = in.p_des_base;
        out.f_des_base  = in.f_des_base;

        // ---------------------------------------------------------------------
        // STEP 2) Measured force preprocessing (Fext)
        // ---------------------------------------------------------------------
        out.f_meas_base_raw = in.f_meas_base;

        Eigen::Vector3d f_meas_used = in.f_meas_base;
        if (cfg_.use_force_lpf) {
            const double tau = std::max(1e-6, cfg_.force_lpf_tau_s);
            const double alpha = clampScalar_(dt / (tau + dt), 0.0, 1.0);

            if (!f_lpf_initialized_) {
                f_meas_filt_ = in.f_meas_base;
                f_lpf_initialized_ = true;
            } else {
                f_meas_filt_ = (1.0 - alpha) * f_meas_filt_ + alpha * in.f_meas_base;
            }
            f_meas_used = f_meas_filt_;
        } else {
            f_meas_filt_ = in.f_meas_base;
            f_lpf_initialized_ = true;
            f_meas_used = in.f_meas_base;
        }

        out.f_meas_base_filt = f_meas_filt_;
        out.f_meas_base      = f_meas_used;

        // ---------------------------------------------------------------------
        // STEP 2.5) Force-control axis configuration
        // ---------------------------------------------------------------------
        std::array<bool,3> axis_force_enable_eff = cfg_.force_ctrl_enable;
        int hybrid_force_axis = -1;

        if (cfg_.use_hybrid_force_position_mode) {
            hybrid_force_axis = clampAxis_(cfg_.hybrid_force_axis);
            axis_force_enable_eff = {{false, false, false}};
            axis_force_enable_eff[static_cast<std::size_t>(hybrid_force_axis)] = true;
        }

        out.axis_force_enable_eff = axis_force_enable_eff;
        out.hybrid_force_axis = hybrid_force_axis;

        // contact flag (diagnostics only; NO gating)
        {
            double s2 = 0.0;
            bool any = false;
            for (int ax = 0; ax < 3; ++ax) {
                if (axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    s2 += f_meas_used(ax) * f_meas_used(ax);
                    any = true;
                }
            }
            const double metric = any ? std::sqrt(s2) : f_meas_used.norm();
            contact_state_ = (metric >= std::max(0.0, cfg_.contact_detect_threshold_N));
            out.contact_on = contact_state_;
        }

        // ---------------------------------------------------------------------
        // STEP 3) Force error (Fd vs Fext)
        // ---------------------------------------------------------------------
        Eigen::Vector3d f_err;
        if (cfg_.force_error_des_minus_meas) f_err = (in.f_des_base - f_meas_used);
        else                                 f_err = (f_meas_used - in.f_des_base);

        for (int ax = 0; ax < 3; ++ax) {
            const double sgn = (cfg_.force_error_axis_sign[ax] >= 0.0) ? 1.0 : -1.0;
            f_err(ax) *= sgn;
        }
        out.f_err = f_err;

        // ---------------------------------------------------------------------
        // STEP 4) Basic Cartesian Admittance (Manipulator-style concept)
        //
        // For force-enabled axis:
        //   M * xdd + D * xd + K * (x - Xd) = f_err
        //   => xdd = (f_err - D*xd - K*(x - Xd)) / M
        //
        // For non-force axis:
        //   Xcmd = Xd directly (position follow)
        // ---------------------------------------------------------------------
        if (!initialized_) {
            initialized_ = true;

            // initialize Xcmd state close to measured pose to avoid sudden jump
            x_cmd_state_ = p_meas;
            xdot_state_.setZero();

            // for non-force axes, snap to Xd immediately
            for (int ax = 0; ax < 3; ++ax) {
                if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    x_cmd_state_(ax) = in.p_des_base(ax);
                }
            }
        }

        Eigen::Vector3d p_cmd = x_cmd_state_;

        for (int ax = 0; ax < 3; ++ax) {
            const bool axis_force_on = axis_force_enable_eff[static_cast<std::size_t>(ax)];

            if (!axis_force_on) {
                // pure position follow on non-force axes
                x_cmd_state_(ax) = in.p_des_base(ax);
                xdot_state_(ax) = 0.0;
                p_cmd(ax) = x_cmd_state_(ax);
                continue;
            }

            const double M = std::max(1e-6, cfg_.mass[ax]);
            const double D = std::max(0.0,  cfg_.damping[ax]);
            const double K = std::max(0.0,  cfg_.stiffness[ax]);

            const double x   = x_cmd_state_(ax);
            const double xd  = xdot_state_(ax);
            const double Xd  = in.p_des_base(ax);

            const double xdd = (f_err(ax) - D * xd - K * (x - Xd)) / M;

            const double xd_new = xd + xdd * dt;
            const double x_new  = x + xd_new * dt;

            xdot_state_(ax)  = xd_new;
            x_cmd_state_(ax) = x_new;
            p_cmd(ax) = x_new;
        }

        out.p_cmd_base = p_cmd;
        out.adm_offset = (p_cmd - in.p_des_base);
        out.adm_velocity = xdot_state_;

        // ---------------------------------------------------------------------
        // STEP 5) IK target construction (selected finger only)
        // ---------------------------------------------------------------------
        std::array<Eigen::Vector3d,5> tips_target;
        for (int i = 0; i < 5; ++i) tips_target[i] = safeTip_(tips_cur, i);
        tips_target[static_cast<std::size_t>(finger_id_)] = p_cmd;

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

        ik_opt.mask    = {{false, false, false, false, false}};
        ik_opt.weights = {{0.0, 0.0, 0.0, 0.0, 0.0}};
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
            out.q_cmd_4_mimic = q_sol20[qb + 2]; // q4 = mimic(q3)

            last_success_valid_ = true;
            last_q_cmd_123_ = out.q_cmd_123;
            last_q_cmd20_   = q_sol20;

            debugStep_(out);
            return out;
        }

        // ---------------------------------------------------------------------
        // STEP 5.5) IK fail fallback (minimal)
        // ---------------------------------------------------------------------
        if (cfg_.damp_velocity_on_ik_fail) {
            const double r = clampScalar_(cfg_.ik_fail_velocity_damping, 0.0, 1.0);
            xdot_state_ *= r;
        }

        out.q_cmd20 = in.q_hand_current20;
        if (out.q_cmd20.size() < 20) out.q_cmd20.resize(20, 0.0);

        const int qb = finger_id_ * 4;

        if (cfg_.keep_last_success_on_ik_fail && last_success_valid_ && last_q_cmd20_.size() >= 20) {
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
            } else {
                out.q_cmd_123 = {{0.0, 0.0, 0.0}};
                out.q_cmd_4_mimic = 0.0;
            }
        }

        if (cfg_.verbose) {
            std::printf("[HandAdm] IK fail finger=%s(%d), fallback applied.\n",
                        fingerName(finger_id_).c_str(), finger_id_);
        }

        debugStep_(out);
        return out;
    }

    // convenience overload
    bool step(const Eigen::Vector3d& p_des_base,
              const Eigen::Vector3d& f_des_base,
              const Eigen::Vector3d& f_meas_base,
              const std::vector<double>& q_hand_current20,
              double dt_s,
              std::array<double,3>& q_cmd_123_out)
    {
        StepInput in;
        in.p_des_base = p_des_base;
        in.f_des_base = f_des_base;
        in.f_meas_base = f_meas_base;
        in.q_hand_current20 = q_hand_current20;
        in.dt_s = dt_s;

        StepOutput out = step(in);
        q_cmd_123_out = out.q_cmd_123;
        return (out.controller_ok && out.ik_ok);
    }

private:
    // =========================================================================
    // Helpers
    // =========================================================================
    static int clampFingerId_(int fid)
    {
        if (fid < 0) return 0;
        if (fid > 4) return 4;
        return fid;
    }

    static int clampAxis_(int ax)
    {
        if (ax < 0) return 0;
        if (ax > 2) return 2;
        return ax;
    }

    static double clampScalar_(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    static Eigen::Vector3d safeTip_(const std::vector<Eigen::Vector3d>& tips, int idx)
    {
        if (idx < 0 || idx >= static_cast<int>(tips.size())) return Eigen::Vector3d::Zero();
        return tips[static_cast<std::size_t>(idx)];
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
            "[HandAdm][%s] dt=%.4f ik_ok=%d contact=%d | axis_force=[%d%d%d] hax=%d | "
            "Xd=(%.4f %.4f %.4f) Xcmd=(%.4f %.4f %.4f) | "
            "Fd=(%.3f %.3f %.3f) Fext=(%.3f %.3f %.3f) Ferr=(%.3f %.3f %.3f) | "
            "off=(%.5f %.5f %.5f) vel=(%.5f %.5f %.5f)",
            fingerName(finger_id_).c_str(),
            out.used_dt_s,
            static_cast<int>(out.ik_ok),
            static_cast<int>(out.contact_on),
            static_cast<int>(out.axis_force_enable_eff[0]),
            static_cast<int>(out.axis_force_enable_eff[1]),
            static_cast<int>(out.axis_force_enable_eff[2]),
            out.hybrid_force_axis,
            out.p_des_base.x(), out.p_des_base.y(), out.p_des_base.z(),
            out.p_cmd_base.x(), out.p_cmd_base.y(), out.p_cmd_base.z(),
            out.f_des_base.x(), out.f_des_base.y(), out.f_des_base.z(),
            out.f_meas_base.x(), out.f_meas_base.y(), out.f_meas_base.z(),
            out.f_err.x(), out.f_err.y(), out.f_err.z(),
            out.adm_offset.x(), out.adm_offset.y(), out.adm_offset.z(),
            out.adm_velocity.x(), out.adm_velocity.y(), out.adm_velocity.z());
    }

private:
    bool ok_{false};
    bool initialized_{false};

    std::shared_ptr<HandForwardKinematics> hand_fk_;
    std::shared_ptr<HandInverseKinematics> hand_ik_;

    int finger_id_{RING};
    Config cfg_;

    // Admittance state (absolute Xcmd + velocity)
    Eigen::Vector3d x_cmd_state_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d xdot_state_{Eigen::Vector3d::Zero()};

    // LPF state for measured force
    Eigen::Vector3d f_meas_filt_{Eigen::Vector3d::Zero()};
    bool f_lpf_initialized_{false};

    // contact flag (diagnostics only)
    bool contact_state_{false};

    // fallback / IK seed
    bool last_success_valid_{false};
    std::array<double,3> last_q_cmd_123_ {{0.0, 0.0, 0.0}};
    std::vector<double> last_q_cmd20_;

    // debug
    rclcpp::Logger debug_logger_{rclcpp::get_logger("HandAdmittanceControl")};
    bool has_debug_logger_{false};
    int debug_counter_{0};
};

} // namespace dualarm_forcecon

#endif // DUALARM_FORCECON_KINEMATICS_HAND_ADMITTANCE_CONTROL_HPP_