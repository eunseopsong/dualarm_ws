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

#include "dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_inverse_kinematics.hpp"

namespace dualarm_forcecon {

// ============================================================================
// HandAdmittanceControl (Strategy A: 손가락별 독립 컨트롤러)
// - 한 객체가 "한 손가락"만 담당
// - Method 1: 기존 HandInverseKinematics::solveIKFingertips(...) 재사용
//   -> 선택된 손가락만 target 변경, 나머지 손가락은 현재 fingertip 유지
//
// Finger order (HandIK/FK와 동일):
//   0: thumb, 1: index, 2: middle, 3: ring, 4: baby
//
// q20 order (호환):
//   [thumb1..4, index1..4, middle1..4, ring1..4, baby1..4]
//
// 출력:
//   q1_cmd, q2_cmd, q3_cmd (q4는 mimic => q3)
//
// 참고:
// - 이 클래스는 "제어 로직"에 집중.
// - measured force frame 변환(센서 frame -> hand base frame)은 callback 레벨에서 끝낸 뒤
//   여기에는 hand base frame 기준 force를 넣는 것을 권장.
// ============================================================================

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
        // Admittance params (axis: x,y,z in HAND BASE frame)
        std::array<double,3> mass;
        std::array<double,3> damping;
        std::array<double,3> stiffness;

        // 축별 force-admittance 활성화 여부
        // false인 축은 p_cmd(axis)=p_des(axis) (순수 위치 유지)
        std::array<bool,3> force_ctrl_enable;

        // [PATCH] force error 축별 부호 보정 (빠른 디버그용)
        // 실제 접촉/축 정의에 따라 +1/-1로 맞출 수 있음
        std::array<double,3> force_error_axis_sign;

        // [PATCH] 축별 force error deadband [N]
        std::array<double,3> force_deadband_N;

        // [PATCH] measured force LPF
        bool use_force_lpf;
        double force_lpf_tau_s;  // e.g., 0.03~0.08 s

        // Admittance offset 제한: p_cmd = p_des + x_adm
        std::array<double,3> max_offset_m;   // [m]

        // 한 step 당 p_cmd 변화량 제한 (rate limit)
        std::array<double,3> max_step_m;     // [m/tick]

        // [PATCH] 축별 admittance velocity 제한 (내부 상태 폭주 방지)
        std::array<double,3> max_adm_velocity_mps; // [m/s]

        // dt clamp (수치 안정화)
        double dt_min_s;
        double dt_max_s;

        // Force error 정의
        // true : f_err = f_des - f_meas  (권장 기본)
        // false: f_err = f_meas - f_des
        bool force_error_des_minus_meas;

        // Contact gate (옵션)
        // - false면 항상 admittance 적용
        // - true면 contact_on일 때만 admittance 적용 (아니면 offset decay 또는 hold)
        bool use_contact_gate;
        double contact_force_threshold_N;   // norm 기준 (legacy/simple gate)

        // [PATCH] contact hysteresis (옵션)
        bool use_contact_hysteresis;
        double contact_on_threshold_N;
        double contact_off_threshold_N;

        // [PATCH] contact metric 계산 시 force_ctrl_enable 축만 볼지 여부
        bool contact_gate_use_enabled_axes_only;

        // contact off 상태에서 offset/velocity를 감쇠시킬지
        bool decay_when_no_contact;
        double no_contact_decay_ratio;      // 0~1, tick당 감소율

        // [PATCH] offset clamp anti-windup
        bool antiwindup_on_offset_clamp;
        bool zero_velocity_on_offset_clamp;
        double offset_clamp_velocity_damping; // 0~1 (zero_velocity=false일 때 사용)

        // [PATCH] 최종 step-limit 적용 후 내부 adm state를 실제 command에 동기화
        // (중요: p_cmd는 제한됐는데 adm_x_가 그대로면 내부/외부 불일치 누적됨)
        bool sync_adm_state_to_final_cmd;

        // IK solver 옵션 (HandInverseKinematics::Options)
        int    ik_max_iters;
        double ik_tol_pos_m;
        double ik_lambda;
        double ik_lambda_min;
        double ik_lambda_max;
        double ik_alpha;
        double ik_alpha_min;
        double ik_max_step;    // rad
        double ik_mu_posture;
        bool   ik_verbose;

        // [PATCH] IK seed 정책
        bool prefer_last_success_q_seed;

        // IK 실패 시 처리
        bool keep_last_success_on_ik_fail;
        bool damp_velocity_on_ik_fail;
        double ik_fail_velocity_damping; // v <- v * factor

        // 디버그
        bool verbose;

        Config()
        : mass{{0.03, 0.03, 0.03}},
          damping{{8.0, 8.0, 8.0}},
          stiffness{{120.0, 120.0, 120.0}},
          force_ctrl_enable{{true, true, true}},
          force_error_axis_sign{{1.0, 1.0, 1.0}},
          force_deadband_N{{0.0, 0.0, 0.0}},
          use_force_lpf(true),
          force_lpf_tau_s(0.04),
          max_offset_m{{0.010, 0.010, 0.010}},
          max_step_m{{0.0010, 0.0010, 0.0010}},
          max_adm_velocity_mps{{0.05, 0.05, 0.05}},
          dt_min_s(1e-4),
          dt_max_s(5e-2),
          force_error_des_minus_meas(true),
          use_contact_gate(false),
          contact_force_threshold_N(0.5),
          use_contact_hysteresis(false),
          contact_on_threshold_N(0.7),
          contact_off_threshold_N(0.3),
          contact_gate_use_enabled_axes_only(true),
          decay_when_no_contact(true),
          no_contact_decay_ratio(0.90),
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
          ik_verbose(false),
          prefer_last_success_q_seed(true),
          keep_last_success_on_ik_fail(true),
          damp_velocity_on_ik_fail(true),
          ik_fail_velocity_damping(0.2),
          verbose(false)
        {}
    };

    struct StepInput {
        Eigen::Vector3d p_des_base{Eigen::Vector3d::Zero()};   // desired fingertip pos in hand base frame
        Eigen::Vector3d f_des_base{Eigen::Vector3d::Zero()};   // desired force in hand base frame
        Eigen::Vector3d f_meas_base{Eigen::Vector3d::Zero()};  // measured force in hand base frame
        std::vector<double> q_hand_current20;                  // current hand joints (20DoF preferred, 15DoF also accepted by HandIK)
        double dt_s{0.001};
    };

    struct StepOutput {
        bool controller_ok{false};   // input validity + FK/IK objects OK
        bool ik_ok{false};

        bool contact_on{false};
        double used_dt_s{0.0};

        // diagnostics
        Eigen::Vector3d p_meas_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_des_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_cmd_base{Eigen::Vector3d::Zero()};

        // measured force diagnostics
        Eigen::Vector3d f_meas_base_raw{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base_filt{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base{Eigen::Vector3d::Zero()};   // actually used in control
        Eigen::Vector3d f_des_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_err{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_drive{Eigen::Vector3d::Zero()};       // sign/deadband processed force input to admittance

        Eigen::Vector3d adm_offset{Eigen::Vector3d::Zero()};
        Eigen::Vector3d adm_velocity{Eigen::Vector3d::Zero()};

        std::array<bool,3> offset_clamped{{false,false,false}};
        std::array<bool,3> step_limited{{false,false,false}};

        // outputs for selected finger only
        std::array<double,3> q_cmd_123 {{0.0, 0.0, 0.0}};
        double q_cmd_4_mimic{0.0}; // = q3

        // optional full hand command (20DoF)
        std::vector<double> q_cmd20;
    };

public:
    HandAdmittanceControl() = default;

    // C++14-safe: default Config() 를 default argument로 두지 않고 오버로드로 분리
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

        last_q_cmd_123_ = {{0.0, 0.0, 0.0}};
        last_q_cmd20_.clear();

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

    void resetState()
    {
        initialized_ = false;
        adm_x_.setZero();
        adm_v_.setZero();
        p_cmd_prev_.setZero();

        f_meas_filt_.setZero();
        f_lpf_initialized_ = false;
        contact_state_ = false;
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
    // step()
    // - Strategy A 기준: finger_id는 객체 생성 시 고정 (입력에 따로 안 받음)
    // - Method 1: whole-hand IK를 호출하되, 선택된 손가락만 target 변경
    // =========================================================================
    StepOutput step(const StepInput& in)
    {
        StepOutput out;
        out.controller_ok = false;

        if (!ok_ || !hand_fk_ || !hand_ik_) return out;
        if (in.q_hand_current20.size() < 15) return out;

        out.controller_ok = true;

        // -----------------------------
        // 0) dt 안정화
        // -----------------------------
        const double dt = clampScalar_(in.dt_s, cfg_.dt_min_s, cfg_.dt_max_s);
        out.used_dt_s = dt;

        // -----------------------------
        // 1) 현재 fingertip 측정 (FK)
        // -----------------------------
        const std::vector<Eigen::Vector3d> tips_cur = hand_fk_->computeFingertips(in.q_hand_current20);
        const Eigen::Vector3d p_meas = safeTip_(tips_cur, finger_id_);

        out.p_meas_base = p_meas;
        out.p_des_base  = in.p_des_base;

        // -----------------------------
        // 1.5) measured force filtering (PATCH)
        // -----------------------------
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
        out.f_des_base       = in.f_des_base;

        // -----------------------------
        // 2) contact gate 판정 (옵션)
        // -----------------------------
        auto contact_metric_norm = [&](const Eigen::Vector3d& f)->double {
            if (!cfg_.contact_gate_use_enabled_axes_only) {
                return f.norm();
            }
            double s2 = 0.0;
            bool any = false;
            for (int ax = 0; ax < 3; ++ax) {
                if (cfg_.force_ctrl_enable[ax]) {
                    s2 += f(ax) * f(ax);
                    any = true;
                }
            }
            return any ? std::sqrt(s2) : f.norm();
        };

        const double f_metric = contact_metric_norm(f_meas_used);

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
            // legacy/simple behavior
            out.contact_on = (f_metric >= std::max(0.0, cfg_.contact_force_threshold_N));
            contact_state_ = out.contact_on;
        }

        // -----------------------------
        // 3) force error 계산
        // -----------------------------
        Eigen::Vector3d f_err;
        if (cfg_.force_error_des_minus_meas) f_err = (in.f_des_base - f_meas_used);
        else                                 f_err = (f_meas_used - in.f_des_base);
        out.f_err = f_err;

        // [PATCH] deadband + axis sign 적용된 admittance 입력 force
        Eigen::Vector3d f_drive = f_err;
        for (int ax = 0; ax < 3; ++ax) {
            const double db = std::max(0.0, cfg_.force_deadband_N[ax]);
            if (std::fabs(f_drive(ax)) <= db) {
                f_drive(ax) = 0.0;
            } else {
                // deadband 이후 연속성 유지(선택): 부호 유지하며 db 만큼 차감
                f_drive(ax) = (f_drive(ax) > 0.0) ? (f_drive(ax) - db) : (f_drive(ax) + db);
            }

            // axis sign (빠른 부호 뒤집기 디버그용)
            const double sgn = (cfg_.force_error_axis_sign[ax] >= 0.0) ? 1.0 : -1.0;
            f_drive(ax) *= sgn;
        }
        out.f_drive = f_drive;

        // -----------------------------
        // 4) 상태 초기화
        // -----------------------------
        if (!initialized_) {
            // desired 기준으로 초기화 -> command jump 완화
            initialized_ = true;
            adm_x_.setZero();
            adm_v_.setZero();

            // p_cmd_prev는 desired/current 중 더 안전한 쪽으로 시작.
            // 기존 동작 호환을 위해 desired 기준 유지.
            p_cmd_prev_ = in.p_des_base;
        }

        // -----------------------------
        // 5) 축별 1D Admittance (position-only)
        //    M*xdd + D*xd + K*x = f_drive
        //    p_cmd = p_des + x
        // -----------------------------
        Eigen::Vector3d p_cmd = in.p_des_base;

        for (int ax = 0; ax < 3; ++ax) {
            if (!cfg_.force_ctrl_enable[ax]) {
                // 이 축은 순수 위치 유지
                adm_x_(ax) = 0.0;
                adm_v_(ax) = 0.0;
                p_cmd(ax)  = in.p_des_base(ax);
                continue;
            }

            const bool apply_adm = (!cfg_.use_contact_gate) || out.contact_on;

            if (apply_adm) {
                const double M = std::max(1e-6, cfg_.mass[ax]);
                const double D = std::max(0.0,  cfg_.damping[ax]);
                const double K = std::max(0.0,  cfg_.stiffness[ax]);

                // semi-implicit Euler
                const double x   = adm_x_(ax);
                const double xd  = adm_v_(ax);
                const double xdd = (f_drive(ax) - D * xd - K * x) / M;

                double xd_new = xd + xdd * dt;

                // [PATCH] velocity clamp (internal state blow-up 방지)
                const double vmax = std::fabs(cfg_.max_adm_velocity_mps[ax]);
                if (vmax > 0.0) {
                    xd_new = clampScalar_(xd_new, -vmax, vmax);
                }

                double x_new  = x  + xd_new * dt;

                // offset clamp + anti-windup
                const double x_lim = std::fabs(cfg_.max_offset_m[ax]);
                bool offset_clamped = false;
                if (x_lim > 0.0) {
                    const double x_clamped = clampScalar_(x_new, -x_lim, x_lim);
                    offset_clamped = (std::fabs(x_clamped - x_new) > 1e-15);
                    x_new = x_clamped;
                }

                if (cfg_.antiwindup_on_offset_clamp && offset_clamped) {
                    if (cfg_.zero_velocity_on_offset_clamp) {
                        xd_new = 0.0;
                    } else {
                        const double r = clampScalar_(cfg_.offset_clamp_velocity_damping, 0.0, 1.0);
                        xd_new *= r;
                    }
                }

                adm_v_(ax) = xd_new;
                adm_x_(ax) = x_new;
                out.offset_clamped[static_cast<std::size_t>(ax)] = offset_clamped;
            } else {
                // no-contact 시 감쇠/리셋
                if (cfg_.decay_when_no_contact) {
                    const double r = clampScalar_(cfg_.no_contact_decay_ratio, 0.0, 1.0);
                    adm_x_(ax) *= r;
                    adm_v_(ax) *= r;
                } else {
                    adm_x_(ax) = 0.0;
                    adm_v_(ax) = 0.0;
                }
            }

            p_cmd(ax) = in.p_des_base(ax) + adm_x_(ax);
        }

        // -----------------------------
        // 6) p_cmd 안정화 (desired 기준 offset 재클램프 + step limit)
        // -----------------------------
        for (int ax = 0; ax < 3; ++ax) {
            const double lo = in.p_des_base(ax) - std::fabs(cfg_.max_offset_m[ax]);
            const double hi = in.p_des_base(ax) + std::fabs(cfg_.max_offset_m[ax]);
            p_cmd(ax) = clampScalar_(p_cmd(ax), lo, hi);

            const double dmax = std::fabs(cfg_.max_step_m[ax]);
            const double d    = p_cmd(ax) - p_cmd_prev_(ax);

            double d_applied = d;
            if (dmax > 0.0) {
                d_applied = clampScalar_(d, -dmax, dmax);
                out.step_limited[static_cast<std::size_t>(ax)] = (std::fabs(d_applied - d) > 1e-15);
            }
            p_cmd(ax) = p_cmd_prev_(ax) + d_applied;
        }

        // [PATCH] 최종 command에 내부 adm state 동기화 (중요)
        // step limit / final clamp 때문에 실제 command와 adm_x_가 어긋나면 누적 오차 유발
        if (cfg_.sync_adm_state_to_final_cmd) {
            for (int ax = 0; ax < 3; ++ax) {
                if (!cfg_.force_ctrl_enable[ax]) {
                    adm_x_(ax) = 0.0;
                    adm_v_(ax) = 0.0;
                    continue;
                }
                adm_x_(ax) = p_cmd(ax) - in.p_des_base(ax);

                // step limit이 걸린 축은 속도도 보수적으로 감쇠
                if (out.step_limited[static_cast<std::size_t>(ax)]) {
                    adm_v_(ax) *= 0.5;
                }
            }
        }

        out.p_cmd_base   = p_cmd;
        out.adm_offset   = adm_x_;
        out.adm_velocity = adm_v_;

        // -----------------------------
        // 7) Method 1: whole-hand IK 재사용
        //    - 현재 fingertip들을 target로 복사
        //    - 선택 finger만 p_cmd로 변경
        //    - mask로 해당 finger만 활성화
        // -----------------------------
        std::array<Eigen::Vector3d,5> tips_target;
        for (int i = 0; i < 5; ++i) tips_target[i] = safeTip_(tips_cur, i);
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

        // finger mask: 선택된 손가락만 활성화
        ik_opt.mask = {{false, false, false, false, false}};
        ik_opt.weights = {{0.0, 0.0, 0.0, 0.0, 0.0}};
        ik_opt.mask[static_cast<std::size_t>(finger_id_)] = true;
        ik_opt.weights[static_cast<std::size_t>(finger_id_)] = 1.0;

        // [PATCH] IK seed: current q vs last successful q 선택
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
            out.q_cmd_4_mimic = q_sol20[qb + 2]; // q4 = q3

            last_success_valid_ = true;
            last_q_cmd_123_ = out.q_cmd_123;
            last_q_cmd20_ = q_sol20;

            p_cmd_prev_ = p_cmd;  // IK 성공 시에만 command history 갱신
            return out;
        }

        // -----------------------------
        // 8) IK 실패 처리
        // -----------------------------
        if (cfg_.damp_velocity_on_ik_fail) {
            const double r = clampScalar_(cfg_.ik_fail_velocity_damping, 0.0, 1.0);
            adm_v_ *= r;
        }

        // fallback q_cmd20 생성
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
            // 현재값 유지 fallback
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

        return out;
    }

    // 편의 오버로드 (질문에서 원하는 형태에 가까운 API)
    // - 객체가 finger_id를 이미 들고 있으므로 hand index는 생성 시 지정
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
    static int clampFingerId_(int fid)
    {
        if (fid < 0) return 0;
        if (fid > 4) return 4;
        return fid;
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

private:
    bool ok_{false};
    bool initialized_{false};

    std::shared_ptr<HandForwardKinematics> hand_fk_;
    std::shared_ptr<HandInverseKinematics> hand_ik_;

    int finger_id_{RING};
    Config cfg_;

    // admittance internal states (for one finger)
    Eigen::Vector3d adm_x_{Eigen::Vector3d::Zero()};     // offset [m]
    Eigen::Vector3d adm_v_{Eigen::Vector3d::Zero()};     // offset velocity [m/s]
    Eigen::Vector3d p_cmd_prev_{Eigen::Vector3d::Zero()};

    // [PATCH] measured force filter state
    Eigen::Vector3d f_meas_filt_{Eigen::Vector3d::Zero()};
    bool f_lpf_initialized_{false};

    // [PATCH] contact hysteresis state
    bool contact_state_{false};

    // fallback
    bool last_success_valid_{false};
    std::array<double,3> last_q_cmd_123_ {{0.0, 0.0, 0.0}};
    std::vector<double> last_q_cmd20_;
};

} // namespace dualarm_forcecon

#endif // DUALARM_FORCECON_KINEMATICS_HAND_ADMITTANCE_CONTROL_HPP_