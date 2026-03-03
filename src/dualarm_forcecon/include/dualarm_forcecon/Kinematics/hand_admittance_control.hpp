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
#include <sstream>
#include <functional>   // NEW

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp"
#include "dualarm_forcecon/Kinematics/hand_inverse_kinematics.hpp"

namespace dualarm_forcecon {

// ============================================================================
// HandAdmittanceControl (Strategy A: finger-wise independent controller)
// ----------------------------------------------------------------------------
// 목적:
// - "한 손가락"에 대해 1축 force + 2축 position-hold (hybrid) 지원
// - 접촉 순간 tangent anchor 고정
// - force target ramp / slip detection / slip guard 지원
// - 기존 HandIK/FK 재사용 (Method 1)
// - NEW(v19 draft): raw force(sensor frame) -> hand base frame 변환 지원
//   * f_meas_base = R_pre * R_dyn(q) * R_post * (f_sensor_raw - bias)
//   * R_dyn(q)는 콜백으로 주입 (legacy hand_func_v6 스타일 구조 채용)
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

    // NEW: q 기반 force rotation provider
    // 입력: q_hand_current20, finger_id
    // 출력: "동적 회전" R_dyn(q)
    // 최종 변환은 Config의 R_pre / R_post와 조합되어 사용됨.
    using ForceRotationProvider =
        std::function<Eigen::Matrix3d(const std::vector<double>&, int)>;

    struct Config {
        // --------------------------------------------------------------------
        // [STEP 4] Admittance MDK parameters (axis: x,y,z in HAND BASE frame)
        // --------------------------------------------------------------------
        std::array<double,3> mass;
        std::array<double,3> damping;
        std::array<double,3> stiffness;

        // 기본 축별 force-admittance 활성화 여부
        // false인 축은 p_cmd(axis)=p_des(axis) (순수 위치 유지)
        std::array<bool,3> force_ctrl_enable;

        // --------------------------------------------------------------------
        // [PATCH A] Hybrid Position/Force mode
        // - 법선축 1개만 force control, 나머지 2축은 tangent hold
        // --------------------------------------------------------------------
        bool use_hybrid_force_position_mode;
        int  hybrid_force_axis;   // 0=x, 1=y, 2=z (HAND BASE frame 기준)
        bool hold_tangent_anchor_on_contact;
        bool tangent_anchor_use_measured_pose;  // true: contact-on 시 p_meas를 anchor로 사용
        bool release_tangent_anchor_on_contact_off;

        // --------------------------------------------------------------------
        // [NEW v19] Measured force frame transform (sensor frame -> hand base)
        // --------------------------------------------------------------------
        // hand_admittance_control 내부 계산은 "항상 hand base frame" 기준으로 수행한다.
        // 아래 옵션을 켜면 StepInput::f_meas_sensor_raw 를 입력받아 hand base로 변환한다.
        bool use_meas_force_sensor_to_base_transform;

        // q 기반 동적 회전 R_dyn(q) 사용 여부 (provider 필요)
        bool meas_force_use_q_dependent_rotation;

        // provider가 없을 때 입력 f_meas_base로 fallback 허용 여부
        bool meas_force_fallback_to_input_base_if_transform_unavailable;

        // sensor frame에서 bias 제거
        bool meas_force_subtract_bias_in_sensor_frame;
        std::array<double,3> meas_force_bias_sensor;

        // legacy 스타일:
        // f_base = R_pre * R_dyn(q) * R_post * f_sensor
        // - R_post: 센서 장착 고정 오프셋 보정 (sensor -> link-ish)
        // - R_pre : 추가 프레임 정렬 (thumb base 보정 등)
        // row-major 저장
        std::array<double,9> meas_force_R_pre_rowmajor;
        std::array<double,9> meas_force_R_post_rowmajor;

        // --------------------------------------------------------------------
        // [PATCH B] Force signal preprocessing
        // --------------------------------------------------------------------
        // force error 축별 부호 보정 (빠른 디버그용)
        std::array<double,3> force_error_axis_sign;

        // 축별 force error deadband [N]
        std::array<double,3> force_deadband_N;

        // measured force LPF
        bool use_force_lpf;
        double force_lpf_tau_s;  // e.g., 0.03~0.08 s

        // force target ramp
        bool use_force_target_ramp;
        bool force_ramp_only_when_contact;
        std::array<double,3> force_target_ramp_rate_Nps;   // [N/s]
        std::array<double,3> force_target_release_rate_Nps;// [N/s], contact off 시 0으로 내리는 속도

        // --------------------------------------------------------------------
        // [PATCH C] Safety / limit
        // --------------------------------------------------------------------
        // Admittance offset 제한: p_cmd = p_ref + x_adm
        std::array<double,3> max_offset_m;   // [m]

        // 한 step 당 p_cmd 변화량 제한 (rate limit)
        std::array<double,3> max_step_m;     // [m/tick]

        // 내부 admittance velocity 제한
        std::array<double,3> max_adm_velocity_mps; // [m/s]

        // dt clamp
        double dt_min_s;
        double dt_max_s;

        // Force error 정의
        // true : f_err = f_des - f_meas
        // false: f_err = f_meas - f_des  (legacy hand_func_v6 식과 같은 부호 해석)
        bool force_error_des_minus_meas;

        // --------------------------------------------------------------------
        // [STEP 3] Contact gate
        // --------------------------------------------------------------------
        bool use_contact_gate;
        double contact_force_threshold_N;   // legacy/simple gate norm

        bool use_contact_hysteresis;
        double contact_on_threshold_N;
        double contact_off_threshold_N;

        // true이면 contact metric 계산 시 "실제로 force control하는 축"만 사용
        bool contact_gate_use_enabled_axes_only;

        // contact off 상태에서 offset/velocity 감쇠
        bool decay_when_no_contact;
        double no_contact_decay_ratio;      // 0~1, tick당 감소율

        // offset clamp anti-windup
        bool antiwindup_on_offset_clamp;
        bool zero_velocity_on_offset_clamp;
        double offset_clamp_velocity_damping; // 0~1

        // 최종 step-limit 적용 후 내부 adm state를 실제 command에 동기화
        bool sync_adm_state_to_final_cmd;

        // --------------------------------------------------------------------
        // [PATCH D] Slip detection / guard
        // --------------------------------------------------------------------
        bool use_slip_detection;
        double tangent_slip_threshold_m;     // e.g., 1~2 mm => 0.001~0.002
        bool use_slip_guard;
        double slip_guard_force_scale;       // 0~1, slip시 force target scale
        bool slip_guard_reanchor_tangent;    // slip 발생 시 tangent anchor를 현재 위치로 갱신
        double slip_guard_velocity_damping;  // adm_v *= r

        // --------------------------------------------------------------------
        // [STEP 7] IK solver options
        // --------------------------------------------------------------------
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

        bool prefer_last_success_q_seed;

        // IK 실패 시 처리
        bool keep_last_success_on_ik_fail;
        bool damp_velocity_on_ik_fail;
        double ik_fail_velocity_damping; // v <- v * factor

        // --------------------------------------------------------------------
        // [PATCH E] Debug (rclcpp)
        // --------------------------------------------------------------------
        bool debug_enable_rclcpp;
        int  debug_decimation;     // N-step마다 1회 출력
        bool debug_print_all_steps; // false면 핵심 요약 위주
        bool verbose;              // 기존 printf 스타일

        Config()
        : mass{{0.03, 0.03, 0.03}},
          damping{{8.0, 8.0, 8.0}},
          stiffness{{120.0, 120.0, 120.0}},
          force_ctrl_enable{{true, true, true}},
          use_hybrid_force_position_mode(true),
          hybrid_force_axis(0),                   // 기본 x축 force control
          hold_tangent_anchor_on_contact(true),
          tangent_anchor_use_measured_pose(true),
          release_tangent_anchor_on_contact_off(false),

          // NEW v19 force transform defaults
          use_meas_force_sensor_to_base_transform(false), // 기본은 기존 v18 호환
          meas_force_use_q_dependent_rotation(true),
          meas_force_fallback_to_input_base_if_transform_unavailable(true),
          meas_force_subtract_bias_in_sensor_frame(false),
          meas_force_bias_sensor{{0.0, 0.0, 0.0}},
          meas_force_R_pre_rowmajor{{1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0}},
          meas_force_R_post_rowmajor{{1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0}},

          force_error_axis_sign{{1.0, 1.0, 1.0}},
          force_deadband_N{{0.0, 0.0, 0.0}},
          use_force_lpf(true),
          force_lpf_tau_s(0.04),
          use_force_target_ramp(true),
          // v19: pre-contact push를 위해 기본 false 권장 (이전 v18 이슈 완화)
          force_ramp_only_when_contact(false),
          force_target_ramp_rate_Nps{{20.0, 20.0, 20.0}},
          force_target_release_rate_Nps{{40.0, 40.0, 40.0}},
          max_offset_m{{0.005, 0.001, 0.001}},
          max_step_m{{0.0003, 0.00005, 0.00005}},
          max_adm_velocity_mps{{0.05, 0.02, 0.02}},
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
          debug_decimation(20),
          debug_print_all_steps(false),
          verbose(false)
        {}
    };

    struct StepInput {
        Eigen::Vector3d p_des_base{Eigen::Vector3d::Zero()};   // desired fingertip pos in hand base frame
        Eigen::Vector3d f_des_base{Eigen::Vector3d::Zero()};   // desired force in hand base frame

        // v18 legacy-compatible path: 이미 base frame으로 정렬된 measured force
        Eigen::Vector3d f_meas_base{Eigen::Vector3d::Zero()};

        // NEW v19 path: raw sensor-frame measured force
        // use_meas_force_sensor_to_base_transform=true 이고 has_f_meas_sensor_raw=true 이면 우선 사용됨
        Eigen::Vector3d f_meas_sensor_raw{Eigen::Vector3d::Zero()};
        bool has_f_meas_sensor_raw{false};

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
        double contact_metric_N{0.0};

        // diagnostics: position
        Eigen::Vector3d p_meas_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_des_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d p_ref_base{Eigen::Vector3d::Zero()};   // 실제 adm 기준점 (anchor 반영)
        Eigen::Vector3d p_cmd_base{Eigen::Vector3d::Zero()};

        // diagnostics: force transform (NEW v19)
        bool meas_force_transform_applied{false};
        bool meas_force_transform_used_fallback_base{false};
        Eigen::Vector3d f_meas_sensor_raw{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_sensor_bias_removed{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base_from_transform{Eigen::Vector3d::Zero()};

        // diagnostics: force
        Eigen::Vector3d f_meas_base_raw{Eigen::Vector3d::Zero()};   // LPF 전, 제어 프레임(base)
        Eigen::Vector3d f_meas_base_filt{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_meas_base{Eigen::Vector3d::Zero()};       // control에 실제 사용된 값
        Eigen::Vector3d f_des_base{Eigen::Vector3d::Zero()};        // 입력 목표
        Eigen::Vector3d f_des_ramped{Eigen::Vector3d::Zero()};      // ramp 후 목표
        Eigen::Vector3d f_des_used{Eigen::Vector3d::Zero()};        // slip guard 반영 후 최종 목표
        Eigen::Vector3d f_err{Eigen::Vector3d::Zero()};
        Eigen::Vector3d f_drive{Eigen::Vector3d::Zero()};           // sign/deadband 처리 후 adm 입력

        // diagnostics: admittance states (legacy 대응: delta_pos / delta_velo)
        Eigen::Vector3d adm_offset{Eigen::Vector3d::Zero()};
        Eigen::Vector3d adm_velocity{Eigen::Vector3d::Zero()};

        std::array<bool,3> axis_force_enable_eff{{false,false,false}};
        int hybrid_force_axis{-1};

        std::array<bool,3> offset_clamped{{false,false,false}};
        std::array<bool,3> step_limited{{false,false,false}};

        // tangent anchor / slip
        bool tangent_anchor_valid{false};
        Eigen::Vector3d tangent_anchor_base{Eigen::Vector3d::Zero()};
        Eigen::Vector3d tangent_drift_vec{Eigen::Vector3d::Zero()};
        double tangent_drift_norm{0.0};
        bool slip_detected{false};

        // outputs for selected finger only
        std::array<double,3> q_cmd_123 {{0.0, 0.0, 0.0}};
        double q_cmd_4_mimic{0.0}; // = q3

        // optional full hand command (20DoF)
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

    // 선택적 rclcpp debug logger 주입
    void setDebugLogger(const rclcpp::Logger& logger) {
        debug_logger_ = logger;
        has_debug_logger_ = true;
    }

    // NEW v19: q 기반 동적 회전 provider 등록
    void setMeasuredForceRotationProvider(const ForceRotationProvider& provider) {
        meas_force_rot_provider_ = provider;
        has_meas_force_rot_provider_ = static_cast<bool>(meas_force_rot_provider_);
    }

    void clearMeasuredForceRotationProvider() {
        meas_force_rot_provider_ = nullptr;
        has_meas_force_rot_provider_ = false;
    }

    // NEW v19: 고정 회전 설정 helper (row-major)
    void setMeasuredForcePreRotationRowMajor(const std::array<double,9>& r_pre_rowmajor) {
        cfg_.meas_force_R_pre_rowmajor = r_pre_rowmajor;
    }

    void setMeasuredForcePostRotationRowMajor(const std::array<double,9>& r_post_rowmajor) {
        cfg_.meas_force_R_post_rowmajor = r_post_rowmajor;
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

    // =========================================================================
    // step()
    // =========================================================================
    StepOutput step(const StepInput& in)
    {
        StepOutput out;
        out.controller_ok = false;

        if (!ok_ || !hand_fk_ || !hand_ik_) return out;
        if (in.q_hand_current20.size() < 15) return out;

        out.controller_ok = true;

        // =====================================================================
        // STEP 0) dt stabilize / basic guards
        // =====================================================================
        const double dt = clampScalar_(in.dt_s, cfg_.dt_min_s, cfg_.dt_max_s);
        out.used_dt_s = dt;

        // =====================================================================
        // STEP 1) Current fingertip pose (FK)
        // - 현재 손 관절로 fingertip 위치 계산
        // =====================================================================
        const std::vector<Eigen::Vector3d> tips_cur = hand_fk_->computeFingertips(in.q_hand_current20);
        const Eigen::Vector3d p_meas = safeTip_(tips_cur, finger_id_);

        out.p_meas_base = p_meas;
        out.p_des_base  = in.p_des_base;

        // =====================================================================
        // STEP 2) Measured force preprocessing input stage
        // - NEW v19: sensor frame raw force -> hand base transform (q-dependent hook)
        // - 이후 LPF는 base frame 값에 적용
        // =====================================================================
        Eigen::Vector3d f_meas_base_input = in.f_meas_base; // v18 fallback path
        out.meas_force_transform_applied = false;
        out.meas_force_transform_used_fallback_base = false;
        out.f_meas_sensor_raw = in.f_meas_sensor_raw;
        out.f_meas_sensor_bias_removed = in.f_meas_sensor_raw;
        out.f_meas_base_from_transform = Eigen::Vector3d::Zero();

        if (cfg_.use_meas_force_sensor_to_base_transform && in.has_f_meas_sensor_raw) {
            Eigen::Vector3d f_sensor = in.f_meas_sensor_raw;

            if (cfg_.meas_force_subtract_bias_in_sensor_frame) {
                f_sensor.x() -= cfg_.meas_force_bias_sensor[0];
                f_sensor.y() -= cfg_.meas_force_bias_sensor[1];
                f_sensor.z() -= cfg_.meas_force_bias_sensor[2];
            }
            out.f_meas_sensor_bias_removed = f_sensor;

            const Eigen::Matrix3d R_pre  = mat3FromRowMajor_(cfg_.meas_force_R_pre_rowmajor);
            const Eigen::Matrix3d R_post = mat3FromRowMajor_(cfg_.meas_force_R_post_rowmajor);

            Eigen::Matrix3d R_dyn = Eigen::Matrix3d::Identity();
            bool dyn_ok = true;

            if (cfg_.meas_force_use_q_dependent_rotation) {
                if (has_meas_force_rot_provider_ && meas_force_rot_provider_) {
                    R_dyn = meas_force_rot_provider_(in.q_hand_current20, finger_id_);
                    dyn_ok = matrixFinite_(R_dyn);
                } else {
                    dyn_ok = false;
                }
            }

            if (!cfg_.meas_force_use_q_dependent_rotation || dyn_ok) {
                f_meas_base_input = R_pre * R_dyn * R_post * f_sensor;
                out.meas_force_transform_applied = true;
                out.f_meas_base_from_transform = f_meas_base_input;
            } else {
                if (cfg_.meas_force_fallback_to_input_base_if_transform_unavailable) {
                    f_meas_base_input = in.f_meas_base;
                    out.meas_force_transform_used_fallback_base = true;
                } else {
                    // provider 없는데 transform required면 0으로 두어 안전측으로 처리
                    f_meas_base_input.setZero();
                }
            }
        } else {
            // v18 path 그대로 사용
            f_meas_base_input = in.f_meas_base;
        }

        // =====================================================================
        // STEP 2.1) Measured force LPF (base frame)
        // =====================================================================
        out.f_meas_base_raw = f_meas_base_input;

        Eigen::Vector3d f_meas_used = f_meas_base_input;
        if (cfg_.use_force_lpf) {
            const double tau = std::max(1e-6, cfg_.force_lpf_tau_s);
            const double alpha = clampScalar_(dt / (tau + dt), 0.0, 1.0);

            if (!f_lpf_initialized_) {
                f_meas_filt_ = f_meas_base_input;
                f_lpf_initialized_ = true;
            } else {
                f_meas_filt_ = (1.0 - alpha) * f_meas_filt_ + alpha * f_meas_base_input;
            }
            f_meas_used = f_meas_filt_;
        } else {
            f_meas_filt_ = f_meas_base_input;
            f_lpf_initialized_ = true;
            f_meas_used = f_meas_base_input;
        }

        out.f_meas_base_filt = f_meas_filt_;
        out.f_meas_base      = f_meas_used;
        out.f_des_base       = in.f_des_base;

        // =====================================================================
        // STEP 2.5) Hybrid axis configuration (1-axis force + 2-axis hold)
        // - cfg.force_ctrl_enable를 기반으로 하되, hybrid 모드면 강제로 1축만 활성
        // =====================================================================
        std::array<bool,3> axis_force_enable_eff = cfg_.force_ctrl_enable;
        int hybrid_force_axis = -1;

        if (cfg_.use_hybrid_force_position_mode) {
            hybrid_force_axis = clampAxis_(cfg_.hybrid_force_axis);
            axis_force_enable_eff = {{false, false, false}};
            axis_force_enable_eff[static_cast<std::size_t>(hybrid_force_axis)] = true;
        }

        out.axis_force_enable_eff = axis_force_enable_eff;
        out.hybrid_force_axis     = hybrid_force_axis;

        // =====================================================================
        // STEP 3) Contact detection (gate / hysteresis)
        // - force metric를 기반으로 contact on/off 판정
        // - enabled axis만 metric에 반영 가능
        // =====================================================================
        auto contact_metric_norm = [&](const Eigen::Vector3d& f)->double {
            if (!cfg_.contact_gate_use_enabled_axes_only) {
                return f.norm();
            }
            double s2 = 0.0;
            bool any = false;
            for (int ax = 0; ax < 3; ++ax) {
                if (axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    s2 += f(ax) * f(ax);
                    any = true;
                }
            }
            return any ? std::sqrt(s2) : f.norm();
        };

        const double f_metric = contact_metric_norm(f_meas_used);
        out.contact_metric_N = f_metric;
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

        // =====================================================================
        // STEP 3.5) Tangent anchor handling (contact-on 순간 접선축 고정)
        // - force축이 아닌 2개 축을 anchor에 고정
        // =====================================================================
        Eigen::Vector3d p_ref = in.p_des_base;  // admittance 기준점 (anchor 반영 전 기본값)

        if (cfg_.hold_tangent_anchor_on_contact) {
            if (out.contact_rising_edge) {
                tangent_anchor_valid_ = true;
                tangent_anchor_p_ = cfg_.tangent_anchor_use_measured_pose ? p_meas : in.p_des_base;
            }

            if (out.contact_falling_edge && cfg_.release_tangent_anchor_on_contact_off) {
                tangent_anchor_valid_ = false;
            }

            if (out.contact_on && tangent_anchor_valid_) {
                for (int ax = 0; ax < 3; ++ax) {
                    if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                        p_ref(ax) = tangent_anchor_p_(ax);  // tangent 2축 hold
                    }
                }
            }
        }

        out.p_ref_base = p_ref;
        out.tangent_anchor_valid = tangent_anchor_valid_;
        out.tangent_anchor_base  = tangent_anchor_p_;

        // =====================================================================
        // STEP 3.6) Force target ramp (0 -> Fd)
        // - legacy 스타일 forcecon에서 pre-contact push가 가능하도록
        //   hybrid force axis는 contact 전에도 ramp 허용 가능
        // =====================================================================
        Eigen::Vector3d f_des_ramped = in.f_des_base;

        if (!f_des_ramp_initialized_) {
            f_des_ramped_ = Eigen::Vector3d::Zero();
            f_des_ramp_initialized_ = true;
        }

        if (cfg_.use_force_target_ramp) {
            for (int ax = 0; ax < 3; ++ax) {
                const bool axis_force_on = axis_force_enable_eff[static_cast<std::size_t>(ax)];

                // tangent hold 축은 force target 0 유지
                if (!axis_force_on) {
                    f_des_ramped_(ax) = 0.0;
                    continue;
                }

                const double target_contact = in.f_des_base(ax);
                const double target_release = 0.0;

                const bool is_hybrid_force_axis =
                    (cfg_.use_hybrid_force_position_mode && (ax == hybrid_force_axis));

                const bool allow_precontact_ramp_for_force_axis =
                    axis_force_on && (!cfg_.use_hybrid_force_position_mode || is_hybrid_force_axis);

                const bool allow_ramp_to_target =
                    (!cfg_.force_ramp_only_when_contact) || out.contact_on || allow_precontact_ramp_for_force_axis;

                const double target = allow_ramp_to_target ? target_contact : target_release;

                const double rate_up   = std::max(1e-9, cfg_.force_target_ramp_rate_Nps[ax]);
                const double rate_down = std::max(1e-9, cfg_.force_target_release_rate_Nps[ax]);

                const double diff = target - f_des_ramped_(ax);
                const double rate = (std::fabs(target) >= std::fabs(f_des_ramped_(ax))) ? rate_up : rate_down;
                const double step = rate * dt;

                if (std::fabs(diff) <= step) {
                    f_des_ramped_(ax) = target;
                } else {
                    f_des_ramped_(ax) += ((diff > 0.0) ? step : -step);
                }
            }
            f_des_ramped = f_des_ramped_;
        } else {
            f_des_ramped = in.f_des_base;
            for (int ax = 0; ax < 3; ++ax) {
                if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    f_des_ramped(ax) = 0.0;
                }
            }
            f_des_ramped_ = f_des_ramped;
            f_des_ramp_initialized_ = true;
        }

        out.f_des_ramped = f_des_ramped;

        // =====================================================================
        // STEP 3.7) Slip detection / guard (tangent drift 기반)
        // - 접촉 중 tangent drift가 임계값 초과하면 slip으로 판단
        // - 선택적으로 force target 축소 / tangent re-anchor
        // =====================================================================
        Eigen::Vector3d tangent_drift = Eigen::Vector3d::Zero();
        double tangent_drift_norm = 0.0;
        bool slip_detected = false;

        if (cfg_.use_slip_detection && out.contact_on && tangent_anchor_valid_) {
            for (int ax = 0; ax < 3; ++ax) {
                if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    tangent_drift(ax) = p_meas(ax) - tangent_anchor_p_(ax);
                } else {
                    tangent_drift(ax) = 0.0;
                }
            }
            tangent_drift_norm = tangent_drift.norm();
            slip_detected = (tangent_drift_norm > std::max(0.0, cfg_.tangent_slip_threshold_m));
        }

        Eigen::Vector3d f_des_used = f_des_ramped;

        if (cfg_.use_slip_guard && slip_detected) {
            const double s = clampScalar_(cfg_.slip_guard_force_scale, 0.0, 1.0);

            for (int ax = 0; ax < 3; ++ax) {
                if (axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    f_des_used(ax) *= s;
                } else {
                    f_des_used(ax) = 0.0;
                }
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

        // =====================================================================
        // STEP 4) Force error / force drive for admittance
        // - v19도 v18 구조 유지 (legacy hand_func_v6와 동형)
        //   xdd = (force_term - D*xd - K*x)/M
        // - force_term의 부호는 force_error_des_minus_meas로 제어
        // =====================================================================
        Eigen::Vector3d f_err;
        if (cfg_.force_error_des_minus_meas) f_err = (f_des_used - f_meas_used);
        else                                 f_err = (f_meas_used - f_des_used); // legacy와 같은 해석
        out.f_err = f_err;

        Eigen::Vector3d f_drive = f_err;
        for (int ax = 0; ax < 3; ++ax) {
            if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                f_drive(ax) = 0.0;  // tangent hold 축에는 force drive를 넣지 않음
                continue;
            }

            const double db = std::max(0.0, cfg_.force_deadband_N[ax]);
            if (std::fabs(f_drive(ax)) <= db) {
                f_drive(ax) = 0.0;
            } else {
                f_drive(ax) = (f_drive(ax) > 0.0) ? (f_drive(ax) - db) : (f_drive(ax) + db);
            }

            const double sgn = (cfg_.force_error_axis_sign[ax] >= 0.0) ? 1.0 : -1.0;
            f_drive(ax) *= sgn;
        }
        out.f_drive = f_drive;

        // =====================================================================
        // STEP 5) Initialize internal states (first call)
        // =====================================================================
        if (!initialized_) {
            initialized_ = true;
            adm_x_.setZero();
            adm_v_.setZero();
            p_cmd_prev_ = p_ref;
        }

        // =====================================================================
        // STEP 6) 1D Admittance per-axis (legacy 구조와 동일한 물리식)
        // - force축: M*xdd + D*xd + K*x = f_drive
        // - tangent축: hold (p_cmd = p_ref)
        // - p_cmd = p_ref + x_adm
        // =====================================================================
        Eigen::Vector3d p_cmd = p_ref;

        for (int ax = 0; ax < 3; ++ax) {
            const bool axis_force_on = axis_force_enable_eff[static_cast<std::size_t>(ax)];

            if (!axis_force_on) {
                adm_x_(ax) = 0.0;
                adm_v_(ax) = 0.0;
                p_cmd(ax)  = p_ref(ax);
                continue;
            }

            // v19: pre-contact 상태에서도 "force target이 존재하는 force축"은 push 허용
            const bool force_target_present = (std::fabs(f_des_used(ax)) > 1e-9);

            const bool is_hybrid_force_axis =
                (cfg_.use_hybrid_force_position_mode && (ax == hybrid_force_axis));

            const bool allow_precontact_push =
                axis_force_on && force_target_present &&
                (!cfg_.use_hybrid_force_position_mode || is_hybrid_force_axis);

            const bool apply_adm =
                (!cfg_.use_contact_gate) || out.contact_on || allow_precontact_push;

            if (apply_adm) {
                const double M = std::max(1e-6, cfg_.mass[ax]);
                const double D = std::max(0.0,  cfg_.damping[ax]);
                const double K = std::max(0.0,  cfg_.stiffness[ax]);

                // legacy 대응:
                // delta_accel = (f_drive - D*delta_velo - K*delta_pos) / M
                const double x   = adm_x_(ax);
                const double xd  = adm_v_(ax);
                const double xdd = (f_drive(ax) - D * xd - K * x) / M;

                double xd_new = xd + xdd * dt;

                const double vmax = std::fabs(cfg_.max_adm_velocity_mps[ax]);
                if (vmax > 0.0) {
                    xd_new = clampScalar_(xd_new, -vmax, vmax);
                }

                double x_new  = x + xd_new * dt;

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

        // =====================================================================
        // STEP 6.5) Final p_cmd stabilization (offset clamp around p_ref + step limit)
        // - v19: 접촉 중 tangent축은 hard lock (step-limit 우회)로 미끄러짐 억제
        // =====================================================================
        for (int ax = 0; ax < 3; ++ax) {
            // 접촉 중 tangent축 exact hold
            if (out.contact_on && tangent_anchor_valid_ &&
                !axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                p_cmd(ax) = p_ref(ax);
                out.step_limited[static_cast<std::size_t>(ax)] = false;
                continue;
            }

            const double lo = p_ref(ax) - std::fabs(cfg_.max_offset_m[ax]);
            const double hi = p_ref(ax) + std::fabs(cfg_.max_offset_m[ax]);
            p_cmd(ax) = clampScalar_(p_cmd(ax), lo, hi);

            const double dmax = std::fabs(cfg_.max_step_m[ax]);
            const double d    = p_cmd(ax) - p_cmd_prev_(ax);

            double d_applied = d;
            if (dmax > 0.0) {
                d_applied = clampScalar_(d, -dmax, dmax);
                out.step_limited[static_cast<std::size_t>(ax)] =
                    (std::fabs(d_applied - d) > 1e-15);
            }
            p_cmd(ax) = p_cmd_prev_(ax) + d_applied;
        }

        // internal adm state sync
        if (cfg_.sync_adm_state_to_final_cmd) {
            for (int ax = 0; ax < 3; ++ax) {
                if (!axis_force_enable_eff[static_cast<std::size_t>(ax)]) {
                    adm_x_(ax) = 0.0;
                    adm_v_(ax) = 0.0;
                    continue;
                }
                adm_x_(ax) = p_cmd(ax) - p_ref(ax);

                if (out.step_limited[static_cast<std::size_t>(ax)]) {
                    adm_v_(ax) *= 0.5;
                }
            }
        }

        out.p_cmd_base   = p_cmd;
        out.adm_offset   = adm_x_;
        out.adm_velocity = adm_v_;

        // =====================================================================
        // STEP 7) IK target construction (Method 1)
        // - 현재 fingertips를 기준으로 복사
        // - 선택 finger만 p_cmd로 교체
        // - IK mask로 해당 finger만 활성
        // =====================================================================
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

        ik_opt.mask = {{false, false, false, false, false}};
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
            out.q_cmd_4_mimic = q_sol20[qb + 2];

            last_success_valid_ = true;
            last_q_cmd_123_ = out.q_cmd_123;
            last_q_cmd20_ = q_sol20;

            p_cmd_prev_ = p_cmd;

            debugStep_(out);
            return out;
        }

        // =====================================================================
        // STEP 7.5) IK fail handling / fallback
        // =====================================================================
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

    // convenience overload (v18 호환)
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
    // ------------------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------------------
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

    static Eigen::Matrix3d mat3FromRowMajor_(const std::array<double,9>& a)
    {
        Eigen::Matrix3d R;
        R << a[0], a[1], a[2],
             a[3], a[4], a[5],
             a[6], a[7], a[8];
        return R;
    }

    static bool matrixFinite_(const Eigen::Matrix3d& R)
    {
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                if (!std::isfinite(R(r,c))) return false;
            }
        }
        return true;
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
            "[HandAdm][%s] dt=%.4f contact=%d rise=%d fall=%d cm=%.3f slip=%d | "
            "axis_force=[%d%d%d] hax=%d | tf_applied=%d tf_fb=%d | "
            "p_des=(%.4f %.4f %.4f) p_ref=(%.4f %.4f %.4f) p_cmd=(%.4f %.4f %.4f) | "
            "f_meas=(%.3f %.3f %.3f) f_des=(%.3f %.3f %.3f) f_ramp=(%.3f %.3f %.3f) f_used=(%.3f %.3f %.3f) | "
            "f_err=(%.3f %.3f %.3f)",
            fingerName(finger_id_).c_str(),
            out.used_dt_s,
            static_cast<int>(out.contact_on),
            static_cast<int>(out.contact_rising_edge),
            static_cast<int>(out.contact_falling_edge),
            out.contact_metric_N,
            static_cast<int>(out.slip_detected),
            static_cast<int>(out.axis_force_enable_eff[0]),
            static_cast<int>(out.axis_force_enable_eff[1]),
            static_cast<int>(out.axis_force_enable_eff[2]),
            out.hybrid_force_axis,
            static_cast<int>(out.meas_force_transform_applied),
            static_cast<int>(out.meas_force_transform_used_fallback_base),
            out.p_des_base.x(), out.p_des_base.y(), out.p_des_base.z(),
            out.p_ref_base.x(), out.p_ref_base.y(), out.p_ref_base.z(),
            out.p_cmd_base.x(), out.p_cmd_base.y(), out.p_cmd_base.z(),
            out.f_meas_base.x(), out.f_meas_base.y(), out.f_meas_base.z(),
            out.f_des_base.x(), out.f_des_base.y(), out.f_des_base.z(),
            out.f_des_ramped.x(), out.f_des_ramped.y(), out.f_des_ramped.z(),
            out.f_des_used.x(), out.f_des_used.y(), out.f_des_used.z(),
            out.f_err.x(), out.f_err.y(), out.f_err.z());

        if (!cfg_.debug_print_all_steps) return;

        RCLCPP_INFO(
            logger,
            "  [TF] f_sensor_raw=(%.3f %.3f %.3f) bias_rm=(%.3f %.3f %.3f) f_base_tf=(%.3f %.3f %.3f)",
            out.f_meas_sensor_raw.x(), out.f_meas_sensor_raw.y(), out.f_meas_sensor_raw.z(),
            out.f_meas_sensor_bias_removed.x(), out.f_meas_sensor_bias_removed.y(), out.f_meas_sensor_bias_removed.z(),
            out.f_meas_base_from_transform.x(), out.f_meas_base_from_transform.y(), out.f_meas_base_from_transform.z());

        RCLCPP_INFO(
            logger,
            "  [S2 force] raw_base=(%.3f %.3f %.3f) filt=(%.3f %.3f %.3f) drive=(%.3f %.3f %.3f)",
            out.f_meas_base_raw.x(), out.f_meas_base_raw.y(), out.f_meas_base_raw.z(),
            out.f_meas_base_filt.x(), out.f_meas_base_filt.y(), out.f_meas_base_filt.z(),
            out.f_drive.x(), out.f_drive.y(), out.f_drive.z());

        RCLCPP_INFO(
            logger,
            "  [S3 anchor/slip] valid=%d anchor=(%.4f %.4f %.4f) drift=(%.4f %.4f %.4f) | drift_norm=%.5f th=%.5f",
            static_cast<int>(out.tangent_anchor_valid),
            out.tangent_anchor_base.x(), out.tangent_anchor_base.y(), out.tangent_anchor_base.z(),
            out.tangent_drift_vec.x(), out.tangent_drift_vec.y(), out.tangent_drift_vec.z(),
            out.tangent_drift_norm,
            cfg_.tangent_slip_threshold_m);

        RCLCPP_INFO(
            logger,
            "  [S6 adm] x=(%.5f %.5f %.5f) v=(%.5f %.5f %.5f) | clamp_off=[%d%d%d] step_lim=[%d%d%d] | ik_ok=%d",
            out.adm_offset.x(), out.adm_offset.y(), out.adm_offset.z(),
            out.adm_velocity.x(), out.adm_velocity.y(), out.adm_velocity.z(),
            static_cast<int>(out.offset_clamped[0]), static_cast<int>(out.offset_clamped[1]), static_cast<int>(out.offset_clamped[2]),
            static_cast<int>(out.step_limited[0]),   static_cast<int>(out.step_limited[1]),   static_cast<int>(out.step_limited[2]),
            static_cast<int>(out.ik_ok));
    }

private:
    bool ok_{false};
    bool initialized_{false};

    std::shared_ptr<HandForwardKinematics> hand_fk_;
    std::shared_ptr<HandInverseKinematics> hand_ik_;

    int finger_id_{RING};
    Config cfg_;

    // admittance internal states (for one finger)
    // legacy 대응: adm_x_ ~ delta_pos, adm_v_ ~ delta_velo
    Eigen::Vector3d adm_x_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d adm_v_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d p_cmd_prev_{Eigen::Vector3d::Zero()};

    // measured force filter state (base frame)
    Eigen::Vector3d f_meas_filt_{Eigen::Vector3d::Zero()};
    bool f_lpf_initialized_{false};

    // contact hysteresis state
    bool contact_state_{false};
    bool prev_contact_state_{false};

    // tangent anchor state (for 2-axis hold)
    bool tangent_anchor_valid_{false};
    Eigen::Vector3d tangent_anchor_p_{Eigen::Vector3d::Zero()};

    // force target ramp state
    Eigen::Vector3d f_des_ramped_{Eigen::Vector3d::Zero()};
    bool f_des_ramp_initialized_{false};

    // fallback / IK seed
    bool last_success_valid_{false};
    std::array<double,3> last_q_cmd_123_ {{0.0, 0.0, 0.0}};
    std::vector<double> last_q_cmd20_;

    // NEW v19: q-dependent force rotation provider
    ForceRotationProvider meas_force_rot_provider_{nullptr};
    bool has_meas_force_rot_provider_{false};

    // debug
    rclcpp::Logger debug_logger_{rclcpp::get_logger("HandAdmittanceControl")};
    bool has_debug_logger_{false};
    int debug_counter_{0};
};

} // namespace dualarm_forcecon

#endif // DUALARM_FORCECON_KINEMATICS_HAND_ADMITTANCE_CONTROL_HPP_