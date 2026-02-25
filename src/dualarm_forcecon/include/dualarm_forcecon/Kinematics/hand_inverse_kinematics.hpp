#ifndef DUALARM_FORCECON_KINEMATICS_HAND_INVERSE_KINEMATICS_HPP_
#define DUALARM_FORCECON_KINEMATICS_HAND_INVERSE_KINEMATICS_HPP_

#include <Eigen/Dense>

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdio>

#include "dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp"

namespace dualarm_forcecon {

class HandInverseKinematics {
public:
    // =========================================================================
    // Options (C++14-safe)
    // =========================================================================
    struct Options {
        int    max_iters;      // per-finger
        double tol_pos_m;      // per-finger position error tolerance
        double lambda;         // DLS damping initial
        double lambda_min;
        double lambda_max;
        double alpha;          // initial step size
        double alpha_min;
        double max_step;       // per-joint dq clamp [rad]
        double mu_posture;     // posture regularization (toward init)
        bool   verbose;

        // finger mask/weights order: thumb,index,middle,ring,baby
        std::array<bool,5>   mask;
        std::array<double,5> weights;

        // joint limit mode
        bool use_urdf_like_limits;
        double q_min_global;
        double q_max_global;

        Options()
        : max_iters(80),
          tol_pos_m(5e-4),
          lambda(1e-2),
          lambda_min(1e-5),
          lambda_max(1.0),
          alpha(0.8),
          alpha_min(0.05),
          max_step(0.15),
          mu_posture(1e-4),
          verbose(false),
          use_urdf_like_limits(true),
          q_min_global(-M_PI),
          q_max_global( M_PI)
        {
            mask    = {{true, true, true, true, true}};
            weights = {{1.0,  1.0,  1.0,  1.0,  1.0 }};
        }
    };

public:
    // 기존 스타일 호환 생성자
    HandInverseKinematics() = default;

    HandInverseKinematics(const std::string& urdf_path,
                          const std::string& base_name_unused,
                          const std::vector<std::string>& tips_unused)
    {
        Init(urdf_path, base_name_unused, tips_unused);
    }

    bool Init(const std::string& urdf_path,
              const std::string& base_name_unused,
              const std::vector<std::string>& tips_unused)
    {
        fk_ = std::make_shared<HandForwardKinematics>(urdf_path, base_name_unused, tips_unused);
        ok_ = (fk_ != nullptr && fk_->ok());
        return ok_;
    }

    bool isOk() const { return ok_; }
    bool ok()   const { return ok_; }

    // =========================================================================
    // Main API (20DoF compatibility)
    // q_init20 / q_sol20:
    //   [thumb1..4, index1..4, middle1..4, ring1..4, baby1..4]
    // tips_target order:
    //   thumb, index, middle, ring, baby
    // Internal solver uses 15DoF independent model (joint1~3 per finger),
    // and expands back with mimic rule joint4 = joint3.
    // =========================================================================
    bool solveIKFingertips(const std::vector<double>& q_init20,
                           const std::array<Eigen::Vector3d,5>& tips_target,
                           std::vector<double>& q_sol20,
                           const Options& opt)
    {
        if (!ok_ || !fk_) return false;
        if (q_init20.size() < 15) return false;

        std::vector<double> z15_init;
        NormalizeTo15DoF_(q_init20, z15_init); // accepts 15 or 20 input

        std::vector<double> z15_sol;
        const bool all_ok = solveIKFingertips15(z15_init, tips_target, z15_sol, opt);

        Expand15To20_(z15_sol, q_sol20);
        return all_ok;
    }

    bool solveIKFingertips(const std::vector<double>& q_init20,
                           const std::array<Eigen::Vector3d,5>& tips_target,
                           std::vector<double>& q_sol20)
    {
        Options opt;
        return solveIKFingertips(q_init20, tips_target, q_sol20, opt);
    }

    // vector target overload
    bool solveIKFingertips(const std::vector<double>& q_init20,
                           const std::vector<Eigen::Vector3d>& tips_target_vec,
                           std::vector<double>& q_sol20,
                           const Options& opt)
    {
        if (tips_target_vec.size() < 5) return false;
        std::array<Eigen::Vector3d,5> tips_target;
        for (int i = 0; i < 5; ++i) tips_target[i] = tips_target_vec[i];
        return solveIKFingertips(q_init20, tips_target, q_sol20, opt);
    }

    bool solveIKFingertips(const std::vector<double>& q_init20,
                           const std::vector<Eigen::Vector3d>& tips_target_vec,
                           std::vector<double>& q_sol20)
    {
        Options opt;
        return solveIKFingertips(q_init20, tips_target_vec, q_sol20, opt);
    }

    // =========================================================================
    // Native 15DoF API
    // z15 order:
    // [thumb1,2,3, index1,2,3, middle1,2,3, ring1,2,3, baby1,2,3]
    // =========================================================================
    bool solveIKFingertips15(const std::vector<double>& z15_init,
                             const std::array<Eigen::Vector3d,5>& tips_target,
                             std::vector<double>& z15_sol,
                             const Options& opt)
    {
        if (!ok_ || !fk_) return false;
        if (z15_init.size() != 15) return false;

        Eigen::VectorXd z(15);
        Eigen::VectorXd z_ref(15);
        for (int i = 0; i < 15; ++i) {
            z(i) = z15_init[i];
            z_ref(i) = z15_init[i];
        }

        bool any_active = false;
        bool all_ok = true;

        // Finger-wise independent IK
        for (int finger_id = 0; finger_id < 5; ++finger_id) {
            if (!opt.mask[finger_id]) continue;
            any_active = true;

            const bool ok_f = SolveSingleFingerIK_(finger_id, z, z_ref, tips_target[finger_id], opt);
            if (!ok_f) all_ok = false;
        }

        if (!any_active) {
            z15_sol = z15_init;
            return true;
        }

        z15_sol.resize(15);
        for (int i = 0; i < 15; ++i) z15_sol[i] = z(i);
        return all_ok;
    }

    bool solveIKFingertips15(const std::vector<double>& z15_init,
                             const std::array<Eigen::Vector3d,5>& tips_target,
                             std::vector<double>& z15_sol)
    {
        Options opt;
        return solveIKFingertips15(z15_init, tips_target, z15_sol, opt);
    }

private:
    // finger order: thumb,index,middle,ring,baby
    static inline int zBase_(int finger_id) { return finger_id * 3; }
    static inline int qBase_(int finger_id) { return finger_id * 4; }

    // -------------------------------------------------------------------------
    // 20DoF / 15DoF conversion
    // -------------------------------------------------------------------------
    static void Expand15To20_(const std::vector<double>& z15, std::vector<double>& q20) {
        q20.assign(20, 0.0);
        if (z15.size() < 15) return;

        for (int f = 0; f < 5; ++f) {
            const int zb = zBase_(f);
            const int qb = qBase_(f);

            q20[qb + 0] = z15[zb + 0];
            q20[qb + 1] = z15[zb + 1];
            q20[qb + 2] = z15[zb + 2];
            q20[qb + 3] = z15[zb + 2]; // mimic: joint4 = joint3
        }
    }

    static void Expand15To20_(const Eigen::VectorXd& z15, std::vector<double>& q20) {
        q20.assign(20, 0.0);
        if (z15.size() < 15) return;

        for (int f = 0; f < 5; ++f) {
            const int zb = zBase_(f);
            const int qb = qBase_(f);

            q20[qb + 0] = z15(zb + 0);
            q20[qb + 1] = z15(zb + 1);
            q20[qb + 2] = z15(zb + 2);
            q20[qb + 3] = z15(zb + 2); // mimic
        }
    }

    // accepts 15DoF or 20DoF input
    static void NormalizeTo15DoF_(const std::vector<double>& in, std::vector<double>& z15) {
        z15.assign(15, 0.0);

        if (in.size() >= 20) {
            // compress 20 -> 15 (ignore joint4)
            for (int f = 0; f < 5; ++f) {
                const int qb = qBase_(f);
                const int zb = zBase_(f);
                z15[zb + 0] = in[qb + 0];
                z15[zb + 1] = in[qb + 1];
                z15[zb + 2] = in[qb + 2];
            }
            return;
        }

        // assume 15DoF (or partial)
        const int n = static_cast<int>(std::min<size_t>(in.size(), 15));
        for (int i = 0; i < n; ++i) z15[i] = in[i];
    }

    // -------------------------------------------------------------------------
    // Joint limits (URDF-like, independent joints only)
    // j_local: 0->joint1, 1->joint2, 2->joint3
    // finger_id: 0 thumb, 1 index, 2 middle, 3 ring, 4 baby
    // -------------------------------------------------------------------------
    static void GetJointLimitURDFLike_(int finger_id, int j_local, double& qmin, double& qmax) {
        // joint1 (all fingers)
        if (j_local == 0) {
            qmin = -0.5236;
            qmax =  0.5236;
            return;
        }

        // thumb joint2/joint3
        if (finger_id == 0) {
            if (j_local == 1) { qmin = 0.0; qmax = 1.0471975; return; } // thumb2
            if (j_local == 2) { qmin = 0.0; qmax = 1.0471975; return; } // thumb3
        }

        // index/middle/ring/baby
        if (j_local == 1) { qmin = 0.0; qmax = 1.5707963; return; } // joint2
        if (j_local == 2) { qmin = 0.0; qmax = 1.1344640; return; } // joint3
    }

    static void ClampFinger_(int finger_id, Eigen::VectorXd& z, const Options& opt) {
        const int zb = zBase_(finger_id);
        for (int j = 0; j < 3; ++j) {
            double qmin, qmax;
            if (opt.use_urdf_like_limits) {
                GetJointLimitURDFLike_(finger_id, j, qmin, qmax);
            } else {
                qmin = opt.q_min_global;
                qmax = opt.q_max_global;
            }
            z(zb + j) = std::min(std::max(z(zb + j), qmin), qmax);
        }
    }

    // -------------------------------------------------------------------------
    // FK helper: z15 -> 5 fingertips (hand base frame)
    // -------------------------------------------------------------------------
    std::array<Eigen::Vector3d,5> ComputeTipsFromZ15_(const Eigen::VectorXd& z15) const {
        std::array<Eigen::Vector3d,5> out;
        for (int i = 0; i < 5; ++i) out[i] = Eigen::Vector3d::Zero();

        std::vector<double> q20;
        Expand15To20_(z15, q20);

        const std::vector<Eigen::Vector3d> tips = fk_->computeFingertips(q20);
        for (int i = 0; i < 5 && i < static_cast<int>(tips.size()); ++i) out[i] = tips[i];
        return out;
    }

    // -------------------------------------------------------------------------
    // Single-finger independent IK (3DoF -> 3D position)
    // -------------------------------------------------------------------------
    bool SolveSingleFingerIK_(int finger_id,
                              Eigen::VectorXd& z,              // in/out (15)
                              const Eigen::VectorXd& z_ref,    // posture reference (15)
                              const Eigen::Vector3d& target_p, // target fingertip position (hand base frame)
                              const Options& opt) const
    {
        const double w = std::max(0.0, opt.weights[finger_id]);
        if (w <= 0.0) return true; // disabled by weight

        const int zb = zBase_(finger_id);
        const double eps = 1e-6;

        ClampFinger_(finger_id, z, opt);

        auto current_error = [&](const Eigen::VectorXd& zq) -> Eigen::Vector3d {
            const auto tips = ComputeTipsFromZ15_(zq);
            // e = current - target
            return (tips[finger_id] - target_p) * w;
        };

        Eigen::Vector3d e = current_error(z);
        const double err_init = e.norm();

        if (err_init < opt.tol_pos_m) return true;

        double lambda = std::min(std::max(opt.lambda, opt.lambda_min), opt.lambda_max);
        double alpha  = std::max(opt.alpha, opt.alpha_min);

        double best_err = err_init;
        Eigen::VectorXd z_best = z;
        bool improved = false;

        for (int it = 0; it < opt.max_iters; ++it) {
            const double err_norm = e.norm();
            if (err_norm < opt.tol_pos_m) {
                if (opt.verbose) {
                    std::printf("[HandIK] finger=%d converged in %d iter, err=%.6f\n", finger_id, it, err_norm);
                }
                return true;
            }

            // Numerical Jacobian J (3x3), only perturb this finger's 3 vars
            Eigen::Matrix3d J;
            J.setZero();

            for (int j = 0; j < 3; ++j) {
                Eigen::VectorXd zp = z;
                Eigen::VectorXd zm = z;

                zp(zb + j) += eps;
                zm(zb + j) -= eps;

                ClampFinger_(finger_id, zp, opt);
                ClampFinger_(finger_id, zm, opt);

                const auto tips_p = ComputeTipsFromZ15_(zp);
                const auto tips_m = ComputeTipsFromZ15_(zm);

                Eigen::Vector3d d = (tips_p[finger_id] - tips_m[finger_id]) / (2.0 * eps);
                J.col(j) = d * w;
            }

            // Local variables
            Eigen::Vector3d q_local, q_ref_local;
            for (int j = 0; j < 3; ++j) {
                q_local(j)     = z(zb + j);
                q_ref_local(j) = z_ref(zb + j);
            }

            // DLS + posture regularization
            // (J^T J + (lambda^2 + mu)I) dq = -J^T e - mu(q-q_ref)
            Eigen::Matrix3d A = J.transpose() * J
                              + (lambda * lambda + opt.mu_posture) * Eigen::Matrix3d::Identity();
            Eigen::Vector3d b = -(J.transpose() * e) - opt.mu_posture * (q_local - q_ref_local);

            Eigen::Vector3d dq = A.ldlt().solve(b);

            // dq clamp
            for (int j = 0; j < 3; ++j) {
                dq(j) = std::min(std::max(dq(j), -opt.max_step), opt.max_step);
            }

            // Backtracking line search (local)
            bool accepted = false;
            Eigen::VectorXd z_candidate = z;
            Eigen::Vector3d e_candidate = e;

            double alpha_try = alpha;
            for (int ls = 0; ls < 8; ++ls) {
                z_candidate = z;
                for (int j = 0; j < 3; ++j) z_candidate(zb + j) += alpha_try * dq(j);

                ClampFinger_(finger_id, z_candidate, opt);
                e_candidate = current_error(z_candidate);

                if (e_candidate.norm() <= e.norm()) {
                    accepted = true;
                    break;
                }
                alpha_try *= 0.5;
                if (alpha_try < opt.alpha_min) break;
            }

            if (accepted) {
                z = z_candidate;
                e = e_candidate;
                improved = true;

                if (e.norm() < best_err) {
                    best_err = e.norm();
                    z_best = z;
                }

                lambda = std::max(opt.lambda_min, lambda * 0.7);
                alpha  = std::min(1.0, std::max(alpha_try, opt.alpha_min) * 1.05);
            } else {
                lambda = std::min(opt.lambda_max, lambda * 2.0);
                alpha  = std::max(opt.alpha_min, alpha * 0.5);

                if (alpha <= opt.alpha_min + 1e-12) break;
            }
        }

        // keep best found if any improvement
        if (improved) z = z_best;

        if (opt.verbose) {
            std::printf("[HandIK] finger=%d done: init_err=%.6f, best_err=%.6f, improved=%d\n",
                        finger_id, err_init, best_err, improved ? 1 : 0);
        }

        // "부분 성공" 허용: 개선되었으면 true
        return (best_err < opt.tol_pos_m) || improved;
    }

private:
    bool ok_ = false;
    std::shared_ptr<HandForwardKinematics> fk_;
};

} // namespace dualarm_forcecon

#endif // DUALARM_FORCECON_KINEMATICS_HAND_INVERSE_KINEMATICS_HPP_