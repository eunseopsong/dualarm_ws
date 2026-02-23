#ifndef DUALARM_FORCECON_HAND_INVERSE_KINEMATICS_HPP
#define DUALARM_FORCECON_HAND_INVERSE_KINEMATICS_HPP

#include <Eigen/Dense>

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>

#include "dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp"

namespace dualarm_forcecon {

class HandInverseKinematics {
public:
    // ============================
    // Options (C++14-safe)
    // ============================
    struct Options {
        int    max_iters;      // iterations
        double tol_pos_m;      // stop if ||e|| < tol
        double lambda;         // DLS damping (start)
        double lambda_min;
        double lambda_max;

        double alpha;          // step size (start)
        double alpha_min;

        double mu_posture;     // posture regularization weight
        double max_step;       // per-joint clamp for dq [rad]
        bool   verbose;

        std::array<bool,5>   mask;     // which fingertip constraints are active
        std::array<double,5> weights;  // weight per fingertip

        double q_min;          // joint clamp
        double q_max;

        Options()
        : max_iters(60),
          tol_pos_m(1e-4),
          lambda(1e-2),
          lambda_min(1e-5),
          lambda_max(1.0),
          alpha(0.8),
          alpha_min(0.05),
          mu_posture(1e-3),
          max_step(0.15),
          verbose(false),
          mask{ {true,true,true,true,true} },
          weights{ {1.0,1.0,1.0,1.0,1.0} },
          q_min(-M_PI),
          q_max( M_PI)
        {}
    };

public:
    HandInverseKinematics(const std::string& urdf_path,
                          const std::string& hand_base_link,
                          const std::vector<std::string>& tip_keys)
    {
        fk_ = std::make_shared<HandForwardKinematics>(urdf_path, hand_base_link, tip_keys);
        ok_ = (fk_ != nullptr);
        tip_keys_ = tip_keys;
    }

    bool isOk() const { return ok_; }

    // ------------------------------------------------------------
    // Main solver (no default-arg here -> fixes your GCC error)
    // Targets are expressed in HAND_BASE frame (pos-only).
    // q_init: 20 (5 fingers x 4 joints)
    // tips_target: thumb/index/middle/ring/baby positions
    // ------------------------------------------------------------
    bool solveIKFingertips(const std::vector<double>& q_init,
                           const std::array<Eigen::Vector3d,5>& tips_target,
                           std::vector<double>& q_sol,
                           const Options& opt)
    {
        if (!ok_) return false;
        if (q_init.size() != 20) return false;

        // internal constants
        const double eps = 1e-6;          // numerical diff [rad]
        const int nq = 20;

        Eigen::VectorXd q(nq);
        for (int i = 0; i < nq; ++i) q[i] = q_init[i];

        Eigen::VectorXd q_ref(nq);
        for (int i = 0; i < nq; ++i) q_ref[i] = q_init[i];

        double lambda = std::min(std::max(opt.lambda, opt.lambda_min), opt.lambda_max);
        double alpha  = std::max(opt.alpha, opt.alpha_min);

        // active fingertips count
        int k = 0;
        for (int i = 0; i < 5; ++i) if (opt.mask[i]) k++;

        if (k == 0) {
            q_sol = q_init;
            return true;
        }

        auto clamp_q = [&](Eigen::VectorXd& qq) {
            for (int i = 0; i < nq; ++i) {
                qq[i] = std::min(std::max(qq[i], opt.q_min), opt.q_max);
            }
        };

        auto fk_fingertips = [&](const Eigen::VectorXd& qq) -> std::array<Eigen::Vector3d,5> {
            std::array<Eigen::Vector3d,5> out;
            // HandForwardKinematics signature in your codebase:
            //   computeFingertips(Eigen::VectorXd) -> vector<Eigen::Vector3d> of size 5
            const std::vector<Eigen::Vector3d> v = fk_->computeFingertips(qq);
            for (int i = 0; i < 5; ++i) {
                if (i < (int)v.size()) out[i] = v[i];
                else out[i] = Eigen::Vector3d::Zero();
            }
            return out;
        };

        auto build_error = [&](const std::array<Eigen::Vector3d,5>& tips_cur) -> Eigen::VectorXd {
            Eigen::VectorXd e(3*k);
            int row = 0;
            for (int i = 0; i < 5; ++i) {
                if (!opt.mask[i]) continue;
                const double w = opt.weights[i];
                const Eigen::Vector3d ei = (tips_cur[i] - tips_target[i]) * w;
                e.segment<3>(row) = ei;
                row += 3;
            }
            return e;
        };

        auto build_jacobian_numeric = [&](const Eigen::VectorXd& qq,
                                          const std::array<Eigen::Vector3d,5>& tips_center) -> Eigen::MatrixXd
        {
            Eigen::MatrixXd J(3*k, nq);
            J.setZero();

            for (int j = 0; j < nq; ++j) {
                Eigen::VectorXd q_plus = qq;
                Eigen::VectorXd q_minus = qq;

                q_plus[j]  += eps;
                q_minus[j] -= eps;

                clamp_q(q_plus);
                clamp_q(q_minus);

                const auto tips_p = fk_fingertips(q_plus);
                const auto tips_m = fk_fingertips(q_minus);

                int row = 0;
                for (int i = 0; i < 5; ++i) {
                    if (!opt.mask[i]) continue;
                    const double w = opt.weights[i];
                    const Eigen::Vector3d dp = (tips_p[i] - tips_m[i]) / (2.0 * eps);
                    J.block<3,1>(row, j) = dp * w;
                    row += 3;
                }
            }
            return J;
        };

        // iterate
        auto tips = fk_fingertips(q);
        Eigen::VectorXd e = build_error(tips);
        double err0 = e.norm();

        for (int it = 0; it < opt.max_iters; ++it) {
            if (e.norm() < opt.tol_pos_m) {
                q_sol.resize(nq);
                for (int i = 0; i < nq; ++i) q_sol[i] = q[i];
                return true;
            }

            const Eigen::MatrixXd J = build_jacobian_numeric(q, tips);

            // DLS + posture regularization:
            // dq = argmin ||J dq + e||^2 + lambda^2||dq||^2 + mu||q+dq-q_ref||^2
            // -> (J^T J + (lambda^2 + mu)I) dq = -J^T e - mu (q - q_ref)
            Eigen::MatrixXd A = J.transpose() * J;
            const double reg = lambda*lambda + opt.mu_posture;
            A += reg * Eigen::MatrixXd::Identity(nq, nq);

            Eigen::VectorXd b = -(J.transpose() * e) - opt.mu_posture * (q - q_ref);

            Eigen::VectorXd dq = A.ldlt().solve(b);

            // clamp dq
            for (int i = 0; i < nq; ++i) {
                dq[i] = std::min(std::max(dq[i], -opt.max_step), opt.max_step);
            }

            // trial update
            Eigen::VectorXd q_new = q + alpha * dq;
            clamp_q(q_new);

            const auto tips_new = fk_fingertips(q_new);
            const Eigen::VectorXd e_new = build_error(tips_new);
            const double err_new = e_new.norm();

            const bool accept = (err_new <= e.norm());

            if (accept) {
                q = q_new;
                tips = tips_new;
                e = e_new;

                // make it slightly more aggressive when improving
                lambda = std::max(opt.lambda_min, lambda * 0.7);
                alpha  = std::min(1.0, alpha * 1.05);
            } else {
                // be more conservative
                lambda = std::min(opt.lambda_max, lambda * 2.0);
                alpha  = std::max(opt.alpha_min, alpha * 0.5);

                if (alpha <= opt.alpha_min + 1e-12) {
                    break;
                }
            }

            if (opt.verbose) {
                // keep prints minimal (header-only)
                // printf not included intentionally to avoid pulling <cstdio> in headers
            }
        }

        // return best effort (even if not converged)
        q_sol.resize(nq);
        for (int i = 0; i < nq; ++i) q_sol[i] = q[i];

        // "success" if improved enough
        const double err_final = e.norm();
        return (err_final < err0);
    }

    // Convenience overload (safe): calls default Options()
    bool solveIKFingertips(const std::vector<double>& q_init,
                           const std::array<Eigen::Vector3d,5>& tips_target,
                           std::vector<double>& q_sol)
    {
        Options opt;
        return solveIKFingertips(q_init, tips_target, q_sol, opt);
    }

private:
    bool ok_{false};
    std::vector<std::string> tip_keys_;
    std::shared_ptr<HandForwardKinematics> fk_;
};

} // namespace dualarm_forcecon

#endif // DUALARM_FORCECON_HAND_INVERSE_KINEMATICS_HPP