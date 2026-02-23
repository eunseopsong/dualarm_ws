// ============================================================================
// hand_inverse_kinematics.hpp  (v11)
// - Pinocchio-based iterative IK (pos-only)
// - Solves 5 fingertip positions simultaneously (thumb, index, middle, ring, baby)
// - Base frame: LEFT/RIGHT hand base reference (forced to left_joint_6 / right_joint_6)
// - NOTE: Use two instances for 10 fingers (left + right).
// ============================================================================

#pragma once

#include <cstdio>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>
#include <limits>
#include <cmath>

#include <Eigen/Dense>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace dualarm_forcecon {

class HandInverseKinematics {
public:
    // ============================================================
    // Options
    // ============================================================
    struct Options {
        int    max_iters   = 60;
        double tol_pos_m   = 1e-4;   // stop if ||e||_2 < tol
        double lambda      = 1e-2;   // DLS damping (start)
        double lambda_min  = 1e-5;
        double lambda_max  = 1e+0;
        double alpha       = 0.8;    // step size (start)
        double alpha_min   = 0.05;
        double mu_posture  = 1e-3;   // posture regularization weight
        double max_step    = 0.15;   // per-joint clamp for dq [rad]
        bool   verbose     = false;

        // tip mask/weights: order = thumb,index,middle,ring,baby
        std::array<bool,5>   mask    {true,true,true,true,true};
        std::array<double,5> weights {1.0, 1.0, 1.0, 1.0, 1.0};

        // joint limits (default [-pi, pi])
        double q_min = -M_PI;
        double q_max =  M_PI;
    };

public:
    HandInverseKinematics() = default;

    // ✅ existing ctor-style compatibility (mirrors HandForwardKinematics)
    HandInverseKinematics(const std::string& urdf_path,
                          const std::string& base_name_unused,
                          const std::vector<std::string>& tips_unused)
    {
        (void)tips_unused;
        std::string side = InferSideFromBaseName_(base_name_unused);
        Init(urdf_path, side);
    }

    // existing Init signature compatibility
    bool Init(const std::string& urdf_path,
              const std::string& base_name_unused,
              const std::vector<std::string>& tips_unused)
    {
        (void)tips_unused;
        std::string side = InferSideFromBaseName_(base_name_unused);
        return Init(urdf_path, side);
    }

    // core Init: side="left" or "right"
    bool Init(const std::string& urdf_path, const std::string& side) {
        side_ = side;
        ok_   = false;

        try {
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = pinocchio::Data(model_);
        } catch (const std::exception& e) {
            std::printf("[HandIK Error] buildModel failed: %s\n", e.what());
            return false;
        }

        // ---- base reference forced: left_joint_6 / right_joint_6 ----
        if      (side_ == "left")  base_joint_name_ = "left_joint_6";
        else if (side_ == "right") base_joint_name_ = "right_joint_6";
        else {
            std::printf("[HandIK Error] side must be 'left' or 'right' (got '%s')\n", side_.c_str());
            return false;
        }

        ResolveBaseReference_();

        // ---- tip frame forced: link4 5 tips ----
        tip_names_.clear();
        tip_names_.reserve(5);

        if (side_ == "left") {
            tip_names_ = {
                "left_link4_thumb",
                "left_link4_index",
                "left_link4_middle",
                "left_link4_ring",
                "left_link4_baby"
            };
        } else {
            tip_names_ = {
                "right_link4_thumb",
                "right_link4_index",
                "right_link4_middle",
                "right_link4_ring",
                "right_link4_baby"
            };
        }

        tip_fids_.assign(5, InvalidFrame_());
        int tips_found = 0;
        for (size_t i = 0; i < tip_names_.size(); ++i) {
            const auto& nm = tip_names_[i];
            if (model_.existFrame(nm)) {
                tip_fids_[i] = model_.getFrameId(nm);
                tips_found++;
            }
        }

        // ---- hand joint mapping(20dof) 구성 ----
        BuildHandJointMapping_();

        // ---- Print summary ----
        std::printf("[HandIK Info] Pinocchio OK?=1 base=%s side=%s nv=%d nq=%d nframes=%zu\n",
                    base_ref_name_.c_str(), side_.c_str(), model_.nv, model_.nq, model_.frames.size());

        std::printf("[HandIK Info] Resolved tips (key -> frame):\n");
        for (size_t i = 0; i < tip_names_.size(); ++i) {
            if (tip_fids_[i] < model_.frames.size()) {
                std::printf("  key='%s' -> frame='%s' (fid=%u)\n",
                            tip_names_[i].c_str(), tip_names_[i].c_str(), (unsigned)tip_fids_[i]);
            } else {
                std::printf("  key='%s' -> NOT FOUND\n", tip_names_[i].c_str());
            }
        }

        std::printf("[HandIK Info] Hand joint mapping (20 dof) idx->jointName (qidx,vidx):\n");
        for (size_t i = 0; i < hand_joint_names_.size(); ++i) {
            std::printf("  [%zu] : %s (qidx=%d vidx=%d)\n",
                        i, hand_joint_names_[i].c_str(), hand_qidx_[i], hand_vidx_[i]);
        }

        ok_ = (tips_found == 5);
        if (!ok_) {
            std::printf("[HandIK Warn] tip frames not fully found (%d/5). Check URDF frame names.\n", tips_found);
        }
        return ok_;
    }

    bool ok() const { return ok_; }
    const std::string& side() const { return side_; }
    const std::string& base_ref_name() const { return base_ref_name_; }

    // ============================================================
    // Public solve APIs
    // - target_tip_pos_base: 5 tips in BASE frame (left_joint_6/right_joint_6)
    // - q_init20: 20 DOF in order (thumb1-4, index1-4, middle1-4, ring1-4, baby1-4)
    // ============================================================
    bool solveIKFingertips(const std::vector<double>& q_init20,
                           const std::array<Eigen::Vector3d,5>& target_tip_pos_base,
                           std::vector<double>& q_sol20,
                           const Options& opt = Options())
    {
        Eigen::VectorXd qi(20);
        qi.setZero();
        for (int i = 0; i < 20 && i < (int)q_init20.size(); ++i) qi[i] = q_init20[i];

        Eigen::VectorXd qs(20);
        bool ok = solveIKFingertips(qi, target_tip_pos_base, qs, opt);

        q_sol20.resize(20);
        for (int i = 0; i < 20; ++i) q_sol20[i] = qs[i];
        return ok;
    }

    bool solveIKFingertips(const Eigen::VectorXd& q_init20,
                           const std::array<Eigen::Vector3d,5>& target_tip_pos_base,
                           Eigen::VectorXd& q_sol20,
                           const Options& opt = Options())
    {
        if (!ok_) return false;
        if (q_init20.size() < 20) return false;

        // reference posture (stabilizer): default = initial
        const Eigen::VectorXd q_ref = q_init20.head(20);

        // working state
        Eigen::VectorXd q20 = q_init20.head(20);
        clampJointLimits_(q20, opt.q_min, opt.q_max);

        double lambda = clamp_(opt.lambda, opt.lambda_min, opt.lambda_max);
        double alpha  = clamp_(opt.alpha,  opt.alpha_min,  1.0);

        // pre-count active tips
        int active_tips = 0;
        for (int i = 0; i < 5; ++i) if (opt.mask[i]) active_tips++;
        if (active_tips == 0) {
            q_sol20 = q20;
            return true;
        }

        // helper buffers
        Eigen::VectorXd e;     // stacked error (3*active_tips)
        Eigen::MatrixXd J;     // stacked Jacobian (3*active_tips x 20)

        double prev_cost = std::numeric_limits<double>::infinity();
        bool converged = false;

        for (int iter = 0; iter < opt.max_iters; ++iter) {

            // build e and J at current q20
            if (!buildErrorAndJacobian_(q20, target_tip_pos_base, opt, e, J)) {
                if (opt.verbose) std::printf("[HandIK] buildErrorAndJacobian failed at iter=%d\n", iter);
                break;
            }

            const double cost = 0.5 * e.squaredNorm();
            const double norm_e = std::sqrt(2.0 * cost);

            if (opt.verbose) {
                std::printf("[HandIK] iter=%d | ||e||=%.6g cost=%.6g lambda=%.3g alpha=%.3g\n",
                            iter, norm_e, cost, lambda, alpha);
            }

            if (norm_e < opt.tol_pos_m) {
                converged = true;
                break;
            }

            // Weighted DLS (row scaling by sqrt(w_tip))
            // Here: e and J are already weighted in buildErrorAndJacobian_ (row-scaled).
            // Solve: (J^T J + (lambda^2 + mu)I) dq = -(J^T e + mu(q - q_ref))
            Eigen::MatrixXd A = J.transpose() * J;
            const double reg = lambda * lambda + std::max(0.0, opt.mu_posture);
            A.diagonal().array() += reg;

            Eigen::VectorXd b = J.transpose() * e;
            if (opt.mu_posture > 0.0) {
                b += opt.mu_posture * (q20 - q_ref);
            }

            // dq
            Eigen::VectorXd dq = -A.ldlt().solve(b);
            clampVector_(dq, opt.max_step);

            // candidate update
            bool accepted = false;
            Eigen::VectorXd q20_best = q20;
            double best_cost = cost;

            // simple backtracking with alpha shrink
            double alpha_try = alpha;
            for (int ls = 0; ls < 6; ++ls) {
                Eigen::VectorXd q_try = q20 + alpha_try * dq;
                clampJointLimits_(q_try, opt.q_min, opt.q_max);

                Eigen::VectorXd e_try;
                Eigen::MatrixXd J_try;
                if (!buildErrorAndJacobian_(q_try, target_tip_pos_base, opt, e_try, J_try)) {
                    alpha_try *= 0.5;
                    continue;
                }
                const double cost_try = 0.5 * e_try.squaredNorm();

                if (cost_try <= best_cost) {
                    best_cost = cost_try;
                    q20_best = q_try;
                    accepted = true;
                    break;
                }
                alpha_try *= 0.5;
                if (alpha_try < opt.alpha_min) break;
            }

            if (accepted) {
                q20 = q20_best;

                // if improving well, reduce damping slightly
                if (best_cost < cost) {
                    lambda = clamp_(lambda * 0.8, opt.lambda_min, opt.lambda_max);
                }
                alpha = clamp_(std::max(alpha, alpha_try), opt.alpha_min, 1.0);

                // optional early stop if stagnating
                if (std::isfinite(prev_cost) && std::fabs(prev_cost - best_cost) < 1e-12) {
                    // stagnation
                    if (opt.verbose) std::printf("[HandIK] stagnation detected.\n");
                }
                prev_cost = best_cost;
            } else {
                // reject: increase damping, reduce alpha
                lambda = clamp_(lambda * 2.0, opt.lambda_min, opt.lambda_max);
                alpha  = clamp_(alpha  * 0.7, opt.alpha_min, 1.0);

                if (lambda >= opt.lambda_max * 0.999) {
                    if (opt.verbose) std::printf("[HandIK] lambda hit max -> stop.\n");
                    break;
                }
            }
        }

        q_sol20 = q20;
        return converged;
    }

private:
    // ============================================================
    // Internal utilities
    // ============================================================
    static pinocchio::FrameIndex InvalidFrame_() {
        return std::numeric_limits<pinocchio::FrameIndex>::max();
    }

    static double clamp_(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    static void clampVector_(Eigen::VectorXd& v, double max_abs) {
        if (max_abs <= 0.0) return;
        for (int i = 0; i < v.size(); ++i) {
            if (v[i] >  max_abs) v[i] =  max_abs;
            if (v[i] < -max_abs) v[i] = -max_abs;
        }
    }

    static void clampJointLimits_(Eigen::VectorXd& q20, double qmin, double qmax) {
        for (int i = 0; i < q20.size(); ++i) {
            if (q20[i] < qmin) q20[i] = qmin;
            if (q20[i] > qmax) q20[i] = qmax;
        }
    }

    std::string InferSideFromBaseName_(const std::string& base_name) const {
        // 기존 호출이 "left_hand_base_link"/"right_hand_base_link"라서 여기서 side 추론
        if (base_name.find("left")  != std::string::npos)  return "left";
        if (base_name.find("right") != std::string::npos) return "right";
        return "left";
    }

    void ResolveBaseReference_() {
        base_is_joint_ = false;
        base_joint_id_ = 0;
        base_frame_id_ = InvalidFrame_();
        base_ref_name_.clear();

        // 1) joint 우선
        const pinocchio::JointIndex jid = model_.getJointId(base_joint_name_);
        if (jid > 0) {
            base_is_joint_ = true;
            base_joint_id_ = jid;
            base_ref_name_ = base_joint_name_;
            return;
        }

        // 2) fallback: link6 frame
        const std::string link6 = (side_ == "left") ? "left_link_6" : "right_link_6";
        if (model_.existFrame(link6)) {
            base_is_joint_ = false;
            base_frame_id_ = model_.getFrameId(link6);
            base_ref_name_ = link6;
            std::printf("[HandIK Warn] base joint '%s' not found -> fallback frame '%s'\n",
                        base_joint_name_.c_str(), link6.c_str());
            return;
        }

        // 3) fail
        base_ref_name_ = base_joint_name_;
        std::printf("[HandIK Error] base reference not found: joint '%s' nor frame '%s'\n",
                    base_joint_name_.c_str(),
                    ((side_ == "left") ? "left_link_6" : "right_link_6"));
    }

    void BuildHandJointMapping_() {
        hand_joint_names_.clear();
        hand_joint_names_.reserve(20);

        auto push4 = [&](const std::string& prefix) {
            hand_joint_names_.push_back(prefix + "1");
            hand_joint_names_.push_back(prefix + "2");
            hand_joint_names_.push_back(prefix + "3");
            hand_joint_names_.push_back(prefix + "4");
        };

        if (side_ == "left") {
            push4("left_thumb_joint");
            push4("left_index_joint");
            push4("left_middle_joint");
            push4("left_ring_joint");
            push4("left_baby_joint");
        } else {
            push4("right_thumb_joint");
            push4("right_index_joint");
            push4("right_middle_joint");
            push4("right_ring_joint");
            push4("right_baby_joint");
        }

        hand_qidx_.assign(20, -1);
        hand_vidx_.assign(20, -1);

        for (int i = 0; i < 20; ++i) {
            const std::string& jn = hand_joint_names_[i];
            const pinocchio::JointIndex jid = model_.getJointId(jn);
            if (jid == 0) {
                hand_qidx_[i] = -1;
                hand_vidx_[i] = -1;
                continue;
            }
            hand_qidx_[i] = model_.joints[jid].idx_q(); // nq index
            hand_vidx_[i] = model_.joints[jid].idx_v(); // nv index
        }
    }

    pinocchio::SE3 GetWorldToBase_() const {
        if (base_is_joint_) return data_.oMi[base_joint_id_];
        else                return data_.oMf[base_frame_id_];
    }

    // set full model q from q20 (only hand joints)
    Eigen::VectorXd packFullQ_(const Eigen::VectorXd& q20) const {
        Eigen::VectorXd q = pinocchio::neutral(model_);
        const int n_use = std::min<int>(20, (int)q20.size());
        for (int i = 0; i < n_use; ++i) {
            const int qidx = hand_qidx_[i];
            if (qidx >= 0 && qidx < q.size()) q[qidx] = q20[i];
        }
        return q;
    }

    // build stacked e (weighted) and J (weighted)
    // - target_tip_pos_base: base frame targets
    // - e: [3*active] (weighted)
    // - J: [3*active x 20] (weighted)
    bool buildErrorAndJacobian_(const Eigen::VectorXd& q20,
                               const std::array<Eigen::Vector3d,5>& target_tip_pos_base,
                               const Options& opt,
                               Eigen::VectorXd& e_out,
                               Eigen::MatrixXd& J_out)
    {
        if (!ok_) return false;

        Eigen::VectorXd q = packFullQ_(q20);

        // Kinematics + Jacobians
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);
        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        // base transform (world -> base)
        const pinocchio::SE3 oMb = GetWorldToBase_();
        const Eigen::Matrix3d Rwb = oMb.rotation();
        const Eigen::Matrix3d Rbw = Rwb.transpose();
        const Eigen::Vector3d p_b_w = oMb.translation();

        // count active tips
        int active = 0;
        for (int i = 0; i < 5; ++i) if (opt.mask[i]) active++;
        if (active <= 0) {
            e_out.resize(0);
            J_out.resize(0,20);
            return true;
        }

        e_out.setZero(3 * active);
        J_out.setZero(3 * active, 20);

        int row = 0;
        Eigen::Matrix<double,6,Eigen::Dynamic> J6(6, model_.nv);

        for (int tip = 0; tip < 5; ++tip) {
            if (!opt.mask[tip]) continue;
            if (tip_fids_[tip] >= model_.frames.size()) {
                // missing frame -> fail safe
                return false;
            }

            // current tip position in base frame
            const pinocchio::SE3& oMt = data_.oMf[tip_fids_[tip]];
            const Eigen::Vector3d p_t_w = oMt.translation();
            const Eigen::Vector3d p_bt_b = Rbw * (p_t_w - p_b_w); // base coords

            // error (pos-only): p(q) - p*
            Eigen::Vector3d err = p_bt_b - target_tip_pos_base[tip];

            // frame Jacobian in WORLD (6 x nv)
            // We only need the linear part and only hand columns.
            pinocchio::getFrameJacobian(model_, data_, tip_fids_[tip],
                                        pinocchio::ReferenceFrame::WORLD, J6);

            // Jpos_base(:,j) = Rbw * Jpos_world(:, vidx_j)
            Eigen::Matrix<double,3,20> Jpos_b;
            Jpos_b.setZero();

            for (int j = 0; j < 20; ++j) {
                const int vidx = hand_vidx_[j];
                if (vidx < 0 || vidx >= model_.nv) continue;
                const Eigen::Vector3d col_w = J6.block<3,1>(0, vidx); // linear part
                Jpos_b.col(j) = Rbw * col_w;
            }

            // apply weight (row scaling by sqrt(w))
            const double w = std::max(0.0, opt.weights[tip]);
            const double s = std::sqrt(w);

            e_out.segment<3>(row) = s * err;
            J_out.block(row, 0, 3, 20) = s * Jpos_b;

            row += 3;
        }

        return true;
    }

private:
    pinocchio::Model model_;
    pinocchio::Data  data_{model_};

    bool ok_ = false;
    std::string side_;

    // base reference (forced to *_joint_6, fallback to *_link_6 frame)
    std::string base_joint_name_;
    std::string base_ref_name_;
    bool base_is_joint_ = false;
    pinocchio::JointIndex base_joint_id_ = 0;
    pinocchio::FrameIndex base_frame_id_ = InvalidFrame_();

    // tips
    std::vector<std::string> tip_names_;            // size 5
    std::vector<pinocchio::FrameIndex> tip_fids_;   // size 5

    // hand joint mapping (20 dof)
    std::vector<std::string> hand_joint_names_; // size 20
    std::vector<int> hand_qidx_;                // size 20 (q index)
    std::vector<int> hand_vidx_;                // size 20 (v index)
};

} // namespace dualarm_forcecon