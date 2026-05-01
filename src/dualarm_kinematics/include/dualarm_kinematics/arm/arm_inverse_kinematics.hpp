#ifndef DUALARM_FORCECON_KINEMATICS_ARM_INVERSE_KINEMATICS_HPP_
#define DUALARM_FORCECON_KINEMATICS_ARM_INVERSE_KINEMATICS_HPP_

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>

#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <cmath>
#include <algorithm>
#include <limits>

#include <Eigen/Dense>

class ArmInverseKinematics {
public:
    ArmInverseKinematics(const std::string& urdf_path,
                         const std::string& base_link,
                         const std::string& tip_link)
    : ok_(false),
      num_joints_(0),
      world_T_base_(Eigen::Isometry3d::Identity()),
      auto_frame_inference_enabled_(false)   // ★ 패치: 기본은 strict mode
    {
        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(urdf_path, tree)) {
            std::cerr << "[IK Error] Failed to construct KDL tree from file." << std::endl;
            return;
        }
        if (!tree.getChain(base_link, tip_link, chain_)) {
            std::cerr << "[IK Error] Failed chain: " << base_link << " -> " << tip_link << std::endl;
            return;
        }

        num_joints_ = chain_.getNrOfJoints();
        fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
        vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);

        // Joint limits (기본 [-pi, pi], 필요하면 URDF limit 파싱으로 확장)
        KDL::JntArray q_min(num_joints_), q_max(num_joints_);
        for (unsigned int i = 0; i < num_joints_; ++i) {
            q_min(i) = -M_PI;
            q_max(i) =  M_PI;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
            chain_, q_min, q_max, *fk_solver_, *vel_solver_, 100, 1e-6
        );

        ok_ = true;
        std::cout << "[IK Info] ArmInverseKinematics OK. auto_frame_inference=OFF (strict)" << std::endl;
    }

    bool isOk() const { return ok_; }

    void setWorldBaseTransformXYZEulerDeg(const std::array<double,3>& xyz,
                                          const std::array<double,3>& euler_xyz_deg)
    {
        world_T_base_ = Eigen::Isometry3d::Identity();
        world_T_base_.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);

        Eigen::Vector3d e = Eigen::Vector3d(euler_xyz_deg[0], euler_xyz_deg[1], euler_xyz_deg[2]) * M_PI / 180.0;
        Eigen::AngleAxisd Rx(e.x(), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(e.y(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(e.z(), Eigen::Vector3d::UnitZ());

        // 프로젝트 convention 유지
        Eigen::Quaterniond q = Rx * Ry * Rz;
        q.normalize();
        world_T_base_.linear() = q.toRotationMatrix();
    }

    const Eigen::Isometry3d& world_T_base() const { return world_T_base_; }

    // 필요할 때만 사용 (기본 OFF 유지 권장)
    void setAutoFrameInferenceEnabled(bool en) { auto_frame_inference_enabled_ = en; }
    bool autoFrameInferenceEnabled() const { return auto_frame_inference_enabled_; }

    // ==========================
    // v6 시그니처(7 args)
    // ==========================
    bool solveIK(const std::vector<double>& current_q,
                 const std::array<double,3>& target_xyz,
                 const std::array<double,3>& target_euler,
                 const std::string& targets_frame,   // "base" or "world"
                 const std::string& euler_conv,      // "rpy" or "xyz" or "zyx"
                 const std::string& angle_unit,      // "rad" or "deg" or "auto"
                 std::vector<double>& result_q) const
    {
        if (!ok_ || !ik_solver_) return false;
        if (current_q.size() < num_joints_) return false;

        auto frame_s = toLower_(targets_frame);
        auto conv_s  = toLower_(euler_conv);
        auto unit_s  = toLower_(angle_unit);

        if (frame_s.empty()) frame_s = "base";

        // --------------------------
        // angle unit 처리
        // --------------------------
        auto toRad = [&](double a)->double{
            if (unit_s == "rad") return a;
            if (unit_s == "deg") return a * M_PI / 180.0;
            // auto: 값이 크면 deg로 판단
            if (std::fabs(a) > 3.5) return a * M_PI / 180.0;
            return a;
        };

        const double a0 = toRad(target_euler[0]);
        const double a1 = toRad(target_euler[1]);
        const double a2 = toRad(target_euler[2]);

        // --------------------------
        // Rotation 생성
        // 주의: 프로젝트 기존 관례 유지
        // - "zyx"/"rzyx": Rz*Ry*Rx
        // - 그 외 ("xyz","rpy"): Rx*Ry*Rz
        // --------------------------
        Eigen::AngleAxisd Rx(a0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(a1, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(a2, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q;
        if (conv_s == "zyx" || conv_s == "rzyx") q = Rz * Ry * Rx;
        else                                     q = Rx * Ry * Rz;  // default ("xyz","rpy")
        q.normalize();

        // 입력 target pose (입력 frame 기준)
        Eigen::Isometry3d E_target_input = Eigen::Isometry3d::Identity();
        E_target_input.translation() = Eigen::Vector3d(target_xyz[0], target_xyz[1], target_xyz[2]);
        E_target_input.linear() = q.toRotationMatrix();

        // --------------------------
        // frame 해석 (strict 기본)
        // --------------------------
        bool target_is_world = false;

        if (frame_s == "base" || frame_s == "robot_base" || frame_s == "local") {
            target_is_world = false;
        } else if (frame_s == "world" || frame_s == "global") {
            target_is_world = true;
        } else {
            // 알 수 없는 frame 문자열은 base로 취급 (기존 동작 최대한 유지)
            std::cerr << "[IK Warn] Unknown targets_frame='" << targets_frame
                      << "'. Treat as 'base'." << std::endl;
            target_is_world = false;
        }

        // --------------------------
        // (선택) auto-frame inference
        // 기본 OFF. 정말 필요할 때만 켜기.
        // --------------------------
        if (auto_frame_inference_enabled_ && !target_is_world) {
            const std::string inferred = inferLikelyFrameFromCurrentQ_(current_q, target_xyz);
            if (inferred == "world") {
                target_is_world = true;
                std::cout << "[IK Info] solveIK auto-frame inference: base -> world" << std::endl;
            }
        }

        // --------------------------
        // target를 base frame으로 변환
        // --------------------------
        Eigen::Isometry3d E_base_target = E_target_input;
        if (target_is_world) {
            const Eigen::Isometry3d E_base_world = world_T_base_.inverse();
            E_base_target = E_base_world * E_target_input;
        }

        // Eigen -> KDL::Frame
        KDL::Frame target_frame;
        target_frame.p = KDL::Vector(E_base_target.translation().x(),
                                     E_base_target.translation().y(),
                                     E_base_target.translation().z());

        Eigen::Quaterniond qb(E_base_target.linear());
        qb.normalize();
        target_frame.M = KDL::Rotation::Quaternion(qb.x(), qb.y(), qb.z(), qb.w());

        KDL::JntArray q_init(num_joints_);
        for (unsigned int i = 0; i < num_joints_; ++i) q_init(i) = current_q[i];

        KDL::JntArray q_out(num_joints_);
        const int ret = ik_solver_->CartToJnt(q_init, target_frame, q_out);
        if (ret < 0) return false;

        result_q.resize(num_joints_);
        for (unsigned int i = 0; i < num_joints_; ++i) result_q[i] = q_out(i);
        return true;
    }


    // ==========================
    // Position-only DLS IK fallback
    // - Uses only translational error (3xN numerical Jacobian)
    // - Useful for redundant/mobile-manipulator arms where strict full-pose KDL IK
    //   can fail even for small Cartesian deltas.
    // ==========================
    bool solveIKPositionOnlyDLS(const std::vector<double>& current_q,
                                const std::array<double,3>& target_xyz,
                                const std::string& targets_frame,
                                std::vector<double>& result_q,
                                int max_iters = 100,
                                double pos_tol_m = 5e-4,
                                double lambda = 3e-2,
                                double alpha = 0.7,
                                double max_joint_step_rad = 3e-2,
                                double numerical_eps = 1e-5) const
    {
        if (!ok_ || !fk_solver_) return false;
        if (current_q.size() < num_joints_) return false;
        if (num_joints_ == 0) return false;

        auto frame_s = toLower_(targets_frame);
        if (frame_s.empty()) frame_s = "base";

        Eigen::Vector3d p_target(target_xyz[0], target_xyz[1], target_xyz[2]);
        if (!std::isfinite(p_target.x()) || !std::isfinite(p_target.y()) || !std::isfinite(p_target.z())) {
            return false;
        }

        if (frame_s == "world" || frame_s == "global") {
            p_target = world_T_base_.inverse() * p_target;
        } else if (!(frame_s == "base" || frame_s == "robot_base" || frame_s == "local")) {
            std::cerr << "[IK Warn] Unknown targets_frame='" << targets_frame
                      << "'. Position-only DLS treats it as 'base'." << std::endl;
        }

        if (max_iters <= 0) max_iters = 100;
        if (pos_tol_m <= 0.0) pos_tol_m = 5e-4;
        if (lambda <= 0.0) lambda = 3e-2;
        if (alpha <= 0.0) alpha = 0.7;
        if (max_joint_step_rad <= 0.0) max_joint_step_rad = 3e-2;
        if (numerical_eps <= 0.0) numerical_eps = 1e-5;

        std::vector<double> q(num_joints_, 0.0);
        for (unsigned int i = 0; i < num_joints_; ++i) {
            q[i] = current_q[i];
            if (!std::isfinite(q[i])) q[i] = 0.0;
        }

        auto fk_pos = [&](const std::vector<double>& q_in, Eigen::Vector3d& p_out)->bool {
            if (q_in.size() < num_joints_) return false;
            KDL::JntArray q_kdl(num_joints_);
            for (unsigned int i = 0; i < num_joints_; ++i) q_kdl(i) = q_in[i];
            KDL::Frame T;
            if (fk_solver_->JntToCart(q_kdl, T) < 0) return false;
            p_out = Eigen::Vector3d(T.p.x(), T.p.y(), T.p.z());
            return std::isfinite(p_out.x()) && std::isfinite(p_out.y()) && std::isfinite(p_out.z());
        };

        Eigen::Vector3d p_cur = Eigen::Vector3d::Zero();
        if (!fk_pos(q, p_cur)) return false;

        double best_err = (p_target - p_cur).norm();
        std::vector<double> best_q = q;

        for (int iter = 0; iter < max_iters; ++iter) {
            if (!fk_pos(q, p_cur)) return false;

            const Eigen::Vector3d e = p_target - p_cur;
            const double err = e.norm();

            if (err < best_err) {
                best_err = err;
                best_q = q;
            }
            if (err <= pos_tol_m) {
                result_q = q;
                return true;
            }

            Eigen::MatrixXd J(3, num_joints_);
            J.setZero();

            for (unsigned int j = 0; j < num_joints_; ++j) {
                std::vector<double> q_plus = q;
                std::vector<double> q_minus = q;
                q_plus[j] += numerical_eps;
                q_minus[j] -= numerical_eps;

                Eigen::Vector3d p_plus, p_minus;
                if (!fk_pos(q_plus, p_plus) || !fk_pos(q_minus, p_minus)) {
                    return false;
                }
                J.col(j) = (p_plus - p_minus) / (2.0 * numerical_eps);
            }

            const Eigen::Matrix3d A = J * J.transpose()
                                      + (lambda * lambda) * Eigen::Matrix3d::Identity();
            Eigen::VectorXd dq = J.transpose() * A.ldlt().solve(e);

            if (dq.size() != static_cast<int>(num_joints_)) return false;
            for (int i = 0; i < dq.size(); ++i) {
                if (!std::isfinite(dq(i))) return false;
            }

            const double max_abs_dq = dq.cwiseAbs().maxCoeff();
            if (max_abs_dq > max_joint_step_rad) {
                dq *= (max_joint_step_rad / max_abs_dq);
            }

            // Keep the update conservative around the best known state.
            q = best_q;
            for (unsigned int i = 0; i < num_joints_; ++i) {
                q[i] += alpha * dq(static_cast<int>(i));
                q[i] = std::clamp(q[i], -M_PI, M_PI);
            }
        }

        // Accept a near solution only if it is reasonably close. This prevents
        // sending arbitrary joint jumps when the target is unreachable.
        if (best_err <= std::max(5.0 * pos_tol_m, 0.003)) {
            result_q = best_q;
            return true;
        }

        result_q.clear();
        return false;
    }


    // ==========================
    // Pose DLS IK fallback
    // - Uses translational + rotational error (6xN numerical Jacobian)
    // - Intended for RBY1 7-DOF arm delta Cartesian control.
    // - When the delta command has only xyz components, pass the latched home
    //   orientation as target_xyzw to prevent free null-space orientation drift.
    // ==========================
    bool solveIKPoseDLS(const std::vector<double>& current_q,
                        const std::array<double,3>& target_xyz,
                        const std::array<double,4>& target_xyzw,  // x,y,z,w
                        const std::string& targets_frame,
                        std::vector<double>& result_q,
                        int max_iters = 140,
                        double pos_tol_m = 5e-4,
                        double rot_tol_rad = 2e-3,
                        double pos_weight = 1.0,
                        double rot_weight = 0.75,
                        double lambda = 5e-2,
                        double alpha = 0.55,
                        double max_joint_step_rad = 2e-2,
                        double numerical_eps = 1e-5) const
    {
        if (!ok_ || !fk_solver_) return false;
        if (current_q.size() < num_joints_) return false;
        if (num_joints_ == 0) return false;

        auto frame_s = toLower_(targets_frame);
        if (frame_s.empty()) frame_s = "base";

        Eigen::Vector3d p_target(target_xyz[0], target_xyz[1], target_xyz[2]);
        Eigen::Quaterniond q_target_input(
            target_xyzw[3], target_xyzw[0], target_xyzw[1], target_xyzw[2]);

        if (!std::isfinite(p_target.x()) || !std::isfinite(p_target.y()) || !std::isfinite(p_target.z())) {
            return false;
        }
        if (!std::isfinite(q_target_input.w()) || !std::isfinite(q_target_input.x()) ||
            !std::isfinite(q_target_input.y()) || !std::isfinite(q_target_input.z())) {
            return false;
        }
        if (q_target_input.norm() < 1e-12) return false;
        q_target_input.normalize();

        Eigen::Isometry3d E_target_input = Eigen::Isometry3d::Identity();
        E_target_input.translation() = p_target;
        E_target_input.linear() = q_target_input.toRotationMatrix();

        Eigen::Isometry3d E_base_target = E_target_input;
        if (frame_s == "world" || frame_s == "global") {
            E_base_target = world_T_base_.inverse() * E_target_input;
        } else if (!(frame_s == "base" || frame_s == "robot_base" || frame_s == "local")) {
            std::cerr << "[IK Warn] Unknown targets_frame='" << targets_frame
                      << "'. Pose DLS treats it as 'base'." << std::endl;
        }

        p_target = E_base_target.translation();
        const Eigen::Matrix3d R_target = E_base_target.linear();

        if (max_iters <= 0) max_iters = 140;
        if (pos_tol_m <= 0.0) pos_tol_m = 5e-4;
        if (rot_tol_rad <= 0.0) rot_tol_rad = 2e-3;
        if (pos_weight <= 0.0) pos_weight = 1.0;
        if (rot_weight <= 0.0) rot_weight = 0.75;
        if (lambda <= 0.0) lambda = 5e-2;
        if (alpha <= 0.0) alpha = 0.55;
        if (max_joint_step_rad <= 0.0) max_joint_step_rad = 2e-2;
        if (numerical_eps <= 0.0) numerical_eps = 1e-5;

        auto so3Log = [](const Eigen::Matrix3d& R)->Eigen::Vector3d {
            Eigen::Matrix3d R_clean = R;
            if (!R_clean.allFinite()) return Eigen::Vector3d::Zero();

            double cos_angle = (R_clean.trace() - 1.0) * 0.5;
            cos_angle = std::clamp(cos_angle, -1.0, 1.0);
            const double angle = std::acos(cos_angle);

            if (angle < 1e-8) {
                return Eigen::Vector3d(
                    0.5 * (R_clean(2,1) - R_clean(1,2)),
                    0.5 * (R_clean(0,2) - R_clean(2,0)),
                    0.5 * (R_clean(1,0) - R_clean(0,1)));
            }

            Eigen::AngleAxisd aa(R_clean);
            Eigen::Vector3d axis = aa.axis();
            if (!axis.allFinite() || axis.norm() < 1e-12) return Eigen::Vector3d::Zero();
            axis.normalize();
            return angle * axis;
        };

        auto fk_pose = [&](const std::vector<double>& q_in,
                           Eigen::Vector3d& p_out,
                           Eigen::Matrix3d& R_out)->bool {
            if (q_in.size() < num_joints_) return false;
            KDL::JntArray q_kdl(num_joints_);
            for (unsigned int i = 0; i < num_joints_; ++i) q_kdl(i) = q_in[i];

            KDL::Frame T;
            if (fk_solver_->JntToCart(q_kdl, T) < 0) return false;

            p_out = Eigen::Vector3d(T.p.x(), T.p.y(), T.p.z());
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    R_out(r, c) = T.M(r, c);
                }
            }
            return std::isfinite(p_out.x()) && std::isfinite(p_out.y()) && std::isfinite(p_out.z()) && R_out.allFinite();
        };

        std::vector<double> q(num_joints_, 0.0);
        for (unsigned int i = 0; i < num_joints_; ++i) {
            q[i] = current_q[i];
            if (!std::isfinite(q[i])) q[i] = 0.0;
        }

        Eigen::Vector3d p_cur = Eigen::Vector3d::Zero();
        Eigen::Matrix3d R_cur = Eigen::Matrix3d::Identity();
        if (!fk_pose(q, p_cur, R_cur)) return false;

        double best_score = std::numeric_limits<double>::infinity();
        std::vector<double> best_q = q;

        for (int iter = 0; iter < max_iters; ++iter) {
            if (!fk_pose(q, p_cur, R_cur)) return false;

            const Eigen::Vector3d e_pos = p_target - p_cur;
            const Eigen::Vector3d e_rot = so3Log(R_target * R_cur.transpose());

            const double pos_err = e_pos.norm();
            const double rot_err = e_rot.norm();
            const double score = pos_weight * pos_err + rot_weight * rot_err;

            if (score < best_score) {
                best_score = score;
                best_q = q;
            }
            if (pos_err <= pos_tol_m && rot_err <= rot_tol_rad) {
                result_q = q;
                return true;
            }

            Eigen::MatrixXd J(6, num_joints_);
            J.setZero();

            for (unsigned int j = 0; j < num_joints_; ++j) {
                std::vector<double> q_plus = q;
                std::vector<double> q_minus = q;
                q_plus[j] += numerical_eps;
                q_minus[j] -= numerical_eps;

                Eigen::Vector3d p_plus, p_minus;
                Eigen::Matrix3d R_plus, R_minus;
                if (!fk_pose(q_plus, p_plus, R_plus) || !fk_pose(q_minus, p_minus, R_minus)) {
                    return false;
                }

                J.block<3,1>(0, static_cast<int>(j)) = (p_plus - p_minus) / (2.0 * numerical_eps);
                J.block<3,1>(3, static_cast<int>(j)) = so3Log(R_plus * R_minus.transpose()) / (2.0 * numerical_eps);
            }

            Eigen::MatrixXd Jw = J;
            Jw.block(0, 0, 3, static_cast<int>(num_joints_)) *= pos_weight;
            Jw.block(3, 0, 3, static_cast<int>(num_joints_)) *= rot_weight;

            Eigen::Matrix<double,6,1> ew;
            ew.segment<3>(0) = pos_weight * e_pos;
            ew.segment<3>(3) = rot_weight * e_rot;

            const Eigen::Matrix<double,6,6> A =
                Jw * Jw.transpose() + (lambda * lambda) * Eigen::Matrix<double,6,6>::Identity();

            Eigen::VectorXd dq = Jw.transpose() * A.ldlt().solve(ew);
            if (dq.size() != static_cast<int>(num_joints_)) return false;
            for (int i = 0; i < dq.size(); ++i) {
                if (!std::isfinite(dq(i))) return false;
            }

            const double max_abs_dq = dq.cwiseAbs().maxCoeff();
            if (max_abs_dq > max_joint_step_rad) {
                dq *= (max_joint_step_rad / max_abs_dq);
            }

            for (unsigned int i = 0; i < num_joints_; ++i) {
                q[i] += alpha * dq(static_cast<int>(i));
                q[i] = std::clamp(q[i], -M_PI, M_PI);
            }
        }

        if (!best_q.empty()) {
            Eigen::Vector3d p_best = Eigen::Vector3d::Zero();
            Eigen::Matrix3d R_best = Eigen::Matrix3d::Identity();
            if (fk_pose(best_q, p_best, R_best)) {
                const double pos_err = (p_target - p_best).norm();
                const double rot_err = so3Log(R_target * R_best.transpose()).norm();
                if (pos_err <= std::max(5.0 * pos_tol_m, 0.003) &&
                    rot_err <= std::max(5.0 * rot_tol_rad, 0.035)) {
                    result_q = best_q;
                    return true;
                }
            }
        }

        result_q.clear();
        return false;
    }

    // ==========================
    // 기존(4 args) 코드 호환 overload
    // ==========================
    bool solveIK(const std::vector<double>& current_q,
                 const double target_xyz[3],
                 const double target_rpy[3],
                 std::vector<double>& result_q) const
    {
        std::array<double,3> xyz{target_xyz[0], target_xyz[1], target_xyz[2]};
        std::array<double,3> eul{target_rpy[0], target_rpy[1], target_rpy[2]};
        return solveIK(current_q, xyz, eul, "base", "rpy", "rad", result_q);
    }

private:
    bool ok_;
    KDL::Chain chain_;
    unsigned int num_joints_;

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> vel_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;

    Eigen::Isometry3d world_T_base_;
    bool auto_frame_inference_enabled_;

private:
    static std::string toLower_(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
        return s;
    }

    // --------------------------
    // Helpers for optional auto-frame inference
    // --------------------------
    static double distance3_(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return (a - b).norm();
    }

    bool currentBaseTipFromQ_(const std::vector<double>& current_q, Eigen::Isometry3d& E_base_tip) const {
        if (!ok_ || !fk_solver_) return false;
        if (current_q.size() < num_joints_) return false;

        KDL::JntArray q(num_joints_);
        for (unsigned int i = 0; i < num_joints_; ++i) q(i) = current_q[i];

        KDL::Frame base_T_tip;
        if (fk_solver_->JntToCart(q, base_T_tip) < 0) return false;

        E_base_tip = Eigen::Isometry3d::Identity();
        E_base_tip.translation() = Eigen::Vector3d(base_T_tip.p.x(), base_T_tip.p.y(), base_T_tip.p.z());

        double qx, qy, qz, qw;
        base_T_tip.M.GetQuaternion(qx, qy, qz, qw);
        Eigen::Quaterniond qe(qw, qx, qy, qz);
        qe.normalize();
        E_base_tip.linear() = qe.toRotationMatrix();
        return true;
    }

    // 현재 q 기준으로 target_xyz가 base/world 중 어느 쪽에 더 자연스러운지 추정
    // (기본 OFF)
    std::string inferLikelyFrameFromCurrentQ_(const std::vector<double>& current_q,
                                              const std::array<double,3>& target_xyz) const
    {
        Eigen::Isometry3d E_base_tip;
        if (!currentBaseTipFromQ_(current_q, E_base_tip)) {
            return "base"; // 추정 실패 시 기존 동작 유지
        }

        const Eigen::Vector3d p_target(target_xyz[0], target_xyz[1], target_xyz[2]);
        const Eigen::Vector3d p_base_cur  = E_base_tip.translation();
        const Eigen::Vector3d p_world_cur = (world_T_base_ * E_base_tip).translation();

        const double d_to_base  = distance3_(p_target, p_base_cur);
        const double d_to_world = distance3_(p_target, p_world_cur);

        // 강한 조건일 때만 world로 판정 (오판 방지)
        if (d_to_world + 0.05 < d_to_base) {
            return "world";
        }

        const double dz = std::fabs((p_target.z() - p_base_cur.z()) - world_T_base_.translation().z());
        if (d_to_world < 0.08 && dz < 0.03) {
            return "world";
        }

        return "base";
    }
};

#endif