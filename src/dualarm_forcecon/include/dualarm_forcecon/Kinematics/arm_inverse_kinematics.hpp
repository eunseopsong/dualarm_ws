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

#include <Eigen/Dense>

class ArmInverseKinematics {
public:
    ArmInverseKinematics(const std::string& urdf_path,
                         const std::string& base_link,
                         const std::string& tip_link)
    : ok_(false), world_T_base_(Eigen::Isometry3d::Identity())
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

        // Joint limits (기본 [-pi, pi], 필요하면 URDF에서 읽어오도록 확장 가능)
        KDL::JntArray q_min(num_joints_), q_max(num_joints_);
        for (unsigned int i = 0; i < num_joints_; ++i) {
            q_min(i) = -M_PI;
            q_max(i) =  M_PI;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
            chain_, q_min, q_max, *fk_solver_, *vel_solver_, 100, 1e-6
        );

        ok_ = true;
        std::cout << "[IK Info] ArmInverseKinematics OK." << std::endl;
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
        Eigen::Quaterniond q = Rx * Ry * Rz;
        world_T_base_.linear() = q.toRotationMatrix();
    }

    // ==========================
    // v6 시그니처(7 args)
    // ==========================
    bool solveIK(const std::vector<double>& current_q,
                 const std::array<double,3>& target_xyz,
                 const std::array<double,3>& target_euler,
                 const std::string& targets_frame,   // "base" or "world"
                 const std::string& euler_conv,      // "rpy" or "xyz"
                 const std::string& angle_unit,      // "rad" or "deg" or "auto"
                 std::vector<double>& result_q) const
    {
        if (!ok_ || !ik_solver_) return false;
        if (current_q.size() < num_joints_) return false;

        // --------------------------
        // angle unit 처리
        // --------------------------
        auto toRad = [&](double a)->double{
            if (angle_unit == "rad") return a;
            if (angle_unit == "deg") return a * M_PI / 180.0;
            // auto: 값이 크면 deg로 판단
            if (std::fabs(a) > 3.5) return a * M_PI / 180.0;
            return a;
        };

        const double a0 = toRad(target_euler[0]);
        const double a1 = toRad(target_euler[1]);
        const double a2 = toRad(target_euler[2]);

        // Rotation 만들기 (Isaac UI Euler XYZ 기준 or RPY)
        Eigen::AngleAxisd Rx(a0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(a1, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(a2, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q;
        // "xyz"든 "rpy"든 현재 v6/v10 UI 매칭 방식 유지: Rx * Ry * Rz
        (void)euler_conv;
        q = Rx * Ry * Rz;

        // 입력 target pose (입력 frame 기준)
        Eigen::Isometry3d E_target_input = Eigen::Isometry3d::Identity();
        E_target_input.translation() = Eigen::Vector3d(target_xyz[0], target_xyz[1], target_xyz[2]);
        E_target_input.linear() = q.toRotationMatrix();

        // --------------------------
        // v13 patch:
        // targets_frame=="base"인데 실제로 world 좌표(모니터 값)를 넣는 경우 자동 보정
        // --------------------------
        std::string effective_frame = targets_frame;
        if (effective_frame.empty()) effective_frame = "base";

        // heuristic: 사용자가 base로 지정했지만 world pose를 넣은 경우 감지
        if (effective_frame == "base") {
            std::string inferred = inferLikelyFrameFromCurrentQ_(current_q, target_xyz);
            if (inferred == "world") {
                effective_frame = "world";
                // 디버그 로그 (너무 시끄러우면 주석처리 가능)
                std::cout << "[IK Info] solveIK auto-frame patch: input targets_frame='base' "
                          << "but target looks like WORLD (monitor pose). Interpreting as world."
                          << std::endl;
            }
        }

        // --------------------------
        // target를 base frame으로 변환
        // --------------------------
        Eigen::Isometry3d E_base_target = E_target_input;
        if (effective_frame == "world") {
            Eigen::Isometry3d E_base_world = world_T_base_.inverse();
            E_base_target = E_base_world * E_target_input;
        }

        // Eigen -> KDL::Frame
        KDL::Frame target_frame;
        target_frame.p = KDL::Vector(E_base_target.translation().x(),
                                     E_base_target.translation().y(),
                                     E_base_target.translation().z());

        Eigen::Quaterniond qb(E_base_target.linear());
        target_frame.M = KDL::Rotation::Quaternion(qb.x(), qb.y(), qb.z(), qb.w());

        KDL::JntArray q_init(num_joints_);
        for (unsigned int i = 0; i < num_joints_; ++i) q_init(i) = current_q[i];

        KDL::JntArray q_out(num_joints_);
        int ret = ik_solver_->CartToJnt(q_init, target_frame, q_out);
        if (ret < 0) return false;

        result_q.resize(num_joints_);
        for (unsigned int i = 0; i < num_joints_; ++i) result_q[i] = q_out(i);
        return true;
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

private:
    // --------------------------
    // Helpers for v13 auto-frame patch
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
        E_base_tip.linear() = qe.toRotationMatrix();
        return true;
    }

    // 현재 q 기준으로 target_xyz가 base/world 중 어느 쪽에 더 자연스러운지 추정
    // - monitor(FK)는 world pose를 보여주므로, 그 값을 base로 잘못 넣었을 때 자동 보정
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
        // 예: home pose 재전송 시 d_to_world ~ 0, d_to_base ~ 0.306
        if (d_to_world + 0.05 < d_to_base) {
            return "world";
        }

        // 추가 안전장치: z축 차이가 world-base z offset과 유사하면 world 가능성↑
        const double dz = std::fabs((p_target.z() - p_base_cur.z()) - world_T_base_.translation().z());
        if (d_to_world < 0.08 && dz < 0.03) {
            return "world";
        }

        return "base";
    }
};

#endif