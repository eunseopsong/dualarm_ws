#ifndef DUALARM_FORCECON_KINEMATICS_ARM_INVERSE_KINEMATICS_HPP_
#define DUALARM_FORCECON_KINEMATICS_ARM_INVERSE_KINEMATICS_HPP_

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf/model.h>

#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <array>
#include <cmath>
#include <algorithm>

#include "dualarm_forcecon/Kinematics/kinematics_utils.hpp"

class ArmInverseKinematics {
public:
    ArmInverseKinematics(const std::string& urdf_path,
                         const std::string& base_link,
                         const std::string& tip_link)
    {
        if (!urdf_model_.initFile(urdf_path)) {
            std::cerr << "[IK Error] Failed URDF: " << urdf_path << "\n";
            ready_ = false;
            return;
        }

        if (!kdl_parser::treeFromUrdfModel(urdf_model_, tree_)) {
            std::cerr << "[IK Error] Failed KDL tree.\n";
            ready_ = false;
            return;
        }

        if (!tree_.getChain(base_link, tip_link, chain_)) {
            std::cerr << "[IK Error] Chain not found: " << base_link << " -> " << tip_link << "\n";
            ready_ = false;
            return;
        }

        num_joints_ = chain_.getNrOfJoints();
        if (num_joints_ == 0) {
            std::cerr << "[IK Error] 0 joints.\n";
            ready_ = false;
            return;
        }

        // chain 내 joint name 추출 -> URDF limit 매칭
        joint_names_.clear();
        joint_names_.reserve(num_joints_);
        for (unsigned int i=0; i<chain_.getNrOfSegments(); ++i) {
            const auto& seg = chain_.getSegment(i);
            const auto& jnt = seg.getJoint();
            if (jnt.getType() != KDL::Joint::None) joint_names_.push_back(jnt.getName());
        }

        KDL::JntArray q_min(num_joints_), q_max(num_joints_);
        for (unsigned int i=0; i<num_joints_; ++i) {
            double lo = -M_PI, hi = M_PI;
            if (i < joint_names_.size()) {
                auto uj = urdf_model_.getJoint(joint_names_[i]);
                if (uj && uj->limits) { lo = uj->limits->lower; hi = uj->limits->upper; }
            }
            q_min(i) = lo;
            q_max(i) = hi;
        }

        fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
        vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);
        ik_solver_  = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
            chain_, q_min, q_max, *fk_solver_, *vel_solver_, 100, 1e-6);

        world_T_base_ = KDL::Frame::Identity();
        ready_ = true;
        std::cout << "[IK Info] ArmInverseKinematics ready.\n";
    }

    bool isReady() const { return ready_; }

    void setWorldBaseTransformXYZEulerDeg(const std::array<double,3>& xyz_m,
                                         const std::array<double,3>& euler_xyz_deg)
    {
        const double rx = dualarm_forcecon::kin::deg2rad(euler_xyz_deg[0]);
        const double ry = dualarm_forcecon::kin::deg2rad(euler_xyz_deg[1]);
        const double rz = dualarm_forcecon::kin::deg2rad(euler_xyz_deg[2]);

        KDL::Rotation R = KDL::Rotation::RotX(rx) * KDL::Rotation::RotY(ry) * KDL::Rotation::RotZ(rz);
        KDL::Vector p(xyz_m[0], xyz_m[1], xyz_m[2]);
        world_T_base_ = KDL::Frame(R, p);
    }

    // ------------------------------------------------------------
    // Generic IK:
    //  - xyz: target translation
    //  - ang: rpy(rad) or xyz(deg) depending on euler_conv + angle_unit
    //  - target_frame: "base" or "world"
    //  - euler_conv: "rpy" or "xyz"
    //  - angle_unit: "rad" / "deg" / "auto"
    // ------------------------------------------------------------
    bool solveIK(const std::vector<double>& current_q,
                 const std::array<double,3>& xyz,
                 const std::array<double,3>& ang,
                 const std::string& target_frame,
                 const std::string& euler_conv,
                 const std::string& angle_unit,
                 std::vector<double>& result_q) const
    {
        if (!ready_ || !ik_solver_) return false;
        if (current_q.size() < num_joints_) return false;

        // seed
        KDL::JntArray q_init(num_joints_);
        for (unsigned int i=0; i<num_joints_; ++i) q_init(i) = current_q[i];

        // angle unit 결정
        std::string unit = angle_unit;
        if (unit == "auto") {
            double m = std::max({std::abs(ang[0]), std::abs(ang[1]), std::abs(ang[2])});
            unit = dualarm_forcecon::kin::isProbablyDeg(m) ? "deg" : "rad";
        }

        // target frame 구성 (world or base)
        KDL::Frame T;
        T.p = KDL::Vector(xyz[0], xyz[1], xyz[2]);

        if (euler_conv == "xyz") {
            // Isaac UI의 Orient(XYZ deg)로 들어오는 경우가 일반적 -> deg로 쓰는게 가장 안전
            std::array<double,3> e_deg;
            if (unit == "deg") {
                e_deg = {ang[0], ang[1], ang[2]};
            } else {
                // rad로 들어왔으면 deg로 변환(비추지만 지원)
                e_deg = {dualarm_forcecon::kin::rad2deg(ang[0]),
                         dualarm_forcecon::kin::rad2deg(ang[1]),
                         dualarm_forcecon::kin::rad2deg(ang[2])};
            }
            // R = Rx*Ry*Rz
            const double rx = dualarm_forcecon::kin::deg2rad(e_deg[0]);
            const double ry = dualarm_forcecon::kin::deg2rad(e_deg[1]);
            const double rz = dualarm_forcecon::kin::deg2rad(e_deg[2]);
            T.M = KDL::Rotation::RotX(rx) * KDL::Rotation::RotY(ry) * KDL::Rotation::RotZ(rz);
        } else {
            // euler_conv == "rpy"
            std::array<double,3> rpy_rad;
            if (unit == "rad") {
                rpy_rad = {ang[0], ang[1], ang[2]};
            } else {
                rpy_rad = {dualarm_forcecon::kin::deg2rad(ang[0]),
                           dualarm_forcecon::kin::deg2rad(ang[1]),
                           dualarm_forcecon::kin::deg2rad(ang[2])};
            }
            // 기존 네 코드 호환: KDL::Rotation::RPY(roll,pitch,yaw)
            T.M = KDL::Rotation::RPY(rpy_rad[0], rpy_rad[1], rpy_rad[2]);
        }

        // world->base 변환 적용 여부
        KDL::Frame T_base_target;
        if (target_frame == "world") {
            T_base_target = world_T_base_.Inverse() * T;
        } else {
            T_base_target = T;
        }

        KDL::JntArray q_out(num_joints_);
        int ret = ik_solver_->CartToJnt(q_init, T_base_target, q_out);

        if (ret >= 0) {
            result_q.resize(num_joints_);
            for (unsigned int i=0; i<num_joints_; ++i) result_q[i] = q_out(i);
            return true;
        }
        return false;
    }

private:
    bool ready_{false};

    urdf::Model urdf_model_;
    KDL::Tree tree_;
    KDL::Chain chain_;
    unsigned int num_joints_{0};
    std::vector<std::string> joint_names_;

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> vel_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;

    // World <- base
    KDL::Frame world_T_base_{KDL::Frame::Identity()};
};

#endif
