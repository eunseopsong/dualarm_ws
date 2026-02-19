#ifndef DUALARM_FORCECON_KINEMATICS_ARM_FORWARD_KINEMATICS_HPP_
#define DUALARM_FORCECON_KINEMATICS_ARM_FORWARD_KINEMATICS_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <array>

// KDL / URDF
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <geometry_msgs/msg/pose.hpp>

#include "dualarm_forcecon/Kinematics/kinematics_utils.hpp"

class ArmForwardKinematics {
public:
    ArmForwardKinematics(const std::string& urdf_path,
                         const std::string& base_link,
                         const std::string& left_tip_link,
                         const std::string& right_tip_link)
    {
        urdf::Model robot_model;
        if (!robot_model.initFile(urdf_path)) {
            std::cerr << "[FK Error] Failed to parse URDF: " << urdf_path << std::endl;
            ready_ = false;
            return;
        }

        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(robot_model, tree)) {
            std::cerr << "[FK Error] Failed to build KDL tree." << std::endl;
            ready_ = false;
            return;
        }

        if (!tree.getChain(base_link, left_tip_link, left_chain_)) {
            std::cerr << "[FK Error] Left chain not found: " << base_link << " -> " << left_tip_link << std::endl;
            ready_ = false;
            return;
        }
        if (!tree.getChain(base_link, right_tip_link, right_chain_)) {
            std::cerr << "[FK Error] Right chain not found: " << base_link << " -> " << right_tip_link << std::endl;
            ready_ = false;
            return;
        }

        left_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(left_chain_);
        right_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(right_chain_);

        world_T_base_ = KDL::Frame::Identity();
        ready_ = true;
        std::cout << "[FK Info] ArmForwardKinematics ready.\n";
    }

    bool isReady() const { return ready_; }

    // ✅ World <- base 고정 변환 (z offset 포함)
    void setWorldBaseTransformXYZEulerDeg(const std::array<double,3>& xyz_m,
                                         const std::array<double,3>& euler_xyz_deg)
    {
        const double rx = dualarm_forcecon::kin::deg2rad(euler_xyz_deg[0]);
        const double ry = dualarm_forcecon::kin::deg2rad(euler_xyz_deg[1]);
        const double rz = dualarm_forcecon::kin::deg2rad(euler_xyz_deg[2]);

        // R = Rx * Ry * Rz (Isaac rotateXYZ 기준)
        KDL::Rotation R = KDL::Rotation::RotX(rx) * KDL::Rotation::RotY(ry) * KDL::Rotation::RotZ(rz);
        KDL::Vector p(xyz_m[0], xyz_m[1], xyz_m[2]);
        world_T_base_ = KDL::Frame(R, p);
    }

    // ---- joints -> world pose (geometry_msgs::Pose) ----
    geometry_msgs::msg::Pose getLeftFKWorldPose(const std::vector<double>& q) const {
        return computeFKWorld(left_chain_, left_solver_, q);
    }
    geometry_msgs::msg::Pose getRightFKWorldPose(const std::vector<double>& q) const {
        return computeFKWorld(right_chain_, right_solver_, q);
    }

    // ---- joints -> (xyz, eulerXYZdeg) : Isaac UI 출력용 ----
    bool getLeftEE_XYZ_EulerXYZDegIsaac(const std::vector<double>& q,
                                       std::array<double,3>& xyz,
                                       std::array<double,3>& euler_xyz_deg) const
    {
        return computeXYZEuler(left_chain_, left_solver_, q, xyz, euler_xyz_deg);
    }

    bool getRightEE_XYZ_EulerXYZDegIsaac(const std::vector<double>& q,
                                        std::array<double,3>& xyz,
                                        std::array<double,3>& euler_xyz_deg) const
    {
        return computeXYZEuler(right_chain_, right_solver_, q, xyz, euler_xyz_deg);
    }

private:
    bool ready_{false};

    KDL::Chain left_chain_, right_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> left_solver_, right_solver_;

    // World <- base
    KDL::Frame world_T_base_{KDL::Frame::Identity()};

    geometry_msgs::msg::Pose computeFKWorld(const KDL::Chain& chain,
                                            const std::shared_ptr<KDL::ChainFkSolverPos_recursive>& solver,
                                            const std::vector<double>& q) const
    {
        geometry_msgs::msg::Pose out;
        if (!solver) return out;
        if (q.size() != chain.getNrOfJoints()) return out;

        KDL::JntArray j(chain.getNrOfJoints());
        for (size_t i=0; i<q.size(); ++i) j(i) = q[i];

        KDL::Frame T_base_tip;
        if (solver->JntToCart(j, T_base_tip) < 0) return out;

        KDL::Frame T_world_tip = world_T_base_ * T_base_tip;

        out.position.x = T_world_tip.p.x();
        out.position.y = T_world_tip.p.y();
        out.position.z = T_world_tip.p.z();

        double x,y,z,w;
        T_world_tip.M.GetQuaternion(x,y,z,w);
        out.orientation.x = x;
        out.orientation.y = y;
        out.orientation.z = z;
        out.orientation.w = w;
        return out;
    }

    bool computeXYZEuler(const KDL::Chain& chain,
                         const std::shared_ptr<KDL::ChainFkSolverPos_recursive>& solver,
                         const std::vector<double>& q,
                         std::array<double,3>& xyz,
                         std::array<double,3>& euler_xyz_deg) const
    {
        if (!solver) return false;
        if (q.size() != chain.getNrOfJoints()) return false;

        KDL::JntArray j(chain.getNrOfJoints());
        for (size_t i=0; i<q.size(); ++i) j(i) = q[i];

        KDL::Frame T_base_tip;
        if (solver->JntToCart(j, T_base_tip) < 0) return false;

        KDL::Frame T_world_tip = world_T_base_ * T_base_tip;

        xyz = {T_world_tip.p.x(), T_world_tip.p.y(), T_world_tip.p.z()};

        double qx,qy,qz,qw;
        T_world_tip.M.GetQuaternion(qx,qy,qz,qw);

        euler_xyz_deg = dualarm_forcecon::kin::quatToEulerXYZDeg_Isaac(qx,qy,qz,qw);
        return true;
    }
};

#endif
