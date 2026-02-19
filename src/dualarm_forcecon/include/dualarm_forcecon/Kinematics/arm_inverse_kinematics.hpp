#ifndef ARM_INVERSE_KINEMATICS_HPP
#define ARM_INVERSE_KINEMATICS_HPP

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
#include <cmath>

class ArmInverseKinematics {
public:
    ArmInverseKinematics(const std::string& urdf_path,
                         const std::string& base_link,
                         const std::string& tip_link)
    {
        if (!urdf_model_.initFile(urdf_path)) {
            std::cerr << "[IK Error] Failed to parse URDF: " << urdf_path << std::endl;
            ready_ = false;
            return;
        }

        if (!kdl_parser::treeFromUrdfModel(urdf_model_, tree_)) {
            std::cerr << "[IK Error] Failed to construct KDL tree from URDF model." << std::endl;
            ready_ = false;
            return;
        }

        if (!tree_.getChain(base_link, tip_link, chain_)) {
            std::cerr << "[IK Error] Failed to get chain from " << base_link << " to " << tip_link << std::endl;
            ready_ = false;
            return;
        }

        num_joints_ = chain_.getNrOfJoints();
        if (num_joints_ == 0) {
            std::cerr << "[IK Error] Chain has 0 joints." << std::endl;
            ready_ = false;
            return;
        }

        joint_names_.clear();
        joint_names_.reserve(num_joints_);
        for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
            const auto& seg = chain_.getSegment(i);
            const auto& jnt = seg.getJoint();
            if (jnt.getType() != KDL::Joint::None) joint_names_.push_back(jnt.getName());
        }

        KDL::JntArray q_min(num_joints_), q_max(num_joints_);
        for (unsigned int i = 0; i < num_joints_; ++i) {
            double lo = -M_PI, hi = M_PI;
            if (i < joint_names_.size()) {
                auto uj = urdf_model_.getJoint(joint_names_[i]);
                if (uj && uj->limits) {
                    lo = uj->limits->lower;
                    hi = uj->limits->upper;
                }
            }
            q_min(i) = lo;
            q_max(i) = hi;
        }

        fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
        vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);
        ik_solver_  = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
            chain_, q_min, q_max, *fk_solver_, *vel_solver_, 100, 1e-6);

        ready_ = true;
    }

    bool isReady() const { return ready_; }

    bool solveIK(const std::vector<double>& current_q,
                 const double target_xyz[3],
                 const double target_rpy[3],
                 std::vector<double>& result_q)
    {
        if (!ready_ || !ik_solver_) return false;
        if (current_q.size() < num_joints_) return false;

        KDL::JntArray q_init(num_joints_);
        for (unsigned int i = 0; i < num_joints_; ++i) q_init(i) = current_q[i];

        KDL::Frame target;
        target.p = KDL::Vector(target_xyz[0], target_xyz[1], target_xyz[2]);
        target.M = KDL::Rotation::RPY(target_rpy[0], target_rpy[1], target_rpy[2]);

        KDL::JntArray q_out(num_joints_);
        int ret = ik_solver_->CartToJnt(q_init, target, q_out);

        if (ret >= 0) {
            result_q.resize(num_joints_);
            for (unsigned int i = 0; i < num_joints_; ++i) result_q[i] = q_out(i);
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
};

#endif
