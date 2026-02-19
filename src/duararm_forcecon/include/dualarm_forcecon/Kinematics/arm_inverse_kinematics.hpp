#ifndef ARM_INVERSE_KINEMATICS_HPP
#define ARM_INVERSE_KINEMATICS_HPP

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <vector>
#include <iostream>

class ArmInverseKinematics {
public:
    ArmInverseKinematics(const std::string& urdf_path, const std::string& base_link, const std::string& tip_link) {
        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(urdf_path, tree)) {
            std::cerr << "[IK Error] Failed to construct KDL tree" << std::endl;
            return;
        }

        if (!tree.getChain(base_link, tip_link, chain_)) {
            std::cerr << "[IK Error] Failed to get chain from " << base_link << " to " << tip_link << std::endl;
            return;
        }

        num_joints_ = chain_.getNrOfJoints();
        // IK Solver 설정 (수치 해석적 방법: Newton-Raphson with joint limits)
        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
        vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);

        // 관절 한계치 설정 (DSR-M1013 기준 대략적인 값 - 필요 시 수정)
        KDL::JntArray q_min(num_joints_), q_max(num_joints_);
        for(unsigned int i=0; i<num_joints_; i++) {
            q_min(i) = -3.14159; 
            q_max(i) = 3.14159;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(chain_, q_min, q_max, *fk_solver_, *vel_solver_, 100, 1e-6);
    }

    bool solveIK(const std::vector<double>& current_q, const double target_xyz[3], const double target_rpy[3], std::vector<double>& result_q) {
        KDL::JntArray q_init(num_joints_);
        for(unsigned int i=0; i<num_joints_; i++) q_init(i) = current_q[i];

        // 목표 Pose 설정 (XYZ + RPY)
        KDL::Frame target_frame;
        target_frame.p = KDL::Vector(target_xyz[0], target_xyz[1], target_xyz[2]);
        target_frame.M = KDL::Rotation::RPY(target_rpy[0], target_rpy[1], target_rpy[2]);

        KDL::JntArray q_out(num_joints_);
        int ret = ik_solver_->CartToJnt(q_init, target_frame, q_out);

        if (ret >= 0) {
            result_q.clear();
            for(unsigned int i=0; i<num_joints_; i++) result_q.push_back(q_out(i));
            return true;
        }
        return false;
    }

private:
    KDL::Chain chain_;
    unsigned int num_joints_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> vel_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;
};

#endif