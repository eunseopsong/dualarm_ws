#ifndef DUALARM_FORCECON_KINEMATICS_ARM_FORWARD_KINEMATICS_HPP_
#define DUALARM_FORCECON_KINEMATICS_ARM_FORWARD_KINEMATICS_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <geometry_msgs/msg/pose.hpp>

class ArmForwardKinematics {
public:
    ArmForwardKinematics(const std::string& urdf_path,
                         const std::string& base_link,
                         const std::string& left_tip_link,
                         const std::string& right_tip_link)
    {
        urdf::Model robot_model;
        if (!robot_model.initFile(urdf_path)) {
            std::cerr << "[FK Error] Failed to parse URDF file at: " << urdf_path << std::endl;
            ready_ = false;
            return;
        }

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)) {
            std::cerr << "[FK Error] Failed to construct KDL tree from URDF." << std::endl;
            ready_ = false;
            return;
        }

        bool okL = kdl_tree.getChain(base_link, left_tip_link, left_chain_);
        bool okR = kdl_tree.getChain(base_link, right_tip_link, right_chain_);

        if (!okL) {
            std::cerr << "[FK Error] Failed to get left arm chain from "
                      << base_link << " to " << left_tip_link << std::endl;
        }
        if (!okR) {
            std::cerr << "[FK Error] Failed to get right arm chain from "
                      << base_link << " to " << right_tip_link << std::endl;
        }
        if (!okL || !okR) {
            ready_ = false;
            return;
        }

        left_fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(left_chain_);
        right_fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(right_chain_);

        ready_ = true;
        std::cout << "[FK Info] Arm Forward Kinematics initialized successfully!" << std::endl;
    }

    bool isReady() const { return ready_; }

    geometry_msgs::msg::Pose getLeftFK(const std::vector<double>& joint_positions) {
        return computeFK(left_chain_, left_fk_solver_, joint_positions);
    }

    geometry_msgs::msg::Pose getRightFK(const std::vector<double>& joint_positions) {
        return computeFK(right_chain_, right_fk_solver_, joint_positions);
    }

private:
    bool ready_{false};

    KDL::Chain left_chain_;
    KDL::Chain right_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> left_fk_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> right_fk_solver_;

    geometry_msgs::msg::Pose computeFK(const KDL::Chain& chain,
                                       const std::shared_ptr<KDL::ChainFkSolverPos_recursive>& solver,
                                       const std::vector<double>& joint_positions)
    {
        geometry_msgs::msg::Pose pose_msg;

        if (!solver) {
            std::cerr << "[FK Error] Solver not initialized." << std::endl;
            return pose_msg;
        }

        if (joint_positions.size() != chain.getNrOfJoints()) {
            std::cerr << "[FK Error] Joint size mismatch! Expected: " << chain.getNrOfJoints()
                      << ", Got: " << joint_positions.size() << std::endl;
            return pose_msg;
        }

        KDL::JntArray q(chain.getNrOfJoints());
        for (size_t i = 0; i < joint_positions.size(); ++i) q(i) = joint_positions[i];

        KDL::Frame T;
        if (solver->JntToCart(q, T) >= 0) {
            pose_msg.position.x = T.p.x();
            pose_msg.position.y = T.p.y();
            pose_msg.position.z = T.p.z();

            double x, y, z, w;
            T.M.GetQuaternion(x, y, z, w);
            pose_msg.orientation.x = x;
            pose_msg.orientation.y = y;
            pose_msg.orientation.z = z;
            pose_msg.orientation.w = w;
        } else {
            std::cerr << "[FK Error] FK computation failed." << std::endl;
        }

        return pose_msg;
    }
};

#endif  // DUALARM_FORCECON_KINEMATICS_ARM_FORWARD_KINEMATICS_HPP_
