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

#include <Eigen/Dense>
#include <Eigen/Geometry>

class ArmForwardKinematics {
public:
    ArmForwardKinematics(const std::string& urdf_path,
                         const std::string& base_link,
                         const std::string& left_tip_link,
                         const std::string& right_tip_link)
    : world_T_base_(Eigen::Affine3d::Identity())
    {
        urdf::Model robot_model;
        if (!robot_model.initFile(urdf_path)) {
            std::cerr << "[FK Error] Failed to parse URDF file at: " << urdf_path << std::endl;
            ok_ = false;
            return;
        }

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)) {
            std::cerr << "[FK Error] Failed to construct KDL tree from URDF." << std::endl;
            ok_ = false;
            return;
        }

        if (!kdl_tree.getChain(base_link, left_tip_link, left_chain_)) {
            std::cerr << "[FK Error] Failed to get left arm chain from " << base_link
                      << " to " << left_tip_link << std::endl;
            ok_ = false;
            return;
        }
        if (!kdl_tree.getChain(base_link, right_tip_link, right_chain_)) {
            std::cerr << "[FK Error] Failed to get right arm chain from " << base_link
                      << " to " << right_tip_link << std::endl;
            ok_ = false;
            return;
        }

        left_fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(left_chain_);
        right_fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(right_chain_);

        ok_ = true;
        std::cout << "[FK Info] Arm Forward Kinematics initialized successfully!" << std::endl;
    }

    bool isOk() const { return ok_; }

    // World_T_base 설정 (Isaac UI와 동일한 World 기준 출력용)
    void setWorldBaseTransformXYZEulerDeg(const std::vector<double>& xyz_m,
                                         const std::vector<double>& euler_xyz_deg)
    {
        if (xyz_m.size() < 3 || euler_xyz_deg.size() < 3) return;

        double rx = deg2rad(euler_xyz_deg[0]);
        double ry = deg2rad(euler_xyz_deg[1]);
        double rz = deg2rad(euler_xyz_deg[2]);

        Eigen::AngleAxisd ax(rx, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ay(ry, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd az(rz, Eigen::Vector3d::UnitZ());

        // Isaac UI: rotateXYZ => R = Rx * Ry * Rz
        Eigen::Matrix3d R = (ax * ay * az).toRotationMatrix();

        world_T_base_ = Eigen::Affine3d::Identity();
        world_T_base_.linear() = R;
        world_T_base_.translation() = Eigen::Vector3d(xyz_m[0], xyz_m[1], xyz_m[2]);
    }

    // base 기준 FK
    geometry_msgs::msg::Pose getLeftFKBase(const std::vector<double>& joint_positions) {
        return computeFKBase(left_chain_, left_fk_solver_, joint_positions);
    }
    geometry_msgs::msg::Pose getRightFKBase(const std::vector<double>& joint_positions) {
        return computeFKBase(right_chain_, right_fk_solver_, joint_positions);
    }

    // world 기준 FK (World_T_base * base_T_tip)
    geometry_msgs::msg::Pose getLeftFKWorld(const std::vector<double>& joint_positions) {
        auto p_base = getLeftFKBase(joint_positions);
        return applyWorldBase(p_base);
    }
    geometry_msgs::msg::Pose getRightFKWorld(const std::vector<double>& joint_positions) {
        auto p_base = getRightFKBase(joint_positions);
        return applyWorldBase(p_base);
    }

    const Eigen::Affine3d& world_T_base() const { return world_T_base_; }

private:
    bool ok_{false};

    KDL::Chain left_chain_;
    KDL::Chain right_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> left_fk_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> right_fk_solver_;

    Eigen::Affine3d world_T_base_;

    static double deg2rad(double d) { return d * M_PI / 180.0; }

    static Eigen::Affine3d poseToAffine(const geometry_msgs::msg::Pose& p) {
        Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
        return T;
    }

    static geometry_msgs::msg::Pose affineToPose(const Eigen::Affine3d& T) {
        geometry_msgs::msg::Pose p;
        p.position.x = T.translation().x();
        p.position.y = T.translation().y();
        p.position.z = T.translation().z();
        Eigen::Quaterniond q(T.linear());
        p.orientation.x = q.x();
        p.orientation.y = q.y();
        p.orientation.z = q.z();
        p.orientation.w = q.w();
        return p;
    }

    geometry_msgs::msg::Pose applyWorldBase(const geometry_msgs::msg::Pose& base_pose) {
        Eigen::Affine3d T_base_tip = poseToAffine(base_pose);
        Eigen::Affine3d T_world_tip = world_T_base_ * T_base_tip;
        return affineToPose(T_world_tip);
    }

    geometry_msgs::msg::Pose computeFKBase(const KDL::Chain& chain,
                                          const std::shared_ptr<KDL::ChainFkSolverPos_recursive>& solver,
                                          const std::vector<double>& joint_positions)
    {
        geometry_msgs::msg::Pose pose_msg;

        if (!ok_ || !solver) {
            std::cerr << "[FK Error] FK solver not ready." << std::endl;
            return pose_msg;
        }

        if (joint_positions.size() != chain.getNrOfJoints()) {
            std::cerr << "[FK Error] Joint size mismatch! Expected: " << chain.getNrOfJoints()
                      << ", Got: " << joint_positions.size() << std::endl;
            return pose_msg;
        }

        KDL::JntArray q_current(chain.getNrOfJoints());
        for (size_t i = 0; i < joint_positions.size(); ++i) q_current(i) = joint_positions[i];

        KDL::Frame end_effector_pose;
        if (solver->JntToCart(q_current, end_effector_pose) >= 0) {
            pose_msg.position.x = end_effector_pose.p.x();
            pose_msg.position.y = end_effector_pose.p.y();
            pose_msg.position.z = end_effector_pose.p.z();

            double x, y, z, w;
            end_effector_pose.M.GetQuaternion(x, y, z, w);
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

#endif // DUALARM_FORCECON_KINEMATICS_ARM_FORWARD_KINEMATICS_HPP_
