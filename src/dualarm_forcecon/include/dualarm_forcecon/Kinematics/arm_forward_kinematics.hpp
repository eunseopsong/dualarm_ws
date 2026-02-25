#ifndef DUALARM_FORCECON_KINEMATICS_ARM_FORWARD_KINEMATICS_HPP_
#define DUALARM_FORCECON_KINEMATICS_ARM_FORWARD_KINEMATICS_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <array>
#include <cmath>
#include <algorithm>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <Eigen/Dense>

class ArmForwardKinematics {
public:
    enum class PoseFrame {
        BASE = 0,
        WORLD = 1
    };

public:
    ArmForwardKinematics(const std::string& urdf_path,
                         const std::string& base_link,
                         const std::string& left_tip_link,
                         const std::string& right_tip_link)
    : ok_(false),
      world_T_base_(Eigen::Isometry3d::Identity()),
      default_output_frame_(PoseFrame::BASE)   // ★ 패치: 기본 출력은 BASE (TAR_POS와 프레임 일치)
    {
        urdf::Model robot_model;
        if (!robot_model.initFile(urdf_path)) {
            std::cerr << "[FK Error] Failed to parse URDF: " << urdf_path << std::endl;
            return;
        }

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)) {
            std::cerr << "[FK Error] Failed to build KDL tree from URDF." << std::endl;
            return;
        }

        if (!kdl_tree.getChain(base_link, left_tip_link, left_chain_)) {
            std::cerr << "[FK Error] Failed to get left chain: " << base_link << " -> " << left_tip_link << std::endl;
            return;
        }
        if (!kdl_tree.getChain(base_link, right_tip_link, right_chain_)) {
            std::cerr << "[FK Error] Failed to get right chain: " << base_link << " -> " << right_tip_link << std::endl;
            return;
        }

        left_fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(left_chain_);
        right_fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(right_chain_);

        ok_ = true;
        std::cout << "[FK Info] ArmForwardKinematics OK. default_output_frame=BASE" << std::endl;
    }

    bool isOk() const { return ok_; }

    // Isaac UI 매칭용: world<-base transform (기본 z offset 포함)
    void setWorldBaseTransformXYZEulerDeg(const std::array<double,3>& xyz,
                                          const std::array<double,3>& euler_xyz_deg)
    {
        world_T_base_ = Eigen::Isometry3d::Identity();
        world_T_base_.translation() = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);

        Eigen::Vector3d e = Eigen::Vector3d(euler_xyz_deg[0], euler_xyz_deg[1], euler_xyz_deg[2]) * M_PI / 180.0;
        Eigen::AngleAxisd Rx(e.x(), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(e.y(), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(e.z(), Eigen::Vector3d::UnitZ());

        // 기존 프로젝트 convention 유지
        Eigen::Quaterniond q = Rx * Ry * Rz;
        q.normalize();
        world_T_base_.linear() = q.toRotationMatrix();
    }

    const Eigen::Isometry3d& world_T_base() const { return world_T_base_; }

    // -----------------------------
    // 출력 프레임 설정 (선택사항)
    // -----------------------------
    void setDefaultOutputFrame(PoseFrame frame) { default_output_frame_ = frame; }

    void setDefaultOutputFrameFromString(const std::string& frame_str)
    {
        std::string s = toLower_(frame_str);
        if (s == "world" || s == "global") default_output_frame_ = PoseFrame::WORLD;
        else                               default_output_frame_ = PoseFrame::BASE;
    }

    PoseFrame defaultOutputFrame() const { return default_output_frame_; }

    // ===== 명시적 API (권장) =====
    geometry_msgs::msg::Pose fkLeftBase(const std::vector<double>& joint_positions) const {
        return computeFK(left_chain_, left_fk_solver_, joint_positions, PoseFrame::BASE);
    }

    geometry_msgs::msg::Pose fkRightBase(const std::vector<double>& joint_positions) const {
        return computeFK(right_chain_, right_fk_solver_, joint_positions, PoseFrame::BASE);
    }

    geometry_msgs::msg::Pose fkLeftWorld(const std::vector<double>& joint_positions) const {
        return computeFK(left_chain_, left_fk_solver_, joint_positions, PoseFrame::WORLD);
    }

    geometry_msgs::msg::Pose fkRightWorld(const std::vector<double>& joint_positions) const {
        return computeFK(right_chain_, right_fk_solver_, joint_positions, PoseFrame::WORLD);
    }

    // ===== 호환 API =====
    // 기존 코드가 getLeftFK/getRightFK를 호출하면 default_output_frame_ 기준으로 반환
    geometry_msgs::msg::Pose getLeftFK(const std::vector<double>& joint_positions) const {
        return computeFK(left_chain_, left_fk_solver_, joint_positions, default_output_frame_);
    }

    geometry_msgs::msg::Pose getRightFK(const std::vector<double>& joint_positions) const {
        return computeFK(right_chain_, right_fk_solver_, joint_positions, default_output_frame_);
    }

    // Isaac UI Euler XYZ(deg) 변환(Quaternion -> EulerXYZdeg)
    // (Print에서 사용)
    static void quatToEulerXYZDeg_Isaac(const geometry_msgs::msg::Quaternion& q,
                                        double& ex_deg, double& ey_deg, double& ez_deg)
    {
        const double x=q.x, y=q.y, z=q.z, w=q.w;

        const double r00 = 1.0 - 2.0*(y*y + z*z);
        const double r01 = 2.0*(x*y - z*w);
        const double r02 = 2.0*(x*z + y*w);

        const double r12 = 2.0*(y*z - x*w);
        const double r22 = 1.0 - 2.0*(x*x + y*y);

        double s = r02;
        if (s >  1.0) s =  1.0;
        if (s < -1.0) s = -1.0;

        const double ey = std::asin(s);
        const double c = std::cos(ey);

        double ex = 0.0, ez = 0.0;
        if (std::fabs(c) < 1e-9) {
            ex = std::atan2(-r12, r22);
            ez = 0.0;
        } else {
            ex = std::atan2(-r12, r22);
            ez = std::atan2(-r01, r00);
        }

        const double rad2deg = 180.0 / M_PI;
        ex_deg = ex * rad2deg;
        ey_deg = ey * rad2deg;
        ez_deg = ez * rad2deg;
    }

private:
    bool ok_;
    KDL::Chain left_chain_;
    KDL::Chain right_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> left_fk_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> right_fk_solver_;

    Eigen::Isometry3d world_T_base_;
    PoseFrame default_output_frame_;

private:
    static std::string toLower_(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
        return s;
    }

    geometry_msgs::msg::Pose computeFK(const KDL::Chain& chain,
                                       const std::shared_ptr<KDL::ChainFkSolverPos_recursive>& solver,
                                       const std::vector<double>& joint_positions,
                                       PoseFrame out_frame) const
    {
        geometry_msgs::msg::Pose pose_msg{};
        pose_msg.orientation.w = 1.0;  // invalid return 대비 기본값

        if (!ok_ || !solver) return pose_msg;

        if (joint_positions.size() != chain.getNrOfJoints()) {
            std::cerr << "[FK Error] Joint size mismatch. Expected " << chain.getNrOfJoints()
                      << " Got " << joint_positions.size() << std::endl;
            return pose_msg;
        }

        KDL::JntArray q(chain.getNrOfJoints());
        for (size_t i = 0; i < joint_positions.size(); ++i) q(i) = joint_positions[i];

        KDL::Frame base_T_tip;
        if (solver->JntToCart(q, base_T_tip) < 0) {
            std::cerr << "[FK Error] JntToCart failed." << std::endl;
            return pose_msg;
        }

        // base_T_tip -> Eigen
        Eigen::Isometry3d E_base_tip = Eigen::Isometry3d::Identity();
        E_base_tip.translation() = Eigen::Vector3d(base_T_tip.p.x(), base_T_tip.p.y(), base_T_tip.p.z());

        double qx, qy, qz, qw;
        base_T_tip.M.GetQuaternion(qx, qy, qz, qw);
        Eigen::Quaterniond q_e(qw, qx, qy, qz);
        q_e.normalize();
        E_base_tip.linear() = q_e.toRotationMatrix();

        Eigen::Isometry3d E_out = E_base_tip;
        if (out_frame == PoseFrame::WORLD) {
            E_out = world_T_base_ * E_base_tip;
        }

        Eigen::Quaterniond q_out(E_out.linear());
        q_out.normalize();

        pose_msg.position.x = E_out.translation().x();
        pose_msg.position.y = E_out.translation().y();
        pose_msg.position.z = E_out.translation().z();

        pose_msg.orientation.x = q_out.x();
        pose_msg.orientation.y = q_out.y();
        pose_msg.orientation.z = q_out.z();
        pose_msg.orientation.w = q_out.w();

        return pose_msg;
    }
};

#endif