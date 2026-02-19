#ifndef DUALARM_FORCECON_KINEMATICS_ARM_FORWARD_KINEMATICS_HPP_
#define DUALARM_FORCECON_KINEMATICS_ARM_FORWARD_KINEMATICS_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <memory>

// KDL 및 URDF 관련 헤더
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

// ROS 2 메시지 (End-Effector Pose 반환용)
#include <geometry_msgs/msg/pose.hpp>

class ArmForwardKinematics {
public:
    // 생성자: URDF 파일 경로와 링크 이름들을 받아 KDL 트리를 구성합니다.
    ArmForwardKinematics(const std::string& urdf_path,
                         const std::string& base_link,
                         const std::string& left_tip_link,
                         const std::string& right_tip_link) 
    {
        // 1. URDF 파일 로드
        urdf::Model robot_model;
        if (!robot_model.initFile(urdf_path)) {
            std::cerr << "[FK Error] Failed to parse URDF file at: " << urdf_path << std::endl;
            return;
        }

        // 2. KDL 트리 생성
        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(robot_model, kdl_tree)) {
            std::cerr << "[FK Error] Failed to construct KDL tree from URDF." << std::endl;
            return;
        }

        // 3. 양팔의 Chain(관절/링크 연결 구조) 추출
        if (!kdl_tree.getChain(base_link, left_tip_link, left_chain_)) {
            std::cerr << "[FK Error] Failed to get left arm chain from " << base_link << " to " << left_tip_link << std::endl;
        }
        if (!kdl_tree.getChain(base_link, right_tip_link, right_chain_)) {
            std::cerr << "[FK Error] Failed to get right arm chain from " << base_link << " to " << right_tip_link << std::endl;
        }

        // 4. FK 솔버 초기화
        left_fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(left_chain_);
        right_fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(right_chain_);
        
        std::cout << "[FK Info] Arm Forward Kinematics initialized successfully!" << std::endl;
    }

    // 왼팔의 현재 Pose 계산
    geometry_msgs::msg::Pose getLeftFK(const std::vector<double>& joint_positions) {
        return computeFK(left_chain_, left_fk_solver_, joint_positions);
    }

    // 오른팔의 현재 Pose 계산
    geometry_msgs::msg::Pose getRightFK(const std::vector<double>& joint_positions) {
        return computeFK(right_chain_, right_fk_solver_, joint_positions);
    }

private:
    KDL::Chain left_chain_;
    KDL::Chain right_chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> left_fk_solver_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> right_fk_solver_;

    // 공통 FK 계산 로직 (KDL -> geometry_msgs::msg::Pose 변환)
    geometry_msgs::msg::Pose computeFK(const KDL::Chain& chain,
                                       std::shared_ptr<KDL::ChainFkSolverPos_recursive> solver,
                                       const std::vector<double>& joint_positions) 
    {
        geometry_msgs::msg::Pose pose_msg;

        // 조인트 개수 안전성 검사 (보통 6축 로봇이면 6개여야 함)
        if (joint_positions.size() != chain.getNrOfJoints()) {
            std::cerr << "[FK Error] Joint size mismatch! Expected: " << chain.getNrOfJoints() 
                      << ", Got: " << joint_positions.size() << std::endl;
            return pose_msg;
        }

        // std::vector 를 KDL::JntArray 로 변환
        KDL::JntArray q_current(chain.getNrOfJoints());
        for (size_t i = 0; i < joint_positions.size(); ++i) {
            q_current(i) = joint_positions[i];
        }

        KDL::Frame end_effector_pose;
        // JntToCart 연산 수행 (성공 시 0 이상의 값 반환)
        if (solver->JntToCart(q_current, end_effector_pose) >= 0) {
            // 위치(Position) 저장
            pose_msg.position.x = end_effector_pose.p.x();
            pose_msg.position.y = end_effector_pose.p.y();
            pose_msg.position.z = end_effector_pose.p.z();

            // 회전(Orientation - Quaternion) 저장
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