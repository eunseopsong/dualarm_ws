#ifndef HAND_FORWARD_KINEMATICS_HPP
#define HAND_FORWARD_KINEMATICS_HPP

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

class HandForwardKinematics {
public:
    HandForwardKinematics(const std::string& urdf_path,
                          const std::string& hand_base_link,
                          const std::vector<std::string>& finger_tips)
    : ok_(false), hand_base_link_(hand_base_link), finger_tips_(finger_tips)
    {
        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(urdf_path, tree)) {
            std::cerr << "[HandFK Error] Failed to parse URDF: " << urdf_path << std::endl;
            return;
        }

        int built = 0;
        for (const auto& tip : finger_tips_) {
            KDL::Chain chain;
            if (tree.getChain(hand_base_link_, tip, chain)) {
                solvers_[tip] = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
                num_jnts_[tip] = chain.getNrOfJoints();
                built++;
            } else {
                std::cerr << "[HandFK Warn] Chain not found: " << hand_base_link_ << " -> " << tip << std::endl;
            }
        }

        ok_ = (built > 0);
        std::cout << "[HandFK Info] base=" << hand_base_link_
                  << " tips_in=" << finger_tips_.size()
                  << " solvers_built=" << built << std::endl;
    }

    bool isOk() const { return ok_; }

    // v8: 생성자에서 받은 finger_tips_를 그대로 사용 (하드코딩 제거)
    // hand_q: 20 dof, order = (thumb4, index4, middle4, ring4, baby4)
    std::map<std::string, geometry_msgs::msg::Pose> computeFingertips(const std::vector<double>& hand_q) const {
        std::map<std::string, geometry_msgs::msg::Pose> results;
        if (!ok_) return results;

        constexpr int DOF_PER_FINGER = 4;
        const int expected = static_cast<int>(finger_tips_.size()) * DOF_PER_FINGER;
        if (static_cast<int>(hand_q.size()) < expected) {
            // 부족하면 0으로 패딩해서라도 계산 시도
            // (여기서 return하면 print가 고정되어 보이기 쉬움)
        }

        for (size_t i = 0; i < finger_tips_.size(); ++i) {
            const std::string& tip = finger_tips_[i];
            const int start_idx = static_cast<int>(i) * DOF_PER_FINGER;

            auto it = solvers_.find(tip);
            if (it == solvers_.end()) continue;

            const unsigned int nj = num_jnts_.at(tip);
            KDL::JntArray jnt_pos(nj);

            for (unsigned int j = 0; j < nj; ++j) {
                const int idx = start_idx + static_cast<int>(j);
                jnt_pos(j) = (idx >= 0 && idx < static_cast<int>(hand_q.size())) ? hand_q[idx] : 0.0;
            }

            KDL::Frame frame;
            if (it->second->JntToCart(jnt_pos, frame) >= 0) {
                geometry_msgs::msg::Pose p;
                p.position.x = frame.p.x();
                p.position.y = frame.p.y();
                p.position.z = frame.p.z();
                frame.M.GetQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
                results[tip] = p;
            }
        }

        return results;
    }

    // base(world) pose + relative pose -> world point
    static geometry_msgs::msg::Point combinePosePoint(const geometry_msgs::msg::Pose& base_world,
                                                      const geometry_msgs::msg::Pose& rel_in_base)
    {
        Eigen::Translation3d t_b(base_world.position.x, base_world.position.y, base_world.position.z);
        Eigen::Quaterniond q_b(base_world.orientation.w,
                               base_world.orientation.x,
                               base_world.orientation.y,
                               base_world.orientation.z);
        Eigen::Affine3d T_world_base = t_b * q_b;

        Eigen::Translation3d t_r(rel_in_base.position.x, rel_in_base.position.y, rel_in_base.position.z);
        Eigen::Quaterniond q_r(rel_in_base.orientation.w,
                               rel_in_base.orientation.x,
                               rel_in_base.orientation.y,
                               rel_in_base.orientation.z);
        Eigen::Affine3d T_base_rel = t_r * q_r;

        Eigen::Affine3d T_world_rel = T_world_base * T_base_rel;

        geometry_msgs::msg::Point out;
        out.x = T_world_rel.translation().x();
        out.y = T_world_rel.translation().y();
        out.z = T_world_rel.translation().z();
        return out;
    }

private:
    bool ok_;
    std::string hand_base_link_;
    std::vector<std::string> finger_tips_;

    std::map<std::string, std::shared_ptr<KDL::ChainFkSolverPos_recursive>> solvers_;
    std::map<std::string, unsigned int> num_jnts_;
};

#endif
