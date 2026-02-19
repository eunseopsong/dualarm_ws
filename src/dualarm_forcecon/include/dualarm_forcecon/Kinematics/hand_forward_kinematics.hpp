#ifndef HAND_FORWARD_KINEMATICS_HPP
#define HAND_FORWARD_KINEMATICS_HPP

#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <string>
#include <map>
#include <memory>

#include <Eigen/Dense>

class HandForwardKinematics {
public:
    HandForwardKinematics(const std::string& urdf_path,
                          const std::string& hand_base_link,
                          const std::vector<std::string>& finger_tips)
    {
        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(urdf_path, tree)) return;

        for (const auto& tip : finger_tips) {
            KDL::Chain chain;
            if (tree.getChain(hand_base_link, tip, chain)) {
                solvers_[tip] = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
                num_jnts_[tip] = chain.getNrOfJoints();
            }
        }
    }

    std::map<std::string, geometry_msgs::msg::Pose> computeFingertips(const std::vector<double>& hand_q) {
        std::map<std::string, geometry_msgs::msg::Pose> results;
        int start_idx = 0;

        // thumb, index, middle, ring, baby (각 4 dof 가정)
        std::vector<std::string> tips = {"link4_thumb", "link4_index", "link4_middle", "link4_ring", "link4_baby"};

        for (size_t i = 0; i < tips.size(); ++i) {
            const std::string& tip = tips[i];
            if (solvers_.find(tip) == solvers_.end()) {
                start_idx += 4;
                continue;
            }

            KDL::JntArray jnt_pos(num_jnts_[tip]);
            for (unsigned int j=0; j<num_jnts_[tip]; ++j) {
                jnt_pos(j) = hand_q[start_idx + j];
            }

            KDL::Frame frame;
            if (solvers_[tip]->JntToCart(jnt_pos, frame) >= 0) {
                geometry_msgs::msg::Pose p;
                p.position.x = frame.p.x();
                p.position.y = frame.p.y();
                p.position.z = frame.p.z();
                frame.M.GetQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
                results[tip] = p;
            }
            start_idx += 4;
        }
        return results;
    }

    // base(world) pose + relative pose -> world point (include에 존재해야 한다는 요구 반영)
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
    std::map<std::string, std::shared_ptr<KDL::ChainFkSolverPos_recursive>> solvers_;
    std::map<std::string, unsigned int> num_jnts_;
};

#endif
