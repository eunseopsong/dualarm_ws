#ifndef HAND_FORWARD_KINEMATICS_HPP
#define HAND_FORWARD_KINEMATICS_HPP

#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>
#include <map>
#include <memory>

class HandForwardKinematics {
public:
    HandForwardKinematics(const std::string& urdf_path, const std::string& hand_base_link, const std::vector<std::string>& finger_tips) {
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
        
        // Aidin Hand 관절 구조 매핑 (Thumb, Index, Middle, Ring, Baby 순서 가정)
        // 각 손가락은 보통 4개의 관절을 가짐
        std::vector<std::string> tips = {"link4_thumb", "link4_index", "link4_middle", "link4_ring", "link4_baby"};

        for (size_t i = 0; i < tips.size(); ++i) {
            std::string tip = tips[i];
            if (solvers_.find(tip) == solvers_.end()) {
                start_idx += 4;
                continue;
            }

            KDL::JntArray jnt_pos(num_jnts_[tip]);
            for(unsigned int j=0; j<num_jnts_[tip]; j++) {
                jnt_pos(j) = hand_q[start_idx + j];
            }

            KDL::Frame frame;
            if (solvers_[tip]->JntToCart(jnt_pos, frame) >= 0) {
                geometry_msgs::msg::Pose p;
                p.position.x = frame.p.x(); p.position.y = frame.p.y(); p.position.z = frame.p.z();
                frame.M.GetQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
                results[tip] = p;
            }
            start_idx += 4;
        }
        return results;
    }

private:
    std::map<std::string, std::shared_ptr<KDL::ChainFkSolverPos_recursive>> solvers_;
    std::map<std::string, unsigned int> num_jnts_;
};

#endif