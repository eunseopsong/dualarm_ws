#ifndef HAND_FORWARD_KINEMATICS_HPP
#define HAND_FORWARD_KINEMATICS_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <vector>
#include <string>
#include <map>
#include <memory>
#include <iostream>
#include <algorithm>

class HandForwardKinematics {
public:
    // v7 시그니처 유지: (urdf_path, hand_base_link, finger_tips)
    // finger_tips: frame(link) names (ex: link4_thumb, ...)
    HandForwardKinematics(const std::string& urdf_path,
                          const std::string& hand_base_link,
                          const std::vector<std::string>& finger_tips)
    : ok_(false),
      urdf_path_(urdf_path),
      hand_base_link_(hand_base_link),
      finger_tips_(finger_tips)
    {
        try {
            // fixed-base model
            pinocchio::urdf::buildModel(urdf_path_, model_);
            data_ = pinocchio::Data(model_);
        } catch (const std::exception& e) {
            std::cerr << "[HandFK Error] Pinocchio buildModel failed: " << e.what() << std::endl;
            return;
        }

        // infer side from base link string
        side_prefix_ = inferSidePrefix(hand_base_link_);
        if (side_prefix_.empty()) {
            std::cerr << "[HandFK Warn] Cannot infer side from base_link='"
                      << hand_base_link_ << "'. "
                      << "Expecting it contains 'left' or 'right'.\n";
        }

        // base frame id
        base_fid_ = model_.getFrameId(hand_base_link_);
        if (base_fid_ == (pinocchio::FrameIndex)(-1)) {
            std::cerr << "[HandFK Error] Base frame not found in model: " << hand_base_link_ << std::endl;
            return;
        }

        // tip frame ids
        tip_fids_.clear();
        int built = 0;
        for (const auto& tip : finger_tips_) {
            auto fid = model_.getFrameId(tip);
            if (fid == (pinocchio::FrameIndex)(-1)) {
                std::cerr << "[HandFK Warn] Tip frame not found: " << tip << std::endl;
                continue;
            }
            tip_fids_.push_back(fid);
            built++;
        }

        // hand joint ids (20 dof): (thumb4, index4, middle4, ring4, baby4)
        buildHandJointIds();

        ok_ = (built > 0);
        std::cout << "[HandFK Info] Pinocchio OK?=" << ok_
                  << " base=" << hand_base_link_
                  << " tips_in=" << finger_tips_.size()
                  << " tips_found=" << built
                  << " side=" << side_prefix_
                  << std::endl;
    }

    bool isOk() const { return ok_; }

    // hand_q: 20 dof, order = (thumb4, index4, middle4, ring4, baby4)
    // return: tip pose expressed in hand_base_link frame
    std::map<std::string, geometry_msgs::msg::Pose> computeFingertips(const std::vector<double>& hand_q) const
    {
        std::map<std::string, geometry_msgs::msg::Pose> results;
        if (!ok_) return results;

        // q full
        Eigen::VectorXd q = pinocchio::neutral(model_);

        // write hand joints into q
        // 부족하면 0 padding
        const int expected = 20;
        for (int i = 0; i < expected; ++i) {
            if (i >= (int)hand_joint_ids_.size()) break;
            const auto jid = hand_joint_ids_[i];
            if (jid == 0) continue; // not found
            const int idx_q = model_.joints[jid].idx_q();
            const double v = (i < (int)hand_q.size()) ? hand_q[i] : 0.0;
            // 대부분 1DoF revolute -> nq=1
            q[idx_q] = v;
        }

        // FK
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        const pinocchio::SE3& oMbase = data_.oMf[base_fid_];

        // each tip: rel = base^{-1} * tip
        for (size_t i = 0; i < finger_tips_.size(); ++i) {
            const std::string& tip_name = finger_tips_[i];

            // tip fid 찾기(입력 tip list 기준)
            auto fid = model_.getFrameId(tip_name);
            if (fid == (pinocchio::FrameIndex)(-1)) continue;

            const pinocchio::SE3& oMtip = data_.oMf[fid];
            pinocchio::SE3 baseMtip = oMbase.inverse() * oMtip;

            geometry_msgs::msg::Pose p;
            p.position.x = baseMtip.translation()(0);
            p.position.y = baseMtip.translation()(1);
            p.position.z = baseMtip.translation()(2);

            Eigen::Quaterniond qtip(baseMtip.rotation());
            p.orientation.x = qtip.x();
            p.orientation.y = qtip.y();
            p.orientation.z = qtip.z();
            p.orientation.w = qtip.w();

            results[tip_name] = p;
        }

        return results;
    }

    // base(world) pose + relative pose -> world point (v7 유지)
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
    static std::string inferSidePrefix(const std::string& s)
    {
        std::string low = s;
        std::transform(low.begin(), low.end(), low.begin(), ::tolower);
        if (low.find("left") != std::string::npos)  return "left";
        if (low.find("right") != std::string::npos) return "right";
        return "";
    }

    void buildHandJointIds()
    {
        hand_joint_ids_.assign(20, 0);

        // naming convention: "<side>_<finger>_joint<1..4>"
        const std::vector<std::string> fingers = {"thumb","index","middle","ring","baby"};

        int k = 0;
        for (int f = 0; f < 5; ++f) {
            for (int j = 1; j <= 4; ++j) {
                std::string jn;
                if (!side_prefix_.empty()) {
                    jn = side_prefix_ + "_" + fingers[f] + "_joint" + std::to_string(j);
                } else {
                    // side 못 잡히면 그래도 한번 시도(URDF가 side prefix 없이 있을 수도 있으니)
                    jn = fingers[f] + "_joint" + std::to_string(j);
                }

                pinocchio::JointIndex jid = model_.getJointId(jn);
                if (jid == 0) {
                    // 없으면 경고만(빌드/실행은 계속)
                    // std::cerr << "[HandFK Warn] Joint not found: " << jn << std::endl;
                }
                if (k < 20) hand_joint_ids_[k] = jid;
                k++;
            }
        }
    }

private:
    bool ok_;
    std::string urdf_path_;
    std::string hand_base_link_;
    std::vector<std::string> finger_tips_;

    std::string side_prefix_;

    pinocchio::Model model_;
    mutable pinocchio::Data data_;

    pinocchio::FrameIndex base_fid_{(pinocchio::FrameIndex)(-1)};
    std::vector<pinocchio::FrameIndex> tip_fids_;

    // size 20, JointIndex (0 means not found)
    std::vector<pinocchio::JointIndex> hand_joint_ids_;
};

#endif
