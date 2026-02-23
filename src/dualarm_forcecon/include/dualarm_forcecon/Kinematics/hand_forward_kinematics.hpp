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
#include <limits>

class HandForwardKinematics {
public:
    // v7 signature 유지: (urdf_path, hand_base_link, finger_tips)
    // finger_tips: frame(link) base names (ex: link4_thumb, ...)
    HandForwardKinematics(const std::string& urdf_path,
                          const std::string& hand_base_link,
                          const std::vector<std::string>& finger_tips)
    : ok_(false),
      urdf_path_(urdf_path),
      hand_base_link_(hand_base_link),
      finger_tips_in_(finger_tips)
    {
        try {
            pinocchio::urdf::buildModel(urdf_path_, model_);
            data_ = pinocchio::Data(model_);
        } catch (const std::exception& e) {
            std::cerr << "[HandFK Error] Pinocchio buildModel failed: " << e.what() << std::endl;
            return;
        }

        side_prefix_ = inferSidePrefix(hand_base_link_);
        if (side_prefix_.empty()) {
            std::cerr << "[HandFK Warn] Cannot infer side from base_link='"
                      << hand_base_link_ << "'. Expecting it contains 'left' or 'right'.\n";
        }

        // base frame id
        base_fid_ = model_.getFrameId(hand_base_link_);
        if (base_fid_ == (pinocchio::FrameIndex)(-1)) {
            std::cerr << "[HandFK Error] Base frame not found in model: " << hand_base_link_ << std::endl;
            return;
        }

        // build resolved tip specs
        tip_specs_.clear();
        int built = 0;
        for (const auto& tip_in : finger_tips_in_) {
            TipSpec spec;
            spec.key_name = tip_in;

            pinocchio::FrameIndex fid = model_.getFrameId(tip_in);
            std::string resolved = tip_in;

            if (fid == (pinocchio::FrameIndex)(-1) && !side_prefix_.empty()) {
                const std::string pref = side_prefix_ + "_" + tip_in;
                fid = model_.getFrameId(pref);
                if (fid != (pinocchio::FrameIndex)(-1)) {
                    resolved = pref;
                }
            }

            if (fid == (pinocchio::FrameIndex)(-1)) {
                // fallback: substring search (must include side if we know it)
                std::string best_name;
                fid = findFrameIdBySubstring(toLower(tip_in),
                                             side_prefix_.empty() ? "" : side_prefix_,
                                             best_name);
                if (fid != (pinocchio::FrameIndex)(-1)) {
                    resolved = best_name;
                }
            }

            if (fid == (pinocchio::FrameIndex)(-1)) {
                std::cerr << "[HandFK Warn] Tip frame not found (tried exact/prefix/search): " << tip_in << std::endl;
                continue;
            }

            spec.model_frame_name = resolved;
            spec.fid = fid;
            tip_specs_.push_back(spec);
            built++;
        }

        // hand joint ids (20 dof): (thumb4, index4, middle4, ring4, baby4)
        buildHandJointIds();

        ok_ = (built > 0);
        std::cout << "[HandFK Info] Pinocchio OK?=" << ok_
                  << " base=" << hand_base_link_
                  << " tips_in=" << finger_tips_in_.size()
                  << " tips_found=" << built
                  << " side=" << side_prefix_
                  << std::endl;

        if (ok_) {
            std::cout << "[HandFK Info] Resolved tips:\n";
            for (const auto& s : tip_specs_) {
                std::cout << "  key='" << s.key_name << "' -> frame='"
                          << s.model_frame_name << "' (fid=" << (int)s.fid << ")\n";
            }
        }
    }

    bool isOk() const { return ok_; }

    // hand_q: 20 dof, order = (thumb4, index4, middle4, ring4, baby4)
    // return: tip pose expressed in hand_base_link frame
    std::map<std::string, geometry_msgs::msg::Pose> computeFingertips(const std::vector<double>& hand_q) const
    {
        std::map<std::string, geometry_msgs::msg::Pose> results;
        if (!ok_) return results;

        Eigen::VectorXd q = pinocchio::neutral(model_);

        // write hand joints into q (0 padding if 부족)
        const int expected = 20;
        for (int i = 0; i < expected; ++i) {
            if (i >= (int)hand_joint_ids_.size()) break;
            const auto jid = hand_joint_ids_[i];
            if (jid == 0) continue; // not found
            const int idx_q = model_.joints[jid].idx_q();
            const double v = (i < (int)hand_q.size()) ? hand_q[i] : 0.0;
            q[idx_q] = v;
        }

        // FK
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        const pinocchio::SE3& oMbase = data_.oMf[base_fid_];

        // each tip: rel = base^{-1} * tip
        for (const auto& spec : tip_specs_) {
            const pinocchio::SE3& oMtip = data_.oMf[spec.fid];
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

            // IMPORTANT: key는 입력 tip 이름(예: link4_thumb) 유지
            results[spec.key_name] = p;
        }

        return results;
    }

    // base(world) pose + relative pose -> world point (기존 유지)
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
    struct TipSpec {
        std::string key_name;         // input key (ex: link4_thumb)
        std::string model_frame_name; // resolved frame name in URDF (ex: left_link4_thumb)
        pinocchio::FrameIndex fid{(pinocchio::FrameIndex)(-1)};
    };

    static std::string toLower(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(), ::tolower);
        return s;
    }

    static std::string inferSidePrefix(const std::string& s)
    {
        std::string low = toLower(s);
        if (low.find("left") != std::string::npos)  return "left";
        if (low.find("right") != std::string::npos) return "right";
        return "";
    }

    pinocchio::FrameIndex findFrameIdBySubstring(const std::string& needle_low,
                                                 const std::string& must_contain_low,
                                                 std::string& best_name) const
    {
        pinocchio::FrameIndex best = (pinocchio::FrameIndex)(-1);
        size_t best_len = std::numeric_limits<size_t>::max();
        best_name.clear();

        for (pinocchio::FrameIndex i = 0; i < (pinocchio::FrameIndex)model_.nframes; ++i) {
            const std::string& nm = model_.frames[i].name;
            std::string low = toLower(nm);

            if (low.find(needle_low) == std::string::npos) continue;
            if (!must_contain_low.empty() && low.find(must_contain_low) == std::string::npos) continue;

            if (nm.size() < best_len) {
                best = i;
                best_len = nm.size();
                best_name = nm;
            }
        }
        return best;
    }

    void buildHandJointIds()
    {
        hand_joint_ids_.assign(20, 0);

        const std::vector<std::string> fingers = {"thumb","index","middle","ring","baby"};

        auto tryGet = [&](const std::vector<std::string>& cands)->pinocchio::JointIndex{
            for (const auto& n : cands) {
                pinocchio::JointIndex jid = model_.getJointId(n);
                if (jid != 0) return jid;
            }
            return 0;
        };

        int k = 0;
        for (int f = 0; f < 5; ++f) {
            for (int j = 1; j <= 4; ++j) {
                std::vector<std::string> cands;

                if (!side_prefix_.empty()) {
                    cands.push_back(side_prefix_ + "_" + fingers[f] + "_joint" + std::to_string(j));
                    cands.push_back(side_prefix_ + "_" + fingers[f] + "_joint_" + std::to_string(j));
                    cands.push_back(side_prefix_ + "_" + fingers[f] + "_j" + std::to_string(j));
                }
                cands.push_back(fingers[f] + "_joint" + std::to_string(j));
                cands.push_back(fingers[f] + "_joint_" + std::to_string(j));

                pinocchio::JointIndex jid = tryGet(cands);

                if (k < 20) hand_joint_ids_[k] = jid;
                k++;
            }
        }
    }

private:
    bool ok_;
    std::string urdf_path_;
    std::string hand_base_link_;

    std::vector<std::string> finger_tips_in_;   // input tips (canonical keys)
    std::vector<TipSpec> tip_specs_;

    std::string side_prefix_;

    pinocchio::Model model_;
    mutable pinocchio::Data data_;

    pinocchio::FrameIndex base_fid_{(pinocchio::FrameIndex)(-1)};

    // size 20, JointIndex (0 means not found)
    std::vector<pinocchio::JointIndex> hand_joint_ids_;
};

#endif