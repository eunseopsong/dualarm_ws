#ifndef DUALARM_FORCECON_KINEMATICS_CONFIG_HPP_
#define DUALARM_FORCECON_KINEMATICS_CONFIG_HPP_

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace dualarm_forcecon {

struct DualArmKinematicsConfig {
    std::string profile{"doosan_aidin"};

    // URDF and arm kinematic chain names
    std::string urdf_path{};
    std::string arm_base_link{"base_link"};
    std::string left_arm_tip_link{"left_link_6"};
    std::string right_arm_tip_link{"right_link_6"};

    // JointState order-independent mapping for current-state parsing and command publishing.
    std::vector<std::string> left_arm_joint_names{
        "left_joint_1", "left_joint_2", "left_joint_3", "left_joint_4", "left_joint_5", "left_joint_6"};
    std::vector<std::string> right_arm_joint_names{
        "right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4", "right_joint_5", "right_joint_6"};

    // Existing hand pipeline is optional.  For non-AIDIN/Doosan-like robots,
    // set hand.enabled=false and arm-only FK/IK/commanding will still work.
    bool hand_enabled{true};
    std::string left_hand_base_link{"left_joint_6"};
    std::string right_hand_base_link{"right_joint_6"};
    std::vector<std::string> hand_tip_suffixes{
        "link4_thumb", "link4_index", "link4_middle", "link4_ring", "link4_baby"};

    int leftArmDof() const { return static_cast<int>(left_arm_joint_names.size()); }
    int rightArmDof() const { return static_cast<int>(right_arm_joint_names.size()); }

    int findLeftArmJointIndex(const std::string& name) const {
        return findIndex(left_arm_joint_names, name);
    }

    int findRightArmJointIndex(const std::string& name) const {
        return findIndex(right_arm_joint_names, name);
    }

    static int findIndex(const std::vector<std::string>& names, const std::string& name) {
        const auto it = std::find(names.begin(), names.end(), name);
        if (it == names.end()) return -1;
        return static_cast<int>(std::distance(names.begin(), it));
    }

    static std::vector<std::string> defaultJointNames(const std::string& prefix, int dof) {
        std::vector<std::string> names;
        names.reserve(static_cast<std::size_t>(std::max(0, dof)));
        for (int i = 1; i <= dof; ++i) {
            names.push_back(prefix + std::to_string(i));
        }
        return names;
    }

    static DualArmKinematicsConfig fromYaml(const YAML::Node& root,
                                            const std::string& fallback_urdf_path)
    {
        DualArmKinematicsConfig cfg;
        cfg.urdf_path = fallback_urdf_path;

        YAML::Node kin = root ? root["robot_kinematics"] : YAML::Node();
        if (!kin) kin = root ? root["kinematics"] : YAML::Node();
        if (!kin) {
            return cfg;
        }

        readScalar(kin, "profile", cfg.profile);
        readScalar(kin, "urdf_path", cfg.urdf_path);

        const YAML::Node arm = kin["arm"];
        if (arm) {
            readScalar(arm, "base_link", cfg.arm_base_link);
            readScalar(arm, "left_tip_link", cfg.left_arm_tip_link);
            readScalar(arm, "right_tip_link", cfg.right_arm_tip_link);

            readStringVector(arm, "left_joint_names", cfg.left_arm_joint_names);
            readStringVector(arm, "right_joint_names", cfg.right_arm_joint_names);

            // Optional compact fallback:
            //   left_joint_prefix: left_joint_
            //   left_dof: 6
            std::string left_prefix;
            std::string right_prefix;
            int left_dof = -1;
            int right_dof = -1;
            if (readScalar(arm, "left_joint_prefix", left_prefix) && readScalar(arm, "left_dof", left_dof)) {
                cfg.left_arm_joint_names = defaultJointNames(left_prefix, left_dof);
            }
            if (readScalar(arm, "right_joint_prefix", right_prefix) && readScalar(arm, "right_dof", right_dof)) {
                cfg.right_arm_joint_names = defaultJointNames(right_prefix, right_dof);
            }
        }

        const YAML::Node hand = kin["hand"];
        if (hand) {
            readScalar(hand, "enabled", cfg.hand_enabled);
            readScalar(hand, "left_base_link", cfg.left_hand_base_link);
            readScalar(hand, "right_base_link", cfg.right_hand_base_link);
            readStringVector(hand, "tip_suffixes", cfg.hand_tip_suffixes);
        }

        if (cfg.left_arm_joint_names.empty()) {
            std::cerr << "[KinematicsConfig Warn] left_arm_joint_names is empty. Restoring 6-DOF default.\n";
            cfg.left_arm_joint_names = defaultJointNames("left_joint_", 6);
        }
        if (cfg.right_arm_joint_names.empty()) {
            std::cerr << "[KinematicsConfig Warn] right_arm_joint_names is empty. Restoring 6-DOF default.\n";
            cfg.right_arm_joint_names = defaultJointNames("right_joint_", 6);
        }

        return cfg;
    }

private:
    template <typename T>
    static bool readScalar(const YAML::Node& n, const char* key, T& out) {
        if (!n || !n[key]) return false;
        out = n[key].as<T>();
        return true;
    }

    static bool readStringVector(const YAML::Node& n,
                                 const char* key,
                                 std::vector<std::string>& out)
    {
        if (!n || !n[key]) return false;
        const YAML::Node v = n[key];
        if (!v.IsSequence()) return false;
        std::vector<std::string> tmp;
        tmp.reserve(v.size());
        for (std::size_t i = 0; i < v.size(); ++i) {
            tmp.push_back(v[i].as<std::string>());
        }
        out = std::move(tmp);
        return true;
    }
};

} // namespace dualarm_forcecon

#endif // DUALARM_FORCECON_KINEMATICS_CONFIG_HPP_
