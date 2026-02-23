#ifndef DUALARM_FORCECON_KINEMATICS_UTILS_HPP_
#define DUALARM_FORCECON_KINEMATICS_UTILS_HPP_

#include <array>
#include <string>
#include <cmath>
#include <algorithm>
#include <map>
#include <cctype>
#include <limits>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace dualarm_forcecon {
namespace kin {

inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }

inline double wrapDeg(double a) {
    // [-180, 180)
    while (a >= 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

inline bool isProbablyDeg(double v_abs) {
    // auto 판단: 2*pi보다 크면 deg로 본다(보수적으로 6.4 정도)
    return (v_abs > 6.4);
}

inline std::string toLower(std::string s) {
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

inline bool endsWith(const std::string& s, const std::string& suffix) {
    if (s.size() < suffix.size()) return false;
    return (s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0);
}

inline bool startsWith(const std::string& s, const std::string& prefix) {
    return (s.rfind(prefix, 0) == 0);
}

// -------------------------------
// Isaac UI의 Orient(XYZ deg)와 동일하게 나오도록
// Euler XYZ 추출 (R = Rx * Ry * Rz 기준)
// -------------------------------
inline std::array<double,3> rotMatToEulerXYZRad_Isaac(const Eigen::Matrix3d& R) {
    // R = Rx(rx) * Ry(ry) * Rz(rz)
    double sy = std::clamp(R(0,2), -1.0, 1.0);
    double ry = std::asin(sy);

    double cy = std::cos(ry);
    double rx = 0.0;
    double rz = 0.0;

    if (std::abs(cy) > 1e-8) {
        rx = std::atan2(-R(1,2), R(2,2));
        rz = std::atan2(-R(0,1), R(0,0));
    } else {
        rz = 0.0;
        rx = std::atan2(R(2,1), R(1,1));
    }

    return {rx, ry, rz};
}

inline std::array<double,3> quatToEulerXYZDeg_Isaac(double qx, double qy, double qz, double qw) {
    Eigen::Quaterniond q(qw, qx, qy, qz);
    Eigen::Matrix3d R = q.toRotationMatrix();
    auto e_rad = rotMatToEulerXYZRad_Isaac(R);
    return { wrapDeg(rad2deg(e_rad[0])),
             wrapDeg(rad2deg(e_rad[1])),
             wrapDeg(rad2deg(e_rad[2])) };
}

// Euler XYZ(deg) -> RotationMatrix (R = Rx * Ry * Rz)
inline Eigen::Matrix3d eulerXYZDegToRotMat_Isaac(const std::array<double,3>& euler_xyz_deg) {
    double rx = deg2rad(euler_xyz_deg[0]);
    double ry = deg2rad(euler_xyz_deg[1]);
    double rz = deg2rad(euler_xyz_deg[2]);

    Eigen::AngleAxisd Ax(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ay(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Az(rz, Eigen::Vector3d::UnitZ());

    return (Ax * Ay * Az).toRotationMatrix();
}

// -------------------------------
// Pose 합성: base(world) * relative(base)
// -------------------------------
inline geometry_msgs::msg::Point composePoseTranslationWorld(
    const geometry_msgs::msg::Pose& base_world,
    const geometry_msgs::msg::Pose& relative_in_base)
{
    Eigen::Vector3d tb(base_world.position.x, base_world.position.y, base_world.position.z);
    Eigen::Quaterniond qb(base_world.orientation.w,
                          base_world.orientation.x,
                          base_world.orientation.y,
                          base_world.orientation.z);

    Eigen::Vector3d tr(relative_in_base.position.x, relative_in_base.position.y, relative_in_base.position.z);
    Eigen::Quaterniond qr(relative_in_base.orientation.w,
                          relative_in_base.orientation.x,
                          relative_in_base.orientation.y,
                          relative_in_base.orientation.z);

    Eigen::Affine3d T_world_base = Eigen::Translation3d(tb) * qb;
    Eigen::Affine3d T_base_rel   = Eigen::Translation3d(tr) * qr;
    Eigen::Affine3d T_world_rel  = T_world_base * T_base_rel;

    geometry_msgs::msg::Point p;
    p.x = T_world_rel.translation().x();
    p.y = T_world_rel.translation().y();
    p.z = T_world_rel.translation().z();
    return p;
}

// ============================================================================
// v10: Hand joint name parsing (more robust)
//   supports examples:
//     left_thumb_joint1, left_thumb_joint_1, left_thumb_j1
//     right_index_joint4, right_index_joint_4, right_index_j4
//   also tolerates prefix like: l_*, r_* (only when it clearly starts with it)
// ============================================================================
struct HandJointParseResult {
    bool ok{false};
    bool is_left{false};   // false => right
    int finger_id{-1};     // 0 thumb,1 index,2 middle,3 ring,4 baby
    int joint_id{-1};      // 0..3 for joint1..4
};

inline HandJointParseResult parseHandJointName(const std::string& name) {
    HandJointParseResult r;
    const std::string n = toLower(name);

    // ---- side detect (avoid accidental match like "middle" containing 'l') ----
    bool has_left  = (n.find("left")  != std::string::npos) || startsWith(n, "l_")  || startsWith(n, "lh_");
    bool has_right = (n.find("right") != std::string::npos) || startsWith(n, "r_")  || startsWith(n, "rh_");
    if (!has_left && !has_right) return r;
    if (has_left && has_right)   return r; // ambiguous
    r.is_left = has_left;

    // ---- finger ----
    if      (n.find("thumb")  != std::string::npos) r.finger_id = 0;
    else if (n.find("index")  != std::string::npos) r.finger_id = 1;
    else if (n.find("middle") != std::string::npos) r.finger_id = 2;
    else if (n.find("ring")   != std::string::npos) r.finger_id = 3;
    else if (n.find("baby")   != std::string::npos) r.finger_id = 4;
    else return r;

    // ---- joint index: accept joint1 / joint_1 / j1 형태 ----
    if (n.empty()) return r;
    const char last = n.back();
    if (last >= '1' && last <= '4') {
        const int jid = (last - '1'); // 0..3
        // make sure it's really joint indicator
        if (n.find("joint") != std::string::npos || n.find("_j") != std::string::npos || endsWith(n, "j"+std::string(1,last))) {
            r.joint_id = jid;
            r.ok = true;
            return r;
        }
    }

    // fallback (rare): explicit suffixes
    if      (endsWith(n, "joint1") || endsWith(n, "joint_1") || endsWith(n, "_j1")) r.joint_id = 0;
    else if (endsWith(n, "joint2") || endsWith(n, "joint_2") || endsWith(n, "_j2")) r.joint_id = 1;
    else if (endsWith(n, "joint3") || endsWith(n, "joint_3") || endsWith(n, "_j3")) r.joint_id = 2;
    else if (endsWith(n, "joint4") || endsWith(n, "joint_4") || endsWith(n, "_j4")) r.joint_id = 3;
    else return r;

    r.ok = true;
    return r;
}

// ============================================================================
// v8: find pose in map by keyword (case-insensitive)
// ============================================================================
inline bool findPoseByKeywordCI(const std::map<std::string, geometry_msgs::msg::Pose>& m,
                               const std::string& keyword,
                               geometry_msgs::msg::Pose& out)
{
    const std::string k = toLower(keyword);
    for (const auto& kv : m) {
        const std::string key = toLower(kv.first);
        if (key.find(k) != std::string::npos) {
            out = kv.second;
            return true;
        }
    }
    return false;
}

} // namespace kin
} // namespace dualarm_forcecon

#endif