#ifndef DUALARM_FORCECON_KINEMATICS_UTILS_HPP_
#define DUALARM_FORCECON_KINEMATICS_UTILS_HPP_

#include <array>
#include <string>
#include <cmath>
#include <algorithm>

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

// -------------------------------
// Isaac UI의 Orient(XYZ deg)와 동일하게 나오도록
// Euler XYZ 추출 (R = Rx * Ry * Rz 기준)
// -------------------------------
inline std::array<double,3> rotMatToEulerXYZRad_Isaac(const Eigen::Matrix3d& R) {
    // R = Rx(rx) * Ry(ry) * Rz(rz)
    // sy = R02
    double sy = std::clamp(R(0,2), -1.0, 1.0);
    double ry = std::asin(sy);

    double cy = std::cos(ry);

    double rx = 0.0;
    double rz = 0.0;

    if (std::abs(cy) > 1e-8) {
        // rx = atan2(-R12, R22), rz = atan2(-R01, R00)
        rx = std::atan2(-R(1,2), R(2,2));
        rz = std::atan2(-R(0,1), R(0,0));
    } else {
        // gimbal lock: ry ~ +/-90 deg
        // rz를 0으로 두고 rx를 다른 항으로 추정
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

    // R = Rx * Ry * Rz
    return (Ax * Ay * Az).toRotationMatrix();
}

// RPY(rad) -> RotationMatrix (KDL Rotation::RPY와 동일한 의미로 쓰기 위해)
inline Eigen::Matrix3d rpyRadToRotMat(const std::array<double,3>& rpy_rad) {
    double r = rpy_rad[0];
    double p = rpy_rad[1];
    double y = rpy_rad[2];

    Eigen::AngleAxisd Ax(r, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ay(p, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Az(y, Eigen::Vector3d::UnitZ());

    // 일반적으로 RPY는 R = Rz(yaw)*Ry(pitch)*Rx(roll)을 쓰는 경우가 많지만
    // 네 기존 구현은 KDL::Rotation::RPY(r,p,y)를 사용했으므로,
    // "IK에서는 KDL로 직접 생성"하는 쪽이 안전함.
    // 여기 함수는 단순 유틸로 남기되, 실제 IK는 KDL쪽에서 생성하도록 둔다.
    return (Az * Ay * Ax).toRotationMatrix();
}

// -------------------------------
// Pose 합성: base(world) * relative(base)
// (기존 combinePose 역할을 include로 이동)
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

} // namespace kin
} // namespace dualarm_forcecon

#endif
