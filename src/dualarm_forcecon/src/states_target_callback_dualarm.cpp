#include "DualArmForceControl.h"

#include <cstdio>
#include <cmath>
#include <limits>
#include <map>
#include <array>
#include <algorithm>
#include <cctype>

namespace {

inline void setPointFromVec(geometry_msgs::msg::Point& p, const Eigen::Vector3d& v)
{
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
}

inline Eigen::Vector3d pointToVec(const geometry_msgs::msg::Point& p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}

inline bool isFiniteVec3(const Eigen::Vector3d& v)
{
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

inline Eigen::Vector3d safeGetTip(const std::vector<Eigen::Vector3d>& v, int idx)
{
    if (idx < 0 || idx >= static_cast<int>(v.size())) return Eigen::Vector3d::Zero();
    return v[static_cast<std::size_t>(idx)];
}

inline void matrixRowToPoint(const Eigen::Matrix<double,5,3>& M, int row, geometry_msgs::msg::Point& p)
{
    if (row < 0 || row >= 5) {
        p.x = 0.0; p.y = 0.0; p.z = 0.0;
        return;
    }
    p.x = M(row, 0);
    p.y = M(row, 1);
    p.z = M(row, 2);
}

inline void writeMatrixRow(Eigen::Matrix<double,5,3>& M, int row, const Eigen::Vector3d& v)
{
    if (row < 0 || row >= 5) return;
    M(row, 0) = v.x();
    M(row, 1) = v.y();
    M(row, 2) = v.z();
}

inline std::vector<double> compress20to15(const Eigen::VectorXd& qh)
{
    std::vector<double> h15(15, 0.0);

    if (qh.size() >= 20) {
        for (int f = 0; f < 5; ++f) {
            const int b20 = f * 4;
            const int b15 = f * 3;
            h15[b15 + 0] = qh(b20 + 0);
            h15[b15 + 1] = qh(b20 + 1);
            h15[b15 + 2] = qh(b20 + 2);
        }
        return h15;
    }

    const int n = std::min<int>(qh.size(), 15);
    for (int i = 0; i < n; ++i) h15[i] = qh(i);
    return h15;
}

inline std::vector<double> eigenToStdVec(const Eigen::VectorXd& q)
{
    std::vector<double> out(static_cast<std::size_t>(q.size()), 0.0);
    for (int i = 0; i < q.size(); ++i) out[static_cast<std::size_t>(i)] = q(i);
    return out;
}

inline void updatePointSetFromMatrix(const Eigen::Matrix<double,5,3>& M,
                                     geometry_msgs::msg::Point& thumb,
                                     geometry_msgs::msg::Point& index,
                                     geometry_msgs::msg::Point& middle,
                                     geometry_msgs::msg::Point& ring,
                                     geometry_msgs::msg::Point& baby)
{
    matrixRowToPoint(M, 0, thumb);
    matrixRowToPoint(M, 1, index);
    matrixRowToPoint(M, 2, middle);
    matrixRowToPoint(M, 3, ring);
    matrixRowToPoint(M, 4, baby);
}


inline std::string toLowerCopy(std::string s)
{
    for (auto &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

inline double angleToRadForIK(double a, const std::string& angle_unit)
{
    const std::string unit = toLowerCopy(angle_unit);
    if (unit == "deg") return a * M_PI / 180.0;
    if (unit == "auto" && std::fabs(a) > 3.5) return a * M_PI / 180.0;
    return a;
}

inline geometry_msgs::msg::Quaternion eigenQuatToMsg(Eigen::Quaterniond q)
{
    geometry_msgs::msg::Quaternion out;
    if (!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
        !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
        q.norm() < 1e-12) {
        out.x = 0.0;
        out.y = 0.0;
        out.z = 0.0;
        out.w = 1.0;
        return out;
    }

    q.normalize();
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
    return out;
}

inline Eigen::Quaterniond msgQuatToEigen(const geometry_msgs::msg::Quaternion& qmsg)
{
    Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
    if (!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
        !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
        q.norm() < 1e-12) {
        return Eigen::Quaterniond::Identity();
    }
    q.normalize();
    return q;
}

inline geometry_msgs::msg::Quaternion eulerToQuatMsg(double ex,
                                                     double ey,
                                                     double ez,
                                                     const std::string& euler_conv,
                                                     const std::string& angle_unit)
{
    const double a0 = angleToRadForIK(ex, angle_unit);
    const double a1 = angleToRadForIK(ey, angle_unit);
    const double a2 = angleToRadForIK(ez, angle_unit);

    Eigen::AngleAxisd Rx(a0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ry(a1, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rz(a2, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q;
    const std::string conv = toLowerCopy(euler_conv);
    if (conv == "zyx" || conv == "rzyx") q = Rz * Ry * Rx;
    else                                 q = Rx * Ry * Rz;  // existing project convention

    return eigenQuatToMsg(q);
}

inline geometry_msgs::msg::Quaternion composeDeltaOrientation(
    const geometry_msgs::msg::Quaternion& base_q_msg,
    const std::array<double,3>& delta_eul,
    const std::string& euler_conv,
    const std::string& angle_unit)
{
    const Eigen::Quaterniond q_base = msgQuatToEigen(base_q_msg);

    const geometry_msgs::msg::Quaternion dq_msg =
        eulerToQuatMsg(delta_eul[0], delta_eul[1], delta_eul[2], euler_conv, angle_unit);
    const Eigen::Quaterniond q_delta = msgQuatToEigen(dq_msg);

    // Local delta convention: target orientation = home orientation * delta.
    // If delta is zero, this exactly preserves the latched home orientation.
    return eigenQuatToMsg(q_base * q_delta);
}

} // namespace

// ============================================================================
// TargetArmPositionCallback (inverse arm IK)
// msg: 12 = [L x y z r p y, R x y z r p y]
// ============================================================================
void DualArmForceControl::TargetArmPositionCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_arm_control_mode_ != "inverse") return;
    if (!msg || msg->data.size() < 12) return;
    if (!arm_ik_l_ || !arm_ik_r_) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");
    const bool is_rby1 = (kin_cfg_.profile.find("rby1") != std::string::npos);

    if (is_rby1) {
        auto finite3 = [](const std::array<double,3>& v) {
            return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
        };

        std::array<double,3> l_xyz_raw{msg->data[0], msg->data[1], msg->data[2]};
        std::array<double,3> l_eul    {msg->data[3], msg->data[4], msg->data[5]};
        std::array<double,3> r_xyz_raw{msg->data[6], msg->data[7], msg->data[8]};
        std::array<double,3> r_eul    {msg->data[9], msg->data[10], msg->data[11]};

        if (!finite3(l_xyz_raw) || !finite3(l_eul) || !finite3(r_xyz_raw) || !finite3(r_eul)) {
            RCLCPP_WARN(logger, "[TargetArmPositionCallback][RBY1] Non-finite input detected. Ignore.");
            return;
        }

        target_pose_l_.position.x = l_xyz_raw[0];
        target_pose_l_.position.y = l_xyz_raw[1];
        target_pose_l_.position.z = l_xyz_raw[2];

        target_pose_r_.position.x = r_xyz_raw[0];
        target_pose_r_.position.y = r_xyz_raw[1];
        target_pose_r_.position.z = r_xyz_raw[2];

        // RBY1 v32: absolute Cartesian target keeps the requested orientation.
        // The ControlLoop now uses pose-constrained DLS, so orientation no longer
        // has to be ignored to avoid the previous strict KDL full-pose failures.
        target_pose_l_.orientation = eulerToQuatMsg(
            l_eul[0], l_eul[1], l_eul[2], ik_euler_conv_, ik_angle_unit_);
        target_pose_r_.orientation = eulerToQuatMsg(
            r_eul[0], r_eul[1], r_eul[2], ik_euler_conv_, ik_angle_unit_);

        rby1_arm_target_active_ = true;

        // v30 fast-teleop patch: generate and publish one command immediately
        // instead of waiting for the next timer tick. This reduces perceived
        // delay for Manus/glove-driven teleoperation.
        ControlLoop();
        return;
    }

    std::array<double,3> l_xyz_raw{msg->data[0], msg->data[1], msg->data[2]};
    std::array<double,3> l_eul    {msg->data[3], msg->data[4], msg->data[5]};
    std::array<double,3> r_xyz_raw{msg->data[6], msg->data[7], msg->data[8]};
    std::array<double,3> r_eul    {msg->data[9], msg->data[10], msg->data[11]};

    auto finite3 = [](const std::array<double,3>& v) {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    };

    if (!finite3(l_xyz_raw) || !finite3(l_eul) || !finite3(r_xyz_raw) || !finite3(r_eul)) {
        RCLCPP_WARN(logger, "[TargetArmPositionCallback] Non-finite input detected. Ignore.");
        return;
    }

    constexpr bool kInputPoseIsWorldFrame = false;
    constexpr double z_offset = 0.306;

    auto toLower = [](std::string s) {
        for (auto &c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        return s;
    };

    const std::string ik_frame = toLower(ik_targets_frame_);
    const bool ik_expects_base =
        (ik_frame == "base" || ik_frame == "robot_base" || ik_frame == "local");

    std::array<double,3> l_xyz_ik = l_xyz_raw;
    std::array<double,3> r_xyz_ik = r_xyz_raw;

    bool l_offset_applied = false;
    bool r_offset_applied = false;

    if (kInputPoseIsWorldFrame && ik_expects_base) {
        l_xyz_ik[2] -= z_offset;
        r_xyz_ik[2] -= z_offset;
        l_offset_applied = true;
        r_offset_applied = true;
    }

    const std::vector<double> ql = eigenToStdVec(q_l_c_);
    const std::vector<double> qr = eigenToStdVec(q_r_c_);

    std::vector<double> rl, rr;
    bool l_ok = arm_ik_l_->solveIK(
        ql, l_xyz_ik, l_eul, ik_targets_frame_, ik_euler_conv_, ik_angle_unit_, rl);

    bool r_ok = arm_ik_r_->solveIK(
        qr, r_xyz_ik, r_eul, ik_targets_frame_, ik_euler_conv_, ik_angle_unit_, rr);

    if (l_ok && rl.size() >= static_cast<size_t>(q_l_t_.size())) {
        for (int i = 0; i < q_l_t_.size(); ++i) q_l_t_(i) = rl[static_cast<size_t>(i)];
    } else {
        RCLCPP_WARN(logger, "[IK][L] solveIK failed or invalid output size. ok=%d size=%zu expected=%d",
                    static_cast<int>(l_ok), rl.size(), static_cast<int>(q_l_t_.size()));
    }

    if (r_ok && rr.size() >= static_cast<size_t>(q_r_t_.size())) {
        for (int i = 0; i < q_r_t_.size(); ++i) q_r_t_(i) = rr[static_cast<size_t>(i)];
    } else {
        RCLCPP_WARN(logger, "[IK][R] solveIK failed or invalid output size. ok=%d size=%zu expected=%d",
                    static_cast<int>(r_ok), rr.size(), static_cast<int>(q_r_t_.size()));
    }

    target_pose_l_.position.x = l_xyz_raw[0];
    target_pose_l_.position.y = l_xyz_raw[1];
    target_pose_l_.position.z = l_xyz_raw[2];

    target_pose_r_.position.x = r_xyz_raw[0];
    target_pose_r_.position.y = r_xyz_raw[1];
    target_pose_r_.position.z = r_xyz_raw[2];

    auto eulToQuat = [&](double ex, double ey, double ez) {
        double a0 = ex, a1 = ey, a2 = ez;
        if (ik_angle_unit_ == "deg") {
            a0 *= M_PI / 180.0;
            a1 *= M_PI / 180.0;
            a2 *= M_PI / 180.0;
        }

        Eigen::AngleAxisd Rx(a0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(a1, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(a2, Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q;
        const std::string conv = toLower(ik_euler_conv_);

        if (conv == "zyx" || conv == "rzyx") q = Rz * Ry * Rx;
        else                                 q = Rx * Ry * Rz;

        geometry_msgs::msg::Quaternion qq;
        qq.x = q.x();
        qq.y = q.y();
        qq.z = q.z();
        qq.w = q.w();
        return qq;
    };

    target_pose_l_.orientation = eulToQuat(l_eul[0], l_eul[1], l_eul[2]);
    target_pose_r_.orientation = eulToQuat(r_eul[0], r_eul[1], r_eul[2]);

    static int dbg_decim = 0;
    const bool do_dbg = ((dbg_decim++ % 20) == 0);

    if (do_dbg) {
        // RCLCPP_INFO(logger,
        //     "[TargetArmPosCb] arm_mode=inverse frame=%s euler_conv=%s angle_unit=%s z_offset=%.4f input_is_world=%d",
        //     ik_targets_frame_.c_str(), ik_euler_conv_.c_str(), ik_angle_unit_.c_str(),
        //     z_offset, static_cast<int>(kInputPoseIsWorldFrame));

        // RCLCPP_INFO(logger,
        //     "[L] raw xyz=(%.4f %.4f %.4f) -> ik xyz=(%.4f %.4f %.4f) offset=%d | eul=(%.3f %.3f %.3f) | ok=%d",
        //     l_xyz_raw[0], l_xyz_raw[1], l_xyz_raw[2],
        //     l_xyz_ik[0],  l_xyz_ik[1],  l_xyz_ik[2],
        //     static_cast<int>(l_offset_applied),
        //     l_eul[0], l_eul[1], l_eul[2],
        //     static_cast<int>(l_ok && rl.size() >= 6));

        // RCLCPP_INFO(logger,
        //     "[R] raw xyz=(%.4f %.4f %.4f) -> ik xyz=(%.4f %.4f %.4f) offset=%d | eul=(%.3f %.3f %.3f) | ok=%d",
        //     r_xyz_raw[0], r_xyz_raw[1], r_xyz_raw[2],
        //     r_xyz_ik[0],  r_xyz_ik[1],  r_xyz_ik[2],
        //     static_cast<int>(r_offset_applied),
        //     r_eul[0], r_eul[1], r_eul[2],
        //     static_cast<int>(r_ok && rr.size() >= 6));

        // if (l_ok && rl.size() >= 6) {
        //     RCLCPP_INFO(logger,
        //         "[IK Q L] q_t=[%.3f %.3f %.3f %.3f %.3f %.3f]",
        //         q_l_t_(0), q_l_t_(1), q_l_t_(2), q_l_t_(3), q_l_t_(4), q_l_t_(5));
        // }
        // if (r_ok && rr.size() >= 6) {
        //     RCLCPP_INFO(logger,
        //         "[IK Q R] q_t=[%.3f %.3f %.3f %.3f %.3f %.3f]",
        //         q_r_t_(0), q_r_t_(1), q_r_t_(2), q_r_t_(3), q_r_t_(4), q_r_t_(5));
        // }
    }
}

// ============================================================================
// TargetHandPositionCallback (inverse hand target store-only)
//   v25-style patch:
//   - DOES NOT run IK here
//   - stores Cartesian fingertip target x_d_*
//   - final q_cmd is built in ControlLoop via
//       x_d, x_0, F_d, F_ext -> admittance -> x_cmd -> IK -> q_cmd
// ============================================================================

void DualArmForceControl::TargetHandPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!hand_runtime_enabled_) return;
    if (current_hand_control_mode_ != "inverse") return;
    if (!msg) return;
    if (msg->data.size() < 30) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    // left 5 fingertips
    for (int i = 0; i < 5; ++i) {
        const int b = i * 3;
        const Eigen::Vector3d xd(msg->data[b + 0], msg->data[b + 1], msg->data[b + 2]);
        if (!isFiniteVec3(xd)) {
            RCLCPP_WARN(logger, "[TargetHandPositionCallback] Non-finite left target at finger=%d", i);
            return;
        }
        writeMatrixRow(x_l_hand_d_, i, xd);
    }

    // right 5 fingertips
    for (int i = 0; i < 5; ++i) {
        const int b = 15 + i * 3;
        const Eigen::Vector3d xd(msg->data[b + 0], msg->data[b + 1], msg->data[b + 2]);
        if (!isFiniteVec3(xd)) {
            RCLCPP_WARN(logger, "[TargetHandPositionCallback] Non-finite right target at finger=%d", i);
            return;
        }
        writeMatrixRow(x_r_hand_d_, i, xd);
    }

    hand_cartesian_target_l_initialized_ = true;
    hand_cartesian_target_r_initialized_ = true;

    // monitor target points
    updatePointSetFromMatrix(
        x_l_hand_d_,
        t_f_l_thumb_, t_f_l_index_, t_f_l_middle_, t_f_l_ring_, t_f_l_baby_);

    updatePointSetFromMatrix(
        x_r_hand_d_,
        t_f_r_thumb_, t_f_r_index_, t_f_r_middle_, t_f_r_ring_, t_f_r_baby_);

    static int dbg_decim = 0;
    const bool do_dbg = ((dbg_decim++ % 20) == 0);

    if (do_dbg) {
        // RCLCPP_INFO(logger,
        //     "[TargetHandPosCb] hand_mode=inverse | store-only Cartesian target updated "
        //     "| L_ring=(%.4f %.4f %.4f) R_ring=(%.4f %.4f %.4f)",
        //     x_l_hand_d_(3,0), x_l_hand_d_(3,1), x_l_hand_d_(3,2),
        //     x_r_hand_d_(3,0), x_r_hand_d_(3,1), x_r_hand_d_(3,2));
    }
}

// ============================================================================
// DeltaArmPositionCallback
// ============================================================================
void DualArmForceControl::DeltaArmPositionCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_arm_control_mode_ != "inverse") return;
    if (!msg || msg->data.size() < 12) return;
    if (!arm_ik_l_ || !arm_ik_r_) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");
    const bool is_rby1 = (kin_cfg_.profile.find("rby1") != std::string::npos);

    if (is_rby1) {
        auto finite3 = [](const std::array<double,3>& v) {
            return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
        };

        // RBY1 delta command must be interpreted as:
        //   target = latched_home_pose + delta
        // not as an absolute Cartesian target.
        std::array<double,3> l_dxyz{msg->data[0],  msg->data[1],  msg->data[2]};
        std::array<double,3> l_deul{msg->data[3],  msg->data[4],  msg->data[5]};
        std::array<double,3> r_dxyz{msg->data[6],  msg->data[7],  msg->data[8]};
        std::array<double,3> r_deul{msg->data[9],  msg->data[10], msg->data[11]};

        if (!finite3(l_dxyz) || !finite3(l_deul) || !finite3(r_dxyz) || !finite3(r_deul)) {
            RCLCPP_WARN(logger, "[DeltaArmPositionCallback][RBY1] Non-finite delta input detected. Ignore.");
            return;
        }

        if (!delta_arm_base_pose_initialized_) {
            RCLCPP_WARN(logger, "[DeltaArmPositionCallback][RBY1] latched home/base pose is not initialized yet.");
            return;
        }

        target_pose_l_ = delta_arm_base_pose_l_;
        target_pose_r_ = delta_arm_base_pose_r_;

        target_pose_l_.position.x = delta_arm_base_pose_l_.position.x + l_dxyz[0];
        target_pose_l_.position.y = delta_arm_base_pose_l_.position.y + l_dxyz[1];
        target_pose_l_.position.z = delta_arm_base_pose_l_.position.z + l_dxyz[2];

        target_pose_r_.position.x = delta_arm_base_pose_r_.position.x + r_dxyz[0];
        target_pose_r_.position.y = delta_arm_base_pose_r_.position.y + r_dxyz[1];
        target_pose_r_.position.z = delta_arm_base_pose_r_.position.z + r_dxyz[2];

        // RBY1 v32:
        // Preserve latched home orientation when droll/dpitch/dyaw are zero.
        // If orientation delta is nonzero, apply it relative to the home pose.
        // This means xyz-only delta commands move position without letting
        // the end-effector orientation drift in the redundant 7-DOF null-space.
        target_pose_l_.orientation = composeDeltaOrientation(
            delta_arm_base_pose_l_.orientation, l_deul, ik_euler_conv_, ik_angle_unit_);
        target_pose_r_.orientation = composeDeltaOrientation(
            delta_arm_base_pose_r_.orientation, r_deul, ik_euler_conv_, ik_angle_unit_);

        rby1_arm_target_active_ = true;

        static int rby1_delta_dbg_decim = 0;
        if ((rby1_delta_dbg_decim++ % 20) == 0) {
            RCLCPP_INFO(
                logger,
                "[DeltaArmPositionCallback][RBY1] home+delta pose target | "
                "L home=(%.4f %.4f %.4f) d=(%.4f %.4f %.4f) tgt=(%.4f %.4f %.4f) deul=(%.3f %.3f %.3f) | "
                "R home=(%.4f %.4f %.4f) d=(%.4f %.4f %.4f) tgt=(%.4f %.4f %.4f) deul=(%.3f %.3f %.3f)",
                delta_arm_base_pose_l_.position.x,
                delta_arm_base_pose_l_.position.y,
                delta_arm_base_pose_l_.position.z,
                l_dxyz[0], l_dxyz[1], l_dxyz[2],
                target_pose_l_.position.x,
                target_pose_l_.position.y,
                target_pose_l_.position.z,
                l_deul[0], l_deul[1], l_deul[2],
                delta_arm_base_pose_r_.position.x,
                delta_arm_base_pose_r_.position.y,
                delta_arm_base_pose_r_.position.z,
                r_dxyz[0], r_dxyz[1], r_dxyz[2],
                target_pose_r_.position.x,
                target_pose_r_.position.y,
                target_pose_r_.position.z,
                r_deul[0], r_deul[1], r_deul[2]);
        }

        // v30 fast-teleop patch: publish a first resolved-rate step immediately
        // on every delta target packet. The periodic ControlLoop continues the
        // motion afterward.
        ControlLoop();
        return;
    }

    auto finite3 = [](const std::array<double,3>& v) {
        return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
    };

    std::array<double,3> l_dxyz{msg->data[0],  msg->data[1],  msg->data[2]};
    std::array<double,3> l_deul{msg->data[3],  msg->data[4],  msg->data[5]};
    std::array<double,3> r_dxyz{msg->data[6],  msg->data[7],  msg->data[8]};
    std::array<double,3> r_deul{msg->data[9],  msg->data[10], msg->data[11]};

    if (!finite3(l_dxyz) || !finite3(l_deul) || !finite3(r_dxyz) || !finite3(r_deul)) {
        RCLCPP_WARN(logger, "[DeltaArmPositionCallback] Non-finite delta input detected. Ignore.");
        return;
    }

    if (!delta_arm_base_pose_initialized_) {
        RCLCPP_WARN(logger, "[DeltaArmPositionCallback] base pose is not initialized yet.");
        return;
    }

    std::array<double,3> l_xyz_base{
        delta_arm_base_pose_l_.position.x,
        delta_arm_base_pose_l_.position.y,
        delta_arm_base_pose_l_.position.z
    };
    std::array<double,3> r_xyz_base{
        delta_arm_base_pose_r_.position.x,
        delta_arm_base_pose_r_.position.y,
        delta_arm_base_pose_r_.position.z
    };

    double l_ex_deg = 0.0, l_ey_deg = 0.0, l_ez_deg = 0.0;
    double r_ex_deg = 0.0, r_ey_deg = 0.0, r_ez_deg = 0.0;

    ArmForwardKinematics::quatToEulerXYZDeg_Isaac(
        delta_arm_base_pose_l_.orientation, l_ex_deg, l_ey_deg, l_ez_deg);
    ArmForwardKinematics::quatToEulerXYZDeg_Isaac(
        delta_arm_base_pose_r_.orientation, r_ex_deg, r_ey_deg, r_ez_deg);

    auto deg_to_current_unit = [&](double deg_val)->double {
        if (ik_angle_unit_ == "deg") return deg_val;
        return deg_val * M_PI / 180.0;
    };

    std::array<double,3> l_eul_base{
        deg_to_current_unit(l_ex_deg),
        deg_to_current_unit(l_ey_deg),
        deg_to_current_unit(l_ez_deg)
    };
    std::array<double,3> r_eul_base{
        deg_to_current_unit(r_ex_deg),
        deg_to_current_unit(r_ey_deg),
        deg_to_current_unit(r_ez_deg)
    };

    std_msgs::msg::Float64MultiArray abs_msg;
    abs_msg.data.resize(12);

    abs_msg.data[0] = l_xyz_base[0] + l_dxyz[0];
    abs_msg.data[1] = l_xyz_base[1] + l_dxyz[1];
    abs_msg.data[2] = l_xyz_base[2] + l_dxyz[2];
    abs_msg.data[3] = l_eul_base[0] + l_deul[0];
    abs_msg.data[4] = l_eul_base[1] + l_deul[1];
    abs_msg.data[5] = l_eul_base[2] + l_deul[2];

    abs_msg.data[6]  = r_xyz_base[0] + r_dxyz[0];
    abs_msg.data[7]  = r_xyz_base[1] + r_dxyz[1];
    abs_msg.data[8]  = r_xyz_base[2] + r_dxyz[2];
    abs_msg.data[9]  = r_eul_base[0] + r_deul[0];
    abs_msg.data[10] = r_eul_base[1] + r_deul[1];
    abs_msg.data[11] = r_eul_base[2] + r_deul[2];

    static int dbg_decim = 0;
    const bool do_dbg = ((dbg_decim++ % 20) == 0);

    if (do_dbg) {
        // RCLCPP_INFO(logger,
        //     "[DeltaArmPosCb] arm_mode=inverse angle_unit=%s euler_conv=%s | delta interpreted w.r.t latched initial pose",
        //     ik_angle_unit_.c_str(), ik_euler_conv_.c_str());

        // RCLCPP_INFO(logger,
        //     "[L] base xyz=(%.4f %.4f %.4f) + dxyz=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
        //     l_xyz_base[0], l_xyz_base[1], l_xyz_base[2],
        //     l_dxyz[0], l_dxyz[1], l_dxyz[2],
        //     abs_msg.data[0], abs_msg.data[1], abs_msg.data[2]);

        // RCLCPP_INFO(logger,
        //     "[L] base eul=(%.4f %.4f %.4f) + deul=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
        //     l_eul_base[0], l_eul_base[1], l_eul_base[2],
        //     l_deul[0], l_deul[1], l_deul[2],
        //     abs_msg.data[3], abs_msg.data[4], abs_msg.data[5]);

        // RCLCPP_INFO(logger,
        //     "[R] base xyz=(%.4f %.4f %.4f) + dxyz=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
        //     r_xyz_base[0], r_xyz_base[1], r_xyz_base[2],
        //     r_dxyz[0], r_dxyz[1], r_dxyz[2],
        //     abs_msg.data[6], abs_msg.data[7], abs_msg.data[8]);

        // RCLCPP_INFO(logger,
        //     "[R] base eul=(%.4f %.4f %.4f) + deul=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
        //     r_eul_base[0], r_eul_base[1], r_eul_base[2],
        //     r_deul[0], r_deul[1], r_deul[2],
        //     abs_msg.data[9], abs_msg.data[10], abs_msg.data[11]);
    }

    auto abs_msg_ptr = std::make_shared<std_msgs::msg::Float64MultiArray>(abs_msg);
    TargetArmPositionCallback(abs_msg_ptr);
    ControlLoop();
}

// ============================================================================
// DeltaHandPositionCallback
//   v25-style patch:
//   - inverse utility
//   - builds absolute Cartesian fingertip target and forwards to
//     TargetHandPositionCallback()
// ============================================================================
void DualArmForceControl::DeltaHandPositionCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!hand_runtime_enabled_) return;
    if (current_hand_control_mode_ != "inverse") return;
    if (!msg || msg->data.size() < 5) return;
    if (!hand_fk_l_ || !hand_fk_r_) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    const double side_d   = msg->data[0];
    const double finger_d = msg->data[1];

    if (!std::isfinite(side_d) || !std::isfinite(finger_d)) {
        RCLCPP_WARN(logger, "[DeltaHandPositionCallback] Non-finite side/finger input. Ignore.");
        return;
    }

    const int side   = static_cast<int>(std::llround(side_d));
    const int finger = static_cast<int>(std::llround(finger_d));

    if (!(side == 0 || side == 1)) {
        RCLCPP_WARN(logger, "[DeltaHandPositionCallback] invalid side=%d (use 0:left, 1:right)", side);
        return;
    }
    if (finger < 0 || finger > 4) {
        RCLCPP_WARN(logger, "[DeltaHandPositionCallback] invalid finger=%d (use 0..4)", finger);
        return;
    }

    const Eigen::Vector3d dxyz(msg->data[2], msg->data[3], msg->data[4]);
    if (!isFiniteVec3(dxyz)) {
        RCLCPP_WARN(logger, "[DeltaHandPositionCallback] Non-finite delta xyz input. Ignore.");
        return;
    }

    std::array<Eigen::Vector3d,5> cur_l;
    std::array<Eigen::Vector3d,5> cur_r;

    cur_l[0] = pointToVec(f_l_thumb_);
    cur_l[1] = pointToVec(f_l_index_);
    cur_l[2] = pointToVec(f_l_middle_);
    cur_l[3] = pointToVec(f_l_ring_);
    cur_l[4] = pointToVec(f_l_baby_);

    cur_r[0] = pointToVec(f_r_thumb_);
    cur_r[1] = pointToVec(f_r_index_);
    cur_r[2] = pointToVec(f_r_middle_);
    cur_r[3] = pointToVec(f_r_ring_);
    cur_r[4] = pointToVec(f_r_baby_);

    std::array<Eigen::Vector3d,5> tgt_l = cur_l;
    std::array<Eigen::Vector3d,5> tgt_r = cur_r;

    if (side == 0) tgt_l[static_cast<std::size_t>(finger)] += dxyz;
    else           tgt_r[static_cast<std::size_t>(finger)] += dxyz;

    std_msgs::msg::Float64MultiArray abs_msg;
    abs_msg.data.resize(30, 0.0);

    for (int i = 0; i < 5; ++i) {
        const int b = i * 3;
        abs_msg.data[b + 0] = tgt_l[i].x();
        abs_msg.data[b + 1] = tgt_l[i].y();
        abs_msg.data[b + 2] = tgt_l[i].z();
    }

    for (int i = 0; i < 5; ++i) {
        const int b = 15 + i * 3;
        abs_msg.data[b + 0] = tgt_r[i].x();
        abs_msg.data[b + 1] = tgt_r[i].y();
        abs_msg.data[b + 2] = tgt_r[i].z();
    }

    static int dbg_decim = 0;
    const bool do_dbg = ((dbg_decim++ % 20) == 0);

    if (do_dbg) {
        const Eigen::Vector3d p_cur = (side == 0) ? cur_l[static_cast<std::size_t>(finger)]
                                                  : cur_r[static_cast<std::size_t>(finger)];
        const Eigen::Vector3d p_tgt = (side == 0) ? tgt_l[static_cast<std::size_t>(finger)]
                                                  : tgt_r[static_cast<std::size_t>(finger)];

        // RCLCPP_INFO(logger,
        //     "[DeltaHandPosCb] hand_mode=inverse side=%s finger=%d | cur=(%.4f %.4f %.4f) + d=(%.4f %.4f %.4f) -> tgt=(%.4f %.4f %.4f)",
        //     (side == 0 ? "L" : "R"),
        //     finger,
        //     p_cur.x(), p_cur.y(), p_cur.z(),
        //     dxyz.x(), dxyz.y(), dxyz.z(),
        //     p_tgt.x(), p_tgt.y(), p_tgt.z());
    }

    auto abs_msg_ptr = std::make_shared<std_msgs::msg::Float64MultiArray>(abs_msg);
    TargetHandPositionCallback(abs_msg_ptr);
}

// --------------------
// TargetArmJointsCallback (forward)
// --------------------
void DualArmForceControl::TargetArmJointsCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (current_arm_control_mode_ != "forward") return;
    if (!msg) return;

    const size_t l_dof = static_cast<size_t>(q_l_t_.size());
    const size_t r_dof = static_cast<size_t>(q_r_t_.size());
    if (msg->data.size() < l_dof + r_dof) {
        auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");
        RCLCPP_WARN(logger,
                    "[TargetArmJointsCallback] expected %zu values (=L%zu+R%zu), got %zu",
                    l_dof + r_dof, l_dof, r_dof, msg->data.size());
        return;
    }

    for (int i = 0; i < q_l_t_.size(); ++i) {
        q_l_t_(i) = msg->data[static_cast<size_t>(i)];
    }
    for (int i = 0; i < q_r_t_.size(); ++i) {
        q_r_t_(i) = msg->data[l_dof + static_cast<size_t>(i)];
    }

    ControlLoop();
}

// --------------------
// TargetHandJointsCallback (forward)
//   v25-style patch:
//   - writes q_h_motion_t_ (authoritative forward input)
//   - also updates Cartesian target cache x_hand_d_ via FK
//   - final q_cmd(q_h_t_) is built later in ControlLoop
// --------------------
void DualArmForceControl::TargetHandJointsCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // v31 admittance rollback patch:
    // The forward topic writes only q_h_motion_t_. When hand_runtime_enabled_ is
    // true, ControlLoop runs the old v25-style unified hand pipeline:
    //   q_h_motion_t_ -> FK -> x_d -> admittance/IK -> q_h_t_.
    // If hand_runtime_enabled_ is false, ControlLoop falls back to safe joint
    // pass-through for forward mode.
    if (current_hand_control_mode_ != "forward") return;
    if (!msg) return;

    const size_t n = msg->data.size();
    const bool is_rby1 = (kin_cfg_.profile.find("rby1") != std::string::npos);
    const bool enforce_mimic_for_this_profile = !is_rby1;

    auto enforce_mimic_q4_eq_q3 = [&](Eigen::VectorXd& qh20) {
        if (qh20.size() < 20) return;
        for (int f = 0; f < 5; ++f) {
            const int b = f * 4;
            qh20(b + 3) = qh20(b + 2);
        }
    };

    auto assign_hand15_to_qh20 = [&](Eigen::VectorXd& qh20, const size_t offset) {
        if (qh20.size() < 20) return;
        for (int f = 0; f < 5; ++f) {
            const int b15 = f * 3;
            const int b20 = f * 4;

            qh20(b20 + 0) = msg->data[offset + b15 + 0];
            qh20(b20 + 1) = msg->data[offset + b15 + 1];
            qh20(b20 + 2) = msg->data[offset + b15 + 2];
            qh20(b20 + 3) = msg->data[offset + b15 + 2];
        }
    };

    auto assign_hand20_to_qh20 = [&](Eigen::VectorXd& qh20, const size_t offset) {
        if (qh20.size() < 20) return;
        for (int i = 0; i < 20; ++i) qh20(i) = msg->data[offset + i];
        if (enforce_mimic_for_this_profile) enforce_mimic_q4_eq_q3(qh20);
    };

    // NOTE:
    // check larger legacy packet sizes first
    if (n >= 52) {
        assign_hand20_to_qh20(q_l_h_motion_t_, 12);
        assign_hand20_to_qh20(q_r_h_motion_t_, 32);
    }
    else if (n >= 42) {
        assign_hand15_to_qh20(q_l_h_motion_t_, 12);
        assign_hand15_to_qh20(q_r_h_motion_t_, 27);
    }
    else if (n >= 40) {
        assign_hand20_to_qh20(q_l_h_motion_t_, 0);
        assign_hand20_to_qh20(q_r_h_motion_t_, 20);
    }
    else if (n >= 30) {
        assign_hand15_to_qh20(q_l_h_motion_t_, 0);
        assign_hand15_to_qh20(q_r_h_motion_t_, 15);
    }
    else {
        return;
    }

    // Build Cartesian target cache x_d from motion target via FK
    if (hand_fk_l_ && hand_fk_r_) {
        const std::vector<double> ql15 = compress20to15(q_l_h_motion_t_);
        const std::vector<double> qr15 = compress20to15(q_r_h_motion_t_);

        const std::vector<Eigen::Vector3d> tips_l = hand_fk_l_->computeFingertips(ql15);
        const std::vector<Eigen::Vector3d> tips_r = hand_fk_r_->computeFingertips(qr15);

        for (int i = 0; i < 5; ++i) {
            writeMatrixRow(x_l_hand_d_, i, safeGetTip(tips_l, i));
            writeMatrixRow(x_r_hand_d_, i, safeGetTip(tips_r, i));
        }

        hand_cartesian_target_l_initialized_ = true;
        hand_cartesian_target_r_initialized_ = true;

        updatePointSetFromMatrix(
            x_l_hand_d_,
            t_f_l_thumb_, t_f_l_index_, t_f_l_middle_, t_f_l_ring_, t_f_l_baby_);

        updatePointSetFromMatrix(
            x_r_hand_d_,
            t_f_r_thumb_, t_f_r_index_, t_f_r_middle_, t_f_r_ring_, t_f_r_baby_);
    }

    // Keep the old v25 semantics: the callback only updates the motion
    // reference. The final q_l_h_t_ / q_r_h_t_ command is generated by
    // ControlLoop through the unified hand admittance pipeline. If the hand
    // runtime is unavailable, ControlLoop will use safe forward pass-through.
    ControlLoop();
}

// ============================================================================
// TargetAuxJointsCallback
// msg: JointState command for RBY1 non-arm joints.
//   - name + position controls torso_* joints as absolute position targets.
//   - name + velocity controls left_wheel/right_wheel as velocity targets.
// ============================================================================
void DualArmForceControl::TargetAuxJointsCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (!msg) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    auto starts_with = [](const std::string& s, const std::string& prefix) -> bool {
        return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
    };
    auto is_wheel = [](const std::string& n) -> bool {
        return n == "left_wheel" || n == "right_wheel";
    };
    auto is_torso = [&](const std::string& n) -> bool {
        return starts_with(n, "torso_");
    };

    const std::size_t n_pos = std::min(msg->name.size(), msg->position.size());
    for (std::size_t i = 0; i < n_pos; ++i) {
        const std::string& n = msg->name[i];
        const double p = msg->position[i];
        if (!std::isfinite(p)) continue;

        if (is_torso(n)) {
            aux_joint_position_command_[n] = p;
        }
    }

    bool got_wheel_velocity = false;
    const std::size_t n_vel = std::min(msg->name.size(), msg->velocity.size());
    for (std::size_t i = 0; i < n_vel; ++i) {
        const std::string& n = msg->name[i];
        const double v = msg->velocity[i];
        if (!std::isfinite(v)) continue;

        if (is_wheel(n)) {
            aux_joint_velocity_command_[n] = v;
            got_wheel_velocity = true;
        }
    }

    if (got_wheel_velocity) {
        aux_joint_velocity_stamp_ = node_->now();
    }

    if (n_pos == 0 && n_vel == 0) {
        RCLCPP_WARN(logger, "[TargetAuxJointsCallback] Empty position/velocity command. Ignore.");
        return;
    }

    ControlLoop();
}

// ============================================================================
// TargetBaseVelocityCallback
// msg: RBY1 mobile-base velocity command.
//   - linear.x  [m/s]
//   - angular.z [rad/s]
// Converted to wheel angular velocities and published by ControlLoop together
// with the normal full-body hold/arm/hand command stream.
// ============================================================================
void DualArmForceControl::TargetBaseVelocityCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!msg) return;

    const double linear_x = msg->linear.x;
    const double angular_z = msg->angular.z;

    if (!std::isfinite(linear_x) || !std::isfinite(angular_z)) {
        auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");
        RCLCPP_WARN(logger, "[TargetBaseVelocityCallback] Non-finite /cmd_vel. Ignore.");
        return;
    }

    cmd_vel_linear_x_ = linear_x;
    cmd_vel_angular_z_ = angular_z;
    cmd_vel_stamp_ = node_->now();

    ControlLoop();
}

// ============================================================================
// TargetHandForceCallback
//   v25-style patch:
//   - no separate forcecon mode
//   - valid in hand forward / inverse
//   - authoritative desired-force state:
//       f_l_hand_t_ / f_r_hand_t_
//   - legacy single-active-finger latch kept temporarily for migration/debug
// ============================================================================
void DualArmForceControl::TargetHandForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!hand_runtime_enabled_) {
        auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");
        RCLCPP_WARN(logger, "[TargetHandForceCallback] hand runtime FK/IK is disabled; cannot run admittance.");
        return;
    }
    if (current_hand_control_mode_ == "idle") return;
    if (!msg) return;
    if (msg->data.size() < 5) return;

    auto logger = node_ ? node_->get_logger() : rclcpp::get_logger("dualarm_forcecon");

    const double hand_id_d   = msg->data[0];
    const double finger_id_d = msg->data[1];

    if (!std::isfinite(hand_id_d) || !std::isfinite(finger_id_d)) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] non-finite hand_id/finger_id");
        return;
    }

    const int hand_id   = static_cast<int>(std::llround(hand_id_d));
    const int finger_id = static_cast<int>(std::llround(finger_id_d));

    if (!(hand_id == 0 || hand_id == 1)) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] invalid hand_id=%d (use 0:left, 1:right)", hand_id);
        return;
    }
    if (finger_id < 0 || finger_id > 4) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] invalid finger_id=%d (use 0..4)", finger_id);
        return;
    }

    Eigen::Vector3d f_des_base = Eigen::Vector3d::Zero();
    if (msg->data.size() >= 8) {
        // legacy format: [hand_id, finger_id, px, py, pz, fx, fy, fz]
        f_des_base = Eigen::Vector3d(msg->data[5], msg->data[6], msg->data[7]);
    } else {
        // compact format: [hand_id, finger_id, fx, fy, fz]
        f_des_base = Eigen::Vector3d(msg->data[2], msg->data[3], msg->data[4]);
    }

    if (!isFiniteVec3(f_des_base)) {
        RCLCPP_WARN(logger, "[TargetHandForceCallback] non-finite desired force");
        return;
    }

    const bool is_left = (hand_id == 0);

    // single-command semantics 유지:
    // current command packet defines one active fingertip target force.
    f_l_hand_t_.setZero();
    f_r_hand_t_.setZero();

    if (is_left) writeMatrixRow(f_l_hand_t_, finger_id, f_des_base);
    else         writeMatrixRow(f_r_hand_t_, finger_id, f_des_base);

    // legacy latch kept temporarily for downstream migration / debug
    hand_force_cmd_valid_      = true;
    hand_force_cmd_hand_id_    = hand_id;
    hand_force_cmd_finger_id_  = finger_id;
    hand_force_cmd_f_des_base_ = f_des_base;
    hand_force_cmd_stamp_ns_   = node_ ? node_->get_clock()->now().nanoseconds() : 0;

    // reset only the targeted admittance controller state so the next control
    // loop iteration starts from the current fingertip pose cleanly.
    auto& adm_arr = is_left ? hand_adm_l_ : hand_adm_r_;
    if (adm_arr[static_cast<std::size_t>(finger_id)]) {
        Eigen::Vector3d p_init = Eigen::Vector3d::Zero();
        if (is_left) {
            switch (finger_id) {
                case 0: p_init = pointToVec(f_l_thumb_);  break;
                case 1: p_init = pointToVec(f_l_index_);  break;
                case 2: p_init = pointToVec(f_l_middle_); break;
                case 3: p_init = pointToVec(f_l_ring_);   break;
                case 4: p_init = pointToVec(f_l_baby_);   break;
            }
        } else {
            switch (finger_id) {
                case 0: p_init = pointToVec(f_r_thumb_);  break;
                case 1: p_init = pointToVec(f_r_index_);  break;
                case 2: p_init = pointToVec(f_r_middle_); break;
                case 3: p_init = pointToVec(f_r_ring_);   break;
                case 4: p_init = pointToVec(f_r_baby_);   break;
            }
        }
        adm_arr[static_cast<std::size_t>(finger_id)]->resetState(p_init);
    }

    static int dbg_decim = 0;
    if ((dbg_decim++ % 20) == 0) {
        // RCLCPP_INFO(logger,
        //     "[TargetHandForceCb][store-only] hand_mode=%s side=%s finger=%d | "
        //     "f_des=(%.3f %.3f %.3f)",
        //     current_hand_control_mode_.c_str(),
        //     is_left ? "L" : "R",
        //     finger_id,
        //     f_des_base.x(), f_des_base.y(), f_des_base.z());
    }
}
