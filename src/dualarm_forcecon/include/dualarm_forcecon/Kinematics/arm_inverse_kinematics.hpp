#ifndef ARM_INVERSE_KINEMATICS_HPP
#define ARM_INVERSE_KINEMATICS_HPP

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <vector>
#include <iostream>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>

class ArmInverseKinematics {
public:
    enum class AngleUnit { RAD, DEG, AUTO };
    enum class EulerConv { RPY_ZYX, XYZ }; // RPY_ZYX: R = Rz(yaw)*Ry(pitch)*Rx(roll),  XYZ: R = Rx*Ry*Rz

    ArmInverseKinematics(const std::string& urdf_path,
                         const std::string& base_link,
                         const std::string& tip_link)
    {
        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(urdf_path, tree)) {
            std::cerr << "[IK Error] Failed to construct KDL tree from: " << urdf_path << std::endl;
            ok_ = false;
            return;
        }

        if (!tree.getChain(base_link, tip_link, chain_)) {
            std::cerr << "[IK Error] Failed to get chain from " << base_link << " to " << tip_link << std::endl;
            ok_ = false;
            return;
        }

        num_joints_ = chain_.getNrOfJoints();
        fk_solver_  = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
        vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);

        // 기본 joint limit: [-pi, pi] (필요시 URDF limit로 교체 가능)
        KDL::JntArray q_min(num_joints_), q_max(num_joints_);
        for (unsigned int i = 0; i < num_joints_; i++) {
            q_min(i) = -3.141592653589793;
            q_max(i) =  3.141592653589793;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(
            chain_, q_min, q_max, *fk_solver_, *vel_solver_, 100, 1e-6
        );

        ok_ = true;
    }

    bool isOk() const { return ok_; }

    // ------------------------------------------------------------
    // (호환 유지) 기존 방식: target_rpy 는 "rad + RPY" 로 들어온다고 가정
    // ------------------------------------------------------------
    bool solveIK(const std::vector<double>& current_q,
                 const double target_xyz[3],
                 const double target_rpy[3],
                 std::vector<double>& result_q)
    {
        if (!ok_) return false;
        KDL::Frame target;
        target.p = KDL::Vector(target_xyz[0], target_xyz[1], target_xyz[2]);
        target.M = KDL::Rotation::RPY(target_rpy[0], target_rpy[1], target_rpy[2]);
        return solveIKFrame(current_q, target, result_q);
    }

    // ------------------------------------------------------------
    // 확장: Euler 규약/단위 선택 가능 (base frame 기준)
    // ------------------------------------------------------------
    bool solveIKWithEuler(const std::vector<double>& current_q,
                          const double target_xyz[3],
                          const double euler[3],
                          std::vector<double>& result_q,
                          EulerConv conv,
                          AngleUnit unit)
    {
        if (!ok_) return false;

        double a0 = euler[0], a1 = euler[1], a2 = euler[2];
        auto [r0, r1, r2] = normalizeAnglesToRad(a0, a1, a2, unit);

        Eigen::Matrix3d R = eulerToRot(r0, r1, r2, conv);

        KDL::Frame target;
        target.p = KDL::Vector(target_xyz[0], target_xyz[1], target_xyz[2]);
        target.M = eigenToKDLRot(R);
        return solveIKFrame(current_q, target, result_q);
    }

    // ------------------------------------------------------------
    // 확장: world frame 입력을 base frame으로 변환해서 IK
    // world_T_base 가 주어져야 함
    // ------------------------------------------------------------
    bool solveIKWorld(const std::vector<double>& current_q,
                      const Eigen::Affine3d& world_T_base,
                      const double target_xyz_world[3],
                      const double euler_world[3],
                      std::vector<double>& result_q,
                      EulerConv conv,
                      AngleUnit unit)
    {
        if (!ok_) return false;

        double a0 = euler_world[0], a1 = euler_world[1], a2 = euler_world[2];
        auto [r0, r1, r2] = normalizeAnglesToRad(a0, a1, a2, unit);

        Eigen::Matrix3d Rw = eulerToRot(r0, r1, r2, conv);

        Eigen::Affine3d world_T_target = Eigen::Affine3d::Identity();
        world_T_target.linear() = Rw;
        world_T_target.translation() = Eigen::Vector3d(target_xyz_world[0],
                                                       target_xyz_world[1],
                                                       target_xyz_world[2]);

        Eigen::Affine3d base_T_target = world_T_base.inverse() * world_T_target;

        KDL::Frame target;
        target.p = KDL::Vector(base_T_target.translation().x(),
                               base_T_target.translation().y(),
                               base_T_target.translation().z());
        target.M = eigenToKDLRot(base_T_target.linear());

        return solveIKFrame(current_q, target, result_q);
    }

private:
    bool ok_{false};

    KDL::Chain chain_;
    unsigned int num_joints_{0};

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::shared_ptr<KDL::ChainIkSolverVel_pinv>      vel_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR_JL>     ik_solver_;

    static double deg2rad(double d) { return d * M_PI / 180.0; }

    static std::tuple<double,double,double> normalizeAnglesToRad(double a0, double a1, double a2, AngleUnit unit)
    {
        if (unit == AngleUnit::RAD) {
            return {a0, a1, a2};
        } else if (unit == AngleUnit::DEG) {
            return {deg2rad(a0), deg2rad(a1), deg2rad(a2)};
        } else { // AUTO
            // 하나라도 abs>3.2면 deg로 간주(160deg 같은 케이스)
            if (std::fabs(a0) > 3.2 || std::fabs(a1) > 3.2 || std::fabs(a2) > 3.2) {
                return {deg2rad(a0), deg2rad(a1), deg2rad(a2)};
            }
            return {a0, a1, a2};
        }
    }

    static Eigen::Matrix3d eulerToRot(double a0_rad, double a1_rad, double a2_rad, EulerConv conv)
    {
        Eigen::AngleAxisd ax(a0_rad, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ay(a1_rad, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd az(a2_rad, Eigen::Vector3d::UnitZ());

        if (conv == EulerConv::XYZ) {
            // Isaac UI rotateXYZ: R = Rx * Ry * Rz
            return (ax * ay * az).toRotationMatrix();
        } else {
            // RPY (ZYX): roll=X, pitch=Y, yaw=Z => R = Rz(yaw) * Ry(pitch) * Rx(roll)
            return (az * ay * ax).toRotationMatrix();
        }
    }

    static KDL::Rotation eigenToKDLRot(const Eigen::Matrix3d& R)
    {
        return KDL::Rotation(
            R(0,0), R(0,1), R(0,2),
            R(1,0), R(1,1), R(1,2),
            R(2,0), R(2,1), R(2,2)
        );
    }

    bool solveIKFrame(const std::vector<double>& current_q,
                      const KDL::Frame& target_frame,
                      std::vector<double>& result_q)
    {
        if (current_q.size() != num_joints_) {
            std::cerr << "[IK Error] current_q size mismatch. Expected: " << num_joints_
                      << ", Got: " << current_q.size() << std::endl;
            return false;
        }

        KDL::JntArray q_init(num_joints_);
        for (unsigned int i = 0; i < num_joints_; i++) q_init(i) = current_q[i];

        KDL::JntArray q_out(num_joints_);
        int ret = ik_solver_->CartToJnt(q_init, target_frame, q_out);

        if (ret >= 0) {
            result_q.clear();
            result_q.reserve(num_joints_);
            for (unsigned int i = 0; i < num_joints_; i++) result_q.push_back(q_out(i));
            return true;
        }
        return false;
    }
};

#endif // ARM_INVERSE_KINEMATICS_HPP
