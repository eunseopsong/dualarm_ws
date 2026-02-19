#include "DualArmForceControl.h"
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace {

inline double wrapDeg180(double deg) {
    while (deg > 180.0)  deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

struct EulerXYZDeg { double x, y, z; };

// R = Rx(x) * Ry(y) * Rz(z) 기준으로 Euler XYZ 뽑고
// y를 asin으로 제한해서 Isaac UI처럼 y가 [-90,90] 쪽 해로 나오게 고정
inline EulerXYZDeg quatToEulerXYZDeg_Isaac(double qx, double qy, double qz, double qw) {
    const double xx = qx*qx, yy = qy*qy, zz = qz*qz;
    const double xy = qx*qy, xz = qx*qz, yz = qy*qz;
    const double wx = qw*qx, wy = qw*qy, wz = qw*qz;

    const double r00 = 1.0 - 2.0*(yy + zz);
    const double r01 = 2.0*(xy - wz);
    const double r02 = 2.0*(xz + wy);

    const double r11 = 1.0 - 2.0*(xx + zz);
    const double r12 = 2.0*(yz - wx);

    const double r21 = 2.0*(yz + wx);
    const double r22 = 1.0 - 2.0*(xx + yy);

    double sy = std::clamp(r02, -1.0, 1.0);
    double y = std::asin(sy);
    double cy = std::cos(y);

    double x = 0.0, z = 0.0;
    const double eps = 1e-10;
    if (std::fabs(cy) > eps) {
        x = std::atan2(-r12, r22);
        z = std::atan2(-r01, r00);
    } else {
        z = 0.0;
        x = std::atan2(r21, r11);
    }

    EulerXYZDeg out;
    out.x = wrapDeg180(x * 180.0 / M_PI);
    out.y = wrapDeg180(y * 180.0 / M_PI);
    out.z = wrapDeg180(z * 180.0 / M_PI);
    return out;
}

} // namespace

void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (joint_names_.empty()) { joint_names_ = msg->name; is_initialized_ = true; }
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::string n = msg->name[i]; double p = msg->position[i];
        if (n == "left_joint_1") q_l_c_(0)=p; else if (n == "left_joint_2") q_l_c_(1)=p;
        else if (n == "left_joint_3") q_l_c_(2)=p; else if (n == "left_joint_4") q_l_c_(3)=p;
        else if (n == "left_joint_5") q_l_c_(4)=p; else if (n == "left_joint_6") q_l_c_(5)=p;
        else if (n == "right_joint_1") q_r_c_(0)=p; else if (n == "right_joint_2") q_r_c_(1)=p;
        else if (n == "right_joint_3") q_r_c_(2)=p; else if (n == "right_joint_4") q_r_c_(3)=p;
        else if (n == "right_joint_5") q_r_c_(4)=p; else if (n == "right_joint_6") q_r_c_(5)=p;
        else {
            int f_idx = (n.find("thumb")!=std::string::npos)?0:(n.find("index")!=std::string::npos)?4:(n.find("middle")!=std::string::npos)?8:(n.find("ring")!=std::string::npos)?12:(n.find("baby")!=std::string::npos)?16:-1;
            if (f_idx!=-1) {
                int j_idx = (n.find("1")!=std::string::npos)?0:(n.find("2")!=std::string::npos)?1:(n.find("3")!=std::string::npos)?2:(n.find("4")!=std::string::npos)?3:-1;
                if (j_idx!=-1) { if (n.find("left")!=std::string::npos) q_l_h_c_(f_idx+j_idx)=p; else q_r_h_c_(f_idx+j_idx)=p; }
            }
        }
    }
}

void DualArmForceControl::PositionCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    (void)msg;
    if (!arm_fk_ || !is_initialized_) return;
    if (arm_fk_ && !arm_fk_->isReady()) return;

    std::vector<double> jl(q_l_c_.data(), q_l_c_.data()+6), jr(q_r_c_.data(), q_r_c_.data()+6);
    current_pose_l_ = arm_fk_->getLeftFK(jl);
    current_pose_r_ = arm_fk_->getRightFK(jr);

    std::vector<double> hl(q_l_h_c_.data(), q_l_h_c_.data()+20), hr(q_r_h_c_.data(), q_r_h_c_.data()+20);
    auto tl = hand_fk_l_->computeFingertips(hl);
    auto tr = hand_fk_r_->computeFingertips(hr);

    if(!tl.empty()){
        f_l_thumb_  = combinePose(current_pose_l_, tl["link4_thumb"]);
        f_l_index_  = combinePose(current_pose_l_, tl["link4_index"]);
        f_l_middle_ = combinePose(current_pose_l_, tl["link4_middle"]);
        f_l_ring_   = combinePose(current_pose_l_, tl["link4_ring"]);
        f_l_baby_   = combinePose(current_pose_l_, tl["link4_baby"]);
    }
    if(!tr.empty()){
        f_r_thumb_  = combinePose(current_pose_r_, tr["link4_thumb"]);
        f_r_index_  = combinePose(current_pose_r_, tr["link4_index"]);
        f_r_middle_ = combinePose(current_pose_r_, tr["link4_middle"]);
        f_r_ring_   = combinePose(current_pose_r_, tr["link4_ring"]);
        f_r_baby_   = combinePose(current_pose_r_, tr["link4_baby"]);
    }
}

void DualArmForceControl::PrintDualArmStates() {
    if (!is_initialized_) return;

    printf("\033[2J\033[H");
    printf("============================================================================================================\n");
    printf("   Dual Arm & Hand Monitor v5.1 | Mode: [\033[1;32m%-7s\033[0m] | Cyan: Curr, Yel: Targ\n", current_control_mode_.c_str());
    printf("============================================================================================================\n");

    auto print_arm = [&](const char* side, const geometry_msgs::msg::Pose& p,
                         const Eigen::VectorXd& cur_q, const Eigen::VectorXd& tar_q)
    {
        auto e = quatToEulerXYZDeg_Isaac(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);

        printf("[%s ARM] Joints:  \033[1;36m%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f\033[0m\n",
               side, cur_q(0), cur_q(1), cur_q(2), cur_q(3), cur_q(4), cur_q(5));

        // ✅ warning 원인(side 인자) 제거: 여기서는 side 출력 안 함
        printf("        Target: \033[1;33m%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f\033[0m\n",
               tar_q(0), tar_q(1), tar_q(2), tar_q(3), tar_q(4), tar_q(5));

        printf("        Translate: \033[1;35m[%7.5f, %7.5f, %7.5f]\033[0m | Orient(XYZ deg): \033[1;33m[%7.3f, %7.3f, %7.3f]\033[0m\n",
               p.position.x, p.position.y, p.position.z, e.x, e.y, e.z);
    };

    print_arm("L", current_pose_l_, q_l_c_, q_l_t_);
    printf("------------------------------------------------------------------------------------------------------------\n");
    print_arm("R", current_pose_r_, q_r_c_, q_r_t_);
    printf("============================================================================================================\n");

    printf("[FINGERTIP POSITIONS (World Frame)]\n");
    auto print_fingers = [&](const char* side,
                             const geometry_msgs::msg::Point& t,
                             const geometry_msgs::msg::Point& i,
                             const geometry_msgs::msg::Point& m,
                             const geometry_msgs::msg::Point& r,
                             const geometry_msgs::msg::Point& b)
    {
        printf("%s: THUMB[%6.3f, %6.3f, %6.3f] INDEX[%6.3f, %6.3f, %6.3f] MID[%6.3f, %6.3f, %6.3f]\n",
               side, t.x, t.y, t.z, i.x, i.y, i.z, m.x, m.y, m.z);
        printf("   RING [%6.3f, %6.3f, %6.3f] BABY [%6.3f, %6.3f, %6.3f]\n",
               r.x, r.y, r.z, b.x, b.y, b.z);
    };

    print_fingers("L", f_l_thumb_, f_l_index_, f_l_middle_, f_l_ring_, f_l_baby_);
    printf("-\n");
    print_fingers("R", f_r_thumb_, f_r_index_, f_r_middle_, f_r_ring_, f_r_baby_);
    printf("============================================================================================================\n");
}

void DualArmForceControl::TargetPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (current_control_mode_ != "inverse" || msg->data.size() < 12) return;

    double l_xyz[3]={msg->data[0],msg->data[1],msg->data[2]};
    double l_rpy[3]={msg->data[3],msg->data[4],msg->data[5]};
    double r_xyz[3]={msg->data[6],msg->data[7],msg->data[8]};
    double r_rpy[3]={msg->data[9],msg->data[10],msg->data[11]};

    std::vector<double> ql(q_l_c_.data(),q_l_c_.data()+6), qr(q_r_c_.data(),q_r_c_.data()+6), rl, rr;
    if(arm_ik_l_->solveIK(ql,l_xyz,l_rpy,rl)) for(int i=0;i<6;i++) q_l_t_(i)=rl[i];
    if(arm_ik_r_->solveIK(qr,r_xyz,r_rpy,rr)) for(int i=0;i<6;i++) q_r_t_(i)=rr[i];
}

void DualArmForceControl::TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (current_control_mode_=="forward" && msg->data.size()>=12)
        for(int i=0;i<6;i++){q_l_t_(i)=msg->data[i]; q_r_t_(i)=msg->data[i+6];}
}

void DualArmForceControl::ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    current_control_mode_ = (current_control_mode_=="idle")?"forward":(current_control_mode_=="forward")?"inverse":"idle";
    if(current_control_mode_=="idle") idle_synced_=false;
    res->success=true;
    res->message="Mode: "+current_control_mode_;
}

void DualArmForceControl::ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if(msg->data.size()>=6){
        f_l_c_<<msg->data[0],msg->data[1],msg->data[2];
        f_r_c_<<msg->data[3],msg->data[4],msg->data[5];
    }
}
