#include "DualArmForceControl.h"
#include <cstdio>
#include <cmath>

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
    (void)msg; if (!arm_fk_ || !is_initialized_) return;
    
    // 1. Arm FK (Wrist)
    std::vector<double> jl(q_l_c_.data(), q_l_c_.data()+6), jr(q_r_c_.data(), q_r_c_.data()+6);
    current_pose_l_ = arm_fk_->getLeftFK(jl); current_pose_r_ = arm_fk_->getRightFK(jr);

    // 2. Hand FK (Fingertips)
    std::vector<double> hl(q_l_h_c_.data(), q_l_h_c_.data()+20), hr(q_r_h_c_.data(), q_r_h_c_.data()+20);
    auto tl = hand_fk_l_->computeFingertips(hl); auto tr = hand_fk_r_->computeFingertips(hr);

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
    printf("\033[2J\033[H"); // 화면 초기화
    printf("============================================================================================================\n");
    printf("   Dual Arm & Hand Monitor v6 | Mode: [\033[1;32m%-7s\033[0m] | Cyan: Curr, Yel: Targ\n", current_control_mode_.c_str());
    printf("============================================================================================================\n");

    auto print_arm = [&](const char* side, geometry_msgs::msg::Pose& p, Eigen::VectorXd& cur_q, Eigen::VectorXd& tar_q) {
        // RPY 변환
        double qx=p.orientation.x, qy=p.orientation.y, qz=p.orientation.z, qw=p.orientation.w;
        double r = std::atan2(2.0*(qw*qx+qy*qz), 1.0-2.0*(qx*qx+qy*qy));
        double sp = 2.0*(qw*qy-qz*qx);
        double pi = (std::abs(sp)>=1.0)?std::copysign(M_PI/2.0, sp):std::asin(sp);
        double y = std::atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));

        printf("[%s ARM] Joints: \033[1;36m%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\033[0m\n", side, cur_q(0), cur_q(1), cur_q(2), cur_q(3), cur_q(4), cur_q(5));
        printf("        Target: \033[1;33m%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\033[0m\n", tar_q(0), tar_q(1), tar_q(2), tar_q(3), tar_q(4), tar_q(5));
        printf("        Pose XYZ: \033[1;35m[%5.3f, %5.3f, %5.3f]\033[0m | RPY: \033[1;33m[%5.2f, %5.2f, %5.2f]\033[0m\n", p.position.x, p.position.y, p.position.z, r, pi, y);
    };

    print_arm("L", current_pose_l_, q_l_c_, q_l_t_);
    printf("------------------------------------------------------------------------------------------------------------\n");
    print_arm("R", current_pose_r_, q_r_c_, q_r_t_);
    printf("============================================================================================================\n");

    printf("[FINGERTIP POSITIONS (World Frame)]\n");
    auto print_fingers = [&](const char* side, geometry_msgs::msg::Point& t, geometry_msgs::msg::Point& i, geometry_msgs::msg::Point& m, geometry_msgs::msg::Point& r, geometry_msgs::msg::Point& b) {
        printf("%s: THUMB[%5.3f, %5.3f, %5.3f] INDEX[%5.3f, %5.3f, %5.3f] MID[%5.3f, %5.3f, %5.3f]\n", side, t.x, t.y, t.z, i.x, i.y, i.z, m.x, m.y, m.z);
        printf("   RING [%5.3f, %5.3f, %5.3f] BABY [%5.3f, %5.3f, %5.3f]\n", r.x, r.y, r.z, b.x, b.y, b.z);
    };

    print_fingers("L", f_l_thumb_, f_l_index_, f_l_middle_, f_l_ring_, f_l_baby_);
    printf("-\n");
    print_fingers("R", f_r_thumb_, f_r_index_, f_r_middle_, f_r_ring_, f_r_baby_);
    printf("============================================================================================================\n");
}

// 나머지 서비스 및 토픽 콜백 함수들 (기존과 동일)
void DualArmForceControl::TargetPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (current_control_mode_ != "inverse" || msg->data.size() < 12) return;
    double l_xyz[3]={msg->data[0],msg->data[1],msg->data[2]}, l_rpy[3]={msg->data[3],msg->data[4],msg->data[5]};
    double r_xyz[3]={msg->data[6],msg->data[7],msg->data[8]}, r_rpy[3]={msg->data[9],msg->data[10],msg->data[11]};
    std::vector<double> ql(q_l_c_.data(),q_l_c_.data()+6), qr(q_r_c_.data(),q_r_c_.data()+6), rl, rr;
    if(arm_ik_l_->solveIK(ql,l_xyz,l_rpy,rl)) for(int i=0;i<6;i++) q_l_t_(i)=rl[i];
    if(arm_ik_r_->solveIK(qr,r_xyz,r_rpy,rr)) for(int i=0;i<6;i++) q_r_t_(i)=rr[i];
}

void DualArmForceControl::TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (current_control_mode_=="forward" && msg->data.size()>=12) for(int i=0;i<6;i++){q_l_t_(i)=msg->data[i]; q_r_t_(i)=msg->data[i+6];}
}

void DualArmForceControl::ControlModeCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    (void)req; current_control_mode_ = (current_control_mode_=="idle")?"forward":(current_control_mode_=="forward")?"inverse":"idle";
    if(current_control_mode_=="idle") idle_synced_=false;
    res->success=true; res->message="Mode: "+current_control_mode_;
}

void DualArmForceControl::ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if(msg->data.size()>=6){f_l_c_<<msg->data[0],msg->data[1],msg->data[2]; f_r_c_<<msg->data[3],msg->data[4],msg->data[5];}
}