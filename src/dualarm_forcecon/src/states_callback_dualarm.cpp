#include "DualArmForceControl.h"
#include <cstdio>
#include <cmath>
#include <algorithm>

// -------------------------
// Callbacks
// -------------------------
void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (joint_names_.empty()) {
        joint_names_ = msg->name;
        is_initialized_ = true;
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& n = msg->name[i];
        double p = msg->position[i];

        if      (n == "left_joint_1")  q_l_c_(0) = p;
        else if (n == "left_joint_2")  q_l_c_(1) = p;
        else if (n == "left_joint_3")  q_l_c_(2) = p;
        else if (n == "left_joint_4")  q_l_c_(3) = p;
        else if (n == "left_joint_5")  q_l_c_(4) = p;
        else if (n == "left_joint_6")  q_l_c_(5) = p;
        else if (n == "right_joint_1") q_r_c_(0) = p;
        else if (n == "right_joint_2") q_r_c_(1) = p;
        else if (n == "right_joint_3") q_r_c_(2) = p;
        else if (n == "right_joint_4") q_r_c_(3) = p;
        else if (n == "right_joint_5") q_r_c_(4) = p;
        else if (n == "right_joint_6") q_r_c_(5) = p;
        else {
            int f_idx =
                (n.find("thumb")  != std::string::npos) ? 0  :
                (n.find("index")  != std::string::npos) ? 4  :
                (n.find("middle") != std::string::npos) ? 8  :
                (n.find("ring")   != std::string::npos) ? 12 :
                (n.find("baby")   != std::string::npos) ? 16 : -1;

            if (f_idx != -1) {
                int j_idx =
                    (n.find("1") != std::string::npos) ? 0 :
                    (n.find("2") != std::string::npos) ? 1 :
                    (n.find("3") != std::string::npos) ? 2 :
                    (n.find("4") != std::string::npos) ? 3 : -1;

                if (j_idx != -1) {
                    if (n.find("left") != std::string::npos)  q_l_h_c_(f_idx + j_idx) = p;
                    else                                      q_r_h_c_(f_idx + j_idx) = p;
                }
            }
        }
    }
}

void DualArmForceControl::PositionCallback(const sensor_msgs::msg::JointState::SharedPtr /*msg*/) {
    if (!arm_fk_ || !arm_fk_->isReady() || !is_initialized_) return;

    // 1) Arm FK (World Pose)
    std::vector<double> jl(q_l_c_.data(), q_l_c_.data() + 6);
    std::vector<double> jr(q_r_c_.data(), q_r_c_.data() + 6);

    current_pose_l_ = arm_fk_->getLeftFKWorldPose(jl);
    current_pose_r_ = arm_fk_->getRightFKWorldPose(jr);

    // 2) Hand FK (Fingertips relative pose)
    if (!hand_fk_l_ || !hand_fk_r_) return;

    std::vector<double> hl(q_l_h_c_.data(), q_l_h_c_.data() + 20);
    std::vector<double> hr(q_r_h_c_.data(), q_r_h_c_.data() + 20);

    auto tl = hand_fk_l_->computeFingertips(hl);
    auto tr = hand_fk_r_->computeFingertips(hr);

    if (!tl.empty()) {
        f_l_thumb_  = dualarm_forcecon::kin::composePoseTranslationWorld(current_pose_l_, tl["link4_thumb"]);
        f_l_index_  = dualarm_forcecon::kin::composePoseTranslationWorld(current_pose_l_, tl["link4_index"]);
        f_l_middle_ = dualarm_forcecon::kin::composePoseTranslationWorld(current_pose_l_, tl["link4_middle"]);
        f_l_ring_   = dualarm_forcecon::kin::composePoseTranslationWorld(current_pose_l_, tl["link4_ring"]);
        f_l_baby_   = dualarm_forcecon::kin::composePoseTranslationWorld(current_pose_l_, tl["link4_baby"]);
    }

    if (!tr.empty()) {
        f_r_thumb_  = dualarm_forcecon::kin::composePoseTranslationWorld(current_pose_r_, tr["link4_thumb"]);
        f_r_index_  = dualarm_forcecon::kin::composePoseTranslationWorld(current_pose_r_, tr["link4_index"]);
        f_r_middle_ = dualarm_forcecon::kin::composePoseTranslationWorld(current_pose_r_, tr["link4_middle"]);
        f_r_ring_   = dualarm_forcecon::kin::composePoseTranslationWorld(current_pose_r_, tr["link4_ring"]);
        f_r_baby_   = dualarm_forcecon::kin::composePoseTranslationWorld(current_pose_r_, tr["link4_baby"]);
    }
}

void DualArmForceControl::TargetPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (current_control_mode_ != "inverse") return;
    if (!arm_ik_l_ || !arm_ik_r_) return;
    if (msg->data.size() < 12) return;

    std::array<double,3> l_xyz{msg->data[0], msg->data[1], msg->data[2]};
    std::array<double,3> l_ang{msg->data[3], msg->data[4], msg->data[5]};

    std::array<double,3> r_xyz{msg->data[6], msg->data[7], msg->data[8]};
    std::array<double,3> r_ang{msg->data[9], msg->data[10], msg->data[11]};

    std::vector<double> ql_seed(q_l_c_.data(), q_l_c_.data() + 6);
    std::vector<double> qr_seed(q_r_c_.data(), q_r_c_.data() + 6);

    std::vector<double> ql_out, qr_out;

    bool ok_l = arm_ik_l_->solveIK(ql_seed, l_xyz, l_ang,
                                  ik_targets_frame_, ik_euler_conv_, ik_angle_unit_,
                                  ql_out);

    bool ok_r = arm_ik_r_->solveIK(qr_seed, r_xyz, r_ang,
                                  ik_targets_frame_, ik_euler_conv_, ik_angle_unit_,
                                  qr_out);

    if (ok_l && ql_out.size() >= 6) {
        for (int i=0; i<6; ++i) q_l_t_(i) = ql_out[i];
    }
    if (ok_r && qr_out.size() >= 6) {
        for (int i=0; i<6; ++i) q_r_t_(i) = qr_out[i];
    }
}

void DualArmForceControl::TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (current_control_mode_ != "forward") return;
    if (msg->data.size() < 12) return;

    for (int i=0; i<6; ++i) {
        q_l_t_(i) = msg->data[i];
        q_r_t_(i) = msg->data[i+6];
    }
}

void DualArmForceControl::ControlModeCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if      (current_control_mode_ == "idle")    current_control_mode_ = "forward";
    else if (current_control_mode_ == "forward") current_control_mode_ = "inverse";
    else                                        current_control_mode_ = "idle";

    if (current_control_mode_ == "idle") idle_synced_ = false;

    res->success = true;
    res->message = "Mode: " + current_control_mode_;
}

void DualArmForceControl::ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < 6) return;
    f_l_c_ << msg->data[0], msg->data[1], msg->data[2];
    f_r_c_ << msg->data[3], msg->data[4], msg->data[5];
}

void DualArmForceControl::PrintDualArmStates() {
    if (!is_initialized_) return;

    // ---------- helpers ----------
    auto quatToEulerXYZDeg_Isaac = [](const geometry_msgs::msg::Quaternion& q,
                                     double& ex_deg, double& ey_deg, double& ez_deg)
    {
        // Isaac UI가 보여주는 Euler XYZ(= roll,pitch,yaw 라벨로 쓰되, 기존 UI 매칭 컨벤션 유지)
        // quaternion (x,y,z,w)
        const double x = q.x, y = q.y, z = q.z, w = q.w;

        // Rotation matrix from quaternion
        const double r00 = 1.0 - 2.0*(y*y + z*z);
        const double r01 = 2.0*(x*y - z*w);
        const double r02 = 2.0*(x*z + y*w);

        const double r10 = 2.0*(x*y + z*w);
        const double r11 = 1.0 - 2.0*(x*x + z*z);
        const double r12 = 2.0*(y*z - x*w);

        const double r20 = 2.0*(x*z - y*w);
        const double r21 = 2.0*(y*z + x*w);
        const double r22 = 1.0 - 2.0*(x*x + y*y);

        // XYZ intrinsic (Rx * Ry * Rz) equivalent extraction:
        // ey = asin(r02)
        // ex = atan2(-r12, r22)
        // ez = atan2(-r01, r00)
        double ex = 0.0, ey = 0.0, ez = 0.0;

        // clamp for asin
        double s = r02;
        if (s >  1.0) s =  1.0;
        if (s < -1.0) s = -1.0;

        ey = std::asin(s);

        // gimbal handling: cos(ey) ~ 0
        const double c = std::cos(ey);
        if (std::fabs(c) < 1e-9) {
            // fallback
            ex = std::atan2(r21, r11);
            ez = 0.0;
        } else {
            ex = std::atan2(-r12, r22);
            ez = std::atan2(-r01, r00);
        }

        const double rad2deg = 180.0 / M_PI;
        ex_deg = ex * rad2deg;
        ey_deg = ey * rad2deg;
        ez_deg = ez * rad2deg;
    };

    auto makeVec6 = [](const Eigen::VectorXd& q) {
        std::vector<double> v(6, 0.0);
        for (int i = 0; i < 6; ++i) v[i] = q(i);
        return v;
    };

    auto makeVec20 = [](const Eigen::VectorXd& q) {
        std::vector<double> v(20, 0.0);
        for (int i = 0; i < 20; ++i) v[i] = q(i);
        return v;
    };

    auto poseToXYZRPYdeg = [&](const geometry_msgs::msg::Pose& p,
                              double& x, double& y, double& z,
                              double& roll_deg, double& pitch_deg, double& yaw_deg)
    {
        x = p.position.x; y = p.position.y; z = p.position.z;
        quatToEulerXYZDeg_Isaac(p.orientation, roll_deg, pitch_deg, yaw_deg);
    };

    auto composePointWorld = [&](const geometry_msgs::msg::Pose& base_world,
                                 const geometry_msgs::msg::Pose& rel_base,
                                 geometry_msgs::msg::Point& out_world)
    {
        // 기존 combinePose() 역할: position만 world로
        Eigen::Translation3d t_b(base_world.position.x, base_world.position.y, base_world.position.z);
        Eigen::Quaterniond   q_b(base_world.orientation.w,
                                 base_world.orientation.x,
                                 base_world.orientation.y,
                                 base_world.orientation.z);
        Eigen::Affine3d T_world_base = t_b * q_b;

        Eigen::Translation3d t_r(rel_base.position.x, rel_base.position.y, rel_base.position.z);
        Eigen::Quaterniond   q_r(rel_base.orientation.w,
                                 rel_base.orientation.x,
                                 rel_base.orientation.y,
                                 rel_base.orientation.z);
        Eigen::Affine3d T_base_rel = t_r * q_r;

        Eigen::Affine3d T_world_rel = T_world_base * T_base_rel;

        out_world.x = T_world_rel.translation().x();
        out_world.y = T_world_rel.translation().y();
        out_world.z = T_world_rel.translation().z();
    };

    // ---------- compute current/target poses ----------
    geometry_msgs::msg::Pose cur_pose_l = current_pose_l_;
    geometry_msgs::msg::Pose cur_pose_r = current_pose_r_;
    geometry_msgs::msg::Pose tar_pose_l;
    geometry_msgs::msg::Pose tar_pose_r;

    if (arm_fk_) {
        auto jl_t = makeVec6(q_l_t_);
        auto jr_t = makeVec6(q_r_t_);
        tar_pose_l = arm_fk_->getLeftFK(jl_t);
        tar_pose_r = arm_fk_->getRightFK(jr_t);
    }

    double l_cx, l_cy, l_cz, l_cr, l_cp, l_cyw;
    double l_tx, l_ty, l_tz, l_tr, l_tp, l_tyw;
    double r_cx, r_cy, r_cz, r_cr, r_cp, r_cyw;
    double r_tx, r_ty, r_tz, r_tr, r_tp, r_tyw;

    poseToXYZRPYdeg(cur_pose_l, l_cx, l_cy, l_cz, l_cr, l_cp, l_cyw);
    poseToXYZRPYdeg(tar_pose_l, l_tx, l_ty, l_tz, l_tr, l_tp, l_tyw);
    poseToXYZRPYdeg(cur_pose_r, r_cx, r_cy, r_cz, r_cr, r_cp, r_cyw);
    poseToXYZRPYdeg(tar_pose_r, r_tx, r_ty, r_tz, r_tr, r_tp, r_tyw);

    // target force는 아직 별도 입력/상태가 없으므로 NaN 표시 (값 들어오면 여기만 바꾸면 됨)
    const double tf_nan = std::nan("");

    // ---------- hand fingertips (curr / targ) ----------
    geometry_msgs::msg::Point lc_thumb, lc_index, lc_middle, lc_ring, lc_baby;
    geometry_msgs::msg::Point lt_thumb, lt_index, lt_middle, lt_ring, lt_baby;
    geometry_msgs::msg::Point rc_thumb, rc_index, rc_middle, rc_ring, rc_baby;
    geometry_msgs::msg::Point rt_thumb, rt_index, rt_middle, rt_ring, rt_baby;

    auto fillHandPoints = [&](bool is_left,
                              const geometry_msgs::msg::Pose& wrist_pose_world,
                              const std::vector<double>& hand_q,
                              geometry_msgs::msg::Point& thumb,
                              geometry_msgs::msg::Point& index,
                              geometry_msgs::msg::Point& middle,
                              geometry_msgs::msg::Point& ring,
                              geometry_msgs::msg::Point& baby)
    {
        auto& hand_fk = (is_left ? hand_fk_l_ : hand_fk_r_);
        if (!hand_fk) return;

        auto tips_map = hand_fk->computeFingertips(hand_q);
        if (tips_map.empty()) return;

        if (tips_map.find("link4_thumb")  != tips_map.end()) composePointWorld(wrist_pose_world, tips_map["link4_thumb"],  thumb);
        if (tips_map.find("link4_index")  != tips_map.end()) composePointWorld(wrist_pose_world, tips_map["link4_index"],  index);
        if (tips_map.find("link4_middle") != tips_map.end()) composePointWorld(wrist_pose_world, tips_map["link4_middle"], middle);
        if (tips_map.find("link4_ring")   != tips_map.end()) composePointWorld(wrist_pose_world, tips_map["link4_ring"],   ring);
        if (tips_map.find("link4_baby")   != tips_map.end()) composePointWorld(wrist_pose_world, tips_map["link4_baby"],   baby);
    };

    // curr
    fillHandPoints(true,  cur_pose_l, makeVec20(q_l_h_c_), lc_thumb, lc_index, lc_middle, lc_ring, lc_baby);
    fillHandPoints(false, cur_pose_r, makeVec20(q_r_h_c_), rc_thumb, rc_index, rc_middle, rc_ring, rc_baby);
    // targ (wrist pose도 targ pose를 써야 target fingertip이 맞음)
    fillHandPoints(true,  tar_pose_l, makeVec20(q_l_h_t_), lt_thumb, lt_index, lt_middle, lt_ring, lt_baby);
    fillHandPoints(false, tar_pose_r, makeVec20(q_r_h_t_), rt_thumb, rt_index, rt_middle, rt_ring, rt_baby);

    // ---------- print ----------
    printf("\033[2J\033[H"); // 화면 초기화

    printf("============================================================================================================\n");
    printf("   Dual Arm & Hand Monitor v6 | Mode: [\033[1;32m%-7s\033[0m] | Cyan: Curr, Yel: Targ\n", current_control_mode_.c_str());
    printf("============================================================================================================\n");

    // ---- Arms ----
    printf("[L ARM] Curr Position: \033[1;36m%8.4f %8.4f %8.4f  %8.2f %8.2f %8.2f\033[0m\n",
           l_cx, l_cy, l_cz, l_cr, l_cp, l_cyw);
    printf("       Targ Position: \033[1;33m%8.4f %8.4f %8.4f  %8.2f %8.2f %8.2f\033[0m\n",
           l_tx, l_ty, l_tz, l_tr, l_tp, l_tyw);
    printf("       Curr Force   : \033[1;36m%8.3f %8.3f %8.3f\033[0m\n",
           f_l_c_(0), f_l_c_(1), f_l_c_(2));
    printf("       Targ Force   : \033[1;33m%8.3f %8.3f %8.3f\033[0m\n",
           tf_nan, tf_nan, tf_nan);

    printf("------------------------------------------------------------------------------------------------------------\n");

    printf("[R ARM] Curr Position: \033[1;36m%8.4f %8.4f %8.4f  %8.2f %8.2f %8.2f\033[0m\n",
           r_cx, r_cy, r_cz, r_cr, r_cp, r_cyw);
    printf("       Targ Position: \033[1;33m%8.4f %8.4f %8.4f  %8.2f %8.2f %8.2f\033[0m\n",
           r_tx, r_ty, r_tz, r_tr, r_tp, r_tyw);
    printf("       Curr Force   : \033[1;36m%8.3f %8.3f %8.3f\033[0m\n",
           f_r_c_(0), f_r_c_(1), f_r_c_(2));
    printf("       Targ Force   : \033[1;33m%8.3f %8.3f %8.3f\033[0m\n",
           tf_nan, tf_nan, tf_nan);

    printf("============================================================================================================\n");

    // ---- Hands ----
    auto printFingerPos = [&](const char* name,
                              const geometry_msgs::msg::Point& c,
                              const geometry_msgs::msg::Point& t)
    {
        printf("   %-6s Curr Pos: \033[1;36m%8.4f %8.4f %8.4f\033[0m  |  Targ Pos: \033[1;33m%8.4f %8.4f %8.4f\033[0m\n",
               name, c.x, c.y, c.z, t.x, t.y, t.z);
    };

    auto printFingerForce = [&](const char* name) {
        // 현재 구조에는 finger별 force가 없어서 NaN으로 표시
        printf("          Curr F  : \033[1;36m%8.3f %8.3f %8.3f\033[0m  |  Targ F  : \033[1;33m%8.3f %8.3f %8.3f\033[0m\n",
               tf_nan, tf_nan, tf_nan, tf_nan, tf_nan, tf_nan);
    };

    printf("[L HAND]\n");
    printFingerPos("THUMB",  lc_thumb,  lt_thumb);  printFingerForce("THUMB");
    printFingerPos("INDEX",  lc_index,  lt_index);  printFingerForce("INDEX");
    printFingerPos("MIDDLE", lc_middle, lt_middle); printFingerForce("MIDDLE");
    printFingerPos("RING",   lc_ring,   lt_ring);   printFingerForce("RING");
    printFingerPos("BABY",   lc_baby,   lt_baby);   printFingerForce("BABY");

    printf("------------------------------------------------------------------------------------------------------------\n");

    printf("[R HAND]\n");
    printFingerPos("THUMB",  rc_thumb,  rt_thumb);  printFingerForce("THUMB");
    printFingerPos("INDEX",  rc_index,  rt_index);  printFingerForce("INDEX");
    printFingerPos("MIDDLE", rc_middle, rt_middle); printFingerForce("MIDDLE");
    printFingerPos("RING",   rc_ring,   rt_ring);   printFingerForce("RING");
    printFingerPos("BABY",   rc_baby,   rt_baby);   printFingerForce("BABY");

    printf("============================================================================================================\n");
}
