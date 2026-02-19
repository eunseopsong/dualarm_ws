#include "DualArmForceControl.h"
#include <cstdio>
#include <cmath>

void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);

  // 최초 또는 joint name 변경 시 갱신 + pub map 재생성
  const bool need_rebuild =
    (!is_initialized_) ||
    (joint_names_.size() != msg->name.size()) ||
    (!joint_names_.empty() && joint_names_[0] != msg->name[0]);

  if (need_rebuild) {
    joint_names_ = msg->name;
    build_publish_map_locked();
    is_initialized_ = true;
  }

  // msg->name 순서를 joint_names_와 동일하다고 가정(대부분 고정)
  // 만약 환경에서 순서가 바뀐다면, 여기만 name->index map으로 바꾸면 됨.
  for (size_t i=0; i<msg->name.size() && i<pub_map_.size(); ++i) {
    const double p = msg->position[i];
    const auto& e = pub_map_[i];

    switch(e.src){
      case PubMapEntry::Src::L_ARM:  q_l_c_(e.idx) = p; break;
      case PubMapEntry::Src::R_ARM:  q_r_c_(e.idx) = p; break;
      case PubMapEntry::Src::L_HAND: q_l_h_c_(e.idx) = p; break;
      case PubMapEntry::Src::R_HAND: q_r_h_c_(e.idx) = p; break;
      default: break;
    }
  }
}

void DualArmForceControl::TargetPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  // inverse 모드가 아니면 무시
  ControlMode mode_local;
  Eigen::VectorXd ql, qr;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    mode_local = mode_;
    if (mode_local != ControlMode::INVERSE) return;
    if (msg->data.size() < 12) return;

    ql = q_l_c_;
    qr = q_r_c_;
  }

  const double l_xyz[3] = {msg->data[0], msg->data[1], msg->data[2]};
  const double l_rpy[3] = {msg->data[3], msg->data[4], msg->data[5]};
  const double r_xyz[3] = {msg->data[6], msg->data[7], msg->data[8]};
  const double r_rpy[3] = {msg->data[9], msg->data[10], msg->data[11]};

  std::vector<double> ql_vec(ql.data(), ql.data()+6);
  std::vector<double> qr_vec(qr.data(), qr.data()+6);
  std::vector<double> rl, rr;

  bool ok_l = arm_ik_l_ && arm_ik_l_->solveIK(ql_vec, l_xyz, l_rpy, rl);
  bool ok_r = arm_ik_r_ && arm_ik_r_->solveIK(qr_vec, r_xyz, r_rpy, rr);

  if (ok_l || ok_r) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (mode_ != ControlMode::INVERSE) return; // 모드가 바뀌었으면 폐기

    if (ok_l && rl.size()>=6) for(int i=0;i<6;i++) q_l_t_(i) = rl[i];
    if (ok_r && rr.size()>=6) for(int i=0;i<6;i++) q_r_t_(i) = rr[i];
  }
}

void DualArmForceControl::TargetJointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (mode_ != ControlMode::FORWARD) return;
  if (msg->data.size() < 12) return;

  for(int i=0;i<6;i++){
    q_l_t_(i) = msg->data[i];
    q_r_t_(i) = msg->data[i+6];
  }
  // Hand는 고정 유지 (q_*_h_t_ 건드리지 않음)
}

void DualArmForceControl::ControlModeCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  std::lock_guard<std::mutex> lk(mtx_);

  // cycle: idle -> forward -> inverse -> idle
  if (mode_ == ControlMode::IDLE) mode_ = ControlMode::FORWARD;
  else if (mode_ == ControlMode::FORWARD) mode_ = ControlMode::INVERSE;
  else mode_ = ControlMode::IDLE;

  if (mode_ == ControlMode::IDLE) idle_synced_ = false;

  res->success = true;
  res->message = std::string("Mode: ") + mode_to_cstr(mode_);
}

void DualArmForceControl::ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (msg->data.size() >= 6) {
    f_l_c_ << msg->data[0], msg->data[1], msg->data[2];
    f_r_c_ << msg->data[3], msg->data[4], msg->data[5];
  }
}

void DualArmForceControl::PrintDualArmStates()
{
  // 화면 출력은 lock 최소화: 필요한 값만 복사 후 unlock
  ControlMode mode_local;
  Eigen::VectorXd ql_c, qr_c, ql_t, qr_t;
  geometry_msgs::msg::Pose pl, pr;
  geometry_msgs::msg::Point l_t, l_i, l_m, l_r, l_b;
  geometry_msgs::msg::Point r_t, r_i, r_m, r_rp, r_b;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!is_initialized_) return;
    mode_local = mode_;

    ql_c = q_l_c_; qr_c = q_r_c_;
    ql_t = q_l_t_; qr_t = q_r_t_;
    pl = current_pose_l_;
    pr = current_pose_r_;

    l_t = f_l_thumb_; l_i = f_l_index_; l_m = f_l_middle_; l_r = f_l_ring_; l_b = f_l_baby_;
    r_t = f_r_thumb_; r_i = f_r_index_; r_m = f_r_middle_; r_rp = f_r_ring_; r_b = f_r_baby_;
  }

  printf("\033[2J\033[H");
  printf("============================================================================================================\n");
  printf("   Dual Arm & Hand Monitor v5.1 | Mode: [\033[1;32m%-7s\033[0m] | Cyan: Curr, Yel: Targ\n", mode_to_cstr(mode_local));
  printf("============================================================================================================\n");

  auto print_arm = [&](const char* side, geometry_msgs::msg::Pose& p, Eigen::VectorXd& cur_q, Eigen::VectorXd& tar_q) {
    double qx=p.orientation.x, qy=p.orientation.y, qz=p.orientation.z, qw=p.orientation.w;
    double r = std::atan2(2.0*(qw*qx+qy*qz), 1.0-2.0*(qx*qx+qy*qy));
    double sp = 2.0*(qw*qy-qz*qx);
    double pi = (std::abs(sp)>=1.0)?std::copysign(M_PI/2.0, sp):std::asin(sp);
    double y = std::atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz));

    printf("[%s ARM] Joints: \033[1;36m%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\033[0m\n",
      side, cur_q(0), cur_q(1), cur_q(2), cur_q(3), cur_q(4), cur_q(5));
    printf("        Target: \033[1;33m%5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\033[0m\n",
      tar_q(0), tar_q(1), tar_q(2), tar_q(3), tar_q(4), tar_q(5));
    printf("        Pose XYZ: \033[1;35m[%5.3f, %5.3f, %5.3f]\033[0m | RPY: \033[1;33m[%5.2f, %5.2f, %5.2f]\033[0m\n",
      p.position.x, p.position.y, p.position.z, r, pi, y);
  };

  print_arm("L", pl, ql_c, ql_t);
  printf("------------------------------------------------------------------------------------------------------------\n");
  print_arm("R", pr, qr_c, qr_t);
  printf("============================================================================================================\n");

  printf("[FINGERTIP POSITIONS (World Frame)]\n");
  auto print_fingers = [&](const char* side,
                           geometry_msgs::msg::Point& t, geometry_msgs::msg::Point& i,
                           geometry_msgs::msg::Point& m, geometry_msgs::msg::Point& r,
                           geometry_msgs::msg::Point& b) {
    printf("%s: THUMB[%5.3f, %5.3f, %5.3f] INDEX[%5.3f, %5.3f, %5.3f] MID[%5.3f, %5.3f, %5.3f]\n",
      side, t.x, t.y, t.z, i.x, i.y, i.z, m.x, m.y, m.z);
    printf("   RING [%5.3f, %5.3f, %5.3f] BABY [%5.3f, %5.3f, %5.3f]\n", r.x, r.y, r.z, b.x, b.y, b.z);
  };

  print_fingers("L", l_t, l_i, l_m, l_r, l_b);
  printf("-\n");
  print_fingers("R", r_t, r_i, r_m, r_rp, r_b);
  printf("============================================================================================================\n");
}
