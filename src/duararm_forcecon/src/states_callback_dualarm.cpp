#include "DualArmForceControl.h"
#include <iostream>
#include <string>
#include <vector>

// ============================================================================
// JointsCallback
// Topic: /isaac_joint_states
// 설명: Arm(7축) 및 Hand(15축) 관절 상태 파싱
// ============================================================================
void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // 메모리 확보
    if (q_left_.size() != ARM_DOF) q_left_.resize(ARM_DOF);
    if (q_right_.size() != ARM_DOF) q_right_.resize(ARM_DOF);
    if (q_left_hand_.size() != HAND_DOF) q_left_hand_.setZero(HAND_DOF);
    if (q_right_hand_.size() != HAND_DOF) q_right_hand_.setZero(HAND_DOF);

    // 메시지 파싱
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::string name = msg->name[i];
        double pos = msg->position[i];

        // [A] Left Arm
        if (name.find("left_s0") != std::string::npos)      q_left_(0) = pos;
        else if (name.find("left_s1") != std::string::npos) q_left_(1) = pos;
        else if (name.find("left_e0") != std::string::npos) q_left_(2) = pos;
        else if (name.find("left_e1") != std::string::npos) q_left_(3) = pos;
        else if (name.find("left_w0") != std::string::npos) q_left_(4) = pos;
        else if (name.find("left_w1") != std::string::npos) q_left_(5) = pos;
        else if (name.find("left_w2") != std::string::npos) q_left_(6) = pos;

        // [B] Right Arm
        else if (name.find("right_s0") != std::string::npos)      q_right_(0) = pos;
        else if (name.find("right_s1") != std::string::npos) q_right_(1) = pos;
        else if (name.find("right_e0") != std::string::npos) q_right_(2) = pos;
        else if (name.find("right_e1") != std::string::npos) q_right_(3) = pos;
        else if (name.find("right_w0") != std::string::npos) q_right_(4) = pos;
        else if (name.find("right_w1") != std::string::npos) q_right_(5) = pos;
        else if (name.find("right_w2") != std::string::npos) q_right_(6) = pos;

        // [C] Left Hand (Aidin 15 DOF)
        else if (name.find("left_") != std::string::npos && name.find("joint") != std::string::npos) {
            if      (name.find("thumb_joint1") != std::string::npos) q_left_hand_(0) = pos;
            else if (name.find("thumb_joint2") != std::string::npos) q_left_hand_(1) = pos;
            else if (name.find("thumb_joint3") != std::string::npos) q_left_hand_(2) = pos;
            else if (name.find("index_joint1") != std::string::npos) q_left_hand_(3) = pos;
            else if (name.find("index_joint2") != std::string::npos) q_left_hand_(4) = pos;
            else if (name.find("index_joint3") != std::string::npos) q_left_hand_(5) = pos;
            else if (name.find("middle_joint1") != std::string::npos) q_left_hand_(6) = pos;
            else if (name.find("middle_joint2") != std::string::npos) q_left_hand_(7) = pos;
            else if (name.find("middle_joint3") != std::string::npos) q_left_hand_(8) = pos;
            else if (name.find("ring_joint1") != std::string::npos)   q_left_hand_(9) = pos;
            else if (name.find("ring_joint2") != std::string::npos)   q_left_hand_(10) = pos;
            else if (name.find("ring_joint3") != std::string::npos)   q_left_hand_(11) = pos;
            else if (name.find("baby_joint1") != std::string::npos)   q_left_hand_(12) = pos;
            else if (name.find("baby_joint2") != std::string::npos)   q_left_hand_(13) = pos;
            else if (name.find("baby_joint3") != std::string::npos)   q_left_hand_(14) = pos;
        }

        // [D] Right Hand (Aidin 15 DOF)
        else if (name.find("right_") != std::string::npos && name.find("joint") != std::string::npos) {
            if      (name.find("thumb_joint1") != std::string::npos) q_right_hand_(0) = pos;
            else if (name.find("thumb_joint2") != std::string::npos) q_right_hand_(1) = pos;
            else if (name.find("thumb_joint3") != std::string::npos) q_right_hand_(2) = pos;
            else if (name.find("index_joint1") != std::string::npos) q_right_hand_(3) = pos;
            else if (name.find("index_joint2") != std::string::npos) q_right_hand_(4) = pos;
            else if (name.find("index_joint3") != std::string::npos) q_right_hand_(5) = pos;
            else if (name.find("middle_joint1") != std::string::npos) q_right_hand_(6) = pos;
            else if (name.find("middle_joint2") != std::string::npos) q_right_hand_(7) = pos;
            else if (name.find("middle_joint3") != std::string::npos) q_right_hand_(8) = pos;
            else if (name.find("ring_joint1") != std::string::npos)   q_right_hand_(9) = pos;
            else if (name.find("ring_joint2") != std::string::npos)   q_right_hand_(10) = pos;
            else if (name.find("ring_joint3") != std::string::npos)   q_right_hand_(11) = pos;
            else if (name.find("baby_joint1") != std::string::npos)   q_right_hand_(12) = pos;
            else if (name.find("baby_joint2") != std::string::npos)   q_right_hand_(13) = pos;
            else if (name.find("baby_joint3") != std::string::npos)   q_right_hand_(14) = pos;
        }
    }
}

// ============================================================================
// ContactForceCallback
// Topic: /isaac_contact_states
// Type: std_msgs::msg::Float64MultiArray
// 설명: Array 데이터를 Left(0~5), Right(6~11)로 분리 (Wrench 6D 기준)
// ============================================================================
void DualArmForceControl::ContactForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // 데이터가 충분한지 확인 (적어도 6개 이상이어야 함)
    if (msg->data.size() < 6) return;

    // 1. Left Arm Force (Index 0 ~ 5)
    // 순서: [Fx, Fy, Fz, Tx, Ty, Tz]
    force_left_  << msg->data[0], msg->data[1], msg->data[2];
    torque_left_ << msg->data[3], msg->data[4], msg->data[5];

    // 2. Right Arm Force (Index 6 ~ 11) - 데이터가 존재할 경우
    if (msg->data.size() >= 12) {
        force_right_  << msg->data[6], msg->data[7], msg->data[8];
        torque_right_ << msg->data[9], msg->data[10], msg->data[11];
    }

    // 디버깅 (필요시 주석 해제)
    // printf("L_Fz: %.2f | R_Fz: %.2f\n", force_left_(2), force_right_(2));
}