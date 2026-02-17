// src/states_callback_dualarm.cpp

#include "DualArmForceControl.h"
#include <iostream>
#include <string>
#include <vector>

// ============================================================================
// DualArmJointsCallback
// Topic: /isaac_joint_states (sensor_msgs/JointState)
// 설명: Arm(7축) 및 Hand(15축 Aidin Hand) 관절 상태 파싱
// ============================================================================
void DualArmForceControl::JointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // 1. 메모리 안전 장치 (최초 실행 시 크기 확보)
    if (q_left_.size() != ARM_DOF) q_left_.resize(ARM_DOF);
    if (q_right_.size() != ARM_DOF) q_right_.resize(ARM_DOF);
    if (q_left_hand_.size() != HAND_DOF) q_left_hand_.setZero(HAND_DOF);
    if (q_right_hand_.size() != HAND_DOF) q_right_hand_.setZero(HAND_DOF);

    // 2. 메시지 파싱
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::string name = msg->name[i];
        double pos = msg->position[i];

        // --------------------------------------------------------------------
        // [A] Left Arm Joints Parsing (예: left_s0 ... left_w2)
        // --------------------------------------------------------------------
        if (name.find("left_s0") != std::string::npos)      q_left_(0) = pos;
        else if (name.find("left_s1") != std::string::npos) q_left_(1) = pos;
        else if (name.find("left_e0") != std::string::npos) q_left_(2) = pos;
        else if (name.find("left_e1") != std::string::npos) q_left_(3) = pos;
        else if (name.find("left_w0") != std::string::npos) q_left_(4) = pos;
        else if (name.find("left_w1") != std::string::npos) q_left_(5) = pos;
        else if (name.find("left_w2") != std::string::npos) q_left_(6) = pos;

        // --------------------------------------------------------------------
        // [B] Right Arm Joints Parsing
        // --------------------------------------------------------------------
        else if (name.find("right_s0") != std::string::npos) q_right_(0) = pos;
        else if (name.find("right_s1") != std::string::npos) q_right_(1) = pos;
        else if (name.find("right_e0") != std::string::npos) q_right_(2) = pos;
        else if (name.find("right_e1") != std::string::npos) q_right_(3) = pos;
        else if (name.find("right_w0") != std::string::npos) q_right_(4) = pos;
        else if (name.find("right_w1") != std::string::npos) q_right_(5) = pos;
        else if (name.find("right_w2") != std::string::npos) q_right_(6) = pos;

        // --------------------------------------------------------------------
        // [C] Left Hand Parsing (Aidin Hand 15 DOF)
        // Order: Thumb(3)->Index(3)->Middle(3)->Ring(3)->Baby(3)
        // --------------------------------------------------------------------
        else if (name.find("left_") != std::string::npos && name.find("joint") != std::string::npos) {
            // Thumb
            if      (name.find("thumb_joint1") != std::string::npos) q_left_hand_(0) = pos;
            else if (name.find("thumb_joint2") != std::string::npos) q_left_hand_(1) = pos;
            else if (name.find("thumb_joint3") != std::string::npos) q_left_hand_(2) = pos;
            // Index
            else if (name.find("index_joint1") != std::string::npos) q_left_hand_(3) = pos;
            else if (name.find("index_joint2") != std::string::npos) q_left_hand_(4) = pos;
            else if (name.find("index_joint3") != std::string::npos) q_left_hand_(5) = pos;
            // Middle
            else if (name.find("middle_joint1") != std::string::npos) q_left_hand_(6) = pos;
            else if (name.find("middle_joint2") != std::string::npos) q_left_hand_(7) = pos;
            else if (name.find("middle_joint3") != std::string::npos) q_left_hand_(8) = pos;
            // Ring
            else if (name.find("ring_joint1") != std::string::npos)   q_left_hand_(9) = pos;
            else if (name.find("ring_joint2") != std::string::npos)   q_left_hand_(10) = pos;
            else if (name.find("ring_joint3") != std::string::npos)   q_left_hand_(11) = pos;
            // Baby (Pinky)
            else if (name.find("baby_joint1") != std::string::npos)   q_left_hand_(12) = pos;
            else if (name.find("baby_joint2") != std::string::npos)   q_left_hand_(13) = pos;
            else if (name.find("baby_joint3") != std::string::npos)   q_left_hand_(14) = pos;
        }

        // --------------------------------------------------------------------
        // [D] Right Hand Parsing (Aidin Hand 15 DOF)
        // --------------------------------------------------------------------
        else if (name.find("right_") != std::string::npos && name.find("joint") != std::string::npos) {
            // Thumb
            if      (name.find("thumb_joint1") != std::string::npos) q_right_hand_(0) = pos;
            else if (name.find("thumb_joint2") != std::string::npos) q_right_hand_(1) = pos;
            else if (name.find("thumb_joint3") != std::string::npos) q_right_hand_(2) = pos;
            // Index
            else if (name.find("index_joint1") != std::string::npos) q_right_hand_(3) = pos;
            else if (name.find("index_joint2") != std::string::npos) q_right_hand_(4) = pos;
            else if (name.find("index_joint3") != std::string::npos) q_right_hand_(5) = pos;
            // Middle
            else if (name.find("middle_joint1") != std::string::npos) q_right_hand_(6) = pos;
            else if (name.find("middle_joint2") != std::string::npos) q_right_hand_(7) = pos;
            else if (name.find("middle_joint3") != std::string::npos) q_right_hand_(8) = pos;
            // Ring
            else if (name.find("ring_joint1") != std::string::npos)   q_right_hand_(9) = pos;
            else if (name.find("ring_joint2") != std::string::npos)   q_right_hand_(10) = pos;
            else if (name.find("ring_joint3") != std::string::npos)   q_right_hand_(11) = pos;
            // Baby (Pinky)
            else if (name.find("baby_joint1") != std::string::npos)   q_right_hand_(12) = pos;
            else if (name.find("baby_joint2") != std::string::npos)   q_right_hand_(13) = pos;
            else if (name.find("baby_joint3") != std::string::npos)   q_right_hand_(14) = pos;
        }
    }
}