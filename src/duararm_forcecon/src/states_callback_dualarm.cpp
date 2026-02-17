#include "DualArmForceControl.h"
#include <iostream>
#include <string>
#include <vector>
#include <cstdio> // printf 사용

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

// ============================================================================
// PrintDualArmStates
// 설명: 현재 상태(Cyan)와 목표 상태(Yellow)를 터미널에 컬러로 출력
// ============================================================================
void DualArmForceControl::PrintDualArmStates()
{
    // 화면을 지우고 커서를 상단으로 이동 (실시간 갱신 느낌) - 필요 없으면 주석 처리
    // printf("\033[2J\033[1;1H"); 

    printf("================================================================================\n");
    printf("   Dual Arm & Hand State Monitor (Callback Check) \n");
    printf("================================================================================\n");

    // ------------------------------------------------------------------------
    // 1. ARM JOINTS (7 DOF)
    // ------------------------------------------------------------------------
    printf("[ARM]       J1     J2     J3     J4     J5     J6     J7\n");
    
    // Left Arm
    printf("L_Curr: \033[1;36m"); // Cyan
    for(int i=0; i<ARM_DOF; i++) printf("%6.2f ", q_left_(i));
    printf("\033[0m\n");

    printf("L_Targ: \033[1;33m"); // Yellow
    for(int i=0; i<ARM_DOF; i++) printf("%6.2f ", qd_left_(i));
    printf("\033[0m\n");
    
    printf("--------------------------------------------------------------------------------\n");

    // Right Arm
    printf("R_Curr: \033[1;36m"); // Cyan
    for(int i=0; i<ARM_DOF; i++) printf("%6.2f ", q_right_(i));
    printf("\033[0m\n");

    printf("R_Targ: \033[1;33m"); // Yellow
    for(int i=0; i<ARM_DOF; i++) printf("%6.2f ", qd_right_(i));
    printf("\033[0m\n");

    printf("\n");

    // ------------------------------------------------------------------------
    // 2. FORCE / TORQUE SENSOR
    // ------------------------------------------------------------------------
    printf("[FT SENSOR]      Fx      Fy      Fz      Tx      Ty      Tz\n");
    
    // Left FT
    printf("L_FT(C): \033[1;36m");
    printf("%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f", 
        force_left_(0), force_left_(1), force_left_(2),
        torque_left_(0), torque_left_(1), torque_left_(2));
    printf("\033[0m\n");

    // Right FT
    printf("R_FT(C): \033[1;36m");
    printf("%7.2f %7.2f %7.2f %7.2f %7.2f %7.2f", 
        force_right_(0), force_right_(1), force_right_(2),
        torque_right_(0), torque_right_(1), torque_right_(2));
    printf("\033[0m\n");

    printf("\n");

    // ------------------------------------------------------------------------
    // 3. HAND JOINTS (15 DOF) - 너무 길어서 5개씩 끊어서 출력하거나 요약
    // ------------------------------------------------------------------------
    printf("[HAND] (Thumb:1-3, Index:4-6, Mid:7-9, Ring:10-12, Pinky:13-15)\n");
    
    // Left Hand
    printf("L_Hand(C): \033[1;36m");
    for(int i=0; i<HAND_DOF; i++) {
        printf("%4.1f ", q_left_hand_(i));
        if(i==4 || i==9) printf("| "); // 가독성을 위한 구분선
    }
    printf("\033[0m\n");

    // Right Hand
    printf("R_Hand(C): \033[1;36m");
    for(int i=0; i<HAND_DOF; i++) {
        printf("%4.1f ", q_right_hand_(i));
        if(i==4 || i==9) printf("| ");
    }
    printf("\033[0m\n");
    
    printf("================================================================================\n\n");
}