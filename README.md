Dual Arm & Hand Force Control System (v5)

이 패키지는 Isaac Sim 환경의 Dual Arm 로봇과 양손(Aidin Hand)을 ROS2를 통해 제어하고 상태를 모니터링하기 위한 통합 제어 시스템입니다.

## 🚀 주요 기능
* **Real-time Monitoring**: 양팔의 조인트 상태, 끝단(End-effector)의 Cartesian 좌표(XYZ) 및 자세(RPY), 접촉 힘(Force)을 실시간 출력합니다.
* **Forward Kinematics (FK)**: KDL 라이브러리를 이용해 조인트 각도 기반의 실시간 포즈를 계산합니다.
* **Inverse Kinematics (IK)**: 목표 좌표(XYZ, RPY)를 입력받아 실시간으로 최적의 조인트 각도를 산출합니다.
* **Multi-Mode Control**: 서비스 호출을 통해 `idle`, `forward`, `inverse` 모드 간 전환을 지원합니다.

## 🛠 제어 모드 설명
1. **idle**: 안전 모드. 현재 상태를 유지하며 명령을 대기합니다.
2. **forward**: 관절 제어 모드. 12개의 팔 조인트 값을 직접 입력받아 움직입니다.
3. **inverse**: 좌표 제어 모드. 목표 XYZ/RPY 값을 입력받아 IK를 통해 움직입니다.

## 💻 실행 및 모드 전환
### 1. 노드 실행
ros2 run dualarm_forcecon dualarm_forcecon_node
2. 제어 모드 전환 (순환: idle -> forward -> inverse)
Bash
ros2 service call /change_control_mode std_srvs/srv/Trigger

---
### 📄 README.md [파트 2: 예시 명령어 및 주의사항]


## 🎯 예시 명령어 (Topic Pub)

### 3. [Forward 모드] 양팔 조인트 제어
* 데이터: `[L_joint1~6, R_joint1~6]`

ros2 topic pub -1 /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [0.0, -0.78, -2.0, -0.24, 1.34, 0.37,  0.0, 0.78, 2.0, 0.24, -1.33, -0.42]}"

4. [Inverse 모드] 양팔 좌표(Cartesian) 제어
데이터: [L_xyz(3), L_rpy(3), R_xyz(3), R_rpy(3)]

기본 위치로 이동:


ros2 topic pub -1 /target_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [0.53, 0.3, 0.13, 1.85, 0.48, 1.61,  0.53, -0.3, 0.13, 1.85, -0.43, 1.54]}"
캔(Can) 앞으로 양손 모으기:


ros2 topic pub -1 /target_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [0.6, 0.15, 0.12, 1.57, 0.0, 1.57,  0.6, -0.15, 0.12, 1.57, 0.0, 1.57]}"
높게 들기:

Bash
ros2 topic pub -1 /target_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [0.55, 0.3, 0.4, 1.8, 0.5, 1.6,  0.55, -0.3, 0.4, 1.8, -0.4, 1.5]}"
📂 파일 구조
ArmInverseKinematics.hpp: KDL Newton-Raphson Solver 기반 IK 연산.

DualArmForceControl.cpp: 메인 제어 루프 및 모드 관리.

states_callback_dualarm.cpp: 토픽 구독 및 실시간 RPY 변환 출력.

⚠️ 주의사항
IK가 해를 찾지 못하는 가동 범위 밖의 좌표 입력 시 로봇이 움직이지 않습니다.

모니터의 Curr Pose를 확인하며 조금씩 좌표를 변경하며 테스트하세요.

현재 버전에서 핸드는 Arm 제어 시 현재 관절 상태를 유지하도록 고정되어 있습니다.
