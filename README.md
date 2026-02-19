# Dual Arm & Hand Force Control System (v5)

Isaac Sim 환경의 Dual Arm 로봇과 양손(Aidin Hand)을 **ROS 2**로 제어하고 상태를 모니터링하기 위한 **통합 제어 시스템**입니다.  
현재 v5에서는 **Hand는 Arm 제어 중 현재 관절 상태 유지(고정)** 동작을 합니다.

---

## 🚀 주요 기능

- **Real-time Monitoring**
  - 양팔 조인트 상태
  - End-effector Cartesian 좌표 **(XYZ)** 및 자세 **(RPY)**
  - 접촉 힘(Force) 실시간 출력
- **Forward Kinematics (FK)**
  - KDL 기반으로 조인트 각도 → End-effector 포즈 실시간 계산
- **Inverse Kinematics (IK)**
  - 목표 **XYZ/RPY** 입력 → 최적 조인트 각도 산출 (KDL Newton-Raphson)
- **Multi-Mode Control**
  - 서비스 호출로 `idle`, `forward`, `inverse` 모드 전환 지원

---

## 🛠 제어 모드

- **idle**
  - 안전 모드 (현재 상태 유지, 명령 대기)
- **forward**
  - 관절 제어 모드  
  - 입력: `12개 조인트` = `[L_joint1~6, R_joint1~6]`
- **inverse**
  - 좌표 제어 모드  
  - 입력: `12개` = `[L_xyz(3), L_rpy(3), R_xyz(3), R_rpy(3)]`

---

## 💻 실행 및 모드 전환

### 1) 노드 실행
```bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

### 2) 모드 전환 (순환: idle → forward → inverse → idle ...)
```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger
```

---

## 🎯 예시 명령어 (복사/붙여넣기용)

### [Forward 모드] 양팔 조인트 제어
- 토픽: `/forward_joint_targets`
- 타입: `std_msgs/msg/Float64MultiArray`
- 데이터 형식:  
  `[L_joint1, L_joint2, L_joint3, L_joint4, L_joint5, L_joint6, R_joint1, R_joint2, R_joint3, R_joint4, R_joint5, R_joint6]`

```bash
ros2 topic pub -1 /forward_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [0.0, -0.78, -2.0, -0.24, 1.34, 0.37, 0.0, 0.78, 2.0, 0.24, -1.33, -0.42]}"
```

---

### [Inverse 모드] 양팔 Cartesian Pose 제어
- 토픽: `/target_cartesian_pose`
- 타입: `std_msgs/msg/Float64MultiArray`
- 데이터 형식:  
  `[L_x, L_y, L_z, L_roll, L_pitch, L_yaw,  R_x, R_y, R_z, R_roll, R_pitch, R_yaw]`

#### 1) 기본 위치로 이동
```bash
ros2 topic pub -1 /target_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.53, 0.3, 0.13, 1.85, 0.48, 1.61,  0.53, -0.3, 0.13, 1.85, -0.43, 1.54]}"
```

#### 2) 캔(Can) 앞으로 양손 모으기
```bash
ros2 topic pub -1 /target_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.6, 0.15, 0.12, 1.57, 0.0, 1.57,  0.6, -0.15, 0.12, 1.57, 0.0, 1.57]}"
```

#### 3) 높게 들기
```bash
ros2 topic pub -1 /target_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.55, 0.3, 0.4, 1.8, 0.5, 1.6,  0.55, -0.3, 0.4, 1.8, -0.4, 1.5]}"
```

---

## 📂 파일 구조

- `ArmInverseKinematics.hpp`
  - KDL Newton-Raphson Solver 기반 IK 연산
- `DualArmForceControl.cpp`
  - 메인 제어 루프 및 모드 관리
- `states_callback_dualarm.cpp`
  - 토픽 구독 및 실시간 RPY 변환 출력

---

## ⚠️ 주의사항

- IK가 해를 찾지 못하는 **가동 범위 밖 좌표**를 입력하면 로봇이 움직이지 않습니다.
- 모니터에 출력되는 **Curr Pose**를 확인하면서 **좌표를 조금씩 변경**해 테스트하세요.
- 현재 버전에서 **Hand는 Arm 제어 중 현재 관절 상태를 유지(고정)** 되어 있습니다.
