# Dual Arm & Hand Force Control System (v6)

Isaac Sim 환경의 Dual Arm 로봇(양팔)과 Aidin Hand(양손)를 **ROS 2**로 제어하고 상태를 모니터링하기 위한 **통합 제어 시스템**입니다.  
v6에서는 **모니터 출력이 Isaac Sim UI와 동일한 EE Pose(특히 RPY/deg, XYZ/meter)**가 되도록 FK/IK(및 world-base z offset) 보정이 반영되어 있습니다.

---

## 🚀 주요 기능

- **Real-time Monitoring (Terminal UI)**
  - 양팔 조인트(Current/Target)
  - End-effector Cartesian 좌표 **XYZ (m)** 및 자세 **RPY (deg)** 출력
  - 접촉력 **Force (N)** 출력
  - 양손(각 손가락) fingertip position 출력
- **Forward Kinematics (FK)**
  - KDL 기반: Joint → End-effector Pose
  - Isaac Sim UI와 동일한 world 기준 Pose가 되도록 world-base offset 보정
- **Inverse Kinematics (IK)**
  - 목표 **XYZ/RPY** → Joint 산출 (KDL Newton-Raphson)
- **Multi-Mode Control**
  - 서비스 호출로 모드 전환: `idle → forward → inverse → idle ...`

---

## 🧩 DOF 구성

- **Arms**: 12-DOF (Left 6 + Right 6)
- **Hands**: 40-DOF (Left 20 + Right 20, 5 fingers × 4 joints)
- **Total**: **52-DOF**

---

## 🛠 제어 모드

- **idle**
  - 안전 모드. 현재 상태를 타겟으로 동기화하여 유지(드리프트 방지)
- **forward**
  - 관절 제어 모드
  - 입력:
    - **12개(팔만)**: `[L_arm(6), R_arm(6)]`
    - **52개(팔+손)**: `[L_arm(6), R_arm(6), L_hand(20), R_hand(20)]`
- **inverse**
  - 좌표 제어 모드 (팔 IK)
  - 입력 12개:
    - `[L_xyz(3), L_rpy(3), R_xyz(3), R_rpy(3)]`
  - **주의**: v6 기준으로 Hand는 inverse에서 별도 제어 입력이 없으면 현재값 유지(고정)

---

## 📡 통신 규격 (Topics & Service)

### Subscribed
- `/isaac_joint_states` (`sensor_msgs/msg/JointState`)  
  - Isaac Sim의 현재 Joint States
- `/isaac_contact_states` (`std_msgs/msg/Float64MultiArray`)  
  - Contact Force (Fx,Fy,Fz) (좌/우)
- `/forward_joint_targets` (`std_msgs/msg/Float64MultiArray`)  
  - forward 모드 타겟 조인트 배열
- `/target_cartesian_pose` (`std_msgs/msg/Float64MultiArray`)  
  - inverse 모드 타겟 EE pose 배열

### Published
- `/isaac_joint_command` (`sensor_msgs/msg/JointState`)  
  - Isaac Sim으로 전송되는 Joint Position command

### Service
- `/change_control_mode` (`std_srvs/srv/Trigger`)  
  - 모드 순환 토글: `idle → forward → inverse → idle ...`

---

## 💻 실행 및 모드 전환

### 1) 노드 실행
> **Node 실행 파일:** `dualarm_ctrl`

```bash
source ~/dualarm_ws/install/setup.bash
ros2 run dualarm_forcecon dualarm_ctrl
```

### 2) 모드 전환 (순환: idle → forward → inverse → idle ...)
```bash
source ~/dualarm_ws/install/setup.bash
ros2 service call /change_control_mode std_srvs/srv/Trigger
```

---

## 🎯 예시 명령어 (복사/붙여넣기용)

> **단위**
> - Joint: **rad**
> - Position: **m**
> - RPY: **rad** (inverse 입력은 rad 기준)
> - Force: **N** (모니터링 출력)

---

### ✅ [Forward 모드] 팔(12개)만 제어
- 토픽: `/forward_joint_targets`
- 타입: `std_msgs/msg/Float64MultiArray`
- 데이터 형식:
  - `[L_joint1..6, R_joint1..6]` (총 12개)

```bash
ros2 topic pub -1 /forward_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [0.0, -0.78, -2.0, -0.24, 1.34, 0.37,  0.0, 0.78, 2.0, 0.24, -1.33, -0.42]}"
```

---

### ✅ [Forward 모드] **팔+손(52개)** 동시에 제어 (손가락 테스트용)
- 데이터 형식:
  - `[0..5]=L_arm(6), [6..11]=R_arm(6), [12..31]=L_hand(20), [32..51]=R_hand(20)`
- Hand 20개는 (엄지~새끼) × (각 4 joint) 순서로 채워짐

#### 1) 팔은 유지 + **양손 손가락 모두 0.5 rad로 굽히기**
```bash
ros2 topic pub -1 /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0, -0.78, -2.00, -0.24,  1.34,  0.37,
  0.0,  0.79,  2.00,  0.24, -1.33, -0.42,
  0.5, 0.5, 0.5, 0.5,   0.5, 0.5, 0.5, 0.5,   0.5, 0.5, 0.5, 0.5,   0.5, 0.5, 0.5, 0.5,   0.5, 0.5, 0.5, 0.5,
  0.5, 0.5, 0.5, 0.5,   0.5, 0.5, 0.5, 0.5,   0.5, 0.5, 0.5, 0.5,   0.5, 0.5, 0.5, 0.5,   0.5, 0.5, 0.5, 0.5
]}"
```

#### 2) 손가락 다시 펴기 (Hand 전체 0.0 rad)
```bash
ros2 topic pub -1 /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0, -0.78, -2.00, -0.24,  1.34,  0.37,
  0.0,  0.79,  2.00,  0.24, -1.33, -0.42,
  0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0
]}"
```

---

### ✅ [Inverse 모드] 팔 Cartesian Pose 제어 (12개)
- 토픽: `/target_cartesian_pose`
- 타입: `std_msgs/msg/Float64MultiArray`
- 데이터 형식:
  - `[L_x, L_y, L_z, L_roll, L_pitch, L_yaw,  R_x, R_y, R_z, R_roll, R_pitch, R_yaw]`
- **주의**: RPY 단위는 **rad**

#### 1) 기본 위치로 이동
```bash
ros2 topic pub -1 /target_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.53, 0.30, 0.13, 1.85, 0.48, 1.61,  0.53, -0.30, 0.13, 1.85, -0.43, 1.54]}"
```

#### 2) 캔(Can) 앞으로 양손 모으기
```bash
ros2 topic pub -1 /target_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.60, 0.15, 0.12, 1.57, 0.00, 1.57,  0.60, -0.15, 0.12, 1.57, 0.00, 1.57]}"
```

#### 3) 높게 들기
```bash
ros2 topic pub -1 /target_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.55, 0.30, 0.40, 1.80, 0.50, 1.60,  0.55, -0.30, 0.40, 1.80, -0.40, 1.50]}"
```

---

## 📂 코드 구조 규칙 (v6 리팩토링 규칙)

### include/ (Kinematics)
- **Kinematics 관련 연산/유틸은 모두 include/에 존재**
  - FK/IK 구현
  - quaternion ↔ euler 변환(Isaac UI match)
  - world-base transform / z-offset 보정
  - (필요 시) pose 합성 같은 수학 유틸

### src/
- `DualArmForceControl.cpp`
  - **생성자/소멸자 + ControlLoop()만**
- `states_callback_dualarm.cpp`
  - **Callback 이름이 들어간 함수들 + PrintDualArmStates()만**
  - `PrintDualArmStates()`는 **파일 최하단**
- `node_dualarm_main.cpp`
  - 노드 엔트리포인트

---

## ⚠️ 주의사항

- IK가 해를 찾지 못하는 **가동 범위 밖 좌표**를 입력하면 로봇이 움직이지 않습니다.
- 모니터에 출력되는 **Curr Pose (XYZ[m], RPY[deg])**를 확인하면서 **좌표를 조금씩 변경**하며 테스트하세요.
- forward에서 **Hand까지 움직이려면** `/forward_joint_targets`에 **52개 배열**을 보내야 합니다.
