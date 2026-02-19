# Dual Arm & Hand Force Control System (v6)

Isaac Sim 환경의 Dual Arm 로봇과 양손(Aidin Hand)을 **ROS 2**로 제어하고 상태를 모니터링하기 위한 **통합 제어 시스템**입니다.  
현재 v6에서도 **Hand는 Arm 제어 중 현재 관절 상태 유지(고정)** 동작을 합니다.

---

## 🚀 주요 기능

- **Real-time Monitoring**
  - 양팔 조인트 상태 (Curr / Targ)
  - End-effector Cartesian 좌표 **(XYZ, meter)** 및 자세 **(RPY, deg)**
  - 접촉 힘(Force) 실시간 출력 **(N)**
- **Forward Kinematics (FK)**
  - KDL 기반으로 조인트 각도 → End-effector 포즈 실시간 계산
  - **Isaac Sim UI와 EE pose가 동일**하도록 world↔base 보정 포함
- **Inverse Kinematics (IK)**
  - 목표 **XYZ/RPY** 입력 → 최적 조인트 각도 산출 (KDL Newton-Raphson + joint limits)
- **Multi-Mode Control**
  - 서비스 호출로 `idle`, `forward`, `inverse` 모드 전환 지원 (순환)

---

## 🧩 DOF 정보

### Arm (각 팔 6-DOF)
- Left arm joints: `left_joint_1 ... left_joint_6`
- Right arm joints: `right_joint_1 ... right_joint_6`

### Hand (각 손 20-DOF = 5 fingers × 4 joints)
- Fingers: `thumb / index / middle / ring / baby`
- 내부 hand 벡터 인덱스 매핑(고정):
  - **thumb**: 0~3
  - **index**: 4~7
  - **middle**: 8~11
  - **ring**: 12~15
  - **baby**: 16~19

> v6 기준: hand target을 별도로 명령하지 않으며, 기본 동작은 **현재 hand joint 유지**입니다.

---

## 🛠 제어 모드

- **idle**
  - 안전 모드 (현재 상태 유지, 명령 대기)
- **forward**
  - 관절 제어 모드  
  - 입력: `12개 조인트` = `[L_joint1~6, R_joint1~6]` (rad)
- **inverse**
  - 좌표 제어 모드  
  - 입력: `12개` = `[L_xyz(3), L_rpy(3), R_xyz(3), R_rpy(3)]`
  - xyz 단위: **meter (m)**
  - rpy 단위: 기본 **rad** (파라미터로 deg/auto 지원)

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

## 📏 출력 단위 (Print Monitor)

- **Arm Position**
  - XYZ: **meter (m)**
  - RPY: **degree (deg)**  *(Isaac Sim UI 기준과 동일한 Euler 출력)*
- **Force**
  - Fx Fy Fz: **Newton (N)**

---

## ⚙️ Isaac Sim UI와 EE pose 일치 설정 (중요)

Isaac Sim UI에서 보이는 EE pose와 맞추기 위해 world↔base 변환을 파라미터로 설정합니다.

- `urdf_path` (string)  
  URDF 경로
- `world_base_xyz` (double[3])  
  world 기준에서 URDF base(root)가 떠있는 오프셋 보정  
  - 예: Isaac에서 base가 z=0.306 떠있으면 `{0,0,0.306}`
- `world_base_euler_xyz_deg` (double[3])  
  world->base 회전 보정(deg)
- `ik_targets_frame` (`base` or `world`)  
  IK 목표가 어느 frame 기준인지
- `ik_euler_conv` (`rpy` or `xyz`)  
  Euler convention
- `ik_angle_unit` (`rad` | `deg` | `auto`)  
  IK 입력 rpy 단위

> **z-offset 문제는 `world_base_xyz[2]`로 보정**합니다.

(선택) 런치/커맨드라인으로 파라미터를 지정하고 싶으면 예를 들어:
```bash
ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p world_base_xyz:="[0.0, 0.0, 0.306]" \
  -p world_base_euler_xyz_deg:="[0.0, 0.0, 0.0]"
```

---

## 📂 파일 구조 & 역할

```text
dualarm_forcecon/
├── CMakeLists.txt
├── package.xml
├── include/dualarm_forcecon/Kinematics/
│   ├── arm_forward_kinematics.hpp
│   ├── arm_inverse_kinematics.hpp
│   └── hand_forward_kinematics.hpp
└── src/
    ├── DualArmForceControl.cpp
    ├── DualArmForceControl.h   (v6에서는 src/에 유지)
    ├── node_dualarm_main.cpp
    └── states_callback_dualarm.cpp
```

### include/ 규칙 (필수)
- **Kinematics 관련 모든 함수는 include/** 에 존재해야 함  
  (예: quaternion→euler 변환, Isaac UI용 Euler convention 변환, world↔base 변환, z-offset 적용 등)
- 목표:
  - `joints -> (arm_forward_kinematics.hpp) -> EE pose`
  - `EE pose -> (arm_inverse_kinematics.hpp) -> joints`

### src/ 규칙 (필수)
- `src/DualArmForceControl.cpp`
  - **생성자 / 소멸자 / ControlLoop** 만 존재
- `src/states_callback_dualarm.cpp`
  - 이름에 `Callback`이 들어간 함수들 + `PrintDualArmStates()`
  - `PrintDualArmStates()`는 **파일 최하단**에 위치

---

## ⚠️ 주의사항

- IK가 해를 찾지 못하는 **가동 범위 밖 좌표**를 입력하면 로봇이 움직이지 않습니다.
- 모니터에 출력되는 **Curr Pose**를 확인하면서 **좌표를 조금씩 변경**해 테스트하세요.
- 현재 버전에서 **Hand는 Arm 제어 중 현재 관절 상태를 유지(고정)** 되어 있습니다.
