# dualarm_forcecon — Dual Arm & Hand Force Control (v6)

Isaac Sim 환경의 **Dual Arm(좌/우 6-DOF)** + **Aidin Hand(좌/우 20-DOF)** 를 ROS 2로 제어/모니터링하는 패키지입니다.

- **FK**: `joint -> EE pose` (Isaac Sim UI와 일치하도록 world↔base 변환 + z-offset 보정 포함)
- **IK**: `EE pose -> joint` (수치해석 기반, joint limit 포함)
- **Mode**: `idle → forward → inverse` 서비스로 순환 전환
- **Monitor**: arm/hand의 curr/targ position & curr/targ force를 **단위 + 컬러**로 출력

---

## 1) DOF / Joint Layout

### Arm (각 팔 6-DOF)
- Left arm joints: `left_joint_1 ... left_joint_6`
- Right arm joints: `right_joint_1 ... right_joint_6`

### Hand (각 손 20-DOF = 5 fingers × 4 joints)
- Fingers: `thumb / index / middle / ring / baby`
- 각 손가락 4개 joint (1~4)
- 내부 hand 벡터 인덱스 매핑(고정):
  - **thumb**: 0~3
  - **index**: 4~7
  - **middle**: 8~11
  - **ring**: 12~15
  - **baby**: 16~19

> v6 기준: 손가락 target 입력은 별도 제공하지 않으며, 기본 동작은 **현재 hand joint 유지**입니다(arm 제어만 수행).

---

## 2) ROS Interfaces

### Subscribed
- `/isaac_joint_states` (`sensor_msgs/msg/JointState`)
  - 팔+손 전체 joint state
- `/isaac_contact_states` (`std_msgs/msg/Float64MultiArray`)
  - data: `[L_Fx, L_Fy, L_Fz, R_Fx, R_Fy, R_Fz]`  (단위: N)
- `/forward_joint_targets` (`std_msgs/msg/Float64MultiArray`)
  - forward 모드 목표 joint (12개)
  - data: `[L_joint1..6, R_joint1..6]` (단위: rad)
- `/target_cartesian_pose` (`std_msgs/msg/Float64MultiArray`)
  - inverse 모드 목표 EE pose (12개)
  - data: `[L_xyz(3), L_rpy(3), R_xyz(3), R_rpy(3)]`
  - xyz 단위: m
  - rpy 단위: 기본 rad (파라미터로 deg/auto 지원)

### Published
- `/isaac_joint_command` (`sensor_msgs/msg/JointState`)
  - Isaac Sim으로 보내는 command (단위: rad)

### Service
- `/change_control_mode` (`std_srvs/srv/Trigger`)
  - 호출할 때마다 `idle -> forward -> inverse -> idle` 순환

---

## 3) Modes

### idle
- 현재 상태를 target으로 1회 동기화 후 유지 (안전 모드)

### forward
- `/forward_joint_targets`를 그대로 target joint로 사용

### inverse
- `/target_cartesian_pose`를 IK로 풀어 target joint 생성  
- IK 실패 시 해당 팔은 **목표 유지(갱신 안 함)**

---

## 4) Monitor Print (v6)

### Units
- **Arm position**
  - xyz: **meter (m)**
  - rpy: **degree (deg)**  *(Isaac Sim UI와 동일한 Euler convention 적용)*
- **Force**
  - Fx Fy Fz: **Newton (N)**

### Color (v6)
- **Curr Position**: (기존 유지 색)
- **Targ Position**: (기존 유지 색)
- **Curr Force**: **red**
- **Targ Force**: **blue**

### Layout
- 모든 arm/hand에 대해
  - `Curr Position` 과 `Curr Force`를 **같은 줄**
  - `Targ Position` 과 `Targ Force`를 **같은 줄**
- 손가락은 바뀔 때마다 빈 줄로 구분:
  - THMB (curr)
  - THMB (targ)
  - (빈 줄)
  - INDX (curr)
  - INDX (targ)
  - (빈 줄)
  - ...

---

## 5) Isaac UI Match Parameters (중요)

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

> **z-offset 이슈는 `world_base_xyz[2]`로 해결**합니다.

---

## 6) Build & Run

### Build
```bash
cd ~/dualarm_ws
colcon build --symlink-install
source install/setup.bash
```

### Run
기본 실행파일 이름이 환경에 따라 다를 수 있습니다. 우선 아래로 확인:
```bash
ros2 pkg executables dualarm_forcecon
```

대부분 v6에서는 아래를 사용합니다:
```bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

---

## 7) Copy & Paste Examples

### (1) Mode toggle
```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger
```

### (2) Forward targets (12 joints)
```bash
ros2 topic pub -1 /forward_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [0.0, -0.78, -2.0, -0.24, 1.34, 0.37,  0.0, 0.78, 2.0, 0.24, -1.33, -0.42]}"
```

### (3) Inverse targets (12 values)
```bash
ros2 topic pub -1 /target_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.53, 0.30, 0.13,  1.85, 0.48, 1.61,   0.53, -0.30, 0.13,  1.85, -0.43, 1.54]}"
```

---

## 8) Code Structure Rules (필수 규칙)

### include/ : Kinematics + Utility만
- `include/dualarm_forcecon/Kinematics/arm_forward_kinematics.hpp`
  - joint -> EE pose 관련 **모든 함수 포함**
  - Isaac UI match를 위한 quaternion/euler 변환, world↔base 적용 포함
- `include/dualarm_forcecon/Kinematics/arm_inverse_kinematics.hpp`
  - EE pose -> joint 관련 **모든 함수 포함**
  - frame/단위/euler convention 처리 포함
- `include/dualarm_forcecon/Kinematics/hand_forward_kinematics.hpp`
  - hand joint -> fingertip pose 관련 함수 포함

✅ 원칙: src/에 kinematics 수학/좌표변환 함수(예: `quatToEuler...`)를 두지 않습니다.

### src/ : ROS glue만
- `src/DualArmForceControl.cpp`
  - **생성자 / 소멸자 / ControlLoop** 만 존재
- `src/states_callback_dualarm.cpp`
  - 이름에 `Callback`이 들어간 함수들 + `PrintDualArmStates()`
  - `PrintDualArmStates()`는 **파일 최하단**
- `src/node_dualarm_main.cpp`
  - node 생성 및 executor spin

---

## 9) Package Layout
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

---

## Version
- **v6**: Isaac Sim UI와 EE pose 일치(FK/IK), world-base z-offset 파라미터화, include/src 규칙 준수, Print 포맷/색상 개선
