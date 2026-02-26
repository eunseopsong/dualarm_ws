# dualarm_forcecon v14 README

> **Version**: v14  
> **Target**: `dualarm_forcecon` (ROS 2 Humble / Isaac Sim 연동)  
> **핵심 변경점**:  
> - **Arm / Hand forward joint command 분리** (`TargetArmJointsCallback`, `TargetHandJointsCallback`)  
> - **Arm FK/IK frame 정합성 개선** (target z vs current z mismatch 완화)  
> - **Arm IK strict frame 처리** (의도치 않은 world/base 자동 해석 최소화)  
> - v13 기능(dual arm + hand monitor, inverse arm/hand, forward 12/42/52 포맷 호환)을 유지하면서 v14 확장

---

## 0) v14에서 달라진 점 (요약)

### ✅ v14 핵심 업데이트
1. **Forward 모드에서 Arm / Hand를 따로 움직일 수 있도록 콜백 분리**
   - `TargetArmJointsCallback` (arm 전용)
   - `TargetHandJointsCallback` (hand 전용)
   - 따라서 **팔 자세 유지 + 손만 변경**, 혹은 **손 유지 + 팔만 변경**을 더 명확하게 제어 가능

2. **Arm FK/IK frame/z-offset 정합성 개선**
   - `TargetArmPositionCallback`에서 z-offset 이슈로 인해 발생하던 `target z` vs `curr z` 혼선을 줄이도록 패치
   - `arm_forward_kinematics.hpp`, `arm_inverse_kinematics.hpp`도 함께 정합성 패치 적용
   - v14에서는 **입력 좌표 프레임(base/world)을 더 엄격하게 유지**하는 방향

3. **기존 통합 forward 토픽 호환 유지(권장: 분리 토픽 사용)**
   - `/forward_joint_targets` (12 / 42 / 52) 사용 가능
   - 하지만 v14부터는 **분리 토픽 사용**이 디버깅/운용에 더 안정적

---

## 1) 패키지 개요

`dualarm_forcecon`는 다음 기능을 수행합니다.

- **Dual Arm + Dual Hand 상태 모니터링**
  - 양팔 TCP pose (position + orientation)
  - 양손 fingertip position (thumb/index/middle/ring/baby)
  - arm force / hand finger force 모니터링
- **Forward control**
  - arm joint target 명령
  - hand joint target 명령
  - arm+hand 통합 joint target 명령(legacy 호환)
- **Inverse control**
  - arm Cartesian target (양팔 12 values)
  - hand fingertip target (양손 30 values)
- **모드 전환 서비스**
  - idle / forward / inverse 순환(기존 Trigger 기반)

---

## 2) v14 기준 패키지 구조 (변경 금지 규칙 포함)

> 아래 트리와 파일 역할 분리는 v14 기준으로 유지 권장

```bash
dualarm_forcecon/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── dualarm_forcecon/
│       └── Kinematics/
│           ├── arm_forward_kinematics.hpp
│           ├── arm_inverse_kinematics.hpp
│           ├── hand_forward_kinematics.hpp
│           ├── hand_inverse_kinematics.hpp
│           └── kinematics_utils.hpp
└── src/
    ├── DualArmForceControl.cpp
    ├── DualArmForceControl.h
    ├── node_dualarm_main.cpp
    └── states_callback_dualarm.cpp
```

### 파일 역할 분리 규칙 (중요)
- `DualArmForceControl.cpp`
  - **생성자 / 소멸자 / `ControlLoop()` 중심**
- `states_callback_dualarm.cpp`
  - **모든 callback 함수들**
  - 예: `JointsCallback`, `TargetArmPositionCallback`, `TargetHandPositionCallback`,
    `TargetArmJointsCallback`, `TargetHandJointsCallback`, `TargetJointCallback`, ...
- `DualArmForceControl.h`
  - 클래스 선언 / ROS 인터페이스 / 상태 변수 / kinematics 객체 선언
- `node_dualarm_main.cpp`
  - 노드 실행 엔트리포인트

---

## 3) v14 주요 기능 상세

### 3.1 Control Modes
- `idle`
  - 현재 joint state를 target으로 동기화
- `forward`
  - joint target 기반 제어 (arm/hand 분리 가능)
- `inverse`
  - arm Cartesian IK + hand fingertip IK 기반 제어

> `Trigger` 서비스 기반으로 모드를 순환하는 기존 방식 유지

---

### 3.2 Arm Kinematics (v14)
- **ArmForwardKinematics**
  - dual arm FK (left/right)
  - world-base transform 설정 가능
  - 모니터 출력용 pose 계산
- **ArmInverseKinematics**
  - KDL 기반 IK
  - `targets_frame`, `euler_conv`, `angle_unit` 처리
  - v14에서 frame 혼선 최소화를 위한 패치 적용 (strict 해석 중심)

#### v14 frame 관련 실무 포인트
- `ik_targets_frame` 기본값이 `"base"`이면, `/target_arm_cartesian_pose`도 **base frame 기준**으로 보내는 것을 권장
- v14는 v13 대비 **자동 프레임 추정/보정 의존도를 낮춤**
- `curr`와 `target` z가 다르게 보이던 문제를 줄이기 위해, FK/IK/Callback 쪽 frame 해석을 맞추는 방향으로 정리됨

---

### 3.3 Hand Kinematics
- **HandForwardKinematics (Pinocchio)**
  - 각 손 5개 fingertip 위치 계산
  - 독립 15DoF + mimic(q4=q3) 구조 반영
- **HandInverseKinematics**
  - fingertip target 기반 손가락 관절 target 생성
  - 손 task space가 좁기 때문에 **입력 workspace를 벗어나면 IK 실패 가능**

#### Hand IK 입력이 task space 범위를 벗어나는 경우 (운용 관점)
- 일반적으로 hand는 arm보다 workspace가 매우 작음
- 과도한 fingertip target 입력 시:
  - IK 실패(또는 일부 손가락 실패) 가능
  - 보통은 **해당 target 반영 실패 → 이전/현재 target 유지** 형태로 운용하는 것이 안전
- 따라서 실무에서는:
  - home pose 기준으로 작은 변화부터 테스트
  - 손가락별로 단계적으로 이동
  - 큰 점프 명령 대신 작은 step command 사용

---

## 4) v14 토픽 / 서비스 인터페이스 (권장)

### 4.1 Subscribe (입력)
- `/isaac_joint_states` (`sensor_msgs/msg/JointState`)
  - Isaac Sim joint states (current)
- `/isaac_contact_states` (`std_msgs/msg/Float64MultiArray`)
  - 접촉/힘 상태 (프로젝트 설정에 맞춤)
- `/target_arm_cartesian_pose` (`std_msgs/msg/Float64MultiArray`)
  - inverse arm target (12 values)
- `/target_hand_fingertips` (`std_msgs/msg/Float64MultiArray`)
  - inverse hand target (30 values)
- `/forward_joint_targets` (`std_msgs/msg/Float64MultiArray`)
  - **legacy 통합 forward target** (12 / 42 / 52)
- **(v14) `/forward_arm_joint_targets`** (`std_msgs/msg/Float64MultiArray`)
  - arm-only forward target (12)
- **(v14) `/target_hand_joint_targets`** (`std_msgs/msg/Float64MultiArray`)
  - hand-only forward target (30 or 40)

> 위 2개 분리 토픽 이름은 v14 분리 콜백(`TargetArmJointsCallback`, `TargetHandJointsCallback`)에 맞춰 사용 권장  
> (실제 코드에서 토픽명을 다르게 선언했다면 그 이름에 맞춰 사용)

### 4.2 Publish (출력)
- `/isaac_joint_command` (`sensor_msgs/msg/JointState`)
  - 최종 명령 joint vector

### 4.3 Service
- `/change_control_mode` (`std_srvs/srv/Trigger`)
  - 모드 전환 (idle → forward → inverse → ...)

---

## 5) 파라미터 (v14)

생성자 기준 주요 파라미터 예시:

- `urdf_path`
- `world_base_xyz` (default `[0,0,0.306]`)
- `world_base_euler_xyz_deg` (default `[0,0,0]`)
- `ik_targets_frame` (`base` / `world`)
- `ik_euler_conv` (`rpy` / `xyz`)
- `ik_angle_unit` (`rad` / `deg` / `auto`)

### 권장 기본값 (v14)
- `ik_targets_frame := base`
- `ik_euler_conv := rpy`
- `ik_angle_unit := rad`

---

## 6) 실행 방법

### 6.1 빌드
```bash
cd ~/dualarm_ws
colcon build --packages-select dualarm_forcecon
source install/setup.bash
```

### 6.2 실행
```bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

---

## 7) 현재 Home Pose 기준 정보 (사용자 제공 실제 로그 기반, v14 기준)

아래 값은 사용자가 제공한 `/isaac_joint_states` 및 모니터 출력의 **home pose 기준값**입니다.

### 7.1 Home pose arm joint values (rad)
> `/isaac_joint_states` 이름 순서가 interleaved + extra joints(`yaw_joint`, `pitch_joint`) 포함이므로, arm 6+6만 추출한 값

- **Left Arm (home)**
  - `qL_home = [1.4869, -0.0022, -2.1753, 1.4835, 1.4835, -0.1920]`
- **Right Arm (home)**
  - `qR_home = [-0.0054, 0.7796, 2.0043, 0.0000, -1.1869, -0.7854]`

### 7.2 Home pose arm monitor pose (v14 monitor 출력값)
- **Left Arm CUR/TAR**
  - `P = (0.1522, 0.2721, -0.2425)`
  - `RPY(deg) = (162.05, 87.40, -72.96)`
- **Right Arm CUR/TAR**
  - `P = (0.5393, -0.3083, 0.1586)`
  - `RPY(deg) = (-66.05, 88.64, 156.05)`

### 7.3 Home pose hand fingertip positions (각 hand base frame)
#### Left Hand (home)
- THMB = `(-0.0590, -0.1297, 0.1145)`
- INDX = `(-0.0403, -0.0144, 0.2465)`
- MIDL = `(-0.0135, -0.0144, 0.2640)`
- RING = `( 0.0133, -0.0144, 0.2465)`
- BABY = `( 0.0401, -0.0144, 0.2310)`

#### Right Hand (home)
- THMB = `( 0.0590, -0.1297, 0.1145)`
- INDX = `( 0.0403, -0.0144, 0.2465)`
- MIDL = `( 0.0135, -0.0144, 0.2640)`
- RING = `(-0.0133, -0.0144, 0.2465)`
- BABY = `(-0.0401, -0.0144, 0.2310)`

---

## 8) 모드 전환 예시

> `Trigger` 서비스가 순환형이라면 현재 모드 상태에 따라 1~2회 호출 필요할 수 있음

```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

- `idle -> forward`
- 다시 호출 시 `forward -> inverse`
- 다시 호출 시 `inverse -> idle` (프로젝트 구현에 따라 다를 수 있음)

---

## 9) Inverse 모드 예시 명령어 (v14, Home Pose 기준)

> 아래 예시들은 **inverse 모드**에서 실행하세요.  
> 권장 파라미터: `ik_targets_frame=base`, `ik_euler_conv=rpy`, `ik_angle_unit=rad`

### 9.0 inverse 모드 진입
```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"   # idle->forward
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"   # forward->inverse
```

---

### 9.1 Arm inverse: Home pose 재전송 (base frame, rad)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.1522,  0.2721, -0.2425,   2.8283, 1.5254, -1.2734,
  0.5393, -0.3083,  0.1586,  -1.1528, 1.5461,  2.7236
]}"
```

> 위 RPY(rad)는 모니터의 deg 값을 rad로 변환한 근사값입니다.  
> (L: 162.05,87.40,-72.96 / R: -66.05,88.64,156.05)

---

### 9.2 Arm inverse: 양팔 조금 앞으로 (home 기준 small step)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.1900,  0.2850, -0.2200,   2.8283, 1.5254, -1.2734,
  0.5150, -0.2950,  0.1800,  -1.1528, 1.5461,  2.7236
]}"
```

---

### 9.3 Arm inverse: 왼팔 위로 / 오른팔 약간 아래 (비대칭 테스트)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.1800,  0.2900, -0.1700,   2.8283, 1.5254, -1.2734,
  0.5250, -0.3000,  0.1100,  -1.1528, 1.5461,  2.7236
]}"
```

---

### 9.4 Hand inverse: Home pose 재전송 (양손 open)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0590, -0.1297,  0.1145,
  -0.0403, -0.0144,  0.2465,
  -0.0135, -0.0144,  0.2640,
   0.0133, -0.0144,  0.2465,
   0.0401, -0.0144,  0.2310,

   0.0590, -0.1297,  0.1145,
   0.0403, -0.0144,  0.2465,
   0.0135, -0.0144,  0.2640,
  -0.0133, -0.0144,  0.2465,
  -0.0401, -0.0144,  0.2310
]}"
```

---

### 9.5 Hand inverse: 양손 소프트 그립
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0510, -0.1128,  0.1000,
  -0.0403, -0.0151,  0.1800,
  -0.0135, -0.0151,  0.1900,
   0.0133, -0.0151,  0.1800,
   0.0401, -0.0151,  0.1700,

   0.0510, -0.1128,  0.1000,
   0.0403, -0.0151,  0.1800,
   0.0135, -0.0151,  0.1900,
  -0.0133, -0.0151,  0.1800,
  -0.0401, -0.0151,  0.1700
]}"
```

---

### 9.6 Hand inverse: 왼손 V-sign / 오른손 open
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0550, -0.1250,  0.1100,
  -0.0450, -0.0100,  0.2850,
  -0.0050, -0.0100,  0.3000,
   0.0150, -0.0140,  0.2100,
   0.0400, -0.0140,  0.1950,

   0.0590, -0.1297,  0.1145,
   0.0403, -0.0144,  0.2465,
   0.0135, -0.0144,  0.2640,
  -0.0133, -0.0144,  0.2465,
  -0.0401, -0.0144,  0.2310
]}"
```

---

### 9.7 Arm + Hand inverse 동시 예시 (양팔 소폭 이동 + 소프트그립)
```bash
# Arm
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.1900,  0.2850, -0.2200,   2.8283, 1.5254, -1.2734,
  0.5150, -0.2950,  0.1800,  -1.1528, 1.5461,  2.7236
]}"

# Hand
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0510, -0.1128,  0.1000,
  -0.0403, -0.0151,  0.1800,
  -0.0135, -0.0151,  0.1900,
   0.0133, -0.0151,  0.1800,
   0.0401, -0.0151,  0.1700,

   0.0510, -0.1128,  0.1000,
   0.0403, -0.0151,  0.1800,
   0.0135, -0.0151,  0.1900,
  -0.0133, -0.0151,  0.1800,
  -0.0401, -0.0151,  0.1700
]}"
```

---

## 10) Forward 모드 예시 명령어 (v14: 분리 토픽 중심)

> 아래 예시들은 **forward 모드**에서 테스트하세요.  
> v14에서는 **arm / hand 분리 토픽** 사용을 권장합니다.

### 10.0 forward 모드 진입
```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

---

## 10.1 Arm-only forward (v14 분리 콜백)

### (A) Home pose arm 재전송 (12 values)
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_arm_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  1.4869, -0.0022, -2.1753,  1.4835,  1.4835, -0.1920,
 -0.0054,  0.7796,  2.0043,  0.0000, -1.1869, -0.7854
]}"
```

### (B) 양팔 약간 벌리기 (home 기준 small perturbation)
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_arm_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  1.4200,  0.0300, -2.1000,  1.3800,  1.4200, -0.1500,
  0.0400,  0.7300,  1.9500,  0.0800, -1.1200, -0.7200
]}"
```

### (C) 왼팔 들기 / 오른팔 내리기 (비대칭)
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_arm_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  1.5200,  0.0500, -1.9800,  1.2000,  1.3500, -0.0500,
 -0.0300,  0.8800,  2.1500, -0.1500, -1.2500, -0.9200
]}"
```

---

## 10.2 Hand-only forward (v14 분리 콜백)

### Hand input format (권장)
- **30 values** = left 15 + right 15
- 각 hand 15 순서:
  - `[thumb1, thumb2, thumb3, index1, index2, index3, middle1, middle2, middle3, ring1, ring2, ring3, baby1, baby2, baby3]`
- 내부적으로 `joint4 = joint3` mimic 적용 (20DoF canonicalization)

> 구현에 따라 **40 values (20+20)**도 받을 수 있도록 해두면 legacy와 호환성이 좋음

### (A) 양손 open (home-like hand)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0
]}"
```

### (B) 양손 소프트 그립
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0, 0.25, 0.20,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,
  0.0, 0.25, 0.20,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35
]}"
```

### (C) 왼손 V-sign / 오른손 주먹
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.10, 0.30, 0.25,   0.00, 0.05, 0.05,   0.00, 0.05, 0.05,   0.00, 0.80, 0.70,   0.00, 0.85, 0.75,
  0.00, 0.60, 0.50,   0.00, 1.00, 0.90,   0.00, 1.00, 0.90,   0.00, 1.00, 0.90,   0.00, 1.00, 0.90
]}"
```

### (D) 왼손 pinch-like / 오른손 open
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.25, 0.65, 0.55,   0.00, 0.55, 0.45,   0.00, 0.10, 0.10,   0.00, 0.15, 0.10,   0.00, 0.15, 0.10,
  0.00, 0.00, 0.00,   0.00, 0.00, 0.00,   0.00, 0.00, 0.00,   0.00, 0.00, 0.00,   0.00, 0.00, 0.00
]}"
```

---

## 10.3 Combined forward (legacy `/forward_joint_targets`) — 호환 유지

> v14에서도 기존 통합 포맷 사용 가능 (권장 우선순위는 분리 토픽)

### 포맷
- `12` : arm only
- `42` : arm(12) + hand15(left) + hand15(right)
- `52` : arm(12) + hand20(left) + hand20(right)  
  (`joint4 = joint3` canonicalization 적용)

### (A) 12값: arm-only home
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  1.4869, -0.0022, -2.1753,  1.4835,  1.4835, -0.1920,
 -0.0054,  0.7796,  2.0043,  0.0000, -1.1869, -0.7854
]}"
```

### (B) 42값: arm(home) + 양손 소프트 그립
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  1.4869, -0.0022, -2.1753,  1.4835,  1.4835, -0.1920,
 -0.0054,  0.7796,  2.0043,  0.0000, -1.1869, -0.7854,

  0.0, 0.25, 0.20,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,
  0.0, 0.25, 0.20,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35
]}"
```

### (C) 52값: arm(home) + hand20(left/right)
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  1.4869, -0.0022, -2.1753,  1.4835,  1.4835, -0.1920,
 -0.0054,  0.7796,  2.0043,  0.0000, -1.1869, -0.7854,

  # left hand 20 (thumb1..4, index1..4, middle1..4, ring1..4, baby1..4)
  0.0, 0.25, 0.20, 0.20,   0.0, 0.45, 0.35, 0.35,   0.0, 0.45, 0.35, 0.35,   0.0, 0.45, 0.35, 0.35,   0.0, 0.45, 0.35, 0.35,

  # right hand 20
  0.0, 0.25, 0.20, 0.20,   0.0, 0.45, 0.35, 0.35,   0.0, 0.45, 0.35, 0.35,   0.0, 0.45, 0.35, 0.35,   0.0, 0.45, 0.35, 0.35
]}"
```

> 주의: 일부 CLI/YAML 파서는 inline 주석(`# ...`) 포함 문자열을 싫어할 수 있으니, 실패 시 주석 제거 후 사용하세요.

---

## 11) 운영 팁 (v14)

### 11.1 Arm IK 실패 시 점검 순서
- `ik_targets_frame`와 입력 좌표 frame이 일치하는지 확인 (`base` vs `world`)
- `ik_angle_unit`가 실제 입력과 일치하는지 확인 (`rad`/`deg`)
- home pose 기준으로 작은 step부터 시도
- 한 번에 큰 position jump 지양

### 11.2 Hand IK 실패/불안정 시 팁
- fingertip target을 home pose 기준으로 조금씩 변경
- 먼저 엄지/검지 1~2개 finger만 테스트
- z를 급격히 내리는 명령보다 단계적 명령 사용
- inverse 실패가 반복되면 forward hand joint command로 자세를 먼저 만든 뒤 inverse 미세조정

### 11.3 모니터 값 해석
- `CUR`와 `TAR`가 다르면:
  - 아직 이동 중
  - IK 실패로 target joint 갱신이 반영되지 않았을 수 있음
  - frame mismatch 가능성 확인

---

## 12) 알려진 주의사항 / v14 메모

- hand task space는 arm보다 훨씬 좁음 (workspace 바깥 입력 주의)
- `/isaac_joint_states`에는 arm/hand 외 보조 조인트(`yaw_joint`, `pitch_joint`)가 포함될 수 있음
- hand q4는 mimic 관계로 내부 canonicalization(`q4=q3`) 적용
- v14에서는 **분리 토픽 기반 forward 제어**를 우선 권장
  - arm과 hand를 독립 테스트 가능
  - 디버깅 시 어느 subsystem 문제가 났는지 분리하기 쉬움

---

## 13) v13 → v14 마이그레이션 체크리스트

- [ ] `TargetArmJointsCallback` / `TargetHandJointsCallback` 추가
- [ ] 분리 토픽 subscription 추가 (`/forward_arm_joint_targets`, `/target_hand_joint_targets`)
- [ ] 기존 `TargetJointCallback`는 호환용으로 유지(선택)
- [ ] `DualArmForceControl.h`에 콜백 선언 및 subscriber 멤버 반영
- [ ] `DualArmForceControl.cpp` 생성자에서 분리 토픽 subscribe 반영
- [ ] `states_callback_dualarm.cpp`에서 arm/hand forward parsing 로직 분리
- [ ] arm FK/IK frame 정책(v14 strict) 일관성 확인
- [ ] home pose 기준 명령어로 smoke test 완료

---

## 14) 빠른 테스트 시나리오 (추천 순서)

1. **idle 상태에서 모니터 안정 확인**
2. **forward 모드**
   - `/forward_arm_joint_targets`로 arm-only home 재전송
   - `/forward_hand_joint_targets`로 hand soft grip
3. **inverse 모드**
   - `/target_hand_fingertips` home 재전송
   - `/target_arm_cartesian_pose` home 재전송
   - arm small step + hand soft grip 동시 테스트
4. 실패 시 frame / angle_unit / workspace 점검

---

## 부록 A) Home pose 모니터 기준 inverse arm RPY(rad) 근사값

- Left (162.05°, 87.40°, -72.96°)
  - `(2.8283, 1.5254, -1.2734)`
- Right (-66.05°, 88.64°, 156.05°)
  - `(-1.1528, 1.5461, 2.7236)`

> 필요하면 이후 v14.1에서 `deg` 입력 예시 버전도 추가 가능

---

## 부록 B) 자주 쓰는 확인 명령어

### 현재 joint state 1회 확인
```bash
ros2 topic echo --once /isaac_joint_states
```

### 현재 모드 전환(순환)
```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

### 노드 실행
```bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```
