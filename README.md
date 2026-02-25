# dualarm_forcecon v12 README

> 워크스페이스: `~/dualarm_ws`  
> 패키지: `dualarm_forcecon`  
> 현재 버전: **v12**

---

## ✅ v12 Git Commit Message (한 줄)

```bash
v12: switch hand FK/IK modeling to 15-DoF with joint4 mimic and support 42-value forward targets
```

---

## 0) v12 핵심 변경사항 요약

### 🔹 Hand 모델링 변경 (중요)
- 기존: hand를 **20DoF 표현**으로 사용 (finger당 4개 관절)
- v12: **실질 자유도 15DoF 모델링**으로 해석
  - finger당 독립 변수: `joint1, joint2, joint3`
  - `joint4`는 **mimic(`joint4 = joint3`)**로 처리

### 🔹 Forward Kinematics (Hand FK) 변경
- `hand_forward_kinematics.hpp`에서 내부적으로
  - **15DoF independent + mimic(q4=q3)** 구조로 FK 계산
- 입력은 **20DoF/15DoF 모두 호환 가능**
  - 20DoF 입력 시 `joint4` 입력값은 무시하고 `joint3`를 사용 (mimic 가정)

### 🔹 Forward target 입력 개선 (`TargetJointCallback`)
- `/forward_joint_targets` 토픽에서 아래 형식을 모두 지원:
  - **12개**: arm only
  - **42개**: arm12 + left hand15 + right hand15 ✅ (v12 compact)
  - **52개**: arm12 + left hand20 + right hand20 (legacy 호환)
- 내부 저장 시 hand는 20개 표현을 유지하되, **`joint4 = joint3`로 canonicalize**

### 🔹 Hand IK 방향성 (v12)
- Hand는 **15DoF 모델링** 기준으로 정리
- 목표: 손가락 간 독립적인 거동(thumb만 바꾸면 thumb 중심으로 움직이도록) 구조로 개선

---

## 1) 🚫 패키지 트리 (구조 고정)

> 파일 추가/삭제/이동 금지. 내부 코드만 수정.

```text
dualarm_forcecon
├── CMakeLists.txt
├── package.xml
├── include
│   └── dualarm_forcecon
│       └── Kinematics
│           ├── arm_forward_kinematics.hpp
│           ├── arm_inverse_kinematics.hpp
│           ├── hand_forward_kinematics.hpp      # v12: 15DoF FK + mimic(q4=q3)
│           ├── hand_inverse_kinematics.hpp      # (v11/v12 hand IK 로직)
│           └── kinematics_utils.hpp
└── src
    ├── DualArmForceControl.cpp                  # 생성자/소멸자/ControlLoop만
    ├── DualArmForceControl.h
    ├── node_dualarm_main.cpp                    # main()
    └── states_callback_dualarm.cpp              # callbacks + print
```

---

## 2) 🚫 파일 역할 규칙 (유지)

### `DualArmForceControl.cpp`
반드시 아래 3개만 포함:
- Constructor
- Destructor
- `ControlLoop()`

### `states_callback_dualarm.cpp`
모든 callback/print 함수 구현:
- `JointsCallback`
- `PositionCallback`
- `ArmPositionCallback`
- `HandPositionCallback`
- `TargetPositionCallback`
- `TargetHandPositionCallback`
- `TargetJointCallback`
- `PrintDualArmStates`
- 기타 상태 업데이트

---

## 3) v12에서도 유지해야 하는 핵심 불변사항

- ✅ 52-DOF 전체 매핑 규칙(Arm 12 + Hand 40 표현)은 **출력/저장 호환성 관점에서 유지**
- ✅ Isaac Sim UI와 일치하는 EE pose 변환 규칙 유지
- ✅ world-base z offset 기본값 `0.306m` 유지
- ✅ `PrintDualArmStates` 포맷/ANSI 색상 규칙 유지
- ✅ 손가락 tip 출력 프레임: `LEFT_HAND_BASE` / `RIGHT_HAND_BASE`

---

## 4) 빌드 / 실행

### Build
```bash
cd ~/dualarm_ws
colcon build --symlink-install
```

### Source
```bash
source ~/dualarm_ws/install/setup.bash
```

### Run
```bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

---

## 5) 모드 전환 (idle → forward → inverse → idle)

서비스는 호출할 때마다 모드가 순환됨.

```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

- 1회 호출: `idle -> forward`
- 2회 호출: `forward -> inverse`
- 3회 호출: `inverse -> idle`

---

## 6) v12 주요 토픽

### Subscribe
- `/isaac_joint_states` (`sensor_msgs/msg/JointState`)
- `/isaac_contact_states` (`std_msgs/msg/Float64MultiArray`)
- `/forward_joint_targets` (`std_msgs/msg/Float64MultiArray`)
- `/target_arm_cartesian_pose` (`std_msgs/msg/Float64MultiArray`)
- `/target_hand_fingertips` (`std_msgs/msg/Float64MultiArray`)

### Publish
- `/isaac_joint_command` (`sensor_msgs/msg/JointState`)

### 토픽 확인
```bash
ros2 topic list | grep target
```

예상:
- `/forward_joint_targets`
- `/target_arm_cartesian_pose`
- `/target_hand_fingertips`

---

## 7) Forward 명령 포맷 (v12)

### ✅ 42개 compact 포맷 (권장)
구성:
- Arm 12개 = `L6 + R6`
- Left hand 15개 = `(thumb,index,middle,ring,baby) × (joint1,joint2,joint3)`
- Right hand 15개 = 동일

총 **42개**

### ✅ 52개 legacy 포맷 (호환)
- Arm 12 + Left hand 20 + Right hand 20
- v12에서 내부 저장 시 `joint4 = joint3`로 canonicalize

---

## 8) Forward(FK) 예시 명령어 — 42개 (다양한 자세)

> 먼저 **Forward 모드**로 전환해야 함 (`idle -> forward`)

```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

---

### (1) 양손 완전 펼침 (Open Hand)
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0046, -0.7842, -2.0022, -0.2409,  1.3370,  0.3665,
 -0.0028,  0.7876,  1.9970,  0.2444, -1.3335, -0.4224,

  0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0
]}"
```

---

### (2) 양손 소프트 그립 (Soft Close)
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0046, -0.7842, -2.0022, -0.2409,  1.3370,  0.3665,
 -0.0028,  0.7876,  1.9970,  0.2444, -1.3335, -0.4224,

  0.08, 0.45, 0.35,   0.00, 0.65, 0.45,   0.00, 0.65, 0.45,   0.00, 0.65, 0.45,   0.00, 0.70, 0.50,
  0.08, 0.45, 0.35,   0.00, 0.65, 0.45,   0.00, 0.65, 0.45,   0.00, 0.65, 0.45,   0.00, 0.70, 0.50
]}"
```

---

### (3) 양손 주먹 쥐기 (Fist-ish)
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0046, -0.7842, -2.0022, -0.2409,  1.3370,  0.3665,
 -0.0028,  0.7876,  1.9970,  0.2444, -1.3335, -0.4224,

  0.18, 0.95, 0.85,   0.00, 1.35, 1.05,   0.00, 1.35, 1.05,   0.00, 1.30, 1.00,   0.00, 1.35, 1.05,
  0.18, 0.95, 0.85,   0.00, 1.35, 1.05,   0.00, 1.35, 1.05,   0.00, 1.30, 1.00,   0.00, 1.35, 1.05
]}"
```

---

### (4) 왼손 엄지-검지 pinch / 오른손 펼침
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0046, -0.7842, -2.0022, -0.2409,  1.3370,  0.3665,
 -0.0028,  0.7876,  1.9970,  0.2444, -1.3335, -0.4224,

  0.20, 0.80, 0.65,   0.12, 1.00, 0.90,   0.00, 0.20, 0.10,   0.00, 0.20, 0.10,   0.00, 0.25, 0.15,
  0.00, 0.00, 0.00,   0.00, 0.00, 0.00,   0.00, 0.00, 0.00,   0.00, 0.00, 0.00,   0.00, 0.00, 0.00
]}"
```

---

### (5) 왼손 가리키기 (Index Point) / 오른손 소프트 그립
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0046, -0.7842, -2.0022, -0.2409,  1.3370,  0.3665,
 -0.0028,  0.7876,  1.9970,  0.2444, -1.3335, -0.4224,

  0.10, 0.55, 0.45,   0.00, 0.05, 0.02,   0.00, 1.10, 0.90,   0.00, 1.10, 0.90,   0.00, 1.15, 0.95,
  0.08, 0.45, 0.35,   0.00, 0.65, 0.45,   0.00, 0.65, 0.45,   0.00, 0.65, 0.45,   0.00, 0.70, 0.50
]}"
```

---

### (6) 왼손 V-sign(가위) / 오른손 중간 그립
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0046, -0.7842, -2.0022, -0.2409,  1.3370,  0.3665,
 -0.0028,  0.7876,  1.9970,  0.2444, -1.3335, -0.4224,

  0.12, 0.60, 0.50,   0.10, 0.10, 0.05,  -0.10, 0.10, 0.05,   0.00, 1.15, 0.90,   0.00, 1.20, 0.95,
  0.10, 0.70, 0.55,   0.00, 0.80, 0.60,   0.00, 0.80, 0.60,   0.00, 0.80, 0.60,   0.00, 0.85, 0.65
]}"
```

---

## 9) Inverse 명령 예시 (참고)

> 먼저 **Inverse 모드**로 전환 (`forward -> inverse`)

```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

### Arm IK only (`/target_arm_cartesian_pose`)
형식: `[L xyz + euler, R xyz + euler]` 총 12개
```bash
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.5357,  0.2988,  0.4345,   2.801777, 1.301317, -1.720022,
  0.5371, -0.2991,  0.4355,  -2.796192, 1.301143, -1.481261
]}"
```

### Hand IK only (`/target_hand_fingertips`)
형식: left 5 tips xyz + right 5 tips xyz = 총 30개 (HAND_BASE frame 기준)
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

## 10) v12 작업 시 주의사항 (실전 체크리스트)

- [ ] `DualArmForceControl.cpp`에 callback 추가/이동 금지
- [ ] `states_callback_dualarm.cpp`에서 callback만 수정
- [ ] Hand FK에서 `joint4`는 입력값 그대로 쓰지 말고 **`joint3`로 mimic 처리**
- [ ] `/forward_joint_targets` 42/52 포맷 모두 테스트
- [ ] 모니터 출력(`PrintDualArmStates`) 포맷/색상 유지
- [ ] Isaac UI 매칭 pose 변환 규칙 및 z-offset(`0.306`) 유지

---

## 11) 참고 메모 (관측 기반 판단)

`/isaac_joint_states`에서 `hand_joint3` vs `hand_joint4`는 완전 동일하지 않고 **0.000x 수준 차이**가 관측될 수 있음.  
v12에서는 이를 하드웨어/시뮬레이터 오차로 보고, FK/모델링에서는 **mimic (`joint4 = joint3`) 가정**으로 정규화하여 사용.

---

## 12) 백업 추천

```bash
cd ~/dualarm_ws/src/dualarm_forcecon
git add .
git commit -m "v12: switch hand FK/IK modeling to 15-DoF with joint4 mimic and support 42-value forward targets"
```
