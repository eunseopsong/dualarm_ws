# dualarm_forcecon v13 README

## 0) 버전 요약

- **패키지명**: `dualarm_forcecon`
- **워크스페이스**: `~/dualarm_ws`
- **현재 버전**: **v13**

### v13 핵심 변경사항 (v12 대비)

1. **Hand IK를 15DoF 구조로 재구성**
   - 손가락당 독립 변수 3개(`joint1~joint3`) 사용
   - `joint4`는 **mimic(`joint4 = joint3`)**로 가정
   - 내부 IK는 15DoF로 풀고, 출력/명령은 20DoF 표현으로 확장하여 사용

2. **모든 손가락 독립 IK (finger-wise independent IK)**
   - 엄지/검지/중지/약지/소지 각각 독립적으로 position-only IK 수행
   - 특정 fingertip target 변경 시 다른 손가락 동반 움직임 최소화

3. **Arm IK auto-frame patch 적용 (world/base 혼동 자동 보정)**
   - 모니터에 표시되는 Arm pose(FK)는 world 좌표계 기준
   - 사용자가 모니터 값을 그대로 `/target_arm_cartesian_pose`에 넣었을 때,
     기존 `ik_targets_frame=base` 설정 때문에 z-offset(0.306m) 오차가 발생하던 문제 수정
   - 현재 관절값 기준으로 target이 `world`처럼 보이면 IK 내부에서 자동으로 world→base 변환
   - 결과적으로 **home pose 유지 명령**이 정상 동작

4. **v12 유지 사항 포함**
   - Hand FK는 15DoF 모델링 기반 + `joint4` mimic 가정
   - Forward target 입력에서 hand 쪽은 **42값 포맷(arm12 + hand30)** 지원

---

## 1) 패키지 구조 (고정)

> 아래 구조는 유지 (추가/삭제/이동 금지 원칙)

```text
~/dualarm_ws/src/dualarm_forcecon
├── CMakeLists.txt
├── package.xml
├── include/
│   └── dualarm_forcecon/
│       └── Kinematics/
│           ├── arm_forward_kinematics.hpp
│           ├── arm_inverse_kinematics.hpp        # v13 auto-frame patch 반영
│           ├── hand_forward_kinematics.hpp       # v12 15DoF + mimic FK 구조
│           ├── hand_inverse_kinematics.hpp       # v13 15DoF independent finger IK
│           └── kinematics_utils.hpp
└── src/
    ├── DualArmForceControl.cpp
    ├── DualArmForceControl.h
    ├── node_dualarm_main.cpp
    └── states_callback_dualarm.cpp
```

---

## 2) 빌드 / 실행

### 빌드
```bash
cd ~/dualarm_ws
colcon build --symlink-install
```

### 환경 설정
```bash
source ~/dualarm_ws/install/setup.bash
```

### 실행
```bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

---

## 3) 제어 모드 전환 (idle → forward → inverse → idle)

서비스는 호출할 때마다 모드가 순환합니다.

```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

예시:
- 현재 `idle`에서 `forward`로 전환: 1번 호출
- `forward`에서 `inverse`로 전환: 1번 더 호출

즉, `idle -> inverse`로 가려면 보통 2번 호출:

```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

---

## 4) 사용 토픽 요약

### Forward 모드용
- `/forward_joint_targets` (`std_msgs/msg/Float64MultiArray`)
  - **42값 포맷 지원**: `arm12 + hand30(양손 각 15)`
  - 기존 52값 포맷(`arm12 + hand40`)도 코드가 유지 중이면 함께 지원 가능

### Inverse 모드용 (분리 토픽)
- `/target_arm_cartesian_pose` (`std_msgs/msg/Float64MultiArray`)
  - 12개 값: `L(xyz+rpy) + R(xyz+rpy)`
- `/target_hand_fingertips` (`std_msgs/msg/Float64MultiArray`)
  - 30개 값: `L hand 5tip xyz + R hand 5tip xyz`
  - 각 hand base frame 기준 좌표

---

## 5) 데이터 포맷 상세

### 5.1 `/target_arm_cartesian_pose` (12 values)
순서:
- Left arm: `x y z rx ry rz`
- Right arm: `x y z rx ry rz`

> 각도는 기본적으로 rad 기준 (현재 파라미터 설정에 따름)

```text
[data:
  Lx, Ly, Lz, Lrx, Lry, Lrz,
  Rx, Ry, Rz, Rrx, Rry, Rrz]
```

---

### 5.2 `/target_hand_fingertips` (30 values)
순서:
- Left hand (thumb, index, middle, ring, baby): 각 fingertip `x y z`
- Right hand (thumb, index, middle, ring, baby): 각 fingertip `x y z`

```text
[data:
  L_thumb_x, L_thumb_y, L_thumb_z,
  L_index_x, L_index_y, L_index_z,
  L_middle_x, L_middle_y, L_middle_z,
  L_ring_x,  L_ring_y,  L_ring_z,
  L_baby_x,  L_baby_y,  L_baby_z,
  R_thumb_x, R_thumb_y, R_thumb_z,
  R_index_x, R_index_y, R_index_z,
  R_middle_x,R_middle_y,R_middle_z,
  R_ring_x,  R_ring_y,  R_ring_z,
  R_baby_x,  R_baby_y,  R_baby_z]
```

---

### 5.3 `/forward_joint_targets` (42 values, v12+ 권장 포맷)
순서:
- Arm 12개: `left arm 6 + right arm 6`
- Hand 30개: `left hand 15 + right hand 15`
  - hand15 순서(각 손): `thumb(1,2,3), index(1,2,3), middle(1,2,3), ring(1,2,3), baby(1,2,3)`
  - `joint4`는 mimic로 내부/명령 경로에서 처리 (`q4=q3`)

```text
[data:
  L_arm_1..6,
  R_arm_1..6,
  L_thumb_1..3, L_index_1..3, L_middle_1..3, L_ring_1..3, L_baby_1..3,
  R_thumb_1..3, R_index_1..3, R_middle_1..3, R_ring_1..3, R_baby_1..3]
```

---

## 6) Inverse 모드 예시 명령어 (v13)

> 아래 예시들은 **inverse 모드**에서 테스트하세요.

### 6.0 inverse 모드 진입
```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

---

### 6.1 Arm IK only 예시 (`/target_arm_cartesian_pose`)

#### (1) Home pose 유지 (모니터 값 재전송)
> v13 arm IK auto-frame patch로 정상 유지되어야 함

```bash
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.5357,  0.2988,  0.4345,   2.8018, 1.3013, -1.2500,
  0.5371, -0.2991,  0.4355,  -2.7962, 1.3011, -1.8800
]}"
```

#### (2) 양팔 앞으로 조금 뻗기 (+x)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.6357,  0.2988,  0.4345,   2.8018, 1.3013, -1.2500,
  0.6371, -0.2991,  0.4355,  -2.7962, 1.3011, -1.8800
]}"

```

#### (3) 양팔 위로 조금 올리기 (+z)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.5357,  0.2988,  0.5345,   2.8018, 1.3013, -1.2500,
  0.5371, -0.2991,  0.5355,  -2.7962, 1.3011, -1.8800
]}"

```

#### (4) 양팔 벌리기 (|y| 증가)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.5357,  0.400,  0.4345,   2.8018, 1.3013, -1.2500,
  0.5371, -0.400,  0.4355,  -2.7962, 1.3011, -1.8800
]}"

```

#### (5) 양팔 모으기 (|y| 감소)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.5357,  0.200,  0.4345,   2.8018, 1.3013, -1.2500,
  0.5371, -0.200,  0.4355,  -2.7962, 1.3011, -1.8800
]}"

```

### 6.2 Hand IK only 예시 (`/target_hand_fingertips`)

#### (1) Home/open 기준 포즈 (기준점)
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

#### (2) 왼손 엄지만 크게 이동 (thumb independence 테스트)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0950, -0.1200,  0.0900,
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

#### (3) 왼손 검지 가리키기 (index만 전방/상방)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0590, -0.1297,  0.1145,
  -0.0403, -0.0120,  0.2900,
  -0.0135, -0.0144,  0.2250,
   0.0133, -0.0144,  0.2100,
   0.0401, -0.0144,  0.2000,

   0.0590, -0.1297,  0.1145,
   0.0403, -0.0144,  0.2465,
   0.0135, -0.0144,  0.2640,
  -0.0133, -0.0144,  0.2465,
  -0.0401, -0.0144,  0.2310
]}"
```

#### (4) 왼손 V-sign (검지/중지만 펴기)
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

#### (5) 양손 소프트 그립 (모든 fingertip z 낮춤)
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

#### (6) 양손 주먹 느낌 (강한 굴곡)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0450, -0.1050,  0.0850,
  -0.0380, -0.0180,  0.1450,
  -0.0120, -0.0180,  0.1500,
   0.0120, -0.0180,  0.1450,
   0.0360, -0.0180,  0.1380,

   0.0450, -0.1050,  0.0850,
   0.0380, -0.0180,  0.1450,
   0.0120, -0.0180,  0.1500,
  -0.0120, -0.0180,  0.1450,
  -0.0360, -0.0180,  0.1380
]}"
```

#### (7) 왼손 pinch (엄지+검지 접근), 오른손 open
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0300, -0.0900,  0.1550,
  -0.0280, -0.0120,  0.2050,
  -0.0135, -0.0144,  0.2550,
   0.0133, -0.0144,  0.2400,
   0.0401, -0.0144,  0.2250,

   0.0590, -0.1297,  0.1145,
   0.0403, -0.0144,  0.2465,
   0.0135, -0.0144,  0.2640,
  -0.0133, -0.0144,  0.2465,
  -0.0401, -0.0144,  0.2310
]}"
```

#### (8) 오른손만 가리키기 (mirror 테스트)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0590, -0.1297,  0.1145,
  -0.0403, -0.0144,  0.2465,
  -0.0135, -0.0144,  0.2640,
   0.0133, -0.0144,  0.2465,
   0.0401, -0.0144,  0.2310,

   0.0590, -0.1297,  0.1145,
   0.0403, -0.0120,  0.2900,
   0.0135, -0.0144,  0.2250,
  -0.0133, -0.0144,  0.2100,
  -0.0401, -0.0144,  0.2000
]}"
```

#### (9) 양손 비대칭 (왼손 V-sign + 오른손 소프트그립)
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0550, -0.1250,  0.1100,
  -0.0450, -0.0100,  0.2850,
  -0.0050, -0.0100,  0.3000,
   0.0150, -0.0140,  0.2100,
   0.0400, -0.0140,  0.1950,

   0.0510, -0.1128,  0.1000,
   0.0403, -0.0151,  0.1800,
   0.0135, -0.0151,  0.1900,
  -0.0133, -0.0151,  0.1800,
  -0.0401, -0.0151,  0.1700
]}"
```

---

### 6.3 Arm + Hand 동시 테스트 (inverse)

#### (1) 양팔 약간 앞으로 + 양손 소프트 그립
```bash
# Arm
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.5600,  0.3000,  0.4450,   2.8018, 1.3013, -1.7200,
  0.5600, -0.3000,  0.4450,  -2.7962, 1.3011, -1.4813
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

#### (2) 왼팔 위/오른팔 아래 + 왼손 pinch / 오른손 open
```bash
# Arm
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.5450,  0.3050,  0.4900,   2.8018, 1.3013, -1.7200,
  0.5450, -0.3050,  0.4000,  -2.7962, 1.3011, -1.4813
]}"

# Hand
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0300, -0.0900,  0.1550,
  -0.0280, -0.0120,  0.2050,
  -0.0135, -0.0144,  0.2550,
   0.0133, -0.0144,  0.2400,
   0.0401, -0.0144,  0.2250,

   0.0590, -0.1297,  0.1145,
   0.0403, -0.0144,  0.2465,
   0.0135, -0.0144,  0.2640,
  -0.0133, -0.0144,  0.2465,
  -0.0401, -0.0144,  0.2310
]}"
```

---

## 7) Forward 모드 예시 명령어 (42값 포맷)

> 아래 예시들은 **forward 모드**에서 테스트하세요.

### 7.0 forward 모드 진입
(현재 `idle`이면 1번 호출)
```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

---

### 7.1 Forward 기준 홈 자세 (42 values)
- arm12 + hand30(15+15)
- hand15 순서: thumb123, index123, middle123, ring123, baby123

```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0046, -0.7842, -2.0022, -0.2409,  1.3370,  0.3665,
 -0.0028,  0.7876,  1.9970,  0.2444, -1.3335, -0.4224,

  0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0
]}"
```

### 7.2 Forward: 양손 소프트 그립 (arm 유지, hand만 굽힘)
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0046, -0.7842, -2.0022, -0.2409,  1.3370,  0.3665,
 -0.0028,  0.7876,  1.9970,  0.2444, -1.3335, -0.4224,

  0.0, 0.25, 0.20,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,
  0.0, 0.25, 0.20,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35,   0.0, 0.45, 0.35
]}"
```

### 7.3 Forward: 왼손 V-sign / 오른손 주먹
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0046, -0.7842, -2.0022, -0.2409,  1.3370,  0.3665,
 -0.0028,  0.7876,  1.9970,  0.2444, -1.3335, -0.4224,

  0.1, 0.30, 0.25,   0.0, 0.05, 0.05,   0.0, 0.05, 0.05,   0.0, 0.80, 0.70,   0.0, 0.85, 0.75,
  0.0, 0.60, 0.50,   0.0, 1.00, 0.90,   0.0, 1.00, 0.90,   0.0, 1.00, 0.90,   0.0, 1.00, 0.90
]}"
```

### 7.4 Forward: 양팔 약간 벌리고 + 양손 open 유지
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.10, -0.70, -1.95, -0.10, 1.28, 0.30,
 -0.10,  0.70,  1.95,  0.10,-1.28,-0.30,

  0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0
]}"
```

### 7.5 Forward: Pinch-like hand command (양손)
```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0046, -0.7842, -2.0022, -0.2409,  1.3370,  0.3665,
 -0.0028,  0.7876,  1.9970,  0.2444, -1.3335, -0.4224,

  0.25, 0.55, 0.45,   0.10, 0.40, 0.25,   0.0, 0.20, 0.15,   0.0, 0.20, 0.15,   0.0, 0.20, 0.15,
  0.25, 0.55, 0.45,   0.10, 0.40, 0.25,   0.0, 0.20, 0.15,   0.0, 0.20, 0.15,   0.0, 0.20, 0.15
]}"
```

---

## 8) 디버깅/검증 팁

### 8.1 현재 모드 확인
`/change_control_mode` 서비스 응답 메시지에서 `Mode: idle/forward/inverse` 확인

### 8.2 토픽 존재 확인
```bash
ros2 topic list | grep target
```
예상 예시:
- `/forward_joint_targets`
- `/target_arm_cartesian_pose`
- `/target_hand_fingertips`

### 8.3 손가락 독립 IK 검증 추천 절차 (v13)
1. `6.2-(1)` 기준 포즈 전송
2. `6.2-(2)` 엄지만 이동 전송
3. 모니터에서 `L HAND THMB` 변화 확인
4. 다른 손가락 변화가 크게 줄었는지 확인

### 8.4 arm home pose 유지 확인 (v13 auto-frame patch)
- 모니터의 home pose 값을 그대로 `/target_arm_cartesian_pose`에 재전송했을 때
- z축으로 0.306m 올라가는 현상이 없어야 함

---

## 9) 추천 커밋 메시지 (영문, 한 줄)

```text
v13: add 15-DoF independent finger IK and arm IK auto world/base frame correction
```

---

## 10) 참고 메모 (개발 규칙)

- Hand는 **15DoF 모델링(독립 변수)** + `joint4` mimic 가정 유지
- 실제 내부 저장/명령 경로는 기존 호환을 위해 20DoF 표현을 사용할 수 있음
- Arm/Hand 토픽 분리 구조 유지 (`/target_arm_cartesian_pose`, `/target_hand_fingertips`)
- `PrintDualArmStates` 포맷/색상 규칙 유지 권장
- world-base z offset 기본값(0.306m) 유지

