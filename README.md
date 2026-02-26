# DualArmForceControl README (v16)

This README summarizes the current **v16** baseline of `dualarm_forcecon` (dual arm + hand force/control monitor) based on your latest code flow and runtime logs.

---

## 1) Version Summary (v16)

v16 keeps the **v15 inverse delta arm Cartesian control** feature and focuses on **Isaac Sim hand contact force integration** and **monitor consistency**.

### v16 major changes
- **Hand contact force input is now aligned with Isaac Sim Action Graph**
  - Topic: `/isaac_contact_states`
  - Type: `std_msgs/msg/Float32MultiArray`
- **Hand-only contact callback semantics**
  - Existing contact callback is treated as **hand contact callback** (`ContactForceHandCallback` semantics)
  - Arm forces are not measured from this topic
- **Finger force mapping fix**
  - Fixed mismatch where **left ring** contact was displayed as **index**
- **Print order update for hand monitor**
  - Hand fingers are printed in: **BABY → RING → MIDL → INDX → THMB**
- **Display policy update**
  - Arm forces: always shown as `(0, 0, 0)` for now
  - Hand finger forces: `Fx = 0`, `Fy = 0`, `Fz = contact_topic_value`
- **Home pose updated (v16)**
  - README examples now use your latest `/isaac_joint_states` snapshot and monitor pose

### v15 features preserved in v16
- `DeltaArmPositionCallback()` (inverse mode relative arm Cartesian jogging)
- Split forward callbacks
  - `TargetArmJointsCallback()`
  - `TargetHandJointsCallback()`
- Arm FK/IK frame consistency patches (world/base handling + z-offset consistency from earlier fixes)

---

## 2) Package Structure (must preserve)

> Keep the package tree and file-role separation rules unchanged.

```text
dualarm_forcecon/
├── CMakeLists.txt
├── package.xml
├── include/dualarm_forcecon/Kinematics/
│   ├── arm_forward_kinematics.hpp
│   ├── arm_inverse_kinematics.hpp
│   ├── hand_forward_kinematics.hpp
│   ├── hand_inverse_kinematics.hpp
│   └── kinematics_utils.hpp
└── src/
    ├── DualArmForceControl.cpp            # ctor / dtor / ControlLoop only (rule)
    ├── DualArmForceControl.h
    ├── node_dualarm_main.cpp
    └── states_callback_dualarm.cpp        # callbacks + PrintDualArmStates
```

### Important rules (carry-over)
- **Do not change package tree**
- **Do not move callback implementations into `DualArmForceControl.cpp`**
- `DualArmForceControl.cpp` should keep only:
  - constructor
  - destructor
  - `ControlLoop()`
- Preserve:
  - 52-DOF publish mapping
  - Isaac UI Euler convention used in monitor
  - world-base z offset default behavior (0.306 m unless parameterized)
  - `PrintDualArmStates` formatting style (colors/layout), unless intentionally version-updated

---

## 3) Core Functional Overview

### 3.1 Control modes
- `idle`
- `forward`
- `inverse`

### 3.2 What each mode does
- **idle**
  - Synchronizes targets to current states once (safe holding)
- **forward**
  - Accepts **joint-space** commands
  - Arm and hand are controlled through separate topics
- **inverse**
  - Accepts **Cartesian** commands
  - Arm absolute pose (`/target_arm_cartesian_pose`)
  - Arm delta pose (`/delta_arm_cartesian_pose`)  ← v15+, preserved in v16
  - Hand fingertip target positions (`/target_hand_fingertips`)

### 3.3 Monitor output (v16)
- Arm:
  - current/target pose in world display frame
  - force currently displayed as zeros
- Hand:
  - fingertip current/target positions in each hand base frame
  - force display uses hand contact Fz from `/isaac_contact_states` (Fx/Fy are zero-filled)

---

## 4) ROS Interfaces (v16)

## 4.1 Subscriptions

### Joint states
- `/isaac_joint_states` — `sensor_msgs/msg/JointState`
  - Used by:
    - `JointsCallback`
    - `PositionCallback`

### Inverse mode (Cartesian)
- `/target_arm_cartesian_pose` — `std_msgs/msg/Float64MultiArray`
  - **12 values**
  - `[L x y z r p y, R x y z r p y]`
- `/delta_arm_cartesian_pose` — `std_msgs/msg/Float64MultiArray`
  - **12 values**
  - `[L dx dy dz droll dpitch dyaw, R dx dy dz droll dpitch dyaw]`
- `/target_hand_fingertips` — `std_msgs/msg/Float64MultiArray`
  - **30 values**
  - `left(5 fingertips × xyz) + right(5 fingertips × xyz)`

### Forward mode (Joint-space)
- `/forward_arm_joint_targets` — `std_msgs/msg/Float64MultiArray`
  - **12 values**
  - `[left arm 6, right arm 6]`
- `/forward_hand_joint_targets` — `std_msgs/msg/Float64MultiArray`
  - **30 or 40 values**
  - 30 = `left15 + right15`
  - 40 = `left20 + right20` (mimic canonicalization applied internally)

### Hand contact force monitor input (v16)
- `/isaac_contact_states` — `std_msgs/msg/Float32MultiArray`
  - **10 values**
  - hand fingertip scalar contact values (used as **Fz** only for monitor)
  - arm force is **not** taken from this topic

## 4.2 Publisher
- `/isaac_joint_command` — `sensor_msgs/msg/JointState`
  - Consolidated arm + hand command published by `ControlLoop()`

## 4.3 Service
- `/change_control_mode` — `std_srvs/srv/Trigger`
  - Used to cycle/change control mode (implementation-dependent toggle/cycle)
  - Check monitor header (`Mode: [ ... ]`) after each call

---

## 5) Coordinate / Frame Conventions

### 5.1 Arm pose display (monitor)
- `PrintDualArmStates()` arm pose is shown in the **monitor world display frame**
- Euler display uses the project’s Isaac UI matching convention (already patched in FK utilities)

### 5.2 Arm inverse input
- Arm inverse callbacks consume Cartesian targets with configuration from:
  - `ik_targets_frame_` (`base` / `world`)
  - `ik_euler_conv_`
  - `ik_angle_unit_`
- v15/v16 flow assumes your existing arm FK/IK frame patches are already applied (strict/no accidental double z-offset)

### 5.3 Hand pose display
- Hand fingertip positions are printed in:
  - `LEFT_HAND_BASE` frame (left hand)
  - `RIGHT_HAND_BASE` frame (right hand)

### 5.4 Hand fingertip command order vs print order (important)
- **Print order (v16 monitor)**: `BABY → RING → MIDL → INDX → THMB`
- **`/target_hand_fingertips` command order (kept from earlier README/code usage)**:
  - `THMB, INDX, MIDL, RING, BABY` for each hand (each fingertip = xyz)

---

## 6) Home Pose Reference (updated in v16)

## 6.1 Home joint pose (v16 snapshot from `/isaac_joint_states`)

### Arm joints (canonical home reference)
- **Left arm q_home [rad]**
  - `[1.6646, -0.0087, -2.2179, 1.5727, 1.6135, -0.1204]`
- **Right arm q_home [rad]**
  - `[-0.0189, 0.7476, 1.9877, 0.0000, -1.1869, -0.7854]`

### Hand joints (notable nonzero values in v16 home)
- **Left hand**
  - `left_ring_joint2 ≈ -0.0400`
  - `left_thumb_joint2 ≈ 0.3948`
  - `left_ring_joint3 ≈ 0.7854`
  - `left_thumb_joint3 ≈ 0.3927`
  - `left_ring_joint4 ≈ 0.5526`
  - `left_thumb_joint4 ≈ 0.3998`
- **Right hand**
  - `right_thumb_joint2 ≈ 0.3955`
  - `right_thumb_joint3 ≈ 0.3840`
  - `right_thumb_joint4 ≈ 0.3908`

> This means v16 home is **not a fully open hand** baseline anymore (left ring/thumb are intentionally flexed).

## 6.2 Home monitor pose snapshot (v16, from `PrintDualArmStates`)

### Arm (CUR/TAR at home)
- **Left arm**
  - `P[m,deg] = (0.0992, 0.2653, -0.2285 | 155.94, 86.32, -65.29)`
- **Right arm**
  - `P[m,deg] = (0.5464, -0.3184, 0.1380 | 174.49, 88.02, -84.52)`

### Hand fingertip positions at home (hand-base frame, print order)

#### Left hand
- `BABY = ( 0.0401, -0.0144, 0.2310 )`
- `RING = ( 0.0133, -0.0309, 0.2391 )`
- `MIDL = (-0.0135, -0.0144, 0.2640 )`
- `INDX = (-0.0403, -0.0144, 0.2465 )`
- `THMB = (-0.0470, -0.0999, 0.1524 )`

#### Right hand
- `BABY = (-0.0401, -0.0144, 0.2310 )`
- `RING = (-0.0133, -0.0144, 0.2465 )`
- `MIDL = ( 0.0135, -0.0144, 0.2640 )`
- `INDX = ( 0.0403, -0.0144, 0.2465 )`
- `THMB = ( 0.0471, -0.1002, 0.1523 )`

## 6.3 Home pose command examples

### Home arm joint command (12)
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  1.6646, -0.0087, -2.2179, 1.5727, 1.6135, -0.1204,
 -0.0189,  0.7476,  1.9877, 0.0000,-1.1869, -0.7854
]}"
```

### Home hand joint command (30 = left15 + right15)
Order per 15-DoF hand block (each hand):
`[thumb1,2,3, index1,2,3, middle1,2,3, ring1,2,3, baby1,2,3]`

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0000,0.3948,0.3927,   -0.0001,-0.0003,-0.0006,   -0.0005,-0.0002,-0.0007,   0.0000,-0.0400,0.7854,   -0.0001,-0.0003,-0.0007,
  0.0000,0.3955,0.3840,    0.0000,-0.0001,0.0000,     0.0000,-0.0001,0.0000,     0.0000,-0.0001,0.0000,    0.0000,-0.0001,0.0000
]}"
```

---

## 7) Control Mode Switching and Example Commands (v16)

## 7.1 Run node
```bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

## 7.2 Switch control mode (`/change_control_mode`)
> The service is `std_srvs/srv/Trigger`, and your implementation cycles modes.  
> **Always confirm the active mode from the monitor header**:
> `Dual Arm & Hand Monitor ... | Mode: [idle/forward/inverse] | ...`

```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

Call it repeatedly until the monitor shows the desired mode.

### Typical sequence (example)
```bash
# idle -> forward
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"

# forward -> inverse
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"

# inverse -> idle
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

---

## 7.3 Forward mode examples — Arm joints (`/forward_arm_joint_targets`)

> Message type: `std_msgs/msg/Float64MultiArray`  
> Format: **12 values** = `[left arm 6, right arm 6]`

### (A) Arm home pose (v16)
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  1.6646, -0.0087, -2.2179, 1.5727, 1.6135, -0.1204,
 -0.0189,  0.7476,  1.9877, 0.0000,-1.1869, -0.7854
]}"
```

### (B) Symmetric small motion from home (safe test)
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  1.7046, -0.0287, -2.1779, 1.5427, 1.5935, -0.0804,
 -0.0589,  0.7676,  1.9477, 0.0300,-1.1669, -0.8254
]}"
```

### (C) Left arm only move, right arm keep home
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  1.7346,  0.0413, -2.0979, 1.4927, 1.5435, -0.0404,
 -0.0189,  0.7476,  1.9877, 0.0000,-1.1869, -0.7854
]}"
```

---

## 7.4 Forward mode examples — Hand joints (`/forward_hand_joint_targets`)

> Message type: `std_msgs/msg/Float64MultiArray`  
> Supported payload sizes:
> - **30** = `left15 + right15`
> - **40** = `left20 + right20` (joint4 values included; internal canonicalization applies)

### 15-DoF order (per hand, 30-format)
`[thumb1,2,3, index1,2,3, middle1,2,3, ring1,2,3, baby1,2,3]`

### 20-DoF order (per hand, 40-format)
`[thumb1,2,3,4, index1,2,3,4, middle1,2,3,4, ring1,2,3,4, baby1,2,3,4]`

### (A) Hand home pose (30 values, v16)
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0000,0.3948,0.3927,   -0.0001,-0.0003,-0.0006,   -0.0005,-0.0002,-0.0007,   0.0000,-0.0400,0.7854,   -0.0001,-0.0003,-0.0007,
  0.0000,0.3955,0.3840,    0.0000,-0.0001,0.0000,     0.0000,-0.0001,0.0000,     0.0000,-0.0001,0.0000,    0.0000,-0.0001,0.0000
]}"
```

### (B) 40-value full hand command (explicit joint4 values)
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0,0.3948,0.3927,0.3998,   -0.0001,-0.0003,-0.0006,-0.0058,   -0.0005,-0.0002,-0.0007,-0.0060,   0.0,-0.0400,0.7854,0.5526,   -0.0001,-0.0003,-0.0007,-0.0065,
  0.0,0.3955,0.3840,0.3908,    0.0,-0.0001,0.0,0.0014,             0.0,-0.0001,0.0,0.0017,             0.0,-0.0001,0.0,0.0014,      0.0,-0.0001,0.0,0.0012
]}"
```

### (C) Example gesture — Right hand keep home, Left hand thumb flex + ring 90deg (30 values)
(Useful reference because you asked for a similar command earlier.)

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0,0.80,0.80,   0.0,0.0,0.0,   0.0,0.0,0.0,   0.0,0.0,1.5708,   0.0,0.0,0.0,
  0.0,0.3955,0.3840,   0.0,-0.0001,0.0,   0.0,-0.0001,0.0,   0.0,-0.0001,0.0,   0.0,-0.0001,0.0
]}"
```

### (D) Gentle closing test (both hands, 30 values)
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0,0.55,0.45,  0.0,0.25,0.20,  0.0,0.25,0.20,  0.0,0.30,0.25,  0.0,0.20,0.15,
  0.0,0.55,0.45,  0.0,0.25,0.20,  0.0,0.25,0.20,  0.0,0.25,0.20,  0.0,0.20,0.15
]}"
```

---

## 7.5 Inverse mode examples — Arm absolute Cartesian pose (`/target_arm_cartesian_pose`)

> Message type: `std_msgs/msg/Float64MultiArray`  
> Format: **12** = `[L x y z r p y, R x y z r p y]`

### (A) Absolute arm target near v16 home monitor pose (rad)
Euler values below are taken from the v16 monitor home snapshot (degrees → radians).

- Left Euler rad ≈ `(2.7217, 1.5066, -1.1395)`
- Right Euler rad ≈ `(3.0454, 1.5362, -1.4752)`

```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.0992,  0.2653, -0.2285,   2.7217, 1.5066, -1.1395,
  0.5464, -0.3184,  0.1380,   3.0454, 1.5362, -1.4752
]}"
```

### (B) Small Cartesian shift from home-like target (absolute)
```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.1192,  0.2553, -0.2085,   2.7217, 1.5066, -1.1395,
  0.5364, -0.3084,  0.1380,   3.0454, 1.5362, -1.4752
]}"
```

### (C) Left arm only absolute move, right arm hold
```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.1400,  0.2500, -0.2000,   2.7000, 1.5000, -1.1000,
  0.5464, -0.3184,  0.1380,   3.0454, 1.5362, -1.4752
]}"
```

---

## 7.6 Inverse mode examples — Delta arm Cartesian pose (`/delta_arm_cartesian_pose`)  *(v15 feature, used in v16)*

> Message type: `std_msgs/msg/Float64MultiArray`  
> Format: **12** = `[L dx dy dz droll dpitch dyaw, R dx dy dz droll dpitch dyaw]`

### (A) Zero delta (sanity)
```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0,0,0,0,0,0,  0,0,0,0,0,0]}"
```

### (B) Small symmetric jog
- Left: +x, -y, +z
- Right: -x, +y, +z

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
   0.020, -0.010, 0.015,   0.00, 0.00, 0.00,
  -0.020,  0.010, 0.015,   0.00, 0.00, 0.00
]}"
```

### (C) Left-arm orientation jog only
```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.000, 0.000, 0.000,   0.05, -0.03, 0.08,
  0.000, 0.000, 0.000,   0.00,  0.00, 0.00
]}"
```

### (D) Practical example (from your description)
If current left arm pose is `(0.5, 0.5, 0.5)` and you send `(0.3, -0.3, 0.2)`, the target becomes `(0.8, 0.2, 0.7)` (same concept applies to orientation deltas).

---

## 7.7 Inverse mode examples — Hand fingertip Cartesian targets (`/target_hand_fingertips`)

> Message type: `std_msgs/msg/Float64MultiArray`  
> Format: **30** values  
> Order per hand (kept from prior usage):  
> `[THMB xyz, INDX xyz, MIDL xyz, RING xyz, BABY xyz]`  
> Full payload = `left hand 15 + right hand 15`

### (A) Hand home-like fingertip target (v16 monitor values)
```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray \
"{data: [
  -0.0470,-0.0999,0.1524,   -0.0403,-0.0144,0.2465,   -0.0135,-0.0144,0.2640,    0.0133,-0.0309,0.2391,    0.0401,-0.0144,0.2310,
   0.0471,-0.1002,0.1523,    0.0403,-0.0144,0.2465,    0.0135,-0.0144,0.2640,   -0.0133,-0.0144,0.2465,   -0.0401,-0.0144,0.2310
]}"
```

### (B) Left index fingertip press-down (example), others keep near home
```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray \
"{data: [
  -0.0470,-0.0999,0.1524,   -0.0403,-0.0144,0.2365,   -0.0135,-0.0144,0.2640,    0.0133,-0.0309,0.2391,    0.0401,-0.0144,0.2310,
   0.0471,-0.1002,0.1523,    0.0403,-0.0144,0.2465,    0.0135,-0.0144,0.2640,   -0.0133,-0.0144,0.2465,   -0.0401,-0.0144,0.2310
]}"
```

### (C) Both thumbs forward/down test
```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray \
"{data: [
  -0.0400,-0.0850,0.1450,   -0.0403,-0.0144,0.2465,   -0.0135,-0.0144,0.2640,    0.0133,-0.0309,0.2391,    0.0401,-0.0144,0.2310,
   0.0400,-0.0850,0.1450,    0.0403,-0.0144,0.2465,    0.0135,-0.0144,0.2640,   -0.0133,-0.0144,0.2465,   -0.0401,-0.0144,0.2310
]}"
```

> If hand IK fails for a fingertip target, reduce displacement and move gradually (task space is limited for hand kinematics).

---

## 8) Hand Contact Force Handling in v16 (`/isaac_contact_states`)

## 8.1 Topic format (source)
- Topic: `/isaac_contact_states`
- Type: `std_msgs/msg/Float32MultiArray`
- Current Action Graph publishes **10 scalar values** (hand fingertip contact channels)

## 8.2 How v16 monitor uses it
- Arm force:
  - always shown as `(0,0,0)` for now
- Hand force:
  - `Fx = 0`
  - `Fy = 0`
  - `Fz = mapped contact scalar`

## 8.3 Why mapping logic exists
You observed a real mismatch:
- physically pressing **left ring** finger
- monitor initially showed force on **left index**

v16 fixes this by applying a mapping/remap in the hand contact callback path so the displayed finger label matches the actual contacted finger.

> If you later reorder the Action Graph outputs, update the mapping in the callback accordingly.

## 8.4 Debugging tips
```bash
# Confirm raw contact values are coming in
ros2 topic echo /isaac_contact_states

# Publish a test packet manually (Float32)
ros2 topic pub --once /isaac_contact_states std_msgs/msg/Float32MultiArray "{data: [0, 10, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

---

## 9) v15 → v16 Change Summary

### Added / Changed in v16
- Isaac Action Graph contact topic integration standardized to **Float32**
- Hand contact callback semantics clarified (hand-only source topic)
- Finger contact index mapping fixed (ring/index display mismatch resolved)
- Hand print order updated to **BABY → RING → MIDL → INDX → THMB**
- v16 home pose reference updated in README (arm + hand + monitor snapshot)

### Preserved from v15
- `DeltaArmPositionCallback()` (relative arm Cartesian control)
- Split forward arm/hand joint topics
- Existing arm FK/IK conventions and patched frame behavior

---

## 10) Quick Checklist (v16)

### Build / run
```bash
cd ~/dualarm_ws
colcon build --packages-select dualarm_forcecon
source install/setup.bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

### Mode / topics
- Control mode service: `/change_control_mode` (`Trigger`)
- Forward arm joints: `/forward_arm_joint_targets` (`Float64MultiArray`, 12)
- Forward hand joints: `/forward_hand_joint_targets` (`Float64MultiArray`, 30/40)
- Inverse arm absolute: `/target_arm_cartesian_pose` (`Float64MultiArray`, 12)
- Inverse arm delta: `/delta_arm_cartesian_pose` (`Float64MultiArray`, 12)
- Inverse hand fingertips: `/target_hand_fingertips` (`Float64MultiArray`, 30)
- Hand contact monitor: `/isaac_contact_states` (`Float32MultiArray`, 10)

### Common sanity checks
- Confirm monitor mode before sending command
- Start from home pose before large moves
- For IK failures:
  - reduce position delta
  - keep orientation near reachable range
  - move in smaller steps (especially for hand fingertips)
- If contact finger label looks wrong after Action Graph edits:
  - check callback mapping vs Action Graph output order

---

## 11) Suggested Next Steps for v16+ (optional)

- Add explicit parameter/documentation for `/isaac_contact_states` channel ordering (and mapping table)
- Add a debug print option for raw contact array + mapped finger labels
- Add contact topic QoS/type checks at startup (warn if wrong type connected)
- Add optional arm force topic integration (if you later publish arm wrench/contact data separately)
- Update monitor header version string to `v16` everywhere (if any print path still shows old version tag)

---

## 12) Notes

- This README intentionally reflects your **current v16 baseline behavior** and the **latest home pose snapshot you provided**.
- If you change:
  - home pose
  - Action Graph output order
  - IK frame conventions
  - topic names  
  then update the corresponding sections in this README together to avoid confusion during future handoff (v17+).

