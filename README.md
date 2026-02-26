# dualarm_forcecon v15 README

> **Version:** v15  
> **Key additions vs v14:** `DeltaArmPositionCallback()` (inverse mode, relative Cartesian arm motion)  
> **This README reflects the updated home pose snapshot provided on 2026-02-26 and fixes topic-name mismatches from the previous README (e.g., `/forward_arm_joint_targets`, `/forward_hand_joint_targets`).**

---

## 1) Overview

`dualarm_forcecon` is a ROS 2 package for **dual-arm + dual-hand control** in Isaac Sim (and compatible pipelines), supporting:

- **Forward mode** (joint-space command)
  - Arm joint targets (left/right split command in one topic)
  - Hand joint targets (left/right hand targets in one topic, 15DoF or 20DoF-per-hand format)
- **Inverse mode** (task-space command)
  - Arm Cartesian pose target (`TargetArmPositionCallback`)
  - Hand fingertip Cartesian target (`TargetHandPositionCallback`)
  - **Arm Cartesian delta target (`DeltaArmPositionCallback`)** ← **v15**
- Live monitoring of:
  - Current/target arm pose
  - Fingertip positions (hand base frame)
  - Contact forces (arm + finger-level buffers)

This version also preserves the frame-consistency fixes from v14 (FK/IK z mismatch issues around base/world handling).

---

## 2) Package Structure (v15 baseline)

> **Do not change the package tree / file-role separation rules** (handoff baseline preserved).

```text
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
    ├── DualArmForceControl.cpp        # ctor/dtor/ControlLoop only (rule)
    ├── DualArmForceControl.h
    ├── node_dualarm_main.cpp
    └── states_callback_dualarm.cpp    # callbacks (including target callbacks)
```

### File-role rules (important)

- `DualArmForceControl.cpp`
  - **Only** constructor / destructor / `ControlLoop()`
- `states_callback_dualarm.cpp`
  - All callback implementations (`JointsCallback`, `PositionCallback`, target callbacks, mode callback, print helpers)
- Kinematics headers remain separated in `include/dualarm_forcecon/Kinematics/`

---

## 3) Core Features by Mode

### A. Idle mode
- Targets are synchronized to current joints once (`idle_synced_`) to prevent jumps.

### B. Forward mode
- **Arm joint command callback:** `TargetArmJointsCallback`
- **Hand joint command callback:** `TargetHandJointsCallback`
- Arm and hand can be moved **independently** using separate topics.

### C. Inverse mode
- **Arm absolute Cartesian target:** `TargetArmPositionCallback`
- **Hand fingertip Cartesian target:** `TargetHandPositionCallback`
- **Arm relative Cartesian delta target:** `DeltaArmPositionCallback` (**v15**)

---

## 4) Topics (v15, corrected names)

## 4.1 Subscribed topics

### State inputs
- `/isaac_joint_states` (`sensor_msgs/msg/JointState`)
  - Used by both `JointsCallback` and `PositionCallback`
- `/isaac_contact_states` (`std_msgs/msg/Float64MultiArray`)

### Inverse mode targets
- `/target_arm_cartesian_pose` (`std_msgs/msg/Float64MultiArray`)
  - `TargetArmPositionCallback`
- `/target_hand_fingertips` (`std_msgs/msg/Float64MultiArray`)
  - `TargetHandPositionCallback`
- **`/delta_arm_cartesian_pose`** (`std_msgs/msg/Float64MultiArray`) ← **v15**
  - `DeltaArmPositionCallback`
  - Relative Cartesian delta command (left + right arm)

### Forward mode targets (split)
- `/forward_arm_joint_targets` (`std_msgs/msg/Float64MultiArray`)
  - `TargetArmJointsCallback`
- `/forward_hand_joint_targets` (`std_msgs/msg/Float64MultiArray`)
  - `TargetHandJointsCallback`

> ✅ Previous README mismatch fixed: **NOT** `/target_arm_joint_targets` / `/target_hand_joint_targets`

## 4.2 Published topics
- `/isaac_joint_command` (`sensor_msgs/msg/JointState`)

## 4.3 Service
- `/change_control_mode` (`std_srvs/srv/Trigger`)

---

## 5) Message Formats (important)

## 5.1 Forward arm joints (`/forward_arm_joint_targets`)

**Length = 12**

```text
[L_arm(6), R_arm(6)]
= [L_j1..L_j6, R_j1..R_j6]
```

> Note: This order is **not** the same as `/isaac_joint_states` name order because Isaac state list contains extra joints like `yaw_joint`, `pitch_joint` in between.

---

## 5.2 Forward hand joints (`/forward_hand_joint_targets`)

### Supported formats (v14/v15 split-hand callback style)
- **30 values** = left 15DoF + right 15DoF
- **40 values** = left 20DoF + right 20DoF

### 15DoF per hand layout (recommended for human-readable commands)

```text
[thumb1, thumb2, thumb3,
 index1, index2, index3,
 middle1, middle2, middle3,
 ring1, ring2, ring3,
 baby1, baby2, baby3]
```

For `/forward_hand_joint_targets` (30 values):

```text
[L_hand15(15), R_hand15(15)]
```

### 20DoF per hand layout
```text
[thumb1..4, index1..4, middle1..4, ring1..4, baby1..4]
```

- `joint4` is mimic/canonicalized (`q4 = q3`) internally where needed.

---

## 5.3 Inverse arm absolute pose (`/target_arm_cartesian_pose`)

**Length = 12**

```text
[L x y z r p y,  R x y z r p y]
```

- Position unit: **meters**
- Orientation unit: follows `ik_angle_unit_` (`rad` default in v14/v15 snippets)
- Euler convention: follows `ik_euler_conv_` (`rpy` default)
- Frame handling follows `ik_targets_frame_` and v14 frame-consistency patch behavior

---

## 5.4 Inverse arm delta pose (`/delta_arm_cartesian_pose`)  ← v15

**Length = 12**

```text
[L dx dy dz droll dpitch dyaw,  R dx dy dz droll dpitch dyaw]
```

### Behavior (v15)
- Applies a **relative delta** to the **current arm pose**
- Conceptually:
  - `target_pos = current_pos + dpos`
  - `target_euler = current_euler + deuler`
- Then the resulting absolute pose is solved through the same arm IK path (inverse mode)

### Notes
- Position delta unit: **meters**
- Orientation delta unit: follows `ik_angle_unit_` (default `rad`)
- Should be used in **inverse mode**
- Small increments are recommended (e.g., 1–30 mm, a few degrees) for stable IK behavior

---

## 5.5 Inverse hand fingertips (`/target_hand_fingertips`)

Expected as fingertip Cartesian targets in **hand base frame** (left/right each 5 fingertips × xyz).

Canonical fingertip order used in monitoring/kinematics:

```text
thumb, index, middle, ring, baby
```

So the common 30-value layout is:

```text
[L_thumb xyz, L_index xyz, L_middle xyz, L_ring xyz, L_baby xyz,
 R_thumb xyz, R_index xyz, R_middle xyz, R_ring xyz, R_baby xyz]
```

---

## 6) Home Pose (v15 snapshot, updated)

This section reflects the **new home pose** you provided (latest `/isaac_joint_states` + monitor print), replacing the older v14 home reference.

## 6.1 Home pose — arm joints (from `/isaac_joint_states`)

### Left arm (rad)
- `L = [1.4669, 0.0090, -2.4415, 1.4441, 1.4924, 0.0699]`

### Right arm (rad)
- `R = [-0.0135, 0.7709, 1.9998, 0.0000, -1.1869, -0.7854]`

### Combined arm forward command order (12)
```text
[L_j1..L_j6, R_j1..R_j6]
```

```text
[1.4669, 0.0090, -2.4415, 1.4441, 1.4924, 0.0699,
 -0.0135, 0.7709, 1.9998, 0.0000, -1.1869, -0.7854]
```

## 6.2 Home pose — arm Cartesian monitor values (from monitor print)

> These are the values shown in `Dual Arm & Hand Monitor`.

### Left arm (current/target at home)
- Position `[m]` = `(0.1513, 0.2701, -0.1461)`
- Euler `[deg]` = `(159.93, 87.07, -70.54)`

### Right arm (current/target at home)
- Position `[m]` = `(0.5414, -0.3094, 0.1513)`
- Euler `[deg]` = `(-107.06, 88.92, -162.94)`

## 6.3 Home pose — hand joints (15DoF format, practical command baseline)

### Left hand 15DoF (rad)
Format:
```text
[thumb1,thumb2,thumb3, index1,index2,index3, middle1,middle2,middle3, ring1,ring2,ring3, baby1,baby2,baby3]
```

```text
[0.0000, 0.3936, 0.3927,
 0.0000,-0.0005, 0.0000,
 0.0000,-0.0004, 0.0000,
 0.0000,-0.0002, 0.7854,
 0.0000,-0.0005, 0.0000]
```

### Right hand 15DoF (rad)
```text
[0.0000, 0.3939, 0.3927,
 0.0000,-0.0001, 0.0000,
 0.0000,-0.0001, 0.0000,
 0.0000,-0.0001, 0.0000,
 0.0000,-0.0001, 0.0000]
```

## 6.4 Home pose — fingertip positions (monitor, hand base frame)

### Left hand (hand-base frame)
- THMB = `(-0.0471, -0.1000, 0.1524)`
- INDX = `(-0.0403, -0.0144, 0.2465)`
- MIDL = `(-0.0135, -0.0144, 0.2640)`
- RING = `( 0.0133, -0.0337, 0.2382)`
- BABY = `( 0.0401, -0.0144, 0.2310)`

### Right hand (hand-base frame)
- THMB = `( 0.0470, -0.1000, 0.1524)`
- INDX = `( 0.0403, -0.0144, 0.2465)`
- MIDL = `( 0.0135, -0.0144, 0.2640)`
- RING = `(-0.0133, -0.0144, 0.2465)`
- BABY = `(-0.0401, -0.0144, 0.2310)`

---

## 7) Example Commands (v15, home-pose based)

> All examples use `std_msgs/msg/Float64MultiArray`.  
> Make sure the node is in the correct mode (**forward** or **inverse**) before publishing.

## 7.1 Forward mode examples — Arm joints

### (A) Arm home pose (v15 updated)
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [1.4669, 0.0090, -2.4415, 1.4441, 1.4924, 0.0699,
         -0.0135, 0.7709, 1.9998, 0.0000, -1.1869, -0.7854]}"
```

### (B) Symmetric small arm motion from home (joint-space)
- Left arm slightly reaches forward/up, right arm compensates modestly
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [1.4200, -0.0200, -2.3500, 1.3500, 1.5500, 0.1000,
         0.0200,  0.8200,  1.9300, 0.0500, -1.1500, -0.7600]}"
```

### (C) Left arm only move, right arm stays at home
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [1.3000, 0.0500, -2.2000, 1.2000, 1.4500, 0.2000,
         -0.0135, 0.7709, 1.9998, 0.0000, -1.1869, -0.7854]}"
```

---

## 7.2 Forward mode examples — Hand joints (split callback)

### (A) Hand home pose (30 values = left15 + right15)
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0000,0.3936,0.3927,  0.0000,-0.0005,0.0000,  0.0000,-0.0004,0.0000,  0.0000,-0.0002,0.7854,  0.0000,-0.0005,0.0000,
  0.0000,0.3939,0.3927,  0.0000,-0.0001,0.0000,  0.0000,-0.0001,0.0000,  0.0000,-0.0001,0.0000,  0.0000,-0.0001,0.0000
]}"
```

### (B) Requested-style gesture example
- **Right hand:** keep v15 home pose
- **Left hand:** thumb folds inward (thumb2/3 flex), ring bends ~90° (`ring3 ≈ 1.5708`), others ≈ 0

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0000,0.7000,0.7000,   0.0000,0.0000,0.0000,   0.0000,0.0000,0.0000,   0.0000,0.0000,1.5708,   0.0000,0.0000,0.0000,
  0.0000,0.3939,0.3927,   0.0000,-0.0001,0.0000,  0.0000,-0.0001,0.0000,  0.0000,-0.0001,0.0000,  0.0000,-0.0001,0.0000
]}"
```

### (C) Both hands fully open (approx)
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0,
  0,0,0,  0,0,0,  0,0,0,  0,0,0,  0,0,0
]}"
```

---

## 7.3 Inverse mode examples — Absolute arm Cartesian pose

### (A) Arm home pose via monitor values (rad Euler)
Using monitor home pose and converting Euler deg → rad:
- Left Euler rad ≈ `(2.7913, 1.5197, -1.2312)`
- Right Euler rad ≈ `(-1.8685, 1.5519, -2.8438)`

```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.1513,  0.2701, -0.1461,   2.7913, 1.5197, -1.2312,
  0.5414, -0.3094,  0.1513,  -1.8685, 1.5519, -2.8438
]}"
```

### (B) Small absolute Cartesian perturbation around home
- Left arm +2 cm x, -1 cm y, +2 cm z
- Right arm -1 cm x, +1 cm y, unchanged z
```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.1713,  0.2601, -0.1261,   2.7913, 1.5197, -1.2312,
  0.5314, -0.2994,  0.1513,  -1.8685, 1.5519, -2.8438
]}"
```

> If IK fails, reduce displacement and/or orientation change.

---

## 7.4 Inverse mode examples — Delta arm Cartesian pose (v15)

### (A) Simple relative move (translation only)
- Left: `(+3 cm, -3 cm, +2 cm)`
- Right: no change
```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.03, -0.03, 0.02,   0.0, 0.0, 0.0,
  0.00,  0.00, 0.00,   0.0, 0.0, 0.0
]}"
```

### (B) Relative move with small orientation delta (rad)
- Left: +1 cm z and +10° yaw (`~0.1745 rad`)
- Right: -1 cm z and -10° yaw
```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.00, 0.00, 0.01,   0.0, 0.0,  0.1745,
  0.00, 0.00,-0.01,   0.0, 0.0, -0.1745
]}"
```

### (C) Very small jogging command (safe tuning / teleop style)
```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.005, 0.000, 0.000,   0.000, 0.000, 0.000,
 -0.005, 0.000, 0.000,   0.000, 0.000, 0.000
]}"
```

**Recommended practice for delta mode**
- Start with **5–10 mm** translation steps
- Start with **≤ 5°** orientation steps (`0.087 rad`)
- Avoid large simultaneous translation + rotation jumps in one command

---

## 7.5 Inverse mode example — Hand fingertip Cartesian target (relative-to-home style example)

The exact hand IK feasibility depends on reachable fingertip workspace. Start from home fingertip positions and apply **small** deltas.

Example below:
- Left index fingertip slightly down (`z - 0.01`)
- Others remain near home
- Right hand unchanged at home

```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray \
"{data: [
  -0.0471,-0.1000,0.1524,   -0.0403,-0.0144,0.2365,   -0.0135,-0.0144,0.2640,    0.0133,-0.0337,0.2382,    0.0401,-0.0144,0.2310,
   0.0470,-0.1000,0.1524,    0.0403,-0.0144,0.2465,    0.0135,-0.0144,0.2640,   -0.0133,-0.0144,0.2465,   -0.0401,-0.0144,0.2310
]}"
```

If the requested fingertip target is outside the reachable hand workspace, hand IK may fail (or only partially update depending on implementation).

---

## 8) Known Notes / Practical Tips

## 8.1 Arm FK/IK frame consistency (v14 carried into v15)
- v14 patched `arm_forward_kinematics.hpp` and `arm_inverse_kinematics.hpp` to reduce target/current z mismatch and frame confusion.
- Keep `world_base_xyz` / `world_base_euler_xyz_deg` consistent with your Isaac scene setup.
- Monitor readout in your latest logs indicates the arm display is aligned with the current FK output convention used in code.

## 8.2 Hand workspace is much smaller than arm workspace
- Hand task-space IK is **far more limited** than arm IK.
- Use small fingertip deltas around a known valid pose (usually home pose).
- Prefer staged commands rather than large jumps.

## 8.3 Hand mimic joints (`joint4`)
- Hand joint4 values are mimic-linked to joint3 in the kinematics/model path.
- In 20DoF formats, `q4` may be canonicalized to `q3` internally.

## 8.4 `/isaac_joint_states` includes extra non-arm joints
- `yaw_joint`, `pitch_joint` appear in the state list.
- Do **not** copy raw state order directly into `/forward_arm_joint_targets`.

---

## 9) v14 → v15 Change Summary

### Added
- `DeltaArmPositionCallback()`
  - Relative arm Cartesian control in inverse mode
  - Enables jog/teleop-like incremental motion commands

### Preserved from v14
- Split forward callbacks:
  - `TargetArmJointsCallback`
  - `TargetHandJointsCallback`
- Arm FK/IK frame consistency patches (target/current z mismatch mitigation)
- Separate arm/hand inverse callbacks
- Hand FK/IK setup and fingertip monitoring

### README corrections in this v15 document
- Updated **home pose** (new snapshot)
- Corrected **topic names** to actual code-style names (`/forward_*`)
- Added **delta arm examples** and usage notes

---

## 10) Quick Command Checklist (copy-friendly)

### Forward mode
- Arm joints: `/forward_arm_joint_targets` (12)
- Hand joints: `/forward_hand_joint_targets` (30 or 40)

### Inverse mode
- Arm absolute pose: `/target_arm_cartesian_pose` (12)
- Hand fingertips: `/target_hand_fingertips` (30)
- **Arm delta pose (v15): `/delta_arm_cartesian_pose` (12)**

---

## 11) Suggested next steps for v15+ (optional)

- Add command rate limiting / smoothing for delta-arm mode
- Add per-arm enable mask in delta callback (left-only/right-only without zeros)
- Add explicit frame field (base/world) in message protocol to eliminate ambiguity
- Add hand IK target validation + clamp (workspace-aware pre-check)

---

If you want, I can also prepare a **v15 command cheat sheet** (one-page quick reference) with only the topic formats + common examples.
