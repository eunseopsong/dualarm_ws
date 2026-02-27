# DualArmForceControl README (v18)

This README summarizes the current **v18** baseline of `dualarm_forcecon` (dual arm + hand monitor / control node) based on your latest code flow, runtime monitor outputs, and v17-to-v18 fixes.

---

## 1) Version Summary (v18)

v18 builds on **v17** (hand contact force mapping + force monitor improvements + forcecon skeleton) and adds **control/monitor stability fixes**, **callback file split**, and **hand-axis convention fixes**.

### v18 major changes
- **Callback source file split (important package structure update)**
  - `states_current_callback_dualarm.cpp` (current state / monitor-related callbacks)
  - `states_target_callback_dualarm.cpp` (target / command callbacks)
- **Forcecon mode added to control mode cycle**
  - Mode cycle is now: `idle -> forward -> inverse -> forcecon -> idle`
- **Hand target force print issue fixed (TAR_F visible)**
  - `HandContactForceCallback()` no longer clears hand target-force buffers every sensor callback
  - `TargetHandForceCallback()` updates target-force monitor values early so TAR_F remains visible even if controller/IK step fails
- **ControlModeCallback target-force clearing policy refined**
  - Hand target force is preserved in `forcecon`
  - Hand target force is cleared only when leaving forcecon (`idle/forward/inverse`)
- **New inverse hand delta command path**
  - `DeltaHandPositionCallback()` added (single finger Cartesian jogging in hand base frame)
- **Hand FK axis convention fix (root cause fix)**
  - `hand_forward_kinematics.hpp` now remaps fingertip positions / rotations to the intended hand-base display convention
  - `HandPositionCallback` can remain unchanged
  - Temporary delta-axis compensation in `DeltaHandPositionCallback` was removed after FK-side fix
- **v18 baseline status for forcecon**
  - Target force monitor (`TAR_F`) displays correctly
  - Basic forcecon command path runs, but force tracking performance still needs parameter tuning (`hand_admittance_control.hpp`)

### v17 features preserved in v18
- `/isaac_contact_states` hand contact parsing (Float32MultiArray, 10 scalars)
- Finger contact index mapping fix (observed ActionGraph order support)
- Hand force frame conversion (sensor -> tip -> hand base + empirical correction)
- `hand_admittance_control.hpp` integrated and building
- `TargetHandForceCallback()` and forcecon mode infrastructure

---

## 2) Package Structure (v18, must preserve)

> Keep the package tree and file-role separation rules unchanged.

```text
dualarm_forcecon/
├── CMakeLists.txt
├── package.xml
├── include/dualarm_forcecon/Kinematics/
│   ├── arm_forward_kinematics.hpp
│   ├── arm_inverse_kinematics.hpp
│   ├── hand_admittance_control.hpp
│   ├── hand_forward_kinematics.hpp
│   ├── hand_inverse_kinematics.hpp
│   └── kinematics_utils.hpp
└── src/
    ├── DualArmForceControl.cpp                 # ctor / dtor / ControlLoop only (rule)
    ├── DualArmForceControl.h
    ├── node_dualarm_main.cpp
    ├── states_current_callback_dualarm.cpp     # current-state callbacks + monitor print
    └── states_target_callback_dualarm.cpp      # target/command callbacks
```

### Important rules (must keep)
- **Do not change package tree**
- **Do not merge callbacks back into `DualArmForceControl.cpp`**
- `DualArmForceControl.cpp` should keep only:
  - constructor
  - destructor
  - `ControlLoop()`
- Preserve:
  - 52-DOF publish mapping
  - Isaac UI Euler convention used in monitor
  - world-base z offset default behavior (0.306 m unless parameterized)
  - `PrintDualArmStates` formatting style (colors/layout)
  - include/src separation

---

## 3) Core Functional Overview

### 3.1 Control modes (v18)
- `idle`
- `forward`
- `inverse`
- `forcecon`

### 3.2 What each mode does
- **idle**
  - Safe hold / one-shot sync of targets to current states
- **forward**
  - Joint-space command mode
  - Arm and hand joint targets are accepted via separate topics
- **inverse**
  - Cartesian command mode
  - Arm absolute Cartesian pose
  - Arm delta Cartesian pose
  - Hand absolute fingertip Cartesian targets
  - Hand delta fingertip Cartesian jogging (single finger)
- **forcecon**
  - Hand fingertip force-oriented control mode (currently single-finger command path)
  - Uses `hand_admittance_control.hpp` + hand IK
  - Monitor displays hand target force (`TAR_F`) correctly after v18 patch

### 3.3 Monitor output (v18)
- Arm:
  - current/target pose in display frame
  - arm force monitor still zero-filled unless a separate arm wrench source is integrated
- Hand:
  - fingertip current/target positions in `LEFT_HAND_BASE` / `RIGHT_HAND_BASE`
  - hand current force from `/isaac_contact_states` (mapped and transformed)
  - hand target force from `/target_hand_force` (kept visible in forcecon)

---

## 4) ROS Interfaces (v18)

## 4.1 Subscriptions

### Joint / state input
- `/isaac_joint_states` — `sensor_msgs/msg/JointState`
  - Used by:
    - `JointsCallback`
    - `PositionCallback`

### Hand contact monitor input
- `/isaac_contact_states` — `std_msgs/msg/Float32MultiArray`
  - **10 values** (5 left + 5 right)
  - Scalar fingertip contact values (ActionGraph order observed and remapped internally)

### Forward mode (joint-space)
- `/forward_arm_joint_targets` — `std_msgs/msg/Float64MultiArray`
  - **12 values** = `[left arm 6, right arm 6]`
- `/forward_hand_joint_targets` — `std_msgs/msg/Float64MultiArray`
  - **30** = `left15 + right15`
  - **40** = `left20 + right20` (mimic canonicalization applied)
  - legacy-compat payloads may still be accepted by callback logic

### Inverse mode (Cartesian)
- `/target_arm_cartesian_pose` — `std_msgs/msg/Float64MultiArray`
  - **12 values**
  - `[L x y z r p y, R x y z r p y]`
- `/delta_arm_cartesian_pose` — `std_msgs/msg/Float64MultiArray`
  - **12 values**
  - `[L dx dy dz droll dpitch dyaw, R dx dy dz droll dpitch dyaw]`
- `/target_hand_fingertips` — `std_msgs/msg/Float64MultiArray`
  - **30 values** = `left(15) + right(15)`
  - per hand order = `THMB, INDX, MIDL, RING, BABY` (each xyz)
- `/delta_hand_fingertips` — `std_msgs/msg/Float64MultiArray`
  - **5 values** = `[side, finger, dx, dy, dz]`
  - `side`: `0=left`, `1=right`
  - `finger`: `0=thumb, 1=index, 2=middle, 3=ring, 4=baby`
  - delta expressed in the corrected hand-base display frame (after v18 FK fix)

### Forcecon mode input
- `/target_hand_force` — `std_msgs/msg/Float64MultiArray`
  - **8 values** = `[side, finger, p_des_x, p_des_y, p_des_z, f_des_x, f_des_y, f_des_z]`
  - `side`: `0=left`, `1=right`
  - `finger`: `0=thumb, 1=index, 2=middle, 3=ring, 4=baby`
  - `p_des`, `f_des` are in each hand base frame

## 4.2 Publisher
- `/isaac_joint_command` — `sensor_msgs/msg/JointState`
  - Consolidated arm + hand command published by `ControlLoop()`

## 4.3 Service
- `/change_control_mode` — `std_srvs/srv/Trigger`
  - Cycles mode in order:
    - `idle -> forward -> inverse -> forcecon -> idle`
  - Always verify active mode from monitor header

---

## 5) Coordinate / Frame Conventions (v18)

### 5.1 Arm pose display (monitor)
- `PrintDualArmStates()` arm pose is shown in the monitor display frame
- Euler display follows the project’s Isaac UI matching convention

### 5.2 Arm inverse input
- Arm inverse callbacks use your existing frame configuration fields:
  - `ik_targets_frame_`
  - `ik_euler_conv_`
  - `ik_angle_unit_`
- v18 keeps the earlier z-offset/frame consistency policy (no accidental double z-offset)

### 5.3 Hand pose display / command frame (important)
- Hand fingertip positions are displayed in:
  - `LEFT_HAND_BASE` frame (left hand)
  - `RIGHT_HAND_BASE` frame (right hand)
- In v18, **hand FK output axis order was corrected in `hand_forward_kinematics.hpp`**
  - `HandPositionCallback` remains unchanged
  - `DeltaHandPositionCallback` temporary axis remap workaround was removed after this root fix

### 5.4 Hand fingertip order: print vs command
- **Print order (monitor):** `BABY -> RING -> MIDL -> INDX -> THMB`
- **`/target_hand_fingertips` command order (per hand):** `THMB, INDX, MIDL, RING, BABY` (each xyz)

### 5.5 Hand contact force row order vs sensor order (important)
- **Observed sensor order per hand** (`/isaac_contact_states`): `[BABY, RING, MIDL, INDX, THMB]`
- **Internal canonical row order**: `THMB(0), INDX(1), MIDL(2), RING(3), BABY(4)`
- Monitor print uses print order, but force matrices are stored in canonical order internally

---

## 6) Home Pose Reference (v18 monitor snapshot)

> Values below are from your latest v18 monitor after the hand FK axis fix.

## 6.1 Arm monitor pose snapshot (CUR/TAR near home)

### Left arm
- `CUR  P[m,deg] = (0.1001,  0.2666, -0.1263 | 156.88, 86.30, -66.46)`
- `TAR  P[m,deg] = (0.1006,  0.2671, -0.1252 | 157.50, 86.39, -67.18)`

### Right arm
- `CUR  P[m,deg] = (0.5439, -0.3170,  0.1478 | -173.12, 89.18, -96.89)`
- `TAR  P[m,deg] = (0.5433, -0.3166,  0.1502 | -162.31, 89.45, -107.70)`

## 6.2 Hand fingertip positions at v18 baseline (after hand-FK axis fix)

### Left hand (print order, hand-base frame)
- `BABY = ( 0.2310,  0.0401, -0.0144 )`
- `RING = ( 0.2340,  0.0133, -0.0444 )`
- `MIDL = ( 0.2640, -0.0135, -0.0144 )`
- `INDX = ( 0.2465, -0.0403, -0.0144 )`
- `THMB = ( 0.1524, -0.0471, -0.1000 )`

### Right hand (print order, hand-base frame)
- `BABY = ( 0.2310, -0.0401, -0.0144 )`
- `RING = ( 0.2465, -0.0133, -0.0144 )`
- `MIDL = ( 0.2640,  0.0135, -0.0144 )`
- `INDX = ( 0.2465,  0.0403, -0.0144 )`
- `THMB = ( 0.1522,  0.0472, -0.1003 )`

### TargetHandPosition / target_hand_fingertips command-order versions (same snapshot)
Per-hand command order = `THMB, INDX, MIDL, RING, BABY`:

- **Left hand (15)**
  - `[-0?]` See exact command examples below (we list them explicitly in command order using corrected values)
- **Right hand (15)**
  - same note; use the examples in Section 7.6A

(We avoid duplicating a second representation here to reduce mismatches. Use the command blocks in Section 7.)

---

## 7) Build / Run / Mode Switching / Example Commands (v18)

## 7.1 Build and run node

```bash
cd ~/dualarm_ws
colcon build --packages-select dualarm_forcecon
source install/setup.bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

## 7.2 Switch control mode (`/change_control_mode`)

```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

### Typical cycle (v18)
```bash
# idle -> forward
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"

# forward -> inverse
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"

# inverse -> forcecon
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"

# forcecon -> idle
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

> Always confirm the active mode from the monitor header before publishing commands.

---

## 7.3 Forward mode examples — Arm joints (`/forward_arm_joint_targets`)

> `std_msgs/msg/Float64MultiArray`, 12 values = `[left arm 6, right arm 6]`

### (A) Home-like arm command (example)
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  1.6419, -0.0058, -2.2344, 1.5607, 1.5936, -0.1104,
 -0.0459,  0.7361,  1.9809, 0.0000,-1.1869, -0.7854
]}"
```

### (B) Small symmetric joint jog
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  1.6819, -0.0258, -2.1944, 1.5307, 1.5736, -0.0704,
 -0.0859,  0.7561,  1.9409, 0.0300,-1.1669, -0.8254
]}"
```

### (C) Left arm move, right arm hold
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  1.7219,  0.0342, -2.1244, 1.4907, 1.5436, -0.0304,
 -0.0459,  0.7361,  1.9809, 0.0000,-1.1869, -0.7854
]}"
```

---

## 7.4 Forward mode examples — Hand joints (`/forward_hand_joint_targets`)

> `std_msgs/msg/Float64MultiArray`
> - 30 = `left15 + right15`
> - 40 = `left20 + right20` (joint4 values included; internal mimic canonicalization still applies)

### 15-DoF order (per hand, 30-format)
`[thumb1,2,3, index1,2,3, middle1,2,3, ring1,2,3, baby1,2,3]`

### (A) Home-like hand command (30 values)
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0000,0.3948,0.3927,   0.0000,0.0000,0.0000,   0.0000,0.0000,0.0000,   0.0000,-0.0400,0.7854,   0.0000,0.0000,0.0000,
  0.0000,0.3955,0.3840,   0.0000,0.0000,0.0000,   0.0000,0.0000,0.0000,   0.0000, 0.0000,0.0000,   0.0000,0.0000,0.0000
]}"
```

### (B) Gentle bilateral closing test (30 values)
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0,0.55,0.45,   0.0,0.25,0.20,   0.0,0.25,0.20,   0.0,0.30,0.25,   0.0,0.20,0.15,
  0.0,0.55,0.45,   0.0,0.25,0.20,   0.0,0.25,0.20,   0.0,0.25,0.20,   0.0,0.20,0.15
]}"
```

### (C) Left hand only gesture, right hold
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0,0.80,0.80,   0.0,0.00,0.00,   0.0,0.00,0.00,   0.0,0.00,1.00,   0.0,0.00,0.00,
  0.0,0.3955,0.3840,   0.0,0.00,0.00,   0.0,0.00,0.00,   0.0,0.00,0.00,   0.0,0.00,0.00
]}"
```

---

## 7.5 Inverse mode examples — Arm absolute Cartesian (`/target_arm_cartesian_pose`)

> `std_msgs/msg/Float64MultiArray`, 12 values = `[L x y z r p y, R x y z r p y]`

### (A) Absolute arm target near current baseline monitor values (rad example)
```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.1001,  0.2666, -0.1263,   2.7370, 1.5062, -1.1601,
  0.5439, -0.3170,  0.1478,  -3.0216, 1.5565, -1.6910
]}"
```

### (B) Small absolute Cartesian shift
```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.1201,  0.2566, -0.1163,   2.7370, 1.5062, -1.1601,
  0.5339, -0.3070,  0.1478,  -3.0216, 1.5565, -1.6910
]}"
```

### (C) Left arm only absolute move, right arm hold
```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [
  0.1400,  0.2500, -0.1100,   2.70, 1.50, -1.10,
  0.5439, -0.3170,  0.1478,  -3.0216, 1.5565, -1.6910
]}"
```

---

## 7.6 Inverse mode examples — Delta arm Cartesian (`/delta_arm_cartesian_pose`)

> `std_msgs/msg/Float64MultiArray`, 12 values

### (A) Zero delta (sanity)
```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0,0,0,0,0,0,  0,0,0,0,0,0]}"
```

### (B) Small symmetric position jog
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

---

## 7.7 Inverse mode examples — Hand absolute fingertip targets (`/target_hand_fingertips`)

> `std_msgs/msg/Float64MultiArray`, 30 values = `left15 + right15`
> per-hand order = `THMB, INDX, MIDL, RING, BABY` (each xyz)
>
> NOTE (v18): Values below use the **corrected hand-FK axis convention**.

### (A) Baseline-like hand fingertip target (from current v18 monitor snapshot)
```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray \
"{data: [
   0.1524,-0.0471,-0.1000,   0.2465,-0.0403,-0.0144,   0.2640,-0.0135,-0.0144,   0.2340, 0.0133,-0.0444,   0.2310, 0.0401,-0.0144,
   0.1522, 0.0472,-0.1003,   0.2465, 0.0403,-0.0144,   0.2640, 0.0135,-0.0144,   0.2465,-0.0133,-0.0144,   0.2310,-0.0401,-0.0144
]}"
```

### (B) Left ring fingertip +x small move (absolute target example, others hold)
```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray \
"{data: [
   0.1524,-0.0471,-0.1000,   0.2465,-0.0403,-0.0144,   0.2640,-0.0135,-0.0144,   0.2440, 0.0133,-0.0444,   0.2310, 0.0401,-0.0144,
   0.1522, 0.0472,-0.1003,   0.2465, 0.0403,-0.0144,   0.2640, 0.0135,-0.0144,   0.2465,-0.0133,-0.0144,   0.2310,-0.0401,-0.0144
]}"
```

### (C) Left baby fingertip -z small move (absolute target example)
```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray \
"{data: [
   0.1524,-0.0471,-0.1000,   0.2465,-0.0403,-0.0144,   0.2640,-0.0135,-0.0144,   0.2340, 0.0133,-0.0444,   0.2310, 0.0401,-0.0244,
   0.1522, 0.0472,-0.1003,   0.2465, 0.0403,-0.0144,   0.2640, 0.0135,-0.0144,   0.2465,-0.0133,-0.0144,   0.2310,-0.0401,-0.0144
]}"
```

> If hand IK fails or the result looks odd, reduce displacement and move gradually (e.g., 0.002 to 0.01 m increments).

---

## 7.8 Inverse mode examples — Delta hand fingertip jogging (`/delta_hand_fingertips`)  **(new in v18 flow)**

> `std_msgs/msg/Float64MultiArray`, 5 values = `[side, finger, dx, dy, dz]`
> - `side`: `0=left`, `1=right`
> - `finger`: `0=thumb, 1=index, 2=middle, 3=ring, 4=baby`
>
> v18 note:
> - Temporary axis remap in this callback was removed after the **hand FK axis root fix**.
> - Use the displayed hand-base axes directly now.

### (A) Left ring +x jog (single finger)
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.005, 0.000, 0.000]}"
```

### (B) Left ring -x jog
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [0, 3, -0.005, 0.000, 0.000]}"
```

### (C) Left ring +y jog
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.000, 0.005, 0.000]}"
```

### (D) Left ring +z jog
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.000, 0.000, 0.005]}"
```

### (E) Right index -y jog
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [1, 1, 0.000, -0.005, 0.000]}"
```

### (F) Right thumb +x / -z combined jog
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [1, 0, 0.003, 0.000, -0.003]}"
```

---

## 7.9 Forcecon mode examples — Hand force target (`/target_hand_force`)

> `std_msgs/msg/Float64MultiArray`, 8 values = `[side, finger, p_des_x, p_des_y, p_des_z, f_des_x, f_des_y, f_des_z]`
>
> Current v18 status:
> - TAR_F monitor displays target force correctly
> - Force tracking quality depends on `hand_admittance_control.hpp` tuning and contact conditions

### (A) Enter forcecon mode (from idle)
```bash
# idle -> forward
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
# forward -> inverse
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
# inverse -> forcecon
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

### (B) Left ring forcecon target (example used during debugging)
Using a v17/v18-era left ring target pose + desired force along Fx.

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 3, 0.0133, -0.0331, 0.2384, 10.0, 0.0, 0.0]}"
```

### (C) Left ring forcecon with streaming publish (example)
```bash
ros2 topic pub -r 10 /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 3, 0.0133, -0.0331, 0.2384, 8.0, 0.0, 0.0]}"
```

### (D) Right index forcecon target (example)
(Adjust pose using your current monitor values before real tests.)

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [1, 1, 0.2465, 0.0403, -0.0144, 0.0, 5.0, 0.0]}"
```

> Practical tip: For forcecon debugging, start with small force targets (e.g., 2-5 N), stable contact, and slow updates.

---

## 8) Hand Contact Force Handling (v18)

## 8.1 Topic format
- Topic: `/isaac_contact_states`
- Type: `std_msgs/msg/Float32MultiArray`
- Payload length: **10** (5 left + 5 right)

## 8.2 Observed ActionGraph order (per hand)
- `[BABY, RING, MIDL, INDX, THMB]`

## 8.3 v18 callback policy (important)
`HandContactForceCallback()` updates **current measured force only**:
- updates/reset:
  - `f_l_c_`, `f_r_c_` (arm current monitor force, currently zero)
  - `f_l_hand_c_`, `f_r_hand_c_` (hand current measured force)
- **does NOT reset target force buffers**:
  - `f_l_hand_t_`, `f_r_hand_t_`

This is the key fix that keeps `TAR_F` visible in forcecon.

## 8.4 Force frame processing
The hand contact callback performs:
1. scalar contact -> temporary sensor local force assumption (`+X`)
2. sensor -> tip frame conversion
3. tip -> hand base conversion (using `computeTipRotationsBase`)
4. empirical base correction matrix for display/control convention alignment

---

## 9) Current Known Status / Limitations (v18)

### 9.1 Fixed in v18
- TAR_F monitor not showing in forcecon (target force getting zeroed by sensor callback)
- Hand FK axis display mismatch root issue (fixed in `hand_forward_kinematics.hpp`)
- Delta hand callback no longer needs ad-hoc axis remap after FK fix

### 9.2 Still under tuning / development
- Force tracking performance in forcecon may not converge tightly to desired force yet
  - `hand_admittance_control.hpp` gains / sign / gating / axis selection need tuning
- Some large hand Cartesian targets may exceed IK reachability / joint limits

---

## 10) Quick Checklist (v18)

### Build / run
```bash
cd ~/dualarm_ws
colcon build --packages-select dualarm_forcecon
source install/setup.bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

### Confirm mode before command
- Watch the monitor header:
  - `Mode: [idle]`
  - `Mode: [forward]`
  - `Mode: [inverse]`
  - `Mode: [forcecon]`

### Topics to remember (v18)
- `/forward_arm_joint_targets`
- `/forward_hand_joint_targets`
- `/target_arm_cartesian_pose`
- `/delta_arm_cartesian_pose`
- `/target_hand_fingertips`
- `/delta_hand_fingertips`  **(new inverse hand delta jog)**
- `/target_hand_force`      **(forcecon)**
- `/isaac_contact_states`
- `/isaac_joint_states`
- `/isaac_joint_command`

### Common sanity checks
- If hand Cartesian motion looks wrong:
  - confirm you are using **v18 hand-FK axis-fixed build**
  - confirm no leftover temporary axis remap remains in delta-hand callback
- If force target is not visible in monitor:
  - check mode is `forcecon`
  - confirm target callback runs and `TAR_F` is updated
  - confirm sensor callback is not zeroing target buffers
- If IK fails:
  - reduce displacement / force target
  - move in smaller increments
  - ensure contact is stable before forcecon tests

---

## 11) Suggested Next Steps (v18+)

- Tune `hand_admittance_control.hpp` for stable force tracking
  - verify force error sign (`f_des - f_meas`)
  - test per-axis enable masks
  - tune mass/damping/stiffness and step limits
  - add explicit contact-gating policy per finger
- Add a compact raw contact debug print option (sensor-order + canonical-order)
- Add parameterized finger/contact mapping table (instead of hardcoded observed order)
- Document exact `/target_hand_force` pose references after final hand-FK axis convention is fully frozen

---

## 12) Notes

- This README reflects the **v18 baseline** after:
  - callback file split
  - forcecon TAR_F visibility fix
  - hand FK axis convention fix
  - delta hand position workflow integration
- If you later change:
  - hand FK axis remap
  - contact sensor ordering
  - forcecon gains / controller interface
  - topic names or payload formats
  update this README together to keep future handoff clean.
