# dualarm_forcecon — v11

This README documents **v11** of the `dualarm_forcecon` ROS 2 package (workspace: `~/dualarm_ws`).

v11 focus:
- Split **arm IK** and **hand IK** command topics.
- Add **hand inverse kinematics (pos-only)** using Pinocchio forward kinematics + numerical Jacobian + Damped Least Squares (DLS).
- Keep all v10 invariants (package tree, file roles, 52-DOF mapping, Isaac UI pose matching, print format/colors).

---

## Git commit (one-line message)

`v11: split arm/hand IK topics and add pos-only hand fingertip IK (DLS, Pinocchio FK)`

---

## Project / package layout (MUST NOT CHANGE)

**Do not add/remove/move files. Edit inside files only.**

```
dualarm_forcecon
├── CMakeLists.txt
├── package.xml
├── include
│   └── dualarm_forcecon
│       └── Kinematics
│           ├── arm_forward_kinematics.hpp
│           ├── arm_inverse_kinematics.hpp
│           ├── hand_forward_kinematics.hpp
│           ├── hand_inverse_kinematics.hpp          # v11
│           └── kinematics_utils.hpp
└── src
    ├── DualArmForceControl.cpp                      # constructor/destructor/ControlLoop ONLY
    ├── DualArmForceControl.h
    ├── node_dualarm_main.cpp                        # main() only
    └── states_callback_dualarm.cpp                  # all callbacks + PrintDualArmStates
```

---

## File role rules (STRICT)

### `src/DualArmForceControl.cpp`
Must contain **only**:
- `DualArmForceControl::DualArmForceControl(...)`
- `DualArmForceControl::~DualArmForceControl()`
- `DualArmForceControl::ControlLoop()`

No callbacks / helpers / print functions in this file.

### `src/states_callback_dualarm.cpp`
All subscriber callbacks and state updates live here, including:
- JointState parsing (arms + hands)
- Arm FK update (current/target)
- Target callbacks (arm IK / hand IK / forward joints)
- Console monitor printing

### Kinematics headers
Already validated behavior; do not break:
- 52-DOF mapping (12 arm + 40 hand)
- Isaac Sim UI-matching Euler / base alignment rules
- World-base z offset default ~`0.306 m` (where applied must not be changed)

---

## v11 ROS interfaces

### Subscriptions
- `/isaac_joint_states` (`sensor_msgs/msg/JointState`)  
  Used for current joints + FK updates.
- `/forward_joint_targets` (`std_msgs/msg/Float64MultiArray`)  
  **Forward mode** command (52-DOF).
- `/target_arm_cartesian_pose` (`std_msgs/msg/Float64MultiArray`)  
  **Inverse mode** arm IK command (12 values).
- `/target_hand_fingertips` (`std_msgs/msg/Float64MultiArray`)  
  **Inverse mode** hand IK command (pos-only, 30 values).
- `/isaac_contact_states` (`std_msgs/msg/Float64MultiArray`)  
  External force/state (optional).

### Publication
- `/isaac_joint_command` (`sensor_msgs/msg/JointState`)  
  Published joint position commands for Isaac Sim.

### Service
- `/change_control_mode` (`std_srvs/srv/Trigger`)  
  Cycles modes: `idle -> forward -> inverse -> idle`

---

## Build / run

### Build
```bash
cd ~/dualarm_ws
colcon build --symlink-install
source ~/dualarm_ws/install/setup.bash
```

(If you use alias `cb`, it should run the same build.)

### Run
```bash
source ~/dualarm_ws/install/setup.bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

---

## Mode switching (service)

The service toggles modes in sequence:
`idle -> forward -> inverse -> idle`

To go from `idle` to `inverse`:
```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

(Once more would return to `idle`.)

---

## FK monitoring (no command needed)

The console monitor prints:
- Arm: CUR/TAR pose (world), forces
- Hand: CUR/TAR fingertip positions in each HAND_BASE frame

If you want to confirm active command topics:
```bash
ros2 topic list | grep target
```

Expected (v11):
- `/forward_joint_targets`
- `/target_arm_cartesian_pose`
- `/target_hand_fingertips`

---

## Forward mode: joint targets (arms + hands)

### Enter forward mode
From `idle`:
```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

### Publish 52-DOF targets (example)
Format:
- `[0..5]`   left arm joints (6)
- `[6..11]`  right arm joints (6)
- `[12..31]` left hand joints (20)
- `[32..51]` right hand joints (20)

```bash
ros2 topic pub --once --qos-reliability best_effort /forward_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0,0,0,0,0,0,   0,0,0,0,0,0,
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0,
  0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0
]}"
```

---

## Inverse mode: ARM IK only (`/target_arm_cartesian_pose`)

### Enter inverse mode
From `idle`:
```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

### Message format (12 values)
`[Lx Ly Lz Lrx Lry Lrz  Rx Ry Rz Rrx Rry Rrz]`

- Position units: meters
- Euler units: depends on param `ik_angle_unit` (default typically `rad`)
- Euler convention depends on `ik_euler_conv` (kept as project standard)

Example:
```bash
ros2 topic pub --once --qos-reliability best_effort /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.5357,  0.2988,  0.4345,   2.801777, 1.301317, -1.720022,
  0.5371, -0.2991,  0.4355,  -2.796192, 1.301143, -1.481261
]}"
```

> Tip: If values look wrong, ensure you are in `inverse` mode and confirm `ik_angle_unit`:
```bash
ros2 param get /dualarm_forcecon_node ik_angle_unit
```

---

## Inverse mode: HAND IK only (`/target_hand_fingertips`)

Hand IK is **pos-only**. Targets are fingertip positions expressed in each HAND_BASE frame.

### Message format (30 values)
Order:
- Left hand: THMB, INDX, MIDL, RING, BABY (each xyz)  → 15 values
- Right hand: THMB, INDX, MIDL, RING, BABY (each xyz) → 15 values

`[L_THMB(x y z), L_INDX(x y z), L_MIDL(x y z), L_RING(x y z), L_BABY(x y z),  R_THMB..., R_BABY...]`

### Example: “home” (no motion) test
Use your printed CUR fingertip values as targets:
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

### Example: noticeable motion (finger curl / close)
This tends to be more feasible than trying to translate all tips together.
```bash
ros2 topic pub --once --qos-reliability best_effort /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
  -0.0510, -0.1128,  0.1000,
  -0.0403, -0.0151,  0.1800,
  -0.0135, -0.0151,  0.1900,
   0.0133, -0.0151,  0.1800,
   0.0401, -0.0151,  0.1700,

   0.0590, -0.1297,  0.1145,
   0.0403, -0.0144,  0.2465,
   0.0135, -0.0144,  0.2640,
  -0.0133, -0.0144,  0.2465,
  -0.0401, -0.0144,  0.2310
]}"
```

---

## Notes / known limitations (v11)

- Fingertip targets are currently the **link frame origins** of `*_link4_*` (thumb/index/middle/ring/baby).
  They are valid “EE frames” by definition, but may not coincide with the physical fingertip contact point.
- Hand IK uses numerical Jacobian (finite difference) for robustness without requiring analytic Jacobians.
  Expect small sensitivity differences between fingers and poses.

---

## Quick sanity checklist

- Build succeeds
- `ros2 topic list | grep target` shows:
  - `/target_arm_cartesian_pose`
  - `/target_hand_fingertips`
- Inverse mode:
  - arm-only publish moves arms
  - hand-only publish moves fingers (best with “curl/close” targets)
- Monitor prints keep v11 formatting and colors intact.
