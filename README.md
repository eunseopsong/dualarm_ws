# DualArm Force Control README (v30)

## Quick Start Example Commands

### 1. Build

```bash
cd ~/dualarm_ws
colcon build --symlink-install
source install/setup.bash
```

If using the custom alias:

```bash
cd ~/dualarm_ws
cb
```

---

### 2. Run controller node

```bash
source ~/dualarm_ws/install/setup.bash

ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml
```

The controller node name is:

```text
dualarm_forcecon_node
```

---

### 3. Change control mode

For Doosan/AIDIN dual-arm experiments with arm + hand:

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'forward'}"
```

For RBY1 arm-only experiments:

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'idle'}"
```

---

### 4. Send arm delta Cartesian command

Topic:

```text
/delta_arm_cartesian_pose
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Format:

```text
[L dx dy dz droll dpitch dyaw, R dx dy dz droll dpitch dyaw]
```

Units:

```text
position delta    : meter
orientation delta : radian
```

Example: right arm only, +z 0.1 m from the latched home/base pose.

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.000, 0.000, 0.000, 0.0, 0.0, 0.0,
         0.000, 0.000, 0.100, 0.0, 0.0, 0.0]}"
```

Example: right arm first moves in -y, then moves in +z while keeping the same -y offset.

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.000, 0.000, 0.000, 0.0, 0.0, 0.0,
         0.000, -0.200, 0.000, 0.0, 0.0, 0.0]}"

ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.000, 0.000, 0.000, 0.0, 0.0, 0.0,
         0.000, -0.200, 0.100, 0.0, 0.0, 0.0]}"
```

Important:

```text
/delta_arm_cartesian_pose is interpreted as a home-pose-relative absolute delta.
It is not an incremental delta from the previous command.
```

So, if the first command is:

```text
R = [0.0, -0.2, 0.0, 0, 0, 0]
```

and the next desired pose is 10 cm higher while keeping the same y offset, the next command must be:

```text
R = [0.0, -0.2, 0.1, 0, 0, 0]
```

not:

```text
R = [0.0, 0.0, 0.1, 0, 0, 0]
```

---

### 5. Send hand forward command for Doosan/AIDIN setup

RBY1 v30 is currently arm-only in this controller, so this hand command is mainly for the Doosan/AIDIN dual-arm + hand setup.

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0, 0.0,
  0.2, 0.3, 0.3, 0.3,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,

  0.0, 0.0, 0.0, 0.0,
  0.2, 0.3, 0.3, 0.3,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

### 6. Send selected-finger target force for Doosan/AIDIN setup

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 1, 0.0, 0.0, 5.0]}"
```

Format:

```text
[hand_id, finger_id, fx, fy, fz]
```

where:

```text
hand_id   : 0 = left, 1 = right
finger_id : 0 = thumb, 1 = index, 2 = middle, 3 = ring, 4 = baby
```

---

# 1. Version Summary

## 1.1 v30 purpose

`v30` is the version where the same `dualarm_forcecon` controller is used with two different robot kinematics profiles:

```text
Doosan/AIDIN dual-arm
RBY1 arm-only
```

The controller package remains common, but the FK/IK configuration is selected through robot-specific YAML files.

The main controller role is:

```text
dualarm_forcecon
= ROS 2 node, mode handling, topic/service interface, arm command publishing,
  hand forward/inverse handling, hand force/admittance control, monitor printing
```

The reusable kinematics role is:

```text
dualarm_kinematics
= FK/IK implementation, robot kinematics YAML profiles, joint/link mapping
```

---

## 1.2 Recommended v30 modes

### Doosan/AIDIN dual-arm

```text
arm_mode  = inverse
hand_mode = forward
```

Use this when controlling both arms and hands.

### RBY1

```text
arm_mode  = inverse
hand_mode = idle
```

Use this because the current RBY1 profile in v30 is arm-only and `hand.enabled=false`.

---

# 2. Package Structure

The expected workspace layout is:

```text
dualarm_ws/
└── src/
    ├── dualarm_forcecon_interfaces/
    │   └── srv/
    │       └── SetControlMode.srv
    │
    ├── dualarm_kinematics/
    │   ├── config/
    │   │   ├── doosan_dualarm_kinematics.yaml
    │   │   ├── generic_dualarm_kinematics.yaml
    │   │   └── rby1_kinematics.yaml
    │   └── include/
    │       └── dualarm_kinematics/
    │           ├── arm/
    │           │   ├── arm_forward_kinematics.hpp
    │           │   └── arm_inverse_kinematics.hpp
    │           ├── hand/
    │           │   ├── hand_forward_kinematics.hpp
    │           │   └── hand_inverse_kinematics.hpp
    │           └── core/
    │               ├── kinematics_config.hpp
    │               └── kinematics_utils.hpp
    │
    └── dualarm_forcecon/
        ├── include/
        │   └── dualarm_forcecon/
        │       └── control/
        │           └── hand_admittance_control.hpp
        ├── src/
        │   ├── DualArmForceControl.cpp
        │   ├── DualArmForceControl.h
        │   ├── node_dualarm_main.cpp
        │   ├── states_current_callback_dualarm.cpp
        │   └── states_target_callback_dualarm.cpp
        └── yaml/
            └── forcecon_cfg.yaml
```

---

# 3. Kinematics Profile Selection

## 3.1 Main controller YAML

The runtime controller is launched with:

```bash
-p forcecon_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml
```

Inside `forcecon_cfg.yaml`, the active robot profile should point to the desired kinematics YAML.

v30 supports robot-specific kinematics profile selection, for example:

```yaml
kinematics:
  active_robot_id: 0
  profiles:
    - id: 0
      name: doosan_dualarm
      config_yaml: /home/eunseop/dualarm_ws/src/dualarm_kinematics/config/doosan_dualarm_kinematics.yaml

    - id: 1
      name: rby1
      config_yaml: /home/eunseop/dualarm_ws/src/dualarm_kinematics/config/rby1_kinematics.yaml
```

You can also override the kinematics YAML directly at launch time:

```bash
ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml \
  -p kinematics_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_kinematics/config/rby1_kinematics.yaml
```

---

# 4. Doosan/AIDIN vs RBY1 Configuration Differences

## 4.1 Summary table

| Item | Doosan/AIDIN dual-arm | RBY1 |
|---|---|---|
| Profile name | `doosan_aidin` | `rby1_arm_only` |
| Main use in v30 | Arm + hand control | Arm-only control |
| Arm DOF per side | 6 DOF | 7 DOF |
| Arm base link | `base_link` | `link_torso_5` |
| Left tip link | `left_link_6` | `ee_left` |
| Right tip link | `right_link_6` | `ee_right` |
| Left arm joint names | `left_joint_1` ~ `left_joint_6` | `left_arm_0` ~ `left_arm_6` |
| Right arm joint names | `right_joint_1` ~ `right_joint_6` | `right_arm_0` ~ `right_arm_6` |
| Hand config | enabled | disabled |
| Hand base links | `left_joint_6`, `right_joint_6` | not used |
| Recommended mode | `arm=inverse`, `hand=forward` | `arm=inverse`, `hand=idle` |
| Practical caution | Standard 6-DOF arm IK | Redundant 7-DOF arm; workspace/manipulability matters |

---

## 4.2 Doosan/AIDIN profile

The Doosan/AIDIN profile uses:

```yaml
robot_kinematics:
  profile: doosan_aidin

  arm:
    base_link: base_link
    left_tip_link: left_link_6
    right_tip_link: right_link_6

    left_joint_names:
      - left_joint_1
      - left_joint_2
      - left_joint_3
      - left_joint_4
      - left_joint_5
      - left_joint_6

    right_joint_names:
      - right_joint_1
      - right_joint_2
      - right_joint_3
      - right_joint_4
      - right_joint_5
      - right_joint_6

  hand:
    enabled: true
    left_base_link: left_joint_6
    right_base_link: right_joint_6
```

This means the controller expects:

```text
left arm  : 6 joints
right arm : 6 joints
left hand : enabled
right hand: enabled
```

The Doosan/AIDIN profile is the better fit for the full arm + hand pipeline:

```text
/delta_arm_cartesian_pose
/forward_hand_joint_targets
/target_hand_force
/isaac_contact_states
/hand_force_current_monitor
/hand_force_target_monitor
```

---

## 4.3 RBY1 profile

The RBY1 profile uses:

```yaml
robot_kinematics:
  profile: rby1_arm_only

  arm:
    base_link: link_torso_5
    left_tip_link: ee_left
    right_tip_link: ee_right

    left_joint_names:
      - left_arm_0
      - left_arm_1
      - left_arm_2
      - left_arm_3
      - left_arm_4
      - left_arm_5
      - left_arm_6

    right_joint_names:
      - right_arm_0
      - right_arm_1
      - right_arm_2
      - right_arm_3
      - right_arm_4
      - right_arm_5
      - right_arm_6

  hand:
    enabled: false
```

This means the controller expects:

```text
left arm  : 7 joints
right arm : 7 joints
hand      : disabled in this profile
```

The RBY1 chain intentionally starts from:

```text
link_torso_5
```

not:

```text
base
```

Reason:

```text
If the KDL chain starts from base, the chain includes torso_0~torso_5 plus arm_0~arm_6.
That makes the IK chain larger than the intended arm-only control problem.
For v30 arm-only control, link_torso_5 is used as the arm base.
```

---

## 4.4 Practical behavior difference

### Doosan/AIDIN

Doosan/AIDIN has 6 joints per arm. Cartesian IK is closer to a standard 6-DOF manipulator problem.

In practice:

```text
delta command → target Cartesian pose → arm IK → joint command
```

The same delta command often behaves predictably because the arm has a more direct 6-DOF kinematic chain.

### RBY1

RBY1 has 7 joints per arm in the selected arm-only chain. This gives redundancy, but it also means:

```text
the same Cartesian target can have multiple joint solutions
```

and some motions depend strongly on the current posture.

In v30, the RBY1 arm control should be understood as:

```text
node start home pose is latched
delta command is interpreted relative to that home pose
arm is commanded in inverse mode
hand is idle
workspace/manipulability must be considered
```

A motion like:

```text
right arm +z 0.1 m directly from home
```

may be difficult or unstable if the home pose is close to a poor-manipulability posture for that direction.

A more stable command sequence can be:

```text
1. move the right arm in -y direction
2. then command the combined target: -y offset + z offset
```

Example:

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0, 0, 0, 0, 0, 0,
         0, -0.2, 0, 0, 0, 0]}"

ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0, 0, 0, 0, 0, 0,
         0, -0.2, 0.1, 0, 0, 0]}"
```

---

# 5. Topic Interface

## 5.1 Current state input

```text
/isaac_joint_states
```

Type:

```text
sensor_msgs/msg/JointState
```

Used for:

```text
current joint state
FK current pose
home/base pose latch
unknown joint hold
RBY1 fixed joint hold
```

---

## 5.2 Final command output

```text
/isaac_joint_command
```

Type:

```text
sensor_msgs/msg/JointState
```

This topic publishes the final combined command.

For Doosan/AIDIN:

```text
arm command + hand command
```

For RBY1:

```text
arm command + hold values for non-controlled joints
```

---

## 5.3 Arm inverse Cartesian target

```text
/target_arm_cartesian_pose
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Format:

```text
[L x y z roll pitch yaw, R x y z roll pitch yaw]
```

This topic is an absolute Cartesian target.

---

## 5.4 Arm delta Cartesian target

```text
/delta_arm_cartesian_pose
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Format:

```text
[L dx dy dz droll dpitch dyaw, R dx dy dz droll dpitch dyaw]
```

The delta is applied relative to the initial pose latched after node startup.

---

## 5.5 Arm forward joint target

```text
/forward_arm_joint_targets
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

This topic is used when:

```text
arm_mode = forward
```

---

## 5.6 Hand forward joint target

```text
/forward_hand_joint_targets
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Recommended format for the Doosan/AIDIN hand pipeline:

```text
40 values = left hand 20 + right hand 20
```

Per hand:

```text
5 fingers × 4 joints = 20 values
```

Canonical order:

```text
thumb, index, middle, ring, baby
```

---

## 5.7 Hand target force

```text
/target_hand_force
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Compact format:

```text
[hand_id, finger_id, fx, fy, fz]
```

Legacy-compatible format:

```text
[hand_id, finger_id, px, py, pz, fx, fy, fz]
```

This is mainly for the Doosan/AIDIN hand force/admittance pipeline.

---

# 6. v30 RBY1 Usage Notes

## 6.1 Home pose latch

For RBY1, the intended workflow is:

```text
1. Put the robot in the desired home pose in Isaac.
2. Start the controller node.
3. The node receives /isaac_joint_states.
4. The first stable FK pose becomes the delta command reference.
5. /delta_arm_cartesian_pose commands are applied relative to that reference.
```

---

## 6.2 Delta command is home-relative, not previous-command-relative

This is important for RBY1 testing.

If the robot receives:

```text
R = [0, -0.2, 0, 0, 0, 0]
```

and then the user wants to add:

```text
+z 0.1
```

the next command should be:

```text
R = [0, -0.2, 0.1, 0, 0, 0]
```

not:

```text
R = [0, 0, 0.1, 0, 0, 0]
```

---

## 6.3 Workspace and posture limitation

RBY1 is posture-sensitive.

If a command works in y but fails or behaves strangely in z, it does not automatically mean the delta parser is broken. It may mean:

```text
the current home posture has poor manipulability in that Cartesian direction
```

For example:

```text
home → +z 0.1
```

may be difficult, while:

```text
home → -y 0.2 → combined target (-y 0.2, +z 0.1)
```

may work correctly.

---

# 7. Monitor Interpretation

The monitor prints:

```text
CUR_POS TAR_POS CUR_F TAR_F
```

For arms:

```text
CUR P[m,deg] = current FK position and orientation
TAR P[m,deg] = target Cartesian position and orientation
```

For hands:

```text
P[m] = fingertip position in hand base frame
F[N] = measured/target force
```

For RBY1 with `hand.enabled=false`, hand values may remain zero. This is expected in the arm-only profile.

---

# 8. File Role Rules

Keep the file-role separation.

```text
DualArmForceControl.cpp
= constructor, destructor, ControlLoop, YAML/config loading, ROS interface setup

states_current_callback_dualarm.cpp
= current joint state callbacks, FK current pose update, monitor state update

states_target_callback_dualarm.cpp
= target callbacks for arm/hand Cartesian, delta, forward joint, force target

DualArmForceControl.h
= class declaration, parameters, state variables, ROS interface members

node_dualarm_main.cpp
= minimal ROS node entry point
```

Do not move callback implementations into `DualArmForceControl.cpp`.

Do not move constructor/control-loop implementation into callback files.

---

# 9. Recommended v30 Commit Scope

This version should be saved as:

```text
v30
```

Suggested tag:

```bash
git tag v30
```

Suggested push:

```bash
git push origin main --tags
```

Recommended commit message is provided separately below.
