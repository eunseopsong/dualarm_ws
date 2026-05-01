# DualArm Force Control README (v31)

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

#### RBY1 fast teleop + hand forward

```bash
source ~/dualarm_ws/install/setup.bash

ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=/home/vision/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml \
  -p kinematics_cfg_yaml:=/home/vision/dualarm_ws/src/dualarm_kinematics/config/rby1_kinematics.yaml \
  -p control_loop_period_ms:=5.0 \
  -p rby1_arm_max_cmd_step_rad:=0.015 \
  -p rby1_arm_servo_max_cart_step_m:=0.0025
```

If the robot moves too aggressively, use the conservative setting:

```bash
ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=/home/vision/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml \
  -p kinematics_cfg_yaml:=/home/vision/dualarm_ws/src/dualarm_kinematics/config/rby1_kinematics.yaml \
  -p control_loop_period_ms:=10.0 \
  -p rby1_arm_max_cmd_step_rad:=0.008 \
  -p rby1_arm_servo_max_cart_step_m:=0.0012
```

#### Doosan/AIDIN dual-arm

```bash
source ~/dualarm_ws/install/setup.bash

ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=/home/vision/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml \
  -p kinematics_cfg_yaml:=/home/vision/dualarm_ws/src/dualarm_kinematics/config/doosan_dualarm_kinematics.yaml
```

If the PC username is `eunseop`, replace `/home/vision` with `/home/eunseop`.

---

### 3. Change control mode

#### RBY1 arm inverse + hand forward

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'forward'}"
```

The monitor should show:

```text
Dual Arm & Hand Monitor v31 | Arm:[inverse] Hand:[forward]
```

#### RBY1 arm only

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'idle'}"
```

#### Doosan/AIDIN arm inverse + hand forward

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'forward'}"
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

Example: right arm first moves in `-y`, then moves in `+z` while keeping the same `-y` offset.

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

So if the first command is:

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

### 5. Send hand forward command

Topic:

```text
/forward_hand_joint_targets
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Recommended v31 format:

```text
40 values = left hand 20 + right hand 20
```

Per hand:

```text
5 fingers × 4 joints = 20 values
```

Canonical finger order:

```text
thumb, index, middle, ring, baby
```

Per finger:

```text
[joint1, joint2, joint3, joint4]
```

Full command layout:

```text
[
  L_thumb_1, L_thumb_2, L_thumb_3, L_thumb_4,
  L_index_1, L_index_2, L_index_3, L_index_4,
  L_middle_1, L_middle_2, L_middle_3, L_middle_4,
  L_ring_1, L_ring_2, L_ring_3, L_ring_4,
  L_baby_1, L_baby_2, L_baby_3, L_baby_4,

  R_thumb_1, R_thumb_2, R_thumb_3, R_thumb_4,
  R_index_1, R_index_2, R_index_3, R_index_4,
  R_middle_1, R_middle_2, R_middle_3, R_middle_4,
  R_ring_1, R_ring_2, R_ring_3, R_ring_4,
  R_baby_1, R_baby_2, R_baby_3, R_baby_4
]
```

For RBY1 v31, `hand_mode=forward` is allowed even when the RBY1 kinematics YAML has `hand.enabled=false`. In that case, the hand joints are treated as direct forward targets and are passed to `/isaac_joint_command`. Hand FK/IK/admittance is not used for RBY1 hand forward.

---

#### Hand command 1: open both hands

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,

  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

#### Hand command 2: close both index fingers slightly

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

#### Hand command 3: close both hands moderately

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.2, 0.3, 0.3, 0.3,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5,

  0.2, 0.3, 0.3, 0.3,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5
]}"
```

---

#### Hand command 4: close both hands strongly

Start from moderate values first. Use this only after confirming the joint direction and limits are safe.

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.3, 0.5, 0.5, 0.5,
  0.5, 0.8, 0.8, 0.8,
  0.5, 0.8, 0.8, 0.8,
  0.5, 0.8, 0.8, 0.8,
  0.5, 0.8, 0.8, 0.8,

  0.3, 0.5, 0.5, 0.5,
  0.5, 0.8, 0.8, 0.8,
  0.5, 0.8, 0.8, 0.8,
  0.5, 0.8, 0.8, 0.8,
  0.5, 0.8, 0.8, 0.8
]}"
```

---

#### Hand command 5: left hand close, right hand open

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.2, 0.3, 0.3, 0.3,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5,

  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

#### Hand command 6: left hand open, right hand close

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,

  0.2, 0.3, 0.3, 0.3,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5,
  0.3, 0.5, 0.5, 0.5
]}"
```

---

#### Hand command 7: both thumbs only

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.3, 0.5, 0.5, 0.5,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,

  0.3, 0.5, 0.5, 0.5,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

#### Hand command 8: both index + thumb pinch-ready pose

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.25, 0.45, 0.45, 0.45,
  0.35, 0.55, 0.55, 0.55,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,

  0.25, 0.45, 0.45, 0.45,
  0.35, 0.55, 0.55, 0.55,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

#### Hand command 9: left index only

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0, 0.0,
  0.3, 0.5, 0.5, 0.5,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,

  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

#### Hand command 10: right index only

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,

  0.0, 0.0, 0.0, 0.0,
  0.3, 0.5, 0.5, 0.5,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

#### Hand command 11: contact-ready ring fingers

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.2, 0.4, 0.4, 0.4,
  0.0, 0.0, 0.0, 0.0,

  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.2, 0.4, 0.4, 0.4,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

#### Hand command 12: Manus glove neutral test pose

Use this as a safe intermediate pose when checking Manus glove mapping.

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.1, 0.15, 0.15, 0.15,
  0.1, 0.15, 0.15, 0.15,
  0.1, 0.15, 0.15, 0.15,
  0.1, 0.15, 0.15, 0.15,
  0.1, 0.15, 0.15, 0.15,

  0.1, 0.15, 0.15, 0.15,
  0.1, 0.15, 0.15, 0.15,
  0.1, 0.15, 0.15, 0.15,
  0.1, 0.15, 0.15, 0.15,
  0.1, 0.15, 0.15, 0.15
]}"
```

---

# 1. Version Summary

## 1.1 v31 purpose

`v31` is the fast teleoperation version based on `v30`.

The major goal of v31 is:

```text
RBY1 arm inverse teleop + RBY1 hand forward command at the same time
```

v31 keeps the same common controller structure:

```text
dualarm_forcecon
= ROS 2 node, mode handling, topic/service interface, arm command publishing,
  hand forward/inverse handling, hand force/admittance control, monitor printing
```

and keeps the reusable kinematics package:

```text
dualarm_kinematics
= FK/IK implementation, robot kinematics YAML profiles, joint/link mapping
```

---

## 1.2 v31 changes from v30

v31 adds the following behavior:

```text
1. RBY1 hand_mode=forward is allowed.
2. hand.enabled=false no longer forces requested hand_mode=forward to idle.
3. RBY1 hand forward commands are passed through to /isaac_joint_command.
4. Torso/head/wheel joints are still held at their initial values.
5. Arm delta target callback triggers immediate control update to reduce teleop delay.
6. Default control loop can be run faster for Manus glove teleoperation.
7. RBY1 arm command step and Cartesian servo step can be increased for faster motion.
```

The main practical workflow is now:

```text
RBY1:
  arm_mode  = inverse
  hand_mode = forward

Doosan/AIDIN:
  arm_mode  = inverse
  hand_mode = forward
```

---

# 2. Doosan/AIDIN vs RBY1 Configuration Differences

## 2.1 Summary table

| Item | Doosan/AIDIN dual-arm | RBY1 |
|---|---|---|
| Profile name | `doosan_aidin` | `rby1_arm_only` |
| Main use in v31 | Arm + hand control | Arm inverse + hand forward teleop |
| Arm DOF per side | 6 DOF | 7 DOF |
| Arm base link | `base_link` | `link_torso_5` |
| Left tip link | `left_link_6` | `ee_left` |
| Right tip link | `right_link_6` | `ee_right` |
| Left arm joint names | `left_joint_1` ~ `left_joint_6` | `left_arm_0` ~ `left_arm_6` |
| Right arm joint names | `right_joint_1` ~ `right_joint_6` | `right_arm_0` ~ `right_arm_6` |
| Hand config | `hand.enabled=true` | `hand.enabled=false`, but forward pass-through allowed |
| Hand FK/IK | enabled | not used |
| Hand forward command | joint target command | direct joint target pass-through |
| Recommended mode | `arm=inverse`, `hand=forward` | `arm=inverse`, `hand=forward` |
| Fixed joints | usually not relevant | wheel/torso/head are held at startup values |
| Practical caution | Standard 6-DOF arm IK | Redundant 7-DOF arm; workspace/manipulability matters |

---

## 2.2 Doosan/AIDIN profile

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

This profile is intended for the full arm + hand + force/admittance pipeline:

```text
/delta_arm_cartesian_pose
/target_arm_cartesian_pose
/forward_arm_joint_targets
/forward_hand_joint_targets
/target_hand_force
/isaac_joint_command
```

---

## 2.3 RBY1 profile

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

In v31, `hand.enabled=false` means:

```text
Do not run RBY1 hand FK/IK/admittance.
Do allow hand forward joint command pass-through.
```

So RBY1 can now run:

```text
arm_mode  = inverse
hand_mode = forward
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
For v31 arm-only IK, link_torso_5 is used as the arm base.
```

---

# 3. RBY1 Fixed Joint Behavior

For RBY1, these joints are held at their startup values:

```text
left_wheel
right_wheel
torso_*
head_*
```

These joints are not part of the arm IK command.

The RBY1 arm joints are controlled by arm mode:

```text
left_arm_0 ~ left_arm_6
right_arm_0 ~ right_arm_6
```

The hand joints are controlled by hand forward mode:

```text
hand_mode = forward
→ /forward_hand_joint_targets
→ direct joint command pass-through
→ /isaac_joint_command
```

So the intended v31 behavior is:

```text
wheel / torso / head : startup hold
arm joints           : inverse control
hand joints          : forward control
```

---

# 4. Topic Interface

## 4.1 Current state input

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

## 4.2 Final command output

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

For RBY1 v31:

```text
arm inverse command + hand forward command + torso/head/wheel startup hold
```

---

## 4.3 Arm inverse Cartesian target

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

## 4.4 Arm delta Cartesian target

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

## 4.5 Arm forward joint target

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

## 4.6 Hand forward joint target

```text
/forward_hand_joint_targets
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

This topic is used when:

```text
hand_mode = forward
```

---

## 4.7 Hand target force

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

This is mainly for the Doosan/AIDIN hand force/admittance pipeline.

RBY1 v31 hand forward does not use hand admittance by default.

---

# 5. RBY1 Teleoperation Notes

## 5.1 Home pose latch

For RBY1, the intended workflow is:

```text
1. Put the robot in the desired home pose in Isaac.
2. Start the controller node.
3. The node receives /isaac_joint_states.
4. The first stable FK pose becomes the delta command reference.
5. /delta_arm_cartesian_pose commands are applied relative to that reference.
```

---

## 5.2 Delta command is home-relative

This is important for Manus glove teleoperation.

If the robot receives:

```text
R = [0, -0.2, 0, 0, 0, 0]
```

and the user wants to add:

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

## 5.3 Teleop response tuning

For faster response:

```bash
-p control_loop_period_ms:=5.0
-p rby1_arm_max_cmd_step_rad:=0.015
-p rby1_arm_servo_max_cart_step_m:=0.0025
```

For safer/slower response:

```bash
-p control_loop_period_ms:=10.0
-p rby1_arm_max_cmd_step_rad:=0.008
-p rby1_arm_servo_max_cart_step_m:=0.0012
```

If the arm vibrates, jumps, or overshoots, reduce these values.

If the arm response is too slow during Manus glove teleop, increase them gradually.

---

## 5.4 Workspace and posture limitation

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

# 6. Monitor Interpretation

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

For RBY1 v31, hand forward is direct joint target pass-through. If `hand.enabled=false`, hand FK/IK monitor values may remain zero even when hand joint commands are being published. In that case, verify hand motion through Isaac joint states or the robot visualization.

---

# 7. File Role Rules

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

# 8. Recommended v31 Commit

Suggested commit:

```bash
git add .
git commit -m "v31: enable RBY1 hand forward with fast arm teleop"
```

Suggested tag:

```bash
git tag v31
git push origin main --tags
```
