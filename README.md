# DualArm Force Control README (v31 - Hand Admittance Restored)

## Quick Start Example Commands

Path assumption for all commands:

```text
dualarm workspace: ~/dualarm_ws
MANUS workspace  : ~/manus_ws
```

All package data paths use `package://...` and are resolved from the sourced
ROS workspace, so the commands are independent of the Linux username or PC name.

### 1. Build

```bash
cd ~/dualarm_ws
colcon build --symlink-install
source install/setup.bash
```

---

### 2. Run controller node

#### RBY1 fast teleop + hand admittance

Use this when running RBY1 arm teleoperation with Manus glove hand command and selected-finger admittance correction.

```bash
cd ~/dualarm_ws
source install/setup.bash

ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=package://dualarm_forcecon/yaml/forcecon_cfg.yaml \
  -p kinematics_cfg_yaml:=package://dualarm_kinematics/config/rby1_kinematics.yaml \
  -p control_loop_period_ms:=5.0 \
  -p rby1_arm_max_cmd_step_rad:=0.015 \
  -p rby1_arm_servo_max_cart_step_m:=0.0025 \
  -p enable_hand_runtime_when_yaml_disabled:=true \
  -p fallback_left_hand_base_link:=left_joint_6 \
  -p fallback_right_hand_base_link:=right_joint_6
```

If the robot moves too aggressively:

```bash
cd ~/dualarm_ws
source install/setup.bash

ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=package://dualarm_forcecon/yaml/forcecon_cfg.yaml \
  -p kinematics_cfg_yaml:=package://dualarm_kinematics/config/rby1_kinematics.yaml \
  -p control_loop_period_ms:=10.0 \
  -p rby1_arm_max_cmd_step_rad:=0.008 \
  -p rby1_arm_servo_max_cart_step_m:=0.0012 \
  -p enable_hand_runtime_when_yaml_disabled:=true \
  -p fallback_left_hand_base_link:=left_joint_6 \
  -p fallback_right_hand_base_link:=right_joint_6
```

---

### 3. Run MANUS glove bridge

Use two terminals. Keep the MANUS publisher running in terminal 1, then start
the bridge in terminal 2.

Terminal 1:

```bash
cd ~/dualarm_ws
source ~/manus_ws/install/setup.bash
source install/setup.bash

ros2 run manus_ros2 manus_data_publisher
```

Terminal 2:

```bash
cd ~/dualarm_ws
source ~/manus_ws/install/setup.bash
source install/setup.bash

ros2 run manus_hand_control aidin_hand_control --ros-args \
  -p auto_set_forcecon_mode:=true \
  -p forcecon_arm_mode:=inverse \
  -p forcecon_hand_mode:=forward
```

MANUS topic checks:

```bash
cd ~/dualarm_ws
source ~/manus_ws/install/setup.bash
source install/setup.bash

ros2 topic list | grep manus
ros2 topic echo /manus_glove_0
ros2 topic hz /forward_hand_joint_targets
```

Print incoming MANUS values:

```bash
cd ~/dualarm_ws
source ~/manus_ws/install/setup.bash
source install/setup.bash

ros2 run manus_hand_control aidin_hand_control --ros-args \
  -p print_manus_input:=true \
  -p print_interval_s:=0.5
```

---

### 4. Change control mode

#### RBY1 arm inverse + hand forward

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'forward'}"
```

Expected monitor:

```text
Dual Arm & Hand Monitor v31 | Arm:[inverse] Hand:[forward]
```

#### RBY1 arm inverse + hand inverse

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'inverse'}"
```

#### Full idle

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'idle', hand_mode: 'idle'}"
```

---

## 1. Version Summary

### 1.1 v31 current meaning

The current v31 version combines the following behavior:

```text
RBY1:
  arm  : inverse teleoperation using /delta_arm_cartesian_pose
  hand : forward or inverse hand command
  force: selected-finger admittance correction using /target_hand_force + /isaac_contact_states
```

The important rollback from the temporary fast hand pass-through version is:

```text
RBY1 hand forward is no longer treated as simple direct pass-through only.
It returns to the old v25-style motion-reference + admittance-correction pipeline.
```

---

### 1.2 Hand control architecture

The hand command path is:

```text
motion reference
    +
selected-finger desired force
    +
measured contact force
    ↓
HandAdmittanceControl
    ↓
selected-finger IK correction
    ↓
final hand joint command
```

For hand forward mode:

```text
/forward_hand_joint_targets
    -> q_h_motion_t_
    -> FK
    -> optional selected-finger admittance correction
    -> Hand IK for selected finger
    -> q_h_t_
    -> /isaac_joint_command
```

For hand inverse mode:

```text
/target_hand_fingertips or /delta_hand_fingertips
    -> Hand IK
    -> q_h_motion_t_
    -> optional selected-finger admittance correction
    -> q_h_t_
    -> /isaac_joint_command
```

There is no separate hand `forcecon` mode. Force control is an internal correction layer used while hand mode remains:

```text
forward
or
inverse
```

---

## 2. Robot Configuration Notes

### 2.1 RBY1

For RBY1, the arm chain uses:

```text
base link : link_torso_5
left tip  : ee_left
right tip : ee_right
```

By default, the RBY1 non-arm joints remain held at startup values:

```text
left_wheel
right_wheel
torso_*
head_*
```

The intended v31 behavior is:

```text
wheel joints         : startup hold, or velocity override from /forward_aux_joint_targets
torso joints         : startup hold, or position override from /forward_aux_joint_targets
head joints          : startup hold
arm joints           : inverse control
hand joints          : forward or inverse hand control
selected finger      : optional admittance correction
```

If `rby1_kinematics.yaml` still has:

```yaml
hand:
  enabled: false
```

then run with:

```bash
-p enable_hand_runtime_when_yaml_disabled:=true
```

or change the YAML to:

```yaml
hand:
  enabled: true
```

Long-term, the cleaner approach is to make the RBY1 hand link names valid in `rby1_kinematics.yaml` and use `hand.enabled: true`.

---

### 2.2 Doosan/AIDIN

Doosan/AIDIN uses the original hand FK/IK/admittance path naturally because its hand profile is configured as enabled.

Recommended mode:

```text
arm_mode  = inverse
hand_mode = forward
```

---

## 3. Topic Interface

### 3.1 State input

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
startup fixed joint hold
```

---

### 3.2 Main output

```text
/isaac_joint_command
```

Type:

```text
sensor_msgs/msg/JointState
```

Publishes the final combined command:

```text
arm command + hand command + fixed joint hold values
```

---

### 3.3 Contact input

```text
/isaac_contact_states
```

Type:

```text
std_msgs/msg/Float32MultiArray
```

Expected contact order:

```text
5 left + 5 right
```

The internal canonical finger order is:

```text
0 thumb
1 index
2 middle
3 ring
4 baby
```

---

### 3.4 Arm delta Cartesian target

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

Important:

```text
/delta_arm_cartesian_pose is home-pose-relative absolute delta.
It is not incremental from the previous command.
```

Example:

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.000, 0.000, 0.000, 0.0, 0.0, 0.0,
         0.000, -0.200, 0.100, 0.0, 0.0, 0.0]}"
```

---

### 3.5 Hand forward joint target

```text
/forward_hand_joint_targets
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Recommended format:

```text
40 values = left hand 20 + right hand 20
```

Per hand:

```text
5 fingers × 4 joints = 20 values
```

Finger order:

```text
thumb, index, middle, ring, baby
```

Per finger:

```text
[joint1, joint2, joint3, joint4]
```

The current hand IK/admittance code may internally compress to 15 independent DoF:

```text
joint4 is treated as mimic of joint3 where needed
```

---

### 3.6 RBY1 wheel / torso upright driving

```text
/cmd_vel
```

Type:

```text
geometry_msgs/msg/Twist
```

Run the controller first:

```bash
cd ~/dualarm_ws
source install/setup.bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

Forward:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3}, angular: {z: 0.0}}"
```

Backward:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: -0.3}, angular: {z: 0.0}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

Turn waist 90 degrees left:

```bash
ros2 topic pub --once /forward_aux_joint_targets sensor_msgs/msg/JointState \
"{name: ['torso_0', 'torso_1', 'torso_2', 'torso_3', 'torso_4', 'torso_5'], position: [0.0, 0.0875, 0.0883, -0.1739, 0.0, 1.5708], velocity: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]}"
```

Turn waist 90 degrees right:

```bash
ros2 topic pub --once /forward_aux_joint_targets sensor_msgs/msg/JointState \
"{name: ['torso_0', 'torso_1', 'torso_2', 'torso_3', 'torso_4', 'torso_5'], position: [0.0, 0.0875, 0.0883, -0.1739, 0.0, -1.5708], velocity: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]}"
```

Return waist to center:

```bash
ros2 topic pub --once /forward_aux_joint_targets sensor_msgs/msg/JointState \
"{name: ['torso_0', 'torso_1', 'torso_2', 'torso_3', 'torso_4', 'torso_5'], position: [0.0, 0.0875, 0.0883, -0.1739, 0.0, 0.0], velocity: [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]}"
```

Notes:

```text
Do not publish wheel-only commands directly to /isaac_joint_command.

While /cmd_vel is active, dualarm_forcecon also publishes the configured
torso_* upright position targets so the torso stays vertical during driving.

Wheel velocity commands are published to /isaac_wheel_commands.
This topic is configurable with mobile_base.wheel_command_topic.

In the current Isaac setup, positive /cmd_vel.linear.x is forward because
mobile_base.invert_wheel_velocity_command is true in forcecon_cfg.yaml.

The torso upright target is configured by:
mobile_base.torso_upright_joint_names
mobile_base.torso_upright_positions

Waist left/right uses torso_5. If the visual direction is opposite in Isaac,
swap the signs of +/-1.5708.

For torso commands, velocity is used as the max position speed in rad/s.
Smaller values such as 0.5 move slower; larger values such as 2.0 move faster.
```

---

### 3.7 Hand target force

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

Full array format:

```text
[
  left_thumb_fx,   left_thumb_fy,   left_thumb_fz,
  left_index_fx,   left_index_fy,   left_index_fz,
  left_middle_fx,  left_middle_fy,  left_middle_fz,
  left_ring_fx,    left_ring_fy,    left_ring_fz,
  left_baby_fx,    left_baby_fy,    left_baby_fz,
  right_thumb_fx,  right_thumb_fy,  right_thumb_fz,
  right_index_fx,  right_index_fy,  right_index_fz,
  right_middle_fx, right_middle_fy, right_middle_fz,
  right_ring_fx,   right_ring_fy,   right_ring_fz,
  right_baby_fx,   right_baby_fy,   right_baby_fz
]
```

Legacy-compatible format:

```text
[hand_id, finger_id, px, py, pz, fx, fy, fz]
```

Index convention:

```text
hand_id:
  0 = left
  1 = right

finger_id:
  0 = thumb
  1 = index
  2 = middle
  3 = ring
  4 = baby
```

Force unit:

```text
N
```

Isaac Sim hand force Script Node source:

```text
src/dualarm_forcecon/scripts/aidin_hand_force_sensor.py
```

After build, the installed copy is under:

```bash
ros2 pkg prefix dualarm_forcecon
# <prefix>/share/dualarm_forcecon/scripts/aidin_hand_force_sensor.py
```

Frame:

```text
hand-base frame
```

Important:

```text
/target_hand_force does not replace hand motion target.
For stable admittance behavior, publish a valid hand motion target first.
Then publish /target_hand_force.

Compact and legacy-compatible commands select one active fingertip force target.
Full array commands update all left/right fingertip force targets at once.
```

---

## 4. Hand Forward Example Commands

### Command 1: open both hands

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

### Command 2: neutral slightly bent pose

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.10, 0.15, 0.15, 0.15,
  0.10, 0.15, 0.15, 0.15,
  0.10, 0.15, 0.15, 0.15,
  0.10, 0.15, 0.15, 0.15,
  0.10, 0.15, 0.15, 0.15,

  0.10, 0.15, 0.15, 0.15,
  0.10, 0.15, 0.15, 0.15,
  0.10, 0.15, 0.15, 0.15,
  0.10, 0.15, 0.15, 0.15,
  0.10, 0.15, 0.15, 0.15
]}"
```

---

### Command 3: close both index fingers slightly

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

### Command 4: close both hands moderately

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

### Command 5: close both hands strongly

Thumbs bend strongly; all other fingers bend to the same angle.

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  1.2, 1.2, 1.2, 1.2,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,

  1.2, 1.2, 1.2, 1.2,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0
]}"
```

---

### Command 6: right hand close strongly, left hand open

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,

  1.2, 1.2, 1.2, 1.2,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0
]}"
```

---

### Command 7: left hand close strongly, right hand open

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  1.2, 1.2, 1.2, 1.2,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,
  1.0, 1.0, 1.0, 1.0,

  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

### Command 8: both thumbs only

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

### Command 9: both index + thumb pinch-ready pose

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.25, 0.45, 0.45, 0.45,
  0.35, 0.55, 0.55, 0.55,
  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,

  0.25, 0.45, 0.45, 0.45,
  0.35, 0.55, 0.55, 0.55,
  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0,
  0.0,  0.0,  0.0,  0.0
]}"
```

---

### Command 10: left index only

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

### Command 11: right index only

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

### Command 12: contact-ready ring fingers

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

### Command 13: contact-ready index fingers

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0, 0.0,
  0.25, 0.45, 0.45, 0.45,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,

  0.0, 0.0, 0.0, 0.0,
  0.25, 0.45, 0.45, 0.45,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

### Command 14: left ring contact-ready, right hand open

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
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

### Command 15: right ring contact-ready, left hand open

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
  0.2, 0.4, 0.4, 0.4,
  0.0, 0.0, 0.0, 0.0
]}"
```

---

## 5. Target Hand Force Example Commands

Before sending force target, send a hand motion target first.

A recommended basic sequence is:

```text
1. Set hand_mode=forward
2. Send /forward_hand_joint_targets
3. Send /target_hand_force
4. Watch /hand_force_target_monitor and /hand_force_current_monitor
```

---

### Force command 1: left ring, +5 N z

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 3, 0.0, 0.0, 5.0]}"
```

Meaning:

```text
0 = left hand
3 = ring finger
force = (0, 0, +5) N in hand-base frame
```

---

### Force command 2: right ring, +5 N z

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [1, 3, 0.0, 0.0, 5.0]}"
```

---

### Force command 3: left index, +3 N z

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 1, 0.0, 0.0, 3.0]}"
```

---

### Force command 4: right index, +3 N z

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [1, 1, 0.0, 0.0, 3.0]}"
```

---

### Force command 5: left thumb, +2 N z

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 0, 0.0, 0.0, 2.0]}"
```

---

### Force command 6: right thumb, +2 N z

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [1, 0, 0.0, 0.0, 2.0]}"
```

---

### Force command 7: left middle, +4 N z

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 2, 0.0, 0.0, 4.0]}"
```

---

### Force command 8: right middle, +4 N z

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [1, 2, 0.0, 0.0, 4.0]}"
```

---

### Force command 9: left baby, +2 N z

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 4, 0.0, 0.0, 2.0]}"
```

---

### Force command 10: right baby, +2 N z

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [1, 4, 0.0, 0.0, 2.0]}"
```

---

### Force command 11: release selected force target to zero

This clears force target for the selected finger side/finger.

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 3, 0.0, 0.0, 0.0]}"
```

Right ring release:

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [1, 3, 0.0, 0.0, 0.0]}"
```

---

### Force command 12: opposite sign test

If Isaac contact force sign is opposite to the expected direction, test the sign.

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 3, 0.0, 0.0, -5.0]}"
```

---

### Force command 13: x-axis force test

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 3, 2.0, 0.0, 0.0]}"
```

---

### Force command 14: y-axis force test

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 3, 0.0, 2.0, 0.0]}"
```

---

### Force command 15: legacy-compatible format

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 3, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0]}"
```

Meaning:

```text
[hand_id, finger_id, px, py, pz, fx, fy, fz]
```

The position part is legacy-compatible and is ignored in the current desired-force usage.

---

### Force command 16: full array, left ring + right index

This updates all ten fingertip force targets at once. Values not listed as
nonzero are explicitly commanded to zero.

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 5.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 3.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0
]}"
```

Meaning:

```text
left ring  force = (0, 0, +5) N
right index force = (0, 0, +3) N
all other fingertip force targets = 0 N
```

---

### Force command 17: release all force targets

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0,
  0.0, 0.0, 0.0
]}"
```

---

## 6. Combined Hand Motion + Force Examples

### Example A: left ring contact-ready + left ring force

Step 1: set hand mode.

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'forward'}"
```

Step 2: send left ring contact-ready pose.

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
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"
```

Step 3: send desired force.

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 3, 0.0, 0.0, 5.0]}"
```

---

### Example B: right index contact-ready + right index force

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,

  0.0, 0.0, 0.0, 0.0,
  0.25, 0.45, 0.45, 0.45,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0
]}"

ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [1, 1, 0.0, 0.0, 3.0]}"
```

---

### Example C: keep both hands moderately closed and regulate left ring force

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

ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 3, 0.0, 0.0, 4.0]}"
```

---

## 7. Monitor / Debug Commands

Check command rates:

```bash
ros2 topic hz /forward_hand_joint_targets
ros2 topic hz /target_hand_force
ros2 topic hz /isaac_joint_command
ros2 topic hz /isaac_contact_states
```

Check command output:

```bash
ros2 topic echo --once /isaac_joint_command
```

Check force monitors:

```bash
ros2 topic echo /hand_force_target_monitor
ros2 topic echo /hand_force_current_monitor
```

Recommended rqt plot signals:

```text
/hand_force_target_monitor/data[0..29]
/hand_force_current_monitor/data[0..29]
```

Canonical order:

```text
left  thumb xyz  -> data[0:3]
left  index xyz  -> data[3:6]
left  middle xyz -> data[6:9]
left  ring xyz   -> data[9:12]
left  baby xyz   -> data[12:15]

right thumb xyz  -> data[15:18]
right index xyz  -> data[18:21]
right middle xyz -> data[21:24]
right ring xyz   -> data[24:27]
right baby xyz   -> data[27:30]
```

---

## 8. Tuning Notes

Hand admittance behavior is strongly affected by `forcecon_cfg.yaml`.

Important parameters:

```text
mass
damping
stiffness
force_ctrl_enable
force_error_axis_sign
hybrid_force_axis
contact_on_threshold_N
contact_off_threshold_N
max_offset_m
max_step_m
max_adm_velocity_mps
ik_max_iters
ik_tol_pos_m
ik_lambda
ik_alpha
```

If the finger feels too soft:

```text
increase stiffness
increase damping
increase Isaac joint drive stiffness/damping
increase max effort/force of the finger drive
```

If the finger vibrates:

```text
increase damping
reduce stiffness
reduce max_step_m
reduce max_adm_velocity_mps
increase force_lpf_tau_s slightly
```

If force direction is wrong:

```text
change force_error_axis_sign
or test opposite sign in /target_hand_force
```

---

## 9. Recommended Commit

```bash
git add .
git commit -m "docs: update v31 README for hand admittance and force commands"
```

Long version:

```bash
git commit -m "docs: update v31 README for hand admittance and force commands" \
-m "Document the restored v25-style hand motion-reference plus selected-finger admittance pipeline.

Add /target_hand_force compact and legacy command examples, expanded hand forward poses, and combined hand motion plus force-control sequences for RBY1 and Doosan/AIDIN workflows."
```
