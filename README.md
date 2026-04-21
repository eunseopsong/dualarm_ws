# DualArmForceControl README (v28)

`v28` is the version where `dualarm_forcecon` was extended from the v27 split-package structure to support **RBY1 arm control** using the existing Isaac Sim Action Graph.

The main goals of v28 are:

```text
1. Keep the v27 package split:
   dualarm_forcecon   = ROS 2 control node
   dualarm_kinematics = reusable FK/IK and robot profile package

2. Add RBY1 support:
   - official RBY1 URDF-based kinematics
   - RBY1 arm-only KDL chain
   - 7-DOF left/right arm support
   - full-order command publishing with non-arm joint hold
   - position-only DLS IK fallback for RBY1 inverse control

3. Preserve existing Doosan/AIDIN dualarm workflow.
```

Recommended default operation mode for the current RBY1 test workflow:

```text
arm_mode  = inverse
hand_mode = idle
```

For Doosan/AIDIN dualarm with hand control:

```text
arm_mode  = inverse
hand_mode = forward
```

---

# Quick Start Example Commands

## Build

```bash
cd ~/dualarm_ws
colcon build --symlink-install
source install/setup.bash
```

If using the `cb` alias:

```bash
cd ~/dualarm_ws
cb
```

---

## Run controller node

```bash
source ~/dualarm_ws/install/setup.bash

ros2 run dualarm_forcecon dualarm_forcecon_node
```

or explicitly:

```bash
ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml
```

---

## Select robot profile

In:

```text
~/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml
```

Use:

```yaml
kinematics:
  active_robot_id: 0   # Doosan/AIDIN dualarm
```

or:

```yaml
kinematics:
  active_robot_id: 1   # RBY1
```

Runtime override:

```bash
ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p robot_profile_id:=1
```

Direct kinematics YAML override:

```bash
ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p kinematics_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_kinematics/config/rby1_kinematics.yaml
```

---

## RBY1 recommended test mode

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'idle'}"
```

RBY1 hand is disabled in the current arm-only config:

```yaml
hand:
  enabled: false
```

---

## RBY1 delta Cartesian arm command

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

Unit:

```text
position delta    : meter
orientation delta : radian
```

For RBY1 v28, inverse control uses:

```text
strict full-pose KDL IK first
→ if it fails, position-only DLS IK fallback
```

Start with 1 mm.

### Zero delta

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.000, 0.000, 0.000, 0.0, 0.0, 0.0,
         0.000, 0.000, 0.000, 0.0, 0.0, 0.0]}"
```

### +Z 1 mm

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.000, 0.000, 0.001, 0.0, 0.0, 0.0,
         0.000, 0.000, 0.001, 0.0, 0.0, 0.0]}"
```

### +X 1 mm

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.001, 0.000, 0.000, 0.0, 0.0, 0.0,
         0.001, 0.000, 0.000, 0.0, 0.0, 0.0]}"
```

### Outward Y 1 mm

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.000,  0.001, 0.000, 0.0, 0.0, 0.0,
         0.000, -0.001, 0.000, 0.0, 0.0, 0.0]}"
```

### +Z 3 mm

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.000, 0.000, 0.003, 0.0, 0.0, 0.0,
         0.000, 0.000, 0.003, 0.0, 0.0, 0.0]}"
```

### +Z 5 mm

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.000, 0.000, 0.005, 0.0, 0.0, 0.0,
         0.000, 0.000, 0.005, 0.0, 0.0, 0.0]}"
```

---

## RBY1 forward joint command test

Forward mode is useful to verify command publishing, Action Graph mapping, and joint order.

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'forward', hand_mode: 'idle'}"
```

Home-like command:

```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
-0.78, 0.26, 0.00, 0.00, 0.00, -0.78, 0.00,
-0.78,-0.26, 0.00, 0.00, 0.00, -0.78, 1.30
]}"
```

Small left/right shoulder movement:

```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [
-0.76, 0.26, 0.00, 0.00, 0.00, -0.78, 0.00,
-0.76,-0.26, 0.00, 0.00, 0.00, -0.78, 1.30
]}"
```

Expected command size for RBY1:

```text
left arm  7 DOF
right arm 7 DOF
total     14 values
```

---

## Verify home pose latch

After node starts, v28 should keep the current USD default pose instead of sending zero pose.

```bash
ros2 topic echo --once /isaac_joint_states
ros2 topic echo --once /isaac_joint_command
```

Expected behavior:

```text
/isaac_joint_command has the full RBY1 joint list
non-arm joints are held at the first /isaac_joint_states snapshot
arm joints are initialized from current USD home pose
```

---

## Verify command topic

```bash
ros2 topic info /isaac_joint_command -v
ros2 topic hz /isaac_joint_command
ros2 topic echo --once /isaac_joint_command
```

For RBY1 v28, command publishing uses full joint order:

```text
left_wheel
right_wheel
torso_0
...
left_arm_0
right_arm_0
...
hand joints
```

But only the arm target is updated by the controller.

---

# 0. Dependency

## 0.1 Target environment

```text
Ubuntu 22.04
ROS 2 Humble
Isaac Sim 5.0.0
C++17
```

Recommended shell setup:

```bash
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash
```

---

## 0.2 ROS 2 packages

Workspace packages:

```text
dualarm_forcecon_interfaces
dualarm_kinematics
dualarm_forcecon
```

Optional:

```text
rby1_joystick_teleop
```

---

## 0.3 External libraries

Required:

```text
Eigen3
yaml-cpp
Pinocchio
KDL / kdl_parser
urdfdom
```

Typical install:

```bash
sudo apt update
sudo apt install -y \
  libeigen3-dev \
  libyaml-cpp-dev \
  ros-humble-kdl-parser \
  ros-humble-orocos-kdl \
  liburdfdom-dev
```

Pinocchio install depends on system setup. Check:

```bash
pkg-config --modversion pinocchio
```

or:

```bash
ros2 pkg prefix pinocchio
```

---

## 0.4 Official RBY1 resources

Official SDK:

```bash
cd ~/dualarm_ws/src/dualarm_kinematics
mkdir -p urdf/rby1_official
cd urdf/rby1_official

git clone https://github.com/RainbowRobotics/rby1-sdk.git
git clone https://github.com/RainbowRobotics/rby1-dev.git
```

Official URDF candidate used in v28:

```text
dualarm_kinematics/urdf/rby1_official/rby1-sdk/models/rby1a/urdf/model.urdf
```

This file is copied and cleaned into:

```text
dualarm_kinematics/urdf/rby1a_kdl.urdf
```

---

# 1. Usage and Example Commands

## 1.1 Build

```bash
cd ~/dualarm_ws
cb
```

or:

```bash
cd ~/dualarm_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 1.2 Run node

```bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

The default config file is:

```text
/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml
```

Explicit run:

```bash
ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml
```

---

## 1.3 Force controller config

Main runtime config:

```text
dualarm_forcecon/yaml/forcecon_cfg.yaml
```

Robot profile selector:

```yaml
kinematics:
  active_robot_id: 1

  profiles:
    - id: 0
      name: doosan_dualarm
      config_yaml: /home/eunseop/dualarm_ws/src/dualarm_kinematics/config/doosan_dualarm_kinematics.yaml

    - id: 1
      name: rby1
      config_yaml: /home/eunseop/dualarm_ws/src/dualarm_kinematics/config/rby1_kinematics.yaml
```

Command publish mode for RBY1:

```yaml
command_publish:
  mode: full_hold_non_arm
  publish_rate_hz: 60.0
  arm_command_filter_enabled: true
  arm_command_max_step_rad: 0.002
```

Meaning:

```text
full_hold_non_arm:
  /isaac_joint_command publishes the full joint list
  arm joints are updated by control target
  non-arm joints are held at first /isaac_joint_states snapshot
```

This keeps the existing Isaac Action Graph working while preventing torso/wheel/head from being actively overwritten by new commands.

---

## 1.4 RBY1 kinematics config

File:

```text
dualarm_kinematics/config/rby1_kinematics.yaml
```

Current v28 config:

```yaml
robot_kinematics:
  profile: rby1_arm_only

  urdf_path: /home/eunseop/dualarm_ws/src/dualarm_kinematics/urdf/rby1a_kdl.urdf

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

Important:

```text
base_link: link_torso_5
```

Do not use:

```text
base_link: base
```

If `base` is used, the KDL chain becomes:

```text
base -> torso_0~5 -> arm_0~6 -> ee
```

Then the chain expects 13 joints, while the arm controller only provides 7 joints.

Correct arm-only KDL chain:

```text
link_torso_5 -> ee_left
link_torso_5 -> ee_right
```

---

## 1.5 Convert official RBY1 URDF to KDL-compatible URDF

Official URDF contains elements that standard ROS/KDL parser may not accept, such as unsupported geometry or incomplete parser requirements.

Raw copy:

```bash
cd ~/dualarm_ws/src/dualarm_kinematics/urdf

cp rby1_official/rby1-sdk/models/rby1a/urdf/model.urdf rby1a_official_raw.urdf
```

Clean script:

```bash
gedit ~/dualarm_ws/src/dualarm_kinematics/urdf/make_rby1a_kdl_urdf.py
```

Script:

```python
#!/usr/bin/env python3
from pathlib import Path
import xml.etree.ElementTree as ET

SRC = Path("/home/eunseop/dualarm_ws/src/dualarm_kinematics/urdf/rby1a_official_raw.urdf")
DST = Path("/home/eunseop/dualarm_ws/src/dualarm_kinematics/urdf/rby1a_kdl.urdf")

tree = ET.parse(SRC)
root = tree.getroot()

# Remove unsupported collision geometry for standard ROS urdfdom/KDL parser.
for link in root.findall("link"):
    for collision in list(link.findall("collision")):
        link.remove(collision)

# Ensure joint limit fields are parser-safe.
for joint in root.findall("joint"):
    jtype = joint.attrib.get("type", "")
    if jtype in ("revolute", "prismatic"):
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        if "lower" not in limit.attrib:
            limit.set("lower", "-3.141592653589793")
        if "upper" not in limit.attrib:
            limit.set("upper", "3.141592653589793")
        if "effort" not in limit.attrib:
            limit.set("effort", "1000.0")
        if "velocity" not in limit.attrib:
            limit.set("velocity", "10.0")

    elif jtype == "continuous":
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        if "effort" not in limit.attrib:
            limit.set("effort", "1000.0")
        if "velocity" not in limit.attrib:
            limit.set("velocity", "10.0")

ET.indent(tree, space="  ")
tree.write(DST, encoding="utf-8", xml_declaration=True)

print(f"[OK] Wrote cleaned URDF: {DST}")
```

Run:

```bash
chmod +x ~/dualarm_ws/src/dualarm_kinematics/urdf/make_rby1a_kdl_urdf.py
python3 ~/dualarm_ws/src/dualarm_kinematics/urdf/make_rby1a_kdl_urdf.py
```

Check:

```bash
check_urdf ~/dualarm_ws/src/dualarm_kinematics/urdf/rby1a_kdl.urdf
```

Expected tree includes:

```text
root Link: base
...
link_torso_5
  child: link_left_arm_0 -> ... -> link_left_arm_6 -> FT_sensor_L -> ee_left
  child: link_right_arm_0 -> ... -> link_right_arm_6 -> FT_sensor_R -> ee_right
```

---

## 1.6 RBY1 control mode service

Service:

```text
/change_control_mode
```

Type:

```text
dualarm_forcecon_interfaces/srv/SetControlMode
```

RBY1 arm inverse:

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'idle'}"
```

RBY1 arm forward:

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'forward', hand_mode: 'idle'}"
```

Idle:

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'idle', hand_mode: 'idle'}"
```

---

## 1.7 Main topics

### State input

```text
/isaac_joint_states
```

Type:

```text
sensor_msgs/msg/JointState
```

RBY1 joint order observed:

```text
left_wheel
right_wheel
torso_0
torso_1
torso_2
torso_3
torso_4
torso_5
head_0
left_arm_0
right_arm_0
head_1
left_arm_1
right_arm_1
...
left_arm_6
right_arm_6
hand joints...
```

---

### Command output

```text
/isaac_joint_command
```

Type:

```text
sensor_msgs/msg/JointState
```

In v28 RBY1 mode, this publishes the full joint list because the existing Isaac Action Graph expects full-order joint commands.

Only arm joints are actively changed by controller logic.

---

### Delta arm command

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

---

### Forward arm command

```text
/forward_arm_joint_targets
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

RBY1 format:

```text
[left_arm_0 ... left_arm_6, right_arm_0 ... right_arm_6]
```

Total:

```text
14 values
```

---

## 1.8 Doosan/AIDIN existing usage

Select:

```yaml
kinematics:
  active_robot_id: 0
```

Recommended mode:

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'forward'}"
```

Delta arm command:

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.000, 0.000, 0.001, 0.0, 0.0, 0.0,
         0.000, 0.000, 0.001, 0.0, 0.0, 0.0]}"
```

Hand forward command example:

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

Target hand force example:

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray \
"{data: [0, 1, 0.0, 0.0, 5.0]}"
```

---

# 2. Package Structure

## 2.1 Workspace structure

```text
dualarm_ws/
└── src/
    ├── dualarm_forcecon_interfaces/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── srv/
    │       └── SetControlMode.srv
    │
    ├── dualarm_kinematics/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/
    │   │   ├── doosan_dualarm_kinematics.yaml
    │   │   ├── generic_dualarm_kinematics.yaml
    │   │   └── rby1_kinematics.yaml
    │   ├── urdf/
    │   │   ├── rby1_official/
    │   │   │   ├── rby1-sdk/
    │   │   │   └── rby1-dev/
    │   │   ├── rby1a_official_raw.urdf
    │   │   ├── rby1a_kdl.urdf
    │   │   └── make_rby1a_kdl_urdf.py
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
        ├── CMakeLists.txt
        ├── package.xml
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

## 2.2 `dualarm_kinematics` role

`dualarm_kinematics` owns reusable kinematics logic.

It owns:

```text
arm FK
arm IK
position-only DLS IK fallback
hand FK
hand IK
kinematics config loader
robot-specific YAML profiles
RBY1 KDL-compatible URDF
```

It should not own:

```text
ROS publishers
ROS subscribers
ROS service callbacks
control mode state machine
force/admittance controller runtime
```

---

## 2.3 `dualarm_forcecon` role

`dualarm_forcecon` owns ROS 2 runtime control logic.

It owns:

```text
ROS 2 node
current-state callbacks
target callbacks
mode service
control loop
command publishing
full_hold_non_arm publishing policy
hand admittance control
monitor printing
force monitor topics
```

---

## 2.4 File-role separation rules

Preserve this structure.

```text
DualArmForceControl.cpp
= constructor / destructor / ControlLoop only

states_current_callback_dualarm.cpp
= current state callbacks
= joint state parsing
= USD home pose latch
= full joint hold snapshot
= FK monitor update
= contact force callback
= mode service callback
= PrintDualArmStates
= force monitor publish

states_target_callback_dualarm.cpp
= forward arm command callback
= target arm Cartesian pose callback
= delta arm Cartesian pose callback
= forward hand joint target callback
= hand target/delta callbacks
= target hand force callback

node_dualarm_main.cpp
= ROS 2 node creation and spin only
```

---

# 3. Remaining Content

## 3.1 v28 change summary

Major changes from v27:

```text
1. Added RBY1 official URDF workflow.
2. Converted official RBY1 URDF to KDL-compatible URDF.
3. Added RBY1 kinematics config.
4. Set RBY1 arm-only KDL chain base to link_torso_5.
5. Removed remaining 6DOF hardcoding in arm target/current callbacks.
6. Added full_hold_non_arm command publish mode.
7. Added USD home pose latch from first /isaac_joint_states.
8. Added RBY1 7DOF forward arm control support.
9. Added position-only DLS IK fallback for inverse control.
10. Verified RBY1 forward and inverse arm commands.
```

---

## 3.2 Why `full_hold_non_arm` is used

The existing Isaac Action Graph worked with full-order JointState command.

When only 14 arm joints were published:

```text
/isaac_joint_command = left_arm_0~6 + right_arm_0~6 only
```

the graph did not move the robot as expected.

Therefore v28 uses:

```text
full_hold_non_arm
```

Meaning:

```text
/isaac_joint_command publishes all RBY1 joints in the original /isaac_joint_states order.

arm joints:
  updated by controller

non-arm joints:
  held at first observed USD home pose
```

This solves:

```text
1. Existing Action Graph compatibility
2. Torso/wheel/head unwanted command changes
3. USD home pose reset-to-zero problem
```

---

## 3.3 Why `link_torso_5` is used as kinematics base

If RBY1 KDL chain uses:

```text
base -> ee_left
```

then the chain includes:

```text
torso_0~5 + left_arm_0~6
```

Total:

```text
13 joints
```

But the controller provides only 7 arm joints.

Therefore, for arm-only control:

```text
base_link = link_torso_5
```

This makes the chain:

```text
link_torso_5 -> ee_left
link_torso_5 -> ee_right
```

Expected DOF:

```text
7 joints per arm
```

---

## 3.4 Why position-only DLS IK fallback is used

RBY1 KDL full-pose IK failed even for 1 mm delta in the current pose.

Observed failure:

```text
[IK][L] solveIK failed or invalid output size. ok=0 size=0 expected=7
[IK][R] solveIK failed or invalid output size. ok=0 size=0 expected=7
```

So v28 keeps the existing strict IK first, then uses fallback:

```text
strict full-pose KDL IK
  if success:
    use strict IK result
  else:
    use position-only DLS IK
```

DLS equation:

```text
e_p = p_des - p(q)

J_p = ∂p / ∂q

dq = J_p^T (J_p J_p^T + λ² I)^-1 e_p

q_next = q + α dq
```

The fallback controls position only. Orientation is not strictly guaranteed in this fallback mode.

---

## 3.5 Home pose latch behavior

v28 must not send zero pose at startup.

At first `/isaac_joint_states` message:

```text
q_current is latched as USD home pose
q_target is initialized from q_current
q_publish is initialized from q_current
non-arm hold snapshot is initialized from q_current
```

Expected log:

```text
[JointStateInit] latched USD home pose: total_joints=... Ldof=7 Rdof=7 mode=full_hold_non_arm
```

Expected result:

```text
Node startup should not move RBY1 away from USD default pose.
```

---

## 3.6 RBY1 validated status

Validated:

```text
RBY1 /isaac_joint_states received
RBY1 official joint names match Isaac joint names
RBY1 KDL-compatible URDF generated
check_urdf success
RBY1 FK loaded
RBY1 IK object constructed
RBY1 command publish connected to Isaac Action Graph
RBY1 forward joint command works
RBY1 inverse delta command works using DLS fallback
```

Current limitation:

```text
RBY1 inverse fallback is position-only.
Orientation delta is not fully controlled yet.
RBY1 hand/contact force control is not enabled yet.
```

---

## 3.7 RBY1 next tasks

Recommended next steps:

```text
1. Add RBY1 hand command support.
2. Add /isaac_contact_states for RBY1 hands.
3. Add RBY1 selected-finger force control.
4. Improve DLS IK with orientation weighting.
5. Add null-space posture control for 7DOF arm.
6. Add joint limit avoidance.
7. Add torso fixed-base or torso hold policy if needed.
8. Add launch file for selecting robot_profile_id.
```

---

## 3.8 Troubleshooting

### Node starts but robot moves to zero pose

Cause:

```text
current USD home pose was not latched before publishing command
```

Check:

```bash
ros2 topic echo --once /isaac_joint_states
ros2 topic echo --once /isaac_joint_command
```

Arm and non-arm command should match the current state at startup.

---

### FK error: expected 13 got 7

Cause:

```text
rby1_kinematics.yaml uses base_link: base
```

Fix:

```yaml
base_link: link_torso_5
```

---

### Node starts but official URDF parse fails

Cause:

```text
official URDF contains parser-incompatible elements
```

Fix:

```bash
python3 ~/dualarm_ws/src/dualarm_kinematics/urdf/make_rby1a_kdl_urdf.py
check_urdf ~/dualarm_ws/src/dualarm_kinematics/urdf/rby1a_kdl.urdf
```

---

### Forward works but inverse does not

Cause:

```text
strict full-pose IK failed
```

Fix:

```text
Use v28 position-only DLS IK fallback.
Start with 1 mm delta.
```

---

### Robot vibrates when arm moves

Possible causes:

```text
full command updates non-arm joints incorrectly
arm target step is too large
arm command filter is too aggressive
torso/base drive is weak
```

Recommended settings:

```yaml
command_publish:
  mode: full_hold_non_arm
  publish_rate_hz: 60.0
  arm_command_filter_enabled: true
  arm_command_max_step_rad: 0.002
```

Start with:

```text
1 mm delta
```

---

### Robot does not move but command is published

Check subscriber:

```bash
ros2 topic info /isaac_joint_command -v
```

Expected:

```text
Subscription count: 1
Node name: _World_ActionGraph_ros2_subscribe_joint_state
```

Check Action Graph:

```text
ROS2 Subscribe Joint State
  positionCommand -> Articulation Controller.positionCommand
  jointNames      -> Articulation Controller.jointNames

Articulation Controller.robotPath
  /World/rby1
```

---

## 3.9 Recommended git commit

```bash
git add .
git commit -m "Add RBY1 full-order hold and DLS inverse support"
```

More detailed:

```bash
git add .
git commit -m "Add RBY1 full-order hold and DLS inverse support" \
  -m "Introduce RBY1 kinematics configuration, KDL-compatible URDF workflow, full_hold_non_arm command publishing, USD home pose latch, 7-DOF arm callback handling, and position-only DLS IK fallback for inverse arm control."
```

Optional tag:

```bash
git tag -a v28 -m "v28: RBY1 arm inverse control support"
git push origin main
git push origin v28
```

---

# 4. v28 Summary

Before v28:

```text
v27:
  dualarm_kinematics split package exists
  YAML robot profile selector exists
  Doosan/AIDIN dualarm works
  RBY1 profile exists but inverse control is not ready
```

After v28:

```text
v28:
  RBY1 official URDF is used through KDL-compatible cleaned URDF
  RBY1 arm-only FK/IK chain is configured from link_torso_5 to ee_left/right
  RBY1 7DOF arm current/target callbacks work
  /isaac_joint_command keeps full joint order for existing Isaac Action Graph
  non-arm joints are held at first USD home pose
  forward arm joint control works
  inverse delta Cartesian arm control works through position-only DLS fallback
```

Default RBY1 command flow:

```text
/isaac_joint_states
    -> latch current USD home pose
    -> parse current RBY1 arm states

/delta_arm_cartesian_pose
    -> update target EE position
    -> strict full-pose IK
    -> position-only DLS fallback if strict IK fails
    -> update arm target joints

ControlLoop
    -> full_hold_non_arm publish
    -> /isaac_joint_command
```

Recommended RBY1 test mode:

```text
arm_mode  = inverse
hand_mode = idle
```

Recommended first delta:

```text
1 mm
```

RBY1 hand/contact force control is not yet enabled in v28. It should be added after arm-only behavior is stable.
