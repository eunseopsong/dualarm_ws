# DualArmForceControl README (v27)

`v27` is the version where the original `dualarm_forcecon` package was refactored into a controller package plus a reusable include-only kinematics package.

The main goal of v27 is:

```text
dualarm_forcecon
= ROS 2 controller node, mode handling, topic/service interface, force/admittance control

dualarm_kinematics
= reusable FK/IK, robot kinematics YAML profile, joint/link mapping
```

---

# Quick Start Example Commands

## Build

```bash
cd ~/dualarm_ws
colcon build --symlink-install
source install/setup.bash
```

If you use the `cb` alias:

```bash
cd ~/dualarm_ws
cb
```

---

## Run dualarm force controller

```bash
source ~/dualarm_ws/install/setup.bash

ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml
```

---

## Set control mode

### Idle

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'idle', hand_mode: 'idle'}"
```

### Arm forward mode

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'forward', hand_mode: 'idle'}"
```

### Arm inverse mode

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'idle'}"
```

### Hand forward mode

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'idle', hand_mode: 'forward'}"
```

### Arm + hand forward mode

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'forward', hand_mode: 'forward'}"
```

---

## Arm forward command example

For the existing 6-DOF left arm + 6-DOF right arm configuration:

```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

For a 7-DOF left arm + 7-DOF right arm robot such as an RBY1-style profile:

```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

The expected command length is automatically determined from the selected kinematics YAML profile.

---

## Arm inverse command example

First, switch to arm inverse mode:

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'inverse', hand_mode: 'idle'}"
```

Then publish the target Cartesian pose:

```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray \
"{data: [0.30,  0.20, 0.40, 0.0, 0.0, 0.0,
         0.30, -0.20, 0.40, 0.0, 0.0, 0.0]}"
```

Format:

```text
[L x y z roll pitch yaw, R x y z roll pitch yaw]
```

Default unit convention:

```text
position    : meter
orientation : radian
```

---

## Check output command

```bash
ros2 topic info /isaac_joint_command
ros2 topic echo /isaac_joint_command
```

---

# 0. Dependency

## 0.1 ROS 2 environment

Target environment:

```text
Ubuntu 22.04
ROS 2 Humble
C++17
```

Recommended setup:

```bash
source /opt/ros/humble/setup.bash
source ~/dualarm_ws/install/setup.bash
```

---

## 0.2 ROS 2 packages

The workspace should contain at least:

```text
dualarm_forcecon_interfaces
dualarm_kinematics
dualarm_forcecon
```

Optional or external packages may also exist in the same workspace, for example:

```text
rby1_joystick_teleop
```

---

## 0.3 External libraries

The following libraries are required by the kinematics and controller code:

```text
Eigen3
yaml-cpp
Pinocchio
```

Typical installation:

```bash
sudo apt update
sudo apt install -y \
  libeigen3-dev \
  libyaml-cpp-dev
```

Pinocchio installation depends on the current system setup.  
If Pinocchio is already used in the previous `dualarm_forcecon` version, no additional change is usually required.

Check whether CMake can find Pinocchio:

```bash
pkg-config --modversion pinocchio
```

or:

```bash
ros2 pkg prefix pinocchio
```

depending on how Pinocchio was installed.

---

## 0.4 Package dependency relationship

```text
dualarm_kinematics
  ├── Eigen3
  ├── yaml-cpp
  ├── Pinocchio
  └── geometry_msgs

dualarm_forcecon
  ├── rclcpp
  ├── std_msgs
  ├── sensor_msgs
  ├── geometry_msgs
  ├── dualarm_forcecon_interfaces
  └── dualarm_kinematics
```

`dualarm_kinematics` should remain independent from ROS runtime logic as much as possible.  
It should not own publishers, subscribers, services, or control mode logic.

---

# 1. Usage and Example Commands

## 1.1 Build the workspace

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

## 1.2 Run the controller node

```bash
source ~/dualarm_ws/install/setup.bash

ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml
```

The node name is:

```text
dualarm_forcecon_node
```

---

## 1.3 Select the robot kinematics profile

In v27, robot-specific FK/IK configuration is separated from the controller.

The controller YAML file is:

```text
dualarm_forcecon/yaml/forcecon_cfg.yaml
```

Inside this file, select the kinematics YAML profile:

```yaml
kinematics:
  config_package: dualarm_kinematics
  config_file: doosan_dualarm_kinematics.yaml
```

To switch to RBY1 later:

```yaml
kinematics:
  config_package: dualarm_kinematics
  config_file: rby1_kinematics.yaml
```

Available kinematics YAML files are stored in:

```text
dualarm_kinematics/config/
```

Example:

```text
dualarm_kinematics/config/doosan_dualarm_kinematics.yaml
dualarm_kinematics/config/rby1_kinematics.yaml
dualarm_kinematics/config/generic_dualarm_kinematics.yaml
```

---

## 1.4 Existing Doosan / AIDIN dualarm usage

For the current existing dualarm setup, use:

```yaml
kinematics:
  config_package: dualarm_kinematics
  config_file: doosan_dualarm_kinematics.yaml
```

Then run:

```bash
ros2 run dualarm_forcecon dualarm_forcecon_node --ros-args \
  -p forcecon_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_forcecon/yaml/forcecon_cfg.yaml
```

Switch to arm forward mode:

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'forward', hand_mode: 'idle'}"
```

Send a 12-dimensional command:

```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

---

## 1.5 RBY1-style arm-only usage

For RBY1, first update:

```text
dualarm_kinematics/config/rby1_kinematics.yaml
```

The current RBY1 YAML is a template.  
The following fields must match the actual RBY1 URDF:

```yaml
robot:
  name: rby1
  type: dual_arm
  urdf_path: /absolute/path/to/rby1.urdf

arm:
  left:
    base_link: base_link
    tip_link: left_ee_link
    joint_names:
      - left_arm_joint_1
      - left_arm_joint_2
      - left_arm_joint_3
      - left_arm_joint_4
      - left_arm_joint_5
      - left_arm_joint_6
      - left_arm_joint_7

  right:
    base_link: base_link
    tip_link: right_ee_link
    joint_names:
      - right_arm_joint_1
      - right_arm_joint_2
      - right_arm_joint_3
      - right_arm_joint_4
      - right_arm_joint_5
      - right_arm_joint_6
      - right_arm_joint_7

hand:
  enabled: false
```

Then select it in:

```text
dualarm_forcecon/yaml/forcecon_cfg.yaml
```

```yaml
kinematics:
  config_package: dualarm_kinematics
  config_file: rby1_kinematics.yaml
```

For 7-DOF + 7-DOF arm forward control, send a 14-dimensional command:

```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

---

## 1.6 Control mode service

Service:

```text
/change_control_mode
```

Type:

```text
dualarm_forcecon_interfaces/srv/SetControlMode
```

Request:

```text
string arm_mode
string hand_mode
```

Response:

```text
bool success
string message
```

Supported arm modes:

```text
idle
forward
inverse
```

Supported hand modes:

```text
idle
forward
inverse
```

There is no separate hand `forcecon` mode in v27.

---

## 1.7 Main ROS topics

### State input

```text
/isaac_joint_states
```

Type:

```text
sensor_msgs/msg/JointState
```

Used for:

```text
current arm joint state
current hand joint state
current FK monitor
target/current state printing
```

---

### Contact input

```text
/isaac_contact_states
```

Type:

```text
std_msgs/msg/Float32MultiArray
```

Expected layout for current hand force pipeline:

```text
5 left fingers + 5 right fingers
```

---

### Arm forward command

```text
/forward_arm_joint_targets
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Command size:

```text
left_arm_dof + right_arm_dof
```

Examples:

```text
6 + 6 = 12 for current Doosan/AIDIN dualarm profile
7 + 7 = 14 for RBY1-style arm profile
```

---

### Arm inverse command

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

---

### Arm delta inverse command

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

Delta arm command is interpreted relative to the latched initial base pose.

---

### Hand forward command

```text
/forward_hand_joint_targets
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Supported lengths depend on the selected hand configuration.  
For the existing hand pipeline, common supported lengths are:

```text
30 = left15 + right15
40 = left20 + right20
42 = legacy-compatible
52 = legacy-compatible
```

---

### Hand inverse command

```text
/target_hand_fingertips
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Expected length:

```text
30
```

Per-hand order:

```text
thumb xyz
index xyz
middle xyz
ring xyz
baby xyz
```

---

### Hand delta command

```text
/delta_hand_fingertips
```

Type:

```text
std_msgs/msg/Float64MultiArray
```

Format:

```text
[side, finger, dx, dy, dz]
```

where:

```text
side   : 0 = left, 1 = right
finger : 0 = thumb, 1 = index, 2 = middle, 3 = ring, 4 = baby
```

---

### Hand desired force

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

---

### Main command output

```text
/isaac_joint_command
```

Type:

```text
sensor_msgs/msg/JointState
```

This topic publishes the final combined arm + hand command.

---

### Force monitor topics

```text
/hand_force_current_monitor
/hand_force_target_monitor
```

Type:

```text
std_msgs/msg/Float32MultiArray
```

Useful for:

```text
rqt_plot
force tracking debug
checking current/target force sign and axis convention
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
    │   └── include/
    │       └── dualarm_kinematics/
    │           ├── arm/
    │           │   ├── arm_forward_kinematics.hpp
    │           │   └── arm_inverse_kinematics.hpp
    │           │
    │           ├── hand/
    │           │   ├── hand_forward_kinematics.hpp
    │           │   └── hand_inverse_kinematics.hpp
    │           │
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

`dualarm_kinematics` is an include-only reusable kinematics package.

It owns:

```text
arm FK
arm IK
hand FK
hand IK
kinematics configuration loader
joint/link name mapping
robot kinematics YAML profiles
```

It should not own:

```text
ROS publishers
ROS subscribers
ROS services
control loop timers
force control state machine
mode management
```

This separation allows other packages to reuse the same FK/IK code.

Possible future users:

```text
dualarm_forcecon
dualarm_teleop
dualarm_planner
dualarm_imitation
rby1_control
```

---

## 2.3 `dualarm_forcecon` role

`dualarm_forcecon` owns the runtime control node.

It owns:

```text
ROS 2 node
topic subscriptions
command publishers
mode service
control loop
state monitor
hand admittance control
force control logic
final joint command publishing
```

It depends on `dualarm_kinematics` for FK/IK.

---

## 2.4 File-role separation rules

These rules should be preserved.

```text
DualArmForceControl.cpp
= constructor / destructor / ControlLoop only

states_current_callback_dualarm.cpp
= current-state callbacks
= joint state parsing
= FK monitor update
= contact force callback
= mode service callback
= PrintDualArmStates
= force monitor publish

states_target_callback_dualarm.cpp
= target command callbacks
= arm target pose
= arm delta pose
= hand target pose
= hand delta pose
= forward joint target
= desired hand force target

node_dualarm_main.cpp
= ROS 2 node creation and spin only
```

Do not move all callback logic back into `DualArmForceControl.cpp`.

---

# 3. Remaining Content

## 3.1 Version Summary

`v27` is based on the previous `v26` YAML-based kinematics patch, but introduces a cleaner package split.

Major changes:

```text
1. Created new include-only package: dualarm_kinematics
2. Moved arm FK/IK code into dualarm_kinematics
3. Moved hand FK/IK code into dualarm_kinematics
4. Moved kinematics config loader into dualarm_kinematics
5. Kept hand_admittance_control.hpp inside dualarm_forcecon
6. Updated dualarm_forcecon to depend on dualarm_kinematics
7. Added robot-specific kinematics YAML profiles
8. Preserved existing ROS topic/service control flow
```

---

## 3.2 Why `hand_admittance_control.hpp` stays in `dualarm_forcecon`

Although `hand_admittance_control.hpp` was previously located under `Kinematics`, its actual role is controller-side logic.

It uses:

```text
desired fingertip position
measured force
desired force
contact state
admittance dynamics
IK output correction
```

Therefore, it belongs to:

```text
dualarm_forcecon/include/dualarm_forcecon/control/
```

not:

```text
dualarm_kinematics/include/dualarm_kinematics/
```

The kinematics package should remain focused on reusable geometry and kinematics calculations.

---

## 3.3 Kinematics YAML design

Robot-specific FK/IK information is stored in:

```text
dualarm_kinematics/config/*.yaml
```

Recommended YAML structure:

```yaml
robot:
  name: doosan_dualarm
  type: dual_arm
  urdf_path: /absolute/path/to/robot.urdf

arm:
  left:
    base_link: base_link
    tip_link: left_link_6
    joint_names:
      - left_joint_1
      - left_joint_2
      - left_joint_3
      - left_joint_4
      - left_joint_5
      - left_joint_6

  right:
    base_link: base_link
    tip_link: right_link_6
    joint_names:
      - right_joint_1
      - right_joint_2
      - right_joint_3
      - right_joint_4
      - right_joint_5
      - right_joint_6

hand:
  enabled: true
```

For robots without the existing hand model:

```yaml
hand:
  enabled: false
```

---

## 3.4 Controller YAML design

Controller-side configuration remains in:

```text
dualarm_forcecon/yaml/forcecon_cfg.yaml
```

Example:

```yaml
kinematics:
  config_package: dualarm_kinematics
  config_file: doosan_dualarm_kinematics.yaml
```

The force controller should not directly hard-code robot-specific joint/link names.

---

## 3.5 Arm control behavior

### Arm idle mode

```text
current arm state -> target arm state
```

In idle mode, target arm joints are synchronized to the current arm joints.

---

### Arm forward mode

```text
/forward_arm_joint_targets
    -> TargetArmJointsCallback()
    -> q_l_t_, q_r_t_
    -> ControlLoop()
    -> /isaac_joint_command
```

The expected command length is:

```text
left_arm_joint_names.size() + right_arm_joint_names.size()
```

This is determined from the selected kinematics YAML file.

---

### Arm inverse mode

```text
/target_arm_cartesian_pose
or
/delta_arm_cartesian_pose
    -> Arm IK
    -> q_l_t_, q_r_t_
    -> ControlLoop()
    -> /isaac_joint_command
```

The IK backend is selected through the kinematics package and the selected robot YAML profile.

---

## 3.6 Hand control behavior

The v27 hand control behavior follows the v25/v26 unified structure.

### Hand idle mode

```text
current hand joints -> motion target
current hand joints -> final command
clear desired hand force monitor
```

---

### Hand forward mode

```text
/forward_hand_joint_targets
    -> q_h_motion_t_
    -> optional selected-finger admittance correction
    -> q_h_t_
    -> /isaac_joint_command
```

---

### Hand inverse mode

```text
/target_hand_fingertips
or
/delta_hand_fingertips
    -> hand Cartesian target
    -> optional selected-finger admittance correction
    -> hand IK
    -> q_h_t_
    -> /isaac_joint_command
```

---

### Selected-finger force correction

Desired hand force is received from:

```text
/target_hand_force
```

The selected finger gets admittance-based correction.  
Non-selected fingers continue to follow the motion reference.

---

## 3.7 Important internal state variables

### Arm state

```text
q_l_c_ : current left arm joints
q_r_c_ : current right arm joints

q_l_t_ : target/final left arm command
q_r_t_ : target/final right arm command
```

The vector size depends on the selected kinematics YAML profile.

---

### Hand state

```text
q_l_h_c_        : current left hand joints
q_r_h_c_        : current right hand joints

q_l_h_motion_t_ : left hand motion reference
q_r_h_motion_t_ : right hand motion reference

q_l_h_t_        : final left hand command
q_r_h_t_        : final right hand command
```

---

### Hand force state

```text
f_l_hand_c_ : current left hand measured/contact force
f_r_hand_c_ : current right hand measured/contact force

f_l_hand_t_ : desired left hand force
f_r_hand_t_ : desired right hand force
```

Canonical finger row order:

```text
0 thumb
1 index
2 middle
3 ring
4 baby
```

---

### Active desired-force latch

```text
hand_force_cmd_valid_
hand_force_cmd_hand_id_
hand_force_cmd_finger_id_
hand_force_cmd_f_des_base_
hand_force_cmd_stamp_ns_
```

---

## 3.8 Unit and frame convention

Default convention:

```text
position    : meter
orientation : radian
force       : Newton
```

Arm Cartesian command format:

```text
x y z roll pitch yaw
```

Default target frame is controlled by:

```yaml
ik_targets_frame
ik_euler_conv
ik_angle_unit
```

Typical values:

```yaml
ik_targets_frame: base
ik_euler_conv: rpy
ik_angle_unit: rad
```

The default world-base z offset behavior from the previous baseline is preserved.

---

## 3.9 Preserved invariants

The following behavior should remain compatible with the previous baseline:

```text
existing dualarm command publish flow
separated arm/hand mode service
Isaac UI-matching pose convention
world-base z offset default behavior
PrintDualArmStates layout and ordering
selected-finger-only hand force correction
hand force monitor topics
```

The main structural change is package separation, not a change in the external ROS interface.

---

## 3.10 Current RBY1 status

RBY1 support is structurally prepared but not fully validated.

Current state:

```text
dualarm_kinematics package exists
rby1_kinematics.yaml template exists
forcecon can select rby1_kinematics.yaml
arm DOF is no longer hard-coded to 6
```

Still required:

```text
actual RBY1 URDF path
actual RBY1 left/right arm joint names
actual RBY1 base link name
actual RBY1 left/right EE tip link names
IK seed tuning
joint limit validation
runtime FK/IK test
```

Recommended next check:

```bash
ros2 run xacro xacro /path/to/rby1.urdf.xacro > /tmp/rby1.urdf
check_urdf /tmp/rby1.urdf
```

Then inspect names:

```bash
grep 'name=".*joint' /tmp/rby1.urdf | head -100
grep 'name=".*link'  /tmp/rby1.urdf | head -100
```

---

## 3.11 Troubleshooting

### Error: cannot find `dualarm_kinematics/arm/arm_forward_kinematics.hpp`

Cause:

```text
dualarm_forcecon is not receiving the include path exported by dualarm_kinematics
```

Check:

```bash
ls ~/dualarm_ws/src/dualarm_kinematics/include/dualarm_kinematics/arm/arm_forward_kinematics.hpp
```

Then check `dualarm_forcecon/CMakeLists.txt`:

```cmake
find_package(dualarm_kinematics REQUIRED)
```

and:

```cmake
ament_target_dependencies(dualarm_forcecon_node
  ...
  dualarm_kinematics
)
```

Also check `dualarm_forcecon/package.xml`:

```xml
<depend>dualarm_kinematics</depend>
```

---

### Error: `pinocchio::pinocchio` target was not found

Cause:

```text
dualarm_kinematics exports a Pinocchio-linked interface target,
but the consuming package did not call find_package(pinocchio REQUIRED)
```

Fix in `dualarm_forcecon/CMakeLists.txt`:

```cmake
find_package(pinocchio REQUIRED)
```

---

### Error after failed build: local_setup.bash not found

Example:

```text
not found: "/home/eunseop/dualarm_ws/install/dualarm_forcecon/share/dualarm_forcecon/local_setup.bash"
```

Cause:

```text
dualarm_forcecon build failed, so its install setup file was not generated
```

Fix the build error first, then rebuild:

```bash
cd ~/dualarm_ws
rm -rf build/dualarm_forcecon install/dualarm_forcecon log
colcon build --symlink-install
source install/setup.bash
```

---

### Command publishes but robot does not move

Check command topic type and subscriber count:

```bash
ros2 topic info /isaac_joint_command
```

Check whether node is publishing:

```bash
ros2 topic echo /isaac_joint_command
```

Check current joint state input:

```bash
ros2 topic echo --once /isaac_joint_states
```

Check control mode:

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode \
"{arm_mode: 'forward', hand_mode: 'idle'}"
```

---

## 3.12 Recommended git commit

```bash
git add .
git commit -m "Refactor kinematics into reusable dualarm_kinematics package" \
  -m "Move arm and hand FK/IK headers into a new include-only dualarm_kinematics package. Update dualarm_forcecon to depend on the external kinematics backend while preserving the v27 controller structure and YAML-based robot configuration."
```

Optional tag:

```bash
git tag -a v27 -m "v27: split reusable kinematics package"
git push origin main
git push origin v27
```

---

# 4. v27 Summary

`v27` keeps the controller behavior close to the previous baseline, while making the architecture more extensible.

Before v27:

```text
dualarm_forcecon
= controller + FK/IK + robot-specific kinematics code
```

After v27:

```text
dualarm_kinematics
= reusable robot kinematics package

dualarm_forcecon
= ROS 2 force/admittance controller package
```

This makes it easier to add:

```text
Doosan dualarm
RBY1
custom dual-arm robot
arm-only robot
future hand models
```

by editing or adding kinematics YAML profiles instead of modifying the force controller core.
