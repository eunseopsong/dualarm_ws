# DualArmForceControl README (v22)

This README summarizes the current **v22** baseline of `dualarm_forcecon` (dual arm + hand monitor / control node) based on the latest package structure, force-control flow, monitor outputs, v20/v21 simplifications, and the new **latched-base delta arm command behavior** added in v22.

---

## 1) Version Summary (v22)

v22 builds on:
- **v20**: hand admittance force control successfully working, target/current hand-force monitor topics added
- **v21**: hand admittance configuration cleanup (unused false-path options removed, YAML simplified)
- **v22**: arm delta Cartesian command changed to use a **latched initial base pose** rather than a continuously updated current pose

### v22 major changes
- **Delta arm command reference behavior changed**
  - `DeltaArmPositionCallback()` no longer interprets delta relative to the current arm pose at every callback
  - It now uses:
    - **position target = latched_initial_position + delta_position**
    - **orientation target = latched_initial_orientation + delta_orientation**
- **Latched initial arm base pose added**
  - `delta_arm_base_pose_l_`, `delta_arm_base_pose_r_`
  - `delta_arm_base_pose_initialized_`
  - These are captured once after node start when current arm pose becomes valid
- **v21 config simplification preserved**
  - Unused false-path config logic removed from hand admittance code and YAML loader
  - Force control path is cleaner and easier to maintain
- **v20 force-control success preserved**
  - Hand admittance control remains operational
  - `/hand_force_current_monitor`
  - `/hand_force_target_monitor`
  can be used for plotting in `rqt_plot`

### Important behavioral note for v22
`/delta_arm_cartesian_pose` is no longer an incremental jog based on the latest pose at each callback.  
It is now an **initial-pose-relative offset command**.

That means:

\[
X_{target} = X_{init} + \Delta X
\]

not

\[
X_{target}(k) = X_{current}(k) + \Delta X(k)
\]

So if you keep publishing the same delta command repeatedly, the target remains the same.

---

## 2) Package Structure (v22, must preserve)

> Keep the package tree and file-role separation rules unchanged.

```text
dualarm_forcecon/
├── CMakeLists.txt
├── package.xml
├── yaml/
│   └── forcecon_cfg.yaml
├── include/dualarm_forcecon/Kinematics/
│   ├── arm_forward_kinematics.hpp
│   ├── arm_inverse_kinematics.hpp
│   ├── hand_admittance_control.hpp
│   ├── hand_forward_kinematics.hpp
│   ├── hand_inverse_kinematics.hpp
│   └── kinematics_utils.hpp
└── src/
    ├── DualArmForceControl.cpp                 # ctor / dtor / ControlLoop only
    ├── DualArmForceControl.h
    ├── node_dualarm_main.cpp
    ├── states_current_callback_dualarm.cpp     # current-state callbacks + monitor print
    └── states_target_callback_dualarm.cpp      # target/command callbacks
```

### Important rules (must keep)
- **Do not change package tree**
- **Do not move callbacks back into `DualArmForceControl.cpp`**
- `DualArmForceControl.cpp` should keep only:
  - constructor
  - destructor
  - `ControlLoop()`
- Preserve:
  - 52-DOF publish mapping
  - Isaac UI-matching Euler convention
  - world-base z offset default behavior (`0.306 m`)
  - `PrintDualArmStates` formatting style (colors/layout)
  - include/src separation

---

## 3) Core Functional Overview

### 3.1 Control modes
- `idle`
- `forward`
- `inverse`
- `forcecon`

### 3.2 What each mode does
- **idle**
  - safe hold / target sync to current state
- **forward**
  - direct joint-space command mode
  - arm and hand joint targets are received separately
- **inverse**
  - Cartesian target mode
  - arm absolute Cartesian targets
  - arm delta Cartesian targets
  - hand absolute fingertip Cartesian targets
  - hand delta fingertip jog commands
- **forcecon**
  - hand fingertip force-control mode
  - single-finger force-oriented command path via `HandAdmittanceControl`
  - arm/wrist is frozen by forcecon hold snapshot
  - selected finger is updated by force controller + hand IK

### 3.3 Monitor output
- Arm:
  - current / target pose
  - current / target force (arm force monitor still unused, typically zero)
- Hand:
  - current / target fingertip positions in hand-base frame
  - current / target hand forces
- Extra plot topics:
  - `/hand_force_current_monitor`
  - `/hand_force_target_monitor`

---

## 4) ROS Interfaces (v22)

### 4.1 Subscriptions

#### Joint / state input
- `/isaac_joint_states` — `sensor_msgs/msg/JointState`
  - used by:
    - `JointsCallback`
    - `PositionCallback`

#### Hand contact monitor input
- `/isaac_contact_states` — `std_msgs/msg/Float32MultiArray`
  - length = **10** (`5 left + 5 right`)
  - scalar fingertip contact values from Isaac Sim

#### Forward mode inputs
- `/forward_arm_joint_targets` — `std_msgs/msg/Float64MultiArray`
  - **12 values**
  - `[left arm 6, right arm 6]`

- `/forward_hand_joint_targets` — `std_msgs/msg/Float64MultiArray`
  - **30 values** = `left15 + right15`
  - **40 values** = `left20 + right20`

#### Inverse mode inputs
- `/target_arm_cartesian_pose` — `std_msgs/msg/Float64MultiArray`
  - **12 values**
  - `[L x y z r p y, R x y z r p y]`

- `/delta_arm_cartesian_pose` — `std_msgs/msg/Float64MultiArray`
  - **12 values**
  - `[L dx dy dz droll dpitch dyaw, R dx dy dz droll dpitch dyaw]`
  - **v22 behavior**:
    - delta is interpreted relative to the **latched initial pose**, not current pose

- `/target_hand_fingertips` — `std_msgs/msg/Float64MultiArray`
  - **30 values**
  - per hand order = `THMB, INDX, MIDL, RING, BABY` (each xyz)

- `/delta_hand_fingertips` — `std_msgs/msg/Float64MultiArray`
  - **5 values**
  - `[side, finger, dx, dy, dz]`
  - `side: 0=left, 1=right`
  - `finger: 0=thumb, 1=index, 2=middle, 3=ring, 4=baby`

#### Forcecon mode input
- `/target_hand_force` — `std_msgs/msg/Float64MultiArray`
  - **8 values**
  - `[side, finger, p_des_x, p_des_y, p_des_z, f_des_x, f_des_y, f_des_z]`

### 4.2 Publishers

- `/isaac_joint_command` — `sensor_msgs/msg/JointState`
  - consolidated arm + hand command output

- `/hand_force_current_monitor` — `std_msgs/msg/Float32MultiArray`
  - hand current force monitor topic
  - intended for plotting in `rqt_plot`

- `/hand_force_target_monitor` — `std_msgs/msg/Float32MultiArray`
  - hand target force monitor topic
  - intended for plotting in `rqt_plot`

### 4.3 Service

- `/change_control_mode` — `std_srvs/srv/Trigger`
  - cycles:
    - `idle -> forward -> inverse -> forcecon -> idle`

---

## 5) Coordinate / Frame Conventions

### 5.1 Arm pose display
- Arm pose in monitor uses project Isaac UI-matching Euler convention
- world-base transform default:
  - translation = `[0.0, 0.0, 0.306]`
  - rotation = `[0, 0, 0]` deg unless parameterized

### 5.2 Arm inverse input
- controlled by:
  - `ik_targets_frame_`
  - `ik_euler_conv_`
  - `ik_angle_unit_`

### 5.3 Hand pose display / command frame
- Hand fingertip positions are displayed in:
  - `LEFT_HAND_BASE`
  - `RIGHT_HAND_BASE`
- Hand FK axis convention root fix is already integrated
- `HandPositionCallback()` uses corrected hand-base display convention

### 5.4 Hand fingertip order
- **monitor print order**:
  - `BABY -> RING -> MIDL -> INDX -> THMB`

- **command order for `/target_hand_fingertips` (per hand)**:
  - `THMB, INDX, MIDL, RING, BABY`

### 5.5 Hand contact sensor ordering
- observed sensor order per hand from Isaac:
  - `[BABY, RING, MIDL, INDX, THMB]`
- internal canonical storage:
  - `THMB(0), INDX(1), MIDL(2), RING(3), BABY(4)`

---

## 6) Current Baseline Behavior (v22)

### 6.1 Forcecon
- hand force control works
- target force monitor remains visible
- current force and target force can be plotted externally via:
  - `/hand_force_current_monitor`
  - `/hand_force_target_monitor`

### 6.2 Delta arm command
- initial arm pose is latched once after node start
- delta arm target is computed from that latched pose
- repeated publication of the same delta command does **not accumulate**
- this is intentional in v22

### 6.3 Config state
- unused false-path hand-admittance config logic has been removed
- YAML is simplified
- controller path is cleaner than v20/v21 transitional versions

---

## 7) Build / Run / Mode Switching / Example Commands (v22)

### 7.1 Build and run node

```bash
cd ~/dualarm_ws
colcon build --packages-select dualarm_forcecon
source install/setup.bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

### 7.2 Change control mode

```bash
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

#### Mode cycle
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

> Always verify the current mode from the monitor header before sending commands.

### 7.3 Forward mode examples — arm joint targets

#### Topic
`/forward_arm_joint_targets`

#### Format
`[left arm 6, right arm 6]`

#### Example A — home-like arm command
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  1.6419, -0.0058, -2.2344, 1.5607, 1.5936, -0.1104,
 -0.0459,  0.7361,  1.9809, 0.0000,-1.1869, -0.7854
]}"
```

#### Example B — small symmetric arm change
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  1.6819, -0.0258, -2.1944, 1.5307, 1.5736, -0.0704,
 -0.0859,  0.7561,  1.9409, 0.0300,-1.1669, -0.8254
]}"
```

#### Example C — left arm move only
```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  1.7219,  0.0342, -2.1244, 1.4907, 1.5436, -0.0304,
 -0.0459,  0.7361,  1.9809, 0.0000,-1.1869, -0.7854
]}"
```

### 7.4 Forward mode examples — hand joint targets

#### Topic
`/forward_hand_joint_targets`

#### 15-DoF order per hand
`[thumb1,2,3, index1,2,3, middle1,2,3, ring1,2,3, baby1,2,3]`

#### Example A — home-like hand command
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0000,0.3948,0.3927,   0.0000,0.0000,0.0000,   0.0000,0.0000,0.0000,   0.0000,-0.0400,0.7854,   0.0000,0.0000,0.0000,
  0.0000,0.3955,0.3840,   0.0000,0.0000,0.0000,   0.0000,0.0000,0.0000,   0.0000, 0.0000,0.0000,   0.0000,0.0000,0.0000
]}"
```

#### Example B — gentle closing motion
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0,0.55,0.45,   0.0,0.25,0.20,   0.0,0.25,0.20,   0.0,0.30,0.25,   0.0,0.20,0.15,
  0.0,0.55,0.45,   0.0,0.25,0.20,   0.0,0.25,0.20,   0.0,0.25,0.20,   0.0,0.20,0.15
]}"
```

#### Example C — left-only gesture
```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0,0.80,0.80,   0.0,0.00,0.00,   0.0,0.00,0.00,   0.0,0.00,1.00,   0.0,0.00,0.00,
  0.0,0.3955,0.3840,   0.0,0.00,0.00,   0.0,0.00,0.00,   0.0,0.00,0.00,   0.0,0.00,0.00
]}"
```

### 7.5 Inverse mode examples — absolute arm Cartesian targets

#### Topic
`/target_arm_cartesian_pose`

#### Format
`[L x y z r p y, R x y z r p y]`

#### Example A — near current baseline
```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.1001,  0.2666, -0.1263,   2.7370, 1.5062, -1.1601,
  0.5439, -0.3170,  0.1478,  -3.0216, 1.5565, -1.6910
]}"
```

#### Example B — small Cartesian shift
```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.1201,  0.2566, -0.1163,   2.7370, 1.5062, -1.1601,
  0.5339, -0.3070,  0.1478,  -3.0216, 1.5565, -1.6910
]}"
```

#### Example C — left arm only
```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.1400,  0.2500, -0.1100,   2.70, 1.50, -1.10,
  0.5439, -0.3170,  0.1478,  -3.0216, 1.5565, -1.6910
]}"
```

### 7.6 Inverse mode examples — delta arm Cartesian targets (v22 behavior)

#### Topic
`/delta_arm_cartesian_pose`

#### Format
`[L dx dy dz droll dpitch dyaw, R dx dy dz droll dpitch dyaw]`

#### Important v22 behavior
The delta is interpreted relative to the **latched initial arm pose**, not the latest current pose.

#### Example A — zero delta
```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0,  0,0,0,0,0,0]}"
```

#### Example B — symmetric position offset from initial pose
```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
   0.020, -0.010, 0.015,   0.00, 0.00, 0.00,
  -0.020,  0.010, 0.015,   0.00, 0.00, 0.00
]}"
```

#### Example C — left orientation offset from initial pose
```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
  0.000, 0.000, 0.000,   0.05, -0.03, 0.08,
  0.000, 0.000, 0.000,   0.00,  0.00, 0.00
]}"
```

#### Example D — mixed offset command
```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
   0.030, 0.000, 0.020,   0.04, 0.00, -0.02,
  -0.010, 0.015, 0.010,   0.00, 0.03,  0.00
]}"
```

> Repeating the same command does not accumulate motion in v22.

### 7.7 Inverse mode examples — absolute hand fingertip targets

#### Topic
`/target_hand_fingertips`

#### Format
30 values = `left15 + right15`  
per hand order = `THMB, INDX, MIDL, RING, BABY` (each xyz)

#### Example A — baseline-like fingertip target
```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
   0.1524,-0.0471,-0.1000,   0.2465,-0.0403,-0.0144,   0.2640,-0.0135,-0.0144,   0.2340, 0.0133,-0.0444,   0.2310, 0.0401,-0.0144,
   0.1522, 0.0472,-0.1003,   0.2465, 0.0403,-0.0144,   0.2640, 0.0135,-0.0144,   0.2465,-0.0133,-0.0144,   0.2310,-0.0401,-0.0144
]}"
```

#### Example B — left ring target move
```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
   0.1524,-0.0471,-0.1000,   0.2465,-0.0403,-0.0144,   0.2640,-0.0135,-0.0144,   0.2440, 0.0133,-0.0444,   0.2310, 0.0401,-0.0144,
   0.1522, 0.0472,-0.1003,   0.2465, 0.0403,-0.0144,   0.2640, 0.0135,-0.0144,   0.2465,-0.0133,-0.0144,   0.2310,-0.0401,-0.0144
]}"
```

#### Example C — left baby target move
```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
   0.1524,-0.0471,-0.1000,   0.2465,-0.0403,-0.0144,   0.2640,-0.0135,-0.0144,   0.2340, 0.0133,-0.0444,   0.2310, 0.0401,-0.0244,
   0.1522, 0.0472,-0.1003,   0.2465, 0.0403,-0.0144,   0.2640, 0.0135,-0.0144,   0.2465,-0.0133,-0.0144,   0.2310,-0.0401,-0.0144
]}"
```

### 7.8 Inverse mode examples — delta hand fingertip jog

#### Topic
`/delta_hand_fingertips`

#### Format
`[side, finger, dx, dy, dz]`

- `side: 0=left, 1=right`
- `finger: 0=thumb, 1=index, 2=middle, 3=ring, 4=baby`

#### Example A — left ring +x jog
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.005, 0.000, 0.000]}"
```

#### Example B — left ring -x jog
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [0, 3, -0.005, 0.000, 0.000]}"
```

#### Example C — left ring +z jog
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.000, 0.000, 0.005]}"
```

#### Example D — right index -y jog
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [1, 1, 0.000, -0.005, 0.000]}"
```

#### Example E — right thumb combined jog
```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [1, 0, 0.003, 0.000, -0.003]}"
```

### 7.9 Forcecon mode examples — target hand force

#### Topic
`/target_hand_force`

#### Format
`[side, finger, p_des_x, p_des_y, p_des_z, f_des_x, f_des_y, f_des_z]`

#### Step A — enter forcecon mode
```bash
# idle -> forward
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"

# forward -> inverse
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"

# inverse -> forcecon
ros2 service call /change_control_mode std_srvs/srv/Trigger "{}"
```

#### Example B — left ring force target
```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.0133, -0.0331, 0.2384, 10.0, 0.0, 0.0]}"
```

#### Example C — streaming force target
```bash
ros2 topic pub -r 10 /target_hand_force std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.0133, -0.0331, 0.2384, 8.0, 0.0, 0.0]}"
```

#### Example D — right index force target
```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [1, 1, 0.2465, 0.0403, -0.0144, 0.0, 5.0, 0.0]}"
```

### 7.10 Plotting current / target force in rqt_plot

#### Current hand force monitor
```bash
ros2 topic echo /hand_force_current_monitor
```

#### Target hand force monitor
```bash
ros2 topic echo /hand_force_target_monitor
```

#### Open rqt_plot
```bash
rqt_plot
```

Typical plot targets:
- `/hand_force_current_monitor/data[0]`
- `/hand_force_current_monitor/data[1]`
- `/hand_force_current_monitor/data[2]`
- `/hand_force_target_monitor/data[0]`
- `/hand_force_target_monitor/data[1]`
- `/hand_force_target_monitor/data[2]`

> Exact indices depend on how your monitor publisher packs the 5-finger x/y/z data.

---

## 8) Hand Contact Force Handling

### Topic format
- topic: `/isaac_contact_states`
- type: `std_msgs/msg/Float32MultiArray`
- payload length: `10`

### Current interpretation
- Isaac provides scalar contact values
- code assumes force acts along the fingertip nail/normal direction
- callback reconstructs a directional force vector from scalar magnitude

### Processing flow
1. scalar contact -> temporary local sensor force
2. sensor -> tip frame
3. tip -> hand-base frame
4. empirical output-axis correction
5. saved to:
   - `f_l_hand_c_`
   - `f_r_hand_c_`

### Important rule
`HandContactForceCallback()` updates **current** force buffers only.  
It must **not** clear hand target-force monitor buffers.

---

## 9) Current Known Status / Limitations (v22)

### Stable / working
- forcecon command path works
- hand admittance control operates
- current / target hand-force monitor topics available
- simplified YAML config path is working
- delta arm command now behaves deterministically relative to initial pose

### Still under tuning
- steady-state force tracking error may still remain
- force-control gains may still need tuning per finger
- hand IK may fail for large or poorly conditioned fingertip targets
- force tracking depends heavily on:
  - contact direction assumption
  - scalar-to-axis interpretation
  - hand-base conversion quality

---

## 10) Quick Checklist (v22)

### Build / run
```bash
cd ~/dualarm_ws
colcon build --packages-select dualarm_forcecon
source install/setup.bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

### Confirm active mode
Check monitor header:
- `Mode: [idle]`
- `Mode: [forward]`
- `Mode: [inverse]`
- `Mode: [forcecon]`

### Main topics
- `/forward_arm_joint_targets`
- `/forward_hand_joint_targets`
- `/target_arm_cartesian_pose`
- `/delta_arm_cartesian_pose`
- `/target_hand_fingertips`
- `/delta_hand_fingertips`
- `/target_hand_force`
- `/isaac_contact_states`
- `/isaac_joint_states`
- `/isaac_joint_command`
- `/hand_force_current_monitor`
- `/hand_force_target_monitor`

### Common sanity checks
- If delta arm command does not move:
  - verify mode is `inverse`
  - verify base pose was latched after startup
- If force target is not visible:
  - verify mode is `forcecon`
  - verify `/target_hand_force` callback is triggered
- If IK fails:
  - reduce displacement
  - reduce force target
  - use smaller incremental fingertip targets
- If force tracking looks unstable:
  - reduce force magnitude
  - ensure contact is stable
  - tune damping / mass / step clamps conservatively

---

## 11) Suggested Next Steps (v22+)

- improve force-control stability
  - reduce steady-state error
  - refine damping / mass / clamp tuning
- verify scalar-contact direction model more carefully
- consider per-finger gain tuning
- consider more explicit contact/no-contact approach policy
- add compact monitor mapping documentation for `/hand_force_current_monitor` index layout
- update README again if:
  - topic payload layout changes
  - delta-arm semantics change again
  - forcecon controller interface changes

---

## 12) Notes

This README reflects the **v22 baseline** after:
- v20 successful hand admittance force control
- v21 hand-admittance config cleanup
- v22 latched-base delta arm command behavior

If you later change:
- delta-arm reference policy
- force monitor packing
- hand contact mapping
- forcecon gains/interface

update this README together to keep future handoff clean.
