# DualArmForceControl README (v25)

This README summarizes the current **v25** baseline of `dualarm_forcecon` (dual arm + hand monitor/control node), following the same overall structure/style as the **v24 README** and updated to reflect the latest **hand motion + force integrated control flow**. The v24 README was used as the formatting/reference baseline for this document.

---

## 1) Version Summary (v25)

v25 builds on the following milestones:

- **v22**: arm delta Cartesian command changed to use a **latched initial base pose**
- **v23**: arm/hand control modes separated and `/change_control_mode` changed to custom service `dualarm_forcecon_interfaces/srv/SetControlMode`
- **v24**: separate hand `forcecon` mode removed, and hand force control moved into the internal logic of hand `forward` / `inverse`
- **v25**: hand control refined into a clearer **motion-reference + admittance-correction** structure, so that **hand forward and hand inverse both share the same selected-finger force-control pipeline**

### v25 major changes

- **Hand control now has an explicit two-stage structure**
  - **motion reference**
  - **final command**
- New internal hand buffers:
  - `q_l_h_motion_t_`
  - `q_r_h_motion_t_`
- Final published hand command remains:
  - `q_l_h_t_`
  - `q_r_h_t_`

This means:

- the node first defines a **motion target**
- then optionally applies **admittance-based correction** to one active finger
- then publishes the corrected command to `/isaac_joint_command`

### Hand motion path in v25

#### Hand forward mode

```text
/forward_hand_joint_targets
    -> q_h_motion_t_
    -> (optional selected-finger admittance correction using /target_hand_force + contact)
    -> q_h_t_
    -> /isaac_joint_command
```

#### Hand inverse mode

```text
/target_hand_fingertips or /delta_hand_fingertips
    -> Hand IK
    -> q_h_motion_t_
    -> (optional selected-finger admittance correction using /target_hand_force + contact)
    -> q_h_t_
    -> /isaac_joint_command
```

### Desired-force handling in v25

`/target_hand_force` is now treated as:

- **desired force reference for one selected finger**
- usable while hand mode is:
  - `forward`
  - `inverse`

There is still **no separate hand `forcecon` mode**.

### Practical v25 behavior

In practice, v25 works best with the following interpretation:

- **motion target** defines where the finger/hand should stay or move
- **desired force** defines what contact force should be regulated for the selected finger
- **selected finger only** gets admittance-based correction
- **all non-selected fingers** keep following the motion target

### Important operational note in v25

For reliable hand force-control experiments in **hand forward mode**, it is important to give **both**:

1. a valid hand motion target (often the current hand joint state)
2. a desired force command on `/target_hand_force`

If only desired force is sent without a stable motion reference, the behavior can be less predictable.

---

## 2) Package Structure (v25, must preserve)

> Keep the package tree and file-role separation rules unchanged.

```text
dualarm_ws/
└── src/
    ├── dualarm_forcecon_interfaces/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── srv/
    │       └── SetControlMode.srv
    │
    └── dualarm_forcecon/
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
            ├── states_current_callback_dualarm.cpp     # current-state callbacks + monitor print + service callback
            └── states_target_callback_dualarm.cpp      # target/command callbacks
```

### File-role separation rules (must keep)

- Do **not** change the package tree
- Do **not** move callback logic back into `DualArmForceControl.cpp`
- `DualArmForceControl.cpp` must keep only:
  - constructor
  - destructor
  - `ControlLoop()`

### Preserve these invariants

- 52-DOF publish mapping
- Isaac UI-matching Euler convention
- world-base z offset default behavior (`0.306 m`)
- current `PrintDualArmStates` layout / colors / ordering
- include/src separation

---

## 3) Core Functional Overview

## 3.1 Control modes

### Arm modes

- `idle`
- `forward`
- `inverse`

### Hand modes

- `idle`
- `forward`
- `inverse`

There is **no** hand `forcecon` mode in v25.

---

## 3.2 Arm control summary

### Arm idle

- target syncs to current arm state

### Arm forward

```text
/forward_arm_joint_targets
    -> TargetArmJointsCallback()
    -> q_l_t_, q_r_t_
    -> /isaac_joint_command
```

### Arm inverse

```text
/target_arm_cartesian_pose
or
/delta_arm_cartesian_pose
    -> TargetArmPositionCallback() / DeltaArmPositionCallback()
    -> Arm IK
    -> q_l_t_, q_r_t_
    -> /isaac_joint_command
```

### Arm delta behavior (preserved from v22)

Delta arm Cartesian command is interpreted relative to the **latched initial base pose**, not the continuously updated current pose.

---

## 3.3 Hand control summary

This is the most important part of v25.

### Hand idle

- hand motion target syncs to current hand joints
- final hand command syncs to current hand joints
- desired hand force monitor is cleared

### Hand forward

```text
/forward_hand_joint_targets
    -> TargetHandJointsCallback()
    -> q_h_motion_t_
    -> ControlLoop()
    -> q_h_t_
    -> /isaac_joint_command
```

If a valid `/target_hand_force` exists:

- selected finger gets admittance-based correction
- other fingers stay on `q_h_motion_t_`

### Hand inverse

```text
/target_hand_fingertips or /delta_hand_fingertips
    -> TargetHandPositionCallback() / DeltaHandPositionCallback()
    -> Hand IK
    -> q_h_motion_t_
    -> ControlLoop()
    -> q_h_t_
    -> /isaac_joint_command
```

If a valid `/target_hand_force` exists:

- selected finger gets admittance-based correction
- other fingers stay on the motion reference built from inverse IK

---

## 3.4 Hand force-control structure in v25

### Current force

Current fingertip contact force comes from:

```text
/isaac_contact_states
    -> HandContactForceCallback()
    -> scalar contact values
    -> sensor-frame reconstruction
    -> tip/base rotation transform
    -> wrist(hand-base) mapped force
    -> f_l_hand_c_ / f_r_hand_c_
```

### Desired force

Desired fingertip force comes from:

```text
/target_hand_force
    -> TargetHandForceCallback()
    -> active hand / active finger / desired force latch
    -> f_l_hand_t_ / f_r_hand_t_
    -> ControlLoop()
```

### Hand forward with force correction

```text
q_h_motion_t_ --FK--> x_target
x_target + F_target + F_current
    -> HandAdmittanceControl::step()
    -> corrected x_cmd
    -> Hand IK
    -> selected finger joints replaced in q_h_t_
```

### Hand inverse with force correction

```text
target fingertip position
    -> Hand IK
    -> q_h_motion_t_

selected finger:
x_target + F_target + F_current
    -> HandAdmittanceControl::step()
    -> corrected x_cmd
    -> Hand IK
    -> selected finger joints replaced in q_h_t_
```

### Selected-finger-only update

In v25, the active force-control branch updates only the **selected finger joints**:

- thumb / index / middle / ring / baby (one at a time)
- all remaining fingers continue to follow the motion reference

This is a key design feature of v25.

---

## 4) Important Internal State Variables (v25)

### Arm

- `q_l_c_`, `q_r_c_` : current arm joints
- `q_l_t_`, `q_r_t_` : final arm command

### Hand

- `q_l_h_c_`, `q_r_h_c_` : current hand joints
- `q_l_h_motion_t_`, `q_r_h_motion_t_` : hand motion reference
- `q_l_h_t_`, `q_r_h_t_` : final hand command actually published

### Hand force

- `f_l_hand_c_`, `f_r_hand_c_` : current hand contact force (canonical finger row order)
- `f_l_hand_t_`, `f_r_hand_t_` : desired hand force monitor

### Active desired-force latch

- `hand_force_cmd_valid_`
- `hand_force_cmd_hand_id_`
- `hand_force_cmd_finger_id_`
- `hand_force_cmd_f_des_base_`
- `hand_force_cmd_stamp_ns_`

---

## 5) HandAdmittanceControl logic (v25 baseline)

`hand_admittance_control.hpp` remains the core selected-finger contact controller.

The controller operates per active finger and uses:

- desired fingertip position in hand-base frame
- desired force in hand-base frame
- measured force in hand-base frame
- current hand joint state
- timestep `dt`

### High-level internal stages

1. **Current fingertip pose from FK**
2. **Measured force preprocessing**
   - optional LPF
3. **Hybrid axis selection**
   - typically force axis = `z` in hand-base frame
4. **Contact detection / hysteresis**
5. **Tangent anchor / slip guard**
6. **Admittance state update**
   - offset
   - velocity
7. **Corrected fingertip command**
8. **Hand IK**
9. **Selected finger command output**

### YAML-driven behavior

The controller still uses the YAML config in `forcecon_cfg.yaml`, including:

- `mass`
- `damping`
- `stiffness`
- `force_ctrl_enable`
- `hybrid_force_axis`
- contact thresholds / hysteresis
- slip detection
- anti-windup
- IK parameters

So v25 keeps the existing config path rather than introducing a new configuration system.

---

## 6) ROS Interfaces (v25)

## 6.1 Subscriptions

### State input

**`/isaac_joint_states`** — `sensor_msgs/msg/JointState`

Used by:

- `JointsCallback()`
- `PositionCallback()`

### Contact input

**`/isaac_contact_states`** — `std_msgs/msg/Float32MultiArray`

- expected length = `10`
- `5 left + 5 right`

### Arm forward input

**`/forward_arm_joint_targets`** — `std_msgs/msg/Float64MultiArray`

- length = `12`
- `[left arm 6, right arm 6]`

### Arm inverse inputs

**`/target_arm_cartesian_pose`** — `std_msgs/msg/Float64MultiArray`

- length = `12`
- `[L x y z r p y, R x y z r p y]`

**`/delta_arm_cartesian_pose`** — `std_msgs/msg/Float64MultiArray`

- length = `12`
- `[L dx dy dz droll dpitch dyaw, R dx dy dz droll dpitch dyaw]`

### Hand forward input

**`/forward_hand_joint_targets`** — `std_msgs/msg/Float64MultiArray`

Supported lengths:

- `30` = `left15 + right15`
- `40` = `left20 + right20`

Legacy-compatible lengths handled in callback logic:

- `42`
- `52`

### Hand inverse inputs

**`/target_hand_fingertips`** — `std_msgs/msg/Float64MultiArray`

- length = `30`
- per hand order:
  - `thumb xyz`
  - `index xyz`
  - `middle xyz`
  - `ring xyz`
  - `baby xyz`

**`/delta_hand_fingertips`** — `std_msgs/msg/Float64MultiArray`

- length = `5`
- format:
  - `[side, finger, dx, dy, dz]`

where:

- `side`:
  - `0 = left`
  - `1 = right`
- `finger`:
  - `0 = thumb`
  - `1 = index`
  - `2 = middle`
  - `3 = ring`
  - `4 = baby`

### Hand desired-force input

**`/target_hand_force`** — `std_msgs/msg/Float64MultiArray`

Supported formats:

#### Compact format

```text
[hand_id, finger_id, fx, fy, fz]
```

#### Legacy-compatible format

```text
[hand_id, finger_id, px, py, pz, fx, fy, fz]
```

In current v25 usage, the desired force part is the important part. The compact format is recommended.

---

## 6.2 Publishers

### Main command output

**`/isaac_joint_command`** — `sensor_msgs/msg/JointState`

- final combined arm + hand command

### Hand force monitor topics

**`/hand_force_current_monitor`** — `std_msgs/msg/Float32MultiArray`

**`/hand_force_target_monitor`** — `std_msgs/msg/Float32MultiArray`

These are useful for:

- `rqt_plot`
- debugging steady-state force error
- checking whether target/current force are aligned

---

## 6.3 Service

**`/change_control_mode`** — `dualarm_forcecon_interfaces/srv/SetControlMode`

### Request

```text
string arm_mode
string hand_mode
```

### Response

```text
bool success
string message
```

---

## 7) Control Mode Change Commands (v25)

This remains one of the most important operational interfaces.

## 7.1 Check service type

```bash
ros2 service type /change_control_mode
```

Expected:

```text
dualarm_forcecon_interfaces/srv/SetControlMode
```

## 7.2 Valid mode values

### Arm

- `idle`
- `forward`
- `inverse`

### Hand

- `idle`
- `forward`
- `inverse`

There is **no** `hand_mode: forcecon`.

## 7.3 Example mode change commands

### A. Arm forward / Hand forward

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'forward', hand_mode: 'forward'}"
```

### B. Arm inverse / Hand inverse

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'inverse', hand_mode: 'inverse'}"
```

### C. Arm inverse / Hand forward

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'inverse', hand_mode: 'forward'}"
```

### D. Arm forward / Hand inverse

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'forward', hand_mode: 'inverse'}"
```

### E. Full idle

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'idle', hand_mode: 'idle'}"
```

---

## 8) Coordinate / Frame Conventions

## 8.1 Arm pose display

Arm pose monitor follows the project’s Isaac UI-matching Euler convention.

Default world-base transform:

- translation = `[0.0, 0.0, 0.306]`
- rotation = `[0.0, 0.0, 0.0] deg`

## 8.2 Hand fingertip position frame

Hand fingertip positions are expressed in:

- `LEFT_HAND_BASE`
- `RIGHT_HAND_BASE`

## 8.3 Hand contact ordering

Observed Isaac message order per hand:

```text
[BABY, RING, MIDL, INDX, THMB]
```

Internal canonical row order:

```text
THMB(0), INDX(1), MIDL(2), RING(3), BABY(4)
```

## 8.4 Monitor print order

The monitor prints fingers in this order:

```text
BABY -> RING -> MIDL -> INDX -> THMB
```

---

## 9) Build / Run (v25)

## 9.1 Build

```bash
cd ~/dualarm_ws
colcon build --packages-select dualarm_forcecon_interfaces dualarm_forcecon
source install/setup.bash
```

## 9.2 Run node

```bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

---

## 10) Example Commands by Mode

## 10.1 Arm forward / Hand forward

### Change mode

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'forward', hand_mode: 'forward'}"
```

### Arm joint target

```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray "{data: [1.6419, -0.0058, -2.2344, 1.5607, 1.5936, -0.1104, -0.0459, 0.7361, 1.9809, 0.0, -1.1869, -0.7854]}"
```

### Hand joint target

Example 30-value command (`left15 + right15`):

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
0.0,0.3948,0.3927, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,-0.0400,0.7854, 0.0,0.0,0.0,
0.0,0.3955,0.3840, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0
]}"
```

### Hand desired force

Example: left hand, ring finger, desired force = `(0, 0, 5)`

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.0, 0.0, 5.0]}"
```

### Important v25 practical recipe for forward-mode force control

In hand `forward`, the most reliable test is:

1. send a hand motion target first
2. then send desired force

For example, hold the hand at the current posture and then regulate ring force.

---

## 10.2 Arm inverse / Hand inverse

### Change mode

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'inverse', hand_mode: 'inverse'}"
```

### Arm Cartesian target

```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
0.1001, 0.2666, -0.1263, 2.7370, 1.5062, -1.1601,
0.5439, -0.3170, 0.1478, -3.0216, 1.5565, -1.6910
]}"
```

### Arm delta Cartesian target

```bash
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
0.020, -0.010, 0.015, 0.00, 0.00, 0.00,
-0.020, 0.010, 0.015, 0.00, 0.00, 0.00
]}"
```

### Hand fingertip target

```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
0.1524,-0.0471,-0.1000, 0.2465,-0.0403,-0.0144, 0.2640,-0.0135,-0.0144, 0.2382,0.0133,-0.0337, 0.2310,0.0401,-0.0144,
0.1522,0.0472,-0.1003, 0.2465,0.0403,-0.0144, 0.2640,0.0135,-0.0144, 0.2465,-0.0133,-0.0144, 0.2310,-0.0401,-0.0144
]}"
```

### Hand delta fingertip target

```bash
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.005, 0.000, -0.002]}"
```

### Hand desired force during inverse mode

Example: left ring finger desired force `(0, 0, 5)`

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.0, 0.0, 5.0]}"
```

---

## 10.3 Arm inverse / Hand forward

This is one of the most useful combined operation modes.

### Change mode

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'inverse', hand_mode: 'forward'}"
```

### Arm Cartesian target

```bash
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
0.1001, 0.2666, -0.1263, 2.7370, 1.5062, -1.1601,
0.5439, -0.3170, 0.1478, -3.0216, 1.5565, -1.6910
]}"
```

### Hand joint target

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
0.0,0.3948,0.3927, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,-0.0400,0.7854, 0.0,0.0,0.0,
0.0,0.3955,0.3840, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0
]}"
```

### Hand desired force

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.0, 0.0, 5.0]}"
```

---

## 10.4 Arm forward / Hand inverse

### Change mode

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'forward', hand_mode: 'inverse'}"
```

### Arm joint target

```bash
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray "{data: [1.6419, -0.0058, -2.2344, 1.5607, 1.5936, -0.1104, -0.0459, 0.7361, 1.9809, 0.0, -1.1869, -0.7854]}"
```

### Hand fingertip target

```bash
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
0.1524,-0.0471,-0.1000, 0.2465,-0.0403,-0.0144, 0.2640,-0.0135,-0.0144, 0.2382,0.0133,-0.0337, 0.2310,0.0401,-0.0144,
0.1522,0.0472,-0.1003, 0.2465,0.0403,-0.0144, 0.2640,0.0135,-0.0144, 0.2465,-0.0133,-0.0144, 0.2310,-0.0401,-0.0144
]}"
```

### Hand desired force

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [1, 1, 0.0, 0.0, 3.0]}"
```

---

## 10.5 Full idle

### Change mode

```bash
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'idle', hand_mode: 'idle'}"
```

Behavior:

- arm target syncs to current arm state
- hand motion target syncs to current hand state
- final hand command syncs to current hand state
- desired hand-force monitor is cleared

---

## 10.6 Forward hand force-control example using the captured current hand state

This example matches the working v25 test pattern:

- keep the hand at the current posture
- apply desired force only to the left ring finger

### Step 1: keep current hand posture as motion target

```bash
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
  0.0044, 0.3995, 0.3914,
  0.0,   -0.0002, 0.0,
  0.0,   -0.0002, 0.0,
  0.0,   -0.0001, 0.7854,
  0.0,   -0.0003, 0.0,

 -0.0013, 0.4023, 0.3696,
  0.0,   -0.0001, 0.0,
  0.0,   -0.0001, 0.0,
  0.0,   -0.0001, 0.0,
  0.0,   -0.0001, 0.0
]}"
```

### Step 2: command left ring force

```bash
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.0, 0.0, 5.0]}"
```

Where:

- `0` = left hand
- `3` = ring finger
- `(0.0, 0.0, 5.0)` = desired force in hand-base frame

---

## 11) YAML configuration notes (v25)

Current controller behavior is still strongly affected by:

```text
yaml/forcecon_cfg.yaml
```

Especially:

- `mass`
- `damping`
- `stiffness`
- `force_error_axis_sign`
- `hybrid_force_axis`
- `contact_on_threshold_N`
- `contact_off_threshold_N`
- `max_offset_m`
- `max_step_m`
- `max_adm_velocity_mps`
- IK parameters

### Practical note from current v25 testing

Increasing `stiffness` changes the restoring / contact behavior noticeably.
For example, setting:

```yaml
stiffness: [300.0, 300.0, 300.0]
```

can materially change the selected finger response.

This means v25 tuning is now closely tied to both:

- motion reference quality
- YAML MDK/contact tuning

---

## 12) Current limitations / observed behavior in v25

At the current v25 baseline:

- hand force control is functioning
- target force and current force can still show **steady-state error**
- tuning is still important
- correct motion target initialization is important, especially in hand `forward`

So v25 should be considered:

- a working and more structured baseline
- but still a tuning-sensitive research/control baseline rather than a fully finalized controller

---

## 13) Current Baseline Conclusion (v25)

The essential meaning of v25 is:

- arm control structure remains stable
- hand control is now clearly split into:
  - **motion reference**
  - **optional selected-finger force-aware correction**
- hand `forward` and `inverse` now share the same general force-control architecture
- `/target_hand_force` is used as a desired-force input while hand mode stays `forward` or `inverse`
- for practical force-control experiments in forward mode, a motion reference should be published first

In short:

```text
arm: same split-mode structure as v23/v24
hand: motion target first, selected-finger admittance correction second
```

That is the current **v25** baseline.
