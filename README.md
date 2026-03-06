# DualArmForceControl README (v20)

This README summarizes the current **v20** baseline of `dualarm_forcecon`.

v20 preserves the **v18 package structure / invariants** and the **v19 forcecon architecture**, and marks the first version where **hand admittance-based force control is confirmed to work in simulation**.

---

## 1) Version Summary (v20)

### 1.1 One-line summary
- `dualarm_forcecon` is a ROS 2 Humble package for controlling a **dual-arm (6DoF x2) + dual-hand (20DoF canonical x2)** system in Isaac Sim, supporting:
  - `idle`
  - `forward`
  - `inverse`
  - `forcecon`
- **v20 baseline status:** hand **admittance control succeeded** in `forcecon` mode.

### 1.2 What v20 means
v20 is the version after the v19 forcecon stabilization work where:
- admittance controller configuration was tuned enough to make force control actually respond,
- target-force tracking became observable in runtime,
- and dedicated **monitor topics for hand current/target force** were added so the behavior can be visualized in `rqt_plot`.

### 1.3 Practical status of v20
- **Builds successfully**
- **Forcecon runs**
- **Hand current force and target force can both be monitored as ROS topics**
- **Admittance force control works in simulation**
- Further work is still needed for:
  - stability improvement,
  - reduced steady-state error,
  - more robust contact/no-contact transition handling,
  - better force-axis calibration and control quality

---

## 2) Version History / Evolution

## 2.1 v18 baseline recap
v18 established the major baseline before the successful v20 forcecon milestone.

### v18 major points
- Callback source file split finalized:
  - `states_current_callback_dualarm.cpp`
  - `states_target_callback_dualarm.cpp`
- Mode cycle finalized:
  - `idle -> forward -> inverse -> forcecon -> idle`
- `TAR_F` zeroing bug fixed
  - `HandContactForceCallback()` no longer clears target-force buffers
- `DeltaHandPositionCallback()` added
- Hand FK axis convention root fix applied in `hand_forward_kinematics.hpp`
- Forcecon command path existed, but force tracking quality was still incomplete

## 2.2 v19 recap
v19 was the **forcecon stabilization stage**.

### v19 key architectural changes
- YAML-based hand admittance configuration loading added
- Per-finger `HandAdmittanceControl` instances created from config
- `TargetHandForceCallback()` changed to **latch-only command storage**
- Real execution moved into `ControlLoop()` at 100 Hz
- **Forcecon hold snapshot** added:
  - arm targets fixed at forcecon entry
  - hand targets fixed at forcecon entry
  - only the commanded finger is updated during forcecon

### v19 main problem
- Admittance structure existed, but force tracking was not yet reliable
- In particular, force control often failed because of the combination of:
  - contact / non-contact dependent logic,
  - force-direction assumptions from scalar contact input,
  - tuning sensitivity

## 2.3 v20 milestone
v20 is the first version where the **hand admittance controller is confirmed to operate successfully**.

### v20 major changes
- Admittance parameter set refined so forcecon becomes responsive in simulation
- Runtime behavior confirmed: target-force command can drive fingertip force behavior
- New monitor topics added for continuous force visualization:
  - `/hand_force_current_monitor`
  - `/hand_force_target_monitor`
- `rqt_plot`-based comparison of current force vs target force is now possible

---

## 3) Package Structure (must preserve)

> The package tree and file-role separation rules must remain unchanged.

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
    ├── DualArmForceControl.cpp
    ├── DualArmForceControl.h
    ├── node_dualarm_main.cpp
    ├── states_current_callback_dualarm.cpp
    └── states_target_callback_dualarm.cpp
```

### Strict file-role rules
- `DualArmForceControl.cpp` must contain only:
  - constructor
  - destructor
  - `ControlLoop()`
- Current-state callbacks / monitor printing stay in:
  - `states_current_callback_dualarm.cpp`
- Target / command callbacks stay in:
  - `states_target_callback_dualarm.cpp`
- Preserve include/src separation
- Do not collapse the callback split back into a single file

---

## 4) Invariants (do not break)

The following invariants must be preserved across future patches.

### 4.1 Mechanical / mapping invariants
- **52-DOF publish mapping** must remain correct
  - arms: 12
  - hands: 40 canonical
- Hand joint layout remains canonical 20DoF representation:
  - `[thumb1..4, index1..4, middle1..4, ring1..4, baby1..4]`
- Mimic rule stays:
  - `joint4 = joint3`

### 4.2 Frame / display invariants
- Isaac UI matching Euler convention must remain unchanged
- World-base z offset default behavior remains:
  - `world_base_xyz = [0.0, 0.0, 0.306]`
- Hand fingertip display remains in each hand base frame:
  - `LEFT_HAND_BASE`
  - `RIGHT_HAND_BASE`

### 4.3 Contact ordering invariants
For `/isaac_contact_states`:
- observed msg order per hand:
  - `[BABY, RING, MIDL, INDX, THMB]`
- internal canonical row order:
  - `[THMB, INDX, MIDL, RING, BABY]`
- mapping:
  - `BABY -> 4`
  - `RING -> 3`
  - `MIDL -> 2`
  - `INDX -> 1`
  - `THMB -> 0`

### 4.4 Monitor invariants
- `PrintDualArmStates` format / colors should remain stable
- Hand monitor print order remains:
  - `BABY -> RING -> MIDL -> INDX -> THMB`

---

## 5) Control Modes

### 5.1 Mode cycle
`idle -> forward -> inverse -> forcecon -> idle`

### 5.2 Mode behavior
#### idle
- one-shot sync of targets to current states
- safe hold baseline

#### forward
- joint-space command mode
- accepts arm and hand joint targets

#### inverse
- Cartesian command mode
- accepts:
  - arm absolute pose
  - arm delta pose
  - hand absolute fingertip targets
  - hand delta fingertip jogging

#### forcecon
- fingertip force-oriented control mode
- one active commanded finger at a time
- uses:
  - scalar contact input from Isaac Sim
  - reconstructed fingertip/base-frame force estimate
  - hand admittance controller
  - hand IK
- arm and non-commanded hand joints are held from forcecon entry snapshot

---

## 6) Core Kinematics / Control Components

## 6.1 Arm FK / IK
### `arm_forward_kinematics.hpp`
- KDL-based FK
- supports BASE / WORLD output modes
- includes Isaac UI-matching Euler conversion helper

### `arm_inverse_kinematics.hpp`
- KDL-based IK
- supports frame selection and Euler convention selection
- strict frame handling preserved

## 6.2 Hand FK / IK
### `hand_forward_kinematics.hpp`
- Pinocchio-based hand FK
- supports 15DoF and canonical 20DoF input
- applies hand-base axis remap for display / user frame consistency
- provides:
  - fingertip positions
  - fingertip rotations in hand base frame

### `hand_inverse_kinematics.hpp`
- finger-wise position IK
- internally solves in 15DoF independent space
- expands back to canonical 20DoF with mimic rule

## 6.3 Hand admittance controller
### `hand_admittance_control.hpp`
- hybrid force/position structure
- supports:
  - per-axis MDK dynamics
  - force LPF
  - contact gate / hysteresis
  - target-force ramp
  - tangent anchor hold
  - slip detection / guard
  - offset / velocity / step clamps
  - IK fallback handling

### v20 status
- Parameter tuning reached a regime where **admittance force control works** in simulation
- This is the most important functional milestone of v20

---

## 7) ROS Interfaces (v20)

## 7.1 Subscriptions
### State input
- `/isaac_joint_states` — `sensor_msgs/msg/JointState`

### Contact input
- `/isaac_contact_states` — `std_msgs/msg/Float32MultiArray`
  - length: 10
  - 5 left + 5 right
  - scalar contact values only

### Forward-mode input
- `/forward_arm_joint_targets` — `std_msgs/msg/Float64MultiArray`
- `/forward_hand_joint_targets` — `std_msgs/msg/Float64MultiArray`

### Inverse-mode input
- `/target_arm_cartesian_pose` — `std_msgs/msg/Float64MultiArray`
- `/delta_arm_cartesian_pose` — `std_msgs/msg/Float64MultiArray`
- `/target_hand_fingertips` — `std_msgs/msg/Float64MultiArray`
- `/delta_hand_fingertips` — `std_msgs/msg/Float64MultiArray`

### Forcecon-mode input
- `/target_hand_force` — `std_msgs/msg/Float64MultiArray`
  - format:
    - `[hand_id, finger_id, p_des_x, p_des_y, p_des_z, f_des_x, f_des_y, f_des_z]`
  - `hand_id`: `0=left`, `1=right`
  - `finger_id`: `0=thumb, 1=index, 2=middle, 3=ring, 4=baby`

## 7.2 Publishers
### Main command output
- `/isaac_joint_command` — `sensor_msgs/msg/JointState`

### New v20 monitor outputs
- `/hand_force_current_monitor` — `std_msgs/msg/Float32MultiArray`
- `/hand_force_target_monitor` — `std_msgs/msg/Float32MultiArray`

These topics were added so that **current force and target force can be plotted continuously** in `rqt_plot`.

## 7.3 Service
- `/change_control_mode` — `std_srvs/srv/Trigger`

---

## 8) Hand Force Handling

## 8.1 Isaac contact limitation
Isaac Sim contact sensor currently provides only a **scalar** value per fingertip contact point.

That means the package cannot directly receive a full 3D force vector from the simulator. Instead, the code reconstructs a force estimate under a **1-axis normal-force assumption**.

## 8.2 Modeling assumption
The implemented assumption is:
- the fingertip force acts mainly along the finger's intended pressing direction,
- and the scalar contact magnitude represents that normal component.

Then the code transforms this scalar-derived force through:
1. sensor-frame assumption,
2. tip-frame conversion,
3. hand-base / wrist-frame conversion,
4. empirical output-axis calibration.

## 8.3 Force matrices in runtime
- current hand force:
  - `f_l_hand_c_`
  - `f_r_hand_c_`
- target hand force:
  - `f_l_hand_t_`
  - `f_r_hand_t_`

Rows use canonical order:
- `THMB=0`
- `INDX=1`
- `MIDL=2`
- `RING=3`
- `BABY=4`

---

## 9) Forcecon Architecture (v20)

## 9.1 High-level flow
1. User sends `/target_hand_force`
2. `TargetHandForceCallback()` latches command only
3. `ControlLoop()` executes forcecon at 100 Hz
4. Entry snapshot holds:
   - left/right arm targets
   - left/right hand targets
5. Only the selected finger gets updated by admittance + IK
6. Final canonical hand joint command is published through `/isaac_joint_command`

## 9.2 Hold snapshot policy
When entering `forcecon`:
- arm targets are frozen from current arm state
- hand targets are frozen from current hand state
- this prevents arm/wrist drift during fingertip force control

This behavior must be preserved.

## 9.3 Why v19 struggled and v20 succeeded
v19 already had the architecture, but force tracking quality depended heavily on:
- contact / non-contact policy,
- scalar-force interpretation,
- admittance tuning,
- IK sensitivity.

v20 is the version where the configuration reached a working regime and force control became effective in practice.

---

## 10) Force Monitor Topics (new in v20)

## 10.1 Purpose
`/target_hand_force` is a command input topic, but for plotting we want continuous monitor topics.

So v20 adds:
- `/hand_force_current_monitor`
- `/hand_force_target_monitor`

These are published continuously from the control side, so they can be visualized as time series.

## 10.2 Message layout
Both topics use:
- `std_msgs/msg/Float32MultiArray`
- length = 30

Flattened layout:
```text
left:
  thumb(x,y,z), index(x,y,z), middle(x,y,z), ring(x,y,z), baby(x,y,z)
right:
  thumb(x,y,z), index(x,y,z), middle(x,y,z), ring(x,y,z), baby(x,y,z)
```

## 10.3 Index rule
```text
idx = hand_offset + finger_id*3 + axis_id
hand_offset = 0  (left)
hand_offset = 15 (right)

finger_id: thumb=0, index=1, middle=2, ring=3, baby=4
axis_id: x=0, y=1, z=2
```

### Examples
- left ring z = `0 + 3*3 + 2 = 11`
- left baby z = `0 + 4*3 + 2 = 14`
- right index z = `15 + 1*3 + 2 = 20`

## 10.4 Example rqt_plot usage
To compare **left ring z current vs target**:
- `/hand_force_current_monitor/data[11]`
- `/hand_force_target_monitor/data[11]`

---

## 11) Build / Run

## 11.1 Build
```bash
cd ~/dualarm_ws
colcon build --packages-select dualarm_forcecon
source install/setup.bash
```

## 11.2 Run
```bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

---

## 12) Configuration

## 12.1 YAML config
- file:
  - `yaml/forcecon_cfg.yaml`
- loaded in constructor through yaml-cpp

## 12.2 Current v20 interpretation
The working v20 setup came from **admittance parameter tuning** rather than a major structural rewrite.

The key meaning of v20 is:
- with the present configuration, forcecon is able to produce observable force-control behavior.

Future tuning will still be needed.

---

## 13) Recommended Debug / Validation Procedure

### 13.1 Before forcecon
- verify monitor pose and fingertip positions
- verify correct finger index / side command
- verify `TAR_F` updates when `/target_hand_force` is sent

### 13.2 During forcecon
- use `PrintDualArmStates()` to check:
  - selected finger target position
  - selected finger current force
  - selected finger target force
- use `rqt_plot` to compare:
  - `/hand_force_current_monitor/...`
  - `/hand_force_target_monitor/...`

### 13.3 If forcecon becomes unstable
Check in this order:
1. forcecon mode actually active
2. correct finger / hand commanded
3. contact scalar present in `/isaac_contact_states`
4. YAML parameter values
5. force direction / sign assumption
6. IK saturation / step clamp limits

---

## 14) Known Limitations in v20

Even though v20 successfully achieves admittance force control, the following limitations remain.

### 14.1 Scalar contact limitation
- Isaac provides only scalar contact value per fingertip
- full 3D contact wrench is not available
- force-direction reconstruction is model-based / assumed

### 14.2 Stability margin still limited
- control may still be sensitive to:
  - contact onset,
  - parameter changes,
  - commanded force magnitude,
  - IK feasibility

### 14.3 Steady-state force accuracy not yet optimized
- the controller can work,
- but force tracking is not yet fully optimized for low steady-state error or high robustness

---

## 15) Future Work

### Main future work after v20
**Improve control stability and reduce steady-state error.**

This includes:
- reducing steady-state force tracking error,
- improving stability near contact transitions,
- improving robustness against force spikes / slip,
- refining admittance and damping behavior,
- improving long-duration force regulation performance,
- possibly refining contact-axis modeling if simulation limitations allow it

### Additional future directions
- more systematic forcecon tuning workflow
- better non-contact / contact transition policy
- better IK robustness under constrained fingertip motion
- richer debug/monitor topics if needed

---

## 16) Final Summary

### v20 meaning in one sentence
v20 is the first stable milestone where **hand admittance-based force control is confirmed to work**, while preserving the v18/v19 package structure and forcecon architecture.

### What must be remembered going forward
- v20 is the new baseline
- forcecon architecture should be preserved
- hold snapshot behavior should be preserved
- monitor topics for current/target hand force should be preserved
- next work should focus on **stability improvement and steady-state error reduction**

