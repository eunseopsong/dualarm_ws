# DualArmForceControl README (v24)

This README summarizes the current **v24** baseline of `dualarm_forcecon` (dual arm + hand monitor / control node) based on the latest package structure, the split arm/hand mode service interface, the v22 latched-base delta arm behavior, and the new **contact-aware hand control logic without a separate `forcecon` mode**.

---

## 1) Version Summary (v24)

v24 builds on:
- **v20**: hand admittance force control successfully working, target/current hand-force monitor topics added
- **v21**: hand admittance configuration cleanup (unused false-path options removed, YAML simplified)
- **v22**: arm delta Cartesian command changed to use a **latched initial base pose** rather than a continuously updated current pose
- **v23**: arm / hand control modes separated and `/change_control_mode` changed to custom service `dualarm_forcecon_interfaces/srv/SetControlMode`
- **v24**: **removed hand `forcecon` mode**, and changed hand force control into an **internal contact-aware branch inside `forward` / `inverse` hand modes**

### v24 major changes
- **Hand `forcecon` mode removed**
  - previous hand mode set:
    - `idle / forward / inverse / forcecon`
  - current hand mode set:
    - `idle / forward / inverse`

- **Hand force control is no longer a separate control mode**
  - force regulation is now handled **inside hand `forward` / `inverse`**
  - logic is:
    - **no contact** → pure motion control
    - **contact detected** → motion target + admittance-based force-aware correction

- **`/target_hand_force` changed in meaning**
  - it is now a **desired-force reference input**
  - it is **not tied to a dedicated `forcecon` mode**
  - it can be used while hand mode is:
    - `forward`
    - `inverse`

- **New hand command flow**
  - **hand forward mode**
    - no contact:
      \[
      q_{cmd} = q_{target}
      \]
    - contact:
      \[
      q_{target} \xrightarrow{FK} x_{target}, \quad
      (x_{target}, F_{target}, F_{ext}) \xrightarrow{\text{Admittance}} q_{cmd}
      \]

  - **hand inverse mode**
    - no contact:
      \[
      x_{target} \xrightarrow{IK} q_{cmd}
      \]
    - contact:
      \[
      (x_{target}, F_{target}, F_{ext}) \xrightarrow{\text{Admittance}} q_{cmd}
      \]

- **Motion target and final command are separated internally**
  - new internal concept:
    - hand motion reference target
    - final hand joint command
  - this allows contact-aware correction to be added on top of the motion target

### Important behavioral note for v24
There is **no separate hand `forcecon` mode anymore**.  
If hand force control is needed, use:

- `hand=forward` or `hand=inverse`
- publish `/target_hand_force`
- when contact is detected, admittance-based correction is applied internally

So in v24:

\[
\text{mode} \neq \text{force control state}
\]

Instead:

- **mode** = command representation (`forward` or `inverse`)
- **contact state** = internal control branch (`no contact` / `contact`)

---

## 2) Package Structure (v24, must preserve)

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
            
            
            
Important rules (must keep)

Do not change package tree

Do not move callbacks back into DualArmForceControl.cpp

DualArmForceControl.cpp should keep only:

constructor

destructor

ControlLoop()

Preserve:

52-DOF publish mapping

Isaac UI-matching Euler convention

world-base z offset default behavior (0.306 m)

PrintDualArmStates formatting style (colors/layout)

include/src separation

3) Core Functional Overview
3.1 Control modes
Arm modes

idle

forward

inverse

Hand modes

idle

forward

inverse

3.2 What each mode does
Arm idle

safe hold / target sync to current state

Arm forward

direct arm joint-space command mode

input:

/forward_arm_joint_targets

Arm inverse

arm Cartesian target mode

input:

/target_arm_cartesian_pose

/delta_arm_cartesian_pose

Hand idle

safe hold / target sync to current state

Hand forward

hand joint target mode

input:

/forward_hand_joint_targets

behavior:

no contact → direct motion target tracking

contact + valid /target_hand_force → selected finger uses admittance correction internally

Hand inverse

hand fingertip Cartesian target mode

input:

/target_hand_fingertips

/delta_hand_fingertips

behavior:

no contact → fingertip IK motion control

contact + valid /target_hand_force → selected finger uses admittance correction internally

3.3 Monitor output

Arm:

current / target pose

current / target force (arm force monitor still unused, typically zero)

Hand:

current / target fingertip positions in hand-base frame

current / target hand forces

Extra plot topics:

/hand_force_current_monitor

/hand_force_target_monitor

4) v24 Hand Contact-Aware Control Logic
4.1 Hand forward mode

Input:

q_current

q_target

optional F_target

measured F_ext

No contact
𝑞
𝑐
𝑚
𝑑
=
𝑞
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
q
cmd
	​

=q
target
	​

Contact detected

convert motion target to fingertip target:

𝑥
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
=
𝐹
𝐾
(
𝑞
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
)
x
target
	​

=FK(q
target
	​

)

compute force-aware position correction using admittance:

(
𝑥
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
,
𝐹
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
,
𝐹
𝑒
𝑥
𝑡
)
→
𝑥
𝑐
𝑚
𝑑
(x
target
	​

,F
target
	​

,F
ext
	​

)→x
cmd
	​


convert to joint command:

𝑥
𝑐
𝑚
𝑑
→
𝐼
𝐾
𝑞
𝑐
𝑚
𝑑
x
cmd
	​

IK
	​

q
cmd
	​

4.2 Hand inverse mode

Input:

x_current

x_target

optional F_target

measured F_ext

No contact
𝑥
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
→
𝐼
𝐾
𝑞
𝑐
𝑚
𝑑
x
target
	​

IK
	​

q
cmd
	​

Contact detected
(
𝑥
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
,
𝐹
𝑡
𝑎
𝑟
𝑔
𝑒
𝑡
,
𝐹
𝑒
𝑥
𝑡
)
→
Admittance
𝑥
𝑐
𝑚
𝑑
→
𝐼
𝐾
𝑞
𝑐
𝑚
𝑑
(x
target
	​

,F
target
	​

,F
ext
	​

)
Admittance
	​

x
cmd
	​

IK
	​

q
cmd
	​

4.3 Important interpretation

In v24, hand force control is not a standalone mode.
It is an internal branch activated by contact state.

That means:

forward / inverse = motion command type

contact/no-contact = internal execution branch

5) ROS Interfaces (v24)
5.1 Subscriptions
Joint / state input

/isaac_joint_states — sensor_msgs/msg/JointState

used by:

JointsCallback

PositionCallback

Hand contact monitor input

/isaac_contact_states — std_msgs/msg/Float32MultiArray

length = 10 (5 left + 5 right)

scalar fingertip contact values from Isaac Sim

Forward mode inputs

/forward_arm_joint_targets — std_msgs/msg/Float64MultiArray

12 values

[left arm 6, right arm 6]

/forward_hand_joint_targets — std_msgs/msg/Float64MultiArray

30 values = left15 + right15

40 values = left20 + right20

Inverse mode inputs

/target_arm_cartesian_pose — std_msgs/msg/Float64MultiArray

12 values

[L x y z r p y, R x y z r p y]

/delta_arm_cartesian_pose — std_msgs/msg/Float64MultiArray

12 values

[L dx dy dz droll dpitch dyaw, R dx dy dz droll dpitch dyaw]

v22/v24 behavior:

delta is interpreted relative to the latched initial pose, not current pose

/target_hand_fingertips — std_msgs/msg/Float64MultiArray

30 values

per hand order = THMB, INDX, MIDL, RING, BABY (each xyz)

/delta_hand_fingertips — std_msgs/msg/Float64MultiArray

5 values

[side, finger, dx, dy, dz]

side: 0=left, 1=right

finger: 0=thumb, 1=index, 2=middle, 3=ring, 4=baby

Hand desired-force input (v24)

/target_hand_force — std_msgs/msg/Float64MultiArray

Supported formats:

compact v24 format

5 values

[side, finger, f_des_x, f_des_y, f_des_z]

legacy compatible format

8 values

[side, finger, p_des_x, p_des_y, p_des_z, f_des_x, f_des_y, f_des_z]

Important note:

in v24, the legacy p_des_* part is ignored

only the desired force is used

fingertip target position is generated internally from:

current hand motion target

current hand mode (forward or inverse)

5.2 Publishers

/isaac_joint_command — sensor_msgs/msg/JointState

consolidated arm + hand command output

/hand_force_current_monitor — std_msgs/msg/Float32MultiArray

hand current force monitor topic

intended for plotting in rqt_plot

/hand_force_target_monitor — std_msgs/msg/Float32MultiArray

hand target force monitor topic

intended for plotting in rqt_plot

5.3 Service

/change_control_mode — dualarm_forcecon_interfaces/srv/SetControlMode

Service request:

string arm_mode
string hand_mode

Service response:

bool success
string message
6) Control Mode Change Commands (v24)

This is one of the most important changes in v24.

6.1 Service type check
ros2 service type /change_control_mode

Expected:

dualarm_forcecon_interfaces/srv/SetControlMode
6.2 Valid mode values
Arm

idle

forward

inverse

Hand

idle

forward

inverse

6.3 Mode change command examples
A. Arm forward / Hand forward
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'forward', hand_mode: 'forward'}"
B. Arm inverse / Hand inverse
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'inverse', hand_mode: 'inverse'}"
C. Arm inverse / Hand forward
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'inverse', hand_mode: 'forward'}"
D. Arm forward / Hand inverse
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'forward', hand_mode: 'inverse'}"
E. Full idle
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'idle', hand_mode: 'idle'}"
6.4 Important v24 note

The following is no longer valid:

hand_mode: 'forcecon'

That mode was removed in v24.

7) Coordinate / Frame Conventions
7.1 Arm pose display

Arm pose in monitor uses project Isaac UI-matching Euler convention

world-base transform default:

translation = [0.0, 0.0, 0.306]

rotation = [0, 0, 0] deg unless parameterized

7.2 Arm inverse input

controlled by:

ik_targets_frame_

ik_euler_conv_

ik_angle_unit_

7.3 Hand pose display / command frame

Hand fingertip positions are displayed in:

LEFT_HAND_BASE

RIGHT_HAND_BASE

Hand FK axis convention root fix is already integrated

HandPositionCallback() uses corrected hand-base display convention

7.4 Hand fingertip order

monitor print order:

BABY -> RING -> MIDL -> INDX -> THMB

command order for /target_hand_fingertips (per hand):

THMB, INDX, MIDL, RING, BABY

7.5 Hand contact sensor ordering

observed sensor order per hand from Isaac:

[BABY, RING, MIDL, INDX, THMB]

internal canonical storage:

THMB(0), INDX(1), MIDL(2), RING(3), BABY(4)

8) Current Baseline Behavior (v24)
8.1 Arm

arm mode structure from v23 is preserved:

idle / forward / inverse

delta arm command still uses:

latched initial pose

not continuously updated current pose

8.2 Hand

hand mode is now:

idle / forward / inverse

there is no separate forcecon mode

force-aware correction happens only when:

valid /target_hand_force is available

contact is detected internally by admittance controller

8.3 Desired force path

current force:

comes from /isaac_contact_states

converted into hand-base force vectors

desired force:

comes from /target_hand_force

target force monitor remains visible through:

/hand_force_target_monitor

8.4 Motion reference vs final command

Internally, v24 separates:

motion reference target

final joint command

This is the key design that makes contact-aware blending possible.

9) Build / Run (v24)
9.1 Build
cd ~/dualarm_ws
colcon build --packages-select dualarm_forcecon_interfaces dualarm_forcecon
source install/setup.bash
9.2 Run node
ros2 run dualarm_forcecon dualarm_forcecon_node
10) Example Commands by Mode
10.1 Arm forward / Hand forward
Change mode
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'forward', hand_mode: 'forward'}"
Arm joint target
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray "{data: [1.6419, -0.0058, -2.2344, 1.5607, 1.5936, -0.1104, -0.0459, 0.7361, 1.9809, 0.0, -1.1869, -0.7854]}"
Hand joint target (30 values = left15 + right15)
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
0.0,0.3948,0.3927, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,-0.0400,0.7854, 0.0,0.0,0.0,
0.0,0.3955,0.3840, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0
]}"
Hand desired force (compact v24 format)

Example: left hand, ring finger, desired force = (8, 0, 0)

ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [0, 3, 8.0, 0.0, 0.0]}"
Hand desired force (legacy compatible format)
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.0133, -0.0331, 0.2384, 8.0, 0.0, 0.0]}"
10.2 Arm inverse / Hand inverse
Change mode
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'inverse', hand_mode: 'inverse'}"
Arm Cartesian target
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
0.1001, 0.2666, -0.1263, 2.7370, 1.5062, -1.1601,
0.5439, -0.3170, 0.1478, -3.0216, 1.5565, -1.6910
]}"
Arm delta Cartesian target
ros2 topic pub --once /delta_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
0.020, -0.010, 0.015, 0.00, 0.00, 0.00,
-0.020, 0.010, 0.015, 0.00, 0.00, 0.00
]}"
Hand absolute fingertip target
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
0.1524,-0.0471,-0.1000, 0.2465,-0.0403,-0.0144, 0.2640,-0.0135,-0.0144, 0.2340,0.0133,-0.0444, 0.2310,0.0401,-0.0144,
0.1522,0.0472,-0.1003, 0.2465,0.0403,-0.0144, 0.2640,0.0135,-0.0144, 0.2465,-0.0133,-0.0144, 0.2310,-0.0401,-0.0144
]}"
Hand delta fingertip target
ros2 topic pub --once /delta_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [0, 3, 0.005, 0.000, 0.000]}"
Hand desired force during inverse mode

Example: right hand, index finger, desired force = (5, 0, 0)

ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [1, 1, 5.0, 0.0, 0.0]}"
10.3 Arm inverse / Hand forward

This is one of the most important teleoperation-like combinations.

Change mode
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'inverse', hand_mode: 'forward'}"
Arm Cartesian target
ros2 topic pub --once /target_arm_cartesian_pose std_msgs/msg/Float64MultiArray "{data: [
0.1001, 0.2666, -0.1263, 2.7370, 1.5062, -1.1601,
0.5439, -0.3170, 0.1478, -3.0216, 1.5565, -1.6910
]}"
Hand joint target
ros2 topic pub --once /forward_hand_joint_targets std_msgs/msg/Float64MultiArray "{data: [
0.0,0.3948,0.3927, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,-0.0400,0.7854, 0.0,0.0,0.0,
0.0,0.3955,0.3840, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0
]}"
Hand desired force

Example: left hand, ring finger

ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [0, 3, 10.0, 0.0, 0.0]}"

This means:

arm is controlled in Cartesian inverse mode

hand posture comes from forward joint targets

if contact is detected, selected finger gets contact-aware correction internally

10.4 Arm forward / Hand inverse
Change mode
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'forward', hand_mode: 'inverse'}"
Arm joint target
ros2 topic pub --once /forward_arm_joint_targets std_msgs/msg/Float64MultiArray "{data: [1.6419, -0.0058, -2.2344, 1.5607, 1.5936, -0.1104, -0.0459, 0.7361, 1.9809, 0.0, -1.1869, -0.7854]}"
Hand Cartesian fingertip target
ros2 topic pub --once /target_hand_fingertips std_msgs/msg/Float64MultiArray "{data: [
0.1524,-0.0471,-0.1000, 0.2465,-0.0403,-0.0144, 0.2640,-0.0135,-0.0144, 0.2340,0.0133,-0.0444, 0.2310,0.0401,-0.0144,
0.1522,0.0472,-0.1003, 0.2465,0.0403,-0.0144, 0.2640,0.0135,-0.0144, 0.2465,-0.0133,-0.0144, 0.2310,-0.0401,-0.0144
]}"
Hand desired force
ros2 topic pub --once /target_hand_force std_msgs/msg/Float64MultiArray "{data: [1, 0, 6.0, 0.0, 0.0]}"
10.5 Full idle
Change mode
ros2 service call /change_control_mode dualarm_forcecon_interfaces/srv/SetControlMode "{arm_mode: 'idle', hand_mode: 'idle'}"

Behavior:

arm target syncs to current arm state

hand target syncs to current hand state

desired force target is cleared internally

11) Important Notes for v24 Teleoperation Use
11.1 Recommended role split

For future teleoperation-like structure, the natural mapping is:

arm:

Vive tracker

arm_mode = inverse

hand posture:

Manus glove

hand_mode = forward

hand force intent:

human intent → /target_hand_force

contact-aware regulation handled internally

11.2 Why this is better than old forcecon mode

Because v24 allows:

motion command representation to remain clear

contact-aware correction to be injected internally

no separate mode switching just for hand force regulation

That makes:

mode policy simpler

teleoperation pipeline cleaner

motion/force blending more natural

12) Current Baseline Conclusion

The essential meaning of v24 is:

arm / hand split mode service from v23 is preserved

hand forcecon mode is removed

hand modes are now only:

idle / forward / inverse

hand force regulation is no longer a standalone mode

it is now an internal contact-aware admittance branch inside hand forward / inverse

/target_hand_force is now interpreted as a desired-force reference input, not a separate control mode trigger

In short:

hand mode
=
motion representation
hand mode=motion representation
contact state
=
force-aware internal branch
contact state=force-aware internal branch

This v24 structure is the new baseline.
