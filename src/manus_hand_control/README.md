# manus_hand_control

MANUS glove to AIDIN hand bridge for `dualarm_forcecon`.

`aidin_hand_control` subscribes to:

- `/manus_glove_0`
- `/manus_glove_1`

and publishes a `std_msgs/msg/Float64MultiArray` to:

- `/forward_hand_joint_targets`

The output is 30 values: left hand 15 plus right hand 15, ordered for the
current `dualarm_forcecon` forward hand path:

```text
left thumb/index/middle/ring/baby xyz, then right thumb/index/middle/ring/baby xyz
```

The old `manus_to_isaac_forcecon.py` script used a reversed finger order. Set
`legacy_reverse_finger_order:=true` only if you are running an older forcecon
callback that expects that order.

## Run

Source the MANUS workspace in every terminal so `manus_ros2_msgs` and the SDK
publisher are available.

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

ros2 run manus_hand_control aidin_hand_control
```

Optional mode service call on startup:

```bash
cd ~/dualarm_ws
source ~/manus_ws/install/setup.bash
source install/setup.bash

ros2 run manus_hand_control aidin_hand_control --ros-args \
  -p auto_set_forcecon_mode:=true \
  -p forcecon_arm_mode:=inverse \
  -p forcecon_hand_mode:=forward
```

Print incoming MANUS glove ergonomics values:

```bash
cd ~/dualarm_ws
source ~/manus_ws/install/setup.bash
source install/setup.bash

ros2 run manus_hand_control aidin_hand_control --ros-args \
  -p print_manus_input:=true \
  -p print_interval_s:=0.5
```

If `~/manus_ws` has the extended `ManusGlove` message, this log also includes
`battery=<percent>%` and `rssi=<transmission_strength>`.
