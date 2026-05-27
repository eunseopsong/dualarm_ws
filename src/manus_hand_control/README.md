# manus_hand_control

MANUS glove to AIDIN hand bridge for `dualarm_forcecon`.

`aidin_hand_control` subscribes to:

- `/manus_glove_0`
- `/manus_glove_1`

and publishes a `std_msgs/msg/Float64MultiArray` to:

- `/forward_hand_joint_targets`

The output is 30 values: left hand 15 plus right hand 15, ordered for the
current `dualarm_forcecon` forward hand path.

## Run

Source the MANUS workspace first so `manus_ros2_msgs` and the SDK publisher are
available:

```bash
source ~/manus_ws/install/setup.bash
source ~/dualarm_ws/install/setup.bash
ros2 run manus_ros2 manus_data_publisher
ros2 run manus_hand_control aidin_hand_control
```

Optional mode service call on startup:

```bash
ros2 run manus_hand_control aidin_hand_control --ros-args \
  -p auto_set_forcecon_mode:=true \
  -p forcecon_arm_mode:=inverse \
  -p forcecon_hand_mode:=forward
```
