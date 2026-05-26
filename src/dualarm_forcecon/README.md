# dualarm_forcecon

ROS 2 Humble control node for dual-arm, hand, torso hold, and RBY1 wheel command forwarding in Isaac.

## RBY1 Wheel Commands

Run `dualarm_forcecon_node` and send base velocity commands through `/cmd_vel`. Do not publish wheel-only commands directly to `/isaac_joint_command`, because that bypasses the full-body hold command for torso/head/arms/hands.

Forward:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3}, angular: {z: 0.0}}"
```

Backward:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: -0.3}, angular: {z: 0.0}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.0}}"
```

The `/cmd_vel` command is converted to `left_wheel` and `right_wheel` velocity commands using `mobile_base` parameters in `yaml/forcecon_cfg.yaml`.
