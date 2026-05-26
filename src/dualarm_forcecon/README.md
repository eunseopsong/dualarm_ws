# dualarm_forcecon

ROS 2 Humble control node for dual-arm, hand, torso hold, and RBY1 wheel command forwarding in Isaac.

## RBY1 Wheel Commands

Run `dualarm_forcecon_node` and send base velocity commands through `/cmd_vel`. Do not publish wheel-only commands directly to `/isaac_joint_command`, because that bypasses the full-body hold command for torso/head/arms/hands.

While a wheel command is active, `dualarm_forcecon_node` also publishes the configured `torso_*` upright position targets from `yaml/forcecon_cfg.yaml` so the torso is held vertical during forward/backward motion.

Start the controller:

```bash
cd ~/dualarm_ws
source install/setup.bash
ros2 run dualarm_forcecon dualarm_forcecon_node
```

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

In the current Isaac setup, positive `/cmd_vel.linear.x` is forward because `mobile_base.invert_wheel_velocity_command` is set to `true` in `yaml/forcecon_cfg.yaml`.

The `/cmd_vel` command is converted to `left_wheel` and `right_wheel` velocity commands using `mobile_base` parameters in `yaml/forcecon_cfg.yaml`. The torso upright target is configured by `mobile_base.torso_upright_joint_names` and `mobile_base.torso_upright_positions`.
