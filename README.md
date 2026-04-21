# v27 split: `dualarm_kinematics` + `dualarm_forcecon`

## What this patch does

This patch separates reusable FK/IK code from the ROS2 force-control node.

- `dualarm_kinematics`
  - header-only reusable kinematics package
  - URDF/link/joint-name YAML profiles
  - arm FK/IK, hand FK/IK, kinematics utilities

- `dualarm_forcecon`
  - ROS2 node, callbacks, mode logic, force/admittance control
  - uses `dualarm_kinematics` headers
  - keeps `DualArmForceControl.cpp` role: constructor / destructor / `ControlLoop()` only

## Fast apply method

From your workspace source directory:

```bash
cd ~/dualarm_ws/src
unzip /path/to/dualarm_v27_split_kinematics_package.zip
bash dualarm_v27_split_kinematics_package/apply_v27_split.sh
```

## Manual apply method

```bash
cd ~/dualarm_ws/src
cp -a dualarm_forcecon dualarm_forcecon_backup_before_v27
cp -a dualarm_v27_split_kinematics_package/dualarm_kinematics ./
cp -a dualarm_v27_split_kinematics_package/dualarm_forcecon/include/dualarm_forcecon/control ./dualarm_forcecon/include/dualarm_forcecon/
cp -a dualarm_v27_split_kinematics_package/dualarm_forcecon/src/* ./dualarm_forcecon/src/
cp -a dualarm_v27_split_kinematics_package/dualarm_forcecon/yaml/forcecon_cfg.yaml ./dualarm_forcecon/yaml/forcecon_cfg.yaml
```

The old folder can remain temporarily but should no longer be included by code:

```bash
mv ~/dualarm_ws/src/dualarm_forcecon/include/dualarm_forcecon/Kinematics \
   ~/dualarm_ws/src/dualarm_forcecon/include/dualarm_forcecon/Kinematics_old_v26
```

## Required `dualarm_forcecon` CMake/package.xml change

The apply script tries to do this automatically. If build fails, check manually.

Add this dependency to `dualarm_forcecon/package.xml`:

```xml
<depend>dualarm_kinematics</depend>
```

Add this to `dualarm_forcecon/CMakeLists.txt`:

```cmake
find_package(dualarm_kinematics REQUIRED)
```

Then add `dualarm_kinematics` to the `ament_target_dependencies(...)` block for your executable.

If your target still fails to link KDL/yaml/pinocchio symbols, explicitly link the exported interface target:

```cmake
target_link_libraries(your_node_target
  dualarm_kinematics
)
```

## Build

```bash
cd ~/dualarm_ws
colcon build --symlink-install
source install/setup.bash
```

## Select robot profile

Default force-control config points to the Doosan/AIDIN profile:

```yaml
kinematics:
  config_yaml: /home/eunseop/dualarm_ws/src/dualarm_kinematics/config/doosan_dualarm_kinematics.yaml
```

For RBY1:

```yaml
kinematics:
  config_yaml: /home/eunseop/dualarm_ws/src/dualarm_kinematics/config/rby1_kinematics.yaml
```

Or override at runtime:

```bash
ros2 run dualarm_forcecon node_dualarm_main --ros-args \
  -p kinematics_cfg_yaml:=/home/eunseop/dualarm_ws/src/dualarm_kinematics/config/rby1_kinematics.yaml
```

## Important

`rby1_kinematics.yaml` is a template. You still need to replace `base_link`, `left_ee_link`, `right_ee_link`, and joint names with the exact names from your RBY1 URDF.
