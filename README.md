# dualarm_forcecon

**Dual Arm Force Control Package** based on ROS 2.

## 📝 Version History

### **v2: State Feedback Implementation**
- **Callback Separation**: Moved callback implementations to `src/states_callback_dualarm.cpp` for better code structure.
- **Joint State Parsing**: Implemented `JointsCallback` to subscribe to `/isaac_joint_states`.
- **Hardware Support**:
  - Parsed **Dual Arm** joints (Left/Right, 7 DOF each).
  - Parsed **Aidin Hands** joints (Left/Right, 15 DOF each).
- **Build System**: Updated `CMakeLists.txt` to include the separated source files.

### **v1: Initial Setup**
- Created the basic package structure.
- Implemented the skeleton for the main class: `DualArmForceControl`.
- Configured the main executable node: `dualarm_ctrl`.
- Set up `CMakeLists.txt` and `package.xml`.

## ⚙️ Package Information

| Component | Name | Description |
| :--- | :--- | :--- |
| **Package Name** | `dualarm_forcecon` | The ROS 2 package name. |
| **Executable Node** | `dualarm_ctrl` | The main entry point for the control loop. |
| **Main Class** | `DualArmForceControl` | The core C++ class handling force control logic. |
| **Callbacks File** | `states_callback_dualarm.cpp` | Handles sensor data parsing (Joints, FT Sensor). |

## 🚀 Build & Run

### 1. Build
```bash
cd ~/dualarm_ws
colcon build --symlink-install --packages-select dualarm_forcecon
source install/setup.bash
