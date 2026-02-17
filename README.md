# dualarm_forcecon

**Dual Arm Force Control Package** based on ROS 2.

## 📝 Version History

### **v1: Initial Setup**
- Created the basic package structure.
- Implemented the skeleton for the main class: `DualArmForceControl`.
- Configured the main executable node: `dualarm_ctrl`.
- Set up `CMakeLists.txt` and `package.xml` for successful build.

## ⚙️ Package Information

| Component | Name | Description |
| :--- | :--- | :--- |
| **Package Name** | `dualarm_forcecon` | The ROS 2 package name. |
| **Executable Node** | `dualarm_ctrl` | The main entry point for the control loop. |
| **Main Class** | `DualArmForceControl` | The core C++ class handling force control logic. |

## 🚀 Build & Run

### 1. Build
```bash
cd ~/dualarm_ws
colcon build --symlink-install --packages-select dualarm_forcecon
source install/setup.bash
