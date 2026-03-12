# EL-A3 Robotic Arm ROS2 Control System

> 7-DOF desktop robotic arm based on `ros2_control`, CAN bus Robstride motors, Pinocchio RNEA gravity compensation, dual-mode Python SDK, Xbox teleoperation, MoveIt2 planning, and multi-arm management.

---

## Documentation / 文档

| Language | File |
|----------|------|
| 中文 | [README_zh.md](README_zh.md) |
| English | [README_en.md](README_en.md) |

---

## Quick Start

```bash
# Install dependencies
cd scripts && sudo ./install_deps.sh

# Build workspace
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Setup CAN interface
sudo ./scripts/setup_can.sh can0 1000000

# Real hardware + Xbox teleop
ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0

# Simulation (no hardware)
ros2 launch el_a3_moveit_config demo.launch.py
```

---

## Packages

| Package | Description |
|---------|-------------|
| `el_a3_hardware` | ros2_control hardware interface + CAN driver + ZeroTorqueController plugin |
| `el_a3_description` | URDF (7 joints), controller config, launch files |
| `el_a3_moveit_config` | MoveIt2 motion planning (arm + gripper groups) |
| `el_a3_teleop` | Xbox Cartesian teleoperation + master-slave teleoperation |
| `el_a3_sdk` | Python SDK -- Direct CAN + ROS Control dual-mode, ArmManager, Pinocchio dynamics |
| `el_a3_web_ui` | Flask + SocketIO web control interface via SDK Bridge |

---

## License

Apache-2.0
