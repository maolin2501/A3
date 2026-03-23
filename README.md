# EL-A3 Robotic Arm

> 7-DOF 桌面机械臂项目，包含纯 Python SDK 和 ROS2 控制系统两个独立子项目。

---

## 子项目

| 项目 | 路径 | 说明 |
|------|------|------|
| **Python SDK** | [`el_a3_sdk/`](el_a3_sdk/) | 纯 Python SDK，Direct CAN 通信，多臂管理，Pinocchio 动力学 |
| **ROS2 控制系统** | [`el_a3_ros/`](el_a3_ros/) | ros2_control 硬件接口，MoveIt2 运动规划，URDF 描述 |

各子项目的详细文档请参阅对应目录下的 README.md。
