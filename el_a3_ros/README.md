# EL-A3 ROS2 Control System

> 7-DOF 桌面机械臂 ROS2 控制系统，基于 ros2_control、CAN 总线 Robstride 电机、Pinocchio RNEA 重力补偿、MoveIt2 运动规划。

---

## 快速开始

```bash
# 安装依赖
cd scripts && sudo ./install_deps.sh

# 编译
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 配置 CAN 接口
sudo bash scripts/setup_can.sh can0 1000000

# 启动真实硬件
ros2 launch el_a3_description el_a3_control.launch.py can_interface:=can0

# 仿真（无硬件）
ros2 launch el_a3_moveit_config demo.launch.py
```

---

## 功能包

| 功能包 | 说明 |
|--------|------|
| `el_a3_hardware` | ros2_control 硬件接口 + CAN 驱动 + ZeroTorqueController 插件 |
| `el_a3_description` | URDF (7 关节)、控制器配置、Launch 文件 |
| `el_a3_moveit_config` | MoveIt2 运动规划（机械臂 + 夹爪分组） |
| `EDULITE-A3` | 旧版 URDF 参考包 |
| `EL_A3_urdf` | 备用 URDF |

---

## 目录结构

| 目录/文件 | 说明 |
|-----------|------|
| `el_a3_description/` | URDF、Meshes、控制器配置、Launch 文件 |
| `el_a3_hardware/` | C++ ros2_control 硬件接口和 CAN 驱动 |
| `el_a3_moveit_config/` | MoveIt2 配置（SRDF、运动学、关节限制） |
| `scripts/` | 安装依赖、CAN 配置、标定脚本、测试 |

---

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
