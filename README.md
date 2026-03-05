# EL-A3 机械臂 ROS2 控制系统

> **EL-A3** 是一款 7 自由度（6 臂关节 + L7 夹爪）桌面级机械臂，基于标准 `ros2_control` 分层架构构建，通过 CAN 总线驱动 Robstride 电机。系统采用 `controller_manager` 管理多控制器（位置/夹爪/零力矩），Pinocchio RNEA 动力学实现重力补偿，提供双模式 Python SDK（CAN 直连 + ROS Control），支持 Xbox 手柄笛卡尔遥操作、ROS namespace 主从遥操作、拖动示教、MoveIt2 运动规划及多臂管理。

---

## 目录

- [控制架构](#控制架构)
- [系统概述](#系统概述)
- [硬件要求](#硬件要求)
- [软件环境](#软件环境)
- [功能包说明](#功能包说明)
- [安装配置](#安装配置)
- [快速开始](#快速开始)
- [控制参数](#控制参数)
- [ROS2 接口](#ros2-接口)
- [电机通信协议](#电机通信协议)
- [SDK 协议与接口说明](#sdk-协议与接口说明)
- [故障排除](#故障排除)
- [目录结构](#目录结构)
- [English Version](#control-architecture)

---

## 控制架构

### 分层架构总览

系统严格遵循 `ros2_control` 标准分层设计，每层职责清晰：

```
┌─────────────────────────────────────────────────────────────┐
│                        应用层                                │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐  │
│  │ MoveIt2  │ │  Xbox    │ │  主从    │ │  Python SDK   │  │
│  │ 运动规划  │ │  遥操作  │ │  遥操作  │ │ (el_a3_sdk)   │  │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └──────┬────────┘  │
└───────┼────────────┼────────────┼───────────────┼───────────┘
        │Action      │ Topic      │ switch_       │ JointCtrl/
        │ /move_action│ /joint_traj│ controller    │ GripperCtrl
        │            │            │               │
┌───────┼────────────┼────────────┼───────────────┼───────────┐
│       │     controller_manager (200 Hz)         │           │
│  ┌────▼────────────▼───┐ ┌─────▼─────┐ ┌───────▼────────┐  │
│  │   arm_controller    │ │ gripper_  │ │ zero_torque_   │  │
│  │ (JointTrajCtrl)     │ │ controller│ │ controller     │  │
│  │ L1-L6 position cmd  │ │ L7 pos   │ │ L1-L7 effort   │  │
│  └──────────┬──────────┘ └─────┬─────┘ │ (Pinocchio     │  │
│             │                  │        │  RNEA 重力补偿) │  │
│             │ position         │ pos    └───────┬────────┘  │
│             │ command          │ cmd      effort│cmd        │
│  ┌──────────▼──────────────────▼────────────────▼────────┐  │
│  │              joint_state_broadcaster                   │  │
│  │           (发布 /joint_states @ 200Hz)                 │  │
│  └───────────────────────────────────────────────────────┘  │
└──────────────────────────────┬──────────────────────────────┘
                               │
┌──────────────────────────────▼──────────────────────────────┐
│               RsA3HardwareInterface                          │
│            (hardware_interface::SystemInterface)              │
│                                                              │
│  Command Interfaces: position (L1-L7), effort (L1-L7)       │
│  State Interfaces:   position (L1-L7), velocity (L1-L7)     │
│                                                              │
│  write():                                                    │
│    effort_mode → Kp=0, Kd=damping, τ=effort_cmd             │
│    position_mode → Kp=80, Kd=4, τ=gravity_feedforward       │
│                                                              │
│  关节限位保护 (per-joint clamp + 告警)                         │
│  Pinocchio 重力前馈 (简化三角 + RNEA)                          │
│  速度前馈 (位置差分 + 低通滤波)                                 │
└──────────────────────────────┬──────────────────────────────┘
                               │ CAN Bus (1 Mbps)
                               │ MIT-like PD Control
┌──────────────────────────────▼──────────────────────────────┐
│                  Robstride 电机 (7x)                         │
│        τ = Kp(θ_t - θ) + Kd(ω_t - ω) + τ_ff               │
│        RS00 (L1-L3) + EL05/RS05 (L4-L7)                    │
└─────────────────────────────────────────────────────────────┘
```

### 核心设计原则

| 原则 | 实现 |
|------|------|
| **硬件接口只做硬件抽象** | `RsA3HardwareInterface` 仅实现 `read()/write()`，不创建 Service/Topic |
| **模式切换通过切换 Controller** | 位置控制 ↔ 零力矩通过 `controller_manager/switch_controller` 服务 |
| **夹爪独立 Controller** | `gripper_controller` (JointTrajectoryController) 独立管理 L7_joint |
| **命名空间自动跟随** | 所有 topic/service 跟随 ROS namespace，支持多臂部署 |
| **SDK 状态机管理** | `ArmState` 枚举管理生命周期：DISCONNECTED → IDLE → ENABLED → RUNNING/ZERO_TORQUE |

### 控制器配置

| 控制器 | 类型 | 管理关节 | 功能 |
|--------|------|----------|------|
| `arm_controller` | JointTrajectoryController | L1-L6 | 臂关节位置轨迹跟踪 |
| `gripper_controller` | JointTrajectoryController | L7 | 夹爪位置控制 |
| `zero_torque_controller` | el_a3_hardware/ZeroTorqueController | L1-L7 | 重力补偿零力矩（拖动示教） |
| `joint_state_broadcaster` | JointStateBroadcaster | L1-L7 | 关节状态发布 |

### 模式切换流程

```bash
# 位置控制 → 零力矩（拖动示教）
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['zero_torque_controller'], \
    deactivate_controllers: ['arm_controller', 'gripper_controller'], \
    strictness: 1}"

# 零力矩 → 位置控制
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['arm_controller', 'gripper_controller'], \
    deactivate_controllers: ['zero_torque_controller'], \
    strictness: 1}"
```

### SDK 双模式架构

```
┌───────────────────────────────────────────────┐
│                el_a3_sdk                       │
│                                               │
│  ┌─────────────────┐  ┌───────────────────┐  │
│  │  ELA3Interface   │  │ ELA3ROSInterface  │  │
│  │  (CAN 直连)      │  │ (ROS Control)     │  │
│  │  调试/测试用      │  │ 正式控制用         │  │
│  └───────┬─────────┘  └────────┬──────────┘  │
│          │                     │              │
│  ┌───────▼─────────┐  ┌───────▼──────────┐  │
│  │ RobstrideCanDrv  │  │ ROS2 Topics/     │  │
│  │ (SocketCAN)      │  │ Actions/Services │  │
│  └───────┬─────────┘  └────────┬──────────┘  │
└──────────┼─────────────────────┼──────────────┘
           │                     │
      CAN Bus              controller_manager
           │                     │
      ┌────▼─────────────────────▼────┐
      │       Robstride 电机           │
      └───────────────────────────────┘

  ArmManager: 单例多臂管理器，统一注册/获取 CAN/ROS 臂实例
  ArmState: DISCONNECTED → IDLE → ENABLED → RUNNING / ZERO_TORQUE
```

两种模式共享 `protocol.py`（协议枚举）、`data_types.py`（数据结构）、`kinematics.py`（Pinocchio 动力学）。

---

## 系统概述

### 主要特性

- **标准 ros2_control 架构**: controller_manager 管理多控制器，硬件接口严格只做 read/write
- **独立零力矩控制器**: 自定义 `ZeroTorqueController` 插件，通过 Pinocchio RNEA 实现重力补偿拖动示教
- **独立夹爪控制器**: L7 使用独立 `gripper_controller`，与臂控制器解耦
- **Pinocchio 动力学**: 完整 RNEA 重力补偿（C++ 硬件层 + Python SDK 层），支持标定惯量参数加载
- **关节限位保护**: 硬件层 per-joint 软限位钳位 + 告警日志
- **实时笛卡尔控制**: 50Hz Xbox 手柄笛卡尔空间遥操作
- **MoveIt2 集成**: 运动规划、IK 求解、笛卡尔路径规划
- **多臂管理**: ROS namespace 隔离 + `ArmManager` 单例统一管理
- **双模式 Python SDK**: CAN 直连（调试）+ ROS Control（正式），7 关节统一
- **ArmState 状态机**: SDK 生命周期管理，方法调用前自动状态验证
- **速度前馈 + 多级平滑**: 位置差分前馈 + 低通滤波 + 加速度限制

### 机械臂参数

| 属性 | 规格 |
|------|------|
| 自由度 | 7 DOF（6 臂关节 + L7 夹爪） |
| 末端执行器 | 可定制 |
| 通信协议 | Robstride 私有协议 (CAN 2.0, 扩展帧29位ID) |
| 波特率 | 1Mbps |
| 控制模式 | 运控模式 (MIT-like PD Control) |

### 电机配置

4-7 号关节支持 EL05 和 RS05 两种电机型号，通过 `wrist_motor_type` 参数切换（默认 EL05）。

| 关节 | 电机ID | 型号 | 力矩限制 | 速度限制 | 位置限制 | 方向 |
|------|--------|------|----------|----------|----------|------|
| L1_joint | 1 | RS00 | ±14 Nm | ±33 rad/s | ±2.79 rad (±160°) | -1 |
| L2_joint | 2 | RS00 | ±14 Nm | ±33 rad/s | 0~3.67 rad (0°~210°) | +1 |
| L3_joint | 3 | RS00 | ±14 Nm | ±33 rad/s | -4.01~0 rad (-230°~0°) | -1 |
| L4_joint | 4 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |
| L5_joint | 5 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | -1 |
| L6_joint | 6 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |
| L7_joint（夹爪） | 7 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |

**EL05 vs RS05 参数差异**:

| 参数 | EL05 | RS05 |
|------|------|------|
| 峰值力矩 | 6.0 Nm | 5.5 Nm |
| 额定力矩 | 1.8 Nm | 1.6 Nm |
| 减速比 | 9:1 | 7.75:1 |
| 最大电流 | 10 Apk | 11 Apk |

---

## 硬件要求

### 必需硬件

- **EL-A3 机械臂** (含 7 个 Robstride 电机)
- **CAN 适配器**: CANdle / gs_usb 兼容设备
- **电源**: 24V/48V 直流电源
- **PC**: Ubuntu 22.04 x86_64

### 可选硬件

- **Xbox 手柄**: Xbox One / Xbox Series X|S / XInput 兼容手柄（有线或蓝牙）

---

## 软件环境

### 系统要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble Hawksbill
- **内核模块**: `gs_usb`

### 依赖安装

```bash
cd scripts && sudo ./install_deps.sh
```

或手动安装：

```bash
# ROS2 Control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-hardware-interface ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller

# MoveIt2
sudo apt install ros-humble-moveit ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-planning-interface ros-humble-moveit-ros-visualization \
  ros-humble-moveit-planners-ompl ros-humble-moveit-kinematics

# 工具
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-joy can-utils

# Python
pip3 install python-can scipy
```

---

## 功能包说明

| 功能包 | 说明 |
|--------|------|
| `el_a3_hardware` | ros2_control 硬件接口 + CAN 驱动 + ZeroTorqueController 插件 |
| `el_a3_description` | URDF 描述（7 关节）、控制器配置、Launch 文件 |
| `el_a3_moveit_config` | MoveIt2 运动规划配置（arm + gripper 规划组） |
| `el_a3_teleop` | Xbox 手柄笛卡尔控制 + ROS namespace 主从遥操作 |
| `el_a3_sdk` | Python SDK — CAN 直连 + ROS Control 双模式、ArmManager、MoveIt 集成、Pinocchio 动力学 |
| `el_a3_web_ui` | Flask + SocketIO Web 控制界面，通过 SDK Bridge 统一调用 SDK 接口 |

### Python SDK (`el_a3_sdk`)

```python
from el_a3_sdk import ArmManager

mgr = ArmManager()

# 注册 ROS namespace 模式臂
master = mgr.register_ros_arm("master", namespace="arm1",
    controller_name="arm1_arm_controller")
master.ConnectPort()
master.EnableArm()

# 关节控制（7 关节）
master.JointCtrl(0.0, 1.57, -0.78, 0.0, 0.0, 0.0, joint_7=0.5)

# 夹爪控制（通过 gripper_controller）
master.GripperCtrl(gripper_angle=0.3)

# MoveIt 运动规划 (ROS 模式独有)
master.PlanToJointGoal([0.0]*6, velocity_scale=0.3)          # 同步关节规划
master.EndPoseCtrl(0.3, 0.0, 0.3, 0.0, 0.0, 0.0)            # 笛卡尔位姿控制
master.PlanToJointGoalAsync([0.5]*6, result_callback=print)   # 异步关节规划

# 动力学 (Pinocchio)
tau_g = master.ComputeGravityTorques()                         # 重力补偿力矩
J = master.GetJacobian()                                       # Jacobian 矩阵

# 零力矩模式（通过 switch_controller）
master.ZeroTorqueMode(True)   # 激活 zero_torque_controller
master.ZeroTorqueMode(False)  # 恢复 arm_controller + gripper_controller

# 状态查询
print(master.arm_state)           # ArmState.ENABLED / RUNNING / ZERO_TORQUE / ...
print(master.GetArmEndPoseMsgs()) # ArmEndPose(x=..., y=..., z=..., rx=..., ry=..., rz=...)
```

安装：
```bash
cd el_a3_sdk
pip install -e .            # 基本安装
pip install -e ".[dynamics]"  # 含 Pinocchio 支持
```

---

## 安装配置

### 1. 编译工作空间

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. 设置 CAN 接口

```bash
sudo ./scripts/setup_can.sh can0 1000000
ip link show can0
candump can0
```

---

## 快速开始

### Xbox 手柄实时控制（推荐）

```bash
# 终端1: 设置 CAN
sudo ./scripts/setup_can.sh can0

# 终端2: 启动控制系统
cd ros2_ws && source install/setup.bash
ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0

# 若使用 RS05 电机（4-7号关节），加参数:
# ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0 wrist_motor_type:=RS05
```

或一键启动：`./scripts/start_real_xbox_control.sh can0`

**手柄控制映射**:

| 按键/摇杆 | 功能 |
|-----------|------|
| 左摇杆 | XY 平移 |
| LT/RT | Z 方向上下 |
| 右摇杆 | Yaw/Pitch 旋转 |
| LB/RB | Roll 旋转 |
| A 键 | 切换速度档位 (5档) |
| B 键 | 回到 home 位置 |
| X 键 | 回到零点位置 |
| Y 键 | 切换零力矩模式（switch_controller 拖动示教） |
| Menu 键 | 切换主从遥操作 |
| 方向键上/下 | 夹爪闭合/张开（通过 gripper_controller） |

### 主从遥操作模式

```bash
# 设置双 CAN
sudo ./scripts/setup_can.sh can0 1000000
sudo ./scripts/setup_can.sh can1 1000000

# 启动多臂控制
ros2 launch el_a3_description multi_arm_control.launch.py

# 启动遥操作
ros2 launch el_a3_teleop real_teleop.launch.py \
  master_namespace:=arm1 slave_namespace:=arm2 \
  master_controller:=arm1_arm_controller slave_controller:=arm2_arm_controller
```

按 Menu 键切换：主臂激活 `zero_torque_controller`（可拖动），从臂通过 ROS 轨迹话题实时跟随。

### 仿真模式

```bash
# MoveIt Demo（无需硬件）
ros2 launch el_a3_moveit_config demo.launch.py

# 仿真 + Xbox 手柄
ros2 launch el_a3_teleop sim_teleop.launch.py
```

### 真实硬件 + MoveIt

```bash
sudo ./scripts/setup_can.sh can0
ros2 launch el_a3_moveit_config robot.launch.py can_interface:=can0
```

---

## 控制参数

### 硬件接口参数 (`el_a3_ros2_control.xacro`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_interface` | can0 | CAN 接口名称 |
| `host_can_id` | 253 (0xFD) | 主机 CAN ID |
| `wrist_motor_type` | EL05 | 腕部电机型号 (EL05 或 RS05)，影响 L4-L7 力矩映射 |
| `position_kp` | 80.0 | 位置 PD 控制 Kp 增益 |
| `position_kd` | 4.0 | 位置 PD 控制 Kd 增益 |
| `velocity_limit` | 10.0 | 速度限制 (rad/s) |
| `smoothing_alpha` | 0.08 | 低通滤波系数 (0-1) |
| `use_pinocchio_gravity` | true | 启用 Pinocchio RNEA 重力补偿 |
| `gravity_feedforward_ratio` | 0.5 | 正常模式重力前馈比例 |
| `zero_torque_kd` | 1.0 | 零力矩模式全局阻尼 |
| `limit_margin` | 0.15 | 关节限位减速区域 (rad) |
| `limit_stop_margin` | 0.02 | 关节限位硬停止区域 (rad) |
| `can_frame_delay_us` | 50 | CAN 帧间延迟 (μs) |

**动态参数**（运行时可通过 ROS2 参数服务调整）：`position_kp`、`position_kd`、`gravity_feedforward_ratio`、`zero_torque_kd`、`smoothing_alpha`

### 重力补偿系统

系统使用 **Pinocchio RNEA 完整动力学模型** 进行重力补偿：

```
τ_gravity = RNEA(model, data, q, v=0, a=0)
```

重力补偿在两个层面运行：

| 层面 | 控制器 | 补偿比例 | 用途 |
|------|--------|----------|------|
| 硬件接口 `write()` | `arm_controller` 激活时 | `gravity_feedforward_ratio` (50%) | 正常位置控制时的前馈 |
| ZeroTorqueController | `zero_torque_controller` 激活时 | 100% (RNEA 全量) | 拖动示教 |

**惯量参数标定**：通过 `scripts/inertia_calibration.py` 自动标定 L2-L6 的质量和质心参数，保存至 `el_a3_description/config/inertia_params.yaml`。

```bash
python3 scripts/inertia_calibration.py          # 完整标定 (~10min)
python3 scripts/inertia_calibration.py --quick   # 快速标定 (~3min)
```

### 零力矩模式（拖动示教）

零力矩模式通过 **控制器切换** 实现（标准 ros2_control 模式）：

1. `controller_manager` 停用 `arm_controller` + `gripper_controller`
2. 激活 `zero_torque_controller`（自定义 ControllerInterface 插件）
3. `zero_torque_controller` claim effort 命令接口，读取关节位置，通过 Pinocchio RNEA 计算重力力矩并写入
4. 硬件接口检测到 effort_mode，设置 Kp=0、Kd=damping、τ_ff=effort_cmd

**SDK / Teleop 调用方式**：

```python
arm.ZeroTorqueMode(True)   # 内部调用 switch_controller
arm.ZeroTorqueMode(False)  # 恢复位置控制器
```

### 控制器参数 (`el_a3_controllers.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `update_rate` | 200 Hz | controller_manager 更新频率 |
| `arm_controller` joints | L1-L6 | 臂关节位置轨迹控制 |
| `gripper_controller` joints | L7 | 夹爪位置控制 |
| `zero_torque_controller` joints | L1-L7 | 零力矩（effort 模式） |
| `zero_torque_controller` kd | 1.0 | 零力矩阻尼系数 |

### Xbox 遥控参数 (`xbox_teleop.yaml`)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `update_rate` | 50.0 Hz | 控制循环频率 |
| `use_fast_ik_mode` | true | 快速 IK 模式 |
| `max_linear_velocity` | 0.15 m/s | 最大线速度 |
| `max_angular_velocity` | 1.5 rad/s | 最大角速度 |
| `joint_smoothing_alpha` | 0.15 | 关节输出平滑系数 |
| `max_joint_velocity` | 1.5 rad/s | 单关节最大速度 |
| `max_ik_jump_threshold` | 0.5 rad | IK 跳变阈值 |
| `enable_collision_check` | true | 碰撞检测 |

---

## ROS2 接口

### Topics

#### 发布

| Topic | 类型 | 频率 | 说明 |
|-------|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | L1-L7 关节状态 |
| `/robot_description` | `std_msgs/String` | latched | URDF 描述 |
| `/target_pose` | `geometry_msgs/PoseStamped` | 50 Hz | 目标末端位姿 |
| `/debug/hw_command` | `sensor_msgs/JointState` | 20 Hz | 控制器命令位置 |
| `/debug/smoothed_command` | `sensor_msgs/JointState` | 20 Hz | 平滑后电机命令 |
| `/debug/gravity_torque` | `sensor_msgs/JointState` | 20 Hz | 重力补偿力矩 |
| `/debug/motor_temperature` | `sensor_msgs/JointState` | 4 Hz | 电机温度 |
| `/zero_torque_controller/gravity_torques` | `sensor_msgs/JointState` | 200 Hz | ZeroTorqueController 重力力矩 |

#### 订阅

| Topic | 类型 | 说明 |
|-------|------|------|
| `/joy` | `sensor_msgs/Joy` | Xbox 手柄输入 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 臂关节轨迹指令 |
| `/gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 夹爪轨迹指令 |

### Services

| Service | 类型 | 说明 |
|---------|------|------|
| `/controller_manager/switch_controller` | `controller_manager_msgs/SwitchController` | 控制器切换（位置 ↔ 零力矩） |
| `/controller_manager/list_controllers` | `controller_manager_msgs/ListControllers` | 列出控制器状态 |
| `/compute_ik` | `moveit_msgs/GetPositionIK` | MoveIt IK 求解 |
| `/compute_cartesian_path` | `moveit_msgs/GetCartesianPath` | MoveIt 笛卡尔路径 |

### Actions

| Action | 类型 | 说明 |
|--------|------|------|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 臂轨迹执行 |
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | 夹爪轨迹执行 |
| `/move_action` | `moveit_msgs/MoveGroup` | MoveIt 运动规划 |
| `/execute_trajectory` | `moveit_msgs/ExecuteTrajectory` | MoveIt 轨迹执行 |

### TF Frames

```
world (virtual_joint)
└── base_link
    └── L1_joint → l1_link_urdf_asm
        └── L2_joint → l2_l3_urdf_asm
            └── L3_joint → l3_lnik_urdf_asm
                └── L4_joint → l4_l5_urdf_asm
                    └── L5_joint → part_9
                        └── L6_joint → l5_l6_urdf_asm
                            ├── end_effector_joint (fixed) → end_effector  ← MoveIt TCP
                            └── L7_joint (revolute) → gripper_link         ← 夹爪
```

> `end_effector` 和 `gripper_link` 是 `l5_l6_urdf_asm` 的两个子节点（siblings）。MoveIt `arm` 规划组的运动链终止于 `end_effector`，`gripper` 规划组包含 `L7_joint`。

### MoveIt 规划组

| 规划组 | 类型 | 内容 |
|--------|------|------|
| `arm` | chain | base_link → end_effector (L1-L6) |
| `gripper` | joint | L7_joint |

---

## 电机通信协议

系统使用 Robstride 私有协议，采用 **运控模式 (MIT-like)** 进行实时控制：

```
τ = Kp × (θ_target - θ_actual) + Kd × (ω_target - ω_actual) + τ_ff
```

| 参数 | 范围 | 说明 |
|------|------|------|
| θ_target | ±12.57 rad | 目标位置 |
| ω_target | 见电机规格 | 速度前馈 |
| Kp | 0~500 | 位置刚度 |
| Kd | 0~5 | 阻尼系数 |
| τ_ff | 见电机规格 | 前馈力矩（重力补偿） |

| 通信类型 | 功能 |
|----------|------|
| 1 | 运控模式控制（位置/速度/Kp/Kd/力矩） |
| 2 | 电机反馈（位置/速度/力矩/温度） |
| 3 | 电机使能 |
| 4 | 电机停止 |
| 6 | 设置零位 |
| 18 | 参数写入 |

详细协议说明参考 [`电机通信协议汇总.md`](./电机通信协议汇总.md)。

---

## SDK 协议与接口说明

### 模块结构

```
el_a3_sdk/
├── __init__.py          # 包入口，延迟导入 ROS/Pinocchio 可选依赖
├── protocol.py          # 协议枚举、电机参数、关节配置常量
├── data_types.py        # 数据结构（SI 单位: rad, Nm, m）
├── can_driver.py        # SocketCAN 底层驱动（帧收发 + 后台接收线程）
├── interface.py         # ELA3Interface — CAN 直连 API
├── ros_interface.py     # ELA3ROSInterface — ROS Control API
├── arm_manager.py       # ArmManager — 单例多臂管理器
├── kinematics.py        # Pinocchio FK/IK/Jacobian/重力/动力学
├── trajectory.py        # S-curve 七段式 + 三次样条轨迹规划
├── utils.py             # float⇆uint16 映射、单位转换、clamp
└── demo/                # 示例脚本
```

> **完整 API 参考手册**: 请查阅 [`el_a3_sdk/docs/SDK_API_Protocol.md`](el_a3_sdk/docs/SDK_API_Protocol.md)，包含所有 API 签名、参数说明、协议枚举、数据结构、CAN 帧格式及使用示例。

### 协议枚举定义 (`protocol.py`)

#### CAN 通信类型 (`CommType`)

| 值 | 名称 | 方向 | 说明 |
|----|------|------|------|
| 0 | `GET_DEVICE_ID` | 主→从 | 获取设备 ID |
| 1 | `MOTION_CONTROL` | 主→从 | 运控模式指令 (PD + τ_ff) |
| 2 | `FEEDBACK` | 从→主 | 电机反馈 (位置/速度/力矩/温度) |
| 3 | `ENABLE` | 主→从 | 使能电机 |
| 4 | `DISABLE` | 主→从 | 停止电机（可选清故障） |
| 6 | `SET_ZERO` | 主→从 | 设置当前位置为零位 |
| 7 | `SET_CAN_ID` | 主→从 | 修改电机 CAN ID |
| 17 | `READ_PARAM` | 双向 | 参数读取 (请求 + 应答) |
| 18 | `WRITE_PARAM` | 主→从 | 参数写入（掉电丢失） |
| 21 | `FAULT_FEEDBACK` | 从→主 | 详细故障码 |
| 22 | `SAVE_PARAMS` | 主→从 | 保存参数到 Flash |
| 26 | `READ_VERSION` | 双向 | 固件版本查询 |

#### 电机型号 (`MotorType`)

| 型号 | 值 | 关节 | 力矩范围 | 速度范围 | 位置范围 |
|------|---|------|----------|----------|----------|
| `RS00` | 0 | L1-L3 | ±14 Nm | ±33 rad/s | ±12.57 rad |
| `EL05` | 1 | L4-L7 (配置A) | ±6 Nm | ±50 rad/s | ±12.57 rad |
| `RS05` | 2 | L4-L7 (配置B) | ±5.5 Nm | ±50 rad/s | ±12.57 rad |

#### 运行模式 (`RunMode`)

| 值 | 名称 | 说明 |
|----|------|------|
| 0 | `MOTION_CONTROL` | 运控模式 (PD + 前馈力矩)，系统默认 |
| 1 | `POSITION_PP` | 位置模式 (PP，梯形规划) |
| 2 | `VELOCITY` | 速度模式 |
| 3 | `CURRENT` | 电流模式 |
| 5 | `POSITION_CSP` | 位置模式 (CSP，连续位置) |

#### 状态机 (`ArmState`)

```
DISCONNECTED ──ConnectPort()──▶ IDLE ──EnableArm()──▶ ENABLED
                                 ▲                      │  ▲
                                 │                      │  │
                          DisableArm()          JointCtrl()/MoveJ()/MoveL()
                                 │                      │  │
                                 │                      ▼  │
                                 │                   RUNNING
                                 │                      │
                                 │              ZeroTorqueMode(True)
                                 │                      │
                                 │                      ▼
                                 ├───────────── ZERO_TORQUE
                                 │         ZeroTorqueMode(False) → ENABLED
                                 │
                          EmergencyStop()
                                 │
                                 ▼
                               ERROR
```

| 状态 | 值 | 允许的操作 |
|------|---|-----------|
| `DISCONNECTED` | 0 | `ConnectPort()` |
| `IDLE` | 1 | `EnableArm()`, `DisconnectPort()` |
| `ENABLED` | 2 | `JointCtrl()`, `MoveJ()`, `MoveL()`, `EndPoseCtrl()`, `CartesianPathCtrl()`, `PlanToJointGoal()`, `GripperCtrl()`, `ZeroTorqueMode()`, `DisableArm()` |
| `RUNNING` | 3 | `JointCtrl()`, `MoveJ()`, `MoveL()`, `EndPoseCtrl()`, `CartesianPathCtrl()`, `PlanToJointGoal()`, `GripperCtrl()`, `ZeroTorqueMode()`, `EmergencyStop()` |
| `ZERO_TORQUE` | 4 | `ZeroTorqueMode(False)`, `EmergencyStop()` |
| `ERROR` | 5 | `EmergencyStop()`, `DisconnectPort()` |

#### 关节默认配置

L4-L7 的默认型号为 EL05，可通过 `wrist_motor_type` 参数切换为 RS05。

| 关节 | 电机ID | 默认型号 | 方向 | 偏移 | 限位 (rad) |
|------|--------|---------|------|------|-----------|
| L1 | 1 | RS00 | -1 | 0.0 | -2.793 ~ +2.793 |
| L2 | 2 | RS00 | +1 | 0.0 | 0.0 ~ +3.665 |
| L3 | 3 | RS00 | -1 | 0.0 | -4.014 ~ 0.0 |
| L4 | 4 | EL05/RS05 | +1 | 0.0 | -1.571 ~ +1.571 |
| L5 | 5 | EL05/RS05 | -1 | 0.0 | -1.571 ~ +1.571 |
| L6 | 6 | EL05/RS05 | +1 | 0.0 | -1.571 ~ +1.571 |
| L7 | 7 | EL05/RS05 | +1 | 0.0 | -1.571 ~ +1.571 |

#### 参数索引 (`ParamIndex`)

| 索引 | 名称 | 说明 |
|------|------|------|
| `0x7005` | `RUN_MODE` | 运行模式切换 |
| `0x7006` | `IQ_REF` | 电流指令 |
| `0x700A` | `SPD_REF` | 速度指令 |
| `0x700B` | `LIMIT_TORQUE` | 力矩限制 |
| `0x7016` | `LOC_REF` | CSP 位置指令 |
| `0x7017` | `LIMIT_SPD` | CSP 速度上限 |
| `0x7019` | `MECH_POS` | 机械位置读取 |
| `0x701A` | `IQF` | 滤波电流读取 |
| `0x701B` | `MECH_VEL` | 机械速度读取 |
| `0x701C` | `VBUS` | 母线电压读取 |
| `0x701E` | `LOC_KP` | 位置环 Kp |
| `0x701F` | `SPD_KP` | 速度环 Kp |

### 数据结构 (`data_types.py`)

所有数据使用 SI 单位（rad, rad/s, Nm, m, °C）。

| 结构体 | 说明 | 主要字段 |
|--------|------|---------|
| `MotorFeedback` | 单电机反馈 (Type 2) | `motor_id`, `position`, `velocity`, `torque`, `temperature`, `fault_code` |
| `ArmJointStates` | 7 关节状态 | `joint_1`~`joint_7`, `to_list(include_gripper=True/False)` |
| `ArmEndPose` | 末端位姿 | `x`, `y`, `z` (m), `rx`, `ry`, `rz` (rad, XYZ 内旋) |
| `ArmStatus` | 臂综合状态 | `ctrl_mode`, `arm_status`, `joint_enabled[]`, `joint_faults[]` |
| `MotorHighSpdInfo` | 高速反馈 | `speed`, `current`, `position`, `torque` |
| `MotorLowSpdInfo` | 低速反馈 | `voltage`, `motor_temp`, `fault_code` |
| `ParamReadResult` | 参数读取结果 | `param_index`, `value`, `success` |
| `FirmwareVersion` | 固件版本 | `version_str` (如 "1.2.3.4.5") |
| `DynamicsInfo` | 动力学信息 | `gravity_torques`, `mass_matrix`, `jacobian` |
| `TrajectoryResult` | 轨迹执行结果 | `success`, `error_code`, `actual_positions` |

### API 参考

#### ELA3Interface (CAN 直连模式)

适用于调试、标定、无 ROS 环境的独立控制。通过 SocketCAN 直接与电机通信。

**连接管理**

| 方法 | 参数 | 返回 | 说明 |
|------|------|------|------|
| `ConnectPort()` | — | `bool` | 打开 CAN socket，启动收发线程，状态→IDLE |
| `DisconnectPort()` | — | — | 停止线程，关闭 socket，状态→DISCONNECTED |

**电机控制**

| 方法 | 参数 | 返回 | 说明 |
|------|------|------|------|
| `EnableArm(motor_num=7, run_mode=MOTION_CONTROL, startup_kd=4.0)` | 使能电机数、运行模式、启动阻尼 | `bool` | 按 disable→set_mode→enable 流程使能，状态→ENABLED |
| `DisableArm(motor_num=7)` | 失能电机数 | `bool` | 失能所有电机，状态→IDLE |
| `EmergencyStop()` | — | `bool` | 立即失能全部电机并清故障，状态→IDLE |
| `SetZeroPosition(motor_num=7)` | — | `bool` | 设置当前位置为零位 |

**运动控制**

| 方法 | 参数 | 返回 | 说明 |
|------|------|------|------|
| `JointCtrl(j1..j6, joint_7=None, kp, kd, torque_ff)` | 6+1 关节角度 (rad), PD 参数 | `bool` | 单帧运控指令，状态→RUNNING |
| `JointCtrlList(positions, **kwargs)` | 6 或 7 元素列表 | `bool` | 列表形式的 JointCtrl |
| `MoveJ(positions, duration, v_max, a_max, kp, kd)` | 目标位置、时长、S-curve 参数 | `bool` | S-curve 轨迹关节运动 |
| `MoveL(target_pose, duration, n_waypoints, kp, kd)` | ArmEndPose 目标、时长 | `bool` | 笛卡尔直线运动 (IK 插值) |
| `EndPoseCtrl(x, y, z, rx, ry, rz, duration, kp, kd)` | 末端位姿 (m/rad) | `bool` | 目标位姿控制 (IK → MoveJ) |
| `CartesianVelocityCtrl(vx, vy, vz, wx, wy, wz, kp, kd)` | 笛卡尔速度 | `bool` | 实时笛卡尔速度控制 (Jacobian) |
| `GripperCtrl(gripper_angle, gripper_effort, kp, kd)` | 夹爪角度 (rad) | `bool` | L7 夹爪独立控制 |
| `ZeroTorqueMode(enable, kd=1.0, gravity_torques)` | 开关、阻尼 | `bool` | 零力矩模式 (Kp=0)，状态→ZERO_TORQUE |
| `ZeroTorqueModeWithGravity(enable, kd=1.0, update_rate=100)` | 开关、阻尼、更新率 | `bool` | 带 Pinocchio 重力补偿的后台零力矩 |

**状态查询**

| 方法 | 返回类型 | 说明 |
|------|---------|------|
| `GetArmJointMsgs()` | `ArmJointStates` | 7 关节角度 (关节坐标系, rad) |
| `GetArmJointVelocities()` | `ArmJointStates` | 7 关节速度 (rad/s) |
| `GetArmJointEfforts()` | `ArmJointStates` | 7 关节力矩 (Nm) |
| `GetArmEndPoseMsgs()` | `ArmEndPose` | 末端位姿 (Pinocchio FK) |
| `GetArmStatus()` | `ArmStatus` | 综合状态（使能/故障/模式） |
| `GetArmEnableStatus()` | `List[bool]` | 各电机使能状态 |
| `GetArmHighSpdInfoMsgs()` | `List[MotorHighSpdInfo]` | 高速反馈 (速度/位置/力矩) |
| `GetArmLowSpdInfoMsgs()` | `List[MotorLowSpdInfo]` | 低速反馈 (温度/电压) |
| `GetMotorStates()` | `Dict[int, MotorFeedback]` | 原始电机反馈 (电机坐标系) |
| `GetFirmwareVersion(motor_id)` | `FirmwareVersion` | 电机固件版本 |
| `GetMotorVoltage(motor_id)` | `float` | 母线电压 (V) |

**动力学（Pinocchio，CAN/ROS 共用）**

| 方法 | 参数 | 返回 | 说明 |
|------|------|------|------|
| `ComputeGravityTorques(positions)` | 关节角度列表，None=当前 | `List[float]` | RNEA 重力补偿力矩 |
| `GetJacobian(positions)` | 同上 | `np.ndarray (6×N)` | 末端 Jacobian 矩阵 |
| `GetMassMatrix(positions)` | 同上 | `np.ndarray (N×N)` | 质量矩阵 M(q) |
| `InverseDynamics(q, v, a)` | 关节角度/速度/加速度 | `List[float]` | RNEA 逆动力学 |
| `ForwardDynamics(q, v, tau)` | 关节角度/速度/力矩 | `List[float]` | ABA 正动力学 |
| `GetDynamicsInfo(positions)` | 同上 | `DynamicsInfo` | 完整动力学信息（重力矩、质量矩阵、Jacobian） |
| `ZeroTorqueModeWithGravity(enable, kd, update_rate)` | 开关、阻尼、更新率 | `bool` | 带 Pinocchio 重力补偿的后台零力矩 |

**参数读写**

| 方法 | 说明 |
|------|------|
| `ReadMotorParameter(motor_id, param_index)` | 读取电机参数 |
| `WriteMotorParameter(motor_id, param_index, value)` | 写入电机参数（掉电丢失） |
| `SearchMotorMaxAngleSpdAccLimit(motor_num, search_content)` | 查询角度/速度/加速度限制 |
| `GetCurrentMotorAngleLimitMaxVel()` | 获取全部电机限位配置 |
| `GetAllMotorMaxAccLimit()` | 获取全部电机最大加速度限制 |

**辅助方法**

| 方法 | 返回 | 说明 |
|------|------|------|
| `GetCurrentSDKVersion()` | `str` | SDK 版本号 |
| `GetCurrentProtocolVersion()` | `str` | 协议版本号 |
| `GetCanFps()` | `float` | CAN 帧接收频率 (Hz) |
| `GetCanName()` | `str` | CAN 接口名 |
| `SetPositionPD(kp, kd)` | — | 设置默认 PD 增益 |
| `SetJointLimitEnabled(enabled)` | — | 开关软件关节限位保护 |
| `ResetArm()` | `bool` | 复位状态机到 IDLE |

**构造函数**

```python
ELA3Interface(
    can_name: str = "can0",        # CAN 接口名
    host_can_id: int = 0xFD,       # 主机 CAN ID
    default_kp: float = 80.0,      # 默认位置 Kp
    default_kd: float = 4.0,       # 默认位置 Kd
    motor_type_map: dict = None,   # 电机 ID→型号映射（默认 1-3=RS00, 4-7=EL05）
    joint_directions: dict = None, # 关节方向
    joint_offsets: dict = None,    # 关节偏移
    urdf_path: str = None,         # URDF 路径 (Pinocchio)
    inertia_config_path: str = None, # 标定惯量参数
    log_level: LogLevel = INFO,
)
```

RS05 配置示例：

```python
from el_a3_sdk import ELA3Interface, MotorType

arm = ELA3Interface(motor_type_map={
    1: MotorType.RS00, 2: MotorType.RS00, 3: MotorType.RS00,
    4: MotorType.RS05, 5: MotorType.RS05, 6: MotorType.RS05, 7: MotorType.RS05,
})
```

#### ELA3ROSInterface (ROS Control 模式)

适用于正式控制场景，通过 ROS2 topic/action/service 与 `controller_manager` 交互。

与 `ELA3Interface` 共享同名 API，但底层实现不同：

| 区别项 | CAN 模式 | ROS 模式 |
|--------|----------|----------|
| 通信方式 | SocketCAN 帧 | ROS2 Topic/Action/Service |
| JointCtrl | 逐电机运控帧 | `/arm_controller/joint_trajectory` Topic |
| GripperCtrl | 直接电机帧 | `/gripper_controller/joint_trajectory` Topic |
| ZeroTorqueMode | Kp=0 + 可选重力线程 | `switch_controller` 服务切换 |
| MoveJ | S-curve 本地规划 | `FollowJointTrajectory` Action |
| MoveL | IK 插值 + 本地执行 | `GetCartesianPath` + `FollowJointTrajectory` |
| EndPoseCtrl | Pinocchio IK → MoveJ | MoveIt IK → MoveJ，回退 Pinocchio |
| CartesianPathCtrl | — (不支持) | MoveIt `GetCartesianPath` + `FollowJointTrajectory` |
| PlanToJointGoal | — (不支持) | MoveIt `MoveGroup` Action（同步阻塞） |
| PlanToJointGoalAsync | — (不支持) | MoveIt `MoveGroup` Action（异步回调） |
| PlanCartesianPathAsync | — (不支持) | MoveIt `GetCartesianPath` + `FollowJointTrajectory`（异步） |
| ComputeIKAsync | — (不支持) | MoveIt `/compute_ik` Service（异步 Future） |
| 参数读写 | CAN Type 17/18 | 不可用 (警告提示) |
| 固件版本 | CAN Type 26 | 不可用 |
| 末端位姿 | Pinocchio FK | TF2 查询，回退 FK |

**构造函数**

```python
ELA3ROSInterface(
    node_name: str = "el_a3_sdk_node",
    namespace: str = "",              # ROS namespace (多臂隔离)
    controller_name: str = "arm_controller",
    gripper_controller_name: str = "gripper_controller",
    urdf_path: str = None,
    inertia_config_path: str = None,
    log_level: LogLevel = INFO,
)
```

**ROS 模式独有 — MoveIt 集成方法**

| 方法 | 参数 | 返回 | 说明 |
|------|------|------|------|
| `EndPoseCtrl(x, y, z, rx, ry, rz, duration)` | 末端位姿 (m/rad)、时长 | `bool` | MoveIt IK → MoveJ，回退 Pinocchio IK |
| `CartesianPathCtrl(waypoints, eef_step, duration)` | ArmEndPose 列表、步长、时长 | `bool` | MoveIt 笛卡尔路径规划 + FollowJointTrajectory 执行 |
| `ComputeIKAsync(target_pose, seed_positions, avoid_collisions, timeout_ns)` | geometry_msgs/Pose、种子角度、碰撞检测、超时 | `Future` | 异步 MoveIt IK 求解，返回 rclpy.Future |
| `PlanToJointGoal(joint_positions, velocity_scale, accel_scale, planning_time, num_attempts, replan, replan_attempts)` | 目标角度(6-DOF)、速度/加速度缩放、规划时间等 | `bool` | 同步 MoveGroup 规划+执行（阻塞） |
| `PlanToJointGoalAsync(joint_positions, ..., result_callback)` | 同 PlanToJointGoal + 完成回调 | — | 异步 MoveGroup 规划+执行，完成后 `callback(success: bool)` |
| `PlanCartesianPathAsync(waypoints, max_step, avoid_collisions, result_callback)` | Pose 列表、步长、碰撞检测、回调 | — | 异步笛卡尔路径规划+执行 |

**ROS/CAN 共有 — 动力学接口（Pinocchio）**

| 方法 | 参数 | 返回 | 说明 |
|------|------|------|------|
| `ComputeGravityTorques(positions)` | 关节角度列表，None=当前 | `List[float]` | RNEA 重力补偿力矩 |
| `GetJacobian(positions)` | 同上 | `np.ndarray (6×N)` | 末端 Jacobian 矩阵 |
| `GetMassMatrix(positions)` | 同上 | `np.ndarray (N×N)` | 质量矩阵 M(q) |
| `GetDynamicsInfo(positions)` | 同上 | `DynamicsInfo` | 完整动力学信息（重力矩、质量矩阵、Jacobian） |
| `ZeroTorqueModeWithGravity(enable)` | 开关 | `bool` | 带重力补偿的零力矩模式（ROS 下等同 ZeroTorqueMode） |

**ROS/CAN 共有 — 辅助方法**

| 方法 | 参数 | 返回 | 说明 |
|------|------|------|------|
| `GetCurrentSDKVersion()` | — | `str` | SDK 版本号 |
| `GetCurrentProtocolVersion()` | — | `str` | 协议版本号 |
| `GetCanFps()` | — | `float` | CAN 帧率（ROS 模式返回 joint_states 频率） |
| `GetCanName()` | — | `str` | CAN 接口名 / ROS namespace |
| `SetPositionPD(kp, kd)` | Kp、Kd | — | 设置默认 PD 增益 |
| `SetJointLimitEnabled(enabled)` | bool | — | 开关软件关节限位保护 |
| `ResetArm()` | — | `bool` | 复位状态机到 IDLE |

#### ArmManager (多臂管理)

单例模式，统一管理多个 CAN/ROS 臂实例。

```python
from el_a3_sdk import ArmManager

mgr = ArmManager()

# 注册 CAN 直连臂
master = mgr.register_can_arm("master", can_name="can0")

# 注册 ROS namespace 臂
slave = mgr.register_ros_arm("slave", namespace="arm2",
                              controller_name="arm2_arm_controller")

# 从配置文件批量创建
mgr = ArmManager.from_config("config/multi_arm_config.yaml", mode="ros")

# 访问
arm = mgr.get_arm("master")   # 或 mgr["master"]
mgr.disconnect_all()
mgr.reset()                    # 销毁 Singleton
```

| 方法 | 说明 |
|------|------|
| `register_can_arm(name, can_name, **kwargs)` | 注册 CAN 模式臂，返回 `ELA3Interface` |
| `register_ros_arm(name, namespace, **kwargs)` | 注册 ROS 模式臂，返回 `ELA3ROSInterface` |
| `get_arm(name)` / `mgr[name]` | 获取已注册臂 |
| `has_arm(name)` / `name in mgr` | 检查臂是否已注册 |
| `unregister(name)` | 注销并断开 |
| `disconnect_all()` | 断开所有臂 |
| `from_config(path, mode, auto_connect)` | 从 YAML 批量创建 |
| `arm_names` / `len(mgr)` | 已注册臂列表/数量 |

#### ELA3Kinematics (运动学/动力学)

基于 Pinocchio，独立于 ROS，CAN 和 ROS 模式共用。需安装 `pinocchio` (`pip install pin` 或 `apt install ros-humble-pinocchio`)。

```python
from el_a3_sdk.kinematics import ELA3Kinematics

kin = ELA3Kinematics(urdf_path=None, ee_frame_name="end_effector")

pose = kin.forward_kinematics([0.0]*6)          # FK → ArmEndPose
q = kin.inverse_kinematics(pose, max_iter=200)   # IK → List[float] 或 None
J = kin.compute_jacobian([0.0]*6)                # 6×6 Jacobian (世界对齐)
tau_g = kin.compute_gravity([0.0]*6)             # RNEA 重力补偿力矩
tau = kin.inverse_dynamics(q, v, a)              # RNEA 逆动力学
acc = kin.forward_dynamics(q, v, tau)            # ABA 正动力学
M = kin.mass_matrix([0.0]*6)                     # CRBA 惯性矩阵 (6×6)
C = kin.coriolis_matrix(q, v)                    # 科氏力矩阵 (6×6)
```

> 运动学模块操作 6 个臂关节 (L1-L6)。L7 夹爪不参与运动链计算。

#### 轨迹规划 (`trajectory.py`)

无 ROS 依赖，CAN 和 ROS 模式共用。

```python
from el_a3_sdk.trajectory import SCurvePlanner, MultiJointPlanner, CubicSplinePlanner

# 单关节 S-curve
planner = SCurvePlanner(v_max=3.0, a_max=10.0, j_max=50.0)
profile = planner.plan(start=0.0, end=1.5)
pos, vel, acc = planner.evaluate(profile, t=0.5)
points = planner.generate_trajectory(profile, dt=0.005)

# 多关节同步
mp = MultiJointPlanner(n_joints=6, v_max=3.0, a_max=10.0)
profiles = mp.plan_sync(starts=[0]*6, ends=[0.5, 1.0, -0.5, 0, 0, 0])
traj = mp.generate_trajectory(profiles, dt=0.005)

# 三次样条多路点
waypoints = [[0]*6, [0.5, 1.0, -0.5, 0, 0, 0], [0]*6]
traj = CubicSplinePlanner.plan_waypoints(waypoints, durations=[2.0, 2.0])
```

### CAN 帧结构

29 位扩展帧 ID 编码：

```
Bit 28~24: 通信类型 (CommType, 5 bits)
Bit 23~8:  数据区2 (16 bits) — 主机 CAN ID 或前馈力矩编码
Bit 7~0:   目标地址 (8 bits) — 电机 CAN ID
```

运控模式帧 (Type 1) 数据域 (8 bytes)：

```
Byte 0-1: 目标位置 (uint16, 线性映射 ±12.57 rad)
Byte 2-3: 目标速度 (uint16, 线性映射 ±50 rad/s)
Byte 4-5: Kp        (uint16, 线性映射 0~500)
Byte 6-7: Kd        (uint16, 线性映射 0~5)
帧 ID 数据区2: τ_ff  (uint16, 线性映射 ±T_max)
```

反馈帧 (Type 2) 数据域 (8 bytes)：

```
Byte 0-1: 当前位置 (uint16)
Byte 2-3: 当前速度 (uint16)
Byte 4-5: 当前力矩 (uint16)
Byte 6-7: 温度     (uint16, ×0.1 °C)
帧 ID Bit22~23: 模式状态 (0=Reset, 1=Cali, 2=Motor)
帧 ID Bit16~21: 故障码 (6 bits)
```

uint16 线性映射公式：

```
编码: uint16 = (value - min) × 65535 / (max - min)
解码: value  = uint16 × (max - min) / 65535 + min
```

---

## 故障排除

### CAN 接口问题

```bash
lsusb | grep -i can
sudo modprobe can && sudo modprobe can_raw && sudo modprobe gs_usb
ip link show type can
sudo ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
candump can0
```

### 电机无响应

1. 检查 CAN 接线和终端电阻
2. 确认电机 ID 配置 (1-7)
3. 检查电源供电
4. 验证主机 CAN ID (默认 253/0xFD)

### 末端抖动

1. 调整 `position_kp` / `position_kd`（运行时可动态调整）
2. 增加 `smoothing_alpha`
3. 降低 `max_joint_velocity`
4. 检查机械间隙

### 编译错误

```bash
cd ros2_ws && rm -rf build install log
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

---

## 目录结构

```
EL-A3/
├── README.md
├── 电机通信协议汇总.md
│
├── el_a3_description/                     # 机器人描述包
│   ├── urdf/
│   │   ├── el_a3.urdf.xacro                  # URDF（含 L7 夹爪 + end_effector 分支）
│   │   └── el_a3_ros2_control.xacro           # ros2_control 硬件接口配置（L1-L7）
│   ├── config/
│   │   ├── el_a3_controllers.yaml             # 控制器配置（arm + gripper + zero_torque）
│   │   ├── multi_arm_config.yaml              # 多臂 CAN/namespace 配置
│   │   ├── master_slave_config.yaml           # 主从映射配置
│   │   └── inertia_params.yaml                # 标定惯量参数
│   └── launch/
│       ├── el_a3_control.launch.py            # 单臂启动
│       └── multi_arm_control.launch.py        # 多臂启动
│
├── el_a3_hardware/                        # ros2_control 硬件接口 + 控制器插件
│   ├── include/el_a3_hardware/
│   │   ├── el_a3_hardware.hpp                 # SystemInterface 头文件
│   │   ├── zero_torque_controller.hpp         # ZeroTorqueController 头文件
│   │   └── robstride_can_driver.hpp           # CAN 驱动
│   ├── src/
│   │   ├── el_a3_hardware.cpp                 # 硬件接口（Pinocchio 重力、关节限位保护）
│   │   ├── zero_torque_controller.cpp         # 零力矩控制器（Pinocchio RNEA）
│   │   └── robstride_can_driver.cpp           # CAN 通信驱动
│   ├── el_a3_hardware_plugin.xml              # 硬件接口插件描述
│   └── el_a3_controller_plugin.xml            # ZeroTorqueController 插件描述
│
├── el_a3_moveit_config/                   # MoveIt2 配置
│   ├── config/
│   │   ├── el_a3.srdf                         # SRDF（arm + gripper 规划组）
│   │   ├── moveit_controllers.yaml            # MoveIt 控制器（arm + gripper）
│   │   ├── kinematics.yaml
│   │   ├── joint_limits.yaml
│   │   └── ompl_planning.yaml
│   └── launch/
│       ├── demo.launch.py
│       └── robot.launch.py
│
├── el_a3_sdk/                             # Python SDK（双模式）
│   ├── interface.py                           # CAN 直连 API（7 关节 + ArmState）
│   ├── ros_interface.py                       # ROS Control API（7 关节 + ArmState）
│   ├── arm_manager.py                         # 多臂管理器 (Singleton)
│   ├── kinematics.py                          # Pinocchio FK/IK/Gravity/Jacobian
│   ├── trajectory.py                          # S-curve & 样条轨迹规划
│   ├── can_driver.py                          # SocketCAN 驱动
│   ├── protocol.py                            # 协议枚举 + ArmState 状态机
│   ├── data_types.py                          # 数据结构（7 关节）
│   └── demo/                                  # 示例脚本
│
├── ros2_ws/src/
│   ├── el_a3_teleop/                      # 遥操作
│   │   ├── el_a3_teleop/
│   │   │   ├── xbox_teleop_node.py            # Xbox 笛卡尔控制
│   │   │   └── master_slave_node.py           # 主从遥操作（switch_controller）
│   │   ├── config/
│   │   └── launch/
│   ├── el_a3_vision/                      # 视觉抓取
│   └── el_a3_web_ui/                      # Web 控制界面 (Flask + SDK Bridge)
│       ├── el_a3_web_ui/
│       │   ├── web_server.py                  # Flask + SocketIO 主服务
│       │   ├── sdk_bridge.py                  # SDK 桥接层（ArmManager → WebSocket）
│       │   └── ros2_bridge.py                 # 旧版 ROS2 直连桥接（保留回退）
│       ├── static/                            # 前端资源 (CSS/JS/URDF)
│       └── templates/                         # HTML 模板
│
├── scripts/                               # 工具脚本
│   ├── inertia_calibration.py                 # Pinocchio 惯量标定
│   ├── setup_can.sh                           # CAN 接口设置
│   ├── setup_multi_can.sh                     # 多 CAN 批量设置
│   ├── install_deps.sh                        # 依赖安装
│   └── start_real_xbox_control.sh             # 一键启动
│
└── hardware/                              # 硬件设计资料
    ├── mechanical/                            # 机械结构 (STEP/STL/图纸)
    └── electronics/                           # 电路板 (原理图/PCB/Gerber/BOM)
```

---

## 脚本参考

| 脚本 | 功能 | 用法 |
|------|------|------|
| `setup_can.sh` | CAN 接口设置 | `sudo ./setup_can.sh can0 1000000` |
| `setup_multi_can.sh` | 多 CAN 批量设置 | `sudo ./setup_multi_can.sh 4` |
| `install_deps.sh` | ROS2 依赖安装 | `sudo ./install_deps.sh` |
| `start_real_xbox_control.sh` | 一键启动 | `./start_real_xbox_control.sh can0` |
| `inertia_calibration.py` | 惯量标定 | `python3 inertia_calibration.py [--quick]` |
| `move_to_zero.py` | 归零 | `python3 move_to_zero.py` |

---

## 许可证

Apache-2.0

---

<br>

# EL-A3 Robotic Arm ROS2 Control System

> **EL-A3** is a 7-DOF (6 arm joints + L7 gripper) desktop robotic arm built on the standard `ros2_control` layered architecture, driven by Robstride motors over CAN bus. The system uses `controller_manager` to manage multiple controllers (position/gripper/zero-torque), Pinocchio RNEA dynamics for gravity compensation, a dual-mode Python SDK (Direct CAN + ROS Control), and supports Xbox gamepad Cartesian teleoperation, ROS namespace-based master-slave teleoperation, gravity-compensated teach mode, MoveIt2 motion planning, and multi-arm management.

---

## Control Architecture

### Layered Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Layer                        │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐  │
│  │ MoveIt2  │ │  Xbox    │ │ Master-  │ │  Python SDK   │  │
│  │ Planner  │ │  Teleop  │ │ Slave    │ │ (el_a3_sdk)   │  │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └──────┬────────┘  │
└───────┼────────────┼────────────┼───────────────┼───────────┘
        │            │            │               │
┌───────┼────────────┼────────────┼───────────────┼───────────┐
│       │     controller_manager (200 Hz)         │           │
│  ┌────▼────────────▼───┐ ┌─────▼─────┐ ┌───────▼────────┐  │
│  │   arm_controller    │ │ gripper_  │ │ zero_torque_   │  │
│  │ (JointTrajCtrl)     │ │ controller│ │ controller     │  │
│  │ L1-L6 position      │ │ L7 pos   │ │ L1-L7 effort   │  │
│  └──────────┬──────────┘ └─────┬─────┘ │ (Pinocchio     │  │
│             │                  │        │  RNEA gravity)  │  │
│             │                  │        └───────┬────────┘  │
└─────────────┼──────────────────┼────────────────┼───────────┘
┌─────────────▼──────────────────▼────────────────▼───────────┐
│               RsA3HardwareInterface                          │
│            (hardware_interface::SystemInterface)              │
│  Command: position (L1-L7), effort (L1-L7)                  │
│  State:   position (L1-L7), velocity (L1-L7)                │
│  Joint limit protection + Pinocchio gravity feedforward      │
└──────────────────────────────┬──────────────────────────────┘
                               │ CAN Bus (1 Mbps)
┌──────────────────────────────▼──────────────────────────────┐
│          Robstride Motors (7x): RS00 + EL05/RS05             │
└─────────────────────────────────────────────────────────────┘
```

### Controllers

| Controller | Type | Joints | Purpose |
|------------|------|--------|---------|
| `arm_controller` | JointTrajectoryController | L1-L6 | Arm position trajectory tracking |
| `gripper_controller` | JointTrajectoryController | L7 | Gripper position control |
| `zero_torque_controller` | el_a3_hardware/ZeroTorqueController | L1-L7 | Gravity-compensated teach mode |
| `joint_state_broadcaster` | JointStateBroadcaster | L1-L7 | State publishing |

### Mode Switching

Mode switching uses standard `controller_manager/switch_controller`:

```bash
# Position → Zero-torque (teach mode)
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['zero_torque_controller'], \
    deactivate_controllers: ['arm_controller', 'gripper_controller'], \
    strictness: 1}"
```

### TF Frames

```
base_link
└── L1_joint → ... → L6_joint → l5_l6_urdf_asm
                                  ├── end_effector_joint (fixed) → end_effector  ← MoveIt TCP
                                  └── L7_joint (revolute) → gripper_link         ← Gripper
```

### Python SDK

Dual-mode API with `ArmState` state machine:

```python
from el_a3_sdk import ArmManager

mgr = ArmManager()
arm = mgr.register_ros_arm("master", namespace="arm1")
arm.ConnectPort()      # → IDLE
arm.EnableArm()        # → ENABLED
arm.JointCtrl(...)     # → RUNNING

# MoveIt integration (ROS mode only)
arm.PlanToJointGoal([0.0]*6)                           # sync plan + execute
arm.EndPoseCtrl(0.3, 0.0, 0.3, 0.0, 0.0, 0.0)        # Cartesian pose → IK → execute
arm.PlanToJointGoalAsync([0.5]*6, result_callback=fn)  # async plan + execute

# Dynamics (Pinocchio, both modes)
tau_g = arm.ComputeGravityTorques()
J = arm.GetJacobian()

arm.ZeroTorqueMode(True)   # → ZERO_TORQUE (via switch_controller)
arm.ZeroTorqueMode(False)  # → ENABLED
```

### SDK Protocol Reference

#### ArmState Lifecycle

```
DISCONNECTED → IDLE → ENABLED → RUNNING / ZERO_TORQUE → ERROR
```

| State | Allowed Operations |
|-------|-------------------|
| `DISCONNECTED` | `ConnectPort()` |
| `IDLE` | `EnableArm()`, `DisconnectPort()` |
| `ENABLED` | `JointCtrl()`, `MoveJ()`, `MoveL()`, `EndPoseCtrl()`, `PlanToJointGoal()`, `CartesianPathCtrl()`, `GripperCtrl()`, `ZeroTorqueMode()` |
| `RUNNING` | `JointCtrl()`, `MoveJ()`, `EndPoseCtrl()`, `PlanToJointGoal()`, `GripperCtrl()`, `ZeroTorqueMode()`, `EmergencyStop()` |
| `ZERO_TORQUE` | `ZeroTorqueMode(False)`, `EmergencyStop()` |

#### Dual-Mode API Comparison

| Feature | CAN Mode (`ELA3Interface`) | ROS Mode (`ELA3ROSInterface`) |
|---------|---------------------------|-------------------------------|
| Transport | SocketCAN frames | ROS2 Topics/Actions/Services |
| JointCtrl | Per-motor motion control frame | `/arm_controller/joint_trajectory` |
| GripperCtrl | Direct motor frame | `/gripper_controller/joint_trajectory` |
| ZeroTorqueMode | Kp=0 + optional gravity thread | `switch_controller` service |
| MoveJ | Local S-curve planning | `FollowJointTrajectory` action |
| MoveL | IK interpolation + local exec | `GetCartesianPath` + action |
| EndPoseCtrl | Pinocchio IK → MoveJ | MoveIt IK → MoveJ, fallback Pinocchio |
| PlanToJointGoal | Not available | MoveIt `MoveGroup` action (sync/async) |
| CartesianPathCtrl | Not available | MoveIt `GetCartesianPath` + action |
| ComputeIKAsync | Not available | MoveIt `/compute_ik` service (async Future) |
| Dynamics | Pinocchio (local) | Pinocchio (local, same API) |
| Parameter R/W | CAN Type 17/18 | Not available |

#### Core API Methods

**Connection**: `ConnectPort()`, `DisconnectPort()`

**Control**: `EnableArm()`, `DisableArm()`, `EmergencyStop()`, `ResetArm()`, `JointCtrl(j1..j6, joint_7)`, `JointCtrlList(positions, duration_ns)`, `GripperCtrl(angle)`, `MoveJ(positions, duration)`, `MoveL(target_pose, duration)`, `EndPoseCtrl(x,y,z,rx,ry,rz,duration)`, `CartesianVelocityCtrl(vx,vy,vz,wx,wy,wz)` (CAN only), `ZeroTorqueMode(enable)`, `ZeroTorqueModeWithGravity(enable)`

**MoveIt Integration** (ROS mode only): `ComputeIKAsync(target_pose)` → Future, `PlanToJointGoal(positions, velocity_scale)` → bool (sync), `PlanToJointGoalAsync(positions, callback)` (async), `PlanCartesianPathAsync(waypoints, callback)` (async), `CartesianPathCtrl(waypoints, eef_step, duration)` → bool

**Feedback**: `GetArmJointMsgs()`, `GetArmJointVelocities()`, `GetArmJointEfforts()`, `GetArmEndPoseMsgs()`, `GetArmStatus()`, `GetArmEnableStatus()`, `GetMotorStates()` (CAN only), `GetArmHighSpdInfoMsgs()`, `GetArmLowSpdInfoMsgs()`

**Dynamics** (Pinocchio, both modes): `ComputeGravityTorques(positions)`, `GetJacobian(positions)`, `GetMassMatrix(positions)`, `GetDynamicsInfo(positions)`, `InverseDynamics(q,v,a)` (CAN), `ForwardDynamics(q,v,tau)` (CAN)

**Multi-arm**: `ArmManager` singleton — `register_can_arm()`, `register_ros_arm()`, `from_config()`, `get_arm()`, `disconnect_all()`

**Kinematics** (Pinocchio): `ELA3Kinematics` — `forward_kinematics()`, `inverse_kinematics()`, `compute_jacobian()`, `compute_gravity()`, `inverse_dynamics()`, `forward_dynamics()`, `mass_matrix()`, `coriolis_matrix()`

**Trajectory**: `SCurvePlanner`, `MultiJointPlanner`, `CubicSplinePlanner`

**Info**: `GetCurrentSDKVersion()`, `GetCurrentProtocolVersion()`, `GetCanFps()`, `SetPositionPD(kp,kd)`, `SetJointLimitEnabled(enabled)`

#### CAN Frame Encoding

29-bit extended ID: `[CommType:5][DataArea2:16][TargetID:8]`

Motion control (Type 1) data: `[pos:u16][vel:u16][Kp:u16][Kd:u16]`, torque in DataArea2.

Feedback (Type 2) data: `[pos:u16][vel:u16][torque:u16][temp:u16]`, mode/fault in ID bits.

Linear mapping: `uint16 = (val - min) * 65535 / (max - min)`

---

## Hardware Requirements

- **EL-A3 Robotic Arm** (7 Robstride motors)
- **CAN Adapter**: CANdle / gs_usb compatible
- **Power Supply**: 24V/48V DC
- **PC**: Ubuntu 22.04 x86_64
- **Optional**: Xbox controller (wired/Bluetooth)

---

## Quick Start

```bash
# Real hardware + Xbox
sudo ./scripts/setup_can.sh can0
cd ros2_ws && source install/setup.bash
ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0

# Simulation (no hardware)
ros2 launch el_a3_moveit_config demo.launch.py
```

---

## ROS2 Interfaces

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/controller_manager/switch_controller` | `SwitchController` | Mode switching (position ↔ zero-torque) |
| `/compute_ik` | `GetPositionIK` | MoveIt IK solver |
| `/compute_cartesian_path` | `GetCartesianPath` | MoveIt Cartesian path |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/arm_controller/follow_joint_trajectory` | `FollowJointTrajectory` | Arm trajectory execution |
| `/gripper_controller/follow_joint_trajectory` | `FollowJointTrajectory` | Gripper trajectory execution |
| `/move_action` | `MoveGroup` | MoveIt motion planning |

---

## License

Apache-2.0

---

**Last Updated**: 2026-02-26
