# EL-A3 Robotic Arm ROS2 Control System

> **EL-A3** is a 7-DOF (6 arm joints + L7 gripper) desktop robotic arm built on the standard `ros2_control` layered architecture, driven by Robstride motors over CAN bus. The system uses `controller_manager` to manage multiple controllers (position/gripper/zero-torque), Pinocchio RNEA dynamics for gravity compensation, a dual-mode Python SDK (Direct CAN + ROS Control), and supports Xbox gamepad Cartesian teleoperation, ROS namespace-based master-slave teleoperation, gravity-compensated teach mode, MoveIt2 motion planning, and multi-arm management.

---

## Table of Contents

- [Control Architecture](#control-architecture)
- [System Overview](#system-overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Environment](#software-environment)
- [Packages](#packages)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Control Parameters](#control-parameters)
- [ROS2 Interfaces](#ros2-interfaces)
- [Motor Communication Protocol](#motor-communication-protocol)
- [SDK Protocol & API Reference](#sdk-protocol--api-reference)
- [Troubleshooting](#troubleshooting)
- [Directory Structure](#directory-structure)

---

## Control Architecture

### Layered Architecture Overview

The system strictly follows the `ros2_control` standard layered design with clear responsibilities per layer:

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Layer                        │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌───────────────┐  │
│  │ MoveIt2  │ │  Xbox    │ │ Master-  │ │  Python SDK   │  │
│  │ Planner  │ │  Teleop  │ │ Slave    │ │ (el_a3_sdk)   │  │
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
│             │                  │        │  RNEA gravity)  │  │
│             │ position         │ pos    └───────┬────────┘  │
│             │ command          │ cmd      effort│cmd        │
│  ┌──────────▼──────────────────▼────────────────▼────────┐  │
│  │              joint_state_broadcaster                   │  │
│  │           (publishes /joint_states @ 200Hz)            │  │
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
│  Joint limit protection (per-joint clamp + warning)          │
│  Pinocchio gravity feedforward (simplified trig + RNEA)      │
│  Velocity feedforward (position diff + low-pass filter)      │
└──────────────────────────────┬──────────────────────────────┘
                               │ CAN Bus (1 Mbps)
                               │ MIT-like PD Control
┌──────────────────────────────▼──────────────────────────────┐
│                  Robstride Motors (7x)                        │
│        τ = Kp(θ_t - θ) + Kd(ω_t - ω) + τ_ff               │
│        RS00 (L1-L3) + EL05/RS05 (L4-L7)                    │
└─────────────────────────────────────────────────────────────┘
```

### Core Design Principles

| Principle | Implementation |
|-----------|---------------|
| **Hardware interface only abstracts hardware** | `RsA3HardwareInterface` only implements `read()/write()`, does not create Services/Topics |
| **Mode switching via controller switching** | Position ↔ zero-torque via `controller_manager/switch_controller` service |
| **Independent gripper controller** | `gripper_controller` (JointTrajectoryController) independently manages L7_joint |
| **Namespace auto-following** | All topics/services follow ROS namespace, supporting multi-arm deployment |
| **SDK state machine management** | `ArmState` enum manages lifecycle: DISCONNECTED → IDLE → ENABLED → RUNNING/ZERO_TORQUE |

### Controllers

| Controller | Type | Joints | Purpose |
|------------|------|--------|---------|
| `arm_controller` | JointTrajectoryController | L1-L6 | Arm position trajectory tracking |
| `gripper_controller` | JointTrajectoryController | L7 | Gripper position control |
| `zero_torque_controller` | el_a3_hardware/ZeroTorqueController | L1-L7 | Gravity-compensated zero-torque (teach mode) |
| `joint_state_broadcaster` | JointStateBroadcaster | L1-L7 | Joint state publishing |

### Mode Switching

```bash
# Position → Zero-torque (teach mode)
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['zero_torque_controller'], \
    deactivate_controllers: ['arm_controller', 'gripper_controller'], \
    strictness: 1}"

# Zero-torque → Position control
ros2 service call /controller_manager/switch_controller \
  controller_manager_msgs/srv/SwitchController \
  "{activate_controllers: ['arm_controller', 'gripper_controller'], \
    deactivate_controllers: ['zero_torque_controller'], \
    strictness: 1}"
```

### SDK Dual-Mode Architecture

```
┌───────────────────────────────────────────────┐
│                el_a3_sdk                       │
│                                               │
│  ┌─────────────────┐  ┌───────────────────┐  │
│  │  ELA3Interface   │  │ ELA3ROSInterface  │  │
│  │  (Direct CAN)    │  │ (ROS Control)     │  │
│  │  Debug/Test      │  │ Production        │  │
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
      │       Robstride Motors         │
      └───────────────────────────────┘

  ArmManager: Singleton multi-arm manager, unified register/get CAN/ROS arm instances
  ArmState: DISCONNECTED → IDLE → ENABLED → RUNNING / ZERO_TORQUE
```

Both modes share `protocol.py` (protocol enums), `data_types.py` (data structures), `kinematics.py` (Pinocchio dynamics).

---

## System Overview

### Key Features

- **Standard ros2_control architecture**: controller_manager manages multiple controllers, hardware interface strictly only does read/write
- **Independent zero-torque controller**: Custom `ZeroTorqueController` plugin, Pinocchio RNEA gravity-compensated teach mode
- **Independent gripper controller**: L7 uses independent `gripper_controller`, decoupled from arm controller
- **Pinocchio dynamics**: Full RNEA gravity compensation (C++ hardware layer + Python SDK layer), supports calibrated inertia parameter loading
- **Joint limit protection**: Hardware layer per-joint soft limit clamping + warning logs
- **Real-time Cartesian control**: 50Hz Xbox gamepad Cartesian space teleoperation
- **MoveIt2 integration**: Motion planning, IK solving, Cartesian path planning
- **Multi-arm management**: ROS namespace isolation + `ArmManager` singleton unified management
- **Dual-mode Python SDK**: Direct CAN (debug) + ROS Control (production), 7-joint unified
- **ArmState state machine**: SDK lifecycle management, automatic state validation before method calls
- **Velocity feedforward + multi-level smoothing**: Position difference feedforward + low-pass filter + acceleration limit

### Arm Specifications

| Property | Specification |
|----------|--------------|
| DOF | 7 DOF (6 arm joints + L7 gripper) |
| End Effector | Customizable |
| Protocol | Robstride proprietary (CAN 2.0, 29-bit extended frame) |
| Baud Rate | 1Mbps |
| Control Mode | Motion control (MIT-like PD Control) |

### Motor Configuration

Joints 4-7 support both EL05 and RS05 motor types, switchable via `wrist_motor_type` parameter (default EL05).

| Joint | Motor ID | Type | Torque Limit | Velocity Limit | Position Limit | Direction |
|-------|----------|------|-------------|----------------|----------------|-----------|
| L1_joint | 1 | RS00 | ±14 Nm | ±33 rad/s | ±2.79 rad (±160°) | -1 |
| L2_joint | 2 | RS00 | ±14 Nm | ±33 rad/s | 0~3.67 rad (0°~210°) | +1 |
| L3_joint | 3 | RS00 | ±14 Nm | ±33 rad/s | -4.01~0 rad (-230°~0°) | -1 |
| L4_joint | 4 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |
| L5_joint | 5 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | -1 |
| L6_joint | 6 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |
| L7_joint (gripper) | 7 | EL05/RS05 | ±6/±5.5 Nm | ±50 rad/s | ±1.57 rad (±90°) | +1 |

**EL05 vs RS05 Comparison**:

| Parameter | EL05 | RS05 |
|-----------|------|------|
| Peak Torque | 6.0 Nm | 5.5 Nm |
| Rated Torque | 1.8 Nm | 1.6 Nm |
| Gear Ratio | 9:1 | 7.75:1 |
| Max Current | 10 Apk | 11 Apk |

---

## Hardware Requirements

### Required

- **EL-A3 Robotic Arm** (7 Robstride motors)
- **CAN Adapter**: CANdle / gs_usb compatible
- **Power Supply**: 24V/48V DC
- **PC**: Ubuntu 22.04 x86_64

### Optional

- **Xbox Controller**: Xbox One / Xbox Series X|S / XInput compatible (wired or Bluetooth)

---

## Software Environment

### System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble Hawksbill
- **Kernel Module**: `gs_usb`

### Dependency Installation

```bash
cd scripts && sudo ./install_deps.sh
```

Or install manually:

```bash
# ROS2 Control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-hardware-interface ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster ros-humble-joint-trajectory-controller

# MoveIt2
sudo apt install ros-humble-moveit ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-planning-interface ros-humble-moveit-ros-visualization \
  ros-humble-moveit-planners-ompl ros-humble-moveit-kinematics

# Tools
sudo apt install ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-joy can-utils

# Pinocchio dynamics library (required for el_a3_hardware compilation, RNEA gravity compensation)
sudo apt install ros-humble-pinocchio

# Python
pip3 install python-can scipy
```

---

## Packages

| Package | Description |
|---------|-------------|
| `el_a3_hardware` | ros2_control hardware interface + CAN driver + ZeroTorqueController plugin |
| `el_a3_description` | URDF description (7 joints), controller config, launch files |
| `el_a3_moveit_config` | MoveIt2 motion planning config (arm + gripper planning groups) |
| `el_a3_teleop` | Xbox gamepad Cartesian control + ROS namespace master-slave teleoperation |
| `el_a3_sdk` | Python SDK -- Direct CAN + ROS Control dual-mode, ArmManager, MoveIt integration, Pinocchio dynamics |
| `el_a3_web_ui` | Flask + SocketIO web control interface via SDK Bridge |

### Python SDK (`el_a3_sdk`)

```python
from el_a3_sdk import ArmManager

mgr = ArmManager()

# Register ROS namespace mode arm
master = mgr.register_ros_arm("master", namespace="arm1",
    controller_name="arm1_arm_controller")
master.ConnectPort()
master.EnableArm()

# Joint control (7 joints)
master.JointCtrl(0.0, 1.57, -0.78, 0.0, 0.0, 0.0, joint_7=0.5)

# Gripper control (via gripper_controller)
master.GripperCtrl(gripper_angle=0.3)

# MoveIt planning (ROS mode only)
master.PlanToJointGoal([0.0]*6, velocity_scale=0.3)          # sync joint planning
master.EndPoseCtrl(0.3, 0.0, 0.3, 0.0, 0.0, 0.0)            # Cartesian pose control
master.PlanToJointGoalAsync([0.5]*6, result_callback=print)   # async joint planning

# Dynamics (Pinocchio)
tau_g = master.ComputeGravityTorques()                         # gravity compensation torques
J = master.GetJacobian()                                       # Jacobian matrix

# Zero-torque mode (via switch_controller)
master.ZeroTorqueMode(True)   # activate zero_torque_controller
master.ZeroTorqueMode(False)  # restore arm_controller + gripper_controller

# Status query
print(master.arm_state)           # ArmState.ENABLED / RUNNING / ZERO_TORQUE / ...
print(master.GetArmEndPoseMsgs()) # ArmEndPose(x=..., y=..., z=..., rx=..., ry=..., rz=...)
```

Installation:

```bash
cd el_a3_sdk
pip install -e .            # basic install
pip install -e ".[dynamics]"  # with Pinocchio support
```

---

## Installation

### 1. Build Workspace

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Setup CAN Interface

```bash
sudo ./scripts/setup_can.sh can0 1000000
ip link show can0
candump can0
```

---

## Quick Start

### Xbox Gamepad Real-Time Control (Recommended)

```bash
# Terminal 1: Setup CAN
sudo ./scripts/setup_can.sh can0

# Terminal 2: Launch control system
cd ros2_ws && source install/setup.bash
ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0

# For RS05 motors (joints 4-7), add parameter:
# ros2 launch el_a3_teleop real_teleop.launch.py can_interface:=can0 wrist_motor_type:=RS05
```

Or one-click launch: `./scripts/start_real_xbox_control.sh can0`

**Gamepad Control Mapping**:

| Button/Stick | Function |
|-------------|----------|
| Left Stick | XY translation |
| LT/RT | Z-axis up/down |
| Right Stick | Yaw/Pitch rotation |
| LB/RB | Roll rotation |
| A Button | Toggle speed level (5 levels) |
| B Button | Return to home position |
| X Button | Return to zero position |
| Y Button | Toggle zero-torque mode (switch_controller teach mode) |
| Menu Button | Toggle master-slave teleoperation |
| D-Pad Up/Down | Gripper close/open (via gripper_controller) |

### Master-Slave Teleoperation

```bash
# Setup dual CAN
sudo ./scripts/setup_can.sh can0 1000000
sudo ./scripts/setup_can.sh can1 1000000

# Launch multi-arm control
ros2 launch el_a3_description multi_arm_control.launch.py

# Launch teleoperation
ros2 launch el_a3_teleop real_teleop.launch.py \
  master_namespace:=arm1 slave_namespace:=arm2 \
  master_controller:=arm1_arm_controller slave_controller:=arm2_arm_controller
```

Press Menu button to toggle: master arm activates `zero_torque_controller` (draggable), slave arm follows in real-time via ROS trajectory topic.

### Simulation Mode

```bash
# MoveIt Demo (no hardware required)
ros2 launch el_a3_moveit_config demo.launch.py

# Simulation + Xbox gamepad
ros2 launch el_a3_teleop sim_teleop.launch.py
```

### Real Hardware + MoveIt

```bash
sudo ./scripts/setup_can.sh can0
ros2 launch el_a3_moveit_config robot.launch.py can_interface:=can0
```

---

## Control Parameters

### Hardware Interface Parameters (`el_a3_ros2_control.xacro`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | can0 | CAN interface name |
| `host_can_id` | 253 (0xFD) | Host CAN ID |
| `wrist_motor_type` | EL05 | Wrist motor type (EL05 or RS05), affects L4-L7 torque mapping |
| `position_kp` | 80.0 | Position PD control Kp gain |
| `position_kd` | 4.0 | Position PD control Kd gain |
| `velocity_limit` | 10.0 | Velocity limit (rad/s) |
| `smoothing_alpha` | 0.08 | Low-pass filter coefficient (0-1) |
| `use_pinocchio_gravity` | true | Enable Pinocchio RNEA gravity compensation |
| `gravity_feedforward_ratio` | 0.5 | Normal mode gravity feedforward ratio |
| `zero_torque_kd` | 1.0 | Zero-torque mode global damping |
| `limit_margin` | 0.15 | Joint limit deceleration zone (rad) |
| `limit_stop_margin` | 0.02 | Joint limit hard stop zone (rad) |
| `can_frame_delay_us` | 50 | CAN frame inter-delay (us) |

**Dynamic parameters** (adjustable at runtime via ROS2 parameter service): `position_kp`, `position_kd`, `gravity_feedforward_ratio`, `zero_torque_kd`, `smoothing_alpha`

### Gravity Compensation System

The system uses the **Pinocchio RNEA full dynamics model** for gravity compensation:

```
τ_gravity = RNEA(model, data, q, v=0, a=0)
```

Gravity compensation runs at two levels:

| Level | Controller | Compensation Ratio | Purpose |
|-------|-----------|-------------------|---------|
| Hardware interface `write()` | When `arm_controller` active | `gravity_feedforward_ratio` (50%) | Normal position control feedforward |
| ZeroTorqueController | When `zero_torque_controller` active | 100% (full RNEA) | Teach mode |

**Inertia parameter calibration**: Auto-calibrate L2-L6 mass and COM parameters via `scripts/inertia_calibration.py`, saved to `el_a3_description/config/inertia_params.yaml`.

```bash
python3 scripts/inertia_calibration.py          # full calibration (~10min)
python3 scripts/inertia_calibration.py --quick   # quick calibration (~3min)
```

### Zero-Torque Mode (Teach Mode)

Zero-torque mode is implemented via **controller switching** (standard ros2_control pattern):

1. `controller_manager` deactivates `arm_controller` + `gripper_controller`
2. Activates `zero_torque_controller` (custom ControllerInterface plugin)
3. `zero_torque_controller` claims effort command interfaces, reads joint positions, computes gravity torques via Pinocchio RNEA and writes them
4. Hardware interface detects effort_mode, sets Kp=0, Kd=damping, τ_ff=effort_cmd

**SDK / Teleop usage**:

```python
arm.ZeroTorqueMode(True)   # internally calls switch_controller
arm.ZeroTorqueMode(False)  # restores position controllers
```

### Controller Parameters (`el_a3_controllers.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_rate` | 200 Hz | controller_manager update frequency |
| `arm_controller` joints | L1-L6 | Arm joint position trajectory control |
| `gripper_controller` joints | L7 | Gripper position control |
| `zero_torque_controller` joints | L1-L7 | Zero-torque (effort mode) |
| `zero_torque_controller` kd | 1.0 | Zero-torque damping coefficient |

### Xbox Teleop Parameters (`xbox_teleop.yaml`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_rate` | 50.0 Hz | Control loop frequency |
| `use_fast_ik_mode` | true | Fast IK mode |
| `max_linear_velocity` | 0.15 m/s | Max linear velocity |
| `max_angular_velocity` | 1.5 rad/s | Max angular velocity |
| `joint_smoothing_alpha` | 0.15 | Joint output smoothing coefficient |
| `max_joint_velocity` | 1.5 rad/s | Single joint max velocity |
| `max_ik_jump_threshold` | 0.5 rad | IK jump threshold |
| `enable_collision_check` | true | Collision detection |

---

## ROS2 Interfaces

### Topics

#### Published

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | 200 Hz | L1-L7 joint states |
| `/robot_description` | `std_msgs/String` | latched | URDF description |
| `/target_pose` | `geometry_msgs/PoseStamped` | 50 Hz | Target end-effector pose |
| `/debug/hw_command` | `sensor_msgs/JointState` | 20 Hz | Controller command positions |
| `/debug/smoothed_command` | `sensor_msgs/JointState` | 20 Hz | Smoothed motor commands |
| `/debug/gravity_torque` | `sensor_msgs/JointState` | 20 Hz | Gravity compensation torques |
| `/debug/motor_temperature` | `sensor_msgs/JointState` | 4 Hz | Motor temperatures |
| `/zero_torque_controller/gravity_torques` | `sensor_msgs/JointState` | 200 Hz | ZeroTorqueController gravity torques |

#### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Xbox gamepad input |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Arm joint trajectory commands |
| `/gripper_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Gripper trajectory commands |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/controller_manager/switch_controller` | `controller_manager_msgs/SwitchController` | Controller switching (position ↔ zero-torque) |
| `/controller_manager/list_controllers` | `controller_manager_msgs/ListControllers` | List controller status |
| `/compute_ik` | `moveit_msgs/GetPositionIK` | MoveIt IK solver |
| `/compute_cartesian_path` | `moveit_msgs/GetCartesianPath` | MoveIt Cartesian path |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Arm trajectory execution |
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Gripper trajectory execution |
| `/move_action` | `moveit_msgs/MoveGroup` | MoveIt motion planning |
| `/execute_trajectory` | `moveit_msgs/ExecuteTrajectory` | MoveIt trajectory execution |

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
                            └── L7_joint (revolute) → gripper_link         ← Gripper
```

> `end_effector` and `gripper_link` are siblings under `l5_l6_urdf_asm`. MoveIt `arm` planning group kinematic chain terminates at `end_effector`, `gripper` planning group contains `L7_joint`.

### MoveIt Planning Groups

| Group | Type | Content |
|-------|------|---------|
| `arm` | chain | base_link → end_effector (L1-L6) |
| `gripper` | joint | L7_joint |

---

## Motor Communication Protocol

The system uses the Robstride proprietary protocol with **motion control mode (MIT-like)** for real-time control:

```
τ = Kp × (θ_target - θ_actual) + Kd × (ω_target - ω_actual) + τ_ff
```

| Parameter | Range | Description |
|-----------|-------|-------------|
| θ_target | ±12.57 rad | Target position |
| ω_target | See motor specs | Velocity feedforward |
| Kp | 0~500 | Position stiffness |
| Kd | 0~5 | Damping coefficient |
| τ_ff | See motor specs | Feedforward torque (gravity compensation) |

| Comm Type | Function |
|-----------|----------|
| 1 | Motion control (position/velocity/Kp/Kd/torque) |
| 2 | Motor feedback (position/velocity/torque/temperature) |
| 3 | Motor enable |
| 4 | Motor stop |
| 6 | Set zero position |
| 18 | Parameter write |

### CAN Frame Encoding

29-bit extended frame ID encoding:

```
Bit 28~24: CommType (5 bits)
Bit 23~8:  DataArea2 (16 bits) — Host CAN ID or feedforward torque encoding
Bit 7~0:   Target Address (8 bits) — Motor CAN ID
```

Motion control frame (Type 1) data field (8 bytes):

```
Byte 0-1: Target position (uint16, linear mapping ±12.57 rad)
Byte 2-3: Target velocity (uint16, linear mapping ±50 rad/s)
Byte 4-5: Kp             (uint16, linear mapping 0~500)
Byte 6-7: Kd             (uint16, linear mapping 0~5)
Frame ID DataArea2: τ_ff  (uint16, linear mapping ±T_max)
```

Feedback frame (Type 2) data field (8 bytes):

```
Byte 0-1: Current position (uint16)
Byte 2-3: Current velocity (uint16)
Byte 4-5: Current torque   (uint16)
Byte 6-7: Temperature      (uint16, ×0.1 °C)
Frame ID Bit22~23: Mode status (0=Reset, 1=Cali, 2=Motor)
Frame ID Bit16~21: Fault code (6 bits)
```

uint16 linear mapping formula:

```
Encode: uint16 = (value - min) × 65535 / (max - min)
Decode: value  = uint16 × (max - min) / 65535 + min
```

---

## SDK Protocol & API Reference

### Module Structure

```
el_a3_sdk/
├── __init__.py          # Package entry, lazy import for ROS/Pinocchio optional deps
├── protocol.py          # Protocol enums, motor params, joint config constants
├── data_types.py        # Data structures (SI units: rad, Nm, m)
├── can_driver.py        # SocketCAN low-level driver (frame I/O + background receive thread)
├── interface.py         # ELA3Interface — Direct CAN API
├── ros_interface.py     # ELA3ROSInterface — ROS Control API
├── arm_manager.py       # ArmManager — Singleton multi-arm manager
├── kinematics.py        # Pinocchio FK/IK/Jacobian/Gravity/Dynamics
├── trajectory.py        # S-curve 7-segment + cubic spline trajectory planning
├── utils.py             # float⇆uint16 mapping, unit conversion, clamp
└── demo/                # Example scripts
```

> **Full API Reference**: See [`el_a3_sdk/docs/SDK_API_Protocol.md`](el_a3_sdk/docs/SDK_API_Protocol.md) for all API signatures, parameter descriptions, protocol enums, data structures, CAN frame formats, and usage examples.

### Protocol Enums (`protocol.py`)

#### CAN Communication Types (`CommType`)

| Value | Name | Direction | Description |
|-------|------|-----------|-------------|
| 0 | `GET_DEVICE_ID` | Host→Motor | Get device ID |
| 1 | `MOTION_CONTROL` | Host→Motor | Motion control command (PD + τ_ff) |
| 2 | `FEEDBACK` | Motor→Host | Motor feedback (position/velocity/torque/temperature) |
| 3 | `ENABLE` | Host→Motor | Enable motor |
| 4 | `DISABLE` | Host→Motor | Stop motor (optional fault clear) |
| 6 | `SET_ZERO` | Host→Motor | Set current position as zero |
| 7 | `SET_CAN_ID` | Host→Motor | Change motor CAN ID |
| 17 | `READ_PARAM` | Bidirectional | Parameter read (request + response) |
| 18 | `WRITE_PARAM` | Host→Motor | Parameter write (lost on power-off) |
| 21 | `FAULT_FEEDBACK` | Motor→Host | Detailed fault code |
| 22 | `SAVE_PARAMS` | Host→Motor | Save parameters to Flash |
| 26 | `READ_VERSION` | Bidirectional | Firmware version query |

#### Motor Types (`MotorType`)

| Type | Value | Joints | Torque Range | Velocity Range | Position Range |
|------|-------|--------|-------------|----------------|----------------|
| `RS00` | 0 | L1-L3 | ±14 Nm | ±33 rad/s | ±12.57 rad |
| `EL05` | 1 | L4-L7 (Config A) | ±6 Nm | ±50 rad/s | ±12.57 rad |
| `RS05` | 2 | L4-L7 (Config B) | ±5.5 Nm | ±50 rad/s | ±12.57 rad |

#### Run Modes (`RunMode`)

| Value | Name | Description |
|-------|------|-------------|
| 0 | `MOTION_CONTROL` | Motion control mode (PD + feedforward torque), system default |
| 1 | `POSITION_PP` | Position mode (PP, trapezoidal profile) |
| 2 | `VELOCITY` | Velocity mode |
| 3 | `CURRENT` | Current mode |
| 5 | `POSITION_CSP` | Position mode (CSP, continuous position) |

#### State Machine (`ArmState`)

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

| State | Value | Allowed Operations |
|-------|-------|-------------------|
| `DISCONNECTED` | 0 | `ConnectPort()` |
| `IDLE` | 1 | `EnableArm()`, `DisconnectPort()` |
| `ENABLED` | 2 | `JointCtrl()`, `MoveJ()`, `MoveL()`, `EndPoseCtrl()`, `CartesianPathCtrl()`, `PlanToJointGoal()`, `GripperCtrl()`, `ZeroTorqueMode()`, `DisableArm()` |
| `RUNNING` | 3 | `JointCtrl()`, `MoveJ()`, `MoveL()`, `EndPoseCtrl()`, `CartesianPathCtrl()`, `PlanToJointGoal()`, `GripperCtrl()`, `ZeroTorqueMode()`, `EmergencyStop()` |
| `ZERO_TORQUE` | 4 | `ZeroTorqueMode(False)`, `EmergencyStop()` |
| `ERROR` | 5 | `EmergencyStop()`, `DisconnectPort()` |

#### Default Joint Configuration

L4-L7 default to EL05, switchable to RS05 via `wrist_motor_type` parameter.

| Joint | Motor ID | Default Type | Direction | Offset | Limits (rad) |
|-------|----------|-------------|-----------|--------|--------------|
| L1 | 1 | RS00 | -1 | 0.0 | -2.793 ~ +2.793 |
| L2 | 2 | RS00 | +1 | 0.0 | 0.0 ~ +3.665 |
| L3 | 3 | RS00 | -1 | 0.0 | -4.014 ~ 0.0 |
| L4 | 4 | EL05/RS05 | +1 | 0.0 | -1.571 ~ +1.571 |
| L5 | 5 | EL05/RS05 | -1 | 0.0 | -1.571 ~ +1.571 |
| L6 | 6 | EL05/RS05 | +1 | 0.0 | -1.571 ~ +1.571 |
| L7 | 7 | EL05/RS05 | +1 | 0.0 | -1.571 ~ +1.571 |

#### Parameter Index (`ParamIndex`)

| Index | Name | Description |
|-------|------|-------------|
| `0x7005` | `RUN_MODE` | Run mode switch |
| `0x7006` | `IQ_REF` | Current command |
| `0x700A` | `SPD_REF` | Velocity command |
| `0x700B` | `LIMIT_TORQUE` | Torque limit |
| `0x7016` | `LOC_REF` | CSP position command |
| `0x7017` | `LIMIT_SPD` | CSP velocity limit |
| `0x7019` | `MECH_POS` | Mechanical position read |
| `0x701A` | `IQF` | Filtered current read |
| `0x701B` | `MECH_VEL` | Mechanical velocity read |
| `0x701C` | `VBUS` | Bus voltage read |
| `0x701E` | `LOC_KP` | Position loop Kp |
| `0x701F` | `SPD_KP` | Velocity loop Kp |

### Data Structures (`data_types.py`)

All data uses SI units (rad, rad/s, Nm, m, °C).

| Structure | Description | Key Fields |
|-----------|-------------|------------|
| `MotorFeedback` | Single motor feedback (Type 2) | `motor_id`, `position`, `velocity`, `torque`, `temperature`, `fault_code` |
| `ArmJointStates` | 7-joint state | `joint_1`~`joint_7`, `to_list(include_gripper=True/False)` |
| `ArmEndPose` | End-effector pose | `x`, `y`, `z` (m), `rx`, `ry`, `rz` (rad, XYZ intrinsic) |
| `ArmStatus` | Arm composite status | `ctrl_mode`, `arm_status`, `joint_enabled[]`, `joint_faults[]` |
| `MotorHighSpdInfo` | High-speed feedback | `speed`, `current`, `position`, `torque` |
| `MotorLowSpdInfo` | Low-speed feedback | `voltage`, `motor_temp`, `fault_code` |
| `ParamReadResult` | Parameter read result | `param_index`, `value`, `success` |
| `FirmwareVersion` | Firmware version | `version_str` (e.g. "1.2.3.4.5") |
| `DynamicsInfo` | Dynamics info | `gravity_torques`, `mass_matrix`, `jacobian` |
| `TrajectoryResult` | Trajectory execution result | `success`, `error_code`, `actual_positions` |

### API Reference

#### ELA3Interface (Direct CAN Mode)

For debugging, calibration, and standalone control without ROS. Communicates directly with motors via SocketCAN.

**Connection Management**

| Method | Parameters | Returns | Description |
|--------|-----------|---------|-------------|
| `ConnectPort()` | — | `bool` | Open CAN socket, start I/O threads, state→IDLE |
| `DisconnectPort()` | — | — | Stop threads, close socket, state→DISCONNECTED |

**Motor Control**

| Method | Parameters | Returns | Description |
|--------|-----------|---------|-------------|
| `EnableArm(motor_num=7, run_mode=MOTION_CONTROL, startup_kd=4.0)` | Motor count, run mode, startup damping | `bool` | disable→set_mode→enable sequence, state→ENABLED |
| `DisableArm(motor_num=7)` | Motor count | `bool` | Disable all motors, state→IDLE |
| `EmergencyStop()` | — | `bool` | Immediately disable all motors and clear faults, state→IDLE |
| `SetZeroPosition(motor_num=7)` | — | `bool` | Set current position as zero |

**Motion Control**

| Method | Parameters | Returns | Description |
|--------|-----------|---------|-------------|
| `JointCtrl(j1..j6, joint_7=None, kp, kd, torque_ff)` | 6+1 joint angles (rad), PD params | `bool` | Single frame motion command, state→RUNNING |
| `JointCtrlList(positions, **kwargs)` | 6 or 7 element list | `bool` | List-form JointCtrl |
| `MoveJ(positions, duration, v_max, a_max, kp, kd)` | Target positions, duration, S-curve params | `bool` | S-curve trajectory joint motion |
| `MoveL(target_pose, duration, n_waypoints, kp, kd)` | ArmEndPose target, duration | `bool` | Cartesian linear motion (IK interpolation) |
| `EndPoseCtrl(x, y, z, rx, ry, rz, duration, kp, kd)` | End-effector pose (m/rad) | `bool` | Target pose control (IK → MoveJ) |
| `CartesianVelocityCtrl(vx, vy, vz, wx, wy, wz, kp, kd)` | Cartesian velocity | `bool` | Real-time Cartesian velocity control (Jacobian) |
| `GripperCtrl(gripper_angle, gripper_effort, kp, kd)` | Gripper angle (rad) | `bool` | L7 gripper independent control |
| `ZeroTorqueMode(enable, kd=1.0, gravity_torques)` | On/off, damping | `bool` | Zero-torque mode (Kp=0), state→ZERO_TORQUE |
| `ZeroTorqueModeWithGravity(enable, kd=1.0, update_rate=100)` | On/off, damping, update rate | `bool` | Background zero-torque with Pinocchio gravity compensation |

**Status Query**

| Method | Returns | Description |
|--------|---------|-------------|
| `GetArmJointMsgs()` | `ArmJointStates` | 7 joint angles (joint frame, rad) |
| `GetArmJointVelocities()` | `ArmJointStates` | 7 joint velocities (rad/s) |
| `GetArmJointEfforts()` | `ArmJointStates` | 7 joint torques (Nm) |
| `GetArmEndPoseMsgs()` | `ArmEndPose` | End-effector pose (Pinocchio FK) |
| `GetArmStatus()` | `ArmStatus` | Composite status (enable/fault/mode) |
| `GetArmEnableStatus()` | `List[bool]` | Per-motor enable status |
| `GetArmHighSpdInfoMsgs()` | `List[MotorHighSpdInfo]` | High-speed feedback (velocity/position/torque) |
| `GetArmLowSpdInfoMsgs()` | `List[MotorLowSpdInfo]` | Low-speed feedback (temperature/voltage) |
| `GetMotorStates()` | `Dict[int, MotorFeedback]` | Raw motor feedback (motor frame) |
| `GetFirmwareVersion(motor_id)` | `FirmwareVersion` | Motor firmware version |
| `GetMotorVoltage(motor_id)` | `float` | Bus voltage (V) |

**Dynamics (Pinocchio, shared by CAN/ROS)**

| Method | Parameters | Returns | Description |
|--------|-----------|---------|-------------|
| `ComputeGravityTorques(positions)` | Joint angle list, None=current | `List[float]` | RNEA gravity compensation torques |
| `GetJacobian(positions)` | Same | `np.ndarray (6×N)` | End-effector Jacobian matrix |
| `GetMassMatrix(positions)` | Same | `np.ndarray (N×N)` | Mass matrix M(q) |
| `InverseDynamics(q, v, a)` | Joint angles/velocities/accelerations | `List[float]` | RNEA inverse dynamics |
| `ForwardDynamics(q, v, tau)` | Joint angles/velocities/torques | `List[float]` | ABA forward dynamics |
| `GetDynamicsInfo(positions)` | Same | `DynamicsInfo` | Full dynamics info (gravity, mass matrix, Jacobian) |

**Parameter Read/Write**

| Method | Description |
|--------|-------------|
| `ReadMotorParameter(motor_id, param_index)` | Read motor parameter |
| `WriteMotorParameter(motor_id, param_index, value)` | Write motor parameter (lost on power-off) |
| `SearchMotorMaxAngleSpdAccLimit(motor_num, search_content)` | Query angle/velocity/acceleration limits |
| `GetCurrentMotorAngleLimitMaxVel()` | Get all motor position limit configuration |
| `GetAllMotorMaxAccLimit()` | Get all motor max acceleration limits |

**Helper Methods**

| Method | Returns | Description |
|--------|---------|-------------|
| `GetCurrentSDKVersion()` | `str` | SDK version string |
| `GetCurrentProtocolVersion()` | `str` | Protocol version string |
| `GetCanFps()` | `float` | CAN frame receive rate (Hz) |
| `GetCanName()` | `str` | CAN interface name |
| `SetPositionPD(kp, kd)` | — | Set default PD gains |
| `SetJointLimitEnabled(enabled)` | — | Toggle software joint limit protection |
| `ResetArm()` | `bool` | Reset state machine to IDLE |

**Constructor**

```python
ELA3Interface(
    can_name: str = "can0",        # CAN interface name
    host_can_id: int = 0xFD,       # Host CAN ID
    default_kp: float = 80.0,      # Default position Kp
    default_kd: float = 4.0,       # Default position Kd
    motor_type_map: dict = None,   # Motor ID→type mapping (default 1-3=RS00, 4-7=EL05)
    joint_directions: dict = None, # Joint directions
    joint_offsets: dict = None,    # Joint offsets
    urdf_path: str = None,         # URDF path (Pinocchio)
    inertia_config_path: str = None, # Calibrated inertia parameters
    log_level: LogLevel = INFO,
)
```

RS05 configuration example:

```python
from el_a3_sdk import ELA3Interface, MotorType

arm = ELA3Interface(motor_type_map={
    1: MotorType.RS00, 2: MotorType.RS00, 3: MotorType.RS00,
    4: MotorType.RS05, 5: MotorType.RS05, 6: MotorType.RS05, 7: MotorType.RS05,
})
```

#### ELA3ROSInterface (ROS Control Mode)

For production control scenarios, interacts with `controller_manager` via ROS2 topics/actions/services.

Shares the same API names as `ELA3Interface`, but with different underlying implementations:

| Aspect | CAN Mode | ROS Mode |
|--------|----------|----------|
| Transport | SocketCAN frames | ROS2 Topics/Actions/Services |
| JointCtrl | Per-motor motion frame | `/arm_controller/joint_trajectory` Topic |
| GripperCtrl | Direct motor frame | `/gripper_controller/joint_trajectory` Topic |
| ZeroTorqueMode | Kp=0 + optional gravity thread | `switch_controller` service |
| MoveJ | Local S-curve planning | `FollowJointTrajectory` Action |
| MoveL | IK interpolation + local exec | `GetCartesianPath` + `FollowJointTrajectory` |
| EndPoseCtrl | Pinocchio IK → MoveJ | MoveIt IK → MoveJ, fallback Pinocchio |
| CartesianPathCtrl | — (not supported) | MoveIt `GetCartesianPath` + `FollowJointTrajectory` |
| PlanToJointGoal | — (not supported) | MoveIt `MoveGroup` Action (sync blocking) |
| PlanToJointGoalAsync | — (not supported) | MoveIt `MoveGroup` Action (async callback) |
| PlanCartesianPathAsync | — (not supported) | MoveIt `GetCartesianPath` + `FollowJointTrajectory` (async) |
| ComputeIKAsync | — (not supported) | MoveIt `/compute_ik` Service (async Future) |
| Parameter R/W | CAN Type 17/18 | Not available (warning) |
| Firmware Version | CAN Type 26 | Not available |
| End-effector Pose | Pinocchio FK | TF2 lookup, fallback FK |

**Constructor**

```python
ELA3ROSInterface(
    node_name: str = "el_a3_sdk_node",
    namespace: str = "",              # ROS namespace (multi-arm isolation)
    controller_name: str = "arm_controller",
    gripper_controller_name: str = "gripper_controller",
    urdf_path: str = None,
    inertia_config_path: str = None,
    log_level: LogLevel = INFO,
)
```

**ROS Mode Exclusive -- MoveIt Integration Methods**

| Method | Parameters | Returns | Description |
|--------|-----------|---------|-------------|
| `EndPoseCtrl(x, y, z, rx, ry, rz, duration)` | End-effector pose (m/rad), duration | `bool` | MoveIt IK → MoveJ, fallback Pinocchio IK |
| `CartesianPathCtrl(waypoints, eef_step, duration)` | ArmEndPose list, step size, duration | `bool` | MoveIt Cartesian path planning + FollowJointTrajectory execution |
| `ComputeIKAsync(target_pose, seed_positions, avoid_collisions, timeout_ns)` | geometry_msgs/Pose, seed angles, collision check, timeout | `Future` | Async MoveIt IK solve, returns rclpy.Future |
| `PlanToJointGoal(joint_positions, velocity_scale, accel_scale, planning_time, num_attempts, replan, replan_attempts)` | Target angles (6-DOF), velocity/accel scaling, planning time, etc. | `bool` | Sync MoveGroup plan+execute (blocking) |
| `PlanToJointGoalAsync(joint_positions, ..., result_callback)` | Same as PlanToJointGoal + completion callback | — | Async MoveGroup plan+execute, `callback(success: bool)` on completion |
| `PlanCartesianPathAsync(waypoints, max_step, avoid_collisions, result_callback)` | Pose list, step size, collision check, callback | — | Async Cartesian path plan+execute |

**ROS/CAN Shared -- Dynamics (Pinocchio)**

| Method | Parameters | Returns | Description |
|--------|-----------|---------|-------------|
| `ComputeGravityTorques(positions)` | Joint angle list, None=current | `List[float]` | RNEA gravity compensation torques |
| `GetJacobian(positions)` | Same | `np.ndarray (6×N)` | End-effector Jacobian matrix |
| `GetMassMatrix(positions)` | Same | `np.ndarray (N×N)` | Mass matrix M(q) |
| `GetDynamicsInfo(positions)` | Same | `DynamicsInfo` | Full dynamics info (gravity, mass matrix, Jacobian) |
| `ZeroTorqueModeWithGravity(enable)` | On/off | `bool` | Gravity-compensated zero-torque mode (in ROS mode equivalent to ZeroTorqueMode) |

**ROS/CAN Shared -- Helper Methods**

| Method | Parameters | Returns | Description |
|--------|-----------|---------|-------------|
| `GetCurrentSDKVersion()` | — | `str` | SDK version string |
| `GetCurrentProtocolVersion()` | — | `str` | Protocol version string |
| `GetCanFps()` | — | `float` | CAN frame rate (ROS mode returns joint_states frequency) |
| `GetCanName()` | — | `str` | CAN interface name / ROS namespace |
| `SetPositionPD(kp, kd)` | Kp, Kd | — | Set default PD gains |
| `SetJointLimitEnabled(enabled)` | bool | — | Toggle software joint limit protection |
| `ResetArm()` | — | `bool` | Reset state machine to IDLE |

#### ArmManager (Multi-Arm Management)

Singleton pattern, unified management of multiple CAN/ROS arm instances.

```python
from el_a3_sdk import ArmManager

mgr = ArmManager()

# Register direct CAN arm
master = mgr.register_can_arm("master", can_name="can0")

# Register ROS namespace arm
slave = mgr.register_ros_arm("slave", namespace="arm2",
                              controller_name="arm2_arm_controller")

# Batch create from config file
mgr = ArmManager.from_config("config/multi_arm_config.yaml", mode="ros")

# Access
arm = mgr.get_arm("master")   # or mgr["master"]
mgr.disconnect_all()
mgr.reset()                    # destroy Singleton
```

| Method | Description |
|--------|-------------|
| `register_can_arm(name, can_name, **kwargs)` | Register CAN mode arm, returns `ELA3Interface` |
| `register_ros_arm(name, namespace, **kwargs)` | Register ROS mode arm, returns `ELA3ROSInterface` |
| `get_arm(name)` / `mgr[name]` | Get registered arm |
| `has_arm(name)` / `name in mgr` | Check if arm is registered |
| `unregister(name)` | Unregister and disconnect |
| `disconnect_all()` | Disconnect all arms |
| `from_config(path, mode, auto_connect)` | Batch create from YAML |
| `arm_names` / `len(mgr)` | Registered arm list/count |

#### ELA3Kinematics (Kinematics/Dynamics)

Based on Pinocchio, independent of ROS, shared by CAN and ROS modes. Requires `pinocchio` (`pip install pin` or `apt install ros-humble-pinocchio`).

```python
from el_a3_sdk.kinematics import ELA3Kinematics

kin = ELA3Kinematics(urdf_path=None, ee_frame_name="end_effector")

pose = kin.forward_kinematics([0.0]*6)          # FK → ArmEndPose
q = kin.inverse_kinematics(pose, max_iter=200)   # IK → List[float] or None
J = kin.compute_jacobian([0.0]*6)                # 6×6 Jacobian (world-aligned)
tau_g = kin.compute_gravity([0.0]*6)             # RNEA gravity compensation torques
tau = kin.inverse_dynamics(q, v, a)              # RNEA inverse dynamics
acc = kin.forward_dynamics(q, v, tau)            # ABA forward dynamics
M = kin.mass_matrix([0.0]*6)                     # CRBA inertia matrix (6×6)
C = kin.coriolis_matrix(q, v)                    # Coriolis matrix (6×6)
```

> The kinematics module operates on 6 arm joints (L1-L6). L7 gripper is not part of the kinematic chain.

#### Trajectory Planning (`trajectory.py`)

No ROS dependency, shared by CAN and ROS modes.

```python
from el_a3_sdk.trajectory import SCurvePlanner, MultiJointPlanner, CubicSplinePlanner

# Single-joint S-curve
planner = SCurvePlanner(v_max=3.0, a_max=10.0, j_max=50.0)
profile = planner.plan(start=0.0, end=1.5)
pos, vel, acc = planner.evaluate(profile, t=0.5)
points = planner.generate_trajectory(profile, dt=0.005)

# Multi-joint synchronized
mp = MultiJointPlanner(n_joints=6, v_max=3.0, a_max=10.0)
profiles = mp.plan_sync(starts=[0]*6, ends=[0.5, 1.0, -0.5, 0, 0, 0])
traj = mp.generate_trajectory(profiles, dt=0.005)

# Cubic spline multi-waypoint
waypoints = [[0]*6, [0.5, 1.0, -0.5, 0, 0, 0], [0]*6]
traj = CubicSplinePlanner.plan_waypoints(waypoints, durations=[2.0, 2.0])
```

---

## Troubleshooting

### CAN Interface Issues

```bash
lsusb | grep -i can
sudo modprobe can && sudo modprobe can_raw && sudo modprobe gs_usb
ip link show type can
sudo ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 && sudo ip link set can0 up
candump can0
```

### Motor Not Responding

1. Check CAN wiring and termination resistors
2. Confirm motor ID configuration (1-7)
3. Check power supply
4. Verify host CAN ID (default 253/0xFD)

### End-Effector Jitter

1. Adjust `position_kp` / `position_kd` (adjustable at runtime)
2. Increase `smoothing_alpha`
3. Reduce `max_joint_velocity`
4. Check mechanical backlash

### Build Errors

```bash
cd ros2_ws && rm -rf build install log
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

---

## Directory Structure

```
EL-A3/
├── README.md                              # Bilingual index
├── README_zh.md                           # Chinese documentation
├── README_en.md                           # English documentation
│
├── el_a3_description/                     # Robot description package
│   ├── urdf/
│   │   ├── el_a3.urdf.xacro                  # URDF (L7 gripper + end_effector branch)
│   │   └── el_a3_ros2_control.xacro           # ros2_control hardware interface config (L1-L7)
│   ├── config/
│   │   ├── el_a3_controllers.yaml             # Controller config (arm + gripper + zero_torque)
│   │   ├── multi_arm_config.yaml              # Multi-arm CAN/namespace config
│   │   ├── master_slave_config.yaml           # Master-slave mapping config
│   │   └── inertia_params.yaml                # Calibrated inertia parameters
│   └── launch/
│       ├── el_a3_control.launch.py            # Single-arm launch
│       └── multi_arm_control.launch.py        # Multi-arm launch
│
├── el_a3_hardware/                        # ros2_control hardware interface + controller plugins
│   ├── include/el_a3_hardware/
│   │   ├── el_a3_hardware.hpp                 # SystemInterface header
│   │   ├── zero_torque_controller.hpp         # ZeroTorqueController header
│   │   └── robstride_can_driver.hpp           # CAN driver
│   ├── src/
│   │   ├── el_a3_hardware.cpp                 # Hardware interface (Pinocchio gravity, joint limit protection)
│   │   ├── zero_torque_controller.cpp         # Zero-torque controller (Pinocchio RNEA)
│   │   └── robstride_can_driver.cpp           # CAN communication driver
│   ├── el_a3_hardware_plugin.xml              # Hardware interface plugin descriptor
│   └── el_a3_controller_plugin.xml            # ZeroTorqueController plugin descriptor
│
├── el_a3_moveit_config/                   # MoveIt2 configuration
│   ├── config/
│   │   ├── el_a3.srdf                         # SRDF (arm + gripper planning groups)
│   │   ├── moveit_controllers.yaml            # MoveIt controllers (arm + gripper)
│   │   ├── kinematics.yaml
│   │   ├── joint_limits.yaml
│   │   └── ompl_planning.yaml
│   └── launch/
│       ├── demo.launch.py
│       └── robot.launch.py
│
├── el_a3_sdk/                             # Python SDK (dual-mode)
│   ├── interface.py                           # Direct CAN API (7 joints + ArmState)
│   ├── ros_interface.py                       # ROS Control API (7 joints + ArmState)
│   ├── arm_manager.py                         # Multi-arm manager (Singleton)
│   ├── kinematics.py                          # Pinocchio FK/IK/Gravity/Jacobian
│   ├── trajectory.py                          # S-curve & spline trajectory planning
│   ├── can_driver.py                          # SocketCAN driver
│   ├── protocol.py                            # Protocol enums + ArmState state machine
│   ├── data_types.py                          # Data structures (7 joints)
│   └── demo/                                  # Example scripts
│
├── ros2_ws/src/
│   ├── el_a3_teleop/                      # Teleoperation
│   │   ├── el_a3_teleop/
│   │   │   ├── xbox_teleop_node.py            # Xbox Cartesian control
│   │   │   └── master_slave_node.py           # Master-slave teleop (switch_controller)
│   │   ├── config/
│   │   └── launch/
│   ├── el_a3_vision/                      # Vision grasping
│   └── el_a3_web_ui/                      # Web control interface (Flask + SDK Bridge)
│       ├── el_a3_web_ui/
│       │   ├── web_server.py                  # Flask + SocketIO main server
│       │   ├── sdk_bridge.py                  # SDK bridge layer (ArmManager → WebSocket)
│       │   └── ros2_bridge.py                 # Legacy ROS2 direct bridge (kept as fallback)
│       ├── static/                            # Frontend assets (CSS/JS/URDF)
│       └── templates/                         # HTML templates
│
├── scripts/                               # Utility scripts
│   ├── inertia_calibration.py                 # Pinocchio inertia calibration
│   ├── setup_can.sh                           # CAN interface setup
│   ├── setup_multi_can.sh                     # Multi-CAN batch setup
│   ├── install_deps.sh                        # Dependency installation
│   └── start_real_xbox_control.sh             # One-click launch
│
└── hardware/                              # Hardware design files
    ├── mechanical/                            # Mechanical (STEP/STL/drawings)
    └── electronics/                           # PCB (schematics/PCB/Gerber/BOM)
```

---

## Script Reference

| Script | Function | Usage |
|--------|----------|-------|
| `setup_can.sh` | CAN interface setup | `sudo ./setup_can.sh can0 1000000` |
| `setup_multi_can.sh` | Multi-CAN batch setup | `sudo ./setup_multi_can.sh 4` |
| `install_deps.sh` | ROS2 dependency installation | `sudo ./install_deps.sh` |
| `start_real_xbox_control.sh` | One-click launch | `./start_real_xbox_control.sh can0` |
| `inertia_calibration.py` | Inertia calibration | `python3 inertia_calibration.py [--quick]` |
| `move_to_zero.py` | Return to zero | `python3 move_to_zero.py` |

---

## License

Apache-2.0
