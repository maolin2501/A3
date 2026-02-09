# RS-A3 视觉抓取系统

基于 RealSense D435 相机的物体检测与视觉伺服抓取系统。

## 功能特性

- **RealSense D435 相机集成**：RGB-D 图像获取，深度对齐
- **YOLOv8 物体检测**：实时检测画面中的物体
- **交互式物体选择**：点击图像选择要抓取的物体
- **视觉伺服控制**：PBVS 位置跟踪，末端自动接近目标
- **抓取状态机**：完整的抓取流程管理
- **PyQt5 可视化界面**：直观的操作控制界面

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                  PyQt5 可视化界面 (grasp_gui)                │
│  - 显示检测画面                                              │
│  - 点击选择物体                                              │
│  - 控制按钮                                                 │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              抓取管理器 (grasp_manager)                      │
│  - 状态机控制                                               │
│  - 协调各模块                                               │
└─────────────────────────────────────────────────────────────┘
          │                   │                    │
          ▼                   ▼                    ▼
┌──────────────┐    ┌──────────────┐    ┌──────────────┐
│  物体检测器   │    │  视觉伺服    │    │  MoveIt      │
│ YOLOv8/SAM   │    │   PBVS       │    │  轨迹规划     │
└──────────────┘    └──────────────┘    └──────────────┘
          │
          ▼
┌──────────────┐
│  相机节点    │
│ RealSense   │
└──────────────┘
```

## 安装

### 1. 安装依赖

```bash
cd ~/RS/A3
./scripts/install_vision_deps.sh
```

### 2. 编译工作空间

```bash
cd ~/RS/A3
# 移除 COLCON_IGNORE（如果存在）
rm -f ros2_ws/COLCON_IGNORE

# 编译
colcon build --packages-select rs_a3_vision rs_a3_grasp_gui

# Source 环境
source install/setup.bash
```

## 使用方法

### 快速启动（完整系统）

```bash
# 启动机械臂 + 视觉抓取系统
ros2 launch rs_a3_vision full_grasp_system.launch.py
```

### 分步启动

```bash
# 终端 1：启动机械臂控制
ros2 launch rs_a3_moveit_config robot.launch.py

# 终端 2：启动视觉抓取系统
ros2 launch rs_a3_vision vision_grasp.launch.py
```

### 测试模式（无硬件）

```bash
# 使用模拟相机和模拟机械臂
ros2 launch rs_a3_vision full_grasp_system.launch.py \
    use_mock_camera:=true \
    use_mock_hardware:=true
```

### 仅测试相机

```bash
# 真实相机
ros2 launch rs_a3_vision camera_only.launch.py

# 模拟相机
ros2 launch rs_a3_vision camera_only.launch.py use_mock:=true
```

## ROS2 话题

### 发布的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/camera/color/image_raw` | sensor_msgs/Image | RGB 图像 |
| `/camera/aligned_depth/image_raw` | sensor_msgs/Image | 对齐的深度图像 |
| `/detection/image` | sensor_msgs/Image | 带检测框的图像 |
| `/target/pose` | geometry_msgs/PoseStamped | 目标位姿（相机坐标系） |
| `/target/pose_base` | geometry_msgs/PoseStamped | 目标位姿（基座坐标系） |
| `/detection/markers` | visualization_msgs/MarkerArray | RViz 可视化标记 |
| `/visual_servo/status` | std_msgs/String | 伺服状态 |
| `/grasp/state` | std_msgs/String | 抓取状态 |

### 订阅的话题

| 话题 | 类型 | 描述 |
|------|------|------|
| `/selected_point` | geometry_msgs/Point | 用户选择的像素点 |
| `/gripper/command` | std_msgs/Bool | 夹爪命令 (True=闭合) |

### 服务

| 服务 | 类型 | 描述 |
|------|------|------|
| `/grasp/start` | std_srvs/Trigger | 开始抓取序列 |
| `/grasp/abort` | std_srvs/Trigger | 中止抓取 |
| `/grasp/reset` | std_srvs/Trigger | 重置系统 |
| `/visual_servo/enable` | std_srvs/SetBool | 启用/禁用伺服 |
| `/visual_servo/start_approach` | std_srvs/Trigger | 开始接近目标 |
| `/visual_servo/stop` | std_srvs/Trigger | 停止伺服运动 |

## 配置文件

### 视觉系统配置

`config/vision_config.yaml`:

```yaml
camera:
  width: 640
  height: 480
  fps: 30

object_detector:
  model_path: "yolov8n.pt"
  confidence_threshold: 0.5

visual_servo:
  kp_linear: 0.5
  max_linear_vel: 0.1
  approach_distance: 0.10
```

### 手眼标定配置

`config/hand_eye_calibration.yaml`:

```yaml
calibration_type: "eye_on_base"  # 或 "eye_in_hand"
translation:
  x: 0.0
  y: -0.30
  z: 0.50
rotation:
  x: 0.0
  y: 0.707
  z: 0.0
  w: 0.707
```

## 手眼标定

相机需要进行手眼标定才能准确将检测到的物体位置转换到机械臂坐标系。

### 方法 1：使用 easy_handeye

```bash
sudo apt install ros-humble-easy-handeye2
# 按照 easy_handeye 文档进行标定
```

### 方法 2：手动测量

1. 测量相机相对于机械臂基座的位置和朝向
2. 修改 `config/hand_eye_calibration.yaml`
3. 重启系统

## 抓取流程

1. **选择目标**：在 GUI 界面点击要抓取的物体
2. **开始抓取**：点击"开始抓取"按钮
3. **自动接近**：视觉伺服控制末端接近目标
4. **执行抓取**：到达位置后自动闭合夹爪
5. **提起物体**：抓取后自动提起
6. **返回初始位置**：完成后返回原位

## 故障排除

### 相机无法连接

```bash
# 检查 USB 连接
lsusb | grep Intel

# 检查权限
sudo chmod 666 /dev/video*
```

### YOLO 模型加载失败

```bash
# 手动下载模型
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

### TF 变换错误

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames

# 检查变换
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## 依赖

- ROS2 Humble
- Python 3.10+
- OpenCV
- PyQt5
- ultralytics (YOLOv8)
- pyrealsense2 (可选，用于真实相机)
- MoveIt2

## 文件结构

```
rs_a3_vision/
├── rs_a3_vision/
│   ├── __init__.py
│   ├── camera_node.py          # 相机驱动
│   ├── object_detector.py      # 物体检测
│   ├── visual_servo.py         # 视觉伺服
│   ├── grasp_manager.py        # 抓取管理
│   └── hand_eye_calibration.py # 手眼标定
├── config/
│   ├── vision_config.yaml
│   └── hand_eye_calibration.yaml
├── launch/
│   ├── vision_grasp.launch.py
│   ├── full_grasp_system.launch.py
│   └── camera_only.launch.py
├── package.xml
├── setup.py
└── README.md

rs_a3_grasp_gui/
├── rs_a3_grasp_gui/
│   ├── __init__.py
│   └── grasp_gui_node.py       # PyQt5 界面
├── launch/
│   └── gui.launch.py
├── package.xml
└── setup.py
```
