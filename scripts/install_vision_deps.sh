#!/bin/bash
# RS-A3 视觉抓取系统依赖安装脚本

set -e

echo "=========================================="
echo "RS-A3 视觉抓取系统依赖安装"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    print_error "ROS2 环境未设置，请先 source ROS2 环境"
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi

print_status "检测到 ROS2 版本: $ROS_DISTRO"

# 更新包列表
print_status "更新包列表..."
sudo apt update

# 安装 ROS2 相关包
print_status "安装 ROS2 依赖包..."
sudo apt install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rqt-image-view \
    ros-${ROS_DISTRO}-visualization-msgs

# 安装 RealSense 相关包
print_status "安装 RealSense ROS2 包..."
sudo apt install -y \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description || {
    print_warning "RealSense ROS2 包安装失败，可能需要手动安装"
}

# 安装 Python 依赖
print_status "安装 Python 依赖..."
pip3 install --upgrade pip

# OpenCV
pip3 install opencv-python opencv-contrib-python

# PyRealsense2 (RealSense Python SDK)
pip3 install pyrealsense2 || {
    print_warning "pyrealsense2 安装失败，可能需要从源码编译"
    echo "  参考: https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python"
}

# NumPy
pip3 install numpy

# PyQt5 (GUI)
print_status "安装 PyQt5..."
pip3 install PyQt5

# YOLO (物体检测)
print_status "安装 YOLOv8 (ultralytics)..."
pip3 install ultralytics

# 可选：安装 SAM (Segment Anything Model)
# print_status "安装 SAM..."
# pip3 install segment-anything

# 可选：安装 GroundingDINO
# pip3 install groundingdino

# 下载 YOLO 模型
print_status "下载 YOLOv8 预训练模型..."
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" || {
    print_warning "YOLO 模型下载失败，首次运行时会自动下载"
}

# 验证安装
print_status "验证安装..."

echo ""
echo "检查 Python 包..."
python3 << 'EOF'
import sys

packages = {
    'cv2': 'OpenCV',
    'numpy': 'NumPy',
    'PyQt5': 'PyQt5',
}

all_ok = True
for pkg, name in packages.items():
    try:
        __import__(pkg)
        print(f"  ✓ {name}")
    except ImportError:
        print(f"  ✗ {name} - 未安装")
        all_ok = False

# 检查可选包
optional = {
    'pyrealsense2': 'RealSense SDK',
    'ultralytics': 'YOLOv8',
}

print("\n可选包:")
for pkg, name in optional.items():
    try:
        __import__(pkg)
        print(f"  ✓ {name}")
    except ImportError:
        print(f"  ○ {name} - 未安装 (可选)")

sys.exit(0 if all_ok else 1)
EOF

echo ""
print_status "=========================================="
print_status "依赖安装完成！"
print_status "=========================================="
echo ""
echo "接下来的步骤:"
echo "  1. 编译工作空间:"
echo "     cd ~/RS/A3 && colcon build --packages-select rs_a3_vision rs_a3_grasp_gui"
echo ""
echo "  2. Source 工作空间:"
echo "     source install/setup.bash"
echo ""
echo "  3. 测试相机（使用模拟模式）:"
echo "     ros2 launch rs_a3_vision camera_only.launch.py use_mock:=true"
echo ""
echo "  4. 启动完整系统:"
echo "     ros2 launch rs_a3_vision full_grasp_system.launch.py"
echo ""
