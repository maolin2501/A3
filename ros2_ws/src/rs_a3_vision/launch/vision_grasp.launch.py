"""
RS-A3 视觉抓取系统启动文件
启动相机、物体检测、视觉伺服和抓取管理
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    """加载 YAML 配置文件"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # 声明参数
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_camera',
            default_value='false',
            description='使用模拟相机（无硬件时测试）'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_realsense_driver',
            default_value='false',
            description='使用官方 RealSense ROS2 驱动（否则使用内置驱动）'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_yolo',
            default_value='true',
            description='启用 YOLO 物体检测'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_gui',
            default_value='true',
            description='启动可视化界面'
        )
    )
    
    # 获取配置
    config_file = PathJoinSubstitution([
        FindPackageShare('rs_a3_vision'),
        'config',
        'vision_config.yaml'
    ])
    
    # 加载配置（用于某些节点）
    vision_config = load_yaml('rs_a3_vision', 'config/vision_config.yaml')
    
    nodes = []
    
    # 相机节点
    camera_node = Node(
        package='rs_a3_vision',
        executable='camera_node',
        name='camera_node',
        parameters=[{
            'width': 640,
            'height': 480,
            'fps': 30,
            'enable_depth': True,
            'enable_color': True,
            'align_depth': True,
            'use_mock': LaunchConfiguration('use_mock_camera'),
        }],
        output='screen',
    )
    nodes.append(camera_node)
    
    # 物体检测节点
    detector_node = Node(
        package='rs_a3_vision',
        executable='object_detector_node',
        name='object_detector',
        parameters=[{
            'model_path': 'yolov8n.pt',
            'confidence_threshold': 0.5,
            'enable_yolo': LaunchConfiguration('enable_yolo'),
            'depth_scale': 0.001,
        }],
        output='screen',
    )
    nodes.append(detector_node)
    
    # 手眼标定节点
    calibration_node = Node(
        package='rs_a3_vision',
        executable='hand_eye_calibration_node',
        name='hand_eye_calibration',
        parameters=[{
            'calibration_type': 'eye_on_base',
            'base_frame': 'base_link',
            'ee_frame': 'end_effector',
            'camera_frame': 'camera_color_optical_frame',
            'translation_x': 0.0,
            'translation_y': -0.3,
            'translation_z': 0.5,
            'rotation_x': 0.0,
            'rotation_y': 0.707,
            'rotation_z': 0.0,
            'rotation_w': 0.707,
        }],
        output='screen',
    )
    nodes.append(calibration_node)
    
    # 视觉伺服节点
    servo_node = Node(
        package='rs_a3_vision',
        executable='visual_servo_node',
        name='visual_servo',
        parameters=[{
            'base_frame': 'base_link',
            'ee_frame': 'end_effector',
            'camera_frame': 'camera_color_optical_frame',
            'kp_linear': 0.5,
            'kp_angular': 0.3,
            'max_linear_vel': 0.1,
            'max_angular_vel': 0.3,
            'approach_distance': 0.10,
            'grasp_distance': 0.02,
            'control_rate': 50.0,
        }],
        output='screen',
    )
    nodes.append(servo_node)
    
    # 抓取管理节点
    grasp_manager_node = Node(
        package='rs_a3_vision',
        executable='grasp_manager_node',
        name='grasp_manager',
        parameters=[{
            'lift_height': 0.1,
            'approach_speed': 0.05,
            'grasp_timeout': 30.0,
        }],
        output='screen',
    )
    nodes.append(grasp_manager_node)
    
    # GUI 节点（条件启动）
    gui_node = Node(
        package='rs_a3_grasp_gui',
        executable='grasp_gui_node',
        name='grasp_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_gui')),
    )
    nodes.append(gui_node)
    
    return LaunchDescription(declared_arguments + nodes)
