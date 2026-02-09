"""
RS-A3 完整抓取系统启动文件
启动机械臂控制 + MoveIt + 视觉抓取系统
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
            description='使用模拟相机'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='使用模拟机械臂硬件'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN 接口'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'enable_yolo',
            default_value='true',
            description='启用 YOLO 检测'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='启动 RViz'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_gui',
            default_value='true',
            description='启动抓取 GUI'
        )
    )
    
    # 包含机械臂启动文件
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rs_a3_moveit_config'),
                'launch',
                'robot.launch.py'
            ])
        ]),
        launch_arguments={
            'can_interface': LaunchConfiguration('can_interface'),
            'use_rviz': LaunchConfiguration('launch_rviz'),
        }.items()
    )
    
    # 延迟启动视觉系统（等待机械臂系统就绪）
    vision_launch = TimerAction(
        period=5.0,  # 延迟 5 秒
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rs_a3_vision'),
                        'launch',
                        'vision_grasp.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_mock_camera': LaunchConfiguration('use_mock_camera'),
                    'enable_yolo': LaunchConfiguration('enable_yolo'),
                    'launch_gui': LaunchConfiguration('launch_gui'),
                }.items()
            )
        ]
    )
    
    return LaunchDescription(declared_arguments + [robot_launch, vision_launch])
