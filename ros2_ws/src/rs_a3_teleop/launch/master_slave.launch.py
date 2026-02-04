"""
RS-A3 主从遥操作 Launch 文件

支持多种启动方式：
1. 配置文件方式（推荐）：支持复杂的一对多、多对多映射
2. 命令行参数方式：简单的一对一或一对多映射
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明参数
    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="主从映射配置文件路径（YAML）。如果指定，将覆盖命令行参数",
        ),
        DeclareLaunchArgument(
            "master_ns",
            default_value="",
            description="主臂命名空间（如 arm1）",
        ),
        DeclareLaunchArgument(
            "slave_ns",
            default_value="",
            description="从臂命名空间（单个从臂，如 arm2）",
        ),
        DeclareLaunchArgument(
            "slave_ns_list",
            default_value="",
            description="从臂命名空间列表（一对多，如 \"['arm2','arm3']\" 或 \"arm2,arm3\"）",
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="50.0",
            description="控制频率 (Hz)",
        ),
    ]
    
    # 主从遥操作节点
    master_slave_node = Node(
        package="rs_a3_teleop",
        executable="master_slave_node",
        name="master_slave_node",
        output="both",
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "master_ns": LaunchConfiguration("master_ns"),
            "slave_ns": LaunchConfiguration("slave_ns"),
            "slave_ns_list": LaunchConfiguration("slave_ns_list"),
            "rate": LaunchConfiguration("rate"),
        }],
    )
    
    return LaunchDescription(declared_arguments + [master_slave_node])
