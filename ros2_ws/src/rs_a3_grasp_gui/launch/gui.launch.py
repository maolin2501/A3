"""
仅启动 GUI 界面（假设其他节点已运行）
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gui_node = Node(
        package='rs_a3_grasp_gui',
        executable='grasp_gui_node',
        name='grasp_gui',
        output='screen',
    )
    
    return LaunchDescription([gui_node])
