"""
仅启动相机节点（用于测试和调试）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock',
            default_value='false',
            description='使用模拟相机'
        )
    )
    
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
            'use_mock': LaunchConfiguration('use_mock'),
        }],
        output='screen',
    )
    
    # 图像查看节点 (rqt_image_view)
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view',
        arguments=['/camera/color/image_raw'],
    )
    
    return LaunchDescription(declared_arguments + [camera_node, image_view_node])
