"""
RS-A3 多机械臂控制 Launch 文件

支持同时启动多条机械臂，每条机械臂使用独立的 CAN 接口和命名空间
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def load_yaml(file_path):
    """加载 YAML 配置文件"""
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_arm_nodes(context, *args, **kwargs):
    """根据配置动态生成机械臂节点"""
    
    # 获取参数
    config_file = LaunchConfiguration('config_file').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    
    # 如果没有指定配置文件，使用默认路径
    if not config_file:
        config_file = os.path.join(
            FindPackageShare('rs_a3_description').perform(context),
            'config', 'multi_arm_config.yaml'
        )
    
    # 加载配置
    config = load_yaml(config_file)
    arms_config = config.get('arms', {})
    global_config = config.get('global', {})
    
    use_mock = global_config.get('use_mock_hardware', False)
    
    nodes = []
    
    # 为每个启用的机械臂创建节点
    for arm_name, arm_cfg in arms_config.items():
        if not arm_cfg.get('enabled', False):
            continue
            
        prefix = arm_cfg.get('prefix', f'{arm_name}_')
        can_interface = arm_cfg.get('can_interface', 'can0')
        host_can_id = arm_cfg.get('host_can_id', 253)
        
        # 生成带前缀的 URDF
        robot_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("rs_a3_description"), "urdf", "rs_a3.urdf.xacro"
            ]),
            f" prefix:={prefix}",
            f" use_mock_hardware:={'true' if use_mock else 'false'}",
            f" can_interface:={can_interface}",
            f" host_can_id:={host_can_id}",
        ])
        
        robot_description = {"robot_description": robot_description_content}
        
        # 生成控制器配置
        controller_params = generate_controller_params(prefix, global_config.get('update_rate', 200))
        
        # 创建节点组（带命名空间）
        arm_group = GroupAction([
            PushRosNamespace(arm_name),
            
            # Controller Manager
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controller_params],
                output="both",
                remappings=[
                    ("~/robot_description", f"/{arm_name}/robot_description"),
                ],
            ),
            
            # Robot State Publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
            
            # Joint State Broadcaster Spawner
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    f"{prefix}joint_state_broadcaster",
                    "--controller-manager", f"/{arm_name}/controller_manager",
                    "--controller-manager-timeout", "60"
                ],
            ),
            
            # Arm Controller Spawner
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    f"{prefix}arm_controller",
                    "--controller-manager", f"/{arm_name}/controller_manager",
                    "--controller-manager-timeout", "60"
                ],
            ),
        ])
        
        nodes.append(arm_group)
    
    # RViz（可选，只启动一个实例）
    if use_rviz.lower() == 'true':
        rviz_config_file = PathJoinSubstitution([
            FindPackageShare("rs_a3_description"), "config", "rs_a3_view.rviz"
        ])
        
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        )
        nodes.append(rviz_node)
    
    return nodes


def generate_controller_params(prefix: str, update_rate: int) -> dict:
    """生成控制器参数配置"""
    
    joints = [f"{prefix}L{i}_joint" for i in range(1, 7)]
    
    return {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": update_rate,
                f"{prefix}joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
                f"{prefix}arm_controller": {
                    "type": "joint_trajectory_controller/JointTrajectoryController"
                }
            }
        },
        f"{prefix}arm_controller": {
            "ros__parameters": {
                "joints": joints,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
                "open_loop_control": True,
                "allow_nonzero_velocity_at_trajectory_end": True,
                "interpolation_method": "splines",
                "state_publish_rate": 200.0,
                "action_monitor_rate": 50.0,
                "constraints": {
                    "stopped_velocity_tolerance": 0.1,
                    "goal_time": 0.0,
                    **{joint: {"goal": 0.03} for joint in joints}
                }
            }
        }
    }


def generate_launch_description():
    # 声明参数
    declared_arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="多机械臂配置文件路径（YAML）",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="是否启动 RViz",
        ),
    ]
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=generate_arm_nodes)]
    )
