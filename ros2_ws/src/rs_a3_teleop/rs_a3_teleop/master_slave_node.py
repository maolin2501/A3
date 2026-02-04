#!/usr/bin/env python3
"""
RS-A3 主从遥操作节点

支持一对一、一对多、多对多的灵活主从映射配置
主臂可使用零力矩模式进行拖动示教，从臂实时跟随
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import SetBool
from builtin_interfaces.msg import Duration

import yaml
import numpy as np
from typing import List, Dict, Optional
from dataclasses import dataclass, field


@dataclass
class MappingConfig:
    """主从映射配置"""
    master: str
    slaves: List[str]
    scale: float = 1.0
    mirror: bool = False
    joint_offset: List[float] = field(default_factory=lambda: [0.0] * 6)
    enabled: bool = True


class MasterSlaveMapping:
    """单个主从映射（一主多从）"""
    
    def __init__(self, node: Node, config: MappingConfig, safety_config: dict, slave_config: dict):
        self.node = node
        self.config = config
        self.safety = safety_config
        self.slave_cfg = slave_config
        
        self.master_state: Optional[JointState] = None
        self.last_target: Optional[List[float]] = None
        
        # 回调组
        self.cb_group = ReentrantCallbackGroup()
        
        # 订阅主臂状态
        self.master_sub = node.create_subscription(
            JointState,
            f'/{config.master}/joint_states',
            self.master_callback,
            10,
            callback_group=self.cb_group
        )
        
        # 为每个从臂创建发布器/客户端
        self.slave_pubs: Dict[str, rclpy.publisher.Publisher] = {}
        self.slave_clients: Dict[str, ActionClient] = {}
        
        for slave_ns in config.slaves:
            if slave_config.get('use_streaming', True):
                # 流式控制：直接发布轨迹命令
                pub = node.create_publisher(
                    JointTrajectory,
                    f'/{slave_ns}/{slave_ns}_arm_controller/joint_trajectory',
                    10
                )
                self.slave_pubs[slave_ns] = pub
            else:
                # Action 控制
                client = ActionClient(
                    node,
                    FollowJointTrajectory,
                    f'/{slave_ns}/{slave_ns}_arm_controller/follow_joint_trajectory',
                    callback_group=self.cb_group
                )
                self.slave_clients[slave_ns] = client
        
        node.get_logger().info(
            f'映射已创建: {config.master} -> {config.slaves}'
        )
    
    def master_callback(self, msg: JointState):
        """主臂状态回调"""
        self.master_state = msg
    
    def update(self):
        """更新从臂位置"""
        if self.master_state is None:
            return
        
        # 获取主臂关节位置
        master_positions = list(self.master_state.position)
        if len(master_positions) < 6:
            return
        
        # 计算目标位置
        target_positions = self.compute_target(master_positions)
        
        # 安全检查
        if not self.safety_check(target_positions):
            return
        
        self.last_target = target_positions
        
        # 发送到所有从臂
        for slave_ns in self.config.slaves:
            self.send_to_slave(slave_ns, target_positions)
    
    def compute_target(self, master_positions: List[float]) -> List[float]:
        """计算从臂目标位置"""
        target = list(master_positions[:6])
        
        # 应用缩放
        target = [p * self.config.scale for p in target]
        
        # 应用镜像（翻转 L1, L4, L6）
        if self.config.mirror:
            target[0] = -target[0]  # L1
            target[3] = -target[3]  # L4
            target[5] = -target[5]  # L6
        
        # 应用偏移
        target = [p + o for p, o in zip(target, self.config.joint_offset)]
        
        return target
    
    def safety_check(self, target: List[float]) -> bool:
        """安全检查"""
        if self.last_target is None:
            return True
        
        # 检查速度限制
        max_vel = self.safety.get('max_velocity', 2.0)
        dt = 1.0 / 50.0  # 假设 50Hz
        
        for i, (curr, prev) in enumerate(zip(target, self.last_target)):
            velocity = abs(curr - prev) / dt
            if velocity > max_vel:
                self.node.get_logger().warn(
                    f'关节 {i+1} 速度超限: {velocity:.2f} > {max_vel}'
                )
                return False
        
        return True
    
    def send_to_slave(self, slave_ns: str, target_positions: List[float]):
        """发送命令到从臂"""
        # 生成关节名称
        prefix = f"{slave_ns}_"
        joint_names = [f"{prefix}L{i}_joint" for i in range(1, 7)]
        
        # 创建轨迹消息
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * 6
        
        # 设置执行时间
        duration_sec = self.slave_cfg.get('trajectory_duration', 0.05)
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        
        trajectory.points = [point]
        
        if slave_ns in self.slave_pubs:
            self.slave_pubs[slave_ns].publish(trajectory)


class MasterSlaveNode(Node):
    """主从遥操作节点"""
    
    def __init__(self):
        super().__init__('master_slave_node')
        
        # 声明参数
        self.declare_parameter('config_file', '')
        self.declare_parameter('master_ns', '')
        self.declare_parameter('slave_ns', '')
        self.declare_parameter('slave_ns_list', '')
        self.declare_parameter('rate', 50.0)
        
        # 获取参数
        config_file = self.get_parameter('config_file').value
        master_ns = self.get_parameter('master_ns').value
        slave_ns = self.get_parameter('slave_ns').value
        slave_ns_list = self.get_parameter('slave_ns_list').value
        rate = self.get_parameter('rate').value
        
        # 加载配置
        self.config = self.load_config(config_file, master_ns, slave_ns, slave_ns_list)
        
        # 更新频率
        self.rate = self.config.get('rate', rate)
        
        # 获取安全和从臂配置
        self.safety_config = self.config.get('safety', {})
        self.slave_config = self.config.get('slave', {})
        self.master_config = self.config.get('master', {})
        
        # 创建映射
        self.mappings: List[MasterSlaveMapping] = []
        
        for mapping_cfg in self.config.get('mappings', []):
            if mapping_cfg.get('enabled', True):
                cfg = MappingConfig(
                    master=mapping_cfg['master'],
                    slaves=mapping_cfg['slaves'],
                    scale=mapping_cfg.get('scale', 1.0),
                    mirror=mapping_cfg.get('mirror', False),
                    joint_offset=mapping_cfg.get('joint_offset', [0.0] * 6),
                    enabled=mapping_cfg.get('enabled', True)
                )
                mapping = MasterSlaveMapping(
                    self, cfg, self.safety_config, self.slave_config
                )
                self.mappings.append(mapping)
        
        # 启用主臂零力矩模式
        if self.master_config.get('use_zero_torque_mode', True):
            self.enable_master_zero_torque()
        
        # 控制定时器
        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)
        
        self.get_logger().info(
            f'主从遥操作节点已启动，频率: {self.rate}Hz，映射数: {len(self.mappings)}'
        )
    
    def load_config(self, config_file: str, master_ns: str, 
                    slave_ns: str, slave_ns_list: str) -> dict:
        """加载配置"""
        config = {}
        
        # 从文件加载
        if config_file:
            try:
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)
                self.get_logger().info(f'已加载配置文件: {config_file}')
            except Exception as e:
                self.get_logger().error(f'加载配置文件失败: {e}')
        
        # 从命令行参数覆盖
        if master_ns and (slave_ns or slave_ns_list):
            # 解析从臂列表
            slaves = []
            if slave_ns_list:
                # 解析列表格式 "['arm2','arm3']" 或 "arm2,arm3"
                slave_ns_list = slave_ns_list.strip("[]'\"")
                slaves = [s.strip().strip("'\"") for s in slave_ns_list.split(',')]
            elif slave_ns:
                slaves = [slave_ns]
            
            # 创建单个映射
            config['mappings'] = [{
                'master': master_ns,
                'slaves': slaves,
                'scale': 1.0,
                'mirror': False,
                'enabled': True
            }]
            
            self.get_logger().info(f'使用命令行参数: {master_ns} -> {slaves}')
        
        return config
    
    def enable_master_zero_torque(self):
        """启用主臂零力矩模式"""
        for mapping in self.mappings:
            master_ns = mapping.config.master
            service_name = f'/{master_ns}/set_zero_torque_mode'
            
            client = self.create_client(SetBool, service_name)
            
            if client.wait_for_service(timeout_sec=2.0):
                request = SetBool.Request()
                request.data = True
                future = client.call_async(request)
                self.get_logger().info(f'已请求启用 {master_ns} 零力矩模式')
            else:
                self.get_logger().warn(f'服务不可用: {service_name}')
    
    def control_loop(self):
        """主控制循环"""
        for mapping in self.mappings:
            mapping.update()


def main(args=None):
    rclpy.init(args=args)
    
    node = MasterSlaveNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
