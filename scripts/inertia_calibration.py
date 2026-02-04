#!/usr/bin/env python3
"""
RS-A3 机械臂惯性参数标定程序

使用 Pinocchio 动力学库，通过在多个关节配置下测量力矩数据，
拟合各连杆的惯性参数（质量、质心位置），实现精确的重力补偿。

标定流程：
1. 生成多关节组合测试点
2. 依次移动到每个配置，采集稳态力矩
3. 使用 scipy.optimize 拟合惯性参数
4. 验证拟合质量并保存配置

使用方法：
    python3 inertia_calibration.py [--quick] [--output PATH]
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

import numpy as np
from scipy.optimize import minimize
from dataclasses import dataclass, field
from typing import List, Dict, Tuple
import time
import argparse
import yaml
from datetime import datetime

try:
    import pinocchio as pin
    PINOCCHIO_AVAILABLE = True
except ImportError:
    PINOCCHIO_AVAILABLE = False
    print("错误: 需要安装 Pinocchio 库")
    print("  sudo apt install ros-humble-pinocchio")


@dataclass
class CalibrationConfig:
    """标定配置"""
    urdf_path: str = "/home/wy/RS/A3/rs_a3_description/urdf/rs_a3.urdf"
    output_path: str = "/home/wy/RS/A3/rs_a3_description/config/inertia_params.yaml"
    
    joint_names: List[str] = field(default_factory=lambda: [
        'L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint'
    ])
    
    # 基准位置（避开零点）
    home_position: List[float] = field(default_factory=lambda: [0.0, 0.785, -0.785, 0.5, 0.5, 0.0])
    
    # 采样参数
    samples_per_config: int = 40
    settle_time: float = 1.5
    sample_interval: float = 0.02
    motion_duration: float = 5.0


class InertiaCalibrator(Node):
    """惯性参数标定器"""
    
    def __init__(self, config: CalibrationConfig):
        super().__init__('inertia_calibrator')
        
        self.config = config
        self.joint_names = config.joint_names
        
        # 当前状态
        self.current_positions = [0.0] * 6
        self.current_efforts = [0.0] * 6
        self.state_received = False
        
        # Pinocchio 模型
        self.model = None
        self.data = None
        
        # 标定数据
        self.calibration_data: List[Tuple[np.ndarray, np.ndarray]] = []
        
        # ROS2 接口
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self._action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        # 初始化 Pinocchio
        if PINOCCHIO_AVAILABLE:
            self.init_pinocchio()
    
    def init_pinocchio(self):
        """初始化 Pinocchio 模型"""
        try:
            self.model = pin.buildModelFromUrdf(self.config.urdf_path)
            self.data = pin.Data(self.model)
            self.get_logger().info(f'Pinocchio 模型加载成功: {self.model.njoints} 个关节')
            return True
        except Exception as e:
            self.get_logger().error(f'Pinocchio 加载失败: {e}')
            return False
    
    def joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
                if idx < len(msg.effort):
                    self.current_efforts[i] = msg.effort[idx]
        self.state_received = True
    
    def move_to_position(self, target: List[float], duration: float = None) -> bool:
        """移动到目标位置"""
        if duration is None:
            duration = self.config.motion_duration
        
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return False
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal_msg.trajectory.points = [point]
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 10)
        
        return True
    
    def collect_samples(self) -> Tuple[np.ndarray, np.ndarray]:
        """在当前位置收集样本"""
        positions = []
        efforts = []
        
        for _ in range(self.config.samples_per_config):
            rclpy.spin_once(self, timeout_sec=self.config.sample_interval)
            positions.append(self.current_positions.copy())
            efforts.append(self.current_efforts.copy())
            time.sleep(self.config.sample_interval)
        
        return np.mean(positions, axis=0), np.mean(efforts, axis=0)
    
    def check_collision_with_base(self, config: List[float]) -> bool:
        """检查配置是否可能与底面发生碰撞
        
        当L2角度较大（末端向下）且L3角度接近0（小臂水平）时，
        末端可能会与底面碰撞。
        
        Returns:
            True 如果可能发生碰撞，应该跳过
        """
        l2 = config[1]  # L2关节角度
        l3 = config[2]  # L3关节角度
        
        # 经验规则（基于实际碰撞测试数据）：
        # 点39: L2=1.37, L3=-0.41 碰撞
        # 点49: L2=1.20, L3=-0.35 碰撞
        # 点53: L2=1.43, L3=-0.55 碰撞
        
        # 规则1: L2 >= 1.2 且 L3 > -0.4 时碰撞
        if l2 >= 1.2 and l3 > -0.4:
            return True
        
        # 规则2: L2 > 1.35 且 L3 > -0.6 时碰撞
        if l2 > 1.35 and l3 > -0.6:
            return True
        
        # 规则3: L2 >= 1.2 且 L3 >= -0.55 的组合区域
        if l2 >= 1.2 and l3 >= -0.55 and (l2 + l3 * 0.5) > 0.9:
            return True
        
        return False
    
    def generate_test_configurations(self, mode: str = 'full') -> List[List[float]]:
        """生成测试配置点
        
        Args:
            mode: 'quick' - 快速模式(~20点), 'full' - 完整模式(~46点), 
                  'high' - 高精度(~80点), 'ultra' - 超高精度(~120点)
        """
        configs = []
        home = np.array(self.config.home_position)
        
        if mode == 'quick':
            # 快速模式：约20个测试点
            l2_range = np.linspace(0.4, 1.4, 6)
            l3_range = np.linspace(-1.2, -0.4, 5)
            l4_range = np.array([0.3, 0.7, 1.0])
            l5_range = np.array([0.3, 0.6])
            grid_step = 2
        elif mode == 'high':
            # 高精度模式：约80个测试点
            l2_range = np.linspace(0.25, 1.55, 12)
            l3_range = np.linspace(-1.35, -0.25, 12)
            l4_range = np.linspace(0.25, 1.25, 7)
            l5_range = np.linspace(0.25, 1.05, 5)
            grid_step = 2
        elif mode == 'ultra':
            # 超高精度模式：约120个测试点
            l2_range = np.linspace(0.2, 1.6, 15)
            l3_range = np.linspace(-1.4, -0.2, 15)
            l4_range = np.linspace(0.2, 1.3, 8)
            l5_range = np.linspace(0.2, 1.1, 6)
            grid_step = 2
        else:
            # 完整模式：约46个测试点
            l2_range = np.linspace(0.3, 1.5, 10)
            l3_range = np.linspace(-1.3, -0.3, 10)
            l4_range = np.linspace(0.3, 1.2, 6)
            l5_range = np.linspace(0.3, 1.0, 5)
            grid_step = 2
        
        # 第一阶段：L2 单独变化
        self.get_logger().info('生成 L2 测试点...')
        for l2 in l2_range:
            cfg = home.copy()
            cfg[1] = l2
            configs.append(cfg.tolist())
        
        # 第二阶段：L3 单独变化
        self.get_logger().info('生成 L3 测试点...')
        for l3 in l3_range:
            cfg = home.copy()
            cfg[2] = l3
            configs.append(cfg.tolist())
        
        # 第三阶段：L2+L3 网格组合
        self.get_logger().info('生成 L2+L3 组合测试点...')
        l2_grid = l2_range[::grid_step]
        l3_grid = l3_range[::grid_step]
        for l2 in l2_grid:
            for l3 in l3_grid:
                cfg = home.copy()
                cfg[1] = l2
                cfg[2] = l3
                configs.append(cfg.tolist())
        
        # 第四阶段：L4+L5 组合变化
        self.get_logger().info('生成 L4+L5 组合测试点...')
        for l4 in l4_range:
            for l5 in l5_range:
                cfg = home.copy()
                cfg[3] = l4
                cfg[4] = l5
                configs.append(cfg.tolist())
        
        # 去重
        unique_configs = []
        for cfg in configs:
            is_dup = False
            for uc in unique_configs:
                if np.allclose(cfg, uc, atol=0.05):
                    is_dup = True
                    break
            if not is_dup:
                unique_configs.append(cfg)
        
        # 过滤掉可能与底面碰撞的配置
        safe_configs = []
        skipped = 0
        for cfg in unique_configs:
            if self.check_collision_with_base(cfg):
                skipped += 1
                self.get_logger().info(f'  跳过碰撞配置: L2={cfg[1]:.2f}, L3={cfg[2]:.2f}')
            else:
                safe_configs.append(cfg)
        
        if skipped > 0:
            self.get_logger().info(f'已过滤 {skipped} 个可能碰撞的配置')
        
        self.get_logger().info(f'总计 {len(safe_configs)} 个安全测试点')
        return safe_configs
    
    def check_wrist_collision(self, config: List[float]) -> bool:
        """检查腕部配置是否可能发生碰撞
        
        当腕部关节组合导致末端与大臂/小臂碰撞时返回 True
        
        碰撞场景：
        1. L4 接近 ±90° 且 L5 较大时，末端可能撞击大臂
        2. L5 负角度较大时，末端向后折叠可能碰撞
        """
        l2 = config[1]
        l3 = config[2]
        l4 = config[3]
        l5 = config[4]
        
        # 规则1：L4 接近极限且 L5 较大时
        # L4 > 1.3 rad (~75°) 且 L5 > 0.8 时可能碰撞
        if abs(l4) > 1.3 and l5 > 0.8:
            return True
        
        # 规则2：当 L2 较大（大臂向下）且 L5 负角度时
        # 末端可能与底面或大臂碰撞
        if l2 > 1.2 and l5 < -0.3:
            return True
        
        # 规则3：L4 接近极限且 L5 也接近极限
        if abs(l4) > 1.2 and abs(l5) > 1.0:
            return True
        
        # 规则4：当 L3 角度小（小臂水平）且 L5 大时
        # L3 > -0.5 且 L5 > 1.0 时可能碰撞
        if l3 > -0.5 and l5 > 1.0:
            return True
        
        return False
    
    def generate_wrist_configurations(self) -> List[List[float]]:
        """生成腕部关节 (L4/L5/L6) 测试配置点 - 改进版
        
        改进点：
        1. L6 是 yaw 轴，单独旋转不产生重力力矩
           需要通过 L5 倾斜时 L6 质量对 L5 的力矩影响来标定
        2. 增加 L5 大角度测试点（提高 L6 质量可辨识性）
        3. 添加碰撞检测避免与大臂/小臂碰撞
        4. 减少 L6 单独变化点（意义不大）
        """
        configs = []
        home = np.array(self.config.home_position)
        
        self.get_logger().info('生成改进版腕部测试配置...')
        self.get_logger().info('  策略: 增加L5大角度点以提高L6质量可辨识性')
        
        # L4 测试范围
        l4_range = np.linspace(0.0, 1.3, 10)     # 10点, 避免极限
        # L5 测试范围 - 增加大角度点以体现 L6 质量影响
        l5_range = np.linspace(-0.5, 1.3, 12)   # 12点, 包含负角度和大角度
        # L6 测试范围（较少点，因为单独变化意义不大）
        l6_range = np.linspace(-0.5, 0.5, 5)    # 5点
        
        # 第一阶段：L4 单独变化（L5/L6 在 home）
        self.get_logger().info('生成 L4 单独测试点...')
        for l4 in l4_range:
            cfg = home.copy()
            cfg[3] = l4
            if not self.check_wrist_collision(cfg):
                configs.append(cfg.tolist())
        
        # 第二阶段：L5 单独变化 - 重点！L6 质量通过 L5 力矩体现
        self.get_logger().info('生成 L5 单独测试点（重点：L6质量通过L5力矩体现）...')
        for l5 in l5_range:
            cfg = home.copy()
            cfg[4] = l5
            if not self.check_wrist_collision(cfg):
                configs.append(cfg.tolist())
        
        # 第三阶段：L4+L5 网格组合（核心数据）
        self.get_logger().info('生成 L4+L5 网格组合测试点...')
        l4_grid = np.linspace(0.0, 1.2, 7)   # 7点
        l5_grid = np.linspace(-0.3, 1.2, 8)  # 8点
        for l4 in l4_grid:
            for l5 in l5_grid:
                cfg = home.copy()
                cfg[3] = l4
                cfg[4] = l5
                if not self.check_wrist_collision(cfg):
                    configs.append(cfg.tolist())
        
        # 第四阶段：L5 大角度 + L6 变化（标定 L6 质量的关键）
        # 当 L5 倾斜时，L6 质量产生的力矩作用在 L5 上
        self.get_logger().info('生成 L5大角度+L6变化 组合点（L6质量标定关键）...')
        l5_large = [0.8, 1.0, 1.2]  # L5 大角度
        l4_fixed = [0.0, 0.5, 1.0]  # L4 固定几个值
        for l4 in l4_fixed:
            for l5 in l5_large:
                for l6 in l6_range:
                    cfg = home.copy()
                    cfg[3] = l4
                    cfg[4] = l5
                    cfg[5] = l6
                    if not self.check_wrist_collision(cfg):
                        configs.append(cfg.tolist())
        
        # 第五阶段：L5 负角度测试（不同重力方向）
        self.get_logger().info('生成 L5负角度测试点...')
        l5_negative = [-0.3, -0.5]
        for l4 in [0.0, 0.5, 1.0]:
            for l5 in l5_negative:
                cfg = home.copy()
                cfg[3] = l4
                cfg[4] = l5
                if not self.check_wrist_collision(cfg):
                    configs.append(cfg.tolist())
        
        # 去重
        unique_configs = []
        for cfg in configs:
            is_dup = False
            for uc in unique_configs:
                if np.allclose(cfg, uc, atol=0.05):
                    is_dup = True
                    break
            if not is_dup:
                unique_configs.append(cfg)
        
        # 再次过滤碰撞配置
        safe_configs = []
        skipped = 0
        for cfg in unique_configs:
            if self.check_wrist_collision(cfg):
                skipped += 1
            else:
                safe_configs.append(cfg)
        
        if skipped > 0:
            self.get_logger().info(f'已过滤 {skipped} 个可能碰撞的配置')
        
        self.get_logger().info(f'总计 {len(safe_configs)} 个安全腕部测试点')
        return safe_configs
    
    def compute_gravity_with_params(self, q: np.ndarray, params: np.ndarray) -> np.ndarray:
        """使用给定的惯性参数计算重力力矩"""
        if self.model is None:
            return np.zeros(6)
        
        # 临时修改模型的惯性参数
        # params 格式: [L2_mass, L2_com_x, L3_mass, L3_com_x, L3_com_y, 
        #               L4_mass, L4_com_x, L4_com_y, L5_mass, L5_com_z, L6_mass, L6_com_z]
        
        # 保存原始值
        original_inertias = []
        for i in range(1, min(7, self.model.nbodies)):
            original_inertias.append({
                'mass': self.model.inertias[i].mass,
                'lever': self.model.inertias[i].lever.copy()
            })
        
        # 应用新参数
        try:
            # L2 (index 2 in model)
            if len(params) > 0:
                self.model.inertias[2].mass = params[0]
            if len(params) > 1:
                self.model.inertias[2].lever[0] = params[1]
            
            # L3 (index 3 in model)
            if len(params) > 2:
                self.model.inertias[3].mass = params[2]
            if len(params) > 3:
                self.model.inertias[3].lever[0] = params[3]
            if len(params) > 4:
                self.model.inertias[3].lever[1] = params[4]
            
            # L4 (index 4 in model)
            if len(params) > 5:
                self.model.inertias[4].mass = params[5]
            if len(params) > 6:
                self.model.inertias[4].lever[0] = params[6]
            if len(params) > 7:
                self.model.inertias[4].lever[1] = params[7]
            
            # L5 (index 5 in model)
            if len(params) > 8:
                self.model.inertias[5].mass = params[8]
            if len(params) > 9:
                self.model.inertias[5].lever[2] = params[9]
            
            # L6 (index 6 in model)
            if len(params) > 10:
                self.model.inertias[6].mass = params[10]
            if len(params) > 11:
                self.model.inertias[6].lever[2] = params[11]
            
            # 重新创建 data
            self.data = pin.Data(self.model)
            
            # 计算重力力矩
            q_pin = np.array(q[:self.model.nq])
            v = np.zeros(self.model.nv)
            a = np.zeros(self.model.nv)
            tau = pin.rnea(self.model, self.data, q_pin, v, a)
            
        finally:
            # 恢复原始值
            for i, orig in enumerate(original_inertias):
                self.model.inertias[i + 1].mass = orig['mass']
                self.model.inertias[i + 1].lever = orig['lever']
            self.data = pin.Data(self.model)
        
        return tau[:6]
    
    def objective_function(self, params: np.ndarray) -> float:
        """优化目标函数：测量力矩与预测力矩的均方误差"""
        total_error = 0.0
        
        for positions, measured_efforts in self.calibration_data:
            predicted = self.compute_gravity_with_params(positions, params)
            
            # 只计算 L2-L6 的误差（L1 绕 Z 轴，不受重力影响）
            for i in range(1, 6):
                error = measured_efforts[i] - predicted[i]
                total_error += error ** 2
        
        return total_error
    
    def wrist_objective_function(self, wrist_params: np.ndarray, fixed_l2l3_params: np.ndarray) -> float:
        """腕部标定目标函数：只优化 L4/L5/L6 参数
        
        改进版：
        - L6 是 yaw 轴，其自身力矩变化很小
        - L6 质量主要通过 L5 的力矩来体现
        - 因此 L5 误差权重应该更高
        
        Args:
            wrist_params: [L4_mass, L4_com_x, L4_com_y, L5_mass, L5_com_z, L6_mass, L6_com_z] (7个参数)
            fixed_l2l3_params: [L2_mass, L2_com_x, L3_mass, L3_com_x, L3_com_y] (5个参数)
        """
        # 组合完整参数
        full_params = np.concatenate([fixed_l2l3_params, wrist_params])
        
        total_error = 0.0
        for positions, measured_efforts in self.calibration_data:
            predicted = self.compute_gravity_with_params(positions, full_params)
            
            # L4 误差 (索引 3)
            error_l4 = measured_efforts[3] - predicted[3]
            total_error += error_l4 ** 2 * 1.0  # 权重 1.0
            
            # L5 误差 (索引 4) - 权重更高，因为 L6 质量通过 L5 力矩体现
            error_l5 = measured_efforts[4] - predicted[4]
            total_error += error_l5 ** 2 * 2.0  # 权重 2.0
            
            # L6 误差 (索引 5) - 权重较低，因为 L6 yaw 轴力矩变化小
            error_l6 = measured_efforts[5] - predicted[5]
            total_error += error_l6 ** 2 * 0.5  # 权重 0.5
        
        return total_error
    
    def run_calibration(self, mode: str = 'full') -> Dict:
        """运行完整的标定流程
        
        Args:
            mode: 'quick', 'full', 'high', 'ultra'
        """
        mode_names = {'quick': '快速', 'full': '完整', 'high': '高精度', 'ultra': '超高精度'}
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  RS-A3 惯性参数标定程序 ({mode_names.get(mode, mode)}模式)')
        self.get_logger().info('=' * 60)
        
        if self.model is None:
            self.get_logger().error('Pinocchio 模型未初始化')
            return {}
        
        # 等待关节状态
        self.get_logger().info('等待关节状态...')
        start = time.time()
        while not self.state_received and (time.time() - start) < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.state_received:
            self.get_logger().error('无法获取关节状态')
            return {}
        
        # 生成测试配置
        test_configs = self.generate_test_configurations(mode)
        
        # 收集数据
        self.get_logger().info(f'\n开始数据采集 ({len(test_configs)} 个测试点)...')
        self.calibration_data.clear()
        
        for idx, config in enumerate(test_configs):
            self.get_logger().info(f'\n[{idx+1}/{len(test_configs)}] 目标: L2={config[1]:.2f}, L3={config[2]:.2f}, L4={config[3]:.2f}, L5={config[4]:.2f}')
            
            if not self.move_to_position(config):
                self.get_logger().warn('  移动失败，跳过')
                continue
            
            time.sleep(self.config.settle_time)
            
            positions, efforts = self.collect_samples()
            self.calibration_data.append((positions, efforts))
            
            self.get_logger().info(f'  实际: L2={positions[1]:.3f}, L3={positions[2]:.3f}')
            self.get_logger().info(f'  力矩: L2={efforts[1]:.4f}, L3={efforts[2]:.4f}, L4={efforts[3]:.4f}')
        
        # 回到 home
        self.get_logger().info('\n回到 Home 位置...')
        self.move_to_position(self.config.home_position)
        
        if len(self.calibration_data) < 10:
            self.get_logger().error('数据点太少，无法标定')
            return {}
        
        # 优化拟合
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  优化拟合惯性参数')
        self.get_logger().info('=' * 60)
        
        # 初始值（从 URDF 默认值）
        initial_params = np.array([
            0.8348, 0.095,      # L2: mass, com_x
            0.1976, -0.056, 0.049,  # L3: mass, com_x, com_y
            0.4606, -0.024, 0.031,  # L4: mass, com_x, com_y
            0.0180, 0.018,      # L5: mass, com_z
            0.5313, 0.070       # L6: mass, com_z
        ])
        
        # 参数边界
        bounds = [
            (0.1, 2.0), (-0.2, 0.2),     # L2
            (0.05, 0.5), (-0.15, 0.0), (0.0, 0.1),  # L3
            (0.1, 1.0), (-0.1, 0.0), (0.0, 0.1),   # L4
            (0.005, 0.1), (0.0, 0.05),   # L5
            (0.1, 1.0), (0.0, 0.15)      # L6
        ]
        
        self.get_logger().info('初始参数:')
        self.get_logger().info(f'  L2: mass={initial_params[0]:.4f}, com_x={initial_params[1]:.4f}')
        self.get_logger().info(f'  L3: mass={initial_params[2]:.4f}, com_x={initial_params[3]:.4f}, com_y={initial_params[4]:.4f}')
        self.get_logger().info(f'  L4: mass={initial_params[5]:.4f}, com_x={initial_params[6]:.4f}, com_y={initial_params[7]:.4f}')
        self.get_logger().info(f'  L5: mass={initial_params[8]:.4f}, com_z={initial_params[9]:.4f}')
        self.get_logger().info(f'  L6: mass={initial_params[10]:.4f}, com_z={initial_params[11]:.4f}')
        
        initial_error = self.objective_function(initial_params)
        self.get_logger().info(f'\n初始误差: {initial_error:.6f}')
        
        # 优化
        self.get_logger().info('开始优化...')
        result = minimize(
            self.objective_function,
            initial_params,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 500, 'disp': False}
        )
        
        optimized_params = result.x
        final_error = result.fun
        
        self.get_logger().info(f'优化完成! 最终误差: {final_error:.6f} (降低 {(1 - final_error/initial_error)*100:.1f}%)')
        
        # 计算 RMSE 和 R²
        n_samples = len(self.calibration_data) * 5  # L2-L6
        rmse = np.sqrt(final_error / n_samples)
        
        # 计算 SS_tot
        all_measured = []
        for _, efforts in self.calibration_data:
            all_measured.extend(efforts[1:6])
        mean_measured = np.mean(all_measured)
        ss_tot = sum((m - mean_measured) ** 2 for m in all_measured)
        r_squared = 1 - final_error / ss_tot if ss_tot > 0 else 0
        
        self.get_logger().info(f'RMSE: {rmse:.4f} Nm')
        self.get_logger().info(f'R²: {r_squared:.4f}')
        
        # 整理结果
        results = {
            'L2': {'mass': float(optimized_params[0]), 'com': [float(optimized_params[1]), 0.0, 0.0]},
            'L3': {'mass': float(optimized_params[2]), 'com': [float(optimized_params[3]), float(optimized_params[4]), 0.003]},
            'L4': {'mass': float(optimized_params[5]), 'com': [float(optimized_params[6]), float(optimized_params[7]), 0.0]},
            'L5': {'mass': float(optimized_params[8]), 'com': [0.004, 0.0, float(optimized_params[9])]},
            'L6': {'mass': float(optimized_params[10]), 'com': [0.0, -0.001, float(optimized_params[11])]},
            'calibration_info': {
                'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'num_samples': len(self.calibration_data),
                'rmse': float(rmse),
                'r_squared': float(r_squared)
            }
        }
        
        self.get_logger().info('\n优化后参数:')
        for joint in ['L2', 'L3', 'L4', 'L5', 'L6']:
            p = results[joint]
            self.get_logger().info(f'  {joint}: mass={p["mass"]:.4f}, com={p["com"]}')
        
        return results
    
    def generate_wrist_configurations_v2(self, arm_poses: List[Tuple[float, float]] = None) -> List[List[float]]:
        """生成腕部测试配置 - V2版本，支持多个大臂姿态
        
        改进点：
        1. 在多个 L2/L3 姿态下测试腕部，避免单一姿态下的碰撞
        2. 更系统的覆盖腕部工作空间
        
        Args:
            arm_poses: [(L2, L3), ...] 大臂姿态列表，默认使用安全姿态
        """
        configs = []
        
        # 默认的大臂安全姿态（避免碰撞的 L2/L3 组合）
        if arm_poses is None:
            arm_poses = [
                (0.785, -0.785),   # home 位置
                (0.5, -1.0),       # 大臂较高，小臂下垂
                (1.0, -1.2),       # 大臂向下，小臂下垂更多
                (0.4, -0.6),       # 大臂高位，小臂水平偏上
            ]
        
        self.get_logger().info(f'使用 {len(arm_poses)} 个大臂姿态进行腕部标定')
        
        # L4/L5 测试范围
        l4_range = np.linspace(0.0, 1.2, 7)    # 7点
        l5_range = np.linspace(-0.3, 1.2, 9)   # 9点（包含负角度）
        l6_range = np.linspace(-0.4, 0.4, 5)   # 5点
        
        for arm_idx, (l2, l3) in enumerate(arm_poses):
            self.get_logger().info(f'  姿态 {arm_idx+1}: L2={l2:.2f}, L3={l3:.2f}')
            
            # 在当前大臂姿态下测试腕部
            # L5 变化（核心：L6 质量通过 L5 力矩体现）
            for l5 in l5_range:
                cfg = [0.0, l2, l3, 0.5, l5, 0.0]
                if not self.check_wrist_collision(cfg):
                    configs.append(cfg)
            
            # L4 + L5 组合
            for l4 in l4_range[::2]:  # 稀疏采样
                for l5 in l5_range[::2]:
                    cfg = [0.0, l2, l3, l4, l5, 0.0]
                    if not self.check_wrist_collision(cfg):
                        configs.append(cfg)
            
            # L5 大角度 + L6 变化（标定 L6 的关键）
            l5_large = [0.8, 1.0, 1.2]
            for l5 in l5_large:
                for l6 in l6_range:
                    cfg = [0.0, l2, l3, 0.5, l5, l6]
                    if not self.check_wrist_collision(cfg):
                        configs.append(cfg)
        
        # 去重
        unique_configs = []
        for cfg in configs:
            is_dup = any(np.allclose(cfg, uc, atol=0.05) for uc in unique_configs)
            if not is_dup:
                unique_configs.append(cfg)
        
        self.get_logger().info(f'总计 {len(unique_configs)} 个测试点')
        return unique_configs
    
    def run_wrist_calibration(self, use_multi_pose: bool = True) -> Dict:
        """运行腕部关节 (L4/L5/L6) 专项标定
        
        保留已标定的 L2/L3 参数，只优化 L4/L5/L6 参数
        
        Args:
            use_multi_pose: 是否使用多大臂姿态模式（避免碰撞）
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('  RS-A3 腕部关节精细标定 (L4/L5/L6) - 改进版')
        self.get_logger().info('=' * 60)
        
        if self.model is None:
            self.get_logger().error('Pinocchio 模型未初始化')
            return {}
        
        # 从现有配置文件读取 L2/L3 参数
        existing_params = self.load_existing_params()
        if existing_params is None:
            self.get_logger().warn('无法读取现有参数，使用 URDF 默认值')
            fixed_l2l3_params = np.array([0.8348, 0.095, 0.1976, -0.056, 0.049])
        else:
            fixed_l2l3_params = np.array([
                existing_params['L2']['mass'], existing_params['L2']['com'][0],
                existing_params['L3']['mass'], existing_params['L3']['com'][0], existing_params['L3']['com'][1]
            ])
            self.get_logger().info(f'使用已标定的 L2/L3 参数:')
            self.get_logger().info(f'  L2: mass={fixed_l2l3_params[0]:.4f}, com_x={fixed_l2l3_params[1]:.4f}')
            self.get_logger().info(f'  L3: mass={fixed_l2l3_params[2]:.4f}, com_x={fixed_l2l3_params[3]:.4f}, com_y={fixed_l2l3_params[4]:.4f}')
        
        # 等待关节状态
        self.get_logger().info('等待关节状态...')
        start = time.time()
        while not self.state_received and (time.time() - start) < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.state_received:
            self.get_logger().error('无法获取关节状态')
            return {}
        
        # 生成腕部测试配置
        if use_multi_pose:
            self.get_logger().info('\n使用多大臂姿态模式（避免碰撞）')
            test_configs = self.generate_wrist_configurations_v2()
        else:
            self.get_logger().info('\n使用单一姿态模式')
            test_configs = self.generate_wrist_configurations()
        
        # 收集数据
        self.get_logger().info(f'\n开始腕部数据采集 ({len(test_configs)} 个测试点)...')
        self.calibration_data.clear()
        
        for idx, config in enumerate(test_configs):
            self.get_logger().info(f'\n[{idx+1}/{len(test_configs)}] 目标: L4={config[3]:.2f}, L5={config[4]:.2f}, L6={config[5]:.2f}')
            
            if not self.move_to_position(config):
                self.get_logger().warn('  移动失败，跳过')
                continue
            
            time.sleep(self.config.settle_time)
            
            positions, efforts = self.collect_samples()
            self.calibration_data.append((positions, efforts))
            
            self.get_logger().info(f'  实际: L4={positions[3]:.3f}, L5={positions[4]:.3f}, L6={positions[5]:.3f}')
            self.get_logger().info(f'  力矩: L4={efforts[3]:.4f}, L5={efforts[4]:.4f}, L6={efforts[5]:.4f}')
        
        # 回到 home
        self.get_logger().info('\n回到 Home 位置...')
        self.move_to_position(self.config.home_position)
        
        if len(self.calibration_data) < 10:
            self.get_logger().error('数据点太少，无法标定')
            return {}
        
        # 优化拟合
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  优化拟合腕部惯性参数 (L4/L5/L6)')
        self.get_logger().info('=' * 60)
        
        self.get_logger().info('\n标定策略说明:')
        self.get_logger().info('  - L6 是 yaw 轴（绕Z轴旋转），自身力矩变化很小')
        self.get_logger().info('  - L6 质量通过 L5 倾斜时的力矩变化来标定')
        self.get_logger().info('  - L5 误差权重设为 2.0（提高 L6 质量估计精度）')
        self.get_logger().info('  - L6 预期质量约 0.6 kg')
        
        # 腕部参数初始值 - 根据预期值调整
        initial_wrist_params = np.array([
            0.5, -0.05, 0.02,   # L4: mass, com_x, com_y (预估值)
            0.05, 0.025,        # L5: mass, com_z (轻量结构)
            0.6, 0.05           # L6: mass=0.6kg (预期值), com_z (质心偏移)
        ])
        
        # 参数边界 - 根据实际预期调整
        wrist_bounds = [
            (0.2, 1.2), (-0.12, 0.02), (-0.02, 0.08),   # L4: 允许更大范围
            (0.01, 0.3), (0.0, 0.10),                    # L5: 结构质量
            (0.3, 1.0), (-0.05, 0.15)                    # L6: 0.3-1.0kg, com_z 可正可负
        ]
        
        self.get_logger().info('初始腕部参数:')
        self.get_logger().info(f'  L4: mass={initial_wrist_params[0]:.4f}, com_x={initial_wrist_params[1]:.4f}, com_y={initial_wrist_params[2]:.4f}')
        self.get_logger().info(f'  L5: mass={initial_wrist_params[3]:.4f}, com_z={initial_wrist_params[4]:.4f}')
        self.get_logger().info(f'  L6: mass={initial_wrist_params[5]:.4f}, com_z={initial_wrist_params[6]:.4f}')
        
        initial_error = self.wrist_objective_function(initial_wrist_params, fixed_l2l3_params)
        self.get_logger().info(f'\n初始误差: {initial_error:.6f}')
        
        # 优化
        self.get_logger().info('开始优化...')
        result = minimize(
            lambda x: self.wrist_objective_function(x, fixed_l2l3_params),
            initial_wrist_params,
            method='L-BFGS-B',
            bounds=wrist_bounds,
            options={'maxiter': 500, 'disp': False}
        )
        
        optimized_wrist = result.x
        final_error = result.fun
        
        self.get_logger().info(f'优化完成! 最终误差: {final_error:.6f} (降低 {(1 - final_error/initial_error)*100:.1f}%)')
        
        # 计算 RMSE 和 R²
        n_samples = len(self.calibration_data) * 3  # L4-L6
        rmse = np.sqrt(final_error / n_samples)
        
        # 计算 SS_tot
        all_measured = []
        for _, efforts in self.calibration_data:
            all_measured.extend(efforts[3:6])
        mean_measured = np.mean(all_measured)
        ss_tot = sum((m - mean_measured) ** 2 for m in all_measured)
        r_squared = 1 - final_error / ss_tot if ss_tot > 0 else 0
        
        self.get_logger().info(f'RMSE: {rmse:.4f} Nm')
        self.get_logger().info(f'R²: {r_squared:.4f}')
        
        # 整理结果（保留 L2/L3，更新 L4/L5/L6）
        results = {
            'L2': {'mass': float(fixed_l2l3_params[0]), 'com': [float(fixed_l2l3_params[1]), 0.0, 0.0]},
            'L3': {'mass': float(fixed_l2l3_params[2]), 'com': [float(fixed_l2l3_params[3]), float(fixed_l2l3_params[4]), 0.003]},
            'L4': {'mass': float(optimized_wrist[0]), 'com': [float(optimized_wrist[1]), float(optimized_wrist[2]), 0.0]},
            'L5': {'mass': float(optimized_wrist[3]), 'com': [0.004, 0.0, float(optimized_wrist[4])]},
            'L6': {'mass': float(optimized_wrist[5]), 'com': [0.0, -0.001, float(optimized_wrist[6])]},
            'calibration_info': {
                'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'num_samples': len(self.calibration_data),
                'rmse': float(rmse),
                'r_squared': float(r_squared)
            }
        }
        
        self.get_logger().info('\n优化后腕部参数:')
        for joint in ['L4', 'L5', 'L6']:
            p = results[joint]
            self.get_logger().info(f'  {joint}: mass={p["mass"]:.4f}, com={p["com"]}')
        
        return results
    
    def generate_arm_configurations(self) -> List[List[float]]:
        """生成大臂关节 (L2/L3) 测试配置点
        
        L4/L5/L6 固定在 home 位置，只变化 L2/L3
        """
        configs = []
        home = np.array(self.config.home_position)
        
        # L2/L3 测试范围
        l2_range = np.linspace(0.3, 1.5, 15)     # 15点
        l3_range = np.linspace(-1.3, -0.3, 15)   # 15点
        
        # 第一阶段：L2 单独变化
        self.get_logger().info('生成 L2 单独测试点...')
        for l2 in l2_range:
            cfg = home.copy()
            cfg[1] = l2
            if not self.check_collision_with_base(cfg):
                configs.append(cfg.tolist())
        
        # 第二阶段：L3 单独变化
        self.get_logger().info('生成 L3 单独测试点...')
        for l3 in l3_range:
            cfg = home.copy()
            cfg[2] = l3
            if not self.check_collision_with_base(cfg):
                configs.append(cfg.tolist())
        
        # 第三阶段：L2+L3 网格组合
        self.get_logger().info('生成 L2+L3 网格组合测试点...')
        l2_grid = np.linspace(0.4, 1.4, 8)   # 8点
        l3_grid = np.linspace(-1.2, -0.4, 8) # 8点
        for l2 in l2_grid:
            for l3 in l3_grid:
                cfg = home.copy()
                cfg[1] = l2
                cfg[2] = l3
                if not self.check_collision_with_base(cfg):
                    configs.append(cfg.tolist())
        
        # 去重
        unique_configs = []
        for cfg in configs:
            is_dup = any(np.allclose(cfg, uc, atol=0.05) for uc in unique_configs)
            if not is_dup:
                unique_configs.append(cfg)
        
        self.get_logger().info(f'总计 {len(unique_configs)} 个大臂测试点')
        return unique_configs
    
    def arm_objective_function(self, arm_params: np.ndarray, fixed_wrist_params: np.ndarray) -> float:
        """大臂标定目标函数：只优化 L2/L3 参数
        
        Args:
            arm_params: [L2_mass, L2_com_x, L3_mass, L3_com_x, L3_com_y] (5个参数)
            fixed_wrist_params: [L4_mass, L4_com_x, L4_com_y, L5_mass, L5_com_z, L6_mass, L6_com_z] (7个参数)
        """
        # 组合完整参数
        full_params = np.concatenate([arm_params, fixed_wrist_params])
        
        total_error = 0.0
        for positions, measured_efforts in self.calibration_data:
            predicted = self.compute_gravity_with_params(positions, full_params)
            
            # 只计算 L2/L3 的误差（索引 1, 2）
            for i in range(1, 3):
                error = measured_efforts[i] - predicted[i]
                total_error += error ** 2
        
        return total_error
    
    def run_arm_calibration(self) -> Dict:
        """运行大臂关节 (L2/L3) 专项标定
        
        保留已标定的 L4/L5/L6 参数，只优化 L2/L3 参数
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('  RS-A3 大臂关节精细标定 (L2/L3)')
        self.get_logger().info('=' * 60)
        
        if self.model is None:
            self.get_logger().error('Pinocchio 模型未初始化')
            return {}
        
        # 从现有配置文件读取 L4/L5/L6 参数
        existing_params = self.load_existing_params()
        if existing_params is None:
            self.get_logger().warn('无法读取现有参数，使用 URDF 默认值')
            fixed_wrist_params = np.array([
                0.4606, -0.024, 0.031,  # L4
                0.0180, 0.018,          # L5
                0.5313, 0.070           # L6
            ])
        else:
            fixed_wrist_params = np.array([
                existing_params['L4']['mass'], existing_params['L4']['com'][0], existing_params['L4']['com'][1],
                existing_params['L5']['mass'], existing_params['L5']['com'][2],
                existing_params['L6']['mass'], existing_params['L6']['com'][2]
            ])
            self.get_logger().info(f'使用已标定的 L4/L5/L6 参数:')
            self.get_logger().info(f'  L4: mass={fixed_wrist_params[0]:.4f}, com_x={fixed_wrist_params[1]:.4f}')
            self.get_logger().info(f'  L5: mass={fixed_wrist_params[3]:.4f}, com_z={fixed_wrist_params[4]:.4f}')
            self.get_logger().info(f'  L6: mass={fixed_wrist_params[5]:.4f}, com_z={fixed_wrist_params[6]:.4f}')
        
        # 等待关节状态
        self.get_logger().info('等待关节状态...')
        start = time.time()
        while not self.state_received and (time.time() - start) < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.state_received:
            self.get_logger().error('无法获取关节状态')
            return {}
        
        # 生成大臂测试配置
        test_configs = self.generate_arm_configurations()
        
        # 收集数据
        self.get_logger().info(f'\n开始大臂数据采集 ({len(test_configs)} 个测试点)...')
        self.calibration_data.clear()
        
        for idx, config in enumerate(test_configs):
            self.get_logger().info(f'\n[{idx+1}/{len(test_configs)}] 目标: L2={config[1]:.2f}, L3={config[2]:.2f}')
            
            if not self.move_to_position(config):
                self.get_logger().warn('  移动失败，跳过')
                continue
            
            time.sleep(self.config.settle_time)
            
            positions, efforts = self.collect_samples()
            self.calibration_data.append((positions, efforts))
            
            self.get_logger().info(f'  实际: L2={positions[1]:.3f}, L3={positions[2]:.3f}')
            self.get_logger().info(f'  力矩: L2={efforts[1]:.4f}, L3={efforts[2]:.4f}')
        
        # 回到 home
        self.get_logger().info('\n回到 Home 位置...')
        self.move_to_position(self.config.home_position)
        
        if len(self.calibration_data) < 10:
            self.get_logger().error('数据点太少，无法标定')
            return {}
        
        # 优化拟合
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  优化拟合大臂惯性参数 (L2/L3)')
        self.get_logger().info('=' * 60)
        
        # 大臂参数初始值
        initial_arm_params = np.array([
            0.8348, 0.095,              # L2: mass, com_x
            0.1976, -0.056, 0.049       # L3: mass, com_x, com_y
        ])
        
        # 参数边界
        arm_bounds = [
            (0.3, 2.0), (0.05, 0.20),           # L2: mass, com_x
            (0.1, 0.8), (-0.18, -0.02), (0.0, 0.1)  # L3: mass, com_x, com_y
        ]
        
        self.get_logger().info('初始大臂参数:')
        self.get_logger().info(f'  L2: mass={initial_arm_params[0]:.4f}, com_x={initial_arm_params[1]:.4f}')
        self.get_logger().info(f'  L3: mass={initial_arm_params[2]:.4f}, com_x={initial_arm_params[3]:.4f}, com_y={initial_arm_params[4]:.4f}')
        
        initial_error = self.arm_objective_function(initial_arm_params, fixed_wrist_params)
        self.get_logger().info(f'\n初始误差: {initial_error:.6f}')
        
        # 优化
        self.get_logger().info('开始优化...')
        result = minimize(
            lambda x: self.arm_objective_function(x, fixed_wrist_params),
            initial_arm_params,
            method='L-BFGS-B',
            bounds=arm_bounds,
            options={'maxiter': 500, 'disp': False}
        )
        
        optimized_arm = result.x
        final_error = result.fun
        
        self.get_logger().info(f'优化完成! 最终误差: {final_error:.6f} (降低 {(1 - final_error/initial_error)*100:.1f}%)')
        
        # 计算 RMSE 和 R²
        n_samples = len(self.calibration_data) * 2  # L2/L3
        rmse = np.sqrt(final_error / n_samples)
        
        # 计算 SS_tot
        all_measured = []
        for _, efforts in self.calibration_data:
            all_measured.extend(efforts[1:3])
        mean_measured = np.mean(all_measured)
        ss_tot = sum((m - mean_measured) ** 2 for m in all_measured)
        r_squared = 1 - final_error / ss_tot if ss_tot > 0 else 0
        
        self.get_logger().info(f'RMSE: {rmse:.4f} Nm')
        self.get_logger().info(f'R²: {r_squared:.4f}')
        
        # 整理结果（更新 L2/L3，保留 L4/L5/L6）
        results = {
            'L2': {'mass': float(optimized_arm[0]), 'com': [float(optimized_arm[1]), 0.0, 0.0]},
            'L3': {'mass': float(optimized_arm[2]), 'com': [float(optimized_arm[3]), float(optimized_arm[4]), 0.003]},
            'L4': {'mass': float(fixed_wrist_params[0]), 'com': [float(fixed_wrist_params[1]), float(fixed_wrist_params[2]), 0.0]},
            'L5': {'mass': float(fixed_wrist_params[3]), 'com': [0.004, 0.0, float(fixed_wrist_params[4])]},
            'L6': {'mass': float(fixed_wrist_params[5]), 'com': [0.0, -0.001, float(fixed_wrist_params[6])]},
            'calibration_info': {
                'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'num_samples': len(self.calibration_data),
                'rmse': float(rmse),
                'r_squared': float(r_squared)
            }
        }
        
        self.get_logger().info('\n优化后大臂参数:')
        for joint in ['L2', 'L3']:
            p = results[joint]
            self.get_logger().info(f'  {joint}: mass={p["mass"]:.4f}, com={p["com"]}')
        
        return results
    
    def check_combo_collision(self, config: List[float]) -> bool:
        """检查联合标定配置是否会发生碰撞
        
        综合考虑 L2/L3/L4/L5 组合的碰撞风险
        
        Returns:
            True 如果可能发生碰撞，应该跳过
        """
        l2 = config[1]
        l3 = config[2]
        l4 = config[3]
        l5 = config[4]
        
        # 规则1: 底面碰撞 - L2 大（大臂向下）且 L3 接近 0（小臂水平）
        if l2 >= 1.2 and l3 > -0.5:
            return True
        
        # 规则2: L2 >= 1.3 时需要 L3 更低
        if l2 >= 1.3 and l3 > -0.7:
            return True
        
        # 规则3: L5 负角度时的大臂限制
        if l5 < -0.2 and l2 > 1.0:
            return True
        
        # 规则4: L4 接近极限且 L5 大时
        if abs(l4) > 1.1 and l5 > 0.9:
            return True
        
        # 规则5: L3 过于接近 0 时限制 L5 负角度
        if l3 > -0.5 and l5 < -0.2:
            return True
        
        return False
    
    def generate_combo_configurations(self) -> List[List[float]]:
        """生成 L2-L5 联合标定的测试配置
        
        覆盖 L2/L3/L4/L5 的全姿态空间，用于同时优化这 4 个关节的惯性参数
        """
        configs = []
        
        # L2/L3 大臂姿态（4x3=12 组合）
        l2_range = [0.4, 0.7, 1.0, 1.3]
        l3_range = [-1.2, -0.9, -0.6]
        
        # L4/L5 腕部姿态（3x4=12 组合）
        l4_range = [0.0, 0.6, 1.2]
        l5_range = [-0.3, 0.3, 0.8, 1.2]
        
        # 全组合
        for l2 in l2_range:
            for l3 in l3_range:
                for l4 in l4_range:
                    for l5 in l5_range:
                        cfg = [0.0, l2, l3, l4, l5, 0.0]
                        if not self.check_combo_collision(cfg):
                            configs.append(cfg)
        
        # 去重
        unique_configs = []
        for cfg in configs:
            is_dup = any(np.allclose(cfg, uc, atol=0.05) for uc in unique_configs)
            if not is_dup:
                unique_configs.append(cfg)
        
        self.get_logger().info(f'联合标定: 生成 {len(unique_configs)} 个测试点')
        return unique_configs
    
    def combo_objective_function(self, params: np.ndarray, fixed_l6_params: np.ndarray) -> float:
        """联合标定目标函数：优化 L2-L5，固定 L6
        
        Args:
            params: [L2_mass, L2_com_x,           # 2
                     L3_mass, L3_com_x, L3_com_y,  # 3
                     L4_mass, L4_com_x, L4_com_y,  # 3
                     L5_mass, L5_com_z]            # 2  共 10 个
            fixed_l6_params: [L6_mass, L6_com_z]   # URDF 值
        """
        # 组合完整 12 参数
        full_params = np.concatenate([params, fixed_l6_params])
        
        total_error = 0.0
        for positions, measured_efforts in self.calibration_data:
            predicted = self.compute_gravity_with_params(positions, full_params)
            # 计算 L2-L5 误差（索引 1-4）
            for i in range(1, 5):
                error = measured_efforts[i] - predicted[i]
                total_error += error ** 2
        
        return total_error
    
    def run_combo_calibration(self) -> Dict:
        """运行 L2-L5 联合标定（固定 L6 为 URDF 值）
        
        核心思想：L6 的可辨识性较差（yaw 轴自身力矩变化小），
        使用 URDF 中的质量作为固定值，联合优化 L2-L5 共 10 个参数
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('  RS-A3 L2-L5 联合标定（固定 L6 为 URDF 值）')
        self.get_logger().info('=' * 60)
        
        if self.model is None:
            self.get_logger().error('Pinocchio 模型未初始化')
            return {}
        
        # 固定 L6 参数（URDF 值）
        # L6 (l5_l6_urdf_asm): mass=0.5317, com_z=-0.070
        fixed_l6_params = np.array([0.5317, -0.070])
        self.get_logger().info(f'固定 L6 参数 (URDF): mass={fixed_l6_params[0]:.4f} kg, com_z={fixed_l6_params[1]:.4f} m')
        
        # 等待关节状态
        self.get_logger().info('等待关节状态...')
        start = time.time()
        while not self.state_received and (time.time() - start) < 10:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.state_received:
            self.get_logger().error('无法获取关节状态')
            return {}
        
        # 生成联合测试配置
        test_configs = self.generate_combo_configurations()
        
        # 收集数据
        self.get_logger().info(f'\n开始联合数据采集 ({len(test_configs)} 个测试点)...')
        self.calibration_data.clear()
        
        for idx, config in enumerate(test_configs):
            self.get_logger().info(f'\n[{idx+1}/{len(test_configs)}] 目标: L2={config[1]:.2f}, L3={config[2]:.2f}, L4={config[3]:.2f}, L5={config[4]:.2f}')
            
            if not self.move_to_position(config):
                self.get_logger().warn('  移动失败，跳过')
                continue
            
            time.sleep(self.config.settle_time)
            
            positions, efforts = self.collect_samples()
            self.calibration_data.append((positions, efforts))
            
            self.get_logger().info(f'  实际: L2={positions[1]:.3f}, L3={positions[2]:.3f}, L4={positions[3]:.3f}, L5={positions[4]:.3f}')
            self.get_logger().info(f'  力矩: L2={efforts[1]:.4f}, L3={efforts[2]:.4f}, L4={efforts[3]:.4f}, L5={efforts[4]:.4f}')
        
        # 回到 home
        self.get_logger().info('\n回到 Home 位置...')
        self.move_to_position(self.config.home_position)
        
        if len(self.calibration_data) < 20:
            self.get_logger().error('数据点太少，无法标定')
            return {}
        
        # 优化拟合
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('  优化拟合 L2-L5 惯性参数')
        self.get_logger().info('=' * 60)
        
        # 初始参数（10 个）- 基于 URDF 值
        initial_params = np.array([
            0.95, 0.095,              # L2: mass, com_x (URDF: 0.946, 0.095)
            0.40, -0.056, 0.049,      # L3: mass, com_x, com_y (URDF: 0.406, -0.056, 0.049)
            0.45, -0.024, 0.031,      # L4: mass, com_x, com_y (URDF: 0.447, -0.024, 0.031)
            0.02, 0.018               # L5: mass, com_z (URDF: 0.018, 0.018)
        ])
        
        # 参数边界
        bounds = [
            (0.5, 2.0), (0.05, 0.15),              # L2: 允许较大范围
            (0.1, 0.8), (-0.15, 0.0), (0.0, 0.1),  # L3
            (0.2, 0.8), (-0.08, 0.05), (-0.02, 0.08), # L4
            (0.01, 0.2), (0.0, 0.06)               # L5
        ]
        
        self.get_logger().info('初始参数:')
        self.get_logger().info(f'  L2: mass={initial_params[0]:.4f}, com_x={initial_params[1]:.4f}')
        self.get_logger().info(f'  L3: mass={initial_params[2]:.4f}, com_x={initial_params[3]:.4f}, com_y={initial_params[4]:.4f}')
        self.get_logger().info(f'  L4: mass={initial_params[5]:.4f}, com_x={initial_params[6]:.4f}, com_y={initial_params[7]:.4f}')
        self.get_logger().info(f'  L5: mass={initial_params[8]:.4f}, com_z={initial_params[9]:.4f}')
        self.get_logger().info(f'  L6: mass={fixed_l6_params[0]:.4f} (固定), com_z={fixed_l6_params[1]:.4f} (固定)')
        
        initial_error = self.combo_objective_function(initial_params, fixed_l6_params)
        self.get_logger().info(f'\n初始误差: {initial_error:.6f}')
        
        # 优化
        self.get_logger().info('开始优化...')
        result = minimize(
            lambda x: self.combo_objective_function(x, fixed_l6_params),
            initial_params,
            method='L-BFGS-B',
            bounds=bounds,
            options={'maxiter': 500, 'disp': False}
        )
        
        optimized_params = result.x
        final_error = result.fun
        
        self.get_logger().info(f'优化完成! 最终误差: {final_error:.6f} (降低 {(1 - final_error/initial_error)*100:.1f}%)')
        
        # 计算 RMSE 和 R²
        n_samples = len(self.calibration_data) * 4  # L2-L5
        rmse = np.sqrt(final_error / n_samples)
        
        # 计算 SS_tot
        all_measured = []
        for _, efforts in self.calibration_data:
            all_measured.extend(efforts[1:5])
        mean_measured = np.mean(all_measured)
        ss_tot = sum((m - mean_measured) ** 2 for m in all_measured)
        r_squared = 1 - final_error / ss_tot if ss_tot > 0 else 0
        
        self.get_logger().info(f'RMSE: {rmse:.4f} Nm')
        self.get_logger().info(f'R²: {r_squared:.4f}')
        
        # 整理结果
        results = {
            'L2': {'mass': float(optimized_params[0]), 'com': [float(optimized_params[1]), 0.0, 0.0]},
            'L3': {'mass': float(optimized_params[2]), 'com': [float(optimized_params[3]), float(optimized_params[4]), 0.003]},
            'L4': {'mass': float(optimized_params[5]), 'com': [float(optimized_params[6]), float(optimized_params[7]), 0.0]},
            'L5': {'mass': float(optimized_params[8]), 'com': [0.004, 0.0, float(optimized_params[9])]},
            'L6': {'mass': float(fixed_l6_params[0]), 'com': [0.0, -0.001, float(fixed_l6_params[1])]},  # URDF 固定值
            'calibration_info': {
                'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'num_samples': len(self.calibration_data),
                'rmse': float(rmse),
                'r_squared': float(r_squared)
            }
        }
        
        self.get_logger().info('\n优化后参数:')
        for joint in ['L2', 'L3', 'L4', 'L5']:
            p = results[joint]
            self.get_logger().info(f'  {joint}: mass={p["mass"]:.4f}, com={p["com"]}')
        self.get_logger().info(f'  L6: mass={results["L6"]["mass"]:.4f} (URDF固定), com={results["L6"]["com"]}')
        
        return results
    
    def load_existing_params(self) -> Dict:
        """从配置文件加载现有的惯性参数"""
        try:
            config_path = self.config.output_path
            with open(config_path, 'r') as f:
                lines = f.readlines()
            
            params = {'L2': {}, 'L3': {}, 'L4': {}, 'L5': {}, 'L6': {}}
            current_joint = None
            
            for line in lines:
                line = line.strip()
                if line.startswith('L') and ':' in line and 'mass' not in line and 'com' not in line:
                    for j in ['L2', 'L3', 'L4', 'L5', 'L6']:
                        if line.startswith(j + ':'):
                            current_joint = j
                            break
                elif current_joint and 'mass:' in line:
                    mass_str = line.split('mass:')[1].strip()
                    params[current_joint]['mass'] = float(mass_str)
                elif current_joint and 'com:' in line:
                    com_str = line.split('com:')[1].strip()
                    com_str = com_str.strip('[]')
                    params[current_joint]['com'] = [float(x.strip()) for x in com_str.split(',')]
            
            # 验证参数完整性
            for j in ['L2', 'L3']:
                if 'mass' not in params[j] or 'com' not in params[j]:
                    return None
            
            return params
        except Exception as e:
            self.get_logger().warn(f'读取配置文件失败: {e}')
            return None
    
    def save_results(self, results: Dict, output_path: str):
        """保存标定结果到 YAML 文件"""
        yaml_content = f"""# RS-A3 机械臂惯性参数配置文件
# 用于 Pinocchio 重力补偿计算
# 这些参数通过标定程序拟合得到，覆盖 URDF 默认值
#
# 标定日期: {results['calibration_info']['date']}
# 标定程序: scripts/inertia_calibration.py
# RMSE: {results['calibration_info']['rmse']:.4f} Nm
# R²: {results['calibration_info']['r_squared']:.4f}

# 是否启用标定后的惯性参数
use_calibrated_params: true

# 各连杆惯性参数
inertia_params:
  # L2 - 大臂
  L2:
    mass: {results['L2']['mass']:.4f}
    com: {results['L2']['com']}
  
  # L3 - 小臂
  L3:
    mass: {results['L3']['mass']:.4f}
    com: {results['L3']['com']}
  
  # L4 - 腕部 roll
  L4:
    mass: {results['L4']['mass']:.4f}
    com: {results['L4']['com']}
  
  # L5 - 腕部 pitch
  L5:
    mass: {results['L5']['mass']:.4f}
    com: {results['L5']['com']}
  
  # L6 - 末端 yaw
  L6:
    mass: {results['L6']['mass']:.4f}
    com: {results['L6']['com']}

# 标定统计信息
calibration_info:
  date: "{results['calibration_info']['date']}"
  num_samples: {results['calibration_info']['num_samples']}
  rmse: {results['calibration_info']['rmse']:.4f}
  r_squared: {results['calibration_info']['r_squared']:.4f}
"""
        
        with open(output_path, 'w') as f:
            f.write(yaml_content)
        
        self.get_logger().info(f'\n结果已保存到: {output_path}')


def main():
    parser = argparse.ArgumentParser(description='RS-A3 惯性参数标定')
    parser.add_argument('--quick', action='store_true', help='快速模式（~20个测试点）')
    parser.add_argument('--high', action='store_true', help='高精度模式（~80个测试点）')
    parser.add_argument('--ultra', action='store_true', help='超高精度模式（~120个测试点）')
    parser.add_argument('--wrist', action='store_true', help='腕部标定模式（只标定L4/L5/L6）')
    parser.add_argument('--wrist-v2', action='store_true', dest='wrist_v2',
                        help='腕部标定V2模式（多大臂姿态避免碰撞，推荐）')
    parser.add_argument('--arm', action='store_true', help='大臂标定模式（只标定L2/L3）')
    parser.add_argument('--combo', action='store_true', 
                        help='L2-L5联合标定模式（固定L6为URDF值，推荐）')
    parser.add_argument('--single-pose', action='store_true', dest='single_pose',
                        help='腕部标定时使用单一大臂姿态（默认使用多姿态）')
    parser.add_argument('--samples', type=int, default=40, help='每个测试点采样次数（默认40）')
    parser.add_argument('--output', type=str, 
                        default='/home/wy/RS/A3/rs_a3_description/config/inertia_params.yaml',
                        help='输出配置文件路径')
    parser.add_argument('--urdf', type=str,
                        default='/home/wy/RS/A3/rs_a3_description/urdf/rs_a3.urdf',
                        help='URDF 文件路径')
    
    args = parser.parse_args()
    
    # 确定模式
    if args.ultra:
        mode = 'ultra'
    elif args.high:
        mode = 'high'
    elif args.quick:
        mode = 'quick'
    else:
        mode = 'full'
    
    if not PINOCCHIO_AVAILABLE:
        return
    
    rclpy.init()
    
    config = CalibrationConfig(
        urdf_path=args.urdf,
        output_path=args.output,
        samples_per_config=args.samples
    )
    
    calibrator = InertiaCalibrator(config)
    
    try:
        if args.combo:
            # L2-L5 联合标定模式（固定 L6）
            results = calibrator.run_combo_calibration()
            mode_desc = "联合 (L2-L5, L6固定)"
        elif args.arm:
            # 大臂标定模式
            results = calibrator.run_arm_calibration()
            mode_desc = "大臂 (L2/L3)"
        elif args.wrist or args.wrist_v2:
            # 腕部标定模式
            use_multi_pose = not args.single_pose  # 默认使用多姿态
            results = calibrator.run_wrist_calibration(use_multi_pose=use_multi_pose)
            mode_desc = "腕部 (L4/L5/L6)"
            if use_multi_pose:
                mode_desc += " [多姿态]"
        else:
            # 完整标定模式
            results = calibrator.run_calibration(mode=mode)
            mode_desc = "完整"
        
        if results:
            calibrator.save_results(results, args.output)
            print("\n" + "=" * 60)
            print(f"  {mode_desc}标定完成!")
            print("  请重启控制器以应用新参数")
            print("=" * 60)
        else:
            print("\n标定失败")
    
    except KeyboardInterrupt:
        print("\n标定被中断")
    
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
