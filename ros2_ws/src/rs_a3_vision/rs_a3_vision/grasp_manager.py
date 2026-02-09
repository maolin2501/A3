#!/usr/bin/env python3
"""
抓取管理器节点
协调视觉检测、运动规划和夹爪控制
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math
from enum import Enum


class GraspState(Enum):
    """抓取状态机"""
    IDLE = 0           # 空闲
    SELECTING = 1      # 等待选择物体
    APPROACHING = 2    # 接近物体
    GRASPING = 3       # 执行抓取
    LIFTING = 4        # 提起物体
    PLACING = 5        # 放置物体
    RETURNING = 6      # 返回初始位置
    ERROR = 7          # 错误状态


class GraspManagerNode(Node):
    """抓取管理器节点"""
    
    def __init__(self):
        super().__init__('grasp_manager')
        
        # 声明参数
        self.declare_parameter('lift_height', 0.1)       # 提起高度
        self.declare_parameter('approach_speed', 0.05)   # 接近速度
        self.declare_parameter('grasp_timeout', 30.0)    # 抓取超时
        
        # 获取参数
        self.lift_height = self.get_parameter('lift_height').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.grasp_timeout = self.get_parameter('grasp_timeout').value
        
        # 订阅器
        self.target_pose_sub = self.create_subscription(
            PoseStamped, '/target/pose_base', self.target_pose_callback, 10)
        self.servo_status_sub = self.create_subscription(
            String, '/visual_servo/status', self.servo_status_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # 发布器
        self.state_pub = self.create_publisher(String, '/grasp/state', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper/command', 10)
        
        # 服务
        self.start_grasp_srv = self.create_service(
            Trigger, '/grasp/start', self.start_grasp_callback)
        self.abort_srv = self.create_service(
            Trigger, '/grasp/abort', self.abort_callback)
        self.reset_srv = self.create_service(
            Trigger, '/grasp/reset', self.reset_callback)
        
        # 服务客户端
        self.servo_enable_client = self.create_client(
            SetBool, '/visual_servo/enable')
        self.servo_approach_client = self.create_client(
            Trigger, '/visual_servo/start_approach')
        self.servo_stop_client = self.create_client(
            Trigger, '/visual_servo/stop')
        
        # Action 客户端 (轨迹执行)
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        
        # 状态变量
        self.state = GraspState.IDLE
        self.target_pose = None
        self.current_joint_state = None
        self.servo_status = ''
        
        # 关节名称
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 
                           'L4_joint', 'L5_joint', 'L6_joint']
        
        # 预定义位置
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pre_grasp_position = None  # 抓取前位置
        
        # 状态机定时器
        self.state_timer = self.create_timer(0.1, self.state_machine_update)
        
        self.get_logger().info('抓取管理器已启动')
        self.publish_state()
    
    def target_pose_callback(self, msg: PoseStamped):
        """目标位姿回调"""
        self.target_pose = msg
    
    def servo_status_callback(self, msg: String):
        """伺服状态回调"""
        self.servo_status = msg.data
    
    def joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        self.current_joint_state = msg
    
    def start_grasp_callback(self, request, response):
        """开始抓取服务回调"""
        if self.state != GraspState.IDLE and self.state != GraspState.SELECTING:
            response.success = False
            response.message = f'当前状态不允许开始: {self.state.name}'
            return response
        
        if self.target_pose is None:
            response.success = False
            response.message = '未选择目标物体'
            return response
        
        self.state = GraspState.APPROACHING
        self.publish_state()
        
        # 启动视觉伺服
        self._call_servo_approach()
        
        response.success = True
        response.message = '开始抓取序列'
        self.get_logger().info('开始抓取序列')
        return response
    
    def abort_callback(self, request, response):
        """中止抓取服务回调"""
        self._abort_grasp()
        response.success = True
        response.message = '已中止'
        return response
    
    def reset_callback(self, request, response):
        """重置服务回调"""
        self._abort_grasp()
        self.state = GraspState.IDLE
        self.target_pose = None
        self.publish_state()
        response.success = True
        response.message = '已重置'
        return response
    
    def state_machine_update(self):
        """状态机更新"""
        if self.state == GraspState.IDLE:
            pass
        
        elif self.state == GraspState.SELECTING:
            # 等待用户选择物体
            if self.target_pose is not None:
                self.get_logger().info('物体已选择')
        
        elif self.state == GraspState.APPROACHING:
            # 检查是否到达
            if '已到达' in self.servo_status:
                self.get_logger().info('已到达目标位置，准备抓取')
                self.state = GraspState.GRASPING
                self.publish_state()
                self._execute_grasp()
        
        elif self.state == GraspState.GRASPING:
            # 抓取执行中
            pass
        
        elif self.state == GraspState.LIFTING:
            # 提起执行中
            pass
        
        elif self.state == GraspState.PLACING:
            # 放置执行中
            pass
        
        elif self.state == GraspState.RETURNING:
            # 返回执行中
            pass
        
        elif self.state == GraspState.ERROR:
            pass
    
    def _call_servo_approach(self):
        """调用视觉伺服接近"""
        if not self.servo_approach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('视觉伺服服务不可用')
            self.state = GraspState.ERROR
            return
        
        request = Trigger.Request()
        future = self.servo_approach_client.call_async(request)
        future.add_done_callback(self._servo_approach_done)
    
    def _servo_approach_done(self, future):
        """伺服接近完成回调"""
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f'视觉伺服启动失败: {response.message}')
                self.state = GraspState.ERROR
        except Exception as e:
            self.get_logger().error(f'视觉伺服调用异常: {e}')
            self.state = GraspState.ERROR
    
    def _execute_grasp(self):
        """执行抓取动作"""
        self.get_logger().info('执行抓取: 闭合夹爪')
        
        # 发送夹爪闭合命令
        gripper_msg = Bool()
        gripper_msg.data = True  # True = 闭合
        self.gripper_pub.publish(gripper_msg)
        
        # 延迟后进入提起状态
        self.create_timer(1.0, self._start_lifting, oneshot=True)
    
    def _start_lifting(self):
        """开始提起物体"""
        self.state = GraspState.LIFTING
        self.publish_state()
        
        self.get_logger().info(f'提起物体 {self.lift_height}m')
        
        # 获取当前关节位置并计算提起位置
        if self.current_joint_state is None:
            self.get_logger().error('无法获取当前关节状态')
            self.state = GraspState.ERROR
            return
        
        # 这里简化处理：直接发送一个向上的运动
        # 实际应用中应该使用 MoveIt 进行规划
        self._move_to_home()
    
    def _move_to_home(self):
        """移动到初始位置"""
        self.get_logger().info('返回初始位置')
        self.state = GraspState.RETURNING
        self.publish_state()
        
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('轨迹控制服务不可用')
            self.state = GraspState.ERROR
            return
        
        # 创建轨迹目标
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.home_position
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=3, nanosec=0)
        
        goal.trajectory.points = [point]
        
        # 发送目标
        send_goal_future = self.trajectory_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self._trajectory_goal_response)
    
    def _trajectory_goal_response(self, future):
        """轨迹目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('轨迹目标被拒绝')
            self.state = GraspState.ERROR
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._trajectory_result)
    
    def _trajectory_result(self, future):
        """轨迹执行结果回调"""
        result = future.result().result
        self.get_logger().info('轨迹执行完成')
        
        # 打开夹爪
        gripper_msg = Bool()
        gripper_msg.data = False  # False = 打开
        self.gripper_pub.publish(gripper_msg)
        
        # 返回空闲状态
        self.state = GraspState.IDLE
        self.target_pose = None
        self.publish_state()
        self.get_logger().info('抓取序列完成')
    
    def _abort_grasp(self):
        """中止抓取"""
        # 停止视觉伺服
        if self.servo_stop_client.wait_for_service(timeout_sec=0.5):
            request = Trigger.Request()
            self.servo_stop_client.call_async(request)
        
        # 打开夹爪
        gripper_msg = Bool()
        gripper_msg.data = False
        self.gripper_pub.publish(gripper_msg)
        
        self.state = GraspState.IDLE
        self.publish_state()
        self.get_logger().info('抓取已中止')
    
    def publish_state(self):
        """发布当前状态"""
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraspManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
