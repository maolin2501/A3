#!/usr/bin/env python3
"""
视觉伺服控制节点
基于位置的视觉伺服 (PBVS) 实现末端跟踪
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose_stamped
import numpy as np
import math


class VisualServoNode(Node):
    """视觉伺服控制节点"""
    
    def __init__(self):
        super().__init__('visual_servo')
        
        # 声明参数
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'end_effector')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('kp_linear', 0.5)
        self.declare_parameter('kp_angular', 0.3)
        self.declare_parameter('max_linear_vel', 0.1)
        self.declare_parameter('max_angular_vel', 0.3)
        self.declare_parameter('approach_distance', 0.10)  # 接近距离
        self.declare_parameter('grasp_distance', 0.02)     # 抓取距离
        self.declare_parameter('control_rate', 50.0)
        
        # 获取参数
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.approach_distance = self.get_parameter('approach_distance').value
        self.grasp_distance = self.get_parameter('grasp_distance').value
        self.control_rate = self.get_parameter('control_rate').value
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅器
        self.target_sub = self.create_subscription(
            PoseStamped, '/target/pose', self.target_callback, 10)
        
        # 发布器
        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.status_pub = self.create_publisher(String, '/visual_servo/status', 10)
        self.target_base_pub = self.create_publisher(
            PoseStamped, '/target/pose_base', 10)
        
        # 服务
        self.enable_srv = self.create_service(
            SetBool, '/visual_servo/enable', self.enable_callback)
        self.start_approach_srv = self.create_service(
            Trigger, '/visual_servo/start_approach', self.start_approach_callback)
        self.stop_srv = self.create_service(
            Trigger, '/visual_servo/stop', self.stop_callback)
        
        # 状态变量
        self.enabled = False
        self.approaching = False
        self.target_pose_camera = None  # 相机坐标系下的目标位姿
        self.target_pose_base = None    # 基座坐标系下的目标位姿
        self.current_ee_pose = None     # 当前末端位姿
        
        # 控制循环
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self.control_loop)
        
        self.get_logger().info('视觉伺服节点已启动')
        self.publish_status('就绪')
    
    def target_callback(self, msg: PoseStamped):
        """目标位姿回调"""
        self.target_pose_camera = msg
        
        # 转换到基座坐标系
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            self.target_pose_base = do_transform_pose_stamped(msg, transform)
            self.target_pose_base.header.frame_id = self.base_frame
            
            # 发布基座坐标系下的目标位置
            self.target_base_pub.publish(self.target_pose_base)
            
            # 广播目标TF
            self._broadcast_target_tf()
            
        except Exception as e:
            self.get_logger().warn(f'目标坐标转换失败: {e}')
    
    def _broadcast_target_tf(self):
        """广播目标位置的TF"""
        if self.target_pose_base is None:
            return
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = 'target_object'
        t.transform.translation.x = self.target_pose_base.pose.position.x
        t.transform.translation.y = self.target_pose_base.pose.position.y
        t.transform.translation.z = self.target_pose_base.pose.position.z
        t.transform.rotation = self.target_pose_base.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def enable_callback(self, request, response):
        """启用/禁用服务回调"""
        self.enabled = request.data
        if self.enabled:
            self.publish_status('已启用')
            self.get_logger().info('视觉伺服已启用')
        else:
            self.approaching = False
            self.publish_status('已禁用')
            self.get_logger().info('视觉伺服已禁用')
        response.success = True
        response.message = '已启用' if self.enabled else '已禁用'
        return response
    
    def start_approach_callback(self, request, response):
        """开始接近服务回调"""
        if self.target_pose_base is None:
            response.success = False
            response.message = '未选择目标'
            return response
        
        self.enabled = True
        self.approaching = True
        self.publish_status('正在接近目标')
        self.get_logger().info('开始接近目标')
        response.success = True
        response.message = '开始接近'
        return response
    
    def stop_callback(self, request, response):
        """停止服务回调"""
        self.approaching = False
        self._send_zero_twist()
        self.publish_status('已停止')
        self.get_logger().info('已停止运动')
        response.success = True
        response.message = '已停止'
        return response
    
    def control_loop(self):
        """控制循环"""
        if not self.enabled or not self.approaching:
            return
        
        if self.target_pose_base is None:
            return
        
        # 获取当前末端位姿
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            current_z = transform.transform.translation.z
            
        except Exception as e:
            self.get_logger().warn(f'获取末端位姿失败: {e}')
            return
        
        # 计算目标位置（在物体上方 approach_distance 处）
        target_x = self.target_pose_base.pose.position.x
        target_y = self.target_pose_base.pose.position.y
        target_z = self.target_pose_base.pose.position.z + self.approach_distance
        
        # 计算位置误差
        error_x = target_x - current_x
        error_y = target_y - current_y
        error_z = target_z - current_z
        
        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        # 检查是否到达
        if distance < self.grasp_distance:
            self.approaching = False
            self._send_zero_twist()
            self.publish_status('已到达目标位置')
            self.get_logger().info('已到达目标位置，可以执行抓取')
            return
        
        # 计算速度命令 (PBVS)
        vel_x = self.kp_linear * error_x
        vel_y = self.kp_linear * error_y
        vel_z = self.kp_linear * error_z
        
        # 限制最大速度
        vel_magnitude = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        if vel_magnitude > self.max_linear_vel:
            scale = self.max_linear_vel / vel_magnitude
            vel_x *= scale
            vel_y *= scale
            vel_z *= scale
        
        # 发布速度命令
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame
        twist.twist.linear.x = vel_x
        twist.twist.linear.y = vel_y
        twist.twist.linear.z = vel_z
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        
        self.twist_pub.publish(twist)
        
        # 更新状态
        self.publish_status(f'接近中: 距离 {distance:.3f}m')
    
    def _send_zero_twist(self):
        """发送零速度命令"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame
        self.twist_pub.publish(twist)
    
    def publish_status(self, status: str):
        """发布状态"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisualServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
