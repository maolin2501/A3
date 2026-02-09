#!/usr/bin/env python3
"""
手眼标定节点
支持 Eye-on-Base (相机固定) 和 Eye-in-Hand (相机在末端) 两种配置
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from std_srvs.srv import Trigger
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class HandEyeCalibrationNode(Node):
    """手眼标定节点"""
    
    def __init__(self):
        super().__init__('hand_eye_calibration')
        
        # 声明参数
        self.declare_parameter('calibration_type', 'eye_on_base')  # eye_on_base 或 eye_in_hand
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'end_effector')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        
        # 手动标定参数（如果没有标定文件）
        self.declare_parameter('translation_x', 0.0)
        self.declare_parameter('translation_y', 0.0)
        self.declare_parameter('translation_z', 0.5)
        self.declare_parameter('rotation_x', 0.0)
        self.declare_parameter('rotation_y', 0.0)
        self.declare_parameter('rotation_z', 0.0)
        self.declare_parameter('rotation_w', 1.0)
        
        # 获取参数
        self.calibration_type = self.get_parameter('calibration_type').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ee_frame = self.get_parameter('ee_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # TF
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 服务
        self.reload_srv = self.create_service(
            Trigger, '/hand_eye/reload', self.reload_callback)
        
        # 加载标定
        self._load_calibration()
        
        # 发布静态变换
        self._publish_calibration_tf()
        
        self.get_logger().info(
            f'手眼标定节点已启动 (类型: {self.calibration_type})'
        )
    
    def _load_calibration(self):
        """加载标定参数"""
        if self.calibration_file and os.path.exists(self.calibration_file):
            self.get_logger().info(f'从文件加载标定: {self.calibration_file}')
            try:
                with open(self.calibration_file, 'r') as f:
                    calib = yaml.safe_load(f)
                
                self.translation = [
                    calib.get('translation', {}).get('x', 0.0),
                    calib.get('translation', {}).get('y', 0.0),
                    calib.get('translation', {}).get('z', 0.5),
                ]
                self.rotation = [
                    calib.get('rotation', {}).get('x', 0.0),
                    calib.get('rotation', {}).get('y', 0.0),
                    calib.get('rotation', {}).get('z', 0.0),
                    calib.get('rotation', {}).get('w', 1.0),
                ]
                
                if 'calibration_type' in calib:
                    self.calibration_type = calib['calibration_type']
                    
            except Exception as e:
                self.get_logger().error(f'加载标定文件失败: {e}')
                self._load_default_calibration()
        else:
            self._load_default_calibration()
    
    def _load_default_calibration(self):
        """加载默认/参数标定"""
        self.get_logger().info('使用参数中的标定值')
        self.translation = [
            self.get_parameter('translation_x').value,
            self.get_parameter('translation_y').value,
            self.get_parameter('translation_z').value,
        ]
        self.rotation = [
            self.get_parameter('rotation_x').value,
            self.get_parameter('rotation_y').value,
            self.get_parameter('rotation_z').value,
            self.get_parameter('rotation_w').value,
        ]
    
    def _publish_calibration_tf(self):
        """发布标定的静态变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        
        if self.calibration_type == 'eye_on_base':
            # 相机固定在基座上：base_link -> camera_link
            t.header.frame_id = self.base_frame
            t.child_frame_id = 'camera_link'
        else:  # eye_in_hand
            # 相机在末端上：end_effector -> camera_link
            t.header.frame_id = self.ee_frame
            t.child_frame_id = 'camera_link'
        
        t.transform.translation.x = self.translation[0]
        t.transform.translation.y = self.translation[1]
        t.transform.translation.z = self.translation[2]
        t.transform.rotation.x = self.rotation[0]
        t.transform.rotation.y = self.rotation[1]
        t.transform.rotation.z = self.rotation[2]
        t.transform.rotation.w = self.rotation[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info(
            f'发布标定变换: {t.header.frame_id} -> {t.child_frame_id}'
        )
        self.get_logger().info(
            f'  平移: [{self.translation[0]:.4f}, {self.translation[1]:.4f}, {self.translation[2]:.4f}]'
        )
        self.get_logger().info(
            f'  旋转: [{self.rotation[0]:.4f}, {self.rotation[1]:.4f}, '
            f'{self.rotation[2]:.4f}, {self.rotation[3]:.4f}]'
        )
        
        # 发布相机光学坐标系的变换 (camera_link -> camera_color_optical_frame)
        # 标准 RealSense 相机坐标系转换
        t_optical = TransformStamped()
        t_optical.header.stamp = self.get_clock().now().to_msg()
        t_optical.header.frame_id = 'camera_link'
        t_optical.child_frame_id = self.camera_frame
        
        # 从相机物理坐标系到光学坐标系的变换
        # 光学坐标系: Z 前, X 右, Y 下
        # 物理坐标系: Z 前, X 右, Y 上 (通常)
        # 旋转 -90° 绕 X 轴, 然后 -90° 绕 Z 轴
        t_optical.transform.translation.x = 0.0
        t_optical.transform.translation.y = 0.0
        t_optical.transform.translation.z = 0.0
        # 单位四元数（不旋转，假设相机驱动已处理）
        t_optical.transform.rotation.x = 0.0
        t_optical.transform.rotation.y = 0.0
        t_optical.transform.rotation.z = 0.0
        t_optical.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t_optical)
    
    def reload_callback(self, request, response):
        """重新加载标定回调"""
        self._load_calibration()
        self._publish_calibration_tf()
        response.success = True
        response.message = '标定已重新加载'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
