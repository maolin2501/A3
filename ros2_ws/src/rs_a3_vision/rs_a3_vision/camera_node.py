#!/usr/bin/env python3
"""
RealSense D435 相机节点
提供RGB和深度图像，以及3D点云
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
    print("警告: pyrealsense2 未安装，将使用模拟模式")


class RealSenseCameraNode(Node):
    """RealSense D435 相机节点"""
    
    def __init__(self):
        super().__init__('realsense_camera_node')
        
        # 声明参数
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('enable_color', True)
        self.declare_parameter('align_depth', True)
        self.declare_parameter('use_mock', False)
        
        # 获取参数
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.enable_depth = self.get_parameter('enable_depth').value
        self.enable_color = self.get_parameter('enable_color').value
        self.align_depth = self.get_parameter('align_depth').value
        self.use_mock = self.get_parameter('use_mock').value or not REALSENSE_AVAILABLE
        
        # 发布器
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.aligned_depth_pub = self.create_publisher(Image, '/camera/aligned_depth/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        
        self.bridge = CvBridge()
        
        # 相机内参（将在初始化时更新）
        self.color_intrinsics = None
        self.depth_intrinsics = None
        
        if self.use_mock:
            self.get_logger().warn('使用模拟相机模式')
            self._init_mock_camera()
        else:
            self._init_realsense()
        
        # 创建定时器
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info(f'RealSense 相机节点已启动 ({self.width}x{self.height}@{self.fps}fps)')
    
    def _init_realsense(self):
        """初始化 RealSense 相机"""
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # 配置深度流
            if self.enable_depth:
                self.config.enable_stream(
                    rs.stream.depth, 
                    self.width, self.height, 
                    rs.format.z16, 
                    self.fps
                )
            
            # 配置彩色流
            if self.enable_color:
                self.config.enable_stream(
                    rs.stream.color, 
                    self.width, self.height, 
                    rs.format.bgr8, 
                    self.fps
                )
            
            # 启动管道
            self.profile = self.pipeline.start(self.config)
            
            # 获取内参
            if self.enable_color:
                color_stream = self.profile.get_stream(rs.stream.color)
                self.color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
            
            if self.enable_depth:
                depth_stream = self.profile.get_stream(rs.stream.depth)
                self.depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
                
                # 获取深度比例
                depth_sensor = self.profile.get_device().first_depth_sensor()
                self.depth_scale = depth_sensor.get_depth_scale()
                self.get_logger().info(f'深度比例: {self.depth_scale}')
            
            # 深度对齐到彩色
            if self.align_depth and self.enable_depth and self.enable_color:
                self.align = rs.align(rs.stream.color)
            else:
                self.align = None
            
            self.get_logger().info('RealSense 相机初始化成功')
            
        except Exception as e:
            self.get_logger().error(f'RealSense 初始化失败: {e}')
            self.get_logger().warn('切换到模拟模式')
            self.use_mock = True
            self._init_mock_camera()
    
    def _init_mock_camera(self):
        """初始化模拟相机"""
        self.pipeline = None
        self.align = None
        self.depth_scale = 0.001  # 1mm
        
        # 模拟内参 (D435 典型值)
        self.mock_fx = 615.0
        self.mock_fy = 615.0
        self.mock_cx = self.width / 2.0
        self.mock_cy = self.height / 2.0
    
    def timer_callback(self):
        """定时回调，读取并发布图像"""
        timestamp = self.get_clock().now().to_msg()
        
        if self.use_mock:
            self._publish_mock_images(timestamp)
        else:
            self._publish_realsense_images(timestamp)
    
    def _publish_realsense_images(self, timestamp):
        """发布真实 RealSense 图像"""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            # 对齐处理
            if self.align:
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
            else:
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
            
            # 发布彩色图像
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
                color_msg.header.stamp = timestamp
                color_msg.header.frame_id = 'camera_color_optical_frame'
                self.rgb_pub.publish(color_msg)
                
                # 发布彩色相机信息
                self._publish_camera_info(
                    self.camera_info_pub, 
                    self.color_intrinsics, 
                    timestamp,
                    'camera_color_optical_frame'
                )
            
            # 发布深度图像
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, 'mono16')
                depth_msg.header.stamp = timestamp
                depth_msg.header.frame_id = 'camera_depth_optical_frame'
                
                if self.align:
                    self.aligned_depth_pub.publish(depth_msg)
                else:
                    self.depth_pub.publish(depth_msg)
                
                # 发布深度相机信息
                intrinsics = self.color_intrinsics if self.align else self.depth_intrinsics
                self._publish_camera_info(
                    self.depth_info_pub, 
                    intrinsics, 
                    timestamp,
                    'camera_depth_optical_frame'
                )
                
        except Exception as e:
            self.get_logger().error(f'读取帧失败: {e}')
    
    def _publish_mock_images(self, timestamp):
        """发布模拟图像"""
        # 创建模拟彩色图像（棋盘格图案）
        color_image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        block_size = 40
        for i in range(0, self.height, block_size):
            for j in range(0, self.width, block_size):
                if (i // block_size + j // block_size) % 2 == 0:
                    color_image[i:i+block_size, j:j+block_size] = [100, 150, 200]
                else:
                    color_image[i:i+block_size, j:j+block_size] = [200, 150, 100]
        
        # 添加一个模拟物体（红色圆形）
        import cv2
        cv2.circle(color_image, (self.width//2, self.height//2), 50, (0, 0, 255), -1)
        cv2.putText(color_image, "Mock Camera", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        color_msg = self.bridge.cv2_to_imgmsg(color_image, 'bgr8')
        color_msg.header.stamp = timestamp
        color_msg.header.frame_id = 'camera_color_optical_frame'
        self.rgb_pub.publish(color_msg)
        
        # 创建模拟深度图像
        depth_image = np.ones((self.height, self.width), dtype=np.uint16) * 1000  # 1m 背景
        # 模拟物体深度 (0.5m)
        cv2.circle(depth_image, (self.width//2, self.height//2), 50, 500, -1)
        
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, 'mono16')
        depth_msg.header.stamp = timestamp
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self.aligned_depth_pub.publish(depth_msg)
        
        # 发布相机信息
        self._publish_mock_camera_info(timestamp)
    
    def _publish_camera_info(self, publisher, intrinsics, timestamp, frame_id):
        """发布相机信息"""
        info = CameraInfo()
        info.header.stamp = timestamp
        info.header.frame_id = frame_id
        info.width = intrinsics.width
        info.height = intrinsics.height
        info.distortion_model = 'plumb_bob'
        info.d = list(intrinsics.coeffs)
        info.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        publisher.publish(info)
    
    def _publish_mock_camera_info(self, timestamp):
        """发布模拟相机信息"""
        info = CameraInfo()
        info.header.stamp = timestamp
        info.header.frame_id = 'camera_color_optical_frame'
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            self.mock_fx, 0.0, self.mock_cx,
            0.0, self.mock_fy, self.mock_cy,
            0.0, 0.0, 1.0
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            self.mock_fx, 0.0, self.mock_cx, 0.0,
            0.0, self.mock_fy, self.mock_cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        self.camera_info_pub.publish(info)
    
    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'pipeline') and self.pipeline:
            self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
