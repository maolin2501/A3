#!/usr/bin/env python3
"""
物体检测节点
使用 YOLOv8 进行物体检测，支持点击选择目标
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import threading

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("警告: ultralytics 未安装，物体检测功能受限")


class ObjectDetectorNode(Node):
    """物体检测节点"""
    
    def __init__(self):
        super().__init__('object_detector')
        
        # 声明参数
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('enable_yolo', True)
        self.declare_parameter('depth_scale', 0.001)  # mm to m
        
        # 获取参数
        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.enable_yolo = self.get_parameter('enable_yolo').value and YOLO_AVAILABLE
        self.depth_scale = self.get_parameter('depth_scale').value
        
        # 订阅器
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/aligned_depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.click_sub = self.create_subscription(
            Point, '/selected_point', self.click_callback, 10)
        
        # 发布器
        self.detection_pub = self.create_publisher(Image, '/detection/image', 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target/pose', 10)
        self.target_pixel_pub = self.create_publisher(PointStamped, '/target/pixel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detection/markers', 10)
        self.detected_objects_pub = self.create_publisher(String, '/detection/objects', 10)
        
        self.bridge = CvBridge()
        
        # 状态变量
        self.current_rgb = None
        self.current_depth = None
        self.camera_intrinsics = None
        self.selected_point = None  # 用户选择的像素点 (u, v)
        self.selected_object_id = None  # 用户选择的检测物体ID
        self.detected_objects = []  # 检测到的物体列表
        
        self.lock = threading.Lock()
        
        # 初始化 YOLO
        if self.enable_yolo:
            try:
                self.model = YOLO(self.model_path)
                self.get_logger().info(f'YOLO 模型已加载: {self.model_path}')
            except Exception as e:
                self.get_logger().error(f'YOLO 加载失败: {e}')
                self.enable_yolo = False
        
        # 定时发布目标位姿
        self.timer = self.create_timer(0.05, self.publish_target_pose)  # 20Hz
        
        self.get_logger().info('物体检测节点已启动')
    
    def camera_info_callback(self, msg: CameraInfo):
        """相机信息回调"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5],
                'width': msg.width,
                'height': msg.height
            }
            self.get_logger().info(
                f'相机内参: fx={self.camera_intrinsics["fx"]:.2f}, '
                f'fy={self.camera_intrinsics["fy"]:.2f}, '
                f'cx={self.camera_intrinsics["cx"]:.2f}, '
                f'cy={self.camera_intrinsics["cy"]:.2f}'
            )
    
    def rgb_callback(self, msg: Image):
        """RGB图像回调"""
        with self.lock:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._process_detection()
    
    def depth_callback(self, msg: Image):
        """深度图像回调"""
        with self.lock:
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, 'mono16')
    
    def click_callback(self, msg: Point):
        """用户点击选择回调"""
        u, v = int(msg.x), int(msg.y)
        self.get_logger().info(f'用户选择点: ({u}, {v})')
        
        with self.lock:
            self.selected_point = (u, v)
            
            # 检查是否点击在某个检测框内
            self.selected_object_id = None
            for obj in self.detected_objects:
                x1, y1, x2, y2 = obj['bbox']
                if x1 <= u <= x2 and y1 <= v <= y2:
                    self.selected_object_id = obj['id']
                    self.selected_point = obj['center']  # 使用物体中心
                    self.get_logger().info(
                        f'选中物体: {obj["class_name"]} (ID: {obj["id"]}, 置信度: {obj["confidence"]:.2f})'
                    )
                    break
    
    def _process_detection(self):
        """处理检测"""
        if self.current_rgb is None:
            return
        
        display_image = self.current_rgb.copy()
        self.detected_objects = []
        
        # YOLO 检测
        if self.enable_yolo:
            try:
                results = self.model(self.current_rgb, verbose=False)
                
                for result in results:
                    boxes = result.boxes
                    for i, box in enumerate(boxes):
                        # 获取边界框
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        conf = float(box.conf[0])
                        cls_id = int(box.cls[0])
                        class_name = self.model.names[cls_id]
                        
                        if conf < self.conf_threshold:
                            continue
                        
                        # 计算中心点
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        
                        obj = {
                            'id': i,
                            'class_name': class_name,
                            'confidence': conf,
                            'bbox': (x1, y1, x2, y2),
                            'center': (center_x, center_y)
                        }
                        self.detected_objects.append(obj)
                        
                        # 绘制边界框
                        color = (0, 255, 0)
                        if self.selected_object_id == i:
                            color = (0, 0, 255)  # 选中的物体用红色
                        
                        cv2.rectangle(display_image, (x1, y1), (x2, y2), color, 2)
                        label = f'{class_name}: {conf:.2f}'
                        cv2.putText(display_image, label, (x1, y1 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        cv2.circle(display_image, (center_x, center_y), 5, color, -1)
                
            except Exception as e:
                self.get_logger().error(f'YOLO 检测失败: {e}')
        
        # 绘制选中的点
        if self.selected_point:
            u, v = self.selected_point
            cv2.circle(display_image, (u, v), 10, (255, 0, 0), 2)
            cv2.drawMarker(display_image, (u, v), (255, 0, 0), 
                          cv2.MARKER_CROSS, 20, 2)
            
            # 显示3D坐标
            if self.current_depth is not None and self.camera_intrinsics:
                pos_3d = self._pixel_to_3d(u, v)
                if pos_3d:
                    x, y, z = pos_3d
                    coord_text = f'3D: ({x:.3f}, {y:.3f}, {z:.3f})m'
                    cv2.putText(display_image, coord_text, (u + 15, v),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # 显示检测物体数量
        cv2.putText(display_image, f'Objects: {len(self.detected_objects)}', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # 发布检测图像
        detection_msg = self.bridge.cv2_to_imgmsg(display_image, 'bgr8')
        detection_msg.header.stamp = self.get_clock().now().to_msg()
        detection_msg.header.frame_id = 'camera_color_optical_frame'
        self.detection_pub.publish(detection_msg)
        
        # 发布检测物体列表
        objects_str = ', '.join([f"{obj['class_name']}({obj['confidence']:.2f})" 
                                 for obj in self.detected_objects])
        self.detected_objects_pub.publish(String(data=objects_str))
        
        # 发布可视化标记
        self._publish_markers()
    
    def _pixel_to_3d(self, u: int, v: int):
        """将像素坐标转换为相机坐标系下的3D坐标"""
        if self.current_depth is None or self.camera_intrinsics is None:
            return None
        
        # 确保坐标在有效范围内
        h, w = self.current_depth.shape
        if not (0 <= u < w and 0 <= v < h):
            return None
        
        # 获取深度值
        depth = self.current_depth[v, u]
        if depth == 0:
            # 尝试取周围区域的中值深度
            radius = 5
            v_min = max(0, v - radius)
            v_max = min(h, v + radius)
            u_min = max(0, u - radius)
            u_max = min(w, u + radius)
            region = self.current_depth[v_min:v_max, u_min:u_max]
            valid_depths = region[region > 0]
            if len(valid_depths) == 0:
                return None
            depth = np.median(valid_depths)
        
        # 转换为米
        z = depth * self.depth_scale
        
        # 计算3D坐标
        fx = self.camera_intrinsics['fx']
        fy = self.camera_intrinsics['fy']
        cx = self.camera_intrinsics['cx']
        cy = self.camera_intrinsics['cy']
        
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        return (x, y, z)
    
    def publish_target_pose(self):
        """发布目标位姿"""
        if self.selected_point is None:
            return
        
        with self.lock:
            u, v = self.selected_point
            pos_3d = self._pixel_to_3d(u, v)
        
        if pos_3d is None:
            return
        
        x, y, z = pos_3d
        
        # 发布目标位姿（相机坐标系）
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'camera_color_optical_frame'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        # 默认朝向（末端垂直向下抓取）
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        self.target_pose_pub.publish(pose_msg)
        
        # 发布像素坐标
        pixel_msg = PointStamped()
        pixel_msg.header = pose_msg.header
        pixel_msg.point.x = float(u)
        pixel_msg.point.y = float(v)
        pixel_msg.point.z = z
        self.target_pixel_pub.publish(pixel_msg)
    
    def _publish_markers(self):
        """发布 RViz 可视化标记"""
        marker_array = MarkerArray()
        
        for i, obj in enumerate(self.detected_objects):
            center_u, center_v = obj['center']
            pos_3d = self._pixel_to_3d(center_u, center_v)
            
            if pos_3d is None:
                continue
            
            x, y, z = pos_3d
            
            # 创建球形标记
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'camera_color_optical_frame'
            marker.ns = 'detected_objects'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            if self.selected_object_id == i:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 100000000  # 0.1s
            
            marker_array.markers.append(marker)
            
            # 创建文本标记
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'object_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = x
            text_marker.pose.position.y = y - 0.05
            text_marker.pose.position.z = z
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.03
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{obj['class_name']}"
            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 100000000
            
            marker_array.markers.append(text_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
