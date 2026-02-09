#!/usr/bin/env python3
"""
RS-A3 抓取可视化控制界面
PyQt5 实现的交互式控制界面
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from cv_bridge import CvBridge
import cv2
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QTextEdit, QSplitter,
    QStatusBar, QMessageBox, QSlider, QSpinBox, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap, QFont


class RosSignals(QObject):
    """ROS 信号桥接到 Qt"""
    image_updated = pyqtSignal(np.ndarray)
    status_updated = pyqtSignal(str)
    grasp_state_updated = pyqtSignal(str)
    target_pose_updated = pyqtSignal(object)


class GraspGuiNode(Node):
    """抓取 GUI ROS2 节点"""
    
    def __init__(self, signals: RosSignals):
        super().__init__('grasp_gui')
        
        self.signals = signals
        self.bridge = CvBridge()
        
        # 订阅器
        self.detection_sub = self.create_subscription(
            Image, '/detection/image', self.detection_callback, 10)
        self.servo_status_sub = self.create_subscription(
            String, '/visual_servo/status', self.servo_status_callback, 10)
        self.grasp_state_sub = self.create_subscription(
            String, '/grasp/state', self.grasp_state_callback, 10)
        self.target_pose_sub = self.create_subscription(
            PoseStamped, '/target/pose_base', self.target_pose_callback, 10)
        
        # 发布器
        self.click_pub = self.create_publisher(Point, '/selected_point', 10)
        self.gripper_pub = self.create_publisher(Bool, '/gripper/command', 10)
        
        # 服务客户端
        self.start_grasp_client = self.create_client(Trigger, '/grasp/start')
        self.abort_grasp_client = self.create_client(Trigger, '/grasp/abort')
        self.reset_grasp_client = self.create_client(Trigger, '/grasp/reset')
        self.servo_enable_client = self.create_client(SetBool, '/visual_servo/enable')
        self.servo_approach_client = self.create_client(Trigger, '/visual_servo/start_approach')
        self.servo_stop_client = self.create_client(Trigger, '/visual_servo/stop')
        
        self.get_logger().info('GUI 节点已启动')
    
    def detection_callback(self, msg: Image):
        """检测图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.signals.image_updated.emit(cv_image)
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}')
    
    def servo_status_callback(self, msg: String):
        """伺服状态回调"""
        self.signals.status_updated.emit(msg.data)
    
    def grasp_state_callback(self, msg: String):
        """抓取状态回调"""
        self.signals.grasp_state_updated.emit(msg.data)
    
    def target_pose_callback(self, msg: PoseStamped):
        """目标位姿回调"""
        self.signals.target_pose_updated.emit(msg)
    
    def publish_click(self, x: int, y: int):
        """发布点击位置"""
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = 0.0
        self.click_pub.publish(point)
        self.get_logger().info(f'发布点击位置: ({x}, {y})')
    
    def call_start_grasp(self):
        """调用开始抓取服务"""
        if not self.start_grasp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('抓取服务不可用')
            return False, '抓取服务不可用'
        
        request = Trigger.Request()
        future = self.start_grasp_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, '服务调用超时'
    
    def call_abort_grasp(self):
        """调用中止抓取服务"""
        if not self.abort_grasp_client.wait_for_service(timeout_sec=1.0):
            return False, '服务不可用'
        
        request = Trigger.Request()
        future = self.abort_grasp_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, '服务调用超时'
    
    def call_reset(self):
        """调用重置服务"""
        if not self.reset_grasp_client.wait_for_service(timeout_sec=1.0):
            return False, '服务不可用'
        
        request = Trigger.Request()
        future = self.reset_grasp_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, '服务调用超时'
    
    def call_servo_approach(self):
        """调用视觉伺服接近"""
        if not self.servo_approach_client.wait_for_service(timeout_sec=1.0):
            return False, '服务不可用'
        
        request = Trigger.Request()
        future = self.servo_approach_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, '服务调用超时'
    
    def call_servo_stop(self):
        """调用停止伺服"""
        if not self.servo_stop_client.wait_for_service(timeout_sec=1.0):
            return False, '服务不可用'
        
        request = Trigger.Request()
        future = self.servo_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            return future.result().success, future.result().message
        return False, '服务调用超时'
    
    def send_gripper_command(self, close: bool):
        """发送夹爪命令"""
        msg = Bool()
        msg.data = close
        self.gripper_pub.publish(msg)


class ImageLabel(QLabel):
    """可点击的图像显示标签"""
    
    clicked = pyqtSignal(int, int)
    
    def __init__(self):
        super().__init__()
        self.setMinimumSize(640, 480)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("border: 2px solid #333; background-color: #1a1a1a;")
        self.setText("等待相机图像...")
        self.setFont(QFont("Arial", 14))
        
        self._scale_x = 1.0
        self._scale_y = 1.0
        self._offset_x = 0
        self._offset_y = 0
    
    def mousePressEvent(self, event):
        """鼠标点击事件"""
        if event.button() == Qt.LeftButton:
            # 计算相对于图像的实际坐标
            x = int((event.pos().x() - self._offset_x) / self._scale_x)
            y = int((event.pos().y() - self._offset_y) / self._scale_y)
            self.clicked.emit(x, y)
    
    def update_image(self, cv_image: np.ndarray):
        """更新显示的图像"""
        h, w, ch = cv_image.shape
        
        # 计算缩放比例以适应标签大小
        label_w = self.width()
        label_h = self.height()
        
        self._scale_x = label_w / w
        self._scale_y = label_h / h
        scale = min(self._scale_x, self._scale_y)
        self._scale_x = scale
        self._scale_y = scale
        
        new_w = int(w * scale)
        new_h = int(h * scale)
        
        self._offset_x = (label_w - new_w) // 2
        self._offset_y = (label_h - new_h) // 2
        
        # 转换为 Qt 图像
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        
        scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
            new_w, new_h, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.setPixmap(scaled_pixmap)


class GraspGuiWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self, ros_node: GraspGuiNode, signals: RosSignals):
        super().__init__()
        
        self.ros_node = ros_node
        self.signals = signals
        
        self.setWindowTitle('RS-A3 物体抓取控制系统')
        self.setGeometry(100, 100, 1200, 800)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #2b2b2b;
            }
            QLabel {
                color: #ffffff;
            }
            QPushButton {
                background-color: #4a4a4a;
                color: #ffffff;
                border: 1px solid #5a5a5a;
                padding: 8px 16px;
                border-radius: 4px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #5a5a5a;
            }
            QPushButton:pressed {
                background-color: #3a3a3a;
            }
            QPushButton:disabled {
                background-color: #3a3a3a;
                color: #888888;
            }
            QGroupBox {
                color: #ffffff;
                border: 1px solid #5a5a5a;
                border-radius: 4px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QTextEdit {
                background-color: #1a1a1a;
                color: #00ff00;
                border: 1px solid #5a5a5a;
                font-family: monospace;
            }
        """)
        
        self._setup_ui()
        self._connect_signals()
        
        # 目标位姿
        self.target_pose = None
    
    def _setup_ui(self):
        """设置 UI"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout(central_widget)
        
        # 左侧：图像显示
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        # 图像标签
        self.image_label = ImageLabel()
        left_layout.addWidget(self.image_label)
        
        # 图像信息
        self.image_info_label = QLabel('点击图像选择目标物体')
        self.image_info_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.image_info_label)
        
        main_layout.addWidget(left_widget, stretch=3)
        
        # 右侧：控制面板
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # 目标信息组
        target_group = QGroupBox('目标信息')
        target_layout = QVBoxLayout(target_group)
        
        self.target_status_label = QLabel('目标: 未选择')
        target_layout.addWidget(self.target_status_label)
        
        self.target_pose_label = QLabel('位置: --')
        target_layout.addWidget(self.target_pose_label)
        
        right_layout.addWidget(target_group)
        
        # 状态组
        status_group = QGroupBox('系统状态')
        status_layout = QVBoxLayout(status_group)
        
        self.grasp_state_label = QLabel('抓取状态: IDLE')
        self.grasp_state_label.setFont(QFont("Arial", 12, QFont.Bold))
        status_layout.addWidget(self.grasp_state_label)
        
        self.servo_status_label = QLabel('伺服状态: 就绪')
        status_layout.addWidget(self.servo_status_label)
        
        right_layout.addWidget(status_group)
        
        # 控制按钮组
        control_group = QGroupBox('抓取控制')
        control_layout = QVBoxLayout(control_group)
        
        # 开始抓取按钮
        self.start_btn = QPushButton('🎯 开始抓取')
        self.start_btn.setStyleSheet("""
            QPushButton {
                background-color: #2e7d32;
                font-size: 16px;
                padding: 12px;
            }
            QPushButton:hover {
                background-color: #388e3c;
            }
        """)
        self.start_btn.clicked.connect(self._on_start_grasp)
        control_layout.addWidget(self.start_btn)
        
        # 接近按钮
        self.approach_btn = QPushButton('📍 接近目标')
        self.approach_btn.clicked.connect(self._on_approach)
        control_layout.addWidget(self.approach_btn)
        
        # 停止按钮
        self.stop_btn = QPushButton('⏹ 停止')
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #c62828;
            }
            QPushButton:hover {
                background-color: #d32f2f;
            }
        """)
        self.stop_btn.clicked.connect(self._on_stop)
        control_layout.addWidget(self.stop_btn)
        
        # 重置按钮
        self.reset_btn = QPushButton('🔄 重置')
        self.reset_btn.clicked.connect(self._on_reset)
        control_layout.addWidget(self.reset_btn)
        
        right_layout.addWidget(control_group)
        
        # 夹爪控制组
        gripper_group = QGroupBox('夹爪控制')
        gripper_layout = QHBoxLayout(gripper_group)
        
        self.gripper_open_btn = QPushButton('✋ 打开')
        self.gripper_open_btn.clicked.connect(lambda: self._on_gripper(False))
        gripper_layout.addWidget(self.gripper_open_btn)
        
        self.gripper_close_btn = QPushButton('✊ 闭合')
        self.gripper_close_btn.clicked.connect(lambda: self._on_gripper(True))
        gripper_layout.addWidget(self.gripper_close_btn)
        
        right_layout.addWidget(gripper_group)
        
        # 日志组
        log_group = QGroupBox('操作日志')
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        log_layout.addWidget(self.log_text)
        
        right_layout.addWidget(log_group)
        
        right_layout.addStretch()
        
        main_layout.addWidget(right_widget, stretch=1)
        
        # 状态栏
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage('就绪')
    
    def _connect_signals(self):
        """连接信号"""
        self.signals.image_updated.connect(self._on_image_updated)
        self.signals.status_updated.connect(self._on_status_updated)
        self.signals.grasp_state_updated.connect(self._on_grasp_state_updated)
        self.signals.target_pose_updated.connect(self._on_target_pose_updated)
        self.image_label.clicked.connect(self._on_image_clicked)
    
    def _on_image_updated(self, cv_image: np.ndarray):
        """图像更新回调"""
        self.image_label.update_image(cv_image)
    
    def _on_status_updated(self, status: str):
        """伺服状态更新回调"""
        self.servo_status_label.setText(f'伺服状态: {status}')
    
    def _on_grasp_state_updated(self, state: str):
        """抓取状态更新回调"""
        self.grasp_state_label.setText(f'抓取状态: {state}')
        
        # 根据状态更新按钮
        if state == 'IDLE':
            self.start_btn.setEnabled(True)
            self.approach_btn.setEnabled(True)
        else:
            self.start_btn.setEnabled(False)
            self.approach_btn.setEnabled(False)
    
    def _on_target_pose_updated(self, pose: PoseStamped):
        """目标位姿更新回调"""
        self.target_pose = pose
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        self.target_pose_label.setText(f'位置: ({x:.3f}, {y:.3f}, {z:.3f}) m')
    
    def _on_image_clicked(self, x: int, y: int):
        """图像点击回调"""
        self.ros_node.publish_click(x, y)
        self.target_status_label.setText(f'目标: 像素 ({x}, {y})')
        self._log(f'选择目标点: ({x}, {y})')
    
    def _on_start_grasp(self):
        """开始抓取"""
        self._log('开始抓取序列...')
        success, message = self.ros_node.call_start_grasp()
        if success:
            self._log(f'✓ {message}')
        else:
            self._log(f'✗ {message}')
            QMessageBox.warning(self, '抓取失败', message)
    
    def _on_approach(self):
        """接近目标"""
        self._log('开始接近目标...')
        success, message = self.ros_node.call_servo_approach()
        if success:
            self._log(f'✓ {message}')
        else:
            self._log(f'✗ {message}')
    
    def _on_stop(self):
        """停止"""
        self._log('停止运动')
        self.ros_node.call_servo_stop()
        self.ros_node.call_abort_grasp()
    
    def _on_reset(self):
        """重置"""
        self._log('重置系统')
        self.ros_node.call_reset()
        self.target_status_label.setText('目标: 未选择')
        self.target_pose_label.setText('位置: --')
    
    def _on_gripper(self, close: bool):
        """夹爪控制"""
        action = '闭合' if close else '打开'
        self._log(f'夹爪{action}')
        self.ros_node.send_gripper_command(close)
    
    def _log(self, message: str):
        """添加日志"""
        from datetime import datetime
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_text.append(f'[{timestamp}] {message}')
        self.statusBar.showMessage(message)


def main(args=None):
    # 初始化 ROS
    rclpy.init(args=args)
    
    # 创建信号对象
    signals = RosSignals()
    
    # 创建 ROS 节点
    ros_node = GraspGuiNode(signals)
    
    # 创建 ROS spin 线程
    def ros_spin():
        try:
            rclpy.spin(ros_node)
        except:
            pass
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # 创建 Qt 应用
    app = QApplication(sys.argv)
    
    # 创建主窗口
    window = GraspGuiWindow(ros_node, signals)
    window.show()
    
    # 运行 Qt 事件循环
    try:
        exit_code = app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
