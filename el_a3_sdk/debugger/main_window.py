"""主窗口：QDockWidget 布局 + 信号连接"""

import time
import logging
from PyQt6.QtWidgets import (
    QMainWindow, QDockWidget, QTabWidget,
    QWidget, QVBoxLayout, QTextEdit, QApplication,
)
from PyQt6.QtCore import Qt, QTimer

from debugger.backend.arm_worker import ArmWorker
from debugger.widgets.toolbar_panel import ToolbarPanel
from debugger.widgets.joint_control_panel import JointControlPanel
from debugger.widgets.monitoring_window import MonitoringWindow
from debugger.widgets.trajectory_panel import TrajectoryPanel
from debugger.widgets.teaching_panel import TeachingPanel
from debugger.widgets.diagnostics_panel import DiagnosticsPanel
from debugger.widgets.gripper_panel import GripperPanel
from debugger.widgets.gamepad_panel import GamepadPanel
from debugger.widgets.viewer_3d import Viewer3DPanel

logger = logging.getLogger("debugger")


class MainWindow(QMainWindow):
    """EL-A3 调试上位机主窗口"""

    UI_UPDATE_INTERVAL_S = 0.05  # 20 Hz UI refresh cap

    def __init__(self, urdf_path=None, mesh_dir=None):
        super().__init__()
        self.setWindowTitle("EL-A3 机械臂调试上位机")
        self.setMinimumSize(1280, 800)
        self.resize(1600, 960)

        self._urdf_path = urdf_path
        self._mesh_dir = mesh_dir
        self._last_joint_states = None
        self._last_effort_states = None
        self._last_ui_update_time = 0.0

        self._init_worker()
        self._init_ui()
        self._connect_signals()

        QTimer.singleShot(500, self._init_3d_model)

    def _init_worker(self):
        self.worker = ArmWorker()
        self.worker.start()

    def _init_ui(self):
        # --- 顶部工具栏（单行固定高度） ---
        toolbar_widget = ToolbarPanel()
        self.toolbar = toolbar_widget
        toolbar_dock = QDockWidget("工具栏", self)
        toolbar_dock.setWidget(toolbar_widget)
        toolbar_dock.setFeatures(QDockWidget.DockWidgetFeature.NoDockWidgetFeatures)
        empty_title = QWidget()
        empty_title.setFixedHeight(0)
        toolbar_dock.setTitleBarWidget(empty_title)
        toolbar_dock.setStyleSheet("QDockWidget { border: none; }")
        self.addDockWidget(Qt.DockWidgetArea.TopDockWidgetArea, toolbar_dock)

        # --- 左侧：3D 可视化 ---
        self.viewer_3d = Viewer3DPanel(
            urdf_path=self._urdf_path,
            mesh_dir=self._mesh_dir,
        )
        viewer_dock = QDockWidget("3D 可视化", self)
        viewer_dock.setWidget(self.viewer_3d)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, viewer_dock)

        # --- 右侧：功能面板 Tab（关节控制、轨迹、示教、电机诊断、夹爪） ---
        tabs = QTabWidget()

        self.joint_panel = JointControlPanel()
        tabs.addTab(self.joint_panel, "关节控制")

        self.trajectory_panel = TrajectoryPanel()
        tabs.addTab(self.trajectory_panel, "轨迹控制")

        self.teaching_panel = TeachingPanel()
        tabs.addTab(self.teaching_panel, "示教模式")

        self.diagnostics_panel = DiagnosticsPanel()
        tabs.addTab(self.diagnostics_panel, "电机诊断")

        self.gripper_panel = GripperPanel()
        tabs.addTab(self.gripper_panel, "夹爪")

        self.gamepad_panel = GamepadPanel()
        tabs.addTab(self.gamepad_panel, "手柄")

        tabs_dock = QDockWidget("功能面板", self)
        tabs_dock.setWidget(tabs)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, tabs_dock)

        # --- 底部日志 ---
        self.log_console = QTextEdit()
        self.log_console.setObjectName("logConsole")
        self.log_console.setReadOnly(True)
        self.log_console.setFixedHeight(100)
        log_dock = QDockWidget("日志", self)
        log_dock.setWidget(self.log_console)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, log_dock)

        # --- 实时监控弹出窗口（按需打开） ---
        self.monitoring_window = MonitoringWindow(self.worker.data_buffer, parent=self)

        self.statusBar().showMessage("就绪")

        QTimer.singleShot(0, lambda: self._adjust_dock_sizes(viewer_dock, tabs_dock))

    def _connect_signals(self):
        tb = self.toolbar
        tb.connect_requested.connect(self._on_connect)
        tb.disconnect_requested.connect(lambda: self.worker.submit_command("disconnect"))
        tb.enable_requested.connect(lambda: self.worker.submit_command("enable"))
        tb.disable_requested.connect(lambda: self.worker.submit_command("disable"))
        tb.emergency_stop_requested.connect(
            lambda: self.worker.submit_command("emergency_stop")
        )
        tb.open_monitor_requested.connect(self._open_monitoring)

        self.worker.connected_changed.connect(tb.set_connected)
        self.worker.enabled_changed.connect(tb.set_enabled)
        self.worker.enabled_changed.connect(self.joint_panel.set_enabled)
        self.worker.error_occurred.connect(self._on_error)
        self.worker.log_message.connect(self._append_log)
        self.worker.can_fps_updated.connect(tb.set_fps)
        self.worker.can_fps_updated.connect(self.diagnostics_panel.update_can_stats)

        self.worker.joints_updated.connect(self._on_joints_updated)
        self.worker.efforts_updated.connect(self._on_efforts_updated)
        self.worker.motor_feedback_updated.connect(
            self.diagnostics_panel.update_motor_feedback
        )

        self.joint_panel.joint_command.connect(
            lambda pos: self.worker.submit_command("joint_ctrl", pos)
        )

        tp = self.trajectory_panel
        tp.move_j_requested.connect(
            lambda pos, dur: self.worker.submit_command("move_j", pos, dur)
        )
        tp.end_pose_requested.connect(
            lambda x, y, z, rx, ry, rz, d: self.worker.submit_command(
                "end_pose_ctrl", x, y, z, rx, ry, rz, d
            )
        )
        tp.cancel_requested.connect(
            lambda: self.worker.submit_command("cancel_motion")
        )

        teach = self.teaching_panel
        teach.zero_torque_requested.connect(
            lambda en: self.worker.submit_command("zero_torque", en)
        )
        teach.zero_torque_gravity_requested.connect(
            lambda en: self.worker.submit_command("zero_torque_gravity", en)
        )
        teach.move_j_requested.connect(
            lambda pos, dur: self.worker.submit_command("move_j", pos, dur)
        )

        gp = self.gripper_panel
        gp.gripper_command.connect(
            lambda angle: self.worker.submit_command("gripper_ctrl", angle)
        )
        gp.set_zero_requested.connect(
            lambda: self.worker.submit_command("set_zero_position", 7)
        )

        diag = self.diagnostics_panel
        diag.read_param_requested.connect(
            lambda mid, pidx: self.worker.submit_command("read_motor_param", mid, pidx)
        )
        diag.write_param_requested.connect(
            lambda mid, pidx, val: self.worker.submit_command(
                "write_motor_param", mid, pidx, val
            )
        )
        diag.set_zero_requested.connect(
            lambda m: self.worker.submit_command("set_zero_position", m)
        )
        diag.verify_zero_sta_requested.connect(
            lambda: self.worker.submit_command("verify_zero_sta")
        )
        self.worker.zero_sta_verified.connect(diag.update_zero_sta_result)

        self.gamepad_panel.gamepad_log.connect(self._append_log)
        self.worker.connected_changed.connect(self._on_connected_for_gamepad)

    def _adjust_dock_sizes(self, viewer_dock, tabs_dock):
        w = self.width()
        left_w = int(w * 0.50)
        right_w = w - left_w
        self.resizeDocks(
            [viewer_dock, tabs_dock], [left_w, right_w], Qt.Orientation.Horizontal
        )

    def _open_monitoring(self):
        mw = self.monitoring_window
        if mw.isVisible():
            mw.raise_()
            mw.activateWindow()
        else:
            mw.show()
            mw.raise_()

    def _on_connected_for_gamepad(self, connected: bool):
        if connected and self.worker.arm is not None:
            self.gamepad_panel.set_arm(self.worker.arm)
        elif not connected:
            self.gamepad_panel.set_arm(None)

    def _on_connect(self, can_name: str, sim_mode: bool):
        self.worker.submit_command("connect", can_name, sim_mode)

    def _on_joints_updated(self, joint_states):
        self._last_joint_states = joint_states
        self.teaching_panel.update_positions(joint_states)
        self.trajectory_panel.update_current_positions(joint_states)

        now = time.monotonic()
        if now - self._last_ui_update_time < self.UI_UPDATE_INTERVAL_S:
            return
        self._last_ui_update_time = now

        self.joint_panel.update_feedback(joint_states)
        self.viewer_3d.update_joint_angles(joint_states)
        self.gripper_panel.update_feedback(joint_states, self._last_effort_states)
        self.diagnostics_panel.update_motor_states(
            joint_states, None, self._last_effort_states
        )

    def _on_efforts_updated(self, effort_states):
        self._last_effort_states = effort_states

    def _on_error(self, msg: str):
        self.toolbar.set_error(msg)
        self._append_log(f"[错误] {msg}")

    def _append_log(self, msg: str):
        timestamp = time.strftime("%H:%M:%S")
        self.log_console.append(f"[{timestamp}] {msg}")
        scrollbar = self.log_console.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())

    def _init_3d_model(self):
        success = self.viewer_3d.initialize_model()
        if success:
            self._append_log("3D 模型加载成功")
        else:
            self._append_log("3D 模型加载失败（检查 URDF 和 mesh 路径）")

    def closeEvent(self, event):
        self._append_log("正在关闭...")
        self.gamepad_panel.cleanup()
        if self.monitoring_window.isVisible():
            self.monitoring_window.close()
        if self.worker.is_connected:
            self.worker.submit_command("disconnect")
        self.worker.stop()
        super().closeEvent(event)
