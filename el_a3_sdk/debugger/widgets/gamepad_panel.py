"""手柄控制面板：设备连接 + 输入数据监控 + 手柄控臂"""

import glob
import math
import threading
import logging
from typing import Optional

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QComboBox, QPushButton, QGroupBox, QProgressBar,
    QScrollArea, QFrame,
)
from PyQt6.QtCore import pyqtSignal, QTimer, Qt

from el_a3_sdk.joystick import LinuxJoystick
from el_a3_sdk.controller_profiles import (
    PROFILES, ControllerProfile, detect_controller, ControllerDetection,
)

logger = logging.getLogger("debugger.gamepad")

SPEED_LEVELS = [
    ("极慢", 0.10),
    ("慢",   0.25),
    ("中",   0.50),
    ("快",   0.75),
    ("最大", 1.00),
]


class _AxisBar(QProgressBar):
    """双向进度条，用于显示 -1.0 ~ 1.0 的轴值"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRange(0, 2000)
        self.setValue(1000)
        self.setTextVisible(False)
        self.setFixedHeight(16)
        self.setStyleSheet(
            "QProgressBar { background: #313244; border: 1px solid #45475a; border-radius: 3px; }"
            "QProgressBar::chunk { background: #89b4fa; border-radius: 2px; }"
        )

    def set_value(self, v: float):
        mapped = int((v + 1.0) * 1000)
        self.setValue(max(0, min(2000, mapped)))


class _ButtonIndicator(QLabel):
    """单个按钮指示灯"""

    _STYLE_OFF = (
        "background: #313244; border: 1px solid #45475a; border-radius: 3px;"
        "min-width: 28px; min-height: 20px; color: #6c7086; font-size: 11px;"
        "qproperty-alignment: AlignCenter;"
    )
    _STYLE_ON = (
        "background: #a6e3a1; border: 1px solid #a6e3a1; border-radius: 3px;"
        "min-width: 28px; min-height: 20px; color: #1e1e2e; font-size: 11px; font-weight: bold;"
        "qproperty-alignment: AlignCenter;"
    )

    def __init__(self, index: int, parent=None):
        super().__init__(str(index), parent)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setStyleSheet(self._STYLE_OFF)

    def set_pressed(self, pressed: bool):
        self.setStyleSheet(self._STYLE_ON if pressed else self._STYLE_OFF)


class GamepadPanel(QWidget):
    """手柄控制 Tab 面板"""

    gamepad_log = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self._joystick: Optional[LinuxJoystick] = None
        self._arm = None
        self._controller = None
        self._ctrl_thread: Optional[threading.Thread] = None
        self._detection: Optional[ControllerDetection] = None

        self._speed_idx = 2
        self._ctrl_running = False

        self._init_ui()

        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(50)  # 20 Hz
        self._poll_timer.timeout.connect(self._poll_input)

    # ---- UI Construction ----

    def _init_ui(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        inner = QWidget()
        layout = QVBoxLayout(inner)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(6)

        layout.addWidget(self._build_connection_group())
        layout.addWidget(self._build_monitor_group())
        layout.addWidget(self._build_control_group())
        layout.addStretch()

        scroll.setWidget(inner)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

    def _build_connection_group(self) -> QGroupBox:
        grp = QGroupBox("连接设置")
        layout = QVBoxLayout()
        layout.setSpacing(4)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("设备:"))
        self._dev_combo = QComboBox()
        self._dev_combo.setEditable(True)
        self._dev_combo.setMinimumWidth(140)
        row1.addWidget(self._dev_combo, 1)

        self._refresh_btn = QPushButton("刷新")
        self._refresh_btn.setFixedWidth(50)
        self._refresh_btn.clicked.connect(self._scan_devices)
        row1.addWidget(self._refresh_btn)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Profile:"))
        self._profile_combo = QComboBox()
        self._profile_combo.addItem("auto", "auto")
        for pid, prof in PROFILES.items():
            self._profile_combo.addItem(prof.display_name, pid)
        row2.addWidget(self._profile_combo, 1)
        layout.addLayout(row2)

        row3 = QHBoxLayout()
        self._connect_btn = QPushButton("连接手柄")
        self._connect_btn.setObjectName("connectBtn")
        self._connect_btn.clicked.connect(self._toggle_connection)
        row3.addWidget(self._connect_btn)
        row3.addStretch()
        layout.addLayout(row3)

        self._info_label = QLabel("未连接")
        self._info_label.setWordWrap(True)
        self._info_label.setStyleSheet("color: #a6adc8; font-size: 11px;")
        layout.addWidget(self._info_label)

        grp.setLayout(layout)
        self._scan_devices()
        return grp

    def _build_monitor_group(self) -> QGroupBox:
        grp = QGroupBox("输入数据监控")
        layout = QVBoxLayout()
        layout.setSpacing(4)

        axes_label = QLabel("轴 (Axes)")
        axes_label.setStyleSheet("font-weight: bold; color: #89b4fa; margin-top: 2px;")
        layout.addWidget(axes_label)

        self._axis_bars: list[_AxisBar] = []
        self._axis_labels: list[QLabel] = []
        axes_grid = QGridLayout()
        axes_grid.setSpacing(2)
        for i in range(LinuxJoystick.MAX_AXES):
            name_lbl = QLabel(f"A{i}:")
            name_lbl.setFixedWidth(24)
            name_lbl.setStyleSheet("font-size: 11px;")
            bar = _AxisBar()
            val_lbl = QLabel(" 0.000")
            val_lbl.setFixedWidth(48)
            val_lbl.setStyleSheet("font-family: monospace; font-size: 11px;")
            axes_grid.addWidget(name_lbl, i, 0)
            axes_grid.addWidget(bar, i, 1)
            axes_grid.addWidget(val_lbl, i, 2)
            self._axis_bars.append(bar)
            self._axis_labels.append(val_lbl)
        layout.addLayout(axes_grid)

        btn_label = QLabel("按钮 (Buttons)")
        btn_label.setStyleSheet("font-weight: bold; color: #89b4fa; margin-top: 4px;")
        layout.addWidget(btn_label)

        self._btn_indicators: list[_ButtonIndicator] = []
        btn_grid = QGridLayout()
        btn_grid.setSpacing(3)
        for i in range(LinuxJoystick.MAX_BUTTONS):
            ind = _ButtonIndicator(i)
            btn_grid.addWidget(ind, i // 8, i % 8)
            self._btn_indicators.append(ind)
        layout.addLayout(btn_grid)

        mapped_label = QLabel("逻辑映射")
        mapped_label.setStyleSheet("font-weight: bold; color: #89b4fa; margin-top: 4px;")
        layout.addWidget(mapped_label)

        self._mapped_display = QLabel("—")
        self._mapped_display.setWordWrap(True)
        self._mapped_display.setStyleSheet(
            "font-family: monospace; font-size: 11px; color: #cdd6f4; "
            "background: #181825; padding: 4px; border-radius: 3px;"
        )
        layout.addWidget(self._mapped_display)

        grp.setLayout(layout)
        return grp

    def _build_control_group(self) -> QGroupBox:
        grp = QGroupBox("手柄控臂")
        layout = QVBoxLayout()
        layout.setSpacing(4)

        row1 = QHBoxLayout()
        self._ctrl_btn = QPushButton("启动控制")
        self._ctrl_btn.setObjectName("enableBtn")
        self._ctrl_btn.setEnabled(False)
        self._ctrl_btn.clicked.connect(self._toggle_control)
        row1.addWidget(self._ctrl_btn)
        row1.addStretch()
        layout.addLayout(row1)

        info_grid = QGridLayout()
        info_grid.setSpacing(2)

        info_grid.addWidget(QLabel("速度档位:"), 0, 0)
        self._speed_label = QLabel("3/5 [中]")
        self._speed_label.setStyleSheet("font-weight: bold;")
        info_grid.addWidget(self._speed_label, 0, 1)

        info_grid.addWidget(QLabel("控制模式:"), 1, 0)
        self._mode_label = QLabel("—")
        info_grid.addWidget(self._mode_label, 1, 1)

        info_grid.addWidget(QLabel("末端位姿:"), 2, 0)
        self._pose_label = QLabel("—")
        self._pose_label.setStyleSheet("font-family: monospace; font-size: 11px;")
        self._pose_label.setWordWrap(True)
        info_grid.addWidget(self._pose_label, 2, 1)

        layout.addLayout(info_grid)
        grp.setLayout(layout)
        return grp

    # ---- Device Scanning ----

    def _scan_devices(self):
        self._dev_combo.clear()
        devices = sorted(glob.glob("/dev/input/js*"))
        if devices:
            for d in devices:
                self._dev_combo.addItem(d)
        else:
            self._dev_combo.addItem("/dev/input/js0")

    # ---- Connection ----

    def _toggle_connection(self):
        if self._joystick and self._joystick.connected:
            self._disconnect_gamepad()
        else:
            self._connect_gamepad()

    def _connect_gamepad(self):
        device = self._dev_combo.currentText().strip()
        if not device:
            return

        profile_id = self._profile_combo.currentData()

        try:
            self._detection = detect_controller(device, requested_profile=profile_id)
        except Exception as e:
            self._info_label.setText(f"Profile 检测失败: {e}")
            self.gamepad_log.emit(f"手柄 profile 检测失败: {e}")
            return

        joy = LinuxJoystick(device=device)
        if not joy.connect():
            self._info_label.setText(f"无法打开 {device}")
            self.gamepad_log.emit(f"手柄连接失败: {device}")
            return

        self._joystick = joy
        det = self._detection
        self._info_label.setText(
            f"设备: {det.name or 'unknown'}  |  "
            f"VID:PID {det.vendor}:{det.product}  |  "
            f"Profile: {det.profile.display_name} ({det.source})"
        )
        self._connect_btn.setText("断开手柄")
        self._connect_btn.setObjectName("disconnectBtn")
        self._connect_btn.setStyle(self._connect_btn.style())
        self._poll_timer.start()
        self._update_ctrl_btn_state()

        self.gamepad_log.emit(
            f"手柄已连接: {device} → {det.profile.display_name} ({det.source})"
        )

    def _disconnect_gamepad(self):
        if self._ctrl_running:
            self._stop_control()

        self._poll_timer.stop()
        if self._joystick:
            self._joystick.disconnect()
            self._joystick = None
        self._detection = None

        self._connect_btn.setText("连接手柄")
        self._connect_btn.setObjectName("connectBtn")
        self._connect_btn.setStyle(self._connect_btn.style())
        self._info_label.setText("未连接")
        self._update_ctrl_btn_state()

        for bar in self._axis_bars:
            bar.set_value(0.0)
        for lbl in self._axis_labels:
            lbl.setText(" 0.000")
        for ind in self._btn_indicators:
            ind.set_pressed(False)
        self._mapped_display.setText("—")

        self.gamepad_log.emit("手柄已断开")

    # ---- Input Polling (20 Hz) ----

    def _poll_input(self):
        joy = self._joystick
        if not joy or not joy.connected:
            self._disconnect_gamepad()
            return

        for i in range(LinuxJoystick.MAX_AXES):
            v = joy.axes[i]
            self._axis_bars[i].set_value(v)
            self._axis_labels[i].setText(f"{v:+.3f}")

        for i in range(LinuxJoystick.MAX_BUTTONS):
            self._btn_indicators[i].set_pressed(bool(joy.buttons[i]))

        if self._detection:
            prof = self._detection.profile
            s = prof.sticks
            parts = []
            parts.append(f"LX={s.lx.read(joy.axes):+.2f}")
            parts.append(f"LY={s.ly.read(joy.axes):+.2f}")
            parts.append(f"RX={s.rx.read(joy.axes):+.2f}")
            parts.append(f"RY={s.ry.read(joy.axes):+.2f}")
            parts.append(f"LT={s.lt.read(joy.axes, joy.buttons):.2f}")
            parts.append(f"RT={s.rt.read(joy.axes, joy.buttons):.2f}")
            parts.append(f"DX={s.dpad_x.read(joy.axes):+.1f}")
            parts.append(f"DY={s.dpad_y.read(joy.axes):+.1f}")
            self._mapped_display.setText("  ".join(parts))

        if self._ctrl_running and self._controller:
            ctrl = self._controller
            idx = ctrl._speed_idx
            name, _ = SPEED_LEVELS[idx]
            self._speed_label.setText(f"{idx+1}/5 [{name}]")

            if ctrl._zero_torque:
                self._mode_label.setText("零力矩")
                self._mode_label.setStyleSheet("color: #e67e22; font-weight: bold;")
            elif ctrl._estop:
                self._mode_label.setText("急停")
                self._mode_label.setStyleSheet("color: #e74c3c; font-weight: bold;")
            else:
                self._mode_label.setText("正常")
                self._mode_label.setStyleSheet("color: #a6e3a1; font-weight: bold;")

            if ctrl._target_pose:
                p = ctrl._target_pose
                self._pose_label.setText(
                    f"XYZ: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m\n"
                    f"RPY: ({p.rx:.2f}, {p.ry:.2f}, {p.rz:.2f}) rad"
                )

            if ctrl.exit_requested:
                self._stop_control()
                self.gamepad_log.emit("手柄控制：收到退出请求")

    # ---- Arm Control ----

    def set_arm(self, arm):
        self._arm = arm
        self._update_ctrl_btn_state()

    def _update_ctrl_btn_state(self):
        can_start = (
            self._joystick is not None
            and self._joystick.connected
            and self._arm is not None
            and not self._ctrl_running
        )
        self._ctrl_btn.setEnabled(can_start or self._ctrl_running)
        if self._ctrl_running:
            self._ctrl_btn.setText("停止控制")
            self._ctrl_btn.setObjectName("disconnectBtn")
        else:
            self._ctrl_btn.setText("启动控制")
            self._ctrl_btn.setObjectName("enableBtn")
        self._ctrl_btn.setStyle(self._ctrl_btn.style())

    def _toggle_control(self):
        if self._ctrl_running:
            self._stop_control()
        else:
            self._start_control()

    def _start_control(self):
        if not self._joystick or not self._joystick.connected:
            self.gamepad_log.emit("请先连接手柄")
            return
        if not self._arm:
            self.gamepad_log.emit("请先连接机械臂")
            return
        if not self._detection:
            return

        from demo.xbox_control import XboxArmController

        self._controller = XboxArmController(
            arm=self._arm,
            joystick=self._joystick,
            profile=self._detection.profile,
            update_rate=100.0,
        )

        self._ctrl_running = True
        self._ctrl_thread = threading.Thread(
            target=self._run_control, daemon=True, name="gamepad_arm_ctrl"
        )
        self._ctrl_thread.start()
        self._update_ctrl_btn_state()
        self.gamepad_log.emit("手柄控臂已启动")

    def _run_control(self):
        try:
            self._controller.start()
        except Exception as e:
            logger.error("手柄控制异常: %s", e)
            self.gamepad_log.emit(f"手柄控制异常: {e}")
        finally:
            self._ctrl_running = False

    def _stop_control(self):
        if self._controller:
            self._controller.stop()
        if self._ctrl_thread:
            self._ctrl_thread.join(timeout=2.0)
            self._ctrl_thread = None
        self._controller = None
        self._ctrl_running = False
        self._update_ctrl_btn_state()

        self._mode_label.setText("—")
        self._mode_label.setStyleSheet("")
        self._pose_label.setText("—")
        self._speed_label.setText(f"{self._speed_idx+1}/5 [{SPEED_LEVELS[self._speed_idx][0]}]")

        self.gamepad_log.emit("手柄控臂已停止")

    # ---- Cleanup ----

    def cleanup(self):
        if self._ctrl_running:
            self._stop_control()
        if self._joystick and self._joystick.connected:
            self._joystick.disconnect()
        self._poll_timer.stop()
