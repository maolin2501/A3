"""夹爪控制面板"""

import math
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QDoubleSpinBox, QPushButton, QGroupBox,
)
from PyQt6.QtCore import pyqtSignal, Qt


class GripperPanel(QWidget):
    """夹爪控制：角度滑块、全开/全关、设零"""

    gripper_command = pyqtSignal(float)  # angle in rad
    set_zero_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._updating = False
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        group = QGroupBox("夹爪控制 (L7)")
        g_layout = QVBoxLayout()

        angle_layout = QHBoxLayout()
        angle_layout.addWidget(QLabel("角度:"))
        self.angle_slider = QSlider(Qt.Orientation.Horizontal)
        self.angle_slider.setRange(-900, 900)
        self.angle_slider.setValue(0)
        self.angle_slider.valueChanged.connect(self._on_slider_changed)
        angle_layout.addWidget(self.angle_slider)

        self.angle_spin = QDoubleSpinBox()
        self.angle_spin.setRange(-90.0, 90.0)
        self.angle_spin.setDecimals(1)
        self.angle_spin.setSuffix("°")
        self.angle_spin.setFixedWidth(100)
        self.angle_spin.valueChanged.connect(self._on_spin_changed)
        angle_layout.addWidget(self.angle_spin)
        g_layout.addLayout(angle_layout)

        fb_layout = QHBoxLayout()
        fb_layout.addWidget(QLabel("实际角度:"))
        self.fb_label = QLabel("0.0°")
        self.fb_label.setStyleSheet("color: #94e2d5; font-weight: bold;")
        fb_layout.addWidget(self.fb_label)
        fb_layout.addSpacing(20)
        fb_layout.addWidget(QLabel("力矩:"))
        self.torque_label = QLabel("0.00 Nm")
        self.torque_label.setStyleSheet("color: #f9e2af;")
        fb_layout.addWidget(self.torque_label)
        fb_layout.addStretch()
        g_layout.addLayout(fb_layout)

        btn_layout = QHBoxLayout()
        send_btn = QPushButton("发送")
        send_btn.setObjectName("enableBtn")
        send_btn.clicked.connect(self._send_command)
        btn_layout.addWidget(send_btn)

        open_btn = QPushButton("全开")
        open_btn.clicked.connect(lambda: self._set_angle(90.0))
        btn_layout.addWidget(open_btn)

        close_btn = QPushButton("全关")
        close_btn.clicked.connect(lambda: self._set_angle(0.0))
        btn_layout.addWidget(close_btn)

        zero_btn = QPushButton("设零位")
        zero_btn.clicked.connect(self.set_zero_requested.emit)
        btn_layout.addWidget(zero_btn)

        btn_layout.addStretch()
        g_layout.addLayout(btn_layout)

        group.setLayout(g_layout)
        layout.addWidget(group)
        layout.addStretch()

    def _on_slider_changed(self, val):
        if self._updating:
            return
        self._updating = True
        self.angle_spin.setValue(val / 10.0)
        self._updating = False

    def _on_spin_changed(self, val):
        if self._updating:
            return
        self._updating = True
        self.angle_slider.setValue(int(val * 10))
        self._updating = False

    def _set_angle(self, deg):
        self._updating = True
        self.angle_spin.setValue(deg)
        self.angle_slider.setValue(int(deg * 10))
        self._updating = False
        self._send_command()

    def _send_command(self):
        angle_rad = math.radians(self.angle_spin.value())
        self.gripper_command.emit(angle_rad)

    def update_feedback(self, joint_states, effort_states=None):
        positions = joint_states.to_list(include_gripper=True)
        if len(positions) >= 7:
            deg = math.degrees(positions[6])
            self.fb_label.setText(f"{deg:.1f}°")
        if effort_states:
            torques = effort_states.to_list(include_gripper=True)
            if len(torques) >= 7:
                self.torque_label.setText(f"{torques[6]:.2f} Nm")
