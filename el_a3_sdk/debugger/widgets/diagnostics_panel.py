"""诊断面板：电机参数、CAN 总线统计、参数读写"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QGroupBox, QGridLayout,
    QTableWidget, QTableWidgetItem, QHeaderView,
    QSpinBox, QComboBox, QDoubleSpinBox, QTextEdit,
)
from PyQt6.QtCore import pyqtSignal, Qt, QTimer

PARAM_NAMES = {
    0x7005: "运行模式 (RUN_MODE)",
    0x7006: "电流给定 (IQ_REF)",
    0x700A: "速度给定 (SPD_REF)",
    0x700B: "力矩限制 (LIMIT_TORQUE)",
    0x7010: "电流Kp (CUR_KP)",
    0x7011: "电流Ki (CUR_KI)",
    0x7016: "位置给定 (LOC_REF)",
    0x7017: "速度限制 (LIMIT_SPD)",
    0x7018: "电流限制 (LIMIT_CUR)",
    0x7019: "机械位置 (MECH_POS)",
    0x701A: "滤波电流 (IQF)",
    0x701B: "机械速度 (MECH_VEL)",
    0x701C: "母线电压 (VBUS)",
    0x701E: "位置Kp (LOC_KP)",
    0x701F: "速度Kp (SPD_KP)",
    0x7020: "速度Ki (SPD_KI)",
}


class DiagnosticsPanel(QWidget):
    """电机诊断与 CAN 总线监控"""

    read_param_requested = pyqtSignal(int, int)   # motor_id, param_index
    write_param_requested = pyqtSignal(int, int, float)  # motor_id, param_index, value
    set_zero_requested = pyqtSignal(int)  # motor_num
    verify_zero_sta_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        motor_group = QGroupBox("电机状态")
        motor_layout = QVBoxLayout()

        self.motor_table = QTableWidget(7, 8)
        self.motor_table.setHorizontalHeaderLabels([
            "电机ID", "位置(rad)", "速度(rad/s)", "力矩(Nm)",
            "温度(°C)", "故障码", "模式", "使能"
        ])
        self.motor_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self.motor_table.setEditTriggers(
            QTableWidget.EditTrigger.NoEditTriggers
        )
        for i in range(7):
            self.motor_table.setItem(i, 0, QTableWidgetItem(str(i + 1)))
            for j in range(1, 8):
                self.motor_table.setItem(i, j, QTableWidgetItem("--"))
        self.motor_table.setMaximumHeight(200)
        self.motor_table.verticalHeader().setDefaultSectionSize(24)
        self.motor_table.verticalHeader().setVisible(False)
        motor_layout.addWidget(self.motor_table)
        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)

        param_group = QGroupBox("参数读写")
        param_vlayout = QVBoxLayout()
        param_vlayout.setSpacing(4)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("电机ID:"))
        self.motor_id_spin = QSpinBox()
        self.motor_id_spin.setRange(1, 7)
        self.motor_id_spin.setFixedWidth(50)
        row1.addWidget(self.motor_id_spin)
        row1.addSpacing(8)
        row1.addWidget(QLabel("参数:"))
        self.param_combo = QComboBox()
        for idx, name in PARAM_NAMES.items():
            self.param_combo.addItem(f"0x{idx:04X} - {name}", idx)
        row1.addWidget(self.param_combo, 1)
        param_vlayout.addLayout(row1)

        row2 = QHBoxLayout()
        self.read_btn = QPushButton("读取")
        self.read_btn.setFixedWidth(60)
        self.read_btn.clicked.connect(self._on_read)
        row2.addWidget(self.read_btn)
        row2.addSpacing(8)
        row2.addWidget(QLabel("值:"))
        self.value_spin = QDoubleSpinBox()
        self.value_spin.setRange(-10000, 10000)
        self.value_spin.setDecimals(4)
        self.value_spin.setFixedWidth(120)
        row2.addWidget(self.value_spin)
        row2.addSpacing(8)
        self.write_btn = QPushButton("写入")
        self.write_btn.setFixedWidth(60)
        self.write_btn.clicked.connect(self._on_write)
        row2.addWidget(self.write_btn)
        row2.addStretch()
        param_vlayout.addLayout(row2)

        param_group.setLayout(param_vlayout)
        layout.addWidget(param_group)

        verify_group = QGroupBox("参数校验")
        verify_layout = QVBoxLayout()
        verify_layout.setSpacing(4)

        verify_row = QHBoxLayout()
        self.verify_btn = QPushButton("校验 ZERO_STA")
        self.verify_btn.setToolTip("依次读取全部电机 ZERO_STA(0x7029) 参数，验证是否为 1")
        self.verify_btn.clicked.connect(self.verify_zero_sta_requested.emit)
        verify_row.addWidget(self.verify_btn)
        self.verify_status = QLabel("未校验")
        self.verify_status.setStyleSheet("font-weight: bold;")
        verify_row.addWidget(self.verify_status)
        verify_row.addStretch()
        verify_layout.addLayout(verify_row)

        self._verify_labels: list[QLabel] = []
        verify_grid = QGridLayout()
        verify_grid.setSpacing(2)
        for i in range(7):
            mid_lbl = QLabel(f"电机{i+1}:")
            mid_lbl.setFixedWidth(50)
            val_lbl = QLabel("—")
            val_lbl.setStyleSheet("font-family: monospace;")
            verify_grid.addWidget(mid_lbl, i // 4, (i % 4) * 2)
            verify_grid.addWidget(val_lbl, i // 4, (i % 4) * 2 + 1)
            self._verify_labels.append(val_lbl)
        verify_layout.addLayout(verify_grid)

        verify_group.setLayout(verify_layout)
        layout.addWidget(verify_group)

        op_group = QGroupBox("操作")
        op_layout = QHBoxLayout()

        zero_layout = QHBoxLayout()
        zero_layout.addWidget(QLabel("设零电机:"))
        self.zero_motor_spin = QSpinBox()
        self.zero_motor_spin.setRange(1, 7)
        zero_layout.addWidget(self.zero_motor_spin)
        zero_btn = QPushButton("设置零位")
        zero_btn.clicked.connect(
            lambda: self.set_zero_requested.emit(self.zero_motor_spin.value())
        )
        zero_layout.addWidget(zero_btn)

        zero_all_btn = QPushButton("全部设零")
        zero_all_btn.clicked.connect(lambda: self.set_zero_requested.emit(0xFF))
        zero_layout.addWidget(zero_all_btn)

        op_layout.addLayout(zero_layout)
        op_group.setLayout(op_layout)
        layout.addWidget(op_group)

        can_group = QGroupBox("CAN 总线状态")
        can_layout = QHBoxLayout()
        self.can_fps_label = QLabel("FPS: --")
        can_layout.addWidget(self.can_fps_label)
        self.can_tx_label = QLabel("TX: --")
        can_layout.addWidget(self.can_tx_label)
        self.can_state_label = QLabel("状态: --")
        can_layout.addWidget(self.can_state_label)
        can_group.setLayout(can_layout)
        layout.addWidget(can_group)

    def _on_read(self):
        motor_id = self.motor_id_spin.value()
        param_idx = self.param_combo.currentData()
        if param_idx is not None:
            self.read_param_requested.emit(motor_id, param_idx)

    def _on_write(self):
        motor_id = self.motor_id_spin.value()
        param_idx = self.param_combo.currentData()
        value = self.value_spin.value()
        if param_idx is not None:
            self.write_param_requested.emit(motor_id, param_idx, value)

    def update_motor_states(self, joint_pos, joint_vel=None, joint_eff=None):
        """更新电机状态表格"""
        pos_list = joint_pos.to_list(include_gripper=True) if joint_pos else [0]*7
        vel_list = joint_vel.to_list(include_gripper=True) if joint_vel else [0]*7
        eff_list = joint_eff.to_list(include_gripper=True) if joint_eff else [0]*7
        for i in range(7):
            self.motor_table.item(i, 1).setText(f"{pos_list[i]:.4f}")
            self.motor_table.item(i, 2).setText(f"{vel_list[i]:.4f}")
            self.motor_table.item(i, 3).setText(f"{eff_list[i]:.4f}")

    def update_motor_feedback(self, feedbacks):
        """从 MotorFeedback 列表更新温度/故障等"""
        for fb in feedbacks:
            if hasattr(fb, 'motor_id'):
                row = fb.motor_id - 1
                if 0 <= row < 7:
                    if hasattr(fb, 'temperature'):
                        self.motor_table.item(row, 4).setText(f"{fb.temperature:.1f}")
                    if hasattr(fb, 'fault_code'):
                        self.motor_table.item(row, 5).setText(f"0x{fb.fault_code:02X}")
                    if hasattr(fb, 'mode_state'):
                        self.motor_table.item(row, 6).setText(str(fb.mode_state))
                    if hasattr(fb, 'is_valid'):
                        self.motor_table.item(row, 7).setText("是" if fb.is_valid else "否")

    def update_zero_sta_result(self, results: list):
        """更新 ZERO_STA 校验结果: [(motor_id, value, success), ...]"""
        all_ok = True
        for motor_id, value, success in results:
            idx = motor_id - 1
            if 0 <= idx < len(self._verify_labels):
                if not success:
                    self._verify_labels[idx].setText("读取失败")
                    self._verify_labels[idx].setStyleSheet(
                        "font-family: monospace; color: #f38ba8; font-weight: bold;")
                    all_ok = False
                elif abs(value - 1.0) < 0.01:
                    self._verify_labels[idx].setText(f"{value:.0f} ✓")
                    self._verify_labels[idx].setStyleSheet(
                        "font-family: monospace; color: #a6e3a1; font-weight: bold;")
                else:
                    self._verify_labels[idx].setText(f"{value:.0f} ✗")
                    self._verify_labels[idx].setStyleSheet(
                        "font-family: monospace; color: #f38ba8; font-weight: bold;")
                    all_ok = False

        if all_ok:
            self.verify_status.setText("全部通过")
            self.verify_status.setStyleSheet("font-weight: bold; color: #a6e3a1;")
        else:
            self.verify_status.setText("存在异常")
            self.verify_status.setStyleSheet("font-weight: bold; color: #f38ba8;")

    def update_can_stats(self, fps: float):
        self.can_fps_label.setText(f"FPS: {fps:.0f}")
