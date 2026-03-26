"""轨迹控制面板：MoveJ / MoveL / 路径点管理"""

import math
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QDoubleSpinBox, QPushButton, QGroupBox, QGridLayout,
    QTableWidget, QTableWidgetItem, QHeaderView, QTabWidget,
)
from PyQt6.QtCore import pyqtSignal, Qt


class TrajectoryPanel(QWidget):
    """轨迹控制：MoveJ、MoveL、路径点"""

    move_j_requested = pyqtSignal(list, float)  # positions, duration
    move_l_requested = pyqtSignal(list, float)   # [x,y,z,rx,ry,rz], duration
    end_pose_requested = pyqtSignal(float, float, float, float, float, float, float)
    cancel_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._current_positions = [0.0] * 6
        self._waypoints = []
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        tabs = QTabWidget()

        tabs.addTab(self._create_movej_tab(), "MoveJ")
        tabs.addTab(self._create_movel_tab(), "MoveL")
        tabs.addTab(self._create_waypoint_tab(), "路径点")

        layout.addWidget(tabs)

        btn_layout = QHBoxLayout()
        self.cancel_btn = QPushButton("取消运动")
        self.cancel_btn.setStyleSheet("background-color: #e67e22; color: white;")
        self.cancel_btn.clicked.connect(self.cancel_requested.emit)
        btn_layout.addWidget(self.cancel_btn)

        self.status_label = QLabel("就绪")
        btn_layout.addWidget(self.status_label)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

    def _create_movej_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        group = QGroupBox("关节目标 (°)")
        grid = QGridLayout()
        self._movej_spins = []
        joint_names = ["L1", "L2", "L3", "L4", "L5", "L6"]
        limits = [(-160, 160), (0, 210), (-230, 0), (-90, 90), (-90, 90), (-90, 90)]
        for i in range(6):
            grid.addWidget(QLabel(joint_names[i]), i // 3, (i % 3) * 2)
            spin = QDoubleSpinBox()
            lo, hi = limits[i]
            spin.setRange(lo, hi)
            spin.setDecimals(2)
            spin.setSuffix("°")
            spin.setValue(0.0 if lo <= 0 <= hi else lo)
            self._movej_spins.append(spin)
            grid.addWidget(spin, i // 3, (i % 3) * 2 + 1)
        group.setLayout(grid)
        layout.addWidget(group)

        dur_layout = QHBoxLayout()
        dur_layout.addWidget(QLabel("运动时间:"))
        self.movej_duration = QDoubleSpinBox()
        self.movej_duration.setRange(0.5, 30.0)
        self.movej_duration.setValue(2.0)
        self.movej_duration.setSuffix(" s")
        dur_layout.addWidget(self.movej_duration)
        dur_layout.addStretch()

        exec_btn = QPushButton("执行 MoveJ")
        exec_btn.setObjectName("enableBtn")
        exec_btn.clicked.connect(self._on_exec_movej)
        dur_layout.addWidget(exec_btn)
        layout.addLayout(dur_layout)

        read_btn = QPushButton("读取当前位置")
        read_btn.clicked.connect(self._fill_current_positions)
        layout.addWidget(read_btn)

        layout.addStretch()
        return w

    def _create_movel_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        group = QGroupBox("笛卡尔目标")
        grid = QGridLayout()
        labels = ["X (m)", "Y (m)", "Z (m)", "Rx (°)", "Ry (°)", "Rz (°)"]
        defaults = [0.3, 0.0, 0.3, 0.0, 0.0, 0.0]
        ranges = [(-1, 1), (-1, 1), (0, 1), (-180, 180), (-180, 180), (-180, 180)]
        self._movel_spins = []
        for i, (label, default, (lo, hi)) in enumerate(zip(labels, defaults, ranges)):
            grid.addWidget(QLabel(label), i // 3, (i % 3) * 2)
            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setDecimals(4 if i < 3 else 2)
            spin.setSingleStep(0.01 if i < 3 else 1.0)
            spin.setValue(default)
            self._movel_spins.append(spin)
            grid.addWidget(spin, i // 3, (i % 3) * 2 + 1)
        group.setLayout(grid)
        layout.addWidget(group)

        dur_layout = QHBoxLayout()
        dur_layout.addWidget(QLabel("运动时间:"))
        self.movel_duration = QDoubleSpinBox()
        self.movel_duration.setRange(0.5, 30.0)
        self.movel_duration.setValue(2.0)
        self.movel_duration.setSuffix(" s")
        dur_layout.addWidget(self.movel_duration)
        dur_layout.addStretch()

        exec_btn = QPushButton("执行 MoveL")
        exec_btn.setObjectName("enableBtn")
        exec_btn.clicked.connect(self._on_exec_movel)
        dur_layout.addWidget(exec_btn)
        layout.addLayout(dur_layout)

        layout.addStretch()
        return w

    def _create_waypoint_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)

        self.waypoint_table = QTableWidget(0, 7)
        self.waypoint_table.setHorizontalHeaderLabels(
            ["L1°", "L2°", "L3°", "L4°", "L5°", "L6°", "时间(s)"]
        )
        self.waypoint_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        layout.addWidget(self.waypoint_table)

        btn_layout = QHBoxLayout()
        add_btn = QPushButton("添加当前位置")
        add_btn.clicked.connect(self._add_current_waypoint)
        btn_layout.addWidget(add_btn)

        del_btn = QPushButton("删除选中")
        del_btn.clicked.connect(self._delete_selected_waypoint)
        btn_layout.addWidget(del_btn)

        clear_btn = QPushButton("清空")
        clear_btn.clicked.connect(lambda: self.waypoint_table.setRowCount(0))
        btn_layout.addWidget(clear_btn)

        btn_layout.addStretch()

        exec_all_btn = QPushButton("顺序执行")
        exec_all_btn.setObjectName("enableBtn")
        exec_all_btn.clicked.connect(self._exec_waypoints)
        btn_layout.addWidget(exec_all_btn)

        layout.addLayout(btn_layout)
        return w

    def _on_exec_movej(self):
        positions = [math.radians(s.value()) for s in self._movej_spins]
        duration = self.movej_duration.value()
        self.move_j_requested.emit(positions, duration)
        self.status_label.setText("MoveJ 执行中...")

    def _on_exec_movel(self):
        x = self._movel_spins[0].value()
        y = self._movel_spins[1].value()
        z = self._movel_spins[2].value()
        rx = math.radians(self._movel_spins[3].value())
        ry = math.radians(self._movel_spins[4].value())
        rz = math.radians(self._movel_spins[5].value())
        duration = self.movel_duration.value()
        self.end_pose_requested.emit(x, y, z, rx, ry, rz, duration)
        self.status_label.setText("MoveL 执行中...")

    def _fill_current_positions(self):
        for i in range(min(6, len(self._current_positions))):
            self._movej_spins[i].setValue(math.degrees(self._current_positions[i]))

    def update_current_positions(self, joint_states):
        positions = joint_states.to_list(include_gripper=False)
        self._current_positions = positions[:6]

    def _add_current_waypoint(self):
        row = self.waypoint_table.rowCount()
        self.waypoint_table.insertRow(row)
        for i in range(6):
            deg = math.degrees(self._current_positions[i]) if i < len(self._current_positions) else 0.0
            self.waypoint_table.setItem(row, i, QTableWidgetItem(f"{deg:.2f}"))
        self.waypoint_table.setItem(row, 6, QTableWidgetItem("2.0"))

    def _delete_selected_waypoint(self):
        rows = set(idx.row() for idx in self.waypoint_table.selectedIndexes())
        for row in sorted(rows, reverse=True):
            self.waypoint_table.removeRow(row)

    def _exec_waypoints(self):
        for row in range(self.waypoint_table.rowCount()):
            positions = []
            for col in range(6):
                item = self.waypoint_table.item(row, col)
                val = float(item.text()) if item else 0.0
                positions.append(math.radians(val))
            dur_item = self.waypoint_table.item(row, 6)
            duration = float(dur_item.text()) if dur_item else 2.0
            self.move_j_requested.emit(positions, duration)
        self.status_label.setText(f"执行 {self.waypoint_table.rowCount()} 个路径点...")
