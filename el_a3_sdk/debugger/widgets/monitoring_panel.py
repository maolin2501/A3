"""实时监控面板：4 通道 pyqtgraph 实时曲线"""

import numpy as np
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QGridLayout, QFileDialog, QLabel,
)
from PyQt6.QtCore import QTimer, Qt

import pyqtgraph as pg

from debugger.utils.style import JOINT_COLORS
from debugger.backend.data_buffer import DataBuffer

pg.setConfigOptions(antialias=True, background="#1e1e2e", foreground="#cdd6f4")

CHANNEL_NAMES = ["L1", "L2", "L3", "L4", "L5", "L6", "L7"]
PLOT_TITLES = ["关节位置 (rad)", "关节速度 (rad/s)", "关节力矩 (Nm)", "关节温度 (°C)"]


class MonitoringPanel(QWidget):
    """2x2 实时曲线监控面板"""

    def __init__(self, data_buffer: DataBuffer, parent=None):
        super().__init__(parent)
        self.data_buffer = data_buffer
        self._paused = False
        self._plots = []
        self._curves = []
        self._init_ui()
        self._start_timer()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        top_bar = QHBoxLayout()
        self.pause_btn = QPushButton("暂停")
        self.pause_btn.setFixedWidth(60)
        self.pause_btn.clicked.connect(self._toggle_pause)
        top_bar.addWidget(self.pause_btn)

        self.clear_btn = QPushButton("清空")
        self.clear_btn.setFixedWidth(60)
        self.clear_btn.clicked.connect(self._clear_data)
        top_bar.addWidget(self.clear_btn)

        self.export_btn = QPushButton("导出 CSV")
        self.export_btn.setFixedWidth(80)
        self.export_btn.clicked.connect(self._export_csv)
        top_bar.addWidget(self.export_btn)

        top_bar.addSpacing(20)
        for ch in range(7):
            swatch = QLabel(f"■ {CHANNEL_NAMES[ch]}")
            swatch.setStyleSheet(
                f"color: {JOINT_COLORS[ch]}; font-weight: bold; font-size: 11px;"
            )
            top_bar.addWidget(swatch)

        top_bar.addStretch()
        layout.addLayout(top_bar)

        grid = QGridLayout()
        grid.setSpacing(4)

        for idx, title in enumerate(PLOT_TITLES):
            pw = pg.PlotWidget(title=title)
            pw.setLabel("bottom", "时间", units="s")
            pw.showGrid(x=True, y=True, alpha=0.3)

            curves = []
            for ch in range(7):
                pen = pg.mkPen(color=JOINT_COLORS[ch], width=1.5)
                curve = pw.plot([], [], pen=pen)
                curves.append(curve)

            self._plots.append(pw)
            self._curves.append(curves)
            grid.addWidget(pw, idx // 2, idx % 2)

        layout.addLayout(grid)

    def _start_timer(self):
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_plots)
        self._timer.start(50)  # 20Hz display refresh

    def _update_plots(self):
        if self._paused or self.data_buffer.count == 0:
            return

        ts, pos, vel, torq, temp = self.data_buffer.get_data()
        if len(ts) == 0:
            return

        t_rel = ts - ts[0]
        datasets = [pos, vel, torq, temp]

        for plot_idx, data_2d in enumerate(datasets):
            for ch in range(7):
                self._curves[plot_idx][ch].setData(t_rel, data_2d[:, ch])

    def _toggle_pause(self):
        self._paused = not self._paused
        self.pause_btn.setText("继续" if self._paused else "暂停")

    def _clear_data(self):
        self.data_buffer.clear()
        for plot_curves in self._curves:
            for c in plot_curves:
                c.setData([], [])

    def _export_csv(self):
        filepath, _ = QFileDialog.getSaveFileName(
            self, "导出数据", "arm_data.csv", "CSV Files (*.csv)"
        )
        if filepath:
            self.data_buffer.export_csv(filepath)
