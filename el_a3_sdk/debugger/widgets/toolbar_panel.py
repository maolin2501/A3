"""工具栏面板：CAN 端口选择/开启 + 连接/使能/急停/状态指示（单行）"""

from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QLabel, QPushButton,
    QComboBox, QMessageBox,
)
from PyQt6.QtCore import pyqtSignal, Qt

from debugger.utils.can_utils import (
    detect_can_interfaces, get_can_state, get_can_bitrate,
    setup_can_interface, shutdown_can_interface,
)

BITRATE_OPTIONS = [
    ("1M", 1000000),
    ("500K", 500000),
    ("250K", 250000),
]


class ToolbarPanel(QWidget):
    """顶部工具栏（单行固定高度）"""

    connect_requested = pyqtSignal(str)
    disconnect_requested = pyqtSignal()
    enable_requested = pyqtSignal()
    disable_requested = pyqtSignal()
    emergency_stop_requested = pyqtSignal()
    open_monitor_requested = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False
        self._enabled = False
        self._interfaces = []
        self.setFixedHeight(44)
        self.setStyleSheet(
            "ToolbarPanel QPushButton { padding: 4px 8px; min-height: 18px; }"
            "ToolbarPanel QComboBox { min-height: 18px; padding: 2px 4px; }"
            "ToolbarPanel QCheckBox { min-height: 18px; }"
        )
        self._init_ui()
        self._refresh_interfaces()

    def _init_ui(self):
        row = QHBoxLayout(self)
        row.setContentsMargins(6, 3, 6, 3)
        row.setSpacing(5)

        row.addWidget(QLabel("CAN:"))
        self.can_combo = QComboBox()
        self.can_combo.setFixedWidth(120)
        self.can_combo.setEditable(True)
        self.can_combo.lineEdit().setPlaceholderText("can0")
        row.addWidget(self.can_combo)

        self.refresh_btn = QPushButton("⟳")
        self.refresh_btn.setFixedSize(28, 28)
        self.refresh_btn.setToolTip("刷新 CAN 接口")
        self.refresh_btn.clicked.connect(self._refresh_interfaces)
        row.addWidget(self.refresh_btn)

        self.can_toggle_btn = QPushButton("开启")
        self.can_toggle_btn.setFixedWidth(46)
        self.can_toggle_btn.clicked.connect(self._on_can_toggle)
        row.addWidget(self.can_toggle_btn)

        self.bitrate_combo = QComboBox()
        self.bitrate_combo.setFixedWidth(60)
        for label, val in BITRATE_OPTIONS:
            self.bitrate_combo.addItem(label, val)
        self.bitrate_combo.setCurrentIndex(0)
        row.addWidget(self.bitrate_combo)

        self._add_sep(row)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.setObjectName("connectBtn")
        self.connect_btn.setFixedWidth(60)
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        row.addWidget(self.connect_btn)

        self.enable_btn = QPushButton("使能")
        self.enable_btn.setObjectName("enableBtn")
        self.enable_btn.setFixedWidth(60)
        self.enable_btn.setEnabled(False)
        self.enable_btn.clicked.connect(self._on_enable_clicked)
        row.addWidget(self.enable_btn)

        self._add_sep(row)

        self.estop_btn = QPushButton("急 停")
        self.estop_btn.setObjectName("emergencyStop")
        self.estop_btn.setFixedWidth(60)
        self.estop_btn.clicked.connect(self.emergency_stop_requested.emit)
        row.addWidget(self.estop_btn)

        self._add_sep(row)

        self.monitor_btn = QPushButton("📊 监控")
        self.monitor_btn.setFixedWidth(76)
        self.monitor_btn.setToolTip("打开实时数据监控窗口")
        self.monitor_btn.clicked.connect(self.open_monitor_requested.emit)
        row.addWidget(self.monitor_btn)

        row.addStretch()

        self.status_label = QLabel("未连接")
        self.status_label.setObjectName("statusLabel")
        row.addWidget(self.status_label)

        self.fps_label = QLabel("FPS: --")
        self.fps_label.setObjectName("fpsLabel")
        self.fps_label.setFixedWidth(70)
        row.addWidget(self.fps_label)

    @staticmethod
    def _add_sep(layout):
        sep = QLabel("|")
        sep.setStyleSheet("color: #585b70; margin: 0 2px;")
        layout.addWidget(sep)

    # ---- CAN 端口管理 ----

    def _refresh_interfaces(self):
        prev_text = self.can_combo.currentText().split(" ")[0].strip()
        self.can_combo.clear()

        self._interfaces = detect_can_interfaces()
        restore_idx = -1

        for i, iface in enumerate(self._interfaces):
            name = iface["name"]
            state = iface["state"]
            bitrate = iface["bitrate"]
            br_str = f" {bitrate // 1000}K" if bitrate else ""
            label = f"{name} ({state}{br_str})"
            self.can_combo.addItem(label, name)
            if name == prev_text:
                restore_idx = i
            if state == "UP" and bitrate:
                self._sync_bitrate_combo(bitrate)

        if not self._interfaces:
            self.can_combo.addItem("can0 (未检测到)")
            self.can_combo.setItemData(0, "can0")

        if restore_idx >= 0:
            self.can_combo.setCurrentIndex(restore_idx)

        self._update_can_toggle_label()

    def _sync_bitrate_combo(self, bitrate: int):
        for i in range(self.bitrate_combo.count()):
            if self.bitrate_combo.itemData(i) == bitrate:
                self.bitrate_combo.setCurrentIndex(i)
                return

    def _get_selected_can_name(self) -> str:
        idx = self.can_combo.currentIndex()
        if idx >= 0 and self.can_combo.itemData(idx):
            return self.can_combo.itemData(idx)
        raw = self.can_combo.currentText().split(" ")[0].strip()
        return raw or "can0"

    def _get_selected_state(self) -> str:
        name = self._get_selected_can_name()
        for iface in self._interfaces:
            if iface["name"] == name:
                return iface["state"]
        return get_can_state(name)

    def _update_can_toggle_label(self):
        state = self._get_selected_state()
        if state == "UP":
            self.can_toggle_btn.setText("关闭")
        else:
            self.can_toggle_btn.setText("开启")

    def _on_can_toggle(self):
        name = self._get_selected_can_name()
        state = self._get_selected_state()

        if state == "UP":
            ok, msg = shutdown_can_interface(name)
        else:
            bitrate = self.bitrate_combo.currentData() or 1000000
            ok, msg = setup_can_interface(name, bitrate)

        if ok:
            self._refresh_interfaces()
        else:
            QMessageBox.warning(self, "CAN 接口操作失败", msg)
            self._refresh_interfaces()

    # ---- 连接 / 使能 / 循环 ----

    def _on_connect_clicked(self):
        if not self._connected:
            can_name = self._get_selected_can_name()
            self.connect_requested.emit(can_name)
        else:
            self.disconnect_requested.emit()

    def _on_enable_clicked(self):
        if not self._enabled:
            self.enable_requested.emit()
        else:
            self.disable_requested.emit()

    # ---- 外部状态更新 ----

    def set_connected(self, connected: bool):
        self._connected = connected
        can_area_enabled = not connected
        if connected:
            self.connect_btn.setText("断开")
            self.connect_btn.setObjectName("disconnectBtn")
            self.enable_btn.setEnabled(True)
            self.status_label.setText("已连接")
        else:
            self.connect_btn.setText("连接")
            self.connect_btn.setObjectName("connectBtn")
            self.enable_btn.setEnabled(False)
            self._enabled = False
            self.status_label.setText("未连接")

        self.can_combo.setEnabled(can_area_enabled)
        self.refresh_btn.setEnabled(can_area_enabled)
        self.can_toggle_btn.setEnabled(can_area_enabled)
        self.bitrate_combo.setEnabled(can_area_enabled)

        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)

    def set_enabled(self, enabled: bool):
        self._enabled = enabled
        if enabled:
            self.enable_btn.setText("失能")
            self.status_label.setText("已使能")
            self.status_label.setObjectName("statusLabel")
        else:
            self.enable_btn.setText("使能")
            if self._connected:
                self.status_label.setText("已连接")
            self.status_label.setObjectName("statusLabel")
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)

    def set_fps(self, fps: float):
        self.fps_label.setText(f"FPS: {fps:.0f}")

    def set_error(self, msg: str):
        self.status_label.setText(f"错误: {msg}")
        self.status_label.setObjectName("errorLabel")
        self.status_label.style().unpolish(self.status_label)
        self.status_label.style().polish(self.status_label)
