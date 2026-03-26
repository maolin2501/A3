"""实时监控弹出窗口：独立窗口显示 4 通道实时曲线"""

from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtCore import Qt

from debugger.widgets.monitoring_panel import MonitoringPanel
from debugger.backend.data_buffer import DataBuffer


class MonitoringWindow(QMainWindow):
    """独立弹出的实时数据监控窗口"""

    def __init__(self, data_buffer: DataBuffer, parent=None):
        super().__init__(parent)
        self.setWindowTitle("实时数据监控")
        self.setMinimumSize(900, 600)
        self.resize(1100, 700)
        self.setWindowFlags(
            Qt.WindowType.Window
            | Qt.WindowType.WindowCloseButtonHint
            | Qt.WindowType.WindowMinMaxButtonsHint
        )

        self.panel = MonitoringPanel(data_buffer)
        self.setCentralWidget(self.panel)

    def closeEvent(self, event):
        self.hide()
        event.ignore()
