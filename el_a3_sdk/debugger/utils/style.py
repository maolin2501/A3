"""深色主题 QSS 样式表"""

DARK_THEME = """
QMainWindow, QWidget {
    background-color: #1e1e2e;
    color: #cdd6f4;
    font-family: "Noto Sans CJK SC", "Source Han Sans SC", "Microsoft YaHei", sans-serif;
    font-size: 13px;
}

QMenuBar {
    background-color: #181825;
    color: #cdd6f4;
    border-bottom: 1px solid #313244;
}
QMenuBar::item:selected { background-color: #313244; }
QMenu {
    background-color: #1e1e2e;
    color: #cdd6f4;
    border: 1px solid #313244;
}
QMenu::item:selected { background-color: #45475a; }

QToolBar {
    background-color: #181825;
    border-bottom: 1px solid #313244;
    spacing: 6px;
    padding: 4px;
}

QStatusBar {
    background-color: #181825;
    color: #a6adc8;
    border-top: 1px solid #313244;
}

QDockWidget {
    color: #cdd6f4;
    titlebar-close-icon: none;
}
QDockWidget::title {
    background-color: #181825;
    padding: 6px;
    border: 1px solid #313244;
}

QTabWidget::pane {
    border: 1px solid #313244;
    background-color: #1e1e2e;
}
QTabBar::tab {
    background-color: #181825;
    color: #a6adc8;
    padding: 8px 16px;
    border: 1px solid #313244;
    border-bottom: none;
    margin-right: 2px;
}
QTabBar::tab:selected {
    background-color: #1e1e2e;
    color: #89b4fa;
    border-bottom: 2px solid #89b4fa;
}
QTabBar::tab:hover { color: #cdd6f4; }

QPushButton {
    background-color: #313244;
    color: #cdd6f4;
    border: 1px solid #45475a;
    border-radius: 4px;
    padding: 6px 14px;
    min-height: 20px;
}
QPushButton:hover { background-color: #45475a; }
QPushButton:pressed { background-color: #585b70; }
QPushButton:disabled { color: #6c7086; background-color: #1e1e2e; }
QPushButton#emergencyStop {
    background-color: #c0392b;
    color: white;
    font-weight: bold;
    font-size: 14px;
    border: 2px solid #e74c3c;
    min-width: 60px;
}
QPushButton#emergencyStop:hover { background-color: #e74c3c; }
QPushButton#connectBtn { background-color: #2ecc71; color: white; }
QPushButton#connectBtn:hover { background-color: #27ae60; }
QPushButton#disconnectBtn { background-color: #e67e22; color: white; }
QPushButton#disconnectBtn:hover { background-color: #d35400; }
QPushButton#enableBtn { background-color: #3498db; color: white; }
QPushButton#enableBtn:hover { background-color: #2980b9; }

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background-color: #313244;
    color: #cdd6f4;
    border: 1px solid #45475a;
    border-radius: 3px;
    padding: 4px 6px;
    min-height: 22px;
}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {
    border: 1px solid #89b4fa;
}

QSlider::groove:horizontal {
    border: none;
    height: 6px;
    background: #45475a;
    border-radius: 3px;
}
QSlider::handle:horizontal {
    background: #89b4fa;
    border: none;
    width: 16px;
    height: 16px;
    margin: -5px 0;
    border-radius: 8px;
}
QSlider::handle:horizontal:hover { background: #b4befe; }

QTableWidget {
    background-color: #1e1e2e;
    color: #cdd6f4;
    gridline-color: #313244;
    border: 1px solid #313244;
    selection-background-color: #45475a;
}
QTableWidget::item { padding: 4px; }
QHeaderView::section {
    background-color: #181825;
    color: #a6adc8;
    padding: 6px;
    border: 1px solid #313244;
    font-weight: bold;
}

QGroupBox {
    border: 1px solid #313244;
    border-radius: 4px;
    margin-top: 12px;
    padding-top: 16px;
    font-weight: bold;
    color: #89b4fa;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
}

QLabel#statusLabel { color: #a6e3a1; font-weight: bold; }
QLabel#errorLabel { color: #f38ba8; font-weight: bold; }
QLabel#fpsLabel { color: #f9e2af; }

QCheckBox { color: #cdd6f4; spacing: 6px; }
QCheckBox::indicator {
    width: 16px; height: 16px;
    border: 1px solid #45475a;
    border-radius: 3px;
    background-color: #313244;
}
QCheckBox::indicator:checked { background-color: #89b4fa; border-color: #89b4fa; }

QScrollBar:vertical {
    background: #1e1e2e;
    width: 10px;
    border: none;
}
QScrollBar::handle:vertical {
    background: #45475a;
    border-radius: 5px;
    min-height: 20px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }

QTextEdit#logConsole {
    background-color: #11111b;
    color: #a6adc8;
    font-family: "JetBrains Mono", "Fira Code", monospace;
    font-size: 12px;
    border: none;
}
"""

JOINT_COLORS = [
    "#f38ba8",  # L1 - red
    "#fab387",  # L2 - peach
    "#f9e2af",  # L3 - yellow
    "#a6e3a1",  # L4 - green
    "#89b4fa",  # L5 - blue
    "#cba6f7",  # L6 - mauve
    "#94e2d5",  # L7 - teal
]
