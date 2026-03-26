"""EL-A3 机械臂调试上位机 - 入口"""

import sys
import os
import argparse
import logging
from pathlib import Path

SDK_ROOT = Path(__file__).parent.parent
REPO_ROOT = SDK_ROOT.parent

if str(SDK_ROOT) not in sys.path:
    sys.path.insert(0, str(SDK_ROOT))
if str(SDK_ROOT.parent) not in sys.path:
    sys.path.insert(0, str(SDK_ROOT.parent))

os.environ.setdefault("QT_API", "pyqt6")


def main():
    parser = argparse.ArgumentParser(description="EL-A3 机械臂调试上位机")
    parser.add_argument("--can", default="can0", help="CAN 接口名称 (default: can0)")
    parser.add_argument("--sim", action="store_true", help="模拟模式（无需硬件）")
    parser.add_argument(
        "--urdf",
        default=str(SDK_ROOT / "resources" / "urdf" / "el_a3.urdf"),
        help="URDF 文件路径",
    )
    parser.add_argument(
        "--meshes",
        default=str(REPO_ROOT / "el_a3_ros" / "el_a3_description" / "meshes"),
        help="STL mesh 文件目录",
    )
    parser.add_argument(
        "--log-level", default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="日志级别",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    from PyQt6.QtWidgets import QApplication
    from PyQt6.QtCore import Qt

    app = QApplication(sys.argv)
    app.setApplicationName("EL-A3 Debugger")

    from debugger.utils.style import DARK_THEME
    app.setStyleSheet(DARK_THEME)

    from debugger.main_window import MainWindow
    window = MainWindow(
        urdf_path=args.urdf,
        mesh_dir=args.meshes,
    )
    window.show()

    if args.sim:
        from PyQt6.QtCore import QTimer
        QTimer.singleShot(200, lambda: window.toolbar.sim_check.setChecked(True))

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
