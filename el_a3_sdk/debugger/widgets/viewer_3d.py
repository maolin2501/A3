"""3D URDF 可视化面板 (PyVistaQt)"""

import time
import numpy as np
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout, QPushButton
from PyQt6.QtCore import Qt

logger = logging.getLogger("debugger.viewer3d")

try:
    import pyvista as pv
    from pyvistaqt import QtInteractor
    import vtkmodules.vtkRenderingCore as vtk_rc
    import vtkmodules.vtkCommonTransforms as vtk_tr
    HAS_PYVISTA = True
except ImportError:
    HAS_PYVISTA = False
    logger.warning("pyvista / pyvistaqt 未安装，3D 可视化不可用")

from debugger.utils.urdf_loader import UrdfModel, _make_transform

URDF_PATH = Path(__file__).parent.parent.parent / "resources" / "urdf" / "el_a3.urdf"
MESH_DIR = Path(__file__).parent.parent.parent.parent / "el_a3_ros" / "el_a3_description" / "meshes"

JOINT_NAMES_ORDERED = [
    "L1_joint", "L2_joint", "L3_joint",
    "L4_joint", "L5_joint", "L6_joint", "L7_joint",
]

UPDATE_INTERVAL_S = 0.1  # 10 Hz max 3D refresh


class Viewer3DPanel(QWidget):
    """3D URDF 实时可视化"""

    def __init__(self, parent=None, urdf_path=None, mesh_dir=None):
        super().__init__(parent)
        self._urdf_path = Path(urdf_path) if urdf_path else URDF_PATH
        self._mesh_dir = Path(mesh_dir) if mesh_dir else MESH_DIR
        self._model: Optional[UrdfModel] = None
        self._plotter: Optional[object] = None
        # Per-link list of (vtk_actor, original_mesh, visual_transform)
        self._link_actors: Dict[str, List[Tuple]] = {}
        self._link_meshes: Dict[str, List[Tuple]] = {}
        self._current_angles = {name: 0.0 for name in JOINT_NAMES_ORDERED}
        self._initialized = False
        self._last_update_time = 0.0
        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        if not HAS_PYVISTA:
            label = QLabel("3D 可视化需要安装 pyvista 和 pyvistaqt\npip install pyvista pyvistaqt")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet("color: #f38ba8; font-size: 14px; padding: 40px;")
            layout.addWidget(label)
            return

        btn_layout = QHBoxLayout()
        reset_btn = QPushButton("重置视角")
        reset_btn.setFixedWidth(80)
        reset_btn.clicked.connect(self._reset_view)
        btn_layout.addWidget(reset_btn)

        wireframe_btn = QPushButton("线框")
        wireframe_btn.setFixedWidth(60)
        wireframe_btn.setCheckable(True)
        wireframe_btn.clicked.connect(self._toggle_wireframe)
        btn_layout.addWidget(wireframe_btn)

        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        try:
            pv.global_theme.allow_empty_mesh = True
            self._plotter = QtInteractor(self)
            self._plotter.set_background("#1e1e2e")
            layout.addWidget(self._plotter.interactor)
        except Exception as e:
            logger.error(f"创建 3D 渲染器失败: {e}")
            self._plotter = None
            label = QLabel(f"3D 渲染器初始化失败:\n{e}\n\n请确保有图形界面环境")
            label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            label.setStyleSheet("color: #f38ba8; font-size: 13px; padding: 20px;")
            label.setWordWrap(True)
            layout.addWidget(label)
            return

    def initialize_model(self):
        """加载 URDF 和 mesh，添加到场景（只调用一次）"""
        if not HAS_PYVISTA or self._plotter is None:
            return False
        if self._initialized:
            return True

        if not self._urdf_path.exists():
            logger.warning(f"URDF 文件不存在: {self._urdf_path}")
            return False
        if not self._mesh_dir.exists():
            logger.warning(f"Mesh 目录不存在: {self._mesh_dir}")
            return False

        try:
            self._model = UrdfModel(str(self._urdf_path), str(self._mesh_dir))
            self._link_meshes = self._model.load_meshes()
            logger.info(f"加载 {len(self._link_meshes)} 个 link mesh")

            transforms = self._model.compute_link_transforms(self._current_angles)

            for link_name, mesh_list in self._link_meshes.items():
                link_T = transforms.get(link_name, np.eye(4))
                actor_entries = []
                for mesh, vis_T, color in mesh_list:
                    world_T = link_T @ vis_T
                    display_mesh = mesh.copy()
                    display_mesh.compute_normals(
                        cell_normals=False, point_normals=True,
                        split_vertices=True, feature_angle=60.0,
                        inplace=True,
                    )
                    display_mesh.transform(world_T, inplace=True)
                    actor = self._plotter.add_mesh(
                        display_mesh,
                        color="white",
                        opacity=1.0,
                        smooth_shading=True,
                        show_edges=False,
                        specular=0.2,
                        name=f"{link_name}_{id(mesh)}",
                    )
                    orig_smooth = mesh.copy()
                    orig_smooth.compute_normals(
                        cell_normals=False, point_normals=True,
                        split_vertices=True, feature_angle=60.0,
                        inplace=True,
                    )
                    actor_entries.append((actor, orig_smooth, vis_T, display_mesh))
                self._link_actors[link_name] = actor_entries

            self._plotter.add_axes()
            grid = pv.Plane(
                center=(0, 0, -0.01), direction=(0, 0, 1),
                i_size=1.0, j_size=1.0, i_resolution=10, j_resolution=10,
            )
            self._plotter.add_mesh(grid, color="#313244", opacity=0.3, name="ground")
            self._reset_view()
            self._initialized = True
            return True

        except Exception as e:
            logger.error(f"模型初始化失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def update_joint_angles(self, joint_states):
        """根据关节反馈更新 3D 模型姿态（节流到 10Hz，使用原地点更新）"""
        if not self._initialized or self._model is None:
            return

        now = time.monotonic()
        if now - self._last_update_time < UPDATE_INTERVAL_S:
            return
        self._last_update_time = now

        positions = joint_states.to_list(include_gripper=True)
        changed = False
        for i, name in enumerate(JOINT_NAMES_ORDERED):
            if i < len(positions):
                if abs(self._current_angles[name] - positions[i]) > 1e-5:
                    self._current_angles[name] = positions[i]
                    changed = True

        if not changed:
            return

        transforms = self._model.compute_link_transforms(self._current_angles)

        for link_name, actor_entries in self._link_actors.items():
            link_T = transforms.get(link_name, np.eye(4))
            for actor, orig_mesh, vis_T, display_mesh in actor_entries:
                world_T = link_T @ vis_T
                new_points = orig_mesh.points.copy()
                R = world_T[:3, :3]
                t = world_T[:3, 3]
                new_points = (R @ new_points.T).T + t
                display_mesh.points = new_points

        try:
            self._plotter.render()
        except Exception:
            pass

    def _reset_view(self):
        if self._plotter:
            self._plotter.camera_position = [
                (0.6, -0.6, 0.5),
                (0.0, 0.0, 0.15),
                (0.0, 0.0, 1.0),
            ]

    def _toggle_wireframe(self, checked):
        if not self._plotter:
            return
        for actor_entries in self._link_actors.values():
            for actor, *_ in actor_entries:
                try:
                    if hasattr(actor, 'GetProperty'):
                        if checked:
                            actor.GetProperty().SetRepresentationToWireframe()
                        else:
                            actor.GetProperty().SetRepresentationToSurface()
                except Exception:
                    pass
        try:
            self._plotter.render()
        except Exception:
            pass

    def closeEvent(self, event):
        if self._plotter:
            try:
                self._plotter.close()
            except Exception:
                pass
        super().closeEvent(event)
