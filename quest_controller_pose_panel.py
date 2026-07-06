from __future__ import annotations

import os
import time
from typing import Any

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel, QVBoxLayout, QWidget

try:
    import pyqtgraph.opengl as gl
except Exception:  # pragma: no cover - depends on local OpenGL support
    gl = None

from run_controller_pose import find_meta_runtime
from steamvr_controller_pose import (
    controller_indices,
    get_universe,
    pose_to_record,
)


RED = (1.0, 0.08, 0.05, 1.0)
GREEN = (0.0, 0.75, 0.15, 1.0)
BLUE = (0.05, 0.25, 1.0, 1.0)
LEFT_COLOR = (0.0, 0.75, 1.0, 1.0)
RIGHT_COLOR = (1.0, 0.55, 0.0, 1.0)
UNKNOWN_COLOR = (0.85, 0.85, 0.85, 1.0)


def map_vr_to_gl(vector: tuple[float, float, float] | np.ndarray) -> np.ndarray:
    """Map OpenVR coordinates (x right, y up, -z forward) to GL view coordinates."""
    x, y, z = float(vector[0]), float(vector[1]), float(vector[2])
    return np.array([x, -z, y], dtype=float)


def quaternion_to_axes(
    qx: float, qy: float, qz: float, qw: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    r00 = 1.0 - 2.0 * (yy + zz)
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)
    r10 = 2.0 * (xy + wz)
    r11 = 1.0 - 2.0 * (xx + zz)
    r12 = 2.0 * (yz - wx)
    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)

    local_x = np.array([r00, r10, r20], dtype=float)
    local_y = np.array([r01, r11, r21], dtype=float)
    local_z = np.array([r02, r12, r22], dtype=float)
    return local_x, local_y, local_z


class QuestControllerPoseThread(QThread):
    records_updated = pyqtSignal(object)
    status_message = pyqtSignal(str)

    def __init__(self, rate_hz: float = 30.0):
        super().__init__()
        self.rate_hz = rate_hz
        self._run_flag = True
        self._openvr = None

    def stop(self) -> None:
        self._run_flag = False
        self.wait(1500)

    def _prepare_meta_runtime_path(self) -> None:
        runtime = find_meta_runtime(None)
        if runtime is None:
            return
        runtime_text = str(runtime)
        current_path = os.environ.get("PATH", "")
        if runtime_text.lower() not in current_path.lower():
            os.environ["PATH"] = f"{runtime_text};{current_path}"

    def _shutdown_openvr(self) -> None:
        if self._openvr is None:
            return
        try:
            self._openvr.shutdown()
        except Exception:
            pass
        self._openvr = None

    def run(self) -> None:
        self._prepare_meta_runtime_path()
        interval = max(1, int(1000.0 / self.rate_hz))

        while self._run_flag:
            try:
                import openvr

                self._openvr = openvr
                openvr.init(openvr.VRApplication_Other)
                vr_system = openvr.VRSystem()
                universe = get_universe(openvr, "standing")
                started_at = time.time()
                self.status_message.emit("OpenVR connected. Waiting for controllers...")

                while self._run_flag:
                    indices = controller_indices(openvr, vr_system, include_unassigned=True)
                    poses = vr_system.getDeviceToAbsoluteTrackingPose(
                        universe, 0, openvr.k_unMaxTrackedDeviceCount
                    )
                    records = [
                        pose_to_record(
                            openvr,
                            vr_system,
                            role,
                            device_index,
                            poses[device_index],
                            started_at,
                        )
                        for role, device_index in sorted(indices.items())
                    ]
                    self.records_updated.emit(records)
                    if records:
                        self.status_message.emit(f"Tracking {len(records)} controller(s).")
                    else:
                        self.status_message.emit("No controller yet. Wake controllers in SteamVR.")
                    self.msleep(interval)

            except Exception as exc:
                self.status_message.emit(
                    "OpenVR not ready. Start Quest Link and SteamVR. "
                    f"Last error: {exc}"
                )
                self._shutdown_openvr()
                self.msleep(2000)

        self._shutdown_openvr()


class QuestControllerPosePanel(QWidget):
    def __init__(self):
        super().__init__()
        self._gl_available = gl is not None
        self._position_items: dict[str, dict[str, Any]] = {}
        self._orientation_items: dict[str, dict[str, Any]] = {}

        self.pose_thread = QuestControllerPoseThread(rate_hz=30.0)
        self.init_ui()
        self.pose_thread.records_updated.connect(self.update_records)
        self.pose_thread.status_message.connect(self.update_status)
        self.pose_thread.start()

    def init_ui(self) -> None:
        layout = QVBoxLayout()
        title = QLabel("Part 5: Quest Controller Pose")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        layout.addWidget(title)

        self.status_label = QLabel("Status: starting OpenVR...")
        self.status_label.setStyleSheet("color: blue; font-style: italic;")
        layout.addWidget(self.status_label)

        grid = QGridLayout()
        self.position_group = QGroupBox("Position in SteamVR standing space")
        self.orientation_group = QGroupBox("Orientation axes")
        self.position_group.setLayout(QVBoxLayout())
        self.orientation_group.setLayout(QVBoxLayout())

        if self._gl_available:
            self.position_view = self._create_view(distance=3.2)
            self.orientation_view = self._create_view(distance=2.3)
            self.position_group.layout().addWidget(self.position_view)
            self.orientation_group.layout().addWidget(self.orientation_view)
            self._add_world_axes(self.position_view, length=0.75)
            self._add_world_axes(self.orientation_view, length=0.45)
        else:
            self.position_group.layout().addWidget(QLabel("3D view unavailable: install PyOpenGL."))
            self.orientation_group.layout().addWidget(QLabel("3D view unavailable: install PyOpenGL."))

        self.position_readout = QLabel("Position: waiting for controller data...")
        self.orientation_readout = QLabel("Orientation: waiting for controller data...")
        for label in (self.position_readout, self.orientation_readout):
            label.setFont(QFont("Consolas", 9))
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        self.position_group.layout().addWidget(self.position_readout)
        self.orientation_group.layout().addWidget(self.orientation_readout)
        grid.addWidget(self.position_group, 0, 0)
        grid.addWidget(self.orientation_group, 0, 1)
        layout.addLayout(grid)

        legend = QLabel("Axes: red=+X, green=+Y up, blue=+Z back. In SteamVR, forward is usually -Z.")
        legend.setFont(QFont("Arial", 9))
        layout.addWidget(legend)
        self.setLayout(layout)

    def _create_view(self, distance: float):
        view = gl.GLViewWidget()
        view.setMinimumHeight(260)
        view.setCameraPosition(distance=distance, elevation=22, azimuth=42)
        grid = gl.GLGridItem()
        grid.setSize(4.0, 4.0)
        grid.setSpacing(0.5, 0.5)
        view.addItem(grid)
        return view

    def _add_line(self, view: Any, color: tuple[float, float, float, float], width: int = 2):
        item = gl.GLLinePlotItem(
            pos=np.zeros((2, 3), dtype=float),
            color=color,
            width=width,
            antialias=True,
        )
        view.addItem(item)
        return item

    def _set_line(self, item: Any, start: np.ndarray, end: np.ndarray) -> None:
        item.setData(pos=np.vstack([start, end]))

    def _add_world_axes(self, view: Any, length: float) -> None:
        origin = np.zeros(3, dtype=float)
        for vector, color in (
            ((length, 0.0, 0.0), RED),
            ((0.0, length, 0.0), GREEN),
            ((0.0, 0.0, length), BLUE),
        ):
            line = self._add_line(view, color, width=3)
            self._set_line(line, origin, map_vr_to_gl(vector))

    def _role_color(self, role: str) -> tuple[float, float, float, float]:
        if role == "left":
            return LEFT_COLOR
        if role == "right":
            return RIGHT_COLOR
        return UNKNOWN_COLOR

    def _orientation_origin(self, role: str) -> np.ndarray:
        if role == "left":
            return np.array([-0.55, 0.0, 0.0], dtype=float)
        if role == "right":
            return np.array([0.55, 0.0, 0.0], dtype=float)
        return np.zeros(3, dtype=float)

    def _ensure_position_items(self, role: str) -> dict[str, Any]:
        if role in self._position_items:
            return self._position_items[role]
        color = self._role_color(role)
        marker = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=color, size=13, pxMode=True)
        radial = self._add_line(self.position_view, color, width=2)
        self.position_view.addItem(marker)
        self._position_items[role] = {"marker": marker, "radial": radial}
        return self._position_items[role]

    def _ensure_orientation_items(self, role: str) -> dict[str, Any]:
        if role in self._orientation_items:
            return self._orientation_items[role]
        marker = gl.GLScatterPlotItem(
            pos=np.zeros((1, 3)), color=self._role_color(role), size=10, pxMode=True
        )
        self.orientation_view.addItem(marker)
        self._orientation_items[role] = {
            "marker": marker,
            "x": self._add_line(self.orientation_view, RED, width=4),
            "y": self._add_line(self.orientation_view, GREEN, width=4),
            "z": self._add_line(self.orientation_view, BLUE, width=4),
        }
        return self._orientation_items[role]

    @pyqtSlot(str)
    def update_status(self, message: str) -> None:
        self.status_label.setText(f"Status: {message}")

    @pyqtSlot(object)
    def update_records(self, records: list[Any]) -> None:
        valid_records = [
            record
            for record in records
            if record.pose_valid and record.x is not None and record.qx is not None
        ]

        if self._gl_available:
            self._update_position_view(valid_records)
            self._update_orientation_view(valid_records)

        self._update_readouts(valid_records)

    def _update_position_view(self, records: list[Any]) -> None:
        origin = np.zeros(3, dtype=float)
        for record in records:
            role = record.role
            items = self._ensure_position_items(role)
            point = map_vr_to_gl((record.x, record.y, record.z))
            items["marker"].setData(pos=np.array([point]), color=self._role_color(role), size=13)
            self._set_line(items["radial"], origin, point)

    def _update_orientation_view(self, records: list[Any]) -> None:
        length = 0.35
        for record in records:
            role = record.role
            items = self._ensure_orientation_items(role)
            origin = self._orientation_origin(role)
            items["marker"].setData(pos=np.array([origin]), color=self._role_color(role), size=10)

            axes = quaternion_to_axes(record.qx, record.qy, record.qz, record.qw)
            for axis_name, axis_vector in zip(("x", "y", "z"), axes):
                end = origin + length * map_vr_to_gl(axis_vector)
                self._set_line(items[axis_name], origin, end)

    def _update_readouts(self, records: list[Any]) -> None:
        if not records:
            self.position_readout.setText("Position: no valid controller pose.")
            self.orientation_readout.setText("Orientation: no valid controller pose.")
            return

        position_lines = []
        orientation_lines = []
        for record in records:
            position_lines.append(
                f"{record.role:<10} x={record.x: .3f} m  y={record.y: .3f} m  z={record.z: .3f} m"
            )
            orientation_lines.append(
                f"{record.role:<10} roll={record.roll_deg: .1f}  "
                f"pitch={record.pitch_deg: .1f}  yaw={record.yaw_deg: .1f}"
            )

        self.position_readout.setText("\n".join(position_lines))
        self.orientation_readout.setText("\n".join(orientation_lines))

    def closeEvent(self, event) -> None:
        self.pose_thread.stop()
        event.accept()
