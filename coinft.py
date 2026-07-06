
import sys
import os
import math
import serial
import time
import cv2
import csv
import numpy as np
import scipy.io
import onnxruntime as ort
from datetime import datetime
from dataclasses import dataclass
from typing import Any
import pyqtgraph as pg

try:
    import pyqtgraph.opengl as gl
except Exception:
    gl = None

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QSlider, QLabel, QColorDialog, QFrame, QGridLayout, QGroupBox)
from PyQt5.QtGui import QColor, QImage, QPixmap, QFont
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot

# ==========================================
# 1. HARDWARE CONFIGURATION
# ==========================================
# LED Arduino Config
LED_SERIAL_PORT = '/dev/cu.usbmodem11101'  
LED_BAUD_RATE = 115200

# FT Sensor Config (UPDATE THESE PATHS)
FT_SERIAL_PORT = '/dev/cu.usbmodem1402' 
FT_BAUD_RATE = 1000000
FT_DATA_DIR = r'/Users/superteo/Desktop/pvft'  
FT_MODEL_FILE = 'PFT5-1_MLP_5L_norm_L2.onnx'
FT_NORM_FILE = 'PFT5-1_norm_constants.mat'


# ==========================================
# QUEST CONTROLLER POSE INTEGRATION
# ==========================================
INVALID_DEVICE_INDEX = 0xFFFFFFFF
QUEST_RED = (1.0, 0.08, 0.05, 1.0)
QUEST_GREEN = (0.0, 0.75, 0.15, 1.0)
QUEST_BLUE = (0.05, 0.25, 1.0, 1.0)
QUEST_LEFT_COLOR = (0.0, 0.75, 1.0, 1.0)
QUEST_RIGHT_COLOR = (1.0, 0.55, 0.0, 1.0)
QUEST_UNKNOWN_COLOR = (0.85, 0.85, 0.85, 1.0)


@dataclass
class QuestControllerPoseRecord:
    timestamp: float
    elapsed: float
    role: str
    device_index: int
    connected: bool
    pose_valid: bool
    tracking_result: str
    x: float | None = None
    y: float | None = None
    z: float | None = None
    qx: float | None = None
    qy: float | None = None
    qz: float | None = None
    qw: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None


def quest_registry_value(root_name: str, subkey: str, value_name: str, wow64_flag: int = 0) -> str | None:
    if os.name != "nt":
        return None
    try:
        import winreg
    except ImportError:
        return None

    root = getattr(winreg, root_name)
    access = winreg.KEY_READ | wow64_flag
    try:
        with winreg.OpenKey(root, subkey, 0, access) as key:
            value, _ = winreg.QueryValueEx(key, value_name)
    except OSError:
        return None
    return str(value)


def quest_registry_values(subkey: str, value_name: str) -> list[str]:
    if os.name != "nt":
        return []
    import winreg

    values = []
    for root_name in ("HKEY_LOCAL_MACHINE", "HKEY_CURRENT_USER"):
        for flag in (winreg.KEY_WOW64_64KEY, winreg.KEY_WOW64_32KEY, 0):
            value = quest_registry_value(root_name, subkey, value_name, flag)
            if value and value not in values:
                values.append(value)
    return values


def quest_meta_runtime_from_root(root: str) -> str | None:
    candidates = [
        root,
        os.path.join(root, "Support", "oculus-runtime"),
    ]
    for candidate in candidates:
        if os.path.exists(os.path.join(candidate, "LibOVRRT64_1.dll")):
            return candidate
    return None


def quest_find_meta_runtime() -> str | None:
    roots = []
    for env_name in ("META_QUEST_LINK_ROOT", "META_HORIZON_ROOT", "OCULUS_BASE"):
        value = os.environ.get(env_name)
        if value:
            roots.append(value)

    roots.extend(quest_registry_values(r"SOFTWARE\Oculus VR, LLC\Oculus", "Base"))

    program_files = [
        os.environ.get("ProgramFiles", r"C:\Program Files"),
        os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"),
    ]
    for base in program_files:
        roots.extend([
            os.path.join(base, "Oculus"),
            os.path.join(base, "Meta Horizon"),
            os.path.join(base, "Meta Quest Link"),
        ])

    for drive in "CDEFG":
        roots.extend([
            f"{drive}:\\Meta Horizon",
            f"{drive}:\\Oculus",
            f"{drive}:\\Meta Quest Link",
        ])

    seen = set()
    for root in roots:
        key = root.lower()
        if key in seen:
            continue
        seen.add(key)
        runtime = quest_meta_runtime_from_root(root)
        if runtime is not None:
            return runtime
    return None


def quest_prepare_meta_runtime_path():
    runtime = quest_find_meta_runtime()
    if runtime is None:
        return
    current_path = os.environ.get("PATH", "")
    if runtime.lower() not in current_path.lower():
        os.environ["PATH"] = f"{runtime};{current_path}"


def quest_get_universe(openvr_module: Any) -> int:
    return openvr_module.TrackingUniverseStanding


def quest_get_tracking_result_name(openvr_module: Any, value: int) -> str:
    known = {
        getattr(openvr_module, "TrackingResult_Uninitialized", None): "Uninitialized",
        getattr(openvr_module, "TrackingResult_Calibrating_InProgress", None): "Calibrating",
        getattr(openvr_module, "TrackingResult_Calibrating_OutOfRange", None): "CalibratingOutOfRange",
        getattr(openvr_module, "TrackingResult_Running_OK", None): "RunningOK",
        getattr(openvr_module, "TrackingResult_Running_OutOfRange", None): "RunningOutOfRange",
    }
    return known.get(value, str(value))


def quest_get_string_property(openvr_module: Any, vr_system: Any, device_index: int, prop_name: str) -> str:
    prop = getattr(openvr_module, prop_name, None)
    if prop is None:
        return ""
    try:
        return vr_system.getStringTrackedDeviceProperty(device_index, prop)
    except Exception:
        return ""


def quest_role_name(openvr_module: Any, role_value: int) -> str:
    if role_value == getattr(openvr_module, "TrackedControllerRole_LeftHand", -1):
        return "left"
    if role_value == getattr(openvr_module, "TrackedControllerRole_RightHand", -1):
        return "right"
    return "unassigned"


def quest_controller_role(openvr_module: Any, vr_system: Any, device_index: int) -> str:
    try:
        role = quest_role_name(openvr_module, vr_system.getControllerRoleForTrackedDeviceIndex(device_index))
    except Exception:
        role = "unassigned"

    if role != "unassigned":
        return role

    model = quest_get_string_property(openvr_module, vr_system, device_index, "Prop_ModelNumber_String")
    serial = quest_get_string_property(openvr_module, vr_system, device_index, "Prop_SerialNumber_String")
    label = f"{model} {serial}".lower()
    if "right" in label:
        return "right"
    if "left" in label:
        return "left"
    return "unassigned"


def quest_controller_indices(openvr_module: Any, vr_system: Any) -> dict[str, int]:
    indices = {}
    for role, label in (
        (openvr_module.TrackedControllerRole_LeftHand, "left"),
        (openvr_module.TrackedControllerRole_RightHand, "right"),
    ):
        try:
            index = vr_system.getTrackedDeviceIndexForControllerRole(role)
        except Exception:
            index = INVALID_DEVICE_INDEX
        if index != INVALID_DEVICE_INDEX:
            indices[label] = int(index)

    max_devices = int(getattr(openvr_module, "k_unMaxTrackedDeviceCount", 64))
    controller_class = getattr(openvr_module, "TrackedDeviceClass_Controller", None)

    for index in range(max_devices):
        try:
            device_class = vr_system.getTrackedDeviceClass(index)
        except Exception:
            continue
        if device_class != controller_class:
            continue

        role = quest_controller_role(openvr_module, vr_system, index)
        if role in {"left", "right"}:
            indices.setdefault(role, index)
        else:
            indices.setdefault(f"unassigned-{index}", index)
    return indices


def quest_matrix34_rows(matrix: Any) -> list[list[float]]:
    raw = getattr(matrix, "m", matrix)
    try:
        if len(raw) == 3:
            return [[float(raw[row][col]) for col in range(4)] for row in range(3)]
    except (IndexError, TypeError, ValueError):
        pass

    flat = [float(item) for item in raw]
    if len(flat) != 12:
        raise ValueError(f"Expected 12 values from OpenVR pose matrix, got {len(flat)}.")
    return [flat[0:4], flat[4:8], flat[8:12]]


def quest_quaternion_from_rotation_matrix(rows: list[list[float]]) -> tuple[float, float, float, float]:
    r00, r01, r02 = rows[0][0], rows[0][1], rows[0][2]
    r10, r11, r12 = rows[1][0], rows[1][1], rows[1][2]
    r20, r21, r22 = rows[2][0], rows[2][1], rows[2][2]
    trace = r00 + r11 + r22

    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r21 - r12) / scale
        qy = (r02 - r20) / scale
        qz = (r10 - r01) / scale
    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / scale
        qx = 0.25 * scale
        qy = (r01 + r10) / scale
        qz = (r02 + r20) / scale
    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / scale
        qx = (r01 + r10) / scale
        qy = 0.25 * scale
        qz = (r12 + r21) / scale
    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / scale
        qx = (r02 + r20) / scale
        qy = (r12 + r21) / scale
        qz = 0.25 * scale

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / norm, qy / norm, qz / norm, qw / norm


def quest_euler_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    sin_roll_cos_pitch = 2.0 * (qw * qx + qy * qz)
    cos_roll_cos_pitch = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)

    sin_pitch = 2.0 * (qw * qy - qz * qx)
    if abs(sin_pitch) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
    else:
        pitch = math.asin(sin_pitch)

    sin_yaw_cos_pitch = 2.0 * (qw * qz + qx * qy)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    to_degrees = 180.0 / math.pi
    return roll * to_degrees, pitch * to_degrees, yaw * to_degrees


def quest_pose_to_record(
    openvr_module: Any,
    vr_system: Any,
    role: str,
    device_index: int,
    pose: Any,
    started_at: float,
) -> QuestControllerPoseRecord:
    timestamp = time.time()
    record = QuestControllerPoseRecord(
        timestamp=timestamp,
        elapsed=timestamp - started_at,
        role=role,
        device_index=device_index,
        connected=bool(getattr(pose, "bDeviceIsConnected", False)),
        pose_valid=bool(getattr(pose, "bPoseIsValid", False)),
        tracking_result=quest_get_tracking_result_name(openvr_module, getattr(pose, "eTrackingResult", -1)),
    )

    if record.pose_valid:
        rows = quest_matrix34_rows(pose.mDeviceToAbsoluteTracking)
        record.x = rows[0][3]
        record.y = rows[1][3]
        record.z = rows[2][3]
        record.qx, record.qy, record.qz, record.qw = quest_quaternion_from_rotation_matrix(rows)
        record.roll_deg, record.pitch_deg, record.yaw_deg = quest_euler_from_quaternion(
            record.qx, record.qy, record.qz, record.qw
        )
    return record


def quest_map_vr_to_gl(vector: tuple[float, float, float] | np.ndarray) -> np.ndarray:
    x, y, z = float(vector[0]), float(vector[1]), float(vector[2])
    return np.array([x, -z, y], dtype=float)


def quest_quaternion_to_axes(
    qx: float, qy: float, qz: float, qw: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz

    return (
        np.array([1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)], dtype=float),
        np.array([2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)], dtype=float),
        np.array([2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)], dtype=float),
    )


class QuestControllerPoseThread(QThread):
    records_updated = pyqtSignal(object)
    status_message = pyqtSignal(str)

    def __init__(self, rate_hz: float = 30.0):
        super().__init__()
        self.rate_hz = rate_hz
        self._run_flag = True
        self._openvr = None

    def stop(self):
        self._run_flag = False
        self.wait(1500)

    def _shutdown_openvr(self):
        if self._openvr is None:
            return
        try:
            self._openvr.shutdown()
        except Exception:
            pass
        self._openvr = None

    def run(self):
        quest_prepare_meta_runtime_path()
        interval = max(1, int(1000.0 / self.rate_hz))

        while self._run_flag:
            try:
                import openvr

                self._openvr = openvr
                openvr.init(openvr.VRApplication_Other)
                vr_system = openvr.VRSystem()
                universe = quest_get_universe(openvr)
                started_at = time.time()
                self.status_message.emit("OpenVR connected. Waiting for controllers...")

                while self._run_flag:
                    indices = quest_controller_indices(openvr, vr_system)
                    poses = vr_system.getDeviceToAbsoluteTrackingPose(
                        universe, 0, openvr.k_unMaxTrackedDeviceCount
                    )
                    records = [
                        quest_pose_to_record(openvr, vr_system, role, device_index, poses[device_index], started_at)
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
        self._position_items = {}
        self._orientation_items = {}

        self.pose_thread = QuestControllerPoseThread(rate_hz=30.0)
        self.init_ui()
        self.pose_thread.records_updated.connect(self.update_records)
        self.pose_thread.status_message.connect(self.update_status)
        self.pose_thread.start()

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        title = QLabel("Part 5: Quest Controller Pose")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        layout.addWidget(title)

        self.status_label = QLabel("Status: starting OpenVR...")
        self.status_label.setStyleSheet("color: blue; font-style: italic;")
        layout.addWidget(self.status_label)

        grid = QGridLayout()
        grid.setSpacing(8)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        self.position_group = QGroupBox("Position in SteamVR standing space")
        self.orientation_group = QGroupBox("Orientation axes")
        self.position_group.setLayout(QVBoxLayout())
        self.orientation_group.setLayout(QVBoxLayout())

        if self._gl_available:
            self.position_view = self._create_view(distance=3.0)
            self.orientation_view = self._create_view(distance=2.0)
            self.position_group.layout().addWidget(self.position_view, 1)
            self.orientation_group.layout().addWidget(self.orientation_view, 1)
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
        layout.addLayout(grid, 1)

        legend = QLabel("Axes: red=+X, green=+Y up, blue=+Z back. In SteamVR, forward is usually -Z.")
        legend.setFont(QFont("Arial", 9))
        layout.addWidget(legend)
        self.setMinimumHeight(520)
        self.setLayout(layout)

    def _create_view(self, distance: float):
        view = gl.GLViewWidget()
        view.setMinimumHeight(390)
        view.setCameraPosition(distance=distance, elevation=22, azimuth=42)
        grid = gl.GLGridItem()
        grid.setSize(4.0, 4.0)
        grid.setSpacing(0.5, 0.5)
        view.addItem(grid)
        return view

    def _add_line(self, view: Any, color: tuple[float, float, float, float], width: int = 2):
        item = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=float), color=color, width=width, antialias=True)
        view.addItem(item)
        return item

    def _set_line(self, item: Any, start: np.ndarray, end: np.ndarray):
        item.setData(pos=np.vstack([start, end]))

    def _add_world_axes(self, view: Any, length: float):
        origin = np.zeros(3, dtype=float)
        for vector, color in (
            ((length, 0.0, 0.0), QUEST_RED),
            ((0.0, length, 0.0), QUEST_GREEN),
            ((0.0, 0.0, length), QUEST_BLUE),
        ):
            line = self._add_line(view, color, width=3)
            self._set_line(line, origin, quest_map_vr_to_gl(vector))

    def _role_color(self, role: str):
        if role == "left":
            return QUEST_LEFT_COLOR
        if role == "right":
            return QUEST_RIGHT_COLOR
        return QUEST_UNKNOWN_COLOR

    def _orientation_origin(self, role: str) -> np.ndarray:
        if role == "left":
            return np.array([-0.55, 0.0, 0.0], dtype=float)
        if role == "right":
            return np.array([0.55, 0.0, 0.0], dtype=float)
        return np.zeros(3, dtype=float)

    def _ensure_position_items(self, role: str):
        if role in self._position_items:
            return self._position_items[role]
        marker = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=self._role_color(role), size=13, pxMode=True)
        radial = self._add_line(self.position_view, self._role_color(role), width=2)
        self.position_view.addItem(marker)
        self._position_items[role] = {"marker": marker, "radial": radial}
        return self._position_items[role]

    def _ensure_orientation_items(self, role: str):
        if role in self._orientation_items:
            return self._orientation_items[role]
        marker = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=self._role_color(role), size=10, pxMode=True)
        self.orientation_view.addItem(marker)
        self._orientation_items[role] = {
            "marker": marker,
            "x": self._add_line(self.orientation_view, QUEST_RED, width=4),
            "y": self._add_line(self.orientation_view, QUEST_GREEN, width=4),
            "z": self._add_line(self.orientation_view, QUEST_BLUE, width=4),
        }
        return self._orientation_items[role]

    @pyqtSlot(str)
    def update_status(self, message: str):
        self.status_label.setText(f"Status: {message}")

    @pyqtSlot(object)
    def update_records(self, records):
        valid_records = [
            record
            for record in records
            if record.pose_valid and record.x is not None and record.qx is not None
        ]

        if self._gl_available:
            self._update_position_view(valid_records)
            self._update_orientation_view(valid_records)
        self._update_readouts(valid_records)

    def _update_position_view(self, records):
        origin = np.zeros(3, dtype=float)
        for record in records:
            items = self._ensure_position_items(record.role)
            point = quest_map_vr_to_gl((record.x, record.y, record.z))
            items["marker"].setData(pos=np.array([point]), color=self._role_color(record.role), size=13)
            self._set_line(items["radial"], origin, point)

    def _update_orientation_view(self, records):
        length = 0.35
        for record in records:
            items = self._ensure_orientation_items(record.role)
            origin = self._orientation_origin(record.role)
            items["marker"].setData(pos=np.array([origin]), color=self._role_color(record.role), size=10)

            axes = quest_quaternion_to_axes(record.qx, record.qy, record.qz, record.qw)
            for axis_name, axis_vector in zip(("x", "y", "z"), axes):
                end = origin + length * quest_map_vr_to_gl(axis_vector)
                self._set_line(items[axis_name], origin, end)

    def _update_readouts(self, records):
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

    def closeEvent(self, event):
        self.pose_thread.stop()
        event.accept()


# ==========================================
# 2. THREAD: FORCE/TORQUE SENSOR
# ==========================================
class FTSensorThread(QThread):
    update_wrench = pyqtSignal(float, float, float, float, float, float)
    status_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.START_BYTE = 2
        self.END_BYTE = 3
        
        self.read_count = 0
        self.initialSampleNum = 1500
        self.offset_CoinFT_list = []
        self.got_initial_offset = False
        self.offset_CoinFT = None
        
        self.ft_bias_ready = False
        self.ft_bias = np.zeros(6)
        self.request_tare = False

        self.single_capture_flag = False
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None

        try:
            onnx_path = os.path.join(FT_DATA_DIR, FT_MODEL_FILE)
            norm_path = os.path.join(FT_DATA_DIR, FT_NORM_FILE)
            self.ort_session = ort.InferenceSession(onnx_path)
            norm_data = scipy.io.loadmat(norm_path)
            self.mu_x = norm_data['norm_const']['mu_x'][0,0].flatten()
            self.sd_x = norm_data['norm_const']['sd_x'][0,0].flatten()
            self.mu_y = norm_data['norm_const']['mu_y'][0,0].flatten()
            self.sd_y = norm_data['norm_const']['sd_y'][0,0].flatten()
            self.model_loaded = True
        except Exception as e:
            self.model_loaded = False
            print(f"Failed to load FT Sensor Model: {e}")

    def run(self):
        if not self.model_loaded:
            self.status_message.emit("Model load failed. Check paths.")
            return

        try:
            self.ser = serial.Serial(FT_SERIAL_PORT, FT_BAUD_RATE, timeout=0.1)
            self.ser.write(b'i')
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.write(b'q')
            time.sleep(0.01)
            packet_size_excludeStartByte_raw = self.ser.read(1)
            if len(packet_size_excludeStartByte_raw) < 1:
                self.status_message.emit("Failed to read packet size from FT sensor.")
                return
            self.packet_size_excludeStartByte = ord(packet_size_excludeStartByte_raw) - 1
            self.ser.write(b's')
            self.status_message.emit("FT Sensor stream started. Collecting offset...")
        except Exception as e:
            self.status_message.emit(f"FT Serial Error: {e}")
            return

        while self._run_flag:
            byte = self.ser.read(1)
            if len(byte) == 0 or byte[0] != self.START_BYTE: continue
            data = self.ser.read(self.packet_size_excludeStartByte)
            if len(data) < self.packet_size_excludeStartByte: continue
            
            if data[-1] == self.END_BYTE:
                sensor_data = []
                for byte_num in range(0, self.packet_size_excludeStartByte-1, 2):
                    val = data[byte_num] + 256*data[byte_num+1]
                    sensor_data.append(val)
                sensor_data = np.array(sensor_data, dtype=np.float64)
                self.read_count += 1
                
                if self.read_count <= self.initialSampleNum:
                    self.offset_CoinFT_list.append(sensor_data)
                    if self.read_count % 300 == 0:
                        self.status_message.emit(f"Calibrating: {self.read_count}/{self.initialSampleNum}")
                    continue
                elif self.read_count == self.initialSampleNum + 1:
                    self.offset_CoinFT = np.mean(self.offset_CoinFT_list[5:], axis=0)
                    self.got_initial_offset = True
                    self.status_message.emit("Calibration complete. Streaming data.")
                    continue
                
                if not self.got_initial_offset: continue
                
                SensorData_offsetted = sensor_data - self.offset_CoinFT
                x_norm = (SensorData_offsetted - self.mu_x) / self.sd_x
                x_input = x_norm.astype(np.float32).reshape(1, -1)
                calibratedFT = self.ort_session.run(None, {"input": x_input})[0].flatten()
                calibratedFT = calibratedFT * self.sd_y + self.mu_y
                
                if not self.ft_bias_ready:
                    self.ft_bias = calibratedFT.copy()
                    self.ft_bias_ready = True
                if self.request_tare:
                    self.ft_bias = calibratedFT.copy()
                    self.request_tare = False
                    self.status_message.emit("Sensor Tared to Zero.")
                
                final_FT = calibratedFT - self.ft_bias
                
                if self.read_count % 10 == 0:
                    self.update_wrench.emit(float(final_FT[0]), float(final_FT[1]), float(final_FT[2]),
                                            float(final_FT[3]), float(final_FT[4]), float(final_FT[5]))

                if self.single_capture_flag:
                    os.makedirs("captures", exist_ok=True)
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                    with open(f"captures/ft_data_{timestamp}.txt", 'w') as f:
                        f.write(f"Fx: {final_FT[0]:.4f}\nFy: {final_FT[1]:.4f}\nFz: {final_FT[2]:.4f}\n")
                        f.write(f"Tx: {final_FT[3]:.4f}\nTy: {final_FT[4]:.4f}\nTz: {final_FT[5]:.4f}\n")
                    self.single_capture_flag = False

                if self.is_recording and self.csv_writer:
                    self.csv_writer.writerow([time.time()] + final_FT.tolist())

    def tare_sensor(self): self.request_tare = True
    def capture_single_frame(self): self.single_capture_flag = True
    def toggle_recording(self, state):
        self.is_recording = state
        if state:
            os.makedirs("recordings", exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_file = open(f"recordings/ft_data_{timestamp}.csv", 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
        else:
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None; self.csv_writer = None
    def stop(self):
        self._run_flag = False
        if self.csv_file: self.csv_file.close()
        if hasattr(self, 'ser') and self.ser.is_open: self.ser.close()
        self.wait()


# ==========================================
# 3. THREAD: VIDEO PROCESSING (ALL 5 FEATURES)
# ==========================================
class VideoThread(QThread):
    # ROW 1
    update_raw_frame = pyqtSignal(np.ndarray, float)
    update_crop_frame = pyqtSignal(np.ndarray)
    # ROW 2
    update_bgs_mask_raw_frame = pyqtSignal(np.ndarray)  
    update_bgs_mask_crop_frame = pyqtSignal(np.ndarray) 
    # ROW 3
    update_bgs_contour_raw_frame = pyqtSignal(np.ndarray)  
    update_bgs_contour_crop_frame = pyqtSignal(np.ndarray) 
    # ROW 4
    update_gel_contour_raw_frame = pyqtSignal(np.ndarray)  
    update_gel_contour_crop_frame = pyqtSignal(np.ndarray) 
    # ROW 5
    update_diff_frame = pyqtSignal(np.ndarray)
    update_crop_diff_frame = pyqtSignal(np.ndarray)
    
    status_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.take_ref_flag = False
        self.reference_frame = None
        self.reference_crop_frame = None
        
        self.back_sub_raw = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=25, detectShadows=True)
        self.back_sub_crop = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16, detectShadows=True)

        self.single_capture_flag = False
        self.is_recording = False
        self.video_writers = None

    def compute_tactile_diff(self, img_current, img_bg, offset=0.5):
        img1 = np.int32(img_current)
        img2 = np.int32(img_bg)
        diff = img1 - img2
        diff = diff / 255.0 + offset
        diff = np.clip(diff, 0.0, 1.0)
        return np.uint8(diff * 255.0)

    def run(self):
        cap = cv2.VideoCapture(0)
        prev_time = time.time()

        while self._run_flag:
            ret, frame = cap.read()
            if not ret: continue

            current_time = time.time()
            fps = 1.0 / (current_time - prev_time) if (current_time - prev_time) > 0 else 0
            prev_time = current_time

            # --- ROW 1: Raw & Crop ---
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.update_raw_frame.emit(rgb_frame, fps)

            h, w, _ = rgb_frame.shape
            crop_size = min(h, w)
            start_x = (w - crop_size) // 2
            start_y = (h - crop_size) // 2
            cropped = rgb_frame[start_y:start_y+crop_size, start_x:start_x+crop_size]
            resized_224 = cv2.resize(cropped, (224, 224), interpolation=cv2.INTER_AREA)
            self.update_crop_frame.emit(resized_224)

            # ---------------------------------------------------------
            # MOG2 Mask Calculation
            # ---------------------------------------------------------
            bgs_raw_mask = self.back_sub_raw.apply(rgb_frame)
            bgs_crop_mask = self.back_sub_crop.apply(resized_224)

            # --- ROW 2: Pure MOG2 Masks ---
            bgs_mask_raw_rgb = cv2.cvtColor(bgs_raw_mask, cv2.COLOR_GRAY2RGB)
            bgs_mask_crop_rgb = cv2.cvtColor(bgs_crop_mask, cv2.COLOR_GRAY2RGB)
            self.update_bgs_mask_raw_frame.emit(bgs_mask_raw_rgb)
            self.update_bgs_mask_crop_frame.emit(bgs_mask_crop_rgb)

            # ---------------------------------------------------------
            # Contour Extractions
            # ---------------------------------------------------------
            _, thresh_raw = cv2.threshold(bgs_raw_mask, 200, 255, cv2.THRESH_BINARY)
            contours_raw, _ = cv2.findContours(thresh_raw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            _, thresh_crop = cv2.threshold(bgs_crop_mask, 200, 255, cv2.THRESH_BINARY)
            contours_crop, _ = cv2.findContours(thresh_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # --- ROW 3: BGS + Green Contours (Color Background) ---
            bgs_contour_raw = rgb_frame.copy()
            cv2.drawContours(bgs_contour_raw, contours_raw, -1, (0, 255, 0), 2)
            
            bgs_contour_crop = resized_224.copy()
            cv2.drawContours(bgs_contour_crop, contours_crop, -1, (0, 255, 0), 2)
            
            self.update_bgs_contour_raw_frame.emit(bgs_contour_raw)
            self.update_bgs_contour_crop_frame.emit(bgs_contour_crop)

            # --- ROW 4: Gel Contour (Black & White) ---
            gel_contour_raw = np.zeros_like(rgb_frame)
            cv2.drawContours(gel_contour_raw, contours_raw, -1, (255, 255, 255), 2)
            
            gel_contour_crop = np.zeros_like(resized_224)
            cv2.drawContours(gel_contour_crop, contours_crop, -1, (255, 255, 255), 2)

            self.update_gel_contour_raw_frame.emit(gel_contour_raw)
            self.update_gel_contour_crop_frame.emit(gel_contour_crop)

            # --- ROW 5: Tactile Diff (Gray) ---
            if self.take_ref_flag:
                self.reference_frame = rgb_frame.copy()
                self.reference_crop_frame = resized_224.copy()
                self.take_ref_flag = False
                self.status_message.emit("Gray Reference frame captured.")

            diff_frame = None
            diff_crop_frame = None

            if self.reference_frame is not None:
                diff_frame = self.compute_tactile_diff(rgb_frame, self.reference_frame)
                self.update_diff_frame.emit(diff_frame)
                
            if self.reference_crop_frame is not None:
                diff_crop_frame = self.compute_tactile_diff(resized_224, self.reference_crop_frame)
                self.update_crop_diff_frame.emit(diff_crop_frame)

            # ---------------------------------------------------------
            # DATA COLLECTION
            # ---------------------------------------------------------
            if self.single_capture_flag:
                # Save all 10 visual states
                self._save_single_capture(
                    frame, resized_224, 
                    bgs_mask_raw_rgb, bgs_mask_crop_rgb,
                    bgs_contour_raw, bgs_contour_crop,
                    gel_contour_raw, gel_contour_crop,
                    diff_frame, diff_crop_frame
                )
                self.single_capture_flag = False

            if self.is_recording:
                if self.video_writers is None:
                    os.makedirs("recordings", exist_ok=True)
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    fourcc = cv2.VideoWriter_fourcc(*'XVID')
                    self.video_writers = {
                        'raw': cv2.VideoWriter(f"recordings/raw_{timestamp}.avi", fourcc, 20.0, (w, h)),
                        'crop': cv2.VideoWriter(f"recordings/crop_{timestamp}.avi", fourcc, 20.0, (224, 224))
                    }
                    self.status_message.emit(f"Recording started: {timestamp}")
                
                self.video_writers['raw'].write(frame)
                self.video_writers['crop'].write(cv2.cvtColor(resized_224, cv2.COLOR_RGB2BGR))
            else:
                if self.video_writers is not None:
                    self.video_writers['raw'].release(); self.video_writers['crop'].release(); self.video_writers = None
                    self.status_message.emit("Recording stopped and saved.")

        if self.video_writers is not None:
            self.video_writers['raw'].release(); self.video_writers['crop'].release()
        cap.release()

    def _save_single_capture(self, raw, crop, mask_raw, mask_crop, cont_raw, cont_crop, gel_raw, gel_crop, diff_raw, diff_crop):
        os.makedirs("captures", exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3] 
        
        cv2.imwrite(f"captures/1_raw_{timestamp}.jpg", raw)
        cv2.imwrite(f"captures/1_crop_{timestamp}.jpg", cv2.cvtColor(crop, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/2_mask_raw_{timestamp}.jpg", cv2.cvtColor(mask_raw, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/2_mask_crop_{timestamp}.jpg", cv2.cvtColor(mask_crop, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/3_bgs_contour_raw_{timestamp}.jpg", cv2.cvtColor(cont_raw, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/3_bgs_contour_crop_{timestamp}.jpg", cv2.cvtColor(cont_crop, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/4_gel_contour_raw_{timestamp}.jpg", cv2.cvtColor(gel_raw, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f"captures/4_gel_contour_crop_{timestamp}.jpg", cv2.cvtColor(gel_crop, cv2.COLOR_RGB2BGR))
        
        if diff_raw is not None: cv2.imwrite(f"captures/5_diff_gray_raw_{timestamp}.jpg", cv2.cvtColor(diff_raw, cv2.COLOR_RGB2BGR))
        if diff_crop is not None: cv2.imwrite(f"captures/5_diff_gray_crop_{timestamp}.jpg", cv2.cvtColor(diff_crop, cv2.COLOR_RGB2BGR))
            
        self.status_message.emit(f"Captured all 10 frames at {timestamp}")

    def capture_reference(self): self.take_ref_flag = True
    def capture_single_frame(self): self.single_capture_flag = True
    def toggle_recording(self, state): self.is_recording = state
    def stop(self): self._run_flag = False; self.wait()


# ==========================================
# 4. UI COMPONENTS (LEDs, FT Plots)
# ==========================================
# [LEDControlPanel and FTSensorPanel remain exactly the same functionally]
class LEDControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.num_leds = 6
        self.led_colors = [(0, 0, 0) for _ in range(self.num_leds)]
        self.serial_conn = None
        try:
            self.serial_conn = serial.Serial(LED_SERIAL_PORT, LED_BAUD_RATE, timeout=1)
            time.sleep(2)
        except: pass
        self.init_ui()
        self.send_serial_command(99, 20, 0, 0)
        self.set_default_rgbrgb()
    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("Part 1: LED Controller"); title.setFont(QFont("Arial", 14, QFont.Bold)); layout.addWidget(title)
        brightness_layout = QVBoxLayout()
        self.brightness_label = QLabel("Global Brightness: 20")
        self.brightness_slider = QSlider(Qt.Horizontal); self.brightness_slider.setRange(0, 255); self.brightness_slider.setValue(20)
        self.brightness_slider.valueChanged.connect(self.on_brightness_changed)
        brightness_layout.addWidget(self.brightness_label); brightness_layout.addWidget(self.brightness_slider); layout.addLayout(brightness_layout)
        led_layout = QHBoxLayout()
        self.led_buttons = []
        for i in range(self.num_leds):
            btn = QPushButton(f"LED {i}"); btn.setFixedSize(60, 60); btn.clicked.connect(lambda checked, idx=i: self.choose_color(idx)); self.led_buttons.append(btn); led_layout.addWidget(btn)
        layout.addLayout(led_layout)
        quick_layout = QHBoxLayout()
        btn_red = QPushButton("All Red"); btn_red.clicked.connect(lambda: self.set_all_color(255, 0, 0))
        btn_green = QPushButton("All Green"); btn_green.clicked.connect(lambda: self.set_all_color(0, 255, 0))
        btn_blue = QPushButton("All Blue"); btn_blue.clicked.connect(lambda: self.set_all_color(0, 0, 255))
        btn_def = QPushButton("Default"); btn_def.clicked.connect(self.set_default_rgbrgb)
        btn_off = QPushButton("Turn Off All"); btn_off.clicked.connect(lambda: self.set_all_color(0, 0, 0))
        for b in (btn_red, btn_green, btn_blue, btn_def, btn_off): quick_layout.addWidget(b)
        layout.addLayout(quick_layout); layout.addStretch(); self.setLayout(layout)
    def send_serial_command(self, index, r, g, b):
        if self.serial_conn and self.serial_conn.is_open: self.serial_conn.write(f"{index},{r},{g},{b}\n".encode('utf-8')); time.sleep(0.01)
    def update_button_color(self, idx, r, g, b):
        style = f"background-color: #{r:02x}{g:02x}{b:02x}; color: {'black' if (0.299*r+0.587*g+0.114*b)>128 else 'white'}; font-weight: bold; border-radius: 5px;"
        self.led_buttons[idx].setStyleSheet(style)
    def on_brightness_changed(self):
        val = self.brightness_slider.value(); self.brightness_label.setText(f"Global Brightness: {val}"); self.send_serial_command(99, val, 0, 0)
    def choose_color(self, idx):
        c = QColorDialog.getColor(QColor(*self.led_colors[idx]), self, f"LED {idx}")
        if c.isValid(): self.set_single_led(idx, c.red(), c.green(), c.blue())
    def set_single_led(self, idx, r, g, b): self.led_colors[idx] = (r, g, b); self.update_button_color(idx, r, g, b); self.send_serial_command(idx, r, g, b)
    def set_all_color(self, r, g, b):
        for i in range(self.num_leds): self.set_single_led(i, r, g, b)
    def set_default_rgbrgb(self):
        p = [(255,0,0), (255,0,0),(0,255,0), (0,255,0), (0,0,255),  (0,0,255)]
        for i in range(self.num_leds): self.set_single_led(i, *p[i])
    def closeEvent(self, e):
        if self.serial_conn and self.serial_conn.is_open: self.serial_conn.close()

class FTSensorPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.history_size = 150
        self.data_fx = np.zeros(self.history_size); self.data_fy = np.zeros(self.history_size); self.data_fz = np.zeros(self.history_size)
        self.data_tx = np.zeros(self.history_size); self.data_ty = np.zeros(self.history_size); self.data_tz = np.zeros(self.history_size)
        self.ft_thread = FTSensorThread(); self.init_ui(); self.ft_thread.update_wrench.connect(self.update_display); self.ft_thread.status_message.connect(self.update_status); self.ft_thread.start()
    def init_ui(self):
        layout = QVBoxLayout()
        header_layout = QHBoxLayout()
        title = QLabel("Part 4: CoinFT Sensor"); title.setFont(QFont("Arial", 14, QFont.Bold)); header_layout.addWidget(title)
        self.btn_tare = QPushButton("Tare (Zero) Sensor"); self.btn_tare.setStyleSheet("background-color: orange; font-weight: bold;"); self.btn_tare.clicked.connect(self.ft_thread.tare_sensor); header_layout.addWidget(self.btn_tare); layout.addLayout(header_layout)
        self.status_label = QLabel("Status: Waiting..."); self.status_label.setStyleSheet("color: blue; font-style: italic;"); layout.addWidget(self.status_label)
        readout_layout = QHBoxLayout()
        force_group = QGroupBox("Forces (N)"); force_layout = QHBoxLayout()
        self.lbl_fx = QLabel("Fx: 0.00"); self.lbl_fy = QLabel("Fy: 0.00"); self.lbl_fz = QLabel("Fz: 0.00")
        for lbl in (self.lbl_fx, self.lbl_fy, self.lbl_fz): lbl.setFont(QFont("Consolas", 12, QFont.Bold)); force_layout.addWidget(lbl)
        force_group.setLayout(force_layout)
        torque_group = QGroupBox("Torques (Nm)"); torque_layout = QHBoxLayout()
        self.lbl_tx = QLabel("Tx: 0.000"); self.lbl_ty = QLabel("Ty: 0.000"); self.lbl_tz = QLabel("Tz: 0.000")
        for lbl in (self.lbl_tx, self.lbl_ty, self.lbl_tz): lbl.setFont(QFont("Consolas", 12, QFont.Bold)); torque_layout.addWidget(lbl)
        torque_group.setLayout(torque_layout); readout_layout.addWidget(force_group); readout_layout.addWidget(torque_group); layout.addLayout(readout_layout)
        pg.setConfigOptions(antialias=True); self.plot_f = pg.PlotWidget(title="Force (N)"); self.plot_f.setFixedHeight(180); self.plot_f.addLegend(); self.plot_f.showGrid(x=False, y=True)
        self.curve_fx = self.plot_f.plot(pen=pg.mkPen('r', width=2), name="Fx"); self.curve_fy = self.plot_f.plot(pen=pg.mkPen('g', width=2), name="Fy"); self.curve_fz = self.plot_f.plot(pen=pg.mkPen('c', width=2), name="Fz")
        self.plot_t = pg.PlotWidget(title="Torque (Nm)"); self.plot_t.setFixedHeight(180); self.plot_t.addLegend(); self.plot_t.showGrid(x=False, y=True)
        self.curve_tx = self.plot_t.plot(pen=pg.mkPen('r', width=2), name="Tx"); self.curve_ty = self.plot_t.plot(pen=pg.mkPen('g', width=2), name="Ty"); self.curve_tz = self.plot_t.plot(pen=pg.mkPen('c', width=2), name="Tz")
        layout.addWidget(self.plot_f); layout.addWidget(self.plot_t); self.setLayout(layout)
    @pyqtSlot(str)
    def update_status(self, msg): self.status_label.setText(f"Status: {msg}")
    @pyqtSlot(float, float, float, float, float, float)
    def update_display(self, fx, fy, fz, tx, ty, tz):
        for lbl, val, fmt in ((self.lbl_fx, fx, "6.2f"), (self.lbl_fy, fy, "6.2f"), (self.lbl_fz, fz, "6.2f"), (self.lbl_tx, tx, "6.3f"), (self.lbl_ty, ty, "6.3f"), (self.lbl_tz, tz, "6.3f")): lbl.setText(f"{lbl.text().split(':')[0]}: {val:{fmt}}")
        for data, val in ((self.data_fx, fx), (self.data_fy, fy), (self.data_fz, fz), (self.data_tx, tx), (self.data_ty, ty), (self.data_tz, tz)): data[:-1] = data[1:]; data[-1] = val
        self.curve_fx.setData(self.data_fx); self.curve_fy.setData(self.data_fy); self.curve_fz.setData(self.data_fz); self.curve_tx.setData(self.data_tx); self.curve_ty.setData(self.data_ty); self.curve_tz.setData(self.data_tz)
    def closeEvent(self, event): self.ft_thread.stop()

# ==========================================
# 5. UI COMPONENTS: VISION & DATA COLLECT (5x2 GRID)
# ==========================================
class CameraMonitorPanel(QWidget):
    req_capture = pyqtSignal()
    req_record = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.video_thread = VideoThread()
        self.init_ui()
        
        # Row 1
        self.video_thread.update_raw_frame.connect(self.update_raw_image)
        self.video_thread.update_crop_frame.connect(self.update_crop_image)
        # Row 2
        self.video_thread.update_bgs_mask_raw_frame.connect(self.update_bgs_mask_raw_image)
        self.video_thread.update_bgs_mask_crop_frame.connect(self.update_bgs_mask_crop_image)
        # Row 3
        self.video_thread.update_bgs_contour_raw_frame.connect(self.update_bgs_contour_raw_image)
        self.video_thread.update_bgs_contour_crop_frame.connect(self.update_bgs_contour_crop_image)
        # Row 4
        self.video_thread.update_gel_contour_raw_frame.connect(self.update_gel_contour_raw_image)
        self.video_thread.update_gel_contour_crop_frame.connect(self.update_gel_contour_crop_image)
        # Row 5
        self.video_thread.update_diff_frame.connect(self.update_diff_image)
        self.video_thread.update_crop_diff_frame.connect(self.update_crop_diff_image)
        
        self.video_thread.status_message.connect(self.update_status)
        self.video_thread.start()

    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel("Part 2 & 3: Vision Analytics Matrix (5x2)"); title.setFont(QFont("Arial", 14, QFont.Bold)); layout.addWidget(title)

        grid = QGridLayout(); grid.setSpacing(5)

        # Removed FixedSize to allow window to shrink/scroll if needed, while maintaining ratios
        def create_video_label():
            lbl = QLabel()
            lbl.setMinimumSize(160, 120) 
            lbl.setStyleSheet("background-color: black; border: 1px solid gray;")
            lbl.setAlignment(Qt.AlignCenter)
            return lbl
            
        def create_title(text):
            lbl = QLabel(text); lbl.setFont(QFont("Arial", 9, QFont.Bold)); lbl.setAlignment(Qt.AlignCenter); return lbl

        # ROW 1
        self.label_raw_title = create_title("1. RAW Input (FPS: --)"); self.label_raw_video = create_video_label()
        self.label_crop_title = create_title("ML Crop (224x224)"); self.label_crop_video = create_video_label()
        grid.addWidget(self.label_raw_title, 0, 0); grid.addWidget(self.label_raw_video, 1, 0)
        grid.addWidget(self.label_crop_title, 0, 1); grid.addWidget(self.label_crop_video, 1, 1)

        # ROW 2
        self.label_bgs_mask_raw_title = create_title("2. BGS Mask (MOG2 RAW)"); self.label_bgs_mask_raw_video = create_video_label()
        self.label_bgs_mask_crop_title = create_title("BGS Mask (MOG2 Crop)"); self.label_bgs_mask_crop_video = create_video_label()
        grid.addWidget(self.label_bgs_mask_raw_title, 2, 0); grid.addWidget(self.label_bgs_mask_raw_video, 3, 0)
        grid.addWidget(self.label_bgs_mask_crop_title, 2, 1); grid.addWidget(self.label_bgs_mask_crop_video, 3, 1)

        # ROW 3
        self.label_bgs_cont_raw_title = create_title("3. BGS + Green Contours"); self.label_bgs_cont_raw_video = create_video_label()
        self.label_bgs_cont_crop_title = create_title("BGS + Green Contours"); self.label_bgs_cont_crop_video = create_video_label()
        grid.addWidget(self.label_bgs_cont_raw_title, 4, 0); grid.addWidget(self.label_bgs_cont_raw_video, 5, 0)
        grid.addWidget(self.label_bgs_cont_crop_title, 4, 1); grid.addWidget(self.label_bgs_cont_crop_video, 5, 1)

        # ROW 4
        self.label_gel_cont_raw_title = create_title("4. Gel Contour (B&W RAW)"); self.label_gel_cont_raw_video = create_video_label()
        self.label_gel_cont_crop_title = create_title("Gel Contour (B&W Crop)"); self.label_gel_cont_crop_video = create_video_label()
        grid.addWidget(self.label_gel_cont_raw_title, 6, 0); grid.addWidget(self.label_gel_cont_raw_video, 7, 0)
        grid.addWidget(self.label_gel_cont_crop_title, 6, 1); grid.addWidget(self.label_gel_cont_crop_video, 7, 1)

        # ROW 5
        self.label_diff_title = create_title("5. Tactile Diff (Gray Ref)"); self.label_diff_video = create_video_label()
        self.label_crop_diff_title = create_title("Tactile Diff (Gray Ref)"); self.label_crop_diff_video = create_video_label()
        grid.addWidget(self.label_diff_title, 8, 0); grid.addWidget(self.label_diff_video, 9, 0)
        grid.addWidget(self.label_crop_diff_title, 8, 1); grid.addWidget(self.label_crop_diff_video, 9, 1)

        layout.addLayout(grid); layout.addSpacing(10)

        # Controls
        controls_layout = QHBoxLayout()
        self.btn_take_ref = QPushButton("Take Static Ref"); self.btn_take_ref.setMinimumHeight(40); self.btn_take_ref.clicked.connect(self.video_thread.capture_reference)
        self.btn_capture = QPushButton("Capture Snap (10 Frames + FT)"); self.btn_capture.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;"); self.btn_capture.setMinimumHeight(40); self.btn_capture.clicked.connect(self.on_capture)
        self.btn_record = QPushButton("Start Recording"); self.btn_record.setMinimumHeight(40); self.btn_record.setCheckable(True); self.btn_record.setStyleSheet("QPushButton:checked { background-color: red; color: white; font-weight: bold; }"); self.btn_record.toggled.connect(self.on_record)
        controls_layout.addWidget(self.btn_take_ref); controls_layout.addWidget(self.btn_capture); controls_layout.addWidget(self.btn_record)
        layout.addLayout(controls_layout)

        self.status_label = QLabel("Status: Ready"); self.status_label.setStyleSheet("color: blue; font-style: italic;")
        layout.addWidget(self.status_label); layout.addStretch(); self.setLayout(layout)

    def on_capture(self): self.video_thread.capture_single_frame(); self.req_capture.emit() 
    def on_record(self, checked):
        self.btn_record.setText("Stop Recording" if checked else "Start Recording")
        self.video_thread.toggle_recording(checked); self.req_record.emit(checked) 

    @pyqtSlot(str)
    def update_status(self, msg): self.status_label.setText(f"Status: {msg}")

    def convert_cv_qt(self, cv_img, target_width=320, target_height=240):
        qh, qw, ch = cv_img.shape; bytes_per_line = ch * qw
        q_img = QImage(cv_img.data, qw, qh, bytes_per_line, QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(q_img)
        return pixmap.scaled(target_width, target_height, Qt.KeepAspectRatio)

    # --- SLOTS FOR ALL 10 STREAMS ---
    @pyqtSlot(np.ndarray, float)
    def update_raw_image(self, cv_img, fps):
        self.label_raw_title.setText(f"1. RAW Input (FPS: {fps:.1f})")
        self.label_raw_video.setPixmap(self.convert_cv_qt(cv_img))

    @pyqtSlot(np.ndarray)
    def update_crop_image(self, cv_img): self.label_crop_video.setPixmap(self.convert_cv_qt(cv_img, 224, 224))
    
    @pyqtSlot(np.ndarray)
    def update_bgs_mask_raw_image(self, cv_img): self.label_bgs_mask_raw_video.setPixmap(self.convert_cv_qt(cv_img))
    @pyqtSlot(np.ndarray)
    def update_bgs_mask_crop_image(self, cv_img): self.label_bgs_mask_crop_video.setPixmap(self.convert_cv_qt(cv_img, 224, 224))

    @pyqtSlot(np.ndarray)
    def update_bgs_contour_raw_image(self, cv_img): self.label_bgs_cont_raw_video.setPixmap(self.convert_cv_qt(cv_img))
    @pyqtSlot(np.ndarray)
    def update_bgs_contour_crop_image(self, cv_img): self.label_bgs_cont_crop_video.setPixmap(self.convert_cv_qt(cv_img, 224, 224))

    @pyqtSlot(np.ndarray)
    def update_gel_contour_raw_image(self, cv_img): self.label_gel_cont_raw_video.setPixmap(self.convert_cv_qt(cv_img))
    @pyqtSlot(np.ndarray)
    def update_gel_contour_crop_image(self, cv_img): self.label_gel_cont_crop_video.setPixmap(self.convert_cv_qt(cv_img, 224, 224))

    @pyqtSlot(np.ndarray)
    def update_diff_image(self, cv_img): self.label_diff_video.setPixmap(self.convert_cv_qt(cv_img))
    @pyqtSlot(np.ndarray)
    def update_crop_diff_image(self, cv_img): self.label_crop_diff_video.setPixmap(self.convert_cv_qt(cv_img, 224, 224))

    def closeEvent(self, event): self.video_thread.stop()

# ==========================================
# 6. MAIN DASHBOARD (Integration)
# ==========================================
class MainApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Vision & CoinFT Ultimate Advanced Analytics Dashboard (5x2 Grid)')
        main_layout = QHBoxLayout()
        left_layout = QVBoxLayout()
        self.led_panel = LEDControlPanel(); self.quest_pose_panel = QuestControllerPosePanel(); self.ft_panel = FTSensorPanel(); self.camera_panel = CameraMonitorPanel()
        
        self.camera_panel.req_capture.connect(self.ft_panel.ft_thread.capture_single_frame)
        self.camera_panel.req_record.connect(self.ft_panel.ft_thread.toggle_recording)

        self.led_panel.setMaximumHeight(240)
        left_layout.setSpacing(8)
        left_layout.addWidget(self.led_panel, 0)
        left_layout.addWidget(self.quest_pose_panel, 3)
        left_layout.addWidget(self.ft_panel, 2)
        
        line = QFrame(); line.setFrameShape(QFrame.VLine); line.setFrameShadow(QFrame.Sunken)
        main_layout.addLayout(left_layout); main_layout.addWidget(line); main_layout.addWidget(self.camera_panel)
        self.setLayout(main_layout)

    def closeEvent(self, event):
        self.camera_panel.closeEvent(event); self.led_panel.closeEvent(event); self.quest_pose_panel.closeEvent(event); self.ft_panel.closeEvent(event)
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv); app.setStyle('Fusion')
    window = MainApp()
    
    # Optional: Maximize window on start since we have 10 video feeds
    window.showMaximized() 
    
    sys.exit(app.exec_())
