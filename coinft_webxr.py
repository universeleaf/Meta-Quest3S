
import sys
import os
import math
import serial
import time
import cv2
import csv
import base64
import hashlib
import json
import numpy as np
import scipy.io
import onnxruntime as ort
import socket
import struct
import threading
import urllib.parse
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
QUEST_POSE_RATE_HZ = 60.0
WEBXR_BIND_HOST = "0.0.0.0"
WEBXR_PORT = 8765
WEBXR_REFERENCE_SPACE = "local-floor"
# Set to None to keep the full trajectory for the whole run.
QUEST_TRAJECTORY_SECONDS = None
QUEST_POSITION_POSE_AXIS_LENGTH = 0.18
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


def quest_trajectory_label() -> str:
    if QUEST_TRAJECTORY_SECONDS is None:
        return "full-session trail"
    return f"last {QUEST_TRAJECTORY_SECONDS:g}s trail"


WEBXR_PAGE_HTML = r"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Quest Controller WebXR Pose Stream</title>
  <style>
    body {
      margin: 0;
      min-height: 100vh;
      background: #101318;
      color: #f4f7fb;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      display: grid;
      place-items: center;
    }
    main {
      width: min(780px, calc(100vw - 48px));
      padding: 32px;
      border: 1px solid #2a3340;
      background: #171d25;
      border-radius: 12px;
    }
    h1 { margin: 0 0 12px; font-size: 28px; }
    p { line-height: 1.55; color: #cbd5e1; }
    button {
      width: 100%;
      min-height: 56px;
      margin: 20px 0 14px;
      border: 0;
      border-radius: 8px;
      background: #4f8cff;
      color: white;
      font-size: 18px;
      font-weight: 700;
    }
    button:disabled { opacity: 0.55; }
    code {
      background: #0b0f14;
      color: #a7f3d0;
      padding: 2px 5px;
      border-radius: 4px;
    }
    #status, #detail {
      white-space: pre-wrap;
      background: #0b0f14;
      color: #d1d5db;
      padding: 12px;
      border-radius: 8px;
      min-height: 22px;
    }
  </style>
</head>
<body>
  <main>
    <h1>Quest Controller WebXR Pose Stream</h1>
    <p>
      Keep this page open in Quest Browser. Press the button below from inside
      the headset, allow VR, and move the Touch controllers.
    </p>
    <button id="start">Enter WebXR and Stream Poses</button>
    <div id="status">Loading...</div>
    <p id="detail"></p>
  </main>
  <script>
    const TARGET_RATE_HZ = Number("__TARGET_RATE_HZ__");
    const REFERENCE_SPACE_NAME = "__REFERENCE_SPACE__";
    const startButton = document.getElementById("start");
    const statusBox = document.getElementById("status");
    const detailBox = document.getElementById("detail");

    let socket = null;
    let session = null;
    let referenceSpace = null;
    let actualReferenceSpaceName = REFERENCE_SPACE_NAME;
    let lastSendTime = 0;
    let frameCount = 0;
    let fpsWindowStart = performance.now();
    let currentFps = 0;

    function setStatus(message) {
      statusBox.textContent = message;
    }

    function setDetail(message) {
      detailBox.textContent = message;
    }

    function websocketUrl() {
      const scheme = location.protocol === "https:" ? "wss" : "ws";
      return `${scheme}://${location.host}/ws`;
    }

    function connectSocket() {
      if (socket && socket.readyState === WebSocket.OPEN) {
        return Promise.resolve();
      }
      return new Promise((resolve, reject) => {
        socket = new WebSocket(websocketUrl());
        socket.onopen = () => {
          socket.send(JSON.stringify({
            type: "hello",
            userAgent: navigator.userAgent,
            targetRateHz: TARGET_RATE_HZ,
            referenceSpace: REFERENCE_SPACE_NAME
          }));
          resolve();
        };
        socket.onerror = () => reject(new Error("WebSocket connection failed"));
        socket.onclose = () => setStatus("WebSocket closed. Refresh this page or press the button again.");
      });
    }

    async function requestReferenceSpace(xrSession) {
      try {
        referenceSpace = await xrSession.requestReferenceSpace(REFERENCE_SPACE_NAME);
        actualReferenceSpaceName = REFERENCE_SPACE_NAME;
      } catch (error) {
        referenceSpace = await xrSession.requestReferenceSpace("local");
        actualReferenceSpaceName = "local";
      }
    }

    async function enterXR() {
      startButton.disabled = true;
      try {
        if (!navigator.xr) {
          throw new Error("navigator.xr is unavailable. Use Quest Browser and a secure/localhost origin.");
        }

        const supported = await navigator.xr.isSessionSupported("immersive-vr");
        if (!supported) {
          throw new Error("immersive-vr is not supported on this browser.");
        }

        await connectSocket();
        session = await navigator.xr.requestSession("immersive-vr", {
          requiredFeatures: [REFERENCE_SPACE_NAME]
        }).catch(() => navigator.xr.requestSession("immersive-vr"));

        const canvas = document.createElement("canvas");
        const gl = canvas.getContext("webgl", { xrCompatible: true });
        await gl.makeXRCompatible();
        session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl) });
        await requestReferenceSpace(session);

        session.addEventListener("end", () => {
          setStatus("XR session ended. Press the button to start again.");
          startButton.disabled = false;
        });

        setStatus(`Streaming controller poses at up to ${TARGET_RATE_HZ} Hz.`);
        setDetail(`Reference space: ${actualReferenceSpaceName}`);
        session.requestAnimationFrame(onXRFrame);
      } catch (error) {
        startButton.disabled = false;
        setStatus(`Could not start WebXR: ${error.message}`);
      }
    }

    function onXRFrame(time, frame) {
      const xrSession = frame.session;
      const poseRecords = [];

      for (const inputSource of xrSession.inputSources) {
        if (!inputSource.gripSpace) {
          continue;
        }
        const pose = frame.getPose(inputSource.gripSpace, referenceSpace);
        if (!pose) {
          continue;
        }
        const transform = pose.transform;
        const position = transform.position;
        const orientation = transform.orientation;
        const handedness = inputSource.handedness || "none";

        poseRecords.push({
          role: handedness === "left" || handedness === "right" ? handedness : "unassigned",
          handedness,
          targetRayMode: inputSource.targetRayMode || "",
          position: [position.x, position.y, position.z],
          orientation: [orientation.x, orientation.y, orientation.z, orientation.w]
        });
      }

      frameCount += 1;
      if (time - fpsWindowStart >= 1000) {
        currentFps = frameCount * 1000 / (time - fpsWindowStart);
        frameCount = 0;
        fpsWindowStart = time;
      }

      if (socket && socket.readyState === WebSocket.OPEN && time - lastSendTime >= 1000 / TARGET_RATE_HZ) {
        socket.send(JSON.stringify({
          type: "poses",
          timestamp: performance.now() / 1000,
          frameRate: currentFps,
          referenceSpace: actualReferenceSpaceName,
          poses: poseRecords
        }));
        lastSendTime = time;
      }

      if (poseRecords.length > 0) {
        const roles = poseRecords.map((record) => record.role).join(", ");
        setDetail(`Reference space: ${actualReferenceSpaceName}\nTracked: ${roles}\nBrowser XR FPS: ${currentFps.toFixed(1)}`);
      } else {
        setDetail(`Reference space: ${actualReferenceSpaceName}\nNo controller grip pose yet. Wake the controllers and keep them visible.`);
      }

      xrSession.requestAnimationFrame(onXRFrame);
    }

    startButton.addEventListener("click", enterXR);
    setStatus(`Ready. Target stream rate: ${TARGET_RATE_HZ} Hz.`);
  </script>
</body>
</html>
"""


def webxr_page_html() -> bytes:
    html = WEBXR_PAGE_HTML.replace("__TARGET_RATE_HZ__", f"{QUEST_POSE_RATE_HZ:g}")
    html = html.replace("__REFERENCE_SPACE__", WEBXR_REFERENCE_SPACE)
    return html.encode("utf-8")


def webxr_local_ip_addresses() -> list[str]:
    addresses: list[str] = []
    try:
        hostname = socket.gethostname()
        for result in socket.getaddrinfo(hostname, None, socket.AF_INET):
            address = result[4][0]
            if address not in addresses and not address.startswith("127."):
                addresses.append(address)
    except OSError:
        pass

    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        address = probe.getsockname()[0]
        probe.close()
        if address not in addresses and not address.startswith("127."):
            addresses.insert(0, address)
    except OSError:
        pass

    return addresses


def webxr_read_http_request(connection: socket.socket):
    data = b""
    while b"\r\n\r\n" not in data and len(data) < 65536:
        chunk = connection.recv(4096)
        if not chunk:
            break
        data += chunk
    if not data:
        return None

    header_data = data.split(b"\r\n\r\n", 1)[0]
    lines = header_data.decode("iso-8859-1", errors="replace").split("\r\n")
    if not lines:
        return None
    parts = lines[0].split()
    if len(parts) < 2:
        return None

    headers = {}
    for line in lines[1:]:
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        headers[key.strip().lower()] = value.strip()
    return parts[0], parts[1], headers


def webxr_send_http_response(connection: socket.socket, status: str, content_type: str, body: bytes):
    header = (
        f"HTTP/1.1 {status}\r\n"
        f"Content-Type: {content_type}\r\n"
        f"Content-Length: {len(body)}\r\n"
        "Cache-Control: no-store\r\n"
        "Connection: close\r\n"
        "\r\n"
    ).encode("utf-8")
    connection.sendall(header + body)


def webxr_accept_key(client_key: str) -> str:
    websocket_guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
    digest = hashlib.sha1((client_key + websocket_guid).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def webxr_send_ws_handshake(connection: socket.socket, headers: dict[str, str]):
    client_key = headers.get("sec-websocket-key", "")
    response = (
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {webxr_accept_key(client_key)}\r\n"
        "\r\n"
    ).encode("ascii")
    connection.sendall(response)


def webxr_read_exact(connection: socket.socket, size: int) -> bytes | None:
    data = b""
    while len(data) < size:
        chunk = connection.recv(size - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def webxr_read_ws_frame(connection: socket.socket):
    header = webxr_read_exact(connection, 2)
    if header is None:
        return None

    first, second = header[0], header[1]
    opcode = first & 0x0F
    masked = bool(second & 0x80)
    length = second & 0x7F

    if length == 126:
        extended = webxr_read_exact(connection, 2)
        if extended is None:
            return None
        length = struct.unpack("!H", extended)[0]
    elif length == 127:
        extended = webxr_read_exact(connection, 8)
        if extended is None:
            return None
        length = struct.unpack("!Q", extended)[0]

    mask = webxr_read_exact(connection, 4) if masked else None
    payload = webxr_read_exact(connection, length)
    if payload is None:
        return None

    if mask is not None:
        payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
    return opcode, payload


def webxr_send_ws_frame(connection: socket.socket, payload: bytes, opcode: int = 0x1):
    length = len(payload)
    first = 0x80 | opcode
    if length < 126:
        header = bytes([first, length])
    elif length < 65536:
        header = bytes([first, 126]) + struct.pack("!H", length)
    else:
        header = bytes([first, 127]) + struct.pack("!Q", length)
    connection.sendall(header + payload)


def webxr_normalize_quaternion(values: list[float]) -> tuple[float, float, float, float]:
    qx, qy, qz, qw = [float(value) for value in values[:4]]
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def webxr_pose_to_record(sample: dict[str, Any], received_at: float, started_at: float, fallback_index: int):
    role = str(sample.get("role") or sample.get("handedness") or f"controller-{fallback_index}")
    if role not in {"left", "right"}:
        role = f"controller-{fallback_index}"

    device_index = 1 if role == "left" else 2 if role == "right" else fallback_index
    record = QuestControllerPoseRecord(
        timestamp=received_at,
        elapsed=received_at - started_at,
        role=role,
        device_index=device_index,
        connected=True,
        pose_valid=False,
        tracking_result="WebXR",
    )

    position = sample.get("position") or []
    orientation = sample.get("orientation") or []
    if len(position) < 3 or len(orientation) < 4:
        return record

    try:
        record.x = float(position[0])
        record.y = float(position[1])
        record.z = float(position[2])
        record.qx, record.qy, record.qz, record.qw = webxr_normalize_quaternion(orientation)
        record.roll_deg, record.pitch_deg, record.yaw_deg = quest_euler_from_quaternion(
            record.qx, record.qy, record.qz, record.qw
        )
        record.pose_valid = True
    except (TypeError, ValueError):
        pass
    return record


class QuestControllerPoseThread(QThread):
    records_updated = pyqtSignal(object)
    status_message = pyqtSignal(str)

    def __init__(self, rate_hz: float = 30.0, host: str = WEBXR_BIND_HOST, port: int = WEBXR_PORT):
        super().__init__()
        self.rate_hz = rate_hz
        self.host = host
        self.port = port
        self._run_flag = True
        self._server_socket = None
        self._client_lock = threading.Lock()
        self._client_count = 0
        self._started_at = time.time()
        self._last_status_at = 0.0

    def stop(self):
        self._run_flag = False
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
        self.wait(1500)

    def _server_message(self) -> str:
        lan_addresses = webxr_local_ip_addresses()
        lan_part = f" LAN URL: http://{lan_addresses[0]}:{self.port}" if lan_addresses else ""
        return (
            f"WebXR receiver ready on http://127.0.0.1:{self.port}."
            f"{lan_part} For Quest Browser, use adb reverse tcp:{self.port} tcp:{self.port} "
            "then open the 127.0.0.1 URL."
        )

    def _handle_http_client(self, connection: socket.socket, address):
        try:
            request = webxr_read_http_request(connection)
            if request is None:
                return
            method, target, headers = request
            path = urllib.parse.urlparse(target).path
            wants_websocket = (
                headers.get("upgrade", "").lower() == "websocket"
                and "upgrade" in headers.get("connection", "").lower()
            )

            if wants_websocket and path == "/ws":
                webxr_send_ws_handshake(connection, headers)
                self._handle_websocket(connection, address)
                return

            if method == "GET" and path in {"", "/", "/index.html"}:
                webxr_send_http_response(connection, "200 OK", "text/html; charset=utf-8", webxr_page_html())
                return

            if method == "GET" and path == "/status":
                body = json.dumps({
                    "ok": True,
                    "target_rate_hz": self.rate_hz,
                    "reference_space": WEBXR_REFERENCE_SPACE,
                }).encode("utf-8")
                webxr_send_http_response(connection, "200 OK", "application/json", body)
                return

            webxr_send_http_response(connection, "404 Not Found", "text/plain; charset=utf-8", b"Not found")
        except OSError:
            return
        finally:
            try:
                connection.close()
            except OSError:
                pass

    def _handle_websocket(self, connection: socket.socket, address):
        with self._client_lock:
            self._client_count += 1
        self.status_message.emit(f"Quest Browser connected from {address[0]}. Enter WebXR in the headset.")

        try:
            while self._run_flag:
                frame = webxr_read_ws_frame(connection)
                if frame is None:
                    break
                opcode, payload = frame
                if opcode == 0x8:
                    break
                if opcode == 0x9:
                    webxr_send_ws_frame(connection, payload, opcode=0xA)
                    continue
                if opcode != 0x1:
                    continue

                try:
                    message = json.loads(payload.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    continue

                if message.get("type") == "hello":
                    self.status_message.emit(
                        f"Quest Browser page connected. Press Enter WebXR. Target {self.rate_hz:g} Hz."
                    )
                    continue

                if message.get("type") != "poses":
                    continue

                received_at = time.time()
                records = [
                    webxr_pose_to_record(sample, received_at, self._started_at, index + 1)
                    for index, sample in enumerate(message.get("poses") or [])
                ]
                self.records_updated.emit(records)

                if received_at - self._last_status_at >= 0.5:
                    valid_count = sum(1 for record in records if record.pose_valid)
                    frame_rate = message.get("frameRate")
                    if isinstance(frame_rate, (int, float)) and frame_rate > 0:
                        fps_text = f", browser XR FPS {frame_rate:.1f}"
                    else:
                        fps_text = ""
                    self.status_message.emit(
                        f"WebXR streaming {valid_count} controller(s), target {self.rate_hz:g} Hz{fps_text}."
                    )
                    self._last_status_at = received_at
        except OSError as exc:
            if self._run_flag:
                self.status_message.emit(f"WebXR connection closed: {exc}")
        finally:
            with self._client_lock:
                self._client_count = max(0, self._client_count - 1)
            if self._run_flag:
                self.status_message.emit("Quest Browser disconnected. Keep the server running and reconnect the page.")

    def run(self):
        try:
            server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_socket.bind((self.host, self.port))
            server_socket.listen(8)
            server_socket.settimeout(0.5)
            self._server_socket = server_socket
            self._started_at = time.time()
            self.status_message.emit(self._server_message())

            while self._run_flag:
                try:
                    connection, address = server_socket.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break

                worker = threading.Thread(
                    target=self._handle_http_client,
                    args=(connection, address),
                    daemon=True,
                )
                worker.start()
        except OSError as exc:
            self.status_message.emit(f"Could not start WebXR receiver on port {self.port}: {exc}")
        finally:
            if self._server_socket is not None:
                try:
                    self._server_socket.close()
                except OSError:
                    pass
            self._server_socket = None


class QuestControllerPosePanel(QWidget):
    def __init__(self):
        super().__init__()
        self._gl_available = gl is not None
        self._position_items = {}
        self._trajectory_history = {}
        self.is_recording = False
        self.pose_csv_file = None
        self.pose_csv_writer = None
        self.pose_recording_path = None

        self.pose_thread = QuestControllerPoseThread(rate_hz=QUEST_POSE_RATE_HZ)
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

        self.status_label = QLabel("Status: starting WebXR receiver...")
        self.status_label.setStyleSheet("color: blue; font-style: italic;")
        layout.addWidget(self.status_label)

        self.webxr_hint_label = QLabel(
            f"Quest Browser: run adb reverse tcp:{WEBXR_PORT} tcp:{WEBXR_PORT}, "
            f"then open http://127.0.0.1:{WEBXR_PORT}"
        )
        self.webxr_hint_label.setFont(QFont("Arial", 9))
        self.webxr_hint_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.webxr_hint_label)

        grid = QGridLayout()
        grid.setSpacing(8)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        self.position_group = QGroupBox(
            f"Position in WebXR {WEBXR_REFERENCE_SPACE} space, {quest_trajectory_label()}"
        )
        self.position_group.setLayout(QVBoxLayout())
        self.robot_pose_placeholder = QWidget()
        self.robot_pose_placeholder.setMinimumHeight(390)

        if self._gl_available:
            self.position_view = self._create_view(distance=3.0)
            self.position_group.layout().addWidget(self.position_view, 1)
            self._add_world_axes(self.position_view, length=0.75)
        else:
            self.position_group.layout().addWidget(QLabel("3D view unavailable: install PyOpenGL."))

        self.position_readout = QLabel("Position: waiting for WebXR controller data...")
        self.position_readout.setFont(QFont("Consolas", 9))
        self.position_readout.setTextInteractionFlags(Qt.TextSelectableByMouse)

        self.position_group.layout().addWidget(self.position_readout)
        grid.addWidget(self.position_group, 0, 0)
        grid.addWidget(self.robot_pose_placeholder, 0, 1)
        layout.addLayout(grid, 1)

        legend = QLabel(
            f"Axes: red=+X, green=+Y up, blue=+Z back. "
            f"Endpoint pose axes are drawn in the Position view. "
            f"WebXR uses meters; in local-floor, -Z is usually forward. "
            f"Target streaming: {QUEST_POSE_RATE_HZ:g} Hz. Trail: {quest_trajectory_label()}."
        )
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

    def _ensure_position_items(self, role: str):
        if role in self._position_items:
            return self._position_items[role]
        marker = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), color=self._role_color(role), size=13, pxMode=True)
        trajectory = self._add_line(self.position_view, self._role_color(role), width=3)
        radial = self._add_line(self.position_view, self._role_color(role), width=2)
        self.position_view.addItem(marker)
        self._position_items[role] = {
            "marker": marker,
            "trajectory": trajectory,
            "radial": radial,
            "pose_x": self._add_line(self.position_view, QUEST_RED, width=4),
            "pose_y": self._add_line(self.position_view, QUEST_GREEN, width=4),
            "pose_z": self._add_line(self.position_view, QUEST_BLUE, width=4),
        }
        return self._position_items[role]

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
        self._update_readouts(valid_records)
        self._record_pose_rows(valid_records)

    def _update_position_view(self, records):
        origin = np.zeros(3, dtype=float)
        for record in records:
            items = self._ensure_position_items(record.role)
            point = quest_map_vr_to_gl((record.x, record.y, record.z))
            items["marker"].setData(pos=np.array([point]), color=self._role_color(record.role), size=13)
            self._set_line(items["radial"], origin, point)
            self._update_trajectory_line(record.role, record.timestamp, point, items["trajectory"])
            self._update_endpoint_pose_axes(record, point, items)

    def _update_trajectory_line(self, role: str, timestamp: float, point: np.ndarray, line_item: Any):
        history = self._trajectory_history.setdefault(role, [])
        history.append((timestamp, point.copy()))

        if QUEST_TRAJECTORY_SECONDS is not None:
            cutoff = timestamp - QUEST_TRAJECTORY_SECONDS
            while history and history[0][0] < cutoff:
                history.pop(0)

        points = np.array([sample[1] for sample in history], dtype=float)
        if len(points) == 1:
            points = np.vstack([points[0], points[0]])
        line_item.setData(pos=points)

    def _update_endpoint_pose_axes(self, record: QuestControllerPoseRecord, point: np.ndarray, items: dict[str, Any]):
        axes = quest_quaternion_to_axes(record.qx, record.qy, record.qz, record.qw)
        for axis_name, axis_vector in zip(("pose_x", "pose_y", "pose_z"), axes):
            end = point + QUEST_POSITION_POSE_AXIS_LENGTH * quest_map_vr_to_gl(axis_vector)
            self._set_line(items[axis_name], point, end)

    def _update_readouts(self, records):
        if not records:
            self.position_readout.setText("Position: no valid controller pose.")
            return

        readout_lines = []
        for record in records:
            readout_lines.append(
                f"{record.role:<10} x={record.x: .3f} m  y={record.y: .3f} m  z={record.z: .3f} m  "
                f"roll={record.roll_deg: .1f}  "
                f"pitch={record.pitch_deg: .1f}  yaw={record.yaw_deg: .1f}"
            )

        self.position_readout.setText("\n".join(readout_lines))

    @pyqtSlot(bool)
    def toggle_recording(self, state: bool):
        if state:
            os.makedirs("recordings", exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.pose_recording_path = f"recordings/controller_trajectory_{timestamp}.csv"
            self.pose_csv_file = open(self.pose_recording_path, "w", newline="")
            self.pose_csv_writer = csv.writer(self.pose_csv_file)
            self.pose_csv_writer.writerow(["timestamp", "elapsed", "role", "device_index", "x_m", "y_m", "z_m"])
            self.is_recording = True
        else:
            self.is_recording = False
            self._close_pose_recording()

    def _record_pose_rows(self, records):
        if not self.is_recording or self.pose_csv_writer is None:
            return

        for record in records:
            self.pose_csv_writer.writerow([
                f"{record.timestamp:.6f}",
                f"{record.elapsed:.6f}",
                record.role,
                record.device_index,
                f"{record.x:.6f}",
                f"{record.y:.6f}",
                f"{record.z:.6f}",
            ])
        if self.pose_csv_file is not None:
            self.pose_csv_file.flush()

    def _close_pose_recording(self):
        if self.pose_csv_file is not None:
            self.pose_csv_file.close()
        self.pose_csv_file = None
        self.pose_csv_writer = None

    def closeEvent(self, event):
        self._close_pose_recording()
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
        self.camera_panel.req_record.connect(self.quest_pose_panel.toggle_recording)

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
