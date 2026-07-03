from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable


INVALID_DEVICE_INDEX = 0xFFFFFFFF


@dataclass
class ControllerPose:
    timestamp: float
    elapsed: float
    role: str
    device_index: int
    connected: bool
    pose_valid: bool
    tracking_result: str
    model: str
    serial: str
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
    vx: float | None = None
    vy: float | None = None
    vz: float | None = None
    angular_vx: float | None = None
    angular_vy: float | None = None
    angular_vz: float | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read real-time left and right controller poses from SteamVR/OpenVR."
    )
    parser.add_argument("--rate", type=float, default=30.0, help="Sampling rate in Hz. Default: 30")
    parser.add_argument("--duration", type=float, default=None, help="Stop after N seconds.")
    parser.add_argument("--once", action="store_true", help="Print one sample and exit.")
    parser.add_argument(
        "--universe",
        choices=("standing", "seated", "raw"),
        default="standing",
        help="OpenVR tracking universe. Default: standing",
    )
    parser.add_argument("--csv", type=Path, default=None, help="Optional CSV output path.")
    parser.add_argument("--jsonl", type=Path, default=None, help="Optional JSON Lines output path.")
    parser.add_argument(
        "--include-unassigned",
        action="store_true",
        help="Also print controller-like devices that SteamVR has not labeled left/right.",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Do not clear the terminal between live samples.",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run math/conversion checks without connecting to SteamVR.",
    )
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="List all tracked devices that SteamVR currently exposes, then exit.",
    )
    return parser.parse_args()


def import_openvr() -> Any:
    try:
        import openvr
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "The Python package 'openvr' is not installed.\n"
            "Install it with: python -m pip install -r requirements.txt"
        ) from exc
    return openvr


def get_universe(openvr: Any, name: str) -> int:
    return {
        "standing": openvr.TrackingUniverseStanding,
        "seated": openvr.TrackingUniverseSeated,
        "raw": openvr.TrackingUniverseRawAndUncalibrated,
    }[name]


def get_tracking_result_name(openvr: Any, value: int) -> str:
    known = {
        getattr(openvr, "TrackingResult_Uninitialized", None): "Uninitialized",
        getattr(openvr, "TrackingResult_Calibrating_InProgress", None): "Calibrating",
        getattr(openvr, "TrackingResult_Calibrating_OutOfRange", None): "CalibratingOutOfRange",
        getattr(openvr, "TrackingResult_Running_OK", None): "RunningOK",
        getattr(openvr, "TrackingResult_Running_OutOfRange", None): "RunningOutOfRange",
    }
    return known.get(value, str(value))


def get_string_property(openvr: Any, vr_system: Any, device_index: int, prop_name: str) -> str:
    prop = getattr(openvr, prop_name, None)
    if prop is None:
        return ""
    try:
        return vr_system.getStringTrackedDeviceProperty(device_index, prop)
    except Exception:
        return ""


def role_name(openvr: Any, role_value: int) -> str:
    if role_value == getattr(openvr, "TrackedControllerRole_LeftHand", -1):
        return "left"
    if role_value == getattr(openvr, "TrackedControllerRole_RightHand", -1):
        return "right"
    return "unassigned"


def device_class_name(openvr: Any, class_value: int) -> str:
    known = {
        getattr(openvr, "TrackedDeviceClass_Invalid", None): "Invalid",
        getattr(openvr, "TrackedDeviceClass_HMD", None): "HMD",
        getattr(openvr, "TrackedDeviceClass_Controller", None): "Controller",
        getattr(openvr, "TrackedDeviceClass_GenericTracker", None): "GenericTracker",
        getattr(openvr, "TrackedDeviceClass_TrackingReference", None): "TrackingReference",
        getattr(openvr, "TrackedDeviceClass_DisplayRedirect", None): "DisplayRedirect",
    }
    return known.get(class_value, str(class_value))


def controller_indices(openvr: Any, vr_system: Any, include_unassigned: bool) -> dict[str, int]:
    indices: dict[str, int] = {}

    for role, label in (
        (openvr.TrackedControllerRole_LeftHand, "left"),
        (openvr.TrackedControllerRole_RightHand, "right"),
    ):
        try:
            index = vr_system.getTrackedDeviceIndexForControllerRole(role)
        except Exception:
            index = INVALID_DEVICE_INDEX
        if index != INVALID_DEVICE_INDEX:
            indices[label] = int(index)

    max_devices = int(getattr(openvr, "k_unMaxTrackedDeviceCount", 64))
    controller_class = getattr(openvr, "TrackedDeviceClass_Controller", None)

    for index in range(max_devices):
        try:
            device_class = vr_system.getTrackedDeviceClass(index)
        except Exception:
            continue
        if device_class != controller_class:
            continue

        try:
            role = role_name(openvr, vr_system.getControllerRoleForTrackedDeviceIndex(index))
        except Exception:
            role = "unassigned"

        if role in {"left", "right"}:
            indices.setdefault(role, index)
        elif include_unassigned:
            indices.setdefault(f"unassigned-{index}", index)

    return indices


def list_devices(openvr: Any, vr_system: Any, universe: int) -> None:
    poses = vr_system.getDeviceToAbsoluteTrackingPose(
        universe, 0, openvr.k_unMaxTrackedDeviceCount
    )
    print("idx class             role       connected valid tracking             model / serial")
    print("-" * 96)
    for index in range(int(getattr(openvr, "k_unMaxTrackedDeviceCount", 64))):
        try:
            device_class = vr_system.getTrackedDeviceClass(index)
        except Exception:
            continue
        if device_class == getattr(openvr, "TrackedDeviceClass_Invalid", -1):
            continue

        pose = poses[index]
        try:
            role = role_name(openvr, vr_system.getControllerRoleForTrackedDeviceIndex(index))
        except Exception:
            role = ""
        model = get_string_property(openvr, vr_system, index, "Prop_ModelNumber_String")
        serial = get_string_property(openvr, vr_system, index, "Prop_SerialNumber_String")
        tracking = get_tracking_result_name(openvr, getattr(pose, "eTrackingResult", -1))
        print(
            f"{index:>3} {device_class_name(openvr, device_class):<17} {role:<10} "
            f"{str(bool(getattr(pose, 'bDeviceIsConnected', False))):<9} "
            f"{str(bool(getattr(pose, 'bPoseIsValid', False))):<5} "
            f"{tracking:<20} {model or 'unknown model'} / {serial or 'unknown serial'}"
        )


def matrix34_rows(matrix: Any) -> list[list[float]]:
    raw = getattr(matrix, "m", matrix)
    rows: list[list[float]] = []

    try:
        if len(raw) == 3 and all(hasattr(raw[i], "__iter__") for i in range(3)):
            for row_index in range(3):
                rows.append([float(raw[row_index][col]) for col in range(4)])
            return rows
    except Exception:
        pass

    flat = [float(item) for item in raw]
    if len(flat) != 12:
        raise ValueError(f"Expected a 3x4 matrix or 12 flat values, got {len(flat)} values.")
    return [flat[0:4], flat[4:8], flat[8:12]]


def quaternion_from_rotation_matrix(rows: list[list[float]]) -> tuple[float, float, float, float]:
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


def euler_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
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


def vector3(vector: Any) -> tuple[float | None, float | None, float | None]:
    try:
        return float(vector[0]), float(vector[1]), float(vector[2])
    except Exception:
        return None, None, None


def pose_to_record(
    openvr: Any,
    vr_system: Any,
    role: str,
    device_index: int,
    pose: Any,
    started_at: float,
) -> ControllerPose:
    timestamp = time.time()
    record = ControllerPose(
        timestamp=timestamp,
        elapsed=timestamp - started_at,
        role=role,
        device_index=device_index,
        connected=bool(getattr(pose, "bDeviceIsConnected", False)),
        pose_valid=bool(getattr(pose, "bPoseIsValid", False)),
        tracking_result=get_tracking_result_name(openvr, getattr(pose, "eTrackingResult", -1)),
        model=get_string_property(openvr, vr_system, device_index, "Prop_ModelNumber_String"),
        serial=get_string_property(openvr, vr_system, device_index, "Prop_SerialNumber_String"),
    )

    if record.pose_valid:
        rows = matrix34_rows(pose.mDeviceToAbsoluteTracking)
        record.x = rows[0][3]
        record.y = rows[1][3]
        record.z = rows[2][3]
        record.qx, record.qy, record.qz, record.qw = quaternion_from_rotation_matrix(rows)
        record.roll_deg, record.pitch_deg, record.yaw_deg = euler_from_quaternion(
            record.qx, record.qy, record.qz, record.qw
        )

    record.vx, record.vy, record.vz = vector3(getattr(pose, "vVelocity", (None, None, None)))
    record.angular_vx, record.angular_vy, record.angular_vz = vector3(
        getattr(pose, "vAngularVelocity", (None, None, None))
    )
    return record


def format_float(value: float | None, width: int = 8, digits: int = 3) -> str:
    if value is None or not math.isfinite(value):
        return "--".rjust(width)
    return f"{value:{width}.{digits}f}"


def render(records: Iterable[ControllerPose], universe: str, clear: bool) -> None:
    if clear:
        print("\033[2J\033[H", end="")

    print(f"SteamVR controller pose monitor | universe={universe} | Ctrl+C to stop")
    print("-" * 104)
    print(
        "role   idx valid connected      x        y        z     roll    pitch      yaw    model / serial"
    )
    print("-" * 104)
    for item in records:
        model = item.model or "unknown model"
        serial = item.serial or "unknown serial"
        print(
            f"{item.role:<6} {item.device_index:>3} "
            f"{str(item.pose_valid):<5} {str(item.connected):<9} "
            f"{format_float(item.x)} {format_float(item.y)} {format_float(item.z)} "
            f"{format_float(item.roll_deg)} {format_float(item.pitch_deg)} {format_float(item.yaw_deg)} "
            f"{model} / {serial}"
        )
    print()
    print("Position is in meters relative to the selected SteamVR tracking universe.")
    sys.stdout.flush()


def write_csv_header(path: Path) -> None:
    if path.exists() and path.stat().st_size > 0:
        return
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(ControllerPose.__dataclass_fields__.keys()))
        writer.writeheader()


def append_csv(path: Path, records: Iterable[ControllerPose]) -> None:
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(ControllerPose.__dataclass_fields__.keys()))
        for record in records:
            writer.writerow(asdict(record))


def append_jsonl(path: Path, records: Iterable[ControllerPose]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(asdict(record), separators=(",", ":")) + "\n")


def run_self_test() -> int:
    identity_rows = [[1, 0, 0, 0.1], [0, 1, 0, 0.2], [0, 0, 1, -0.3]]
    qx, qy, qz, qw = quaternion_from_rotation_matrix(identity_rows)
    roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)

    checks = [
        abs(qx) < 1e-9,
        abs(qy) < 1e-9,
        abs(qz) < 1e-9,
        abs(qw - 1.0) < 1e-9,
        abs(roll) < 1e-9,
        abs(pitch) < 1e-9,
        abs(yaw) < 1e-9,
        matrix34_rows([1, 0, 0, 1, 0, 1, 0, 2, 0, 0, 1, 3])[2][3] == 3.0,
    ]

    if all(checks):
        print("Self-test passed.")
        return 0
    print("Self-test failed.", file=sys.stderr)
    return 1


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()

    if args.rate <= 0:
        raise SystemExit("--rate must be greater than zero.")

    openvr = import_openvr()

    try:
        openvr.init(openvr.VRApplication_Other)
    except Exception as exc:
        raise SystemExit(
            "Could not initialize OpenVR. Start SteamVR, connect Quest Link/Air Link, "
            f"then try again.\nOpenVR error: {exc}"
        ) from exc

    vr_system = openvr.VRSystem()
    universe = get_universe(openvr, args.universe)
    if args.list_devices:
        list_devices(openvr, vr_system, universe)
        openvr.shutdown()
        return 0

    interval = 1.0 / args.rate
    started_at = time.time()

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        write_csv_header(args.csv)
    if args.jsonl:
        args.jsonl.parent.mkdir(parents=True, exist_ok=True)

    try:
        while True:
            indices = controller_indices(openvr, vr_system, args.include_unassigned)
            poses = vr_system.getDeviceToAbsoluteTrackingPose(
                universe, 0, openvr.k_unMaxTrackedDeviceCount
            )
            records = [
                pose_to_record(openvr, vr_system, role, device_index, poses[device_index], started_at)
                for role, device_index in sorted(indices.items())
            ]

            if not records:
                if not args.no_clear:
                    print("\033[2J\033[H", end="")
                print("No left/right controllers found in SteamVR yet.")
                print("Wake both controllers and confirm SteamVR shows both controller icons.")
            else:
                render(records, args.universe, clear=not args.no_clear)
                if args.csv:
                    append_csv(args.csv, records)
                if args.jsonl:
                    append_jsonl(args.jsonl, records)

            if args.once:
                break
            if args.duration is not None and time.time() - started_at >= args.duration:
                break
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        openvr.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
