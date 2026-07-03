from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent
CORE_SCRIPT = REPO_ROOT / "steamvr_controller_pose.py"
REQUIREMENTS = REPO_ROOT / "requirements.txt"
STEAMVR_PROCESS_NAMES = (
    "vrserver.exe",
    "vrmonitor.exe",
    "vrwebhelper.exe",
    "vrcompositor.exe",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convenience launcher for Quest controller pose capture. It can add "
            "the Meta Quest Link runtime directory to PATH, start SteamVR, and "
            "then run steamvr_controller_pose.py."
        )
    )
    parser.add_argument("--rate", type=float, default=60.0, help="Sampling rate in Hz.")
    parser.add_argument("--duration", type=float, default=None, help="Stop after N seconds.")
    parser.add_argument("--once", action="store_true", help="Print one sample and exit.")
    parser.add_argument("--csv", type=Path, default=None, help="Optional CSV output path.")
    parser.add_argument("--jsonl", type=Path, default=None, help="Optional JSONL output path.")
    parser.add_argument(
        "--universe",
        choices=("standing", "seated", "raw"),
        default="standing",
        help="OpenVR tracking universe.",
    )
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="List SteamVR tracked devices, then exit.",
    )
    parser.add_argument(
        "--assigned-only",
        action="store_true",
        help="Only print devices that SteamVR labels as left or right controllers.",
    )
    parser.add_argument(
        "--no-clear",
        action="store_true",
        help="Do not clear the terminal between live samples.",
    )
    parser.add_argument(
        "--restart-steamvr",
        action="store_true",
        help="Close current SteamVR helper processes before launching SteamVR.",
    )
    parser.add_argument(
        "--no-start-steamvr",
        action="store_true",
        help="Do not launch SteamVR; only run the Python pose reader.",
    )
    parser.add_argument(
        "--meta-root",
        type=Path,
        default=None,
        help=(
            "Meta Quest Link installation folder, for example "
            "'D:\\Meta Horizon' or 'C:\\Program Files\\Oculus'."
        ),
    )
    parser.add_argument(
        "--steamvr-root",
        type=Path,
        default=None,
        help=(
            "SteamVR installation folder, for example "
            "'C:\\Program Files (x86)\\Steam\\steamapps\\common\\SteamVR'."
        ),
    )
    parser.add_argument(
        "--skip-install",
        action="store_true",
        help="Do not automatically install missing Python packages.",
    )
    return parser.parse_args()


def registry_value(root_name: str, subkey: str, value_name: str, wow64_flag: int = 0) -> str | None:
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


def registry_values(subkey: str, value_name: str) -> list[str]:
    if os.name != "nt":
        return []
    import winreg

    values: list[str] = []
    for root_name in ("HKEY_LOCAL_MACHINE", "HKEY_CURRENT_USER"):
        for flag in (winreg.KEY_WOW64_64KEY, winreg.KEY_WOW64_32KEY, 0):
            value = registry_value(root_name, subkey, value_name, flag)
            if value and value not in values:
                values.append(value)
    return values


def meta_runtime_from_root(root: Path) -> Path | None:
    candidates = [
        root,
        root / "Support" / "oculus-runtime",
    ]
    for candidate in candidates:
        if (candidate / "LibOVRRT64_1.dll").exists():
            return candidate
    return None


def find_meta_runtime(explicit_root: Path | None) -> Path | None:
    roots: list[Path] = []
    if explicit_root is not None:
        roots.append(explicit_root)

    for env_name in ("META_QUEST_LINK_ROOT", "META_HORIZON_ROOT", "OCULUS_BASE"):
        value = os.environ.get(env_name)
        if value:
            roots.append(Path(value))

    roots.extend(Path(value) for value in registry_values(r"SOFTWARE\Oculus VR, LLC\Oculus", "Base"))

    program_files = [
        Path(os.environ.get("ProgramFiles", r"C:\Program Files")),
        Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")),
    ]
    for base in program_files:
        roots.extend(
            [
                base / "Oculus",
                base / "Meta Horizon",
                base / "Meta Quest Link",
            ]
        )

    for drive in "CDEFG":
        roots.extend(
            [
                Path(f"{drive}:\\Meta Horizon"),
                Path(f"{drive}:\\Oculus"),
                Path(f"{drive}:\\Meta Quest Link"),
            ]
        )

    seen: set[str] = set()
    for root in roots:
        key = str(root).lower()
        if key in seen:
            continue
        seen.add(key)
        runtime = meta_runtime_from_root(root)
        if runtime is not None:
            return runtime
    return None


def steamvr_from_root(root: Path) -> Path | None:
    candidates = [
        root,
        root / "steamapps" / "common" / "SteamVR",
    ]
    for candidate in candidates:
        if (candidate / "bin" / "win64" / "vrstartup.exe").exists():
            return candidate
    return None


def find_steamvr_root(explicit_root: Path | None) -> Path | None:
    roots: list[Path] = []
    if explicit_root is not None:
        roots.append(explicit_root)

    value = os.environ.get("STEAMVR_ROOT")
    if value:
        roots.append(Path(value))

    for registry_path in registry_values(r"SOFTWARE\Valve\Steam", "SteamPath"):
        roots.append(Path(registry_path))
    for registry_path in registry_values(r"SOFTWARE\Valve\Steam", "InstallPath"):
        roots.append(Path(registry_path))

    roots.extend(
        [
            Path(r"C:\Program Files (x86)\Steam"),
            Path(r"C:\Program Files\Steam"),
            Path(r"D:\Steam"),
        ]
    )

    seen: set[str] = set()
    for root in roots:
        key = str(root).lower()
        if key in seen:
            continue
        seen.add(key)
        steamvr = steamvr_from_root(root)
        if steamvr is not None:
            return steamvr
    return None


def ensure_dependencies(skip_install: bool) -> None:
    try:
        import openvr  # noqa: F401
    except ModuleNotFoundError:
        if skip_install:
            raise SystemExit(
                "Missing Python package 'openvr'. Run: python -m pip install -r requirements.txt"
            )
        print("Installing Python dependencies from requirements.txt...", flush=True)
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", str(REQUIREMENTS)])


def process_is_running(image_name: str) -> bool:
    result = subprocess.run(
        ["tasklist", "/FI", f"IMAGENAME eq {image_name}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        check=False,
    )
    return image_name.lower() in result.stdout.lower()


def stop_steamvr_processes() -> None:
    for image_name in STEAMVR_PROCESS_NAMES:
        subprocess.run(
            ["taskkill", "/IM", image_name, "/F"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )


def start_steamvr(steamvr_root: Path, env: dict[str, str], restart: bool) -> None:
    if restart:
        print("Restarting SteamVR helper processes...", flush=True)
        stop_steamvr_processes()
        time.sleep(3)

    if process_is_running("vrserver.exe") and not restart:
        print("SteamVR already appears to be running.", flush=True)
        return

    vrstartup = steamvr_root / "bin" / "win64" / "vrstartup.exe"
    print(f"Starting SteamVR: {vrstartup}", flush=True)
    subprocess.Popen(
        [str(vrstartup)],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(8)


def build_core_args(args: argparse.Namespace) -> list[str]:
    core_args = [
        str(CORE_SCRIPT),
        "--rate",
        str(args.rate),
        "--universe",
        args.universe,
    ]
    if args.duration is not None:
        core_args.extend(["--duration", str(args.duration)])
    if args.once:
        core_args.append("--once")
    if args.csv is not None:
        core_args.extend(["--csv", str(args.csv)])
    if args.jsonl is not None:
        core_args.extend(["--jsonl", str(args.jsonl)])
    if args.list_devices:
        core_args.append("--list-devices")
    if not args.assigned_only:
        core_args.append("--include-unassigned")
    if args.no_clear:
        core_args.append("--no-clear")
    return core_args


def main() -> int:
    args = parse_args()
    ensure_dependencies(args.skip_install)

    env = os.environ.copy()
    meta_runtime = find_meta_runtime(args.meta_root)
    if meta_runtime is None:
        if args.meta_root is not None:
            raise SystemExit(f"Could not find LibOVRRT64_1.dll under: {args.meta_root}")
        print(
            "Warning: Meta Quest Link runtime was not auto-detected. "
            "If SteamVR shows error 1114, rerun with --meta-root.",
            flush=True,
        )
    else:
        env["PATH"] = f"{meta_runtime};{env.get('PATH', '')}"
        print(f"Using Meta runtime: {meta_runtime}", flush=True)

    if not args.no_start_steamvr:
        steamvr_root = find_steamvr_root(args.steamvr_root)
        if steamvr_root is None:
            if args.steamvr_root is not None:
                raise SystemExit(f"Could not find SteamVR under: {args.steamvr_root}")
            print(
                "Warning: SteamVR was not auto-detected. Start SteamVR manually, "
                "or rerun with --steamvr-root.",
                flush=True,
            )
        else:
            start_steamvr(steamvr_root, env, args.restart_steamvr)

    command = [sys.executable, *build_core_args(args)]
    return subprocess.call(command, env=env)


if __name__ == "__main__":
    raise SystemExit(main())
