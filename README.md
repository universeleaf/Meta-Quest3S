# Meta Quest 3S Controller Pose Monitor

Python tool for reading the real-time 6DoF pose of the left and right Meta
Quest 3S controllers through SteamVR/OpenVR.

The Quest does not run this Python script directly. Instead, use Quest Link or
Air Link so the headset and controllers appear to the PC as SteamVR tracked
devices. Python then reads the two controller poses from the SteamVR runtime.

## What it reports

For each controller:

- Position: `x`, `y`, `z` in meters.
- Orientation: quaternion `qx`, `qy`, `qz`, `qw`.
- Euler angles: roll, pitch, yaw in degrees.
- Linear velocity and angular velocity when SteamVR reports them.
- Device index, model name, serial number, connection status, and tracking
  validity.

The position is not GPS or room-absolute localization. It is relative to the
selected SteamVR tracking universe, which defaults to the standing-room origin.

## Hardware and software setup

1. Install the Meta Quest Link app on the Windows PC.
2. Install Steam and SteamVR.
3. Connect Quest 3S to the PC with Quest Link USB-C or Air Link.
4. Start SteamVR and confirm the headset plus both controllers are visible.
5. Keep the controllers awake while running the Python script.

## Python setup

```powershell
python -m venv meta
.\meta\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

## Run

```powershell
python steamvr_controller_pose.py
```

Without activating the environment first, run:

```powershell
.\meta\Scripts\python.exe steamvr_controller_pose.py
```

Useful options:

```powershell
python steamvr_controller_pose.py --rate 60
python steamvr_controller_pose.py --csv controller_pose.csv
python steamvr_controller_pose.py --jsonl controller_pose.jsonl
python steamvr_controller_pose.py --once
python steamvr_controller_pose.py --list-devices
python steamvr_controller_pose.py --universe raw
```

Use `Ctrl+C` to stop the live monitor.

## Coordinate notes

By default, the script uses `TrackingUniverseStanding`. In SteamVR/OpenVR, the
pose is a 3x4 transform from device space to the tracking universe. Translation
is measured in meters. The conventional axes are:

- `+x`: right
- `+y`: up
- `-z`: forward

If your experiment needs coordinates before room calibration, try
`--universe raw`. If it needs seated coordinates, use `--universe seated`.

## Troubleshooting

- `ModuleNotFoundError: openvr`: run `python -m pip install -r requirements.txt`.
- `VRInitError`: start SteamVR first and make sure Quest Link/Air Link is active.
- SteamVR error `1114`, `OculusRuntimeBadInstall`, or `Unable to load LibOVRRT DLL`:
  start SteamVR with Meta's runtime folder on `PATH`:

  ```powershell
  .\tools\start_steamvr_with_meta_runtime.ps1 -RestartSteamVR
  ```

  For a persistent fix, run PowerShell as Administrator and then run:

  ```powershell
  .\tools\fix_oculus_registry_admin.ps1
  ```

- Left or right controller is missing: wake the controller, check battery, and
  confirm SteamVR shows both controller icons.
- To see exactly what SteamVR exposes to Python, run
  `python steamvr_controller_pose.py --list-devices`.
- Pose valid is `False`: the controller is connected but currently not tracked,
  often because it is outside camera view or asleep.
