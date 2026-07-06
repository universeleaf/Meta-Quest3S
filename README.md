# Meta Quest 3S Controller Pose Monitor

Python tools for reading the real-time 6DoF pose of Meta Quest 3S Touch
controllers through SteamVR/OpenVR on a Windows PC.

The Python code runs on the PC, not inside the headset. The Quest must first be
connected to the PC through Quest Link, Air Link, or another PC VR path that
makes the headset and controllers visible in SteamVR.

## What This Reports

For each controller that SteamVR exposes:

- Position: `x`, `y`, `z` in meters.
- Orientation: quaternion `qx`, `qy`, `qz`, `qw`.
- Euler angles: roll, pitch, yaw in degrees.
- Linear velocity and angular velocity when SteamVR reports them.
- Device index, model name, serial number, connection status, and tracking
  validity.

The position is not GPS or global room localization. It is relative to the
selected SteamVR tracking universe. The default is `standing`.

## Repository Files

- `steamvr_controller_pose.py`: the main pose reader.
- `run_controller_pose.py`: optional convenience launcher. It can find Meta
  Quest Link and SteamVR, add the Meta runtime folder to `PATH`, start SteamVR,
  and then run the pose reader.
- `coinft.py`: CoinFT/camera dashboard from the lab, now with an embedded Quest
  controller pose visualization panel.
- `quest_controller_pose_panel.py`: reusable PyQtGraph/OpenVR panel used by
  `coinft.py`.
- `requirements.txt`: Python dependency list.

## Required Software

Install these on the Windows PC:

1. Python 3.10 or newer.
2. Meta Quest Link / Meta Horizon Link.
3. Steam.
4. SteamVR.

In the Meta Quest Link desktop app:

1. Open `Settings`.
2. Enable `Unknown sources`.
3. In the OpenXR Runtime section, make sure Meta Quest Link / Meta Horizon Link
   is the active OpenXR runtime.

## Hardware Setup

1. Charge the controller batteries.
2. Connect the Quest 3S to the PC with Quest Link USB-C or Air Link.
3. Put on the headset and enter the PC VR / SteamVR environment.
4. Make sure SteamVR shows the headset and the controller icons.
5. Keep the controllers awake and visible to the headset cameras.

If only one controller has battery, the script can still read the one available
controller.

## Quick Start

Clone the repository:

```powershell
git clone https://github.com/universeleaf/Meta-Quest3S.git
cd Meta-Quest3S
```

Create and use a virtual environment:

```powershell
python -m venv meta
.\meta\Scripts\python.exe -m pip install -r requirements.txt
```

Start Quest Link / Air Link, enter SteamVR in the headset, then run:

```powershell
.\meta\Scripts\python.exe run_controller_pose.py --restart-steamvr --rate 60
```

Stop the live stream with `Ctrl+C`.

## Run the CoinFT Dashboard

The integrated dashboard keeps the original LED, CoinFT, and camera panels and
adds a Quest controller pose panel in the left-side empty space. Start Quest
Link / Air Link and SteamVR first, then run:

```powershell
.\meta\Scripts\python.exe coinft.py
```

The new panel shows:

- Position view: controller position in the SteamVR standing coordinate space.
- Orientation view: controller local `x`, `y`, and `z` axes as colored arrows.
- Numeric readouts: `x`, `y`, `z`, roll, pitch, and yaw.

The lab hardware settings inside `coinft.py` are intentionally left in the
original file. On a new Windows PC, update the serial ports and model folder in
that file:

```python
LED_SERIAL_PORT = 'COM3'
FT_SERIAL_PORT = 'COM4'
FT_DATA_DIR = r'C:\path\to\pvft'
```

The CoinFT model files must exist in `FT_DATA_DIR`:

- `PFT5-1_MLP_5L_norm_L2.onnx`
- `PFT5-1_norm_constants.mat`

## Verify Devices First

Before recording data, it is useful to check what SteamVR exposes:

```powershell
.\meta\Scripts\python.exe run_controller_pose.py --restart-steamvr --list-devices
```

A working setup should print something like:

```text
idx class             role       connected valid tracking             model / serial
------------------------------------------------------------------------------------------------
  0 HMD               unassigned True      True  RunningOK            Meta Quest 3S / ...
  1 Controller        right      True      True  RunningOK            Meta Quest 3S (Right Controller) / ...
```

If the controller is missing, wake it with a button press, check the battery,
and confirm that SteamVR shows the controller icon.

## Record Data

Print live controller pose at 60 Hz:

```powershell
.\meta\Scripts\python.exe run_controller_pose.py --rate 60
```

Save to CSV:

```powershell
.\meta\Scripts\python.exe run_controller_pose.py --rate 60 --csv controller_pose.csv
```

Save to JSON Lines:

```powershell
.\meta\Scripts\python.exe run_controller_pose.py --rate 60 --jsonl controller_pose.jsonl
```

Print one sample and exit:

```powershell
.\meta\Scripts\python.exe run_controller_pose.py --once
```

Run the core script directly if SteamVR is already working:

```powershell
.\meta\Scripts\python.exe steamvr_controller_pose.py --rate 60 --include-unassigned
```

## If Your Install Paths Are Different

The launcher tries to auto-detect common locations. If auto-detection fails,
pass the paths manually.

Meta Quest Link root examples:

- `D:\Meta Horizon`
- `C:\Program Files\Oculus`
- `C:\Program Files\Meta Horizon`

SteamVR root example:

- `C:\Program Files (x86)\Steam\steamapps\common\SteamVR`

Run with explicit paths:

```powershell
.\meta\Scripts\python.exe run_controller_pose.py `
  --meta-root "D:\Meta Horizon" `
  --steamvr-root "C:\Program Files (x86)\Steam\steamapps\common\SteamVR" `
  --restart-steamvr `
  --rate 60
```

If you do not want the launcher to start SteamVR:

```powershell
.\meta\Scripts\python.exe run_controller_pose.py --no-start-steamvr --rate 60
```

## Manual SteamVR Runtime Workaround

Some Meta Quest Link installations put the Oculus runtime under a custom folder
such as `D:\Meta Horizon`. SteamVR may then show error `1114`,
`OculusRuntimeBadInstall`, or `Unable to load LibOVRRT DLL`.

If that happens, manually add the Meta runtime folder to `PATH` before starting
SteamVR. Replace `D:\Meta Horizon` with your own Meta Quest Link install path:

```powershell
$env:PATH = "D:\Meta Horizon\Support\oculus-runtime;$env:PATH"
& "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrstartup.exe"
```

Then run:

```powershell
.\meta\Scripts\python.exe steamvr_controller_pose.py --rate 60 --include-unassigned
```

The convenience launcher does the same `PATH` setup automatically when it can
find the Meta runtime.

## Coordinate Notes

By default, the script and the CoinFT dashboard use
`TrackingUniverseStanding`. In SteamVR/OpenVR, the pose is a 3x4 transform from
device space to the current standing tracking universe. Translation is measured
in meters. The origin is the calibrated PC VR tracking/play-area origin created
by Quest Link/SteamVR, not a camera coordinate system and not a global room/GPS
coordinate. Re-centering or reconfiguring the VR play area can change this
reference frame. The usual axes are:

- `+x`: right
- `+y`: up
- `-z`: forward

Use `--universe raw` for raw tracking coordinates or `--universe seated` for
seated coordinates:

```powershell
.\meta\Scripts\python.exe run_controller_pose.py --universe raw --rate 60
```

## Troubleshooting

- `ModuleNotFoundError: openvr`: run
  `.\meta\Scripts\python.exe -m pip install -r requirements.txt`.
- `VRInitError`: start Quest Link / Air Link, enter SteamVR in the headset, and
  rerun the command.
- SteamVR error `1114` or `Unable to load LibOVRRT DLL`: run
  `run_controller_pose.py` with `--restart-steamvr`, or use the manual `PATH`
  workaround above.
- No controllers are listed: wake the controllers, check batteries, and confirm
  SteamVR shows the controller icons.
- `pose_valid` is `False`: the controller is connected but not currently tracked,
  often because it is outside camera view or asleep.
