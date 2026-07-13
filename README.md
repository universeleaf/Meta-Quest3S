# Meta Quest 3S Controller Pose Dashboard

This repository provides two ways to stream Meta Quest Touch controller poses
into the CoinFT Python dashboard. Both modes display the controller position,
orientation axes, and full-session 3D trajectory, and both save controller
positions to CSV when the existing `Start Recording` button is enabled.

## Choose a Tracking Mode

| Mode | Entry point | Host operating system | Quest software path |
| --- | --- | --- | --- |
| SteamVR/OpenVR | `coinft.py` | Windows | Meta Quest Link -> SteamVR |
| Quest Browser/WebXR | `coinft_webxr.py` | Windows or Linux | Native Quest Browser |

Use `coinft.py` when the experiment already depends on SteamVR on Windows.
Use `coinft_webxr.py` for Linux, or when SteamVR and Quest Link should not be
part of the tracking pipeline.

## Repository Files

- `coinft.py`: complete dashboard with SteamVR/OpenVR controller tracking.
- `coinft_webxr.py`: complete dashboard with Quest Browser/WebXR tracking.
- `requirements.txt`: shared Python dependencies for both modes.
- `README.md`: installation, operation, recording, and troubleshooting.

No Python program runs directly on the Quest. The dashboard runs on the host
computer. In WebXR mode, a small page served by `coinft_webxr.py` runs in the
native Quest Browser and streams controller poses back to the dashboard.

## Requirements

### Common hardware

- Meta Quest 3S
- One or two paired Touch controllers
- A USB data cable for initial ADB authorization and WebXR port forwarding
- A Windows or Linux computer with OpenGL support

### Python

Python 3.10 or newer is required. Python 3.11 or 3.12 is recommended for the
widest scientific-package compatibility.

The shared `requirements.txt` installs:

- PyQt5 and pyqtgraph for the dashboard
- PyOpenGL for the 3D controller trajectory
- NumPy, SciPy, OpenCV, and ONNX Runtime for the original CoinFT dashboard
- pyserial for the LED and force/torque hardware
- openvr for the Windows SteamVR mode

The WebXR server uses only the Python standard library and does not require an
extra WebSocket package.

## Clone and Install

### Windows

```powershell
git clone https://github.com/universeleaf/Meta-Quest3S.git
cd Meta-Quest3S
py -3.12 -m venv meta
.\meta\Scripts\python.exe -m pip install --upgrade pip
.\meta\Scripts\python.exe -m pip install -r requirements.txt
```

If Python 3.12 is installed as `python` instead of through the Python launcher,
replace `py -3.12` with `python`.

In VS Code, select this interpreter:

```text
<repository>\meta\Scripts\python.exe
```

The folder named `meta` is a Python virtual environment, not a Conda
environment. Do not run `conda activate meta` unless a separate Conda
environment with that name was intentionally created.

### Ubuntu/Debian Linux

Install Git, ADB, Python, and the common Qt/OpenGL runtime libraries:

```bash
sudo apt update
sudo apt install -y \
  git adb android-sdk-platform-tools-common \
  python3 python3-venv python3-pip \
  libgl1 libegl1 libxkbcommon-x11-0 libxcb-xinerama0 libxcb-cursor0
```

Then clone the repository and create the environment:

```bash
git clone https://github.com/universeleaf/Meta-Quest3S.git
cd Meta-Quest3S
python3 -m venv meta
source meta/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

For Fedora, Arch, or another Linux distribution, install the equivalent ADB,
Python venv, Qt/XCB, and OpenGL runtime packages through that distribution's
package manager.

## Local CoinFT Hardware Configuration

Both Python files preserve the original LED, CoinFT sensor, and camera code.
Update these values near the top of the selected Python file when those devices
are required:

```python
LED_SERIAL_PORT = 'COM3'             # Windows example
FT_SERIAL_PORT = 'COM4'              # Windows example
FT_DATA_DIR = r'C:\path\to\pvft'
```

Typical Linux serial ports are `/dev/ttyACM0`, `/dev/ttyACM1`, or
`/dev/ttyUSB0`. Linux users may need serial access:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in after changing group membership.

The CoinFT model directory must contain:

- `PFT5-1_MLP_5L_norm_L2.onnx`
- `PFT5-1_norm_constants.mat`

The dashboard can still run in controller-pose-only mode when the LED serial
device, force/torque sensor, model files, or camera are unavailable. Those
panels will report that their hardware is unavailable.

# Mode A: Windows SteamVR/OpenVR

This mode reads controller poses from the PC SteamVR runtime through the
Python `openvr` package.

## Additional software

Install on Windows:

1. Meta Quest Link / Meta Horizon Link
2. Steam
3. SteamVR

In the Meta Quest Link desktop app:

1. Open `Settings`.
2. Enable `Unknown sources`.
3. Confirm Meta Quest Link is the active OpenXR runtime.

## Run SteamVR mode

1. Turn on the Quest and wake the controllers.
2. Connect with a Quest Link USB cable or Air Link.
3. Enter the Quest Link PC VR environment inside the headset.
4. Start SteamVR on the computer.
5. Confirm SteamVR shows the headset and controller icons.
6. Run the dashboard:

```powershell
.\meta\Scripts\python.exe coinft.py
```

The pose panel should report `OpenVR connected` and begin showing the
available controllers. One-controller operation is supported.

## SteamVR error 1114 workaround

If SteamVR reports `1114`, `OculusRuntimeBadInstall`, or cannot load the Meta
runtime DLL, add the installed Meta runtime folder to `PATH` before launching
SteamVR. Replace the example path with the actual installation location:

```powershell
$env:PATH = "D:\Meta Horizon\Support\oculus-runtime;$env:PATH"
& "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrstartup.exe"
```

Then run `coinft.py` again.

SteamVR itself has Linux builds, but Meta Quest Link does not provide the
equivalent Linux PC VR runtime used by this program. Use WebXR mode on Linux.

# Mode B: Windows/Linux Quest Browser WebXR

This mode does not use SteamVR, OpenVR, Meta Quest Link, or the Meta PC app.
It uses the native Quest Browser WebXR API and sends controller poses to the
Python dashboard over a local WebSocket.

## One-time Quest setup

1. Enable Developer Mode for the Quest under the device owner's Meta developer
   account.
2. Start the Quest and enter its normal standalone home environment.
3. Do not enter Quest Link or SteamVR.
4. Connect the Quest to the host computer with a USB data cable.
5. Put on the headset and accept `Allow USB debugging`.
6. Select `Always allow from this computer` before pressing `Allow`.

Verify the connection on Windows or Linux:

```text
adb devices -l
```

The device state must be `device`, not `unauthorized`:

```text
340YC10GCH0FGJ    device
```

The serial number will be different for another headset.

## Run WebXR mode

Start with the Quest fully booted in its standalone home environment and the
controllers awake.

1. Connect the Quest by USB and verify ADB:

```text
adb devices -l
```

2. Forward Quest localhost port 8765 to the host computer:

```text
adb reverse tcp:8765 tcp:8765
```

3. Start the Python dashboard.

Windows:

```powershell
.\meta\Scripts\python.exe coinft_webxr.py
```

Linux:

```bash
source meta/bin/activate
python coinft_webxr.py
```

4. Keep the Python dashboard running and open the page in the native Quest
   Browser without typing the URL:

```text
adb shell am start -a android.intent.action.VIEW -d http://127.0.0.1:8765
```

5. Inside the Quest Browser, press `Enter WebXR and Stream Poses`.
6. Accept the immersive VR request if the browser asks for confirmation.
7. Keep the controllers awake and visible to the headset tracking cameras.

The Python status should change to a message similar to:

```text
WebXR streaming 2 controller(s), target 60 Hz
```

Opening `http://127.0.0.1:8765` in a browser on the host computer is useful for
checking that the Python server is running, but a desktop browser cannot access
the Quest controller `gripSpace`. Controller streaming must be started from the
native Quest Browser.

## End a WebXR session

1. Press `Stop Recording` if recording is active.
2. Close the Python dashboard.
3. Exit WebXR or close the Quest Browser page.
4. Optionally remove the port forwarding:

```text
adb reverse --remove tcp:8765
```

## Recover an unauthorized ADB connection

First keep the headset awake, unlocked, and in the standalone Quest home. Exit
Quest Link before reconnecting USB.

Restart ADB:

```text
adb kill-server
adb start-server
adb devices -l
```

If the state remains `unauthorized`, disconnect USB, restart the Quest, reconnect
while wearing the headset, and accept the RSA debugging dialog.

If the dialog never appears, back up the host ADB key so ADB creates a new one.

Windows PowerShell:

```powershell
adb kill-server
$stamp = Get-Date -Format yyyyMMddHHmmss
Rename-Item "$env:USERPROFILE\.android\adbkey" "adbkey.$stamp.bak"
Rename-Item "$env:USERPROFILE\.android\adbkey.pub" "adbkey.pub.$stamp.bak"
adb start-server
```

Linux:

```bash
adb kill-server
stamp=$(date +%Y%m%d%H%M%S)
mv ~/.android/adbkey ~/.android/adbkey.$stamp.bak
mv ~/.android/adbkey.pub ~/.android/adbkey.pub.$stamp.bak
adb start-server
```

Reconnect the headset and approve the newly generated computer key. If Linux
reports insufficient USB permissions rather than `unauthorized`, install the
distribution's Android udev rules and reconnect the headset.

## Pose Display and Sampling

Both modes display:

- live controller position in meters
- full-session 3D position trajectory
- controller orientation as red, green, and blue endpoint axes
- numeric `x`, `y`, and `z`
- numeric roll, pitch, and yaw
- an empty right-side region reserved for a future robot-arm pose view

The relevant settings are near the top of each Python file:

```python
QUEST_POSE_RATE_HZ = 60.0
QUEST_TRAJECTORY_SECONDS = None
QUEST_POSITION_POSE_AXIS_LENGTH = 0.18
```

- `QUEST_POSE_RATE_HZ = 60.0` requests a 60 Hz stream/update target.
- `QUEST_TRAJECTORY_SECONDS = None` keeps the complete trajectory until the
  program closes.
- Set `QUEST_TRAJECTORY_SECONDS` to `5.0`, for example, to retain only the most
  recent five seconds.
- `QUEST_POSITION_POSE_AXIS_LENGTH` controls the displayed endpoint-axis size.

The actual update rate can be limited by headset tracking, browser frame rate,
USB/system load, camera processing, and Qt rendering.

## Coordinate Frames

### SteamVR/OpenVR

`coinft.py` requests `TrackingUniverseStanding`. Position is measured in meters
relative to the current SteamVR standing/play-area origin.

The usual raw convention is:

- `+X`: right
- `+Y`: up
- `-Z`: forward

### WebXR

`coinft_webxr.py` requests the WebXR `local-floor` reference space. Position is
measured in meters relative to the floor-level local origin established for the
current browser XR session.

The WebXR convention is:

- `+X`: right
- `+Y`: up
- `-Z`: forward

Neither frame is GPS, a global laboratory frame, nor the camera coordinate
frame. Re-centering, recreating the play area, or starting a new WebXR session
can change the origin. Record a calibration transform when aligning controller
poses with a robot or external tracking system.

## Controller Trajectory Recording

The existing dashboard `Start Recording` button controls controller trajectory
recording in both modes. Press it only after valid controller poses appear.

Files are saved under:

```text
recordings/controller_trajectory_YYYYMMDD_HHMMSS.csv
```

Each row contains:

```text
timestamp,elapsed,role,device_index,x_m,y_m,z_m
```

Press `Stop Recording` before closing the application so all files are closed
cleanly. The original camera and force/torque recording behavior remains
connected to the same button when that hardware is available.

## Update an Existing Clone

From the repository directory:

```text
git pull --ff-only
```

Then update Python dependencies.

Windows:

```powershell
.\meta\Scripts\python.exe -m pip install -r requirements.txt
```

Linux:

```bash
source meta/bin/activate
python -m pip install -r requirements.txt
```

Virtual environments, recordings, camera captures, and generated CSV files are
excluded from Git.

## Troubleshooting

- `ModuleNotFoundError`: select the repository's `meta` interpreter and rerun
  `pip install -r requirements.txt` with that interpreter.
- `FT Sensor Model ... file does not exist`: update `FT_DATA_DIR`, or ignore the
  warning when testing only controller poses.
- `OpenVR not ready`: use `coinft.py`, enter Quest Link, and start SteamVR.
- `navigator.xr is unavailable`: open the page in the native Quest Browser
  through the forwarded localhost URL, not in a desktop browser.
- WebXR page does not open: confirm `coinft_webxr.py` is still running, repeat
  `adb reverse tcp:8765 tcp:8765`, and reopen the page.
- No controller pose: press `Enter WebXR and Stream Poses`, wake the controllers,
  and keep them inside the headset cameras' tracking area.
- Port 8765 is already in use: close an older dashboard process, or change
  `WEBXR_PORT` and use the same new port in both ADB commands.
- Linux serial permission denied: add the user to `dialout` and log in again.
- Linux ADB permission denied: install the Android udev-rules package for the
  distribution, reconnect USB, and run `adb devices -l` again.
