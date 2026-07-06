# Meta Quest 3S Controller Pose in CoinFT

This repository contains a single Python dashboard, `coinft.py`, that combines
the lab's original LED, CoinFT sensor, and vision panels with a live Meta Quest
3S controller pose panel.

The code runs on the Windows PC. It does not run inside the headset. The Quest
3S must be connected to the PC through Quest Link or Air Link, and SteamVR must
show the headset and controllers before controller pose data can appear.

## Files

- `coinft.py`: the full dashboard and all Quest controller pose code.
- `requirements.txt`: Python dependencies.

## Required Software

Install these on the Windows PC:

1. Python 3.10 or newer.
2. Meta Quest Link / Meta Horizon Link.
3. Steam.
4. SteamVR.

In the Meta Quest Link desktop app:

1. Open `Settings`.
2. Enable `Unknown sources`.
3. In `OpenXR Runtime`, make sure Meta Quest Link / Meta Horizon Link is the
   active runtime.

## Install Python Dependencies

Clone the repository and install the dependencies into a local virtual
environment:

```powershell
git clone https://github.com/universeleaf/Meta-Quest3S.git
cd Meta-Quest3S
python -m venv meta
.\meta\Scripts\python.exe -m pip install -r requirements.txt
```

If you already created the `meta` environment, only run the install command
again:

```powershell
.\meta\Scripts\python.exe -m pip install -r requirements.txt
```

## Edit Local Hardware Paths

Before running on a new computer, open `coinft.py` and update the local serial
ports and CoinFT model folder near the top of the file:

```python
LED_SERIAL_PORT = 'COM3'
FT_SERIAL_PORT = 'COM4'
FT_DATA_DIR = r'C:\path\to\pvft'
```

The CoinFT model files must exist inside `FT_DATA_DIR`:

- `PFT5-1_MLP_5L_norm_L2.onnx`
- `PFT5-1_norm_constants.mat`

If you only want to test the Quest pose panel, the dashboard can still open
when the LED or CoinFT serial devices are unavailable; those panels will simply
show their disconnected/default state.

## Start Quest Link and SteamVR

1. Turn on the Quest 3S and wake the controllers.
2. Connect the headset to the PC with Quest Link USB-C or Air Link.
3. Inside the headset, enter the Quest Link / PC VR environment.
4. Start SteamVR on the PC.
5. Confirm the SteamVR status window shows the headset icon and controller
   icons.

If only one controller has battery, the dashboard can still display the one
available controller.

## Run the Dashboard

From the repository folder:

```powershell
.\meta\Scripts\python.exe coinft.py
```

You can also open the folder in VS Code and run `coinft.py` with the Python
interpreter set to:

```text
.\meta\Scripts\python.exe
```

## Quest Controller Pose Panel

`Part 5: Quest Controller Pose` fills the left-side dashboard space between the
LED controls and the CoinFT sensor plots.

It displays:

- Position view: each tracked controller position in SteamVR standing space.
- Orientation view: each controller's local axes as arrows.
- Numeric position: `x`, `y`, `z` in meters.
- Numeric orientation: roll, pitch, yaw in degrees.

Color conventions:

- Red axis: `+X`
- Green axis: `+Y`
- Blue axis: `+Z`
- Cyan marker: left controller
- Orange marker: right controller

## Coordinate Frame

The Quest controller poses come from SteamVR/OpenVR using
`TrackingUniverseStanding`. The position is measured in meters relative to the
current SteamVR standing/play-area origin created by Quest Link and SteamVR.

It is not GPS, not a world coordinate system, and not the camera coordinate
system. Re-centering the headset or reconfiguring the VR play area can change
the origin.

The raw SteamVR convention is usually:

- `+x`: right
- `+y`: up
- `-z`: forward

The numeric readout uses the raw SteamVR coordinates. The 3D visualization maps
them into a display-friendly view while keeping the same meaning.

## SteamVR Runtime Workaround

If SteamVR shows error `1114`, `OculusRuntimeBadInstall`, or cannot load the
Oculus runtime DLL, add the Meta runtime folder to `PATH` before starting
SteamVR. Replace `D:\Meta Horizon` with your actual Meta Quest Link install
folder:

```powershell
$env:PATH = "D:\Meta Horizon\Support\oculus-runtime;$env:PATH"
& "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\bin\win64\vrstartup.exe"
```

Then run:

```powershell
.\meta\Scripts\python.exe coinft.py
```

`coinft.py` also tries to auto-detect the Meta runtime folder before connecting
to OpenVR, but the manual step above is useful when SteamVR itself fails before
Python starts.

## Troubleshooting

- `ModuleNotFoundError`: run
  `.\meta\Scripts\python.exe -m pip install -r requirements.txt`.
- `OpenVR not ready`: enter Quest Link / PC VR in the headset, start SteamVR,
  and confirm SteamVR shows the headset.
- No controller pose: wake the controllers, check batteries, and keep them
  visible to the headset cameras.
- SteamVR says no headset detected: restart Quest Link, reconnect the USB-C
  cable or Air Link session, then restart SteamVR.
- Pose appears frozen or invalid: move the controller into the headset camera
  view and press a controller button to wake it.
