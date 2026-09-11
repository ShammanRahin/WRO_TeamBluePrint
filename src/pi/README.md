# Raspberry Pi perception stack

Camera and lidar, fused, with a browser-based calibration dashboard.

Architecture and reasoning: [main README, section 5](../../README.md#the-pi-perception-stack).
Colour tuning procedure: [main README, section 8](../../README.md#pillar-colour-on-the-pi).

## Files

| File | What it is |
|---|---|
| `worldstate.py` | the lock-guarded seam between threads; unit and sign conventions are fixed here |
| `sensors/camera.py` | Picamera2 capture plus detection functions shared by robot and dashboard |
| `sensors/lidar.py` | seals the `rplidarc1` asyncio world inside one thread |
| `main.py` | fusion loop — attaches a lidar range to each camera bearing |
| `dashboard.py` | Flask calibration dashboard, MJPEG streams, live sliders |
| `templates/index.html` | the dashboard page |
| `config.json` | tuning values, written by the dashboard, read by the robot |
| `robodash.service` | systemd unit for the dashboard |

## Setup

Raspberry Pi OS Bookworm 64-bit.

```bash
python3 -m venv ~/robo-env
source ~/robo-env/bin/activate
pip install -r requirements.txt
```

`picamera2` comes from the system packages, not pip — create the venv with
`--system-site-packages` if you want it visible inside.

Check the lidar is where the code expects it (`/dev/ttyUSB0` at 460800 in
`sensors/lidar.py`):

```bash
ls -l /dev/ttyUSB*
```

## Run

```bash
python3 main.py         # robot: camera + lidar + fusion
python3 dashboard.py    # calibration dashboard on :5000
```

**Only one of these at a time.** Both want the camera. `robodash.service`
declares `Conflicts=robot.service` so systemd enforces it.

## Install the dashboard as a service

Edit the paths and `User=` in `robodash.service` to match your Pi, then:

```bash
sudo cp robodash.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now robodash
journalctl -u robodash -f
```

## Conventions

Fixed in `worldstate.py`, enforced everywhere:

- distances in **mm**, angles in **degrees**
- lidar frame: **0 degrees is robot forward, increasing counter-clockwise**.
  If your mount differs, set `MOUNT_OFFSET_DEG` in `sensors/lidar.py`
- "no return at this angle" is `float('inf')`, never `None`

## Competition settings

- Wi-Fi and Bluetooth disabled at boot in `config.txt`, per rule 11.10
- Run the robot process, not the dashboard
- The dashboard's MJPEG streams cost real CPU; they are for the pit only

## Open items

- The fusion loop prints to the console. It does not yet send the
  `V,<colour>,<dx>,<area>` frame to the STM32.
- `config.json` and `dashboard.py` work in **HSV**. The click-to-sample
  threshold tool we actually use works in **Lab** and is not committed here yet.
- `MOUNT_OFFSET_DEG` is 0 and has not been verified against the physical mount.
- `hfov_deg` is 62.0, which is a standard Pi camera figure, not a fisheye one.
  Confirm which lens is fitted.
