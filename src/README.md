# src

Everything that runs on the car, plus the tools used to tune it.

| Folder | What it is |
|---|---|
| [`open_round/`](open_round/) | STM32 firmware for the Open Challenge |
| [`obstacle_round/`](obstacle_round/) | STM32 firmware for the Obstacle Challenge |
| [`pi/`](pi/) | Raspberry Pi perception — camera, lidar, fusion, calibration dashboard |
| [`tools/calibration/`](tools/calibration/) | Eight calibration sketches plus a menu-driven suite |
| [`tools/bench/`](tools/bench/) | Single-subsystem bench sketches |
| [`sim/`](sim/) | Steering, parking and layout simulations that informed the mechanical design |

## What runs where

| Round | On the STM32 | On the Pi |
|---|---|---|
| Open | `open_round/OpenRound.cpp` | nothing — the Pi is physically disconnected |
| Obstacle | `obstacle_round/ObstacleRound.cpp` | `pi/main.py` |

Both firmware programs share the same navigation core: heading hold on the
straights, gyro-terminated 90 degree turns, and the lane-gap lateral
correction. The obstacle firmware adds the UART vision link, the offset-based
pillar avoidance and the front-proximity failsafe. The open firmware adds a
wall-recovery state.

> **Pin maps differ.** `OpenRound.cpp` (updated 2026-09-15/17) targets the
> re-pinned carrier board; `ObstacleRound.cpp` and `tools/calibration/` still use
> the earlier map. See the pin-map table in the
> [main README](../README.md#pin-maps).

## Firmware and Pi protocol

| Direction | Frame | Meaning |
|---|---|---|
| Pi to STM32 | `V,<colour>,<dx>,<area>` | colour in {R,G,N}; dx = pillar offset in px, + is right; area = blob size. **Sender not written yet** on the Pi side |
| STM32 to Pi | `# st=.. av=.. cw=.. sv=.. hd=.. F=.. L=.. R=.. fl=.. vis=.. dx=.. a=.. cn=..` | telemetry |

UART, 115200 8N1. In `ObstacleRound.cpp` the link is `Serial` (`#define PiSerial Serial`). No checksum is used yet.

## Toolchain

STM32: Arduino core for STM32 (STM32duino), board *BlackPill F411CE*. Libraries —
`SparkFun_BNO08x_Arduino_Library`, `Adafruit_TCS34725`, Pololu `VL53L0X`
(open round) or Pololu `VL53L1X` (obstacle round), plus built-in `Servo`, `SPI`
and `Wire`. The encoder uses a hardware timer directly (TIM5 in the open
round, TIM3 in the obstacle round and calibration sketches). The `.cpp` files
are Arduino sketches: copy one into a folder of the same name as `.ino` to open
it in the Arduino IDE.

Pi: Python 3 with OpenCV and Picamera2. See [`pi/README.md`](pi/README.md).

---

Full explanation of the control logic is in
[`docs/control_architecture.md`](../docs/control_architecture.md); the
calibration procedure is in [`docs/calibration.md`](../docs/calibration.md).
