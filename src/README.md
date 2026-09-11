# src

Everything that runs on the car, plus the tools used to tune it.

| Folder | What it is |
|---|---|
| [`open_round/`](open_round/) | STM32 firmware for the Open Challenge |
| [`obstacle_round/`](obstacle_round/) | STM32 firmware for the Obstacle Challenge |
| [`pi/`](pi/) | Raspberry Pi perception — camera, lidar, fusion, calibration dashboard |
| [`tools/calibration/`](tools/calibration/) | Eight calibration sketches plus a menu-driven suite |
| [`tools/bench/`](tools/bench/) | Single-subsystem bench sketches |
| [`sim/`](sim/) | Steering and parking simulations that informed the mechanical design |

## What runs where

| Round | On the STM32 | On the Pi |
|---|---|---|
| Open | `open_round/OpenRound.cpp` | nothing — the Pi is physically disconnected |
| Obstacle | `obstacle_round/ObstacleRound.cpp` | `pi/main.py` |

Both firmware programs share the same navigation core: heading hold on the
straights, gyro-terminated 90 degree turns, and the lane-gap lateral
correction. The obstacle firmware adds the UART vision link, the offset-based
pillar avoidance and the front-proximity failsafe.

## Firmware and Pi protocol

| Direction | Frame | Meaning |
|---|---|---|
| Pi to STM32 | `V,<colour>,<dx>,<area>` | colour in {R,G,N}; dx = pillar offset in px, + is right; area = blob size |
| STM32 to Pi | `# st=.. av=.. cw=.. sv=.. hd=.. F=.. L=.. R=.. fl=.. vis=.. dx=.. a=.. cn=..` | telemetry |

UART, 115200 8N1, full duplex, on STM32 PA9/PA10 (USART1).

## Toolchain

STM32: Arduino core for STM32 (STM32duino). Libraries —
`SparkFun_BNO08x_Arduino_Library`, Pololu `VL53L1X`, `Adafruit_TCS34725`, plus
built-in `Servo`, `SPI` and `Wire`. The encoder uses the HAL timer directly
(TIM3).

Pi: Python 3 with OpenCV and Picamera2. See [`pi/README.md`](pi/README.md).

---

Full explanation of the control logic, the calibration procedure and the build
is in the [main README](../README.md#5-how-the-car-thinks).
