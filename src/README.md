# src - control software

Everything that runs on the car, plus the bench tools we used to tune it and the early
simulations that informed the mechanical design.

```
src/
  open_round/       STM32 firmware for the Open Challenge (navigation only)
  obstacle_round/   STM32 firmware for the Obstacle Challenge (navigation + vision + avoidance)
  vision/           Raspberry Pi OpenCV node - pillar colour + offset over UART
  tools/            standalone bench sketches used to tune and verify subsystems
  sim/              early steering-study simulations (historical analysis, not on the car)
```

## What runs where

| Round | On the STM32 | On the Pi |
|---|---|---|
| Open | `open_round/OpenRound.cpp` | - (Pi physically unplugged) |
| Obstacle | `obstacle_round/ObstacleRound.cpp` | `vision/main.py` |

The two firmware programs share the same navigation core - heading-hold on the straights,
gyro-terminated 90 degree turns, learned lane correction. The obstacle firmware adds the UART
vision link, the offset-based pillar avoidance, and the front-proximity failsafe.

## Toolchain

- STM32 firmware: Arduino core for STM32 (STM32duino). Libraries:
  `SparkFun_BNO08x_Arduino_Library` (IMU), `Adafruit_TCS34725` (floor colour), Pololu
  `VL53L1X` (distance), and the built-in `Servo`, `SPI`, and `Wire`. The encoder uses the
  STM32 HAL timer directly (TIM3).
- Pi vision: Python 3 + OpenCV. See `vision/README.md`.

## Firmware <-> Pi protocol

| Direction | Frame | Meaning |
|---|---|---|
| Pi -> STM32 | `V,<colour>,<dx>,<area>` | colour in {R,G,N}; dx = pillar offset (px, +right); area = blob size |
| STM32 -> Pi | `# st=.. av=.. cw=.. sv=.. hd=.. F=.. L=.. R=.. fl=.. vis=.. dx=.. a=.. cn=..` | telemetry |

Link is UART, 115200 8N1, full-duplex, on the STM32 PA9/PA10 (USART1).
