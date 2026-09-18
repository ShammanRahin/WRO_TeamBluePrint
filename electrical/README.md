# electrical

The wiring, power distribution, and board design of the car. The connection diagram lives in
[`../schemes/`](../schemes/) (currently out of date — see its README); this file explains the
reasoning a diagram cannot. The full design record is [`ELECTRICAL.md`](ELECTRICAL.md), fab
rules are in [`DESIGN_RULES.md`](DESIGN_RULES.md), and the ToF collimator solver is
[`collimator.py`](collimator.py).

## Two stacked single-sided PCBs

- No plated through-holes -> no ground plane at any layer count.
- Coarse minimum trace/space -> you often cannot route between two adjacent header pins, so
  placement (not routing) is the whole problem.
- One-sided pads lift under cable strain, so high current never crosses the PCB - battery,
  motor and servo power are wired point-to-point through screw terminals.

| Board | Carries |
|---|---|
| Upper | STM32, BTS7960 motor driver, power rails, servo header, encoder, Pi UART, IMU |
| Lower | The TCA9548A multiplexer and its sensor channels, at 3.3 V |

## Power - four domains, one star ground

| Rail | Feeds | Why separate |
|---|---|---|
| Motor | BTS7960 -> 25GA -> 5:1 -> solid rear axle | Motor noise/brown-out stays off logic |
| 6.0 V | Steering servo only (JX PS-1171MG) | Servo current spikes correlate with steering; isolate them |
| 5.0 V | STM32 logic + sensor bus (local 3.3 V regulator) | Clean supply for the MCU and sensors |
| 5.1 V, 5 A | Raspberry Pi 5 + RPLIDAR C1 | Obstacle round only; unplugged for the open round |

Bulk capacitance on the motor rail, local decoupling at every IC, ceramics across the motor
terminals. **No inline fuse** — an accepted risk, recorded in Decision #26. One master switch
powers the car on; a separate momentary button starts the program.

## Pin map (STM32F411)

The carrier was re-pinned in September 2026. Only `src/open_round/OpenRound.cpp` uses the new
map so far; `ObstacleRound.cpp`, `tools/calibration/hardware_config.h` and the older bench
testers still use the earlier one.

| Function | Current carrier (`OpenRound.cpp`) | Earlier map (`ObstacleRound.cpp`) |
|---|---|---|
| Motor (BTS7960) | PA2 RPWM (fwd), PA3 LPWM (rev); EN tied high on board | PB9 RPWM, PB8 LPWM, PB1 EN |
| Servo | PA8, 1000-2000 us | PA8, 1000-2000 us |
| Start button | PB15 reserved, **not read yet** | PA5, INPUT_PULLUP, active-low |
| Encoder | TIM5 on PA0 / PA1 | TIM3 on PA6 / PA7 |
| IMU (BNO085, SPI) | PA5 SCK, PA6 MISO, PA7 MOSI, PA4 CS, PB0 INT, PB1 RST | PB3 SCK, PB4 MISO, PB5 MOSI, PB0 CS, PB13 INT, PB14 RST |
| I2C (to mux) | PB6 SCL, PB7 SDA, PB8 mux reset; TCA9548A 0x70, 400 kHz | PB6 SCL, PB7 SDA; TCA9548A 0x70, 400 kHz |
| ToF | VL53L0X front on mux ch 3 | VL53L1X left / right / front on mux ch 1 / 3 / 4 |
| Floor colour | TCS34725 on mux ch 4 | TCS34725, channel auto-detected |
| Status LEDs | PB12, PB13, PB14 | — |
| Pi link | — (not used in the open round) | `Serial`, 115200 8N1 |

Note that on the current map PA0 (encoder) is also the Black Pill's KEY button — don't press it
while running.

## Wiring rules

- No Dupont jumpers, no breadboard. Soldered or crimped into JST-XH / screw terminals with
  strain relief.
- Twist the encoder A/B leads, route them away from the motor leads, enable pull-ups.
- Threadlocker on every fastener that carries a wire or a board standoff.
