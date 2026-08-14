# electrical

The wiring, power distribution, and board design of the car. The connection diagram lives in
`../schemes/`; this file explains the reasoning a diagram cannot.

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
| 6.0 V | Steering servo only | MG996R current spikes correlate with steering; isolate them |
| 5.0 V | STM32 logic + sensor bus (local 3.3 V regulator) | Clean supply for the MCU and sensors |
| 5.1 V | Raspberry Pi 4B | Obstacle round only; unplugged for the open round |

Bulk capacitance on the motor rail, local decoupling at every IC, ceramics across the motor
terminals. Inline fuse on battery positive, upstream of everything. One master switch powers
the car on; a separate momentary button (to PA5) starts the program.

## Pin map (STM32F411)

| Function | Pin(s) | Notes |
|---|---|---|
| Motor (BTS7960) | PB9 RPWM, PB8 LPWM, PB1 EN | Speed sign selects direction |
| Servo | PA8 | 1000-2000 us |
| Start button | PA5 | INPUT_PULLUP, active-low |
| Encoder (TIM3) | PA6, PA7 | Hardware quadrature |
| IMU (SPI) | PB5 MOSI, PB4 MISO, PB3 SCK, PB0 CS, PB13 INT, PB14 RST | BNO085, dedicated bus |
| I2C (to mux) | PB7 SDA, PB6 SCL | TCA9548A 0x70, 400 kHz |
| ToF left/right/front | mux ch 1/3/4 | VL53L1X, all 0x29 |
| Floor colour | mux (auto-detected) | TCS34725 0x29 |
| Pi UART | PA9 TX, PA10 RX | 115200 8N1 |

## Wiring rules

- No Dupont jumpers, no breadboard. Soldered or crimped into JST-XH / screw terminals with
  strain relief.
- Twist the encoder A/B leads, route them away from the motor leads, enable pull-ups.
- Threadlocker on every fastener that carries a wire or a board standoff.
