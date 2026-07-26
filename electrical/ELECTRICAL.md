# ELECTRICAL — WRO Future Engineers 2026, Team Blueprint

**As built, 2026-07-26.** Block diagram: `schemes/wiring_block_diagram.png`.
Companion solver: `electrical/collimator.py`.

Rule references are to `WRO2026FutureEngineersSelfDrivingCarsGeneralRules.pdf`.

---

## 1. Rule constraints that shape the wiring

| Rule | Text | Consequence |
|---|---|---|
| 9.6 | Vehicle is placed in the starting zone **totally switched off** | Gyro bias can be auto-zeroed at power-on: the car is guaranteed stationary |
| 9.10 | "Only one switch is allowed to switch the vehicle on" | **One master switch gates every rail.** Appendix D's "two batteries, two switches" suggestion is subordinate to this. |
| 9.11 | Vehicle then waits for a **Start button**; only one is allowed | Separate momentary → GPIO. Not a power path. |
| 9.9 | No sensor calibration or switch-config data entry at the line | Any calibration must be firmware-automatic, not team-operated |
| 11.10 | No RF/BT/WiFi during rounds; judges may inspect code and vehicle | Pi WiFi **and** BT disabled in `config.txt`; disabling BT also frees the good PL011 UART |
| 11.13 | Max two drive motors, mechanically coupled, never independent | One motor, one driver channel, solid rear axle via 5:1 — trivially compliant |
| 13.2 / 13.3 / 13.4 / 13.6 | Mat **white**; walls **100 mm tall and black** on every visible face | Drives the entire ToF collimator design (§6) |

---

## 2. Power architecture — four domains, one star ground

```
   XT30      10 A fuse    MASTER SWITCH (rule 9.10 — exactly one)
[3S LiPo]--+--[FUSE]--------[SPST 10 A]--+--> DOMAIN M : BTS7960 -> 25GA -> 5:1 -> rear axle
           |                             |
      (star GND)                         +--> BEC-S  6.0 V / 3 A --> MG996R servo ONLY
           |                             |
           |                             +--> BEC-L  5.0 V / 2 A --> STM32F411
           |                             |                           5x VL53L0X
           |                             |                           TCS34725
           |                             |                           MPU9250
           |                             |
           |                             +--> BEC-C  5.1 V / 3 A --> Pi 4B + fisheye
           |                                        [OPEN ROUND: HARNESS REMOVED]
           +---- all BEC grounds + battery negative meet at ONE M3 brass standoff
```

**Why four and not one.** The failure being engineered out is *motor or servo transient →
MCU brown-out → random reset*, which the rubric names explicitly as "failure point
considerations". MG996R stall is a ~2.5 A step in under 5 ms; Pi 4B boot inrush is
comparable. Either will sag a shared 5 V rail below the STM32 regulator's dropout.
Separating them costs ~30 g.

**The Open-round determinism claim.** Domain C is a physically separate harness. For the
Open Challenge the Pi and BEC-C are unplugged. That is an *inspectable* statement, not a
software flag: the Open configuration cannot be influenced by the Pi because the Pi is
not electrically present.

### Star-ground rules (non-negotiable)
1. BTS7960 ground returns to the star on its own **≥1.0 mm² (18 AWG)** leg. It never
   passes through logic ground.
2. Each sensor harness carries its own return to the board. No sensor is grounded to the
   nearest chassis screw — that creates a ground loop through the frame.
3. The star point is a **soldered brass standoff**, not a terminal block: one mechanical
   joint, located by geometry, per Decision #10.

### Decoupling and EMI
| Location | Part | Purpose |
|---|---|---|
| BTS7960 VBAT input | 1000 µF 25 V low-ESR + 0.1 µF X7R | Commutation transients |
| Motor, terminal-to-terminal | 0.1 µF ceramic 100 V | Brush arc suppression |
| Motor, each terminal to can | 0.1 µF ceramic ×2 | Common-mode path |
| BEC-S output, **at the servo connector** | 470 µF + 0.1 µF | MG996R step load — not on the mainboard |
| Every IC VCC pin | 0.1 µF within 5 mm | Standard |
| STM32 VDDA | ferrite bead + 1 µF + 0.1 µF | ADC noise floor |

---

## 3. Power budget

| Domain | Load | Typ | Peak | Note |
|---|---|---|---|---|
| M | 25GA through BTS7960 | ~0.6 A | 🚩 **stall not measured** | Blocks nothing (BTS7960 is 43 A) but belongs in the journal |
| S | MG996R | 0.5–0.9 A | ~2.5 A stall | BEC-S 3 A cont / 5 A peak |
| L | STM32 40 mA · 5× VL53L0X 100 mA · TCS34725 3 mA · MPU9250 4 mA | **~150 mA** | ~200 mA | 2 A BEC = 10× margin |
| C | Pi 4B 1 GB + fisheye | 0.85 A | ~1.25 A | 3 A BEC |

**Pi 4B supply detail:** set BEC-C to **5.15 V measured at the GPIO header under load**,
feed pins 2/4 (5 V) and 6 (GND) with 18 AWG. Never exceed 5.25 V — feeding the header
bypasses the USB-C input protection. Undervoltage causes silent CPU throttling, which
looks like a vision bug.

**Battery:** 3S LiPo. Rounds are 3 minutes (§9.1–9.2), so even at 3 A continuous a round
costs 150 mAh. An 850 mAh pack (~75 g) covers a full practice day with swaps.

---

## 4. Bus architecture

```
STM32F411 --I2C1 (3.3 V, 2.2k)--> 5x VL53L0X   @ 0x30 0x31 0x32 0x33 0x34
                                  + 5 XSHUT lines (open-drain)

STM32F411 --I2C2 (3.3 V, 2.2k)--> TCS34725     @ 0x29   [ALONE ON THIS BUS]

STM32F411 --SPI1 @ 8 MHz-------->  MPU9250     gyro-Z only, magnetometer DISABLED

STM32F411 --USART1 -------------->  Pi 4B      checksummed frames, obstacle round only
```

### Why the buses are split
**The VL53L0X and the TCS34725 share factory address 0x29**, and the TCS has neither a
shutdown pin nor an address strap. During ToF enumeration each sensor briefly appears at
0x29, so every enumeration write would also land on the colour sensor. Splitting the
buses removes the collision outright.

It also buys fault isolation: the TCS is the **turn trigger**, the single most critical
sensor in the Open round. Putting it on its own bus means no ToF cable fault can take it
down. Moving the MPU9250 to SPI (it is SPI-capable) leaves the TCS as the sole device on
I²C2 and removes the IMU from any I²C lockup path entirely.

### XSHUT — electrical detail that will bite if ignored
The VL53L0X die is **2.8 V logic**. Do **not** drive XSHUT push-pull from a 3.3 V GPIO.

Configure each XSHUT pin as **open-drain output with no internal pull-up**:
pull low = shutdown; release to Hi-Z and the breakout's own pull-up sets the rail. Safe
on 2.8 V-native boards and 3.3 V level-shifted boards alike.

🚩 **Identify the exact breakout before soldering.** Boards with an onboard regulator and
level shifters are safe at 3.3 V; bare 2.8 V-native boards are not, and pulling I²C up to
3.3 V will forward-bias their ESD diodes.

### Enumeration sequence (re-runs on every reset)
Reassigned addresses are **volatile** and are lost on any brownout.

1. All XSHUT low → wait 10 ms
2. Release XSHUT[0] → wait ≥2 ms → write address 0x30
3. Repeat for 0x31 … 0x34
4. Verify all five respond. **If one fails, skip it and continue** — a dead sensor must
   never stall enumeration
5. Report the result on the status LED **before** entering `ARMED`

---

## 5. Pin map — STM32F411CEU6 "Black Pill", 26 of 28 usable pins

The 48-pin package breaks out 34 pins; PA11/PA12 are on-board USB, PA13/PA14 are SWD,
PC14/PC15 carry the LSE crystal → **28 usable**.

Conflicts resolved: SPI1 remapped to PB3/PB4/PB5 (AF5) so TIM3 keeps PA6/PA7 for encoder
mode; **PB11 avoided — it does not exist on the 48-pin F411** (it is VCAP_1).

| Pin | Peripheral | Signal | Detail |
|---|---|---|---|
| PB6 / PB7 | I2C1 (AF4) | ToF SCL / SDA | 2.2 kΩ pull-ups |
| PB10 / PB9 | I2C2 (AF4) | TCS34725 SCL / SDA | 2.2 kΩ pull-ups, sole device |
| PB12, PB13, PB14, PB15, PB8 | GPIO | XSHUT 0–4 | **open-drain, no pull-up** |
| PB3 / PB4 / PB5 | SPI1 (AF5) | SCK / MISO / MOSI | MPU9250 |
| PB0 | GPIO | IMU_CS | |
| PA6 / PA7 | TIM3_CH1 / CH2 | ENC_A / ENC_B | 5 V tolerant · 10 kΩ pull-ups · 100 Ω + 1 nF |
| PA8 | TIM1_CH1 | SERVO_PWM | 1 µs tick, ARR = 19999 → **50 Hz** |
| PA0 / PA1 | TIM2_CH1 / CH2 | BTS7960 RPWM / LPWM | ≤20 kHz |
| PB1 | GPIO | BTS7960 R_EN + L_EN tied | **low at boot = motor disabled** |
| PA9 / PA10 | USART1 | → Pi TX / ← Pi RX | 220 Ω series each |
| PA2 / PA3 | USART2 | Debug TX / RX | **physically unplugged for runs** |
| PA5 | GPIO / EXTI | START_BTN | 10 kΩ pull-up + 1 kΩ + 100 nF (rule 9.11) |
| PA4 | ADC1_IN4 | VBAT_SENSE | 10 kΩ / 2.2 kΩ divider |
| PC13 | GPIO | STATUS_LED | on-board |
| PA15, PB2 | — | **spare ×2** | PB2 is BOOT1 — output only |

### Encoder noise numbers
Signal frequency at full speed is **3995 Hz** (the encoder is upstream of the 5:1). A
100 Ω + 1 nF RC gives f_c ≈ 1.6 MHz — about 400× above signal, so it kills commutation
spikes without touching the waveform. Twist A/B together, twist VCC/GND together, keep
the pair **≥25 mm from motor leads** and cross at 90° where crossing is unavoidable.

### Servo
TIM1 prescaled to a 1 µs tick, ARR = 19999 → CCR is literally the pulse width in
microseconds. **50 Hz is a ceiling, not a choice: the MG996R is analog.** That sets a
hard 20 ms floor on steering command latency. 220 Ω series on the signal line.

An analog servo inside a closed loop hunts across its ~5 µs deadband — current draw,
heat, audible buzz, steering jitter. Quantise output to ≥6 µs steps and hold inside a
±1.5° error window.

### IMU
Gyro-Z only. DLPF ≈ 41 Hz. Polled on a **fixed 1 kHz timer tick**, not on an interrupt
line — a constant integration timestep removes a whole class of drift error.
**The magnetometer is not used**: a BTS7960 switching amps on a 115 mm chassis will
corrupt it. Bias is auto-zeroed at power-on while stationary, which rule 9.6 guarantees.

🚩 Verify `WHO_AM_I` (0x75): 0x71 = genuine MPU9250, 0x70 = MPU6500, 0x73 = MPU9255.
Relabelled parts are common. Any of the three is fine for gyro-Z, but the driver must not
block waiting for a magnetometer that is not there.

---

## 6. ToF mounting — the collimator is a structural part, not an accessory

The VL53L0X has a **fixed ~25° FoV and no programmable ROI**. Combined with a white mat
and black walls, the false target reflects better than the true one.

`d_floor = h / tan(θ_eff)` where `θ_eff = θ_half + rake − wedge`
Side wall distance = (1000 − 115) / 2 = **442.5 mm**

| Configuration | θ_eff | d_floor @ h = 40 mm | Margin |
|---|---|---|---|
| Bare VL53L0X | 13.56° | 166 mm | −277 mm ✗ |
| Snout only | 4.63° | 494 mm | +51 mm — too thin |
| **Snout + 2° wedge** | **2.63°** | **870 mm** | **+428 mm ✓** |

**Spec: printed snout, slot 2.5 mm tall × 10 mm wide × 20 mm deep, plus a +2° upward
mounting wedge.**

- The slot must be **wide, not round** — emitter and receiver apertures sit ~2.5 mm apart
  and a round hole vignettes the receiver.
- The wedge exists because front wheels are 46 mm and rear are 50 mm, giving
  **1.0–1.1° of nose-down rake** that every chassis-mounted sensor inherits, aimed at the
  floor.
- Beam axis at 442.5 mm is then 47 mm high — far under the 100 mm wall top, so nothing is
  lost.
- **Re-run offset calibration after fitting the snout** — the hood changes crosstalk.
- Margin improves as the car grows: at a 90 mm car height, d_floor is 1740 mm.

Firmware layer on top: reject on `RangeStatus != 0`, low signal rate, or high sigma;
median-of-3.

🚩 **Highest-risk open item in the build.** Bench-test signal rate against **black MDF**
with the snout fitted, at 442 mm (90° pair) and at 300/700 mm at 60° incidence (30° pair),
**before printing five mounts.** A 2.5 × 10 mm aperture throws away photons and the wall
is already a poor NIR target. If marginal, order the Sharp GP2Y0A21 fallback immediately —
its lead time is still unrecorded (`BOM.md`, Open Question #3).

---

## 7. Harness and connector standard

**Rule: no two harnesses share both connector family and pin count.** Where two sensors
must never be swapped, put them on **one connector with a Y-split** — cross-plugging then
becomes physically impossible.

| Harness | Family | Pins | Pinout |
|---|---|---|---|
| Battery | XT30 | 2 | +, − |
| Motor out | Screw terminal | 2 | M+, M− |
| Servo | JST-XH | 3 | 6 V, SIG, GND |
| Encoder | JST-XH | 4 | 5 V, A, B, GND |
| **ToF ×5 (Y-split)** | JST-XH | **8** | 3V3, GND, SCL, SDA, XSHUT ×… (per-sensor tails) |
| TCS34725 | JST-PH | 4 | 3V3, SCL, SDA, GND |
| MPU9250 | JST-XH | 6 | 3V3, GND, SCK, MISO, MOSI, CS |
| Pi link | JST-PH | 3 | TX, RX, GND |

Every harness: crimped (not soldered into the housing), **heatshrink strain relief at both
ends**, a service loop, and a printed label. **No Dupont anywhere on the vehicle**
(Decision #11).

---

## 8. Loop rates

| Loop | Rate | Bound by |
|---|---|---|
| IMU gyro-Z + heading integrate | 1000 Hz | SPI, fixed timer tick |
| Encoder | 1000 Hz | free — TIM3 hardware |
| Drive speed PID | 200 Hz | **mandatory** — see Decision #22 |
| **Servo command** | **50 Hz** | **MG996R analog — the steering bottleneck** |
| VL53L0X ×5 | 30–50 Hz | sensor |
| TCS34725 | ≥100 Hz | 2.4 ms integration |
| Pi colour frame | 15–30 Hz | advisory only — never gates a control loop |

At 0.70 m/s the car travels 14 mm per servo frame and 20 mm per TCS-line dwell. Both are
comfortable — this is the payoff from the 5:1 reduction.

---

## 9. Bring-up order — do not skip

1. PDB alone, no loads. Verify all rails at 9 V, 11.1 V and 12.6 V input.
2. Master switch + fuse under a 5 A dummy load. Confirm no arcing or heating.
3. STM32 + SWD only. Blink PC13.
4. One subsystem at a time, **motor disconnected**: encoder → MPU9250 → TCS34725 →
   ToF enumeration → servo.
5. Motor connected, **wheels off the ground**, 20% duty. Watch every sensor's noise
   floor. **If any reading moves when the motor spins, stop and fix the grounding** — do
   not filter it out in firmware.
6. Only then: on the ground, `CENTER_US` bisection sweep.

**Test points to expose on the mainboard:** VBAT, 6 V, 5 V, 3V3, AGND, star GND, ENC_A,
SERVO_PWM. Eight pads, ten minutes of layout, hours of probing saved.

---

## 10. Open electrical items

| # | Item | Blocks |
|---|---|---|
| 1 | 25GA stall current — measure with a clamp meter, wheels locked | Journal power budget only (BTS7960 has huge headroom) |
| 2 | VL53L0X breakout: regulator + level shifters present? | I²C pull-up rail — a wrong guess damages all five |
| 3 | MPU9250 `WHO_AM_I` value | IMU driver |
| 4 | Encoder counts per motor rev (hand-rotate) | Odometry constant |
| 5 | 3S pack capacity and C rating | Mass budget |
