# ELECTRICAL — WRO Future Engineers 2026, Team Blueprint

**Revised 2026-07-28.** Supersedes the 2026-07-26 revision.
Block diagram: `schemes/wiring_block_diagram.png` · Solvers: `electrical/collimator.py`,
`src/sim/geometry_sweep.py` · Fab constraints: `electrical/DESIGN_RULES.md`

Rule references are to `WRO2026FutureEngineersSelfDrivingCarsGeneralRules.pdf`.

### What changed on 2026-07-28 and why

| Change | Was | Now | Decision |
|---|---|---|---|
| I²C topology | 5× ToF on I2C1 with XSHUT reassignment to 0x30–0x34; TCS alone on I2C2 | **PCA9548A mux**, one sensor per channel, all at default 0x29 | #23 |
| IMU | MPU9250 on SPI1 | **MPU6050 on mux ch7 for bench only**; competition IMU is SPI on the mainboard | #24 |
| Physical build | One mainboard | **Two stacked single-sided boards** | #25 |
| Fuse | 10 A inline | **Omitted — accepted risk** | #26 |
| Connectors | JST-XH and JST-PH | **JST-XH only** — PH is not manufacturable at our fab | #27 |
| Steering lock | ±35° | **±40°** | #28 |

The XSHUT enumeration sequence, its 2.8 V open-drain handling, and the volatile-address
brownout problem are all **deleted, not moved** — the mux removes the entire failure mode.

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
| 13.2 / 13.3 / 13.4 / 13.6 | Mat **white**; walls **100 mm tall and black** on every visible face | Drives the entire ToF collimator design (§7) |

---

## 2. Power architecture — four domains, one star ground

```
   [connector]      MASTER SWITCH (rule 9.10 — exactly one)
[3S LiPo 75C]--+------[SPST 10 A]--+--> DOMAIN M : BTS7960 -> 25GA -> 5:1 -> rear axle
               |    (NO FUSE, #26) |
          (star GND)               +--> BEC-S  6.0 V / 3 A --> MG996R servo ONLY
               |                   |
               |                   +--> BEC-L  5.0 V / 2 A --> Board A logic
               |                   |                           Board B (regulates to 3V3 locally)
               |                   |
               |                   +--> BEC-C  5.1 V / 3 A --> Pi 4B + fisheye
               |                              [OPEN ROUND: HARNESS REMOVED]
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

🚩 **No fuse (Decision #26).** Recorded as an accepted risk, not an oversight. A 75C 3S
pack will deliver several hundred amps into a short with no current limiting, and
hand-wired single-sided assembly is where shorts happen. Revisit if any wiring is reworked
in the field.

🚩 **Battery connector unresolved.** JST-XH is rated ~3 A per contact against a peak
system draw near 8 A, and a 3S balance lead is thinner still. **XT30 (30 A) is the
specified part** until decided otherwise.

### Star-ground rules (non-negotiable)
1. BTS7960 ground returns to the star on its own **≥1.0 mm² (18 AWG)** leg. It never
   passes through logic ground.
2. Each sensor harness carries its own return to the board. No sensor is grounded to the
   nearest chassis screw — that creates a ground loop through the frame.
3. The star point is a **soldered brass standoff**, not a terminal block: one mechanical
   joint, located by geometry, per Decision #10.
4. **Board B grounds to the star through the inter-board cable only.** It gets no
   independent path to the battery.

### Decoupling and EMI
| Location | Part | Purpose |
|---|---|---|
| BTS7960 VBAT input | 1000 µF 25 V low-ESR + 0.1 µF X7R | Commutation transients |
| Motor, terminal-to-terminal | 0.1 µF ceramic 100 V | Brush arc suppression |
| Motor, each terminal to can | 0.1 µF ceramic ×2 | Common-mode path |
| BEC-S output, **at the servo connector** | 470 µF + 0.1 µF | MG996R step load — not on the mainboard |
| Every IC VCC pin | 0.1 µF within 5 mm | Standard |
| STM32 VDDA | ferrite bead + 1 µF + 0.1 µF | ADC noise floor |
| Board B 3V3 regulator | 10 µF in, 22 µF out | AMS1117 loop stability needs ≥22 µF |

---

## 3. Power budget

| Domain | Load | Typ | Peak | Note |
|---|---|---|---|---|
| M | 25GA through BTS7960 | ~0.6 A | 🚩 **stall not measured** | Blocks nothing (BTS7960 is 43 A) but belongs in the journal |
| S | MG996R | 0.5–0.9 A | ~2.5 A stall | BEC-S 3 A cont / 5 A peak |
| L | STM32 40 mA · 5–6× VL53L0X 120 mA · TCS34725 3 mA · MPU6050 4 mA · PCA9548A 1 mA | **~170 mA** | ~250 mA | 2 A BEC = 8× margin |
| C | Pi 4B 1 GB + fisheye | 0.85 A | ~1.25 A | 3 A BEC |

**Board B local regulation:** BEC-L 5 V crosses the inter-board cable and is dropped to
3.3 V on Board B by an **AMS1117-3.3**. Dissipation is (5 − 3.3) × 0.25 ≈ **0.43 W**, so
give the tab ≥400 mm² of copper pour. Do **not** feed Board B from the Black Pill's onboard
3.3 V regulator — 250 mA of sensor current sharing the MCU's own rail recreates the
brown-out problem in miniature.

**Pi 4B supply detail:** set BEC-C to **5.15 V measured at the GPIO header under load**,
feed pins 2/4 (5 V) and 6 (GND) with 18 AWG. Never exceed 5.25 V — feeding the header
bypasses the USB-C input protection. Undervoltage causes silent CPU throttling, which
looks like a vision bug.

**Battery:** 3S LiPo 75C. Rounds are 3 minutes (§9.1–9.2), so even at 3 A continuous a
round costs 150 mAh. An 850 mAh pack (~75 g) covers a full practice day with swaps.
🚩 Pack capacity still unrecorded.

---

## 4. Bus architecture — PCA9548A multiplexer

```
                                        ┌── ch0 ── VL53L0X  F
                                        ├── ch1 ── VL53L0X  FL30
STM32F411 --I2C1 (3.3 V, 2.2k)--------> ├── ch2 ── VL53L0X  FR30
   PB6/PB7        + MUX_RST (PB12)      ├── ch3 ── VL53L0X  L90
                  [BOARD B, 6-pin JST]  ├── ch4 ── VL53L0X  R90
                    PCA9548A @ 0x70     ├── ch5 ── (VL53L0X #6, reserved)
                                        ├── ch6 ── TCS34725
                                        └── ch7 ── MPU6050  [BENCH IMU ONLY]

STM32F411 --SPI1 @ 8 MHz (PB3/4/5)----> COMPETITION IMU (BNO08x / ICM-42688)
                                        [mainboard, no mux, no cable]

STM32F411 --USART1 (PA9/PA10)---------> Pi 4B, checksummed frames, obstacle round only
```

### Why the mux replaced XSHUT addressing

The old scheme worked but cost more than it looked. **Every VL53L0X and the TCS34725 share
factory address 0x29**, so the previous design reassigned the ToF sensors to 0x30–0x34 at
boot using five XSHUT lines, and isolated the TCS on a second bus. That carried three
liabilities:

1. **Reassigned addresses are volatile.** Any brownout loses them, so enumeration had to
   re-run on every reset and be tolerant of partial failure.
2. **XSHUT is 2.8 V logic** on a bare L0X die, requiring open-drain handling and a
   different answer depending on which breakout variant you happened to buy.
3. **Five GPIO** spent on address assignment.

With one sensor per mux channel, **no two devices are ever on the bus at the same time**,
so every sensor keeps its default 0x29 and none of the above exists. The five XSHUT pins
and the entire second I²C bus are reclaimed — which is what pays for the SPI competition
IMU and moves the drive PWM off PA0/PA1 (§5).

**Costs accepted, on the record:**
- The mux is now a **single point of failure** for every I²C sensor. `MUX_RST` is wired to
  the STM32 so a hung bus can be recovered without a full reset.
- Each read costs one extra channel-select write, ~25 µs at 400 kHz. Against a 30–50 Hz
  sensor rate this is not measurable.
- The TCS34725 — the turn trigger, and the most critical sensor in the Open round — no
  longer has a private bus. It has a private *channel*, which gives address isolation but
  not fault isolation. **If a ToF cable fault can pull the shared bus down, the turn
  trigger goes with it.** This is the one real regression, and it is why `MUX_RST` is
  mandatory rather than optional.

**Pull-ups: 2.2 kΩ ×2 to 3.3 V, placed on Board B at the mux** — the far end of the cable,
where capacitance is worst. Downstream channels rely on the breakout modules' own pull-ups;
because the mux isolates channels there is no accumulation problem. 🚩 Confirm by measuring
SDA→VCC on one module before trusting it.

### IMU — two parts, two paths (Decision #24)

**The MPU6050 will not race.** It is a bench part and sits on mux channel 7, where latency
does not matter. Decision #1 terminates every corner on measured IMU heading, so the
*racing* gyro must have a private, short, uncontended path — and it does.

| | Bench | Competition |
|---|---|---|
| Part | MPU6050 (I²C, `WHO_AM_I` = 0x68 confirmed) | BNO08x / ICM-42688 |
| Path | mux ch7, across the inter-board cable | **SPI1 on the mainboard** |
| Pins | none of its own | PB3/4/5 + PB0 CS, PB13 INT, PB14 RST |

Gyro-Z only. DLPF ≈ 41 Hz. Polled on a **fixed 1 kHz timer tick**, not on an interrupt
line — a constant integration timestep removes a whole class of drift error.
**No magnetometer**: a BTS7960 switching amps on a 115 mm chassis will corrupt it.
Bias auto-zeroes at power-on while stationary, which rule 9.6 guarantees.

🚩 **The competition IMU is not on hand and not yet ordered.** It is the longest-lead item
left. Contingency: I2C2 (PB10 + a spare) can be brought to the same header, letting the
MPU6050 move off the mux onto a dedicated bus by swapping one cable rather than re-etching
a board.

---

## 5. Pin map — STM32F411CEU6 "Black Pill", 22 of 28 usable

The 48-pin package breaks out 34 pins; PA11/PA12 are on-board USB (the DFU flashing path),
PA13/PA14 are SWD, PC14/PC15 carry the LSE crystal → **28 usable**.

**PB11 does not exist on the 48-pin F411** — pin 22 is `VCAP_1`. Verified against
DS10314 Rev 7 Table 8. (Web searches will tell you otherwise; they are wrong.)

| Pin | Peripheral | AF | Signal | Detail |
|---|---|---|---|---|
| PB6 / PB7 | I2C1 | **AF4** | MUX_SCL / MUX_SDA | To Board B. 2.2 kΩ pull-ups at the mux end |
| PB12 | GPIO | — | MUX_RST | Active low. Bus recovery without a full reset |
| PB3 / PB4 / PB5 | SPI1 | **AF5** | SCK / MISO / MOSI | Competition IMU. **Needs JTAG release — see below** |
| PB0 | GPIO | — | IMU_CS | |
| PB13 | GPIO / EXTI13 | — | IMU_INT | 5 V tolerant (`FT`), unlike PB5 |
| PB14 | GPIO | — | IMU_RST | |
| PA6 / PA7 | TIM3_CH1 / CH2 | **AF2** | ENC_A / ENC_B | Hardware quadrature · 10 kΩ pull-ups · 100 Ω + 1 nF |
| PA8 | TIM1_CH1 | **AF1** | SERVO_PWM | 1 µs tick, ARR = 19999 → **50 Hz** |
| PB8 / PB9 | TIM4_CH3 / CH4 | **AF2** | BTS7960 RPWM / LPWM | **Moved off PA0/PA1 — see below** |
| PB1 | GPIO | — | BTS7960 R_EN + L_EN tied | **low at boot = motor disabled**; 10 kΩ pull-down |
| PA9 / PA10 | USART1 | **AF7** | → Pi TX / ← Pi RX | 220 Ω series each |
| PA2 / PA3 | USART2 | **AF7** | Debug TX / RX | **physically unplugged for runs** |
| PA5 | GPIO / EXTI5 | — | START_BTN | 10 kΩ pull-up + 1 kΩ + 100 nF (rule 9.11) |
| PA4 | ADC1_IN4 | — | VBAT_SENSE | 10 kΩ / 2.2 kΩ divider |
| PC13 | GPIO | — | STATUS_LED | on-board, active low — **see current limit below** |
| PB10, PB15, PA0, PA1, PA15, PB2 | — | — | **spare ×6** | PB2 is BOOT1, output only; PA0 is the KEY button |

### Two corrections to the previous pin map

**Drive PWM moved from PA0/PA1 (TIM2) to PB8/PB9 (TIM4_CH3/CH4).** On the WeAct Black Pill,
**PA0 is wired to the onboard KEY button**, which shorts it to ground when pressed. Driving
PA0 as a push-pull PWM output means anyone pressing KEY shorts a driven output straight to
GND. PB8/PB9 were freed by deleting XSHUT, so this now costs nothing. Both are channels of
the same timer, so the two half-bridge edges stay coherent.

**PB3, PB4 and PA15 default to JTAG after reset** (JTDO-SWO, NJTRST, JTDI). SPI1 sits on
PB3/PB4/PB5, so **the firmware must release JTAG and keep SWD before configuring them**, or
SPI1 silently will not work and it will look like a dead IMU. In CubeMX: **SYS → Debug →
Serial Wire.** Not "Disabled", which also kills SWD.

### Electrical cautions verified against the datasheet

🚩 **PC13 is supplied through the backup power switch, which passes only ~3 mA**
(DS Table 8 note 2). The datasheet states these pins *"must not be used as a current source
(e.g. to drive an LED)."* The Black Pill's LED is active-LOW with its own resistor, so PC13
sinks ~2 mA and works — that is the only reason this is legal. **Do not add load to PC13**,
and keep output speed ≤2 MHz.

🚩 **PB5 is `TC` — 3.3 V only, not 5 V tolerant.** Every other pin in this map is `FT`.
`IMU_INT` was placed on PB13 rather than PB5 for exactly this reason.

### DMA (verified, RM0383 Tables 27–28)

Every peripheral here has DMA, so the ToF bus never blocks the gyro loop:
I2C1 → DMA1 S0/S5 Ch1 (RX), S1 Ch0 (TX) · SPI1 → DMA2 S0/S2 Ch3 (RX), S3/S5 Ch3 (TX) ·
USART1 → DMA2 S2/S5 Ch4 (RX), S7 Ch4 (TX).

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

---

## 6. Physical build — two stacked single-sided boards

Our fab does double-sided **without plated through-holes**, which makes two-layer unusable
and means **no ground plane at any layer count**. Minimum trace and space is 0.5 mm, so a
routing channel is 1.2 mm — roughly four times what a commercial fab needs. The consequence
that drives everything: **no trace can pass between two adjacent header pins** (0.54 mm
available, 1.8 mm required), so every net routes *around* every connector and **placement,
not routing, is the design problem.**

```
   ┌──────────────────────────────────────┐
   │  BOARD A — MAIN  (upper)             │  STM32, BTS7960, BEC rails, servo,
   │                                      │  encoder, Pi UART, competition IMU
   └───────────────┬──────────────────────┘
                   │  6-pin JST-XH, ~40 mm
   ┌───────────────┴──────────────────────┐
   │  BOARD B — SENSOR  (lower)           │  PCA9548A + 8 channels + AMS1117-3.3
   └──────────────────────────────────────┘
```

Splitting also enforces the power-domain separation of Decision #12 physically rather than
only topologically. **High current never crosses PCB copper** — battery, motor and servo
power are wired point-to-point through screw terminals, because on a board with no plated
holes every pad has one-sided adhesion and cable strain lifts pads. Pad lift is an
*intermittent* failure, which is the class this whole vehicle is designed to not have.

### Inter-board cable — 6-pin JST-XH

| Pin | Net | Note |
|---|---|---|
| 1 | 5 V | BEC-L, feeds Board B's AMS1117 |
| 2 | GND | |
| 3 | MUX_SCL | |
| 4 | MUX_RST | Static — acts as a quasi-shield between SCL and SDA |
| 5 | MUX_SDA | |
| 6 | GND | |

Grounds flank the signal group and the static `MUX_RST` separates clock from data. Keep the
cable ≤100 mm (≈40 mm in the stack), twist SCL/SDA, route away from motor leads and the
servo run.

**On inter-board I²C:** Decision #2 bans it, and that ban stands for the **STM32↔Pi** link —
two compute domains, two power domains, a long cable, independent resets. This is a
different case: a passive sensor breakout, single master, one power domain, 40 mm inside a
stack. If it proves unreliable on the mat, **drop to 100 kHz before redesigning anything.**

Full fab rules, drill and pad tables, jumper strategy and the PDF export procedure are in
`electrical/DESIGN_RULES.md`.

---

## 7. ToF mounting — the collimator is a structural part, not an accessory

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
its lead time is still unrecorded.

---

## 8. Harness and connector standard

**JST-XH only, 2.5 mm pitch.** JST-PH is 2.0 mm pitch, which at our fab gives a 0.25 mm
annulus against a 1.0 mm drill on a board with no plated holes — that pad will lift. One
pitch, one crimp tool, and nothing on the vehicle that can be cross-plugged into the wrong
socket.

**Rule: no two harnesses share both connector family and pin count.** Where two sensors
must never be swapped, put them on **one connector with a Y-split** — cross-plugging then
becomes physically impossible.

| Harness | Family | Pins | Pinout |
|---|---|---|---|
| Battery | XT30 | 2 | +, − |
| Motor out | Screw terminal 5.08 | 2 | M+, M− |
| Servo | JST-XH | 3 | 6 V, SIG, GND |
| Encoder | JST-XH | 4 | 5 V, A, B, GND |
| **Board A ↔ Board B** | JST-XH | **6** | 5 V, GND, SCL, RST, SDA, GND |
| ToF ×6 (Board B, one per channel) | JST-XH | 4 | 3V3, GND, SCL, SDA |
| TCS34725 (Board B ch6) | JST-XH | 4 | 3V3, GND, SCL, SDA — **keyed differently from ToF, or Y-split** |
| Competition IMU | JST-XH | 7 | 3V3, GND, SCK, MISO, MOSI, CS, INT |
| Pi link | JST-XH | 4 | GND, TX, RX, GND |

🚩 The ToF and TCS harnesses are now both 4-pin JST-XH with identical pinouts. That
violates the anti-cross-plug rule above. Either key them differently, or accept it on the
grounds that the pinout is genuinely identical so a swap is electrically harmless — it
just puts the sensor on the wrong channel, which enumeration will catch. **Decide and
record.**

Every harness: crimped (not soldered into the housing), **heatshrink strain relief at both
ends**, a service loop, and a printed label. **No Dupont anywhere on the vehicle**
(Decision #11).

---

## 9. Loop rates

| Loop | Rate | Bound by |
|---|---|---|
| IMU gyro-Z + heading integrate | 1000 Hz | SPI, fixed timer tick |
| Encoder | 1000 Hz | free — TIM3 hardware |
| Drive speed PID | 200 Hz | **mandatory** — see Decision #22 |
| **Servo command** | **50 Hz** | **MG996R analog — the steering bottleneck** |
| VL53L0X ×5–6 | 30–50 Hz | sensor + one mux write each |
| TCS34725 | ≥100 Hz | 2.4 ms integration |
| Pi colour frame | 15–30 Hz | advisory only — never gates a control loop |

At 0.70 m/s the car travels 14 mm per servo frame and 20 mm per TCS-line dwell. Both are
comfortable — this is the payoff from the 5:1 reduction.

**Mux budget check:** at 50 Hz across 7 active channels, channel-select writes cost
7 × 50 × 25 µs ≈ **8.75 ms/s, under 1% of bus time.** Not a constraint.

---

## 10. Bring-up order — do not skip

1. PDB alone, no loads. Verify all rails at 9 V, 11.1 V and 12.6 V input.
2. Master switch under a 5 A dummy load. Confirm no arcing or heating.
3. STM32 + SWD only. Blink PC13. **Confirm SWD still works after the JTAG-release change.**
4. Board B alone: verify 3.3 V, then scan I²C for the mux at 0x70.
5. One channel at a time, **motor disconnected**: walk ch0…ch7, confirm each sensor
   answers at 0x29 on its own channel and nowhere else.
6. Then encoder → competition IMU on SPI → servo.
7. Motor connected, **wheels off the ground**, 20% duty. Watch every sensor's noise
   floor. **If any reading moves when the motor spins, stop and fix the grounding** — do
   not filter it out in firmware.
8. Only then: on the ground, `CENTER_US` bisection sweep.

**Test points to expose on Board A:** VBAT, 6 V, 5 V, 3V3, AGND, star GND, ENC_A,
SERVO_PWM. Eight pads, ten minutes of layout, hours of probing saved.

---

## 11. Open electrical items

| # | Item | Blocks |
|---|---|---|
| 1 | **Competition SPI IMU not ordered** | The racing gyro path. Longest lead item remaining. |
| 2 | **Battery connector — XT30 or JST?** | Power harness |
| 3 | VL53L0X breakout: onboard pull-ups present? | Whether Board B needs per-channel pull-ups |
| 4 | 25GA stall current — clamp meter, wheels locked | Journal power budget only |
| 5 | 3S pack capacity and C rating | Mass and run-time budget |
| 6 | Module footprints — caliper every one | Both board layouts |
| 7 | ToF/TCS connector keying (§8) | Harness build |
