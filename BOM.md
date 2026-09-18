# BOM — Bill of Materials

**As-built, updated 2026-09-18** (originally 2026-07-26; supersedes the 2026-07-12 speculative BOM).
⚠️ Source, lead time and price are still **Open Question #3** — fill from a real
inventory check + Daraz/RoboticsBD lookup. TBD = not yet confirmed.

## Locked / on hand

| Part | Spec | Status | Note |
|---|---|---|---|
| Drive motor | 25GA gearmotor, 1331 RPM, encoder on motor | **On hand** | Measured scale: 14.853 ticks/cm (current car) |
| Gear reduction | **5:1**, meshed to solid rear axle | **Built** | → 266 RPM, 0.70 m/s theoretical |
| Rear axle | Solid, no differential | **Built** | Decision #14 |
| Steering | **Ackermann linkage** (107° / 73° arms), lock **±35°** | **Built** | Replaced the parallelogram of Decision #15 in September |
| **I²C mux** | **TCA9548A / PCA9548A module** @ 0x70 | **On hand** | Decision #23 — replaces XSHUT addressing, reverses #20 |
| Steering servo | **JX PS-1171MG** — digital, metal-gear, 17 g | **On hand** | Replaced the MG996R (analog, ~55 g) |
| MCU | **STM32F411CEU6** "Black Pill" | **On hand** | 26/28 usable pins allocated |
| Motor driver | **BTS7960** 43 A | **On hand** | Decision #22 — ~20× oversized, ~66 g, poor low-duty resolution |
| Distance | **VL53L0X** (Open fw: 1× front) and **VL53L1X** (Obstacle fw: left, right, front) | **On hand** | VL53L0X has no programmable ROI — Decision #7 correction; one sensor per mux channel |
| Floor colour | **TCS34725** | **On hand** | I²C 0x29, on its own mux channel |
| IMU | **BNO085**, SPI | **On hand** | Race IMU (Decision #24). Game rotation vector, no magnetometer |
| IMU — bench only | MPU6050, I²C, `WHO_AM_I` = 0x68 | On hand | No longer used |
| SBC (obstacle round only) | **Raspberry Pi 5, 8 GB** | **On hand** | Quad-core A76 @ 2.4 GHz; runs concurrent vision + LIDAR SLAM |
| LIDAR (obstacle round only) | **Slamtec RPLIDAR C1** | **On hand** | 360° DTOF, 12 m range, ~110 g; USB to Pi 5 (Decision #29) |
| Camera | **Fisheye 160° FoV** | **On hand** | Pillar colour detection + horizontal offset |
| Battery | **3S LiPo, 75C** | **On hand** | ⚠️ capacity/mAh still TBD |
| Buck (logic) | 5 V module | **On hand** | Feeds Board A logic + Board B |
| Wheels | Front **46 mm**, rear **50 mm**, printed | **Made** | 4 mm mismatch → 1.0–1.1° rake, corrected by ToF wedge |
| Chassis | Printed, 80 × 130 mm plate | **Built** | |

## Required, not yet sourced

| Part | Spec | Why mandatory |
|---|---|---|
| Master switch | SPST, **≥10 A**, panel mount | Rule 9.10 — exactly one switch may power the vehicle on |
| AMS1117-3.3 + 22 µF | Board B local 3.3 V regulator | ~250 mA sensor load; keeps it off the MCU rail |
| Start button | Momentary NO + 10 kΩ / 1 kΩ / 100 nF debounce | Rule 9.11 — separate from the power switch. ⚠️ Not yet wired/read on the current carrier (PB15 reserved) |
| BEC-S | **6.0 V, 3 A cont / 5 A peak** | Servo stall current must not sag the logic rail |
| BEC-L | **5.0 V, 2 A** | STM32 + 5× ToF + TCS + IMU ≈ 150 mA typ |
| BEC-C | **5.1 V, 5 A**, adjustable | Pi 5 + RPLIDAR C1; prevents low-voltage USB throttling (Decision #29) |
| Battery connector | XT30 | |
| Bulk caps | 1000 µF 25 V low-ESR (BTS7960 input), 470 µF (servo connector) | Brown-out prevention |
| Ceramics | 0.1 µF ×~15 — motor terminals ×1, terminal-to-can ×2, every IC VCC | EMI + decoupling |
| Encoder RC | 10 kΩ pull-ups ×2, 100 Ω + 1 nF ×2 | f_c ≈ 1.6 MHz vs 3995 Hz signal |
| I²C pull-ups | 2.2 kΩ ×4 (2 per bus) | 400 kHz noise margin |
| Series resistors | 220 Ω ×3 (servo signal, UART TX/RX) | Ringing suppression |
| Connectors | **JST-XH only** (2.5 mm) + crimp tool, 5.08 mm screw terminals | **No Dupont anywhere** (#11). **JST-PH dropped** — 2.0 mm pitch is not manufacturable at our fab (#27) |
| Ball bearings | Every axle + steering pivots | Decision #9 — bore/OD depends on shaft, lock shaft first |
| Fasteners | Nyloc nuts, threadlocker / nail polish | Decision #10 |
| **ToF collimator snouts** (one per ToF) | **2.5 × 10 × 20 mm slot + 2° wedge**, printed | **Decision #18 — the L0X has no ROI; this is the only floor mitigation** |

## Contingency — order decision pending a bench test

| Part | Trigger | Risk |
|---|---|---|
| Sharp GP2Y0A21 ×2–5 | If VL53L0X signal rate against **black MDF** is marginal at 442 mm (90° pair) or at 60° incidence (30° pair) | ⚠️ **Lead time still unrecorded.** This is the highest-risk unpriced item in the build — if the ToF mitigation fails on the mat and Sharp is 3 weeks out, there is no recovery path. |
| TB6612FNG | If BTS7960 low-duty control proves erratic | Decision #22 |

## Dropped

| Part | Reason |
|---|---|
| AS5600 | No central pivot to measure — steering is open-loop, closed on IMU heading (Decision #16) |
| ~~TCA9548A mux~~ | ~~XSHUT reassignment fits the pin budget (Decision #20)~~ — **REVERSED by #23.** A PCA9548A is now on hand and in use; XSHUT addressing is dropped instead. |
| MPU9250 | Not on hand. Superseded by MPU6050 (bench) + BNO085 (race) — Decision #24 |
| MG996R servo | Replaced by the JX PS-1171MG in the September rebuild |
| Parallelogram steering | Replaced by the Ackermann linkage in the September rebuild |
| **Inline fuse** | **Team decision #26 — accepted risk, not an oversight** |
| **JST-PH** | 2.0 mm pitch gives a 0.25 mm annulus at our fab, on a board with no plated holes — Decision #27 |
| XSHUT wiring ×5 | Unnecessary once each ToF has its own mux channel — Decision #23 |
| ~~LIDAR~~ | ~~Deferred~~ — **REVERSED by Decision #29.** Slamtec RPLIDAR C1 added for obstacle round. |

## Mass check
Electrical ≈ 3S LiPo 75 g + BTS7960 66 g + 3 BECs 35 g + harness 45 g + JX PS-1171MG 17 g
+ Pi 5 (with active cooler) ~60 g + RPLIDAR C1 ~110 g = **~408 g** before chassis/wheels/motor.
Total vehicle mass is estimated at **~540–580 g** (obstacle configuration; not yet weighed),
comfortably below the 1.5 kg WRO ceiling.

