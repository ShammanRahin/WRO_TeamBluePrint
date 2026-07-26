# BOM — Bill of Materials

**As-built, 2026-07-26.** Supersedes the 2026-07-12 speculative BOM.
⚠️ Source, lead time and price are still **Open Question #3** — fill from a real
inventory check + Daraz/RoboticsBD lookup. TBD = not yet confirmed.

## Locked / on hand

| Part | Spec | Status | Note |
|---|---|---|---|
| Drive motor | 25GA gearmotor, 1331 RPM, 180 counts/rev (encoder on motor) | **On hand** | ⚠️ counts/rev per motor or per wheel — hand-rotate test still open |
| Gear reduction | **5:1**, meshed to solid rear axle | **Built** | → 266 RPM, 0.70 m/s, 0.175 mm/count |
| Rear axle | Solid, no differential | **Built** | Decision #14 |
| Steering | **Parallelogram tie-bar**, lock ±35° | **Built** | Decision #15 |
| Steering servo | **MG996R** — metal-gear, **analog**, ~55 g, ~10 kg·cm | **On hand** | 50 Hz frame rate ceiling; ~2.5 A stall |
| MCU | **STM32F411CEU6** "Black Pill" | **On hand** | 26/28 usable pins allocated |
| Motor driver | **BTS7960** 43 A | **On hand** | Decision #22 — ~20× oversized, ~66 g, poor low-duty resolution |
| Distance ×5 | **VL53L0X** | **On hand ×5** | ⚠️ **no programmable ROI** — Decision #7 correction |
| Floor colour | **TCS34725** | **On hand** | I²C 0x29 — collides with VL53L0X, hence bus split |
| IMU | **MPU9250** | **On hand** | ⚠️ verify `WHO_AM_I` (0x75): 0x71 genuine / 0x70 = MPU6500 / 0x73 = MPU9255. Use **SPI**, gyro-Z only, **magnetometer disabled** |
| SBC (obstacle round only) | **Raspberry Pi 4B, 1 GB** | **On hand** | ~20 mm tall with connectors — drives the height growth |
| Camera | **Fisheye 160° FoV** | **On hand** | pillar colour only |
| Battery | **3S LiPo** | **On hand** | ⚠️ capacity/C-rating TBD |
| Wheels | Front **46 mm**, rear **50 mm**, printed | **Made** | 4 mm mismatch → 1.0–1.1° rake, corrected by ToF wedge |
| Chassis | Printed, 80 × 130 mm plate | **Built** | |

## Required, not yet sourced

| Part | Spec | Why mandatory |
|---|---|---|
| Master switch | SPST, **≥10 A**, panel mount | Rule 9.10 — exactly one switch may power the vehicle on |
| Fuse | 10 A blade, inline on battery + | LiPo safety, upstream of everything |
| Start button | Momentary NO + 10 kΩ / 1 kΩ / 100 nF debounce | Rule 9.11 — separate from the power switch |
| BEC-S | **6.0 V, 3 A cont / 5 A peak** | MG996R stall is ~2.5 A; must not sag the logic rail |
| BEC-L | **5.0 V, 2 A** | STM32 + 5× ToF + TCS + IMU ≈ 150 mA typ |
| BEC-C | **5.1 V, 3 A**, adjustable | Pi 4B; set to 5.15 V at header under load, never >5.25 V |
| Battery connector | XT30 | |
| Bulk caps | 1000 µF 25 V low-ESR (BTS7960 input), 470 µF (servo connector) | Brown-out prevention |
| Ceramics | 0.1 µF ×~15 — motor terminals ×1, terminal-to-can ×2, every IC VCC | EMI + decoupling |
| Encoder RC | 10 kΩ pull-ups ×2, 100 Ω + 1 nF ×2 | f_c ≈ 1.6 MHz vs 3995 Hz signal |
| I²C pull-ups | 2.2 kΩ ×4 (2 per bus) | 400 kHz noise margin |
| Series resistors | 220 Ω ×3 (servo signal, UART TX/RX) | Ringing suppression |
| Connectors | JST-XH / JST-PH kit + crimp tool, screw terminals | **No Dupont anywhere** (Decision #11) |
| Ball bearings | Every axle + steering pivots | Decision #9 — bore/OD depends on shaft, lock shaft first |
| Fasteners | Nyloc nuts, threadlocker / nail polish | Decision #10 |
| **ToF collimator snouts ×5** | **2.5 × 10 × 20 mm slot + 2° wedge**, printed | **Decision #18 — the L0X has no ROI; this is the only floor mitigation** |

## Contingency — order decision pending a bench test

| Part | Trigger | Risk |
|---|---|---|
| Sharp GP2Y0A21 ×2–5 | If VL53L0X signal rate against **black MDF** is marginal at 442 mm (90° pair) or at 60° incidence (30° pair) | ⚠️ **Lead time still unrecorded.** This is the highest-risk unpriced item in the build — if the ToF mitigation fails on the mat and Sharp is 3 weeks out, there is no recovery path. |
| TB6612FNG | If BTS7960 low-duty control proves erratic | Decision #22 |

## Dropped

| Part | Reason |
|---|---|
| AS5600 | No central pivot to measure — steering is open-loop, closed on IMU heading (Decision #16) |
| TCA9548A mux | XSHUT address reassignment fits the pin budget; mux no longer needed (Decision #20) |
| VL53L1X | Never on hand; parts are L0X (Decision #7 correction) |
| LIDAR | Deferred — reserve upper-deck space, do not buy for v1 |

## Mass check
Electrical ≈ 3S LiPo 75 g + BTS7960 66 g + 3 BECs 30 g + harness 40 g + MG996R 55 g
+ Pi 4B 46 g = **~312 g** before chassis/wheels/motor. Against the 1.5 kg limit this is
comfortable; against the original ~450 g target it is not — **re-estimate total mass
once the Fusion model has real materials assigned.**
