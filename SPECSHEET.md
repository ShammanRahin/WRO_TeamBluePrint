# SPECSHEET — WRO Future Engineers 2026

**Team:** Blueprint · **Competition:** WRO Future Engineers 2026 (October 2026)
**Status:** Design phase — v1 print-ready target, then iterate on real mat (Sept–Oct).
This file is a SOURCE OF TRUTH. Update it as decisions lock.

---

## 1. Rule-hard constraints (WRO FE 2026)

| Constraint | Limit |
|---|---|
| Footprint | ≤ 300 × 200 mm |
| Height | ≤ 300 mm |
| Mass | ≤ 1.5 kg |
| Wheels | 4 wheels, ONE driven axle, ONE steering actuator |
| Drive | No diff-drive, no omni. Max 2 drive motors, mechanically coupled to the same axle |
| Wireless | None during runs |
| Parking bay | 200 mm deep × (1.5 × car length). Magenta blocks untouchable (touch = 0 parking points) |
| "Parallel" | Same-side wheels within 2 cm of wall |

## 2. Scoring model (122 total)

- ~75% driving, ~25% engineering journal.
- Parking = 15 pts; +7 pts for starting inside the lot.
- **Strategy: full parallel park.**
- Commit deadlines (gated): 1st commit ≥ 2 months pre-comp (≥ 1/5 of code), 2nd ≥ 1 month, and the **≥ 2-weeks-before commit is the one judges score**. README ≥ 5000 chars.

## 3. Geometry targets (finalise Day 1 with REAL numbers)

`R = wheelbase ÷ tan(steering lock)` · Target **R ≈ 120–150 mm**

| Parameter | Target |
|---|---|
| Wheelbase | 110–130 mm |
| Steering lock | ≥ 40° (aim 45°) |
| Length | 170–190 mm |
| Width | 110–120 mm (bay only 200 mm deep — narrow = parking margin) |
| Wheels | 40–45 mm (smaller = finer odometry + lower speed off a fast motor + low CG) |
| Front overhang | Minimise (front outer corner sweeps toward the entry block) |
| Parking slack | Only 0.5 × car length, ALWAYS |
| Park manoeuvre | Reverse two-arc (rear axle is pivot → reverse tucks rear in first) |

**Locked-in inputs (2026-07-12):** National corridor width = **1 m**. Wheel diameter is **free** — Shanto has a 3D printer and prints wheels to spec, so wheel dia becomes a design variable to be chosen against R and odometry (start at 45 mm, tune).

⚠️ STILL FLAGGED: wheelbase, steering lock, length, width must be set from measured/chosen parts, not guessed.

## 4. Drivetrain / motor

- **Motor:** 25GA, 1331 RPM, 180 counts/rev.
- **Working assumption (2026-07-12): 180 counts per WHEEL rev.** ⚠️ PLACEHOLDER — Shanto to confirm by hand-rotate test and correct if it's per motor rev.
  - At 180/wheel rev @ 45 mm wheel: 180 ÷ (π×45) ≈ 1.3 counts/mm ≈ 0.79 mm/count — adequate for parking.
  - If it turns out per motor rev: multiply by gear ratio — luxurious resolution.
- 1331 RPM is fast → **cap speed in software**, especially for parking.

## 5. Navigation strategy

Gyro heading-hold on straights → detect wall ahead → 90° turn **terminated by IMU** (not by steering angle, so tyre slip can't corrupt it) → re-reference heading to nearest 90° at every corner (bounds gyro drift).
Modelled on the previous BD national winner's approach (fastest + smoothest).

**"Straight" is found by motion, not by eye:** sweep servo µs, drive ~3 m, log average gyro yaw-rate, bisect to zero-crossing → that µs is the true `CENTER_US`. No car is mechanically straight; gyro heading-hold finds the true straight.

## 6. Compute split

- **Open round:** STM32 only (ToF/IR/sonar + IMU + encoder). Fully deterministic, NO Pi in the loop.
- **Obstacle round:** adds **Raspberry Pi 4B (1 GB)** + **fisheye-lens camera** for pillar colour ONLY.
- Inter-board link = **checksummed UART**. NEVER inter-board I2C.
- STM32 chosen over ESP32: clean 12-bit ADC (Sharp IR is analog), hardware quadrature encoder timers, deterministic timing (no WiFi stack stealing cycles).

## 7. Steering

- Single central pivot, **bell-crank** preferred over pure turntable.
- Servo → bearing-supported steering shaft. **AS5600 magnetic encoder ON THAT SHAFT** closes the loop on TRUE wheel angle (kills servo backlash → "command 45°, get 45°" is real).
- Servo, commanded in **microseconds** (not degrees — degrees quantise ~10 µs/deg, too coarse).
- **Part (2026-07-12): MG90S** — metal-gear (✓) but *analog* (Decision #4 wanted digital). Acceptable for v1 because the AS5600 closed loop absorbs the backlash/deadband; watch torque under bump loads and re-evaluate if holding precision drifts on the mat.
- Ackermann rejected for v1 (4–6 slop-prone joints, worst for a novice assembler). Analyse the trade-off in the journal for rubric points instead of building it.

## 8. Sensors

- **Primary distance (2026-07-12, team choice): ToF VL53L1X ×5.** ⚠️ REVERSES the original "ToF rejected" stance (see Decision #7) — VL53L1X's known failure mode is false floor returns on a low car. Mitigation to VALIDATE on the real mat: mount above the floor grazing angle / slight upward tilt, use a tight ROI window, gate out min-range and floor-distance returns, median-filter. If false detections persist, fall back to the plan below.
- **Documented fallback:** analog **Sharp IR** (buy only if needed; calibrated voltage→mm curve, median-filter 5 samples, mount out of the fold-back zone) and/or front **sonar** (walls are big flat perpendicular reflectors = ideal for sonar).
- **LIDAR deferred** — design upper deck to accept it later, build v1 without it (heavy, tall, forces Pi 5 + Linux jitter, works against determinism).
- Sensors + camera mount to the FIXED chassis, never to the pivoting steering beam (else readings swing when steering).

## 9. Reliability spec ("no randomness")

- Ball bearings on every axle AND the steering pivot (plastic-on-shaft wears in and grows slop mid-competition).
- Locate parts by GEOMETRY (shoulders, slots, captured screws) — glue is backup only, never primary.
- Threadlocker / nail polish on every thread; nyloc nuts where possible.
- NO Dupont jumpers, no breadboard on the robot. Solder, or JST/screw terminals with strain relief.
- Separate motor and logic power domains, joined at ONE star ground. Bulk cap 470–1000 µF on motor supply, 0.1 µF near MCU.
- 0.1 µF ceramic caps across motor terminals (EMI suppression).
- Twist encoder A/B leads, route away from motor leads, enable pull-ups.
- **Design principle for a novice assembler: a loose joint must STILL not be able to shift.**

## 10. Reference repos (fetch live, don't copy files)

- KMIDS-GFM 2025 — github.com/Chayanon-Ninyawee/KMIDS-GFM-Future-Engineer-2025 (right-sized, fisheye + lidar + Pi, FreeCAD models — closest analog)
- Nerdvana Taurus 2025 — github.com/andreipopescufilimon/WRO2025_Future_Engineers (best mechanical/steering docs; restrictive licence — study, re-model, do NOT reuse files)
- Official archive of all 64 finalist repos — WRO-Association fe-2025-links
