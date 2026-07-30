# SPECSHEET — WRO Future Engineers 2026

**Team:** Blueprint · **National competition: 20 August 2026** · International: October 2026
**Status:** Vehicle built; electronics in PCB design. Iterating on the real mat through August.
This file is a SOURCE OF TRUTH. Update it as decisions lock.
**Revised 2026-07-28** — steering lock, I²C topology, IMU and electrical build all changed;
see Decisions #23–#28.

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

## 3. Geometry — AS BUILT (2026-07-26)

> **SUPERSEDES** the 2026-07-12 optimizer-locked geometry (track 120 / wheelbase 128 /
> 175x138x95 / wheel 40 mm / single central pivot). That design was never built. The
> optimizer output in `src/sim/optimize_layout.py` is retained as a study, not a spec.
> Divergence rationale: `journal/day-02-plan-revision-2026-07-26.md`, Decision #4 block.

| Parameter | As built | Note |
|---|---|---|
| Track (wheel centre to centre) | **105 mm** | |
| Track (wheel extreme to extreme) | **115 mm** | this is the scored width, not the 80 mm plate |
| Body length | **165 mm** | scored length = projection on mat (Appendix A 2/6) |
| Chassis plate | 80 x 130 mm | |
| **Wheelbase (front axle to rear axle)** | **110 mm** | measured 2026-07-28. Ceiling is 165 - 23 - 25 = 117 mm |
| Front wheel dia | **46 mm** | |
| Rear wheel dia | **50 mm** | drives odometry |
| Steering lock | **+/-35 deg** | parallelogram, equal-angle. **Final.** |
| **Turn radius R** | **157 mm** | 110 / tan 35 deg |
| Car height (current) | **50 mm** | will grow to ~75-90 mm with Pi 4B + BTS7960 stack |
| Steering | **parallelogram tie-bar, open-loop** | Decision #15, #16 |
| Rear axle | solid, no diff, **5:1 gear** | Decision #14 |

**Scored footprint 165 x 115 mm** — well inside the 300 x 200 mm limit.

### Turn radius — RESOLVED 2026-07-28

`R = wheelbase / tan(lock) = 110 / tan(35 deg) = **157 mm**`

**157 mm is above the 120-150 mm target band, by 7 mm.** This is accepted, not overlooked.
Raising the lock to 40 deg would put R at 131 mm and inside the band, and that change was
briefly adopted before being withdrawn: the linkage is built and measured at 35 deg, and
a lock angle the mechanism does not actually reach is worth nothing. The band was a design
guideline from the 2026-07-12 optimiser, not a rule limit, and 5% over it costs nothing
scored. The parking manoeuvre — which is what the band was really protecting — is handled
by the multi-point shuffle regardless of R (Decision #21).

```bash
python3 src/sim/geometry_sweep.py                      # defaults to the measured 110 mm
python3 src/sim/geometry_sweep.py --wheelbase 110 --plot
```

🚩 **105 mm is the TRACK, not the wheelbase.** Keeping the note because the two were
confused more than once: track is lateral (wheel centre to wheel centre), wheelbase is
longitudinal (front axle centre to rear axle centre).

### Chassis rake — a real, inherited error
Front 46 mm vs rear 50 mm = 2 mm axle height difference over the wheelbase:
`rake = atan(2 / 110)` = **1.04 deg nose-down** (was quoted as a 1.0-1.1 deg range while
the wheelbase was unmeasured).
Every chassis-mounted sensor inherits this and is aimed AT THE FLOOR. Corrected by a
+2 deg printed wedge in every ToF mount — see Section 8 and Decision #18.

### Parking geometry — ⚠️ TWO-ARC PARK IS NOT FEASIBLE AT 35 deg LOCK
Bay = 1.5 x 165 = **247.5 mm** -> total longitudinal slack **82.5 mm**.
Limiters are 200 x 20 x 100 mm and the car is 50-90 mm tall, so the car **cannot pass
over them** at any point in the manoeuvre.

`src/sim/park_feasibility.py` (WB **110 mm confirmed 2026-07-28**) sweeps the parked
longitudinal position — the only free variable in a symmetric two-arc — and finds:

| Lock | R | R/L | Best clearance | Verdict |
|---|---|---|---|---|
| **35 deg (as built, FINAL)** | **157 mm** | **0.95** | **-25.6 mm** | **COLLISION** |
| 40 deg | 131 mm | 0.79 | -10.8 mm | COLLISION — considered and withdrawn |
| 45 deg | 110 mm | 0.67 | +0.1 mm | clears by nothing |
| 60 deg | 64 mm | 0.38 | +0.5 mm | clears by nothing |

The 40 and 45 deg rows are kept because they are the evidence that raising the lock does
not solve this. 40 deg still collides; 45 deg clears by 0.1 mm, which on an open-loop
steering system is not clearance.

The front-outer corner sweeps into the entry limiter at ~37 deg heading while the rear
axle is still 164 mm along the bay.

**The problem is SCALE-INVARIANT.** Bay = 1.5 x car length always, so slack = 0.5 x car
length always. Shortening the car does not help — the bay shrinks with it. The only
levers are the ratio **R/L** and the manoeuvre strategy.

**Decision (#21 amended): the primary parking manoeuvre is a MULTI-POINT (3+) SHUFFLE,
not a two-arc.** Enter at whatever angle the geometry allows, then shuffle forward/back
against **IMU yaw** to straighten. Slower, but robust to R error — which matters because
steering is open-loop (Decision #16) and nominal R cannot be trusted. Validate on the mat.
Touching a limiter = 0 parking points (rule 9.24.7); parking is 22 of 122 points.

## 4. Drivetrain / motor - AS BUILT

- **Motor:** 25GA, 1331 RPM, 180 counts/rev, encoder on the motor.
- **Reduction:** **5:1 gear meshed to the solid rear axle** (Decision #14 confirmation).
- Rear wheel speed = 1331 / 5 = **266 RPM**.
- **Top speed** = 266/60 x pi x 0.050 = **0.70 m/s**.
- Encoder edge rate at full speed = **3995 Hz** (encoder is upstream of the 5:1) - trivial for TIM3.

### Odometry
> **SUPERSEDES** the 2026-07-12 figure "180 / (pi x 45) ~ 0.79 mm/count", which used a
> 45 mm wheel (never built) and had no 5:1 term.

If 180 counts/rev is measured **at the motor**, then after the 5:1 there are
**900 counts per rear-wheel revolution**:

`900 / (pi x 50) = 5.73 counts/mm = **0.175 mm/count**`

FLAG - OPEN QUESTION #1, open since 2026-07-12: the hand-rotate test has still not been
done. If 180 turns out to be per WHEEL rev the constant is 5x coarser (0.87 mm/count).
Everything else in this file is independent of the answer; only this constant changes.

**Why the 5:1 is the highest-value decision in the build:** ungeared, the car ran at
2.79 m/s with 0.70 mm/count and a 7.2 ms line-detection window. Geared, it runs at
0.70 m/s with 0.175 mm/count and a 28.6 ms window. It converted three separate problems
(uncontrollable speed, coarse odometry, unreadable colour lines) into non-problems.

**Closed-loop speed control on the encoder is mandatory** - see Decision #22 (BTS7960
has poor resolution at the ~14% duty that parking needs).

## 5. Navigation strategy - AS BUILT

Core (unchanged from Decision #1): gyro heading-hold on straights, 90 deg turns
**terminated by IMU** (not by steering angle, so tyre slip cannot corrupt them),
heading **re-referenced to the nearest 90 deg at every corner** to bound gyro drift.

**CHANGED 2026-07-26:** the turn trigger is now the **TCS34725 floor colour sensor**,
not front wall distance. Front ToF is demoted to confirmation / anti-collision.

### Open round state machine (12 turns = 3 laps x 4 corners)

1. `ARMED` - wait for start button (rule 9.11). Gyro bias auto-zeroed while stationary;
   the car is placed switched off (rule 9.6) so it is guaranteed still. This is firmware
   self-calibration, not team calibration, so rule 9.9 is satisfied.
2. `CENTRE` - L90 + R90 ToF equalise lateral error, then latch heading as reference.
3. `STRAIGHT` - heading-hold on IMU; encoder integrates distance; front ToF monitors
   closing distance as a backstop.
4. `LINE_EVENT` - TCS sees orange or blue.
   - **First pair of the run decodes direction.** Driving direction is drawn randomly
     before each round (rule 9.3). Orange-then-blue vs blue-then-orange sets left vs
     right for all 12 turns. Without this, 50% of runs steer the wrong way.
   - **Hard lockout.** Each corner carries BOTH an orange and a blue line (rule 13.9),
     so a naive trigger fires ~24 times over 3 laps, not 12. All colour input is ignored
     from turn start until the turn completes AND the car has driven clear.
5. `TURN` - steer to lock, integrate yaw, terminate at 90 deg, re-reference heading.
6. Increment turn counter. At 12, `STOP` inside the starting section (rule 9.24.3).

### Timing headroom
20 mm line at 0.70 m/s = **28.6 ms dwell**. TCS at 2.4 ms integration = ~11 samples.
Use single-sample edge detection with hysteresis. **Do not median-filter the colour
sensor** - a median filter smears the event away.

### "Straight" is found by motion, not by eye
Sweep servo microseconds, drive ~3 m, log average gyro yaw-rate, bisect to the
zero-crossing - that microsecond value is the true `CENTER_US`. No car is mechanically
straight; this matters more now that steering is open-loop (Decision #16).

## 6. Compute split

- **Open round:** STM32 only (ToF/IR/sonar + IMU + encoder). Fully deterministic, NO Pi in the loop.
- **Obstacle round:** adds **Raspberry Pi 4B (1 GB)** + **fisheye-lens camera** for pillar colour ONLY.
- Inter-board link = **checksummed UART**. NEVER inter-board I2C.
- STM32 chosen over ESP32: clean 12-bit ADC (Sharp IR is analog), hardware quadrature encoder timers, deterministic timing (no WiFi stack stealing cycles).

## 7. Steering - AS BUILT

> **SUPERSEDES** the 2026-07-12 "single central pivot + AS5600" specification.

- **Parallelogram (equal-angle tie-bar) linkage.** Measured lock **+/-35 deg**.
- **Servo: MG996R** - metal-gear, **analog**. Commanded in microseconds
  (1 us timer tick, ARR = 19999).
- **Frame rate is 50 Hz and cannot be raised** - MG996R is analog. This sets a hard
  20 ms floor on steering command latency regardless of how fast the IMU is read.
  Deadband ~5 us; slew ~150 ms/60 deg at 6 V, so a full +/-35 deg sweep is ~175 ms.
- **No AS5600. Steering is fully open-loop** (Decision #16). The loop is closed on
  vehicle heading by the IMU, so steering only has to be monotonic and repeatable.
- **Firmware requirement:** an analog servo inside any closed loop will hunt across its
  deadband - current draw, heat, audible buzz, steering jitter. Quantise output to
  >= 6 us steps and hold inside a +/-1.5 deg error window.

**Why parallelogram over the simulated central pivot** (full argument in Decision #4
supersession): a central pivot swings each front wheel fore/aft by `(track/2) x sin d`.
At track 105 mm and 35 deg lock that is **+/-30.1 mm per wheel = 36% of the 82.5 mm
parking slack**, spent for no navigational gain. `src/sim/steering_study.py` never
modelled swept envelope, which is why it ranked parallel bell-crank as dominated.

**Costs accepted:** 4+ joints vs 1; no steering feedback; equal-angle scrub unchanged
(parallelogram is NOT Ackermann); turn radius nominal rather than measured.

## 8. Sensors - AS BUILT

> **REVISED 2026-07-28 (Decisions #23, #24).** XSHUT address reassignment is replaced by a
> **PCA9548A multiplexer** — one sensor per channel, every device at its factory 0x29, five
> XSHUT GPIO reclaimed and the volatile-address-on-brownout problem deleted rather than
> managed. The MPU9250 is **not on hand**; the part we have is an MPU6050, which is I2C-only.

| Sensor | Part | Bus | Role |
|---|---|---|---|
| Distance x5 (6th planned) | **VL53L0X** | PCA9548A ch0-ch5, all @ 0x29 | F / FL30 / FR30 / L90 / R90 |
| Floor colour | **TCS34725** | PCA9548A ch6 | turn trigger + direction decode |
| Heading — **race** | **BNO08x / ICM-42688** | **SPI1, mainboard, no mux** | gyro-Z only 🚩 **not yet ordered** |
| Heading — bench only | **MPU6050** (`WHO_AM_I` 0x68) | PCA9548A ch7 | bring-up only, **will not race** |
| Odometry | 25GA encoder | TIM3 quadrature | distance + closed-loop speed |
| Pillar colour | Fisheye 160 deg + Pi 4B | UART | obstacle round only |

**Why the racing IMU stays off the mux:** Decision #1 terminates all 12 corners and every
park arc on measured heading, so the gyro is the most latency-critical signal in the
vehicle. Behind a mux it would queue behind five ToF sensors and a cable. The bench
MPU6050 can afford that; the racing part cannot.

🚩 **The competition IMU is the longest-lead item left and is not ordered.** Contingency:
I2C2 is routed to the same header, so the MPU6050 can move onto a dedicated bus by
swapping a cable rather than re-etching a board.

🚩 **One regression to watch:** the TCS34725 previously had a private bus specifically so
no ToF cable fault could take down the turn trigger. It now has address isolation but not
fault isolation. `MUX_RST` is wired to the STM32 for exactly this reason. If bench testing
shows ToF faults hanging the bus, move the TCS back to its own bus on PB10 — one spare pin.

> **CORRECTS** the 2026-07-12 entry, which specified VL53L1X. The parts on hand are
> **VL53L0X**, which has **no programmable ROI** - the mitigation assumed in that entry
> does not exist on this part.

### The floor-return problem, quantified
The mat is white (rule 13.2); both exterior and interior walls are black on every
visible face (rules 13.4, 13.6). **The false target reflects NIR better than the real
one.** Bare L0X FoV is ~25 deg fixed:

`d_floor = h / tan(theta_eff)`, side wall at (1000 - 115)/2 = **442.5 mm**

| Configuration | theta_eff | d_floor @ h = 40 mm | vs 442.5 mm |
|---|---|---|---|
| Bare L0X | 12.5 deg | 180 mm | catastrophic |
| Snout only | 3.58 deg | 640 mm | ok, thin under rake |
| **Snout + 2 deg wedge** | **2.67 deg** | **859 mm** | **+416 mm** |

**Mitigation is mechanical and mandatory (Decision #18):** printed collimator snout
**2.5 mm tall x 10 mm wide x 20 mm deep** + **+2 deg upward mounting wedge**.
Slot must be WIDE, not round - emitter and receiver apertures are ~2.5 mm apart.
Solver + regenerable table: `electrical/collimator.py`.

Firmware layer on top: reject on `RangeStatus != 0`, low signal rate, or high sigma;
median-of-3. **Re-run offset calibration after fitting the snout** - it changes crosstalk.

### Duty split between the 90 deg and 30 deg pairs (Decision #19)
- **L90 / R90** - open-corridor lane centring. 442.5 mm lateral, inside d_floor.
- **FL30 / FR30** - obstacle-corridor centring when threading a pillar-to-wall gap.
  Slant range = gap / sin 30 deg = 2 x gap: a 150 mm gap reads at 300 mm, a 350 mm gap
  at 700 mm. They degrade exactly when the gap is wide, which is when L90/R90 take over.
- FLAG to bench-test: at 30 deg off-axis the beam hits the wall at 60 deg from normal,
  so return falls by cos 60 = 0.5 on an already-black surface.

### Highest-risk open item in the whole build
**Bench-test VL53L0X signal rate against black MDF, snout fitted, at 442 mm (90 deg pair)
and at 300 / 700 mm at 60 deg incidence (30 deg pair), BEFORE printing five mounts.**
The snout throws away photons and the wall is already a poor NIR target. If signal rate
is marginal, the Sharp GP2Y0A21 fallback must be ordered immediately - and its lead time
is still unrecorded in `BOM.md` (Open Question #3).

### Unchanged
- **LIDAR deferred** - design the upper deck to accept it later.
- **Sensors + camera mount to the FIXED chassis**, never the steering linkage.

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

## 11. Electrical architecture

Full detail: `electrical/ELECTRICAL.md`. Block diagram: `schemes/wiring_block_diagram.png`.

### Power domains - ONE master switch (rule 9.10)
```
  [connector]              MASTER SWITCH (rule 9.10: only one switch may power the vehicle on)
[3S LiPo 75C]--+------------[SPST 10A]--+--> DOMAIN M : BTS7960 -> 25GA -> 5:1 -> solid rear axle
               | (NO FUSE, #26)         +--> BEC-S 6.0V/3A -> MG996R servo ONLY
           (star GND)                   +--> BEC-L 5.0V/2A -> Board A logic + Board B (3V3 local)
               |                        +--> BEC-C 5.1V/3A -> Pi 4B + fisheye  [OPEN ROUND: REMOVED]
               +-- all BEC grounds + battery negative meet at ONE M3 brass standoff
```

🚩 **Fuse deliberately omitted (Decision #26)** — accepted risk, not an oversight. A 75C 3S
pack will deliver several hundred amps into a short with no current limiting.
🚩 **Battery connector unresolved** — JST-XH is ~3 A per contact against ~8 A peak draw;
XT30 (30 A) remains the specified part until decided otherwise.

### Physical build — two stacked single-sided boards (Decision #25)
Our fab does double-sided **without plated through-holes**, so two-layer is unusable and
**no ground plane is available at any layer count**. At 0.5 mm minimum trace and space,
**no trace can pass between two adjacent header pins**, so placement rather than routing
is the design problem.

- **Board A (upper):** STM32, BTS7960, BEC rails, servo, encoder, Pi UART, competition IMU
- **Board B (lower):** PCA9548A + 8 channels + local AMS1117-3.3
- Joined by a ~40 mm **6-pin JST-XH**. High current never crosses PCB copper.

Full detail: `electrical/ELECTRICAL.md`. Fab rules: `electrical/DESIGN_RULES.md`.

The Domain C harness is **physically absent** for the Open Challenge. That is an
inspectable claim, not a software flag: the Open configuration cannot be influenced by
the Pi because the Pi is not electrically present.

### Power budget
| Domain | Load | Typ | Peak |
|---|---|---|---|
| M | 25GA via BTS7960 | ~0.6 A | FLAG - stall not yet measured |
| S | MG996R | 0.5-0.9 A | ~2.5 A stall |
| L | STM32 40 mA + 5x VL53L0X 100 mA + TCS 3 mA + MPU6050 4 mA + PCA9548A 1 mA | **~150 mA** | ~250 mA |
| C | Pi 4B 1 GB + camera | 0.85 A | ~1.25 A |

Pi 4B: set BEC-C to **5.15 V measured at the header under load**, feed pins 2/4/6 with
18 AWG. Never exceed 5.25 V - this bypasses the USB-C input protection.

### Loop rates
| Loop | Rate | Bound by |
|---|---|---|
| IMU gyro-Z + heading integrate | 1000 Hz | SPI, fixed timer tick (constant timestep) |
| Encoder | 1000 Hz | free, TIM3 hardware |
| Drive speed PID | 200 Hz | mandatory - see Decision #22 |
| **Servo command** | **50 Hz** | **MG996R analog - the steering bottleneck** |
| VL53L0X x5 | 30-50 Hz | sensor |
| TCS34725 | >= 100 Hz | 2.4 ms integration |
| Pi colour frame | 15-30 Hz | advisory only, never gates a control loop |

### Reliability, unchanged from Section 9
Star ground; motor return on its own >= 1.0 mm^2 leg, never through logic ground;
1000 uF + 0.1 uF at the BTS7960 input; 470 uF + 0.1 uF at the servo connector;
0.1 uF across motor terminals and 2x terminal-to-can; encoder A/B twisted with
10k pull-ups + 100R/1nF; no Dupont anywhere.
