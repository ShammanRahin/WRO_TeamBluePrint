# Team Blueprint — WRO Future Engineers 2026

A self-driving car built by two students in Bangladesh for the WRO Future Engineers
category. Islamic University of Technology.

This repository is the working record of the vehicle: the decisions, the reasons behind
them, the simulations we used to test those reasons, and the points where the build proved
a decision wrong and we changed it.

We tried to write it the way we wish other repos had been written when we started. Most of
the ones we read told us *what* they built. Very few told us *why*, and almost none told us
what they got wrong. So our mistakes are still in here, with dates on them.

---

## Where to start

Three files are the source of truth:

| File | Contents |
|---|---|
| **`SPECSHEET.md`** | Every locked parameter — geometry, drivetrain, sensors, electrical |
| **`DECISIONS.md`** | One entry per decision **with its reason**, plus dated supersession blocks where the build overruled the design |
| **`BOM.md`** | Parts, status, and what is still unsourced |

Everything else supports those three. `journal/` is chronological, `src/sim/` holds the
evidence behind the decisions, and `electrical/` has the wiring and PCB work.

**Superseded decisions are never deleted.** When the build contradicted a design choice,
the original reasoning stays visible and a dated block goes underneath explaining what
changed. Three of these are load-bearing — steering mechanism, parking manoeuvre, and the
I²C topology — and all three are worth reading before anything else.

---

## The vehicle

| Parameter | Value |
|---|---|
| Scored footprint | **165 × 115 mm** (limit 300 × 200 mm) |
| Height | 50 mm as built; ~75–90 mm with the obstacle-round stack (limit 300 mm) |
| Track | 105 mm centre-to-centre, 115 mm wheel extremes |
| Wheelbase | **110 mm** (measured 2026-07-28) |
| Wheels | 46 mm front, 50 mm rear |
| Steering | Parallelogram tie-bar, **±35° final**, one MG996R |
| Turn radius | **157 mm** (110 ÷ tan 35°) |
| Drive | One 25GA gearmotor → **5:1 gear** → **solid rear axle**, no differential |
| Top speed | **0.70 m/s** |
| Odometry | **0.175 mm/count** |
| Control | STM32F411CEU6 |
| Vision | Raspberry Pi 4B + 160° fisheye — **obstacle round only** |

### Rule compliance
Four wheels, one driven axle, one steering actuator (rule 11.3). One drive motor,
mechanically coupled through a gearbox to the axle, never independently driven
(rules 11.5, 11.13). No wireless of any kind during runs; the Pi's WiFi and Bluetooth are
disabled in `config.txt` (rule 11.10). **One** master switch powers the vehicle on
(rule 9.10) and a **separate** momentary button starts the program (rule 9.11).

---

## How it drives

### Open Challenge — STM32 only, no Pi in the loop

The navigation core is heading-based, not geometry-based:

1. **Arm.** Gyro bias auto-zeroes while stationary — rule 9.6 guarantees the car is placed
   switched off, so this is firmware self-calibration and not the team calibration
   forbidden by rule 9.9.
2. **Centre.** The L90 and R90 time-of-flight sensors equalise lateral error, then the
   heading is latched as reference.
3. **Straight.** The IMU holds heading. The encoder integrates distance. The front ToF
   watches closing distance as a backstop.
4. **Line event.** The floor colour sensor sees an orange or blue corner line.
5. **Turn.** Steer to lock, integrate yaw, **terminate the turn on measured heading**, then
   re-reference heading to the nearest 90°.
6. Repeat for **12 turns** (3 laps × 4 corners), then stop inside the starting section.

Two details in step 4 are not optional, and both were easy to miss:

- **Event lockout.** Every corner carries *both* an orange and a blue line (rule 13.9), so
  a naive "saw a colour" trigger fires about 24 times over three laps rather than 12. All
  colour input is ignored from the moment a turn begins until it completes and the car has
  driven clear.
- **Direction decode.** The driving direction is drawn at random before each round
  (rule 9.3). The *order* of the first line pair — orange-then-blue versus
  blue-then-orange — sets the turn direction for all 12 turns. Without it, half of all runs
  steer the wrong way.

**Why the turn terminates on IMU heading rather than steering angle:** tyre slip corrupts
any angle-based estimate, and re-referencing at every corner bounds gyro drift to a single
straight — about 4 seconds at 0.70 m/s.

### Obstacle Challenge — Pi added for colour only

The Pi 4B and its fisheye camera classify red and green pillars and report over a
checksummed UART frame. The STM32 remains the master of vehicle safety: if no valid frame
arrives within a timeout, it continues on its own deterministic policy. **The Pi can never
stall the car.**

The Pi and its regulator sit on a physically separate harness that is unplugged for the
Open Challenge. That makes the "no Pi in the Open round" claim inspectable rather than a
software flag.

---

## Four findings that changed the design

### 1. The floor out-reflects the wall

The mat is white (rule 13.2) and every visible wall face is black (rules 13.4, 13.6). For
an infrared time-of-flight sensor, **the false target reflects better than the real one.**
Our sensors are VL53L0X, which — unlike the VL53L1X we originally wrote the plan around —
has a fixed field of view and **no programmable region of interest**. The beam cannot be
narrowed in firmware.

`electrical/collimator.py` sizes the mechanical fix. Each sensor gets a printed collimator
snout (a 2.5 × 10 × 20 mm slot) plus a **+2° upward mounting wedge**. The wedge exists
because the 46 mm front and 50 mm rear wheels give the chassis about 1.1° of nose-down
rake that every chassis-mounted sensor inherits — aimed at the floor. Together these push
the first floor return from 166 mm to **870 mm**, against a side wall at 442.5 mm.

This is still the **highest-risk untested item in the build.** The snout throws away
photons and the wall is already a poor NIR target. It gets bench-tested against black MDF
before we print five mounts.

### 2. The textbook parallel park does not fit

`src/sim/park_feasibility.py` checks the swept body polygon against both magenta
limitations. A symmetric reverse two-arc **fails by 25.6 mm** at our 35° lock: the front
outer corner strikes the entry limitation at roughly 37° of heading.

The important part is that **the problem is scale-invariant**. The bay is always
1.5 × car length, so the slack is always 0.5 × car length — shortening the car cannot help,
because the bay shrinks with it. The only levers are the ratio of turn radius to car length
and the manoeuvre itself.

We tried raising the steering lock to 40° and re-ran it. It still fails, by 10.8 mm. A
two-arc needs about R/L ≤ 0.70, which takes 45°, which clears by 0.1 mm — and 0.1 mm of
theoretical clearance on an open-loop steering system is not clearance at all. So the
lock stayed at the as-built 35°, where the linkage actually reaches.

So parking is a **multi-point shuffle closed on IMU yaw**. Slower, but robust to
turn-radius error, and since the steering has no position feedback that error is real.

### 3. We built the wrong steering first

We started with a **single central pivot** — the whole front axle rotating on one bearinged
shaft. We chose it deliberately, with analysis behind it: a kinematic slop Monte-Carlo and
a pymunk friction simulation (`src/sim/`, written up in
`journal/steering-study-2026-07-12.md`) said the single pivot was about **three times more
robust to build slop** than Ackermann, with one joint instead of five. For a team whose
mechanical assemblies sometimes come loose, that looked decisive.

Then we built it, and it was wrong.

When the whole axle rotates about a central point, each front wheel translates fore and aft
by `(track/2) × sin δ`. At our 105 mm track and 35° lock that is **±30.1 mm per wheel — 36%
of the 82.5 mm of parking slack**, spent for zero navigational benefit, against a rule where
touching a magenta limitation scores zero parking points (rule 9.24.7). It also sat high,
raising the centre of gravity we had spent a whole optimisation pass lowering.

**Why the simulation missed it:** `steering_study.py` scored candidates on kinematic scrub,
parking accuracy and slop robustness. It never modelled the **swept envelope of the vehicle
body during the turn**. That missing objective term is the entire reason it ranked the
parallel bell-crank as "strictly dominated". The build found the term the model was missing.

We record this as an iteration, not an error. The original study and its conclusion are
preserved unedited.

**What we'd tell another team:** simulate, but build the thing early. Our simulation was
good work and we'd do it again. It just couldn't see the failure mode that mattered.

### 4. The AS5600 that isn't there

Our original plan put an **AS5600 magnetic encoder** on the steering shaft to close the loop
on true wheel angle, killing servo backlash. When we moved to the parallelogram that became
unbuildable — an AS5600 measures the absolute angle of *one rotating shaft*, and a
parallelogram doesn't have one.

Losing it turned out not to matter, and understanding why is the interesting part. Steering
angle was always a feedforward quantity in this design. Nothing in the control loop ever
*depended* on it being accurate, because turns terminate on measured heading. The loop that
mattered was closed through the gyro from the beginning. We had designed a sensor to solve a
problem our own navigation strategy had already solved.

The residual exposure is parking, which is why that manoeuvre is closed on IMU yaw too.

---

## Electronics

The car runs on **two stacked single-sided PCBs**, made locally in Dhaka. Full detail in
[`electrical/`](electrical/).

Single-sided is a real constraint and it shaped more than we expected. The local shop's
double-sided process has no plated through-holes, which means a two-layer board isn't
usable and **there is no ground plane at any layer count.** Minimum trace and space is
0.5 mm, so a routing channel is 1.2 mm — roughly four times what a commercial fab needs.
The consequence that drives everything: **you cannot route a trace between two adjacent
header pins.** 0.54 mm available, 1.8 mm required. Every net routes around every connector,
and placement rather than routing becomes the whole design problem.

- **Board A (upper):** STM32, motor driver, power rails, servo, encoder, Pi UART, IMU
- **Board B (lower):** the I²C multiplexer and all its sensor channels, at 3.3 V
- Linked by a 40 mm 6-pin cable

High current never crosses the PCB. Battery, motor and servo power are wired point to point
through screw terminals, because with no plated holes every pad has one-sided adhesion and
cable strain lifts pads — an intermittent failure, which is the worst kind.

### Power — four domains, one star ground

| Rail | Feeds |
|---|---|
| Motor | BTS7960 → 25GA → 5:1 → solid rear axle |
| 6.0 V | Steering servo, **and nothing else** |
| 5.0 V | Logic — STM32, and Board B via a local 3.3 V regulator |
| 5.1 V | Raspberry Pi (physically unplugged for the Open round) |

The servo getting its own rail is not fussiness. The MG996R is an analog servo that pulls
about 2.5 A at stall, in a step under 5 milliseconds, and it does that every single time
the car steers. Share that rail with the STM32 and you get voltage dips perfectly
correlated with steering, showing up as random sensor glitches and random resets — the kind
of bug that takes a week to find because it only happens when the car is moving.

### One sensor per mux channel

Every VL53L0X and the TCS34725 ship at the same I²C address, 0x29. Our first answer was to
reassign the ToF sensors to 0x30–0x34 at boot using five XSHUT lines, and give the colour
sensor its own bus. That worked, but reassigned addresses are volatile — any brownout loses
them — and XSHUT is 2.8 V logic on a bare die, so the correct wiring depends on which
breakout variant you happened to buy.

Putting a **PCA9548A multiplexer** in front instead means no two devices are ever on the bus
at once, so every sensor keeps its factory address and none of that exists. It also gives
back five GPIO.

The honest cost: the mux is now a single point of failure for every I²C sensor, and the
colour sensor — which is the turn trigger, and the most critical sensor in the Open round —
lost the private bus that used to isolate it from ToF cable faults. That's why the mux reset
line is wired to the STM32 rather than tied high.

**No Dupont jumpers anywhere on the robot, and no breadboard.** Everything is soldered or
crimped into JST-XH with strain relief. In our experience that single rule eliminates more
"random" faults than anything else on this list.

---

## Reproducing the analysis

```bash
python3 -m pip install matplotlib numpy

# ToF collimator sizing — floor return vs slot geometry, rake and wedge
python3 electrical/collimator.py --plot

# Turn radius and park ratio vs wheelbase and steering lock
python3 src/sim/geometry_sweep.py
python3 src/sim/geometry_sweep.py --wheelbase 110 --plot

# Parallel-park feasibility — swept polygon vs both magenta limitations
python3 src/sim/park_feasibility.py --wheelbase 110 --plot

# Regenerate the wiring block diagram
python3 electrical/make_block_diagram.py
```

`park_feasibility.py` and `geometry_sweep.py` both deliberately have **no default
wheelbase**. The measurement is still outstanding, and they error rather than silently
assume a number. Note that **105 mm is the track, not the wheelbase** — we have confused
those two once already, and it put our recorded turn radius out by 45%.

---

## Repository layout

```
SPECSHEET.md          locked parameters — source of truth
DECISIONS.md          every decision + reason + dated supersessions
BOM.md                parts, status, what is still unsourced
electrical/           ELECTRICAL.md, DESIGN_RULES.md, collimator solver, diagram generator
schemes/              wiring block diagram (PNG)
src/sim/              simulation studies behind the decisions
journal/              dated engineering log
models/               printable parts
media/                plots and figures
t-photos/ v-photos/   team and vehicle photos
video/                driving demonstration link
```

---

## Design principles

**Reliability before cleverness.** Ball bearings on every axle and pivot, because
plastic-on-shaft wears in and grows slop mid-competition. Parts located by geometry —
shoulders, slots, captured screws — with glue as backup and never as the primary locator.
Threadlocker on every thread. The working rule is that **a loose joint must still be unable
to shift**.

**No breadboards, no jumper wires.** Everything soldered or on latching connectors with
strain relief. Loose connections are the single largest source of behaviour that looks
random.

**Separate power domains.** Four rails joined at one star ground. A servo stall is a 2.5 A
step in under 5 milliseconds and will brown out a shared logic rail.

**Measure, don't assume.** Where a number is not yet measured it is marked open rather than
guessed. Straight-line trim is found by driving and bisecting gyro yaw-rate to zero, not by
eye — no car is mechanically straight, and that matters more now that steering is open-loop.

---

## Known open items

| Item | Blocks |
|---|---|
| **Competition SPI IMU** — not ordered; the MPU6050 on hand is bench-only | The racing gyro path |
| Encoder counts per motor revolution (hand-rotate test) | Odometry constant |
| VL53L0X signal rate against black MDF with the snout fitted | The entire distance-sensing approach |
| Battery connector — JST is under-rated for ~8 A peak; XT30 is specified | Power harness |
| 25GA stall current | Power budget |

These are tracked with dates in `journal/` and in `BOM.md`.

---

## Attribution

All design, simulation, firmware and documentation in this repository is the team's own
work. Public repositories from previous seasons were **studied for approach, never copied**
— in particular KMIDS-GFM 2025 and Nerdvana Taurus 2025, the latter of which carries a
restrictive licence and whose files are deliberately not reused. Where a technique is
borrowed conceptually it is named in `DECISIONS.md`.

This repository will remain public for at least one year after the event, as required by
chapter 7 of the rules.
