# Team Blueprint — WRO Future Engineers 2026

Engineering materials for an autonomous self-driving vehicle built for the
**World Robot Olympiad Future Engineers** category, 2026 season.
Bangladesh · Islamic University of Technology.

---

## 1. What this repository is

This is the working record of the vehicle: the decisions, the reasons behind them, the
simulations used to test those reasons, and the points at which the build proved a
decision wrong and we changed it.

Three files are the source of truth:

| File | Contents |
|---|---|
| **`SPECSHEET.md`** | Every locked parameter — geometry, drivetrain, sensors, electrical |
| **`DECISIONS.md`** | One entry per locked decision **with its reason**, plus dated supersession blocks where the build overruled the design |
| **`BOM.md`** | Parts, status, and what is still unsourced |

Everything else supports those three. The journal in `journal/` is chronological; the
simulations in `src/sim/` are the evidence behind the decisions.

**Superseded decisions are never deleted.** When the build contradicted a design choice,
the original reasoning stays visible and a dated **SUPERSEDED** block is added beneath it
explaining what changed and why. Two of these are load-bearing (steering mechanism and
parking manoeuvre) and both are documented in
`journal/day-02-plan-revision-2026-07-26.md`.

---

## 2. The vehicle

| Parameter | Value |
|---|---|
| Scored footprint | **165 × 115 mm** (limit 300 × 200 mm) |
| Height | 50 mm as built; ~75–90 mm with the obstacle-round stack (limit 300 mm) |
| Track | 105 mm centre-to-centre, 115 mm wheel extremes |
| Wheels | 46 mm front, 50 mm rear |
| Steering | Parallelogram tie-bar, **±35° measured**, one MG996R servo |
| Drive | One 25GA gearmotor → **5:1 gear** → **solid rear axle**, no differential |
| Top speed | **0.70 m/s** |
| Odometry | **0.175 mm/count** |
| Control | STM32F411CEU6 |
| Vision | Raspberry Pi 4B + 160° fisheye — **obstacle round only** |

### Rule compliance
Four wheels, one driven axle, one steering actuator (rule 11.3). One drive motor,
mechanically coupled through a gearbox to the axle — never independently driven
(rules 11.5, 11.13). No wireless of any kind during runs; the Pi's WiFi and Bluetooth are
disabled in `config.txt` (rule 11.10). **One** master switch powers the vehicle on
(rule 9.10) and a **separate** momentary button starts the program (rule 9.11).

---

## 3. How it drives

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

Two details in step 4 are not optional and were easy to miss:

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

## 4. Two findings that changed the design

### The floor out-reflects the wall

The mat is white (rule 13.2) and every visible wall face is black (rules 13.4, 13.6). For
an infrared time-of-flight sensor, **the false target reflects better than the real one.**
Our sensors are VL53L0X, which — unlike the VL53L1X — has a fixed field of view and **no
programmable region of interest**, so the beam cannot be narrowed in firmware.

`electrical/collimator.py` sizes the mechanical fix. Each sensor gets a printed collimator
snout (a 2.5 × 10 × 20 mm slot) plus a **+2° upward mounting wedge**. The wedge exists
because the 46 mm front and 50 mm rear wheels give the chassis about 1.1° of nose-down
rake that every chassis-mounted sensor inherits — aimed at the floor. Together these push
the first floor return from 166 mm to **870 mm**, against a side wall at 442.5 mm.

### The textbook parallel park does not fit

`src/sim/park_feasibility.py` checks the swept body polygon against both magenta
limitations. A symmetric reverse two-arc park **fails by 25.6 mm** at our 35° lock: the
front outer corner strikes the entry limitation at roughly 37° of heading.

The important part is that **the problem is scale-invariant**. The bay is always
1.5 × car length, so the slack is always 0.5 × car length — shortening the car cannot help,
because the bay shrinks with it. The only levers are the ratio of turn radius to car
length (ours is 0.95; a two-arc needs roughly ≤0.7) and the manoeuvre itself. The parking
strategy is therefore a **multi-point shuffle closed on IMU yaw**, which is slower but
robust to turn-radius error — and since the steering is open-loop, that error is real.

---

## 5. Repository layout

```
SPECSHEET.md          locked parameters — source of truth
DECISIONS.md          every decision + reason + dated supersessions
BOM.md                parts, status, what is still unsourced
electrical/           ELECTRICAL.md, collimator solver, diagram generator
schemes/              wiring block diagram (PNG)
src/sim/              simulation studies behind the decisions
journal/              dated engineering log
models/ 3D-models/    printable parts
media/                plots and figures
t-photos/ v-photos/   team and vehicle photos
video/                driving demonstration link
```

### Reproducing the analysis

```bash
python3 -m pip install matplotlib numpy

# ToF collimator sizing — floor return vs slot geometry, rake and wedge
python3 electrical/collimator.py --plot

# Parallel-park feasibility — swept polygon vs both magenta limitations
python3 src/sim/park_feasibility.py --wheelbase 110 --plot

# Regenerate the wiring block diagram
python3 electrical/make_block_diagram.py
```

`park_feasibility.py` deliberately has **no default wheelbase**. The measurement is still
outstanding, and the script errors rather than silently assuming a number.

---

## 6. Design principles

**Reliability before cleverness.** Ball bearings on every axle and pivot, because
plastic-on-shaft wears in and grows slop mid-competition. Parts located by geometry —
shoulders, slots, captured screws — with glue as backup and never as the primary locator.
Threadlocker on every thread. The working rule is that **a loose joint must still be
unable to shift**.

**No breadboards, no jumper wires.** Everything is soldered or on latching connectors with
strain relief. Loose connections are the single largest source of behaviour that looks
random.

**Separate power domains.** Four rails joined at one star ground: motor, servo, logic, and
compute. A servo stall is a 2.5 A step in under 5 milliseconds and will brown out a shared
logic rail. Full budget in `electrical/ELECTRICAL.md`.

**Measure, don't assume.** Where a number is not yet measured it is marked as open rather
than guessed. Straight-line trim is found by driving and bisecting gyro yaw-rate to zero,
not by eye — no car is mechanically straight.

---

## 7. Known open items

| Item | Blocks |
|---|---|
| Wheelbase, front axle centre to rear axle centre | Turn radius, park feasibility |
| Encoder counts per motor revolution (hand-rotate test) | Odometry constant |
| VL53L0X signal rate against black MDF with the snout fitted | The entire distance-sensing approach |
| MPU9250 `WHO_AM_I` value | IMU driver |
| 25GA stall current | Power budget |

These are tracked with dates in `journal/` and in `BOM.md`.

---

## 8. Attribution

All design, simulation, firmware and documentation in this repository is the team's own
work. Public repositories from previous seasons were **studied for approach, never copied**
— in particular KMIDS-GFM 2025 and Nerdvana Taurus 2025, the latter of which carries a
restrictive licence and whose files are deliberately not reused. Where a technique is
borrowed conceptually it is named in `DECISIONS.md`.

This repository will remain public for at least one year after the event, as required by
chapter 7 of the rules.
