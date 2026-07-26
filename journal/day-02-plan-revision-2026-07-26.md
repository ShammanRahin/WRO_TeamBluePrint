# Day 02 — as-built revision, electrical architecture, two blocking findings (2026-07-26)

The car has been built, and it diverges from the 2026-07-12 design in four significant
ways. This entry records the divergence, the reasons, and two problems the build exposed
that the design phase had not anticipated.

Nothing from the original design has been deleted. Every conflict is marked
**SUPERSEDED** with a dated block in `DECISIONS.md`, and the superseded reasoning stays
visible above it. A repository that shows *why* a design changed is stronger evidence of
engineering than one that only ever shows the final answer.

---

## 1. What changed from design to build

| Item | Designed 2026-07-12 | Built | Record |
|---|---|---|---|
| Steering | single central pivot + AS5600 closed loop | **parallelogram tie-bar, open-loop** | Decision #4 superseded, #15, #16 |
| Geometry | track 120 / WB 128 / 175×138×95 / wheel 40 | **track 105 (115 outer) / L 165 / wheels 46 F, 50 R** | SPECSHEET §3 |
| Distance sensor | VL53L1X ×5 | **VL53L0X ×5** | Decision #7 corrected, #18 |
| Turn trigger | front wall distance | **TCS34725 floor colour** | Decision #1 amended, #17 |
| Reduction | none specified | **5:1 to solid rear axle** | Decision #14 confirmed |
| Motor driver | TBD | **BTS7960** | Decision #22 |
| IMU | TBD | **MPU9250 on SPI** | Decision #20 |

---

## 2. Why parallelogram, when the simulation said otherwise

`journal/steering-study-2026-07-12.md` ranked parallel bell-crank as *strictly dominated*
and locked the single central pivot. The build went the other way.

**The reason, from assembly:** a central pivot rotates the whole front beam about one
axis, so during a turn one front wheel swings forward and the other swings back. The
car's effective length grows exactly when clearance is tightest.

**Quantified.** Each front wheel translates fore/aft by `(track / 2) × sin δ`. At the
as-built track of 105 mm and the measured 35° lock:

```
(105 / 2) × sin 35° = 30.1 mm per wheel
```

Against a parking bay of 1.5 × 165 mm = 247.5 mm with **82.5 mm of total longitudinal
slack**, that is **36% of the slack consumed for zero navigational benefit** — in a rule
where touching a magenta limitation scores 0 parking points (rule 9.24.7).

**The study was not wrong within its own objective function; the objective function was
incomplete.** `src/sim/steering_study.py` scored kinematic scrub, parking accuracy and
robustness to build slop. It never modelled the **swept envelope of the vehicle body
during a turn**. That missing term is the whole reason it ranked parallel bell-crank as
dominated. The build found the term the model was missing.

**Costs accepted and recorded:** 4+ linkage joints instead of 1 (against Decision #10's
novice-assembler principle); no AS5600, so **steering is fully open-loop**; equal-angle
steering, so scrub is unchanged from the turntable — a parallelogram is *not* Ackermann;
and **turn radius R is nominal, not measured**.

**Why it is still sound:** the loop is closed on *vehicle heading* by the IMU, not on
steering angle. Steering only has to be monotonic and repeatable, not accurate. This is
the same argument that already lets Decision #1 terminate turns on measured heading.

---

## 3. FINDING 1 — the VL53L0X has no ROI, so floor rejection must be mechanical

The 2026-07-12 ToF override assumed a VL53L1X, whose programmable SPAD ROI can narrow the
beam in firmware. **The parts on hand are VL53L0X: fixed ~25° FoV, no ROI.** That
mitigation does not exist on this part.

This matters because of the field colours: the mat is white (rule 13.2) and both exterior
and interior walls are black on every visible face (rules 13.4, 13.6). **The false target
reflects near-infrared better than the true target.** A naive closest-return read locks
onto the floor. This is the concrete form of the original "ToF randomly false-detects the
floor" objection that Decision #7 overrode.

`electrical/collimator.py` solves it. With `d_floor = h / tan(θ_eff)` and a side wall at
(1000 − 115)/2 = 442.5 mm:

| Configuration | θ_eff | d_floor @ h = 40 mm | Margin |
|---|---|---|---|
| Bare VL53L0X | 13.56° | 166 mm | −277 mm ✗ |
| Snout only | 4.63° | 494 mm | +51 mm — too thin |
| **Snout + 2° wedge** | **2.63°** | **870 mm** | **+428 mm ✓** |

**Spec: printed collimator snout, slot 2.5 mm tall × 10 mm wide × 20 mm deep, plus a +2°
upward mounting wedge, on all five sensors.** The slot is wide rather than round because
the L0X emitter and receiver apertures sit ~2.5 mm apart and a round hole vignettes the
receiver.

**The wedge is not decoration.** Front wheels are 46 mm and rear wheels are 50 mm, so the
chassis sits nose-down by `atan(2 / wheelbase)` ≈ **1.0–1.1°**, and every chassis-mounted
sensor inherits that — aimed at the floor. Without correction the margin drops to +51 mm,
which is not a margin.

**Duty split (Decision #19):** L90/R90 handle open-corridor centring at 442.5 mm.
FL30/FR30 handle obstacle-corridor centring when threading a pillar-to-wall gap, where
slant range = gap ÷ sin 30° = 2 × gap — a 150 mm gap reads at 300 mm, a 350 mm gap at
700 mm. They degrade exactly when the gap is wide, which is when L90/R90 take over. That
is a designed split of duties, not a limitation.

**Highest-risk open item in the build.** Bench-test signal rate against black MDF with
the snout fitted, at 442 mm and at 300/700 mm at 60° incidence, **before printing five
mounts**. The aperture throws away photons and the wall is already a poor NIR target. If
signal rate is marginal, the Sharp GP2Y0A21 fallback must be ordered immediately — and
its lead time is still unrecorded.

---

## 4. FINDING 2 — the two-arc parallel park is not feasible, and the problem is scale-invariant

`src/sim/park_feasibility.py` was written to prove the reverse two-arc park carried in
SPECSHEET since 2026-07-12. It disproves it.

The solver sweeps the parked longitudinal position — the only free variable in a
symmetric two-arc at fixed lock — and checks the swept body polygon against both
200 × 20 × 100 mm magenta limitations and the outer wall. The car is 50–90 mm tall and the
limitations are 100 mm tall, so it **cannot pass over them** at any point.

| Lock | R | R/L | Best clearance | Verdict |
|---|---|---|---|---|
| **35° (as built)** | 157 mm | 0.95 | **−25.6 mm** | **COLLISION** |
| 40° | 131 mm | 0.79 | −10.8 mm | COLLISION |
| 45° | 110 mm | 0.67 | +0.1 mm | clears by nothing |
| 60° | 64 mm | 0.38 | +0.5 mm | clears by nothing |

The failure mode is specific: the **front-outer corner** sweeps into the entry limitation
at ~37° heading, while the rear axle is still 164 mm along the bay. The rear clears; the
front does not.

**The problem is scale-invariant, which is the important part.** The bay is always
1.5 × car length, so the slack is always 0.5 × car length. **Shortening the car does not
help — the bay shrinks with it.** The only levers are the ratio **R/L** and the manoeuvre
strategy. A symmetric two-arc needs roughly R/L ≤ 0.7; the as-built car is at 0.95.

Raising the lock to 45° would reach R/L = 0.67 and clear by ~0.1 mm. That is not a margin,
and it is worthless with open-loop steering.

**Decision: the primary parking manoeuvre becomes a multi-point (3+) shuffle, closed on
IMU yaw** (Decision #21 amended). Enter at whatever angle the geometry allows, then
shuffle forward and back to straighten. Slower, but robust to R error — which matters
precisely because there is no steering feedback. Parking is 15 pts plus 7 pts for
finishing in the start section: **22 of 122**, so this is worth the time cost.

---

## 5. Electrical architecture

Written up in full in `electrical/ELECTRICAL.md`; block diagram in
`schemes/wiring_block_diagram.png` (`schemes/` was previously empty, which scores 0 on
rubric Criterion 2).

Two points worth calling out here:

**The 0x29 collision.** The VL53L0X and the TCS34725 both ship at I²C address 0x29, and
the TCS has neither a shutdown pin nor an address strap. During ToF enumeration each
sensor briefly appears at 0x29, so every enumeration write would also hit the colour
sensor. The buses are therefore split: five ToFs alone on I²C1 with XSHUT reassignment to
0x30–0x34, the TCS alone on I²C2, and the MPU9250 moved to SPI. This also isolates the
turn trigger — the most critical sensor in the Open round — from any ToF cable fault.

**The Open-round claim is physical, not a flag.** The Pi 4B and BEC-C are on a separate
harness that is unplugged for the Open Challenge. A judge can verify that the Open
configuration cannot be influenced by the Pi because the Pi is not electrically present.

---

## 6. The 5:1 reduction is the highest-value change in the build

| | Ungeared | With 5:1 |
|---|---|---|
| Top speed | 2.79 m/s | **0.70 m/s** |
| Odometry | 0.70 mm/count | **0.175 mm/count** |
| 20 mm line dwell | 7.2 ms | **28.6 ms** |

It converted three separate problems — uncontrollable speed, coarse odometry, and an
unreadably short colour-line window — into non-problems at once.

---

## 7. Still open

| # | Item | Blocks | Age |
|---|---|---|---|
| 1 | **BD national competition date** | all three commit deadlines | since 2026-07-12 |
| 2 | Wheelbase, front axle centre to rear axle centre | R, park solver | new |
| 3 | Encoder counts per motor rev (hand-rotate) | odometry constant | since 2026-07-12 |
| 4 | MPU9250 `WHO_AM_I` (0x71 / 0x70 / 0x73) | IMU driver | new |
| 5 | VL53L0X breakout — regulator + level shifters? | I²C pull-up rail | new |
| 6 | 25GA stall current | power budget in the journal | since 2026-07-12 |
| 7 | Sharp GP2Y0A21 lead time | ToF fallback path | since 2026-07-12 |

**Item 1 is the critical one.** It gates the first commit deadline (≥2 months pre-event,
≥1/5 of code). `src/` currently contains four simulation studies and **no firmware**.
Everything in this revision is documentation, which does not count as code.

---

## 8. Next

1. Measure the wheelbase; re-run `park_feasibility.py` and lock the parking strategy.
2. Bench-test VL53L0X signal rate against black MDF **before printing five snouts**.
3. Hand-rotate the encoder and close the odometry constant.
4. Get the national date, then schedule firmware against the three commit gates.
5. Begin firmware: HAL bring-up, ToF enumeration, IMU heading integrator, TCS edge
   detector with lockout and direction decode.
