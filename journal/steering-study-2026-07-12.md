# Steering mechanism study — which is most robust for a full parallel park (2026-07-12)

**Question:** single central pivot vs a two-knuckle linkage — which gives the most
robust, repeatable full parallel park? Decided with a kinematic + joint-slop
Monte-Carlo model, not opinion. Code: `src/sim/steering_study.py`. Plots: `media/steering/`.

**Method.** 4-wheel kinematic model at v1 geometry (wheelbase L=120, front track
T=95 mm, target R≈130). Each mechanism is *calibrated* first — its outer loops
(gyro heading-hold + encoder distance + AS5600 angle) null the nominal bias, which
is what actually happens on the car — then stressed with Monte-Carlo joint slop
(200k reverse two-arc parks each). Metric = 95th-percentile lateral parking error
(curb-side wheels must finish within 20 mm of the wall to score "parallel").

## Candidates
- **TT** — Turntable / single central pivot: rigid front axle, both wheels equal angle, **1 pivot joint**. AS5600 on the pivot sees the true axle angle.
- **PAR** — Parallel bell-crank: 2 kingpin knuckles + tie-rod, equal angles, **~5 joints**.
- **ACK** — Ackermann bell-crank: 2 kingpin knuckles, arms angled for Ackermann, **~5 joints**.

## Results
**1. Scrub (tyre fighting) — the tight-turn penalty of equal-angle steering.**
At the tight radii a parallel park needs (R/T ≈ 1.5), forcing both front wheels to
the same angle scrubs badly:

| Commanded centre angle | TT / PAR scrub | ACK scrub |
|---|---|---|
| 20° | 2.5° | 0° |
| 40° (full lock) | **8.4°** | **0°** |
| 45° | 10.1° | 0° |

Realized turn radius at 40° lock: TT/PAR = 167 mm, ACK = 143 mm. Ackermann removes
scrub entirely by giving the inner wheel 51.5° and the outer 32.2°.

**2. Robustness to joint slop (95th-pct lateral park error).**
- **TT: 1.76 mm, essentially slop-independent** — its one bearinged pivot is what the AS5600 measures, so there is almost no *unobserved* slop.
- **ACK/PAR: error grows linearly with unobserved downstream slop** (the 2 link ball-joints + kingpin per wheel that the central AS5600 cannot see). They beat TT only when downstream slop is below **~0.22°**; at 0.5° → 3.7 mm, at 1.0° → 7.4 mm. All stay under the 20 mm "parallel" limit until slop ≈ 2.6°.

**3. Parallel bell-crank is strictly dominated** — it carries TT's full 8.4° scrub
*and* ACK's 5-joint slop exposure. No reason to build it. Drop it.

## Honest caveat (reads in ACK's favour)
The Monte-Carlo *calibrates out* TT's nominal scrub bias, so TT's 1.76 mm is
optimistic. In reality 8° of front-wheel scrub is friction- and surface-dependent,
is not perfectly repeatable, and makes the drive wheels slip — which corrupts
*odometry*, the very thing parking distance control relies on. So TT's true
repeatability is worse than 1.76 mm, and the 0.22° crossover *understates*
Ackermann's real advantage.

## Verdict
1. **Drop Parallel bell-crank** (dominated).
2. **Build Ackermann bell-crank as the v1 target** — it is the only option with zero
   scrub, and scrub is itself a randomness source the "no-randomness" spec should
   reject. Your existing CAD (two bearing knuckles + tie-rod) is already the right
   hardware; it needs (a) the steering arms angled for Ackermann, (b) a stiff
   tie-rod (not the 3 mm flat plate), (c) bearinged/preloaded joints.
3. **Earn it mechanically.** Ackermann only wins if downstream slop is kept low.
   Targets: bearinged kingpins, zero-backlash (preloaded or flexure) links, tie-rod
   stiff in bending/torsion, and mount the AS5600 to read a **knuckle** (or as close
   to the wheels as possible) so more slop is *observed* by the loop. Budget:
   **downstream slop < 0.3–0.5°.**
4. **Fallback gate (measure, don't guess):** print the mechanism, measure actual
   backlash at the wheel. If you cannot get under ~0.5°, fall back to the turntable
   (slop-immune, novice-proof) and accept the scrub. Decision is made on a measured
   number, on the bench.

## Build numbers to model in Fusion
- Ackermann steering-arm angle (knuckle arm pointing at rear-axle centre):
  `atan((T/2)/L) = atan(47.5/120) ≈ 21.6°`.
- Full-lock wheel angles to hit R=143 mm: inner **51.5°**, outer **32.2°**.
- Keep sensors/camera on the fixed chassis (unchanged rule).

## Optional follow-up
A full rigid-body sim (PyBullet/MuJoCo) would add tyre-friction and tip dynamics,
but it will not change this geometry verdict — it would only further penalise TT's
scrub. Worth doing later only if we want to tune drive speed/traction, not to
decide the mechanism.

---

# Dynamic validation (pymunk 2D rigid-body) — added 2026-07-12

PyBullet would not build on the dev machine (no MSVC C++ toolchain; no prebuilt
wheel for Python 3.10/Win64). Used **pymunk** instead (2D rigid-body, installs as a
binary wheel). Top-down car, each tyre applies a lateral grip impulse capped by
friction (mu * load); when a mechanism demands more side force than the tyre can
give (scrub), the tyre SLIPS — the effect the kinematic model idealised away.
Code: `src/sim/steering_dyn.py`. Plot: `media/steering/4_dynamics.png`.

**Test:** reverse two-arc park; sweep surface friction mu = 0.6–1.2; inject built
joint slop (TT 0.15° correlated, ACK 0.5° per wheel). Metric = how far the parked
position wanders with surface (bias, calibratable) and run-to-run (slop, not).

**Results:**
- Surface-friction sensitivity: **TT 2.7 mm vs ACK 2.9 mm span — essentially equal.**
  At parking speed the turntable's 8° scrub does NOT become a large path error; the
  tyres absorb it as small distributed slip. Scrub is cheap at low speed.
- Run-to-run scatter from build slop (@0.5°): **TT 0.42 mm vs ACK 1.25 mm** — the
  single pivot is ~3× tighter, because its one bearinged joint is what the AS5600
  measures (nothing unobserved).

**This revises the kinematic verdict.** The kinematic model overstated Ackermann by
treating scrub as a hard error. Under real low-speed dynamics, scrub barely matters
and build-slop robustness dominates.

## Combined verdict (kinematic + dynamic)
- **For the parallel park (low speed): single central pivot / turntable WINS** —
  equal accuracy, ~3× better robustness to build slop, and 1 joint vs 5 (decisive
  for a novice assembler). This **vindicates the originally-locked Decision #4.**
- **Ackermann's zero-scrub only clearly pays off at higher speed** (open/obstacle
  laps, lateral force ∝ v²/R, tyres saturate) — NOT YET SIMULATED. It also only
  beats the pivot on slop if downstream joints are built to <~0.2–0.3°.
- **Parallel bell-crank: still dominated. Drop it.**

## Recommendation
Keep **single central pivot** locked for v1 (Decision #4 stands, now evidence-backed
for parking). Before finalising, optionally simulate a HIGH-SPEED corner to see if
scrub costs enough on the speed laps to justify Ackermann's extra joints. If built,
Ackermann must hit <0.3° downstream slop and mount the AS5600 on a knuckle.

## Caveats (honesty)
Both sims use assumed numbers (mass split, mat mu, drive authority, 0.5° slop). The
qualitative conclusions (low-speed scrub is cheap; single pivot is slop-robust) are
solid; exact mm values are not gospel. Real decision is confirmed on the mat.
