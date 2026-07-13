# Day 01 — steering verdict, layout optimizer, locked v1 geometry (2026-07-12)

## Steering mechanism decided (evidence-based)
Compared single central pivot (turntable) vs Ackermann bell-crank vs parallel
bell-crank for the parallel park with a kinematic slop Monte-Carlo AND a pymunk
dynamic-friction sim (`src/sim/steering_study.py`, `steering_dyn.py`; plots in
`media/steering/`). Result: **single central pivot wins for parking** — equal
accuracy, ~3× better robustness to build slop (its one bearinged joint is what the
AS5600 measures), and 1 joint vs 5. Parallel bell-crank dominated → dropped.
Ackermann only pays off at higher speed / with joints built <0.3°. Full write-up:
`journal/steering-study-2026-07-12.md`. Decision #4 reaffirmed with the evidence.

## Layout optimizer (objective: lowest CG / most stable)
`src/sim/optimize_layout.py` swept track / wheelbase / wheel-dia over 455 feasible
designs against WRO rules, BOM+PLA mass, turn radius, parking-bay fit, and fisheye
camera coverage. Key findings:
- Tip-stability is huge everywhere (55°+), so the car cannot roll at parking speed —
  stability is NOT the binding constraint; parking-width margin is. So track is set
  to a balanced 120 mm, not the raw optimum 140 mm.
- **The MG996R servo on the deck is 28% of the CG moment — the #1 CG driver.** Mount
  it LOW. Pi 4B is #2 (18%). Battery-in-front-of-motor (80 g, low) is good — keep it.
- A 160° fisheye covers the 1 m lane a car-length ahead trivially; camera height is
  set only by clearing the component stack (~90 mm mast), ~free in CG terms.
- Estimated mass ~450 g (BOM + PLA at ~35% effective infill) — far under 1.5 kg.

## Discrepancies vs prev design "WRo final v5"
- Its 165×80 footprint is very NARROW (80 mm track) → tippy end of range. Widened.
- Its mass readout was default-Steel (fake ~2 kg). Assign real materials for a true number.
- Servo in the model was MG996R (correct) not the earlier-noted MG90S — records fixed.

## LOCKED v1 geometry (now in SPECSHEET §3 + Fusion global parameters)
track 120 · wheelbase 128 (R=128) · wheel_dia 40 · lock ±45° · L×W×H 175×138×95 ·
~450 g · CG ~28 mm · camera mast ~90 mm. Blueprint: `media/car_blueprint.svg`.

## Tooling note
PyBullet could not be installed here (no prebuilt wheel, no MSVC C++ toolchain, no
winget). Dynamics done in pymunk instead — correct tool for this geometry/mass
question anyway; a full rigid-body engine would not change the verdict.

## Next
Shanto models the chassis + steering in Fusion against the locked parameters. Then:
confirm encoder count, assign real materials for true mass/CG, verify the bell-crank
link delivers ±45° without a toggle point, and test on the real mat.
