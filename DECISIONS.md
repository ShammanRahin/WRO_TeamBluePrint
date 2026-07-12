# DECISIONS — locked, with reasons

One line per locked decision + the REASON. Source of truth; append as things lock.

1. **Nav = gyro heading-hold, 90° turns terminated by IMU, re-reference heading at every corner** — turn ends on measured heading not steering angle, so tyre slip can't corrupt it; re-referencing bounds gyro drift. Mirrors the previous BD national winner (fastest + smoothest).

2. **Compute split: Open round = STM32 only; Obstacle round adds Pi + fisheye for pillar colour only; link = checksummed UART, never inter-board I2C** — keeps the driving loop fully deterministic with no Pi/WiFi jitter; I2C between boards is a known glitch source, UART with checksum is robust.

3. **STM32 over ESP32** — clean 12-bit ADC for analog Sharp IR (ESP32 ADC noise becomes distance randomness), hardware quadrature encoder timers, deterministic timing with no WiFi stack stealing cycles.

4. **Steering = single central pivot, bell-crank, bearing-supported shaft, AS5600 encoder on the shaft, servo commanded in microseconds** — bell-crank is the documented RC standard (turntable swings wheels fore/aft + long lever arm for bump loads); AS5600 closes the loop on true wheel angle and kills servo backlash; µs commanding avoids the ~10 µs/deg quantisation of degree commands.
   - **EVIDENCE 2026-07-12 (single central pivot CONFIRMED for parking):** a kinematic slop Monte-Carlo + a pymunk dynamic-friction sim (`src/sim/`, `journal/steering-study-2026-07-12.md`) compared single central pivot (turntable) vs Ackermann bell-crank vs parallel bell-crank for the parallel park. Findings: (a) equal-angle scrub is 8° at full lock kinematically BUT dynamically cheap at parking speed (surface sensitivity ~equal, TT 2.7 mm vs ACK 2.9 mm); (b) single pivot is ~3× more robust to build slop (0.42 vs 1.25 mm run-to-run @0.5° joint slop) because its one bearinged joint is exactly what the AS5600 measures; (c) 1 joint vs 5 — decisive for a novice assembler. Parallel bell-crank is strictly dominated → dropped. Ackermann only pays off at higher speed (untested) or with downstream joints built <0.3°. Net: single central pivot stays locked for v1.

5. **Ackermann rejected for v1** — 4–6 slop-prone joints, worst choice for a novice assembler; analyse the trade-off in the journal for rubric points instead of building it.

6. **"Straight" found by motion, not by eye (sweep µs, drive ~3 m, bisect gyro yaw-rate to zero → CENTER_US)** — no car is mechanically straight; gyro heading-hold finds the true straight, removing a source of randomness.

7. **Sensors = analog Sharp IR (calibrated, median-filtered, mounted out of the fold-back zone) and/or front sonar; ToF rejected; LIDAR deferred** — walls are big flat perpendicular reflectors ideal for IR/sonar; ToF randomly false-detects the floor; LIDAR is heavy/tall and forces Pi 5 + Linux jitter against the determinism goal (design upper deck to accept it later).
   - **REVISION 2026-07-12 (team override):** primary distance = **ToF VL53L1X ×5** (already on hand). This reverses the "ToF rejected" clause above. Reason: parts owned; VL53L1X is far better than the generic ToF the rejection referred to. RISK acknowledged (false floor returns on a low car) → mitigate by above-grazing-angle mount, tight ROI window, min-range + floor gating, median filter, and VALIDATE on the real mat. Sharp IR / sonar demoted to documented fallback (Sharp bought only if ToF fails on the mat). Original rejection reasoning kept above so the trade-off is on record for the journal.

8. **Sensors + camera mount to the FIXED chassis, never the pivoting steering beam** — readings must not swing when the steering turns.

9. **Ball bearings on every axle AND the steering pivot** — plastic-on-shaft wears in and grows slop mid-competition; bearings keep geometry constant.

10. **Locate every part by GEOMETRY (shoulders, slots, captured screws); glue is backup only** — a loose joint must still be unable to shift, because the assembler is a novice and joints sometimes come loose.

11. **No Dupont jumpers / no breadboard on the robot — solder or JST/screw terminals with strain relief** — jumpers/breadboards are the #1 source of "random" glitches.

12. **Separate motor + logic power domains joined at ONE star ground; bulk cap 470–1000 µF on motor rail, 0.1 µF near MCU, 0.1 µF across motor terminals; threadlocker on every thread; twisted encoder leads with pull-ups** — motor brown-out causes random MCU resets and EMI causes random glitches; these kill both.

13. **Deliverable of the design phase = a justified, print-ready v1 — NOT a finished car** — "competition-level" is earned by testing and iterating on a real mat in Sept–Oct, not in chat.
