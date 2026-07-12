# DECISIONS — locked, with reasons

One line per locked decision + the REASON. Source of truth; append as things lock.

1. **Nav = gyro heading-hold, 90° turns terminated by IMU, re-reference heading at every corner** — turn ends on measured heading not steering angle, so tyre slip can't corrupt it; re-referencing bounds gyro drift. Mirrors the previous BD national winner (fastest + smoothest).

2. **Compute split: Open round = STM32 only; Obstacle round adds Pi + fisheye for pillar colour only; link = checksummed UART, never inter-board I2C** — keeps the driving loop fully deterministic with no Pi/WiFi jitter; I2C between boards is a known glitch source, UART with checksum is robust.

3. **STM32 over ESP32** — clean 12-bit ADC for analog Sharp IR (ESP32 ADC noise becomes distance randomness), hardware quadrature encoder timers, deterministic timing with no WiFi stack stealing cycles.

4. **Steering = single central pivot, bell-crank, bearing-supported shaft, AS5600 encoder on the shaft, servo commanded in microseconds** — bell-crank is the documented RC standard (turntable swings wheels fore/aft + long lever arm for bump loads); AS5600 closes the loop on true wheel angle and kills servo backlash; µs commanding avoids the ~10 µs/deg quantisation of degree commands.

5. **Ackermann rejected for v1** — 4–6 slop-prone joints, worst choice for a novice assembler; analyse the trade-off in the journal for rubric points instead of building it.

6. **"Straight" found by motion, not by eye (sweep µs, drive ~3 m, bisect gyro yaw-rate to zero → CENTER_US)** — no car is mechanically straight; gyro heading-hold finds the true straight, removing a source of randomness.

7. **Sensors = analog Sharp IR (calibrated, median-filtered, mounted out of the fold-back zone) and/or front sonar; ToF rejected; LIDAR deferred** — walls are big flat perpendicular reflectors ideal for IR/sonar; ToF randomly false-detects the floor; LIDAR is heavy/tall and forces Pi 5 + Linux jitter against the determinism goal (design upper deck to accept it later).

8. **Sensors + camera mount to the FIXED chassis, never the pivoting steering beam** — readings must not swing when the steering turns.

9. **Ball bearings on every axle AND the steering pivot** — plastic-on-shaft wears in and grows slop mid-competition; bearings keep geometry constant.

10. **Locate every part by GEOMETRY (shoulders, slots, captured screws); glue is backup only** — a loose joint must still be unable to shift, because the assembler is a novice and joints sometimes come loose.

11. **No Dupont jumpers / no breadboard on the robot — solder or JST/screw terminals with strain relief** — jumpers/breadboards are the #1 source of "random" glitches.

12. **Separate motor + logic power domains joined at ONE star ground; bulk cap 470–1000 µF on motor rail, 0.1 µF near MCU, 0.1 µF across motor terminals; threadlocker on every thread; twisted encoder leads with pull-ups** — motor brown-out causes random MCU resets and EMI causes random glitches; these kill both.

13. **Deliverable of the design phase = a justified, print-ready v1 — NOT a finished car** — "competition-level" is earned by testing and iterating on a real mat in Sept–Oct, not in chat.
