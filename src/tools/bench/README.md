# bench

Minimal sketches used during bring-up to prove one subsystem at a time before
anything was wired together. Keep them: when something stops working on the
car, the fastest route to the cause is proving the part in isolation again.

| Sketch | Proves |
|---|---|
| `HallEncoderTester.cpp` | TIM3 hardware quadrature decoding on PA6/PA7 |
| `RearWheelDriveTester.cpp` | BTS7960 direction and PWM |
| `TCS34725_ColorSensorTester.cpp` | Colour sensor reachable behind the TCA9548A mux |
| `tof_test.ino` | Current carrier: three VL53L0X on the mux — front (ch 3) plus two new side sensors (ch 5, ch 6) — read non-blocking, with loop rate. Checks each sensor responds, agrees with a tape measure at 200 / 350 / 700 mm, and reads out-of-range down an open straight |

> [`tof_color_sensor_routine.txt`](tof_color_sensor_routine.txt) describes the earlier version of `tof_test.ino` (live colour stream + o/b/w threshold capture), which was replaced on 2026-09-18 by the three-ToF test. The colour-capture sketch is no longer in the repo; recover it from git history (commit `a6f685e`) if you need to re-measure the floor thresholds.

For measurement rather than bring-up, use
[`../calibration/`](../calibration/) instead.

> `RearWheelDriveTester.cpp` declares RPWM = PB8 / LPWM = PB9, while
> `ObstacleRound.cpp` has them the other way round — and the current carrier
> (`OpenRound.cpp`) has moved the motor to PA2 (forward) / PA3 (reverse),
> bench-confirmed. The older testers here use the earlier pin map.
