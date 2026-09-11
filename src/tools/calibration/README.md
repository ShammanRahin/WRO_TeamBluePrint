# Calibration sketches

Eight standalone sketches, one per calibration step, plus a menu-driven suite
that runs all eight from one flash.

The full procedure — what rig you need, what to expect, how to read the output,
and what to do when it looks wrong — is in
[`docs/CALIBRATION.md`](../../../docs/CALIBRATION.md). Read that first. These
files are the tools; that page is the method.

| File | Step |
|---|---|
| [`hardware_config.h`](hardware_config.h) | shared pins, constants and helpers — **edit pins here only** |
| [`01_encoder_ticks_per_rev.cpp`](01_encoder_ticks_per_rev.cpp) | encoder ticks per wheel revolution |
| [`02_ticks_per_cm.cpp`](02_ticks_per_cm.cpp) | ticks per centimetre, by regression |
| [`03_steering_jerk.cpp`](03_steering_jerk.cpp) | steering jerk vs servo slew limit |
| [`04_true_straight_servo.cpp`](04_true_straight_servo.cpp) | true straight servo angle |
| [`05_tof_signal_threshold.cpp`](05_tof_signal_threshold.cpp) | ToF floor signal threshold |
| [`06_turn_90.cpp`](06_turn_90.cpp) | ninety degree turn law |
| [`07_heading_gain.cpp`](07_heading_gain.cpp) | heading hold gain |
| [`08_floor_colour_thresholds.cpp`](08_floor_colour_thresholds.cpp) | floor colour cut points |
| [`CalibrationSuite.cpp`](CalibrationSuite.cpp) | all eight behind a serial menu |

Run them in order the first time: 2 needs 1, 4 needs 2, 6 needs 4, 7 needs 6.
Steps 3, 5 and 8 are independent.

Each file is a self-contained Arduino sketch with its own `setup()` and
`loop()`. Flash one at a time — do not try to compile the folder as a unit.

Steps 2, 3, 4, 6 and 7 drive the motor. Put the car on the floor with a clear
run ahead of it, never on a bench edge. Every one of them waits for the start
button before it moves.
