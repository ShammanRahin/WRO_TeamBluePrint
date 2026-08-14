# tools - bench sketches used to tune the car

Standalone STM32/Arduino sketches and Pi scripts used to bring up and tune each subsystem.
None run during a scored round; they exist so the tuning is reproducible and the numbers in the
firmware are traceable to a measurement.

| File | Purpose |
|---|---|
| tof_channel_scanner.ino | Find which mux channel each ToF is on. Result: ch1=left, ch3=right, ch4=front. |
| tof_signal_tester.ino | Read raw VL53L1X range + signal rate to set the floor-rejection threshold. |
| FindStraight / SweepStraight / StraightTuner / StraightDiag .ino | Find the true servo centre by motion and tune the straight-line heading hold. |
| PidTest.ino | Exercise the heading-hold PID gains in isolation. |
| TurnTuner.ino | Tune the eased 90 degree turn law. |
| LoopbackTest.ino + loopback_test.py | Verify the STM32 <-> Pi UART link end to end. |

## How the straight-line trim was found

`SERVO_TRUE_STRAIGHT` is not eyeballed. The sweep sketches drive the car over a fixed distance
at a range of trim values, log the average gyro yaw-rate, and bisect to the zero-crossing - the
trim where the car actually drives straight.
