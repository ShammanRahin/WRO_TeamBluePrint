# bench

Minimal sketches used during bring-up to prove one subsystem at a time before
anything was wired together. Keep them: when something stops working on the
car, the fastest route to the cause is proving the part in isolation again.

| Sketch | Proves |
|---|---|
| `HallEncoderTester.cpp` | TIM3 hardware quadrature decoding on PA6/PA7 |
| `RearWheelDriveTester.cpp` | BTS7960 direction and PWM |
| `TCS34725_ColorSensorTester.cpp` | Colour sensor reachable behind the TCA9548A mux |

For measurement rather than bring-up, use
[`../calibration/`](../calibration/) instead.

> `RearWheelDriveTester.cpp` declares RPWM = PB8 / LPWM = PB9. The round
> firmware has them the other way round. One of the two is wrong and it has not
> been confirmed against the board yet.
