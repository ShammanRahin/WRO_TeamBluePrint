# BOM — Bill of Materials

Source of truth for parts. ⚠️ **Owned/buying, source, lead time, and price are Open Question #3** — fill from a real inventory check + Daraz/RoboticsBD lookup before ordering. TBD = not yet confirmed.

| Part | Spec | Owned / Buying | Source | Lead time | Price |
|---|---|---|---|---|---|
| Drive motor | 25GA gearmotor, 1331 RPM, 180 counts/rev (encoder) | TBD | TBD (Daraz / RoboticsBD) | TBD | TBD |
| Steering servo | Digital metal-gear, commanded in µs | TBD | TBD | TBD | TBD |
| MCU (control) | STM32 (12-bit ADC, HW quadrature timers) | TBD | TBD | TBD | TBD |
| SBC (obstacle round only) | Raspberry Pi (Pi 4/5) — pillar colour vision only | TBD | TBD | TBD | TBD |
| Camera | Fisheye module for Pi | TBD | TBD | TBD | TBD |
| Steering-angle encoder | AS5600 magnetic encoder (on steering shaft) | TBD | TBD | TBD | TBD |
| IMU | Gyro/IMU for heading-hold (part # TBD) | TBD | TBD | TBD | TBD |
| Distance sensor — IR | Analog Sharp IR (model TBD; calibrated V→mm) | TBD | TBD | TBD | TBD |
| Distance sensor — sonar | Front ultrasonic (flat-wall reflector) | TBD | TBD | TBD | TBD |
| Wheels | 40–45 mm dia (exact TBD by measure) ×4 | TBD | TBD | TBD | TBD |
| Ball bearings | For every axle + steering pivot (bore/OD TBD) | TBD | TBD | TBD | TBD |
| Motor driver | H-bridge / driver sized to 25GA stall (part TBD) | TBD | TBD | TBD | TBD |
| Battery | Separate motor + logic domains (chem/voltage TBD) | TBD | TBD | TBD | TBD |
| Power caps | 470–1000 µF bulk (motor rail), 0.1 µF ceramic ×N | TBD | TBD | TBD | TBD |
| Connectors | JST / screw terminals + strain relief (NO Dupont) | TBD | TBD | TBD | TBD |
| Fasteners | Nyloc nuts + threadlocker / nail polish | TBD | TBD | TBD | TBD |
| Chassis / mounts | 3D-printed (models in /3D-models) | Make | Self-print | — | filament |
| LIDAR (deferred) | Reserve upper-deck space, do NOT buy for v1 | Deferred | — | — | — |

## Notes
- Confirm the 25GA encoder count (180 per wheel rev vs per motor rev) by hand-rotate test before finalising odometry math.
- Confirm actual wheel diameter by measurement — it drives odometry and R.
- Bearing bore/OD depends on chosen shaft diameter — lock shaft first.
