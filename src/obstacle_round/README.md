# obstacle_round - Obstacle Challenge firmware

`ObstacleRound.cpp` - the same navigation core as the open round, plus the Raspberry Pi vision
link, the offset-based pillar avoidance, and the front-proximity failsafe.

## What it does

Drives 3 laps while passing red pillars on the right and green pillars on the left, then stops.
Corner navigation follows the same core as the open round, with three VL53L1X (left mux ch 1,
right ch 3, front ch 4) so a turn can also fire when the inner side wall ends. There is no
parking phase yet.

> ⚠️ This program still uses the **earlier pin map** (motor PB9/PB8/PB1, encoder TIM3 on
> PA6/PA7, IMU SPI on PB3–PB5, start button PA5) and the earlier calibration
> (`TICKS_PER_CM = 31.933`, `SERVO_TRUE_STRAIGHT = 69.0`). It must be ported to the re-pinned
> carrier used by `OpenRound.cpp` before it can run on the current car.

## The two colour channels do not collide

- The floor colour sensor (TCS34725) reads the orange/blue corner lines - this drives the turns.
- The camera (on the Pi) reads the red/green pillars - this drives the avoidance.

## Offset-based avoidance

| Phase | What happens |
|---|---|
| SWERVE | Steer to the pass side (green->left, red->right) to a computed offset - the pillar dx sets how far. The side ToF only clamps a minimum wall clearance. Remember the displacement. |
| STRAIGHTEN | Ease back onto the lane heading, holding the offset |
| HOLD | Drive straight until the pillar colour has left the frame (debounced) |
| RETURN | Reverse the same remembered displacement back toward centre |
| REALIGN | Ease onto the lane heading and resume normal driving |

Why offset-based and not "hug the wall": the offset is stable across the track, whereas chasing
the side wall to a fixed distance hugs whatever wall is there regardless of the pillar and
breaks at wall gaps and corners.

## Front-proximity failsafe

During phases SWERVE to RETURN, if the front ToF reads below 200 mm the car reverses 15 cm
holding its last steering angle, adds 8 degrees to the swerve angle (max 44), and retries -
escalating until it clears, then restoring the base swerve after the pillar is passed.

## Start + safety

- Start is the momentary button on PA5 (active-low).
- The STM32 is the safety master: vision older than 250 ms is ignored, so a missing frame
  never stalls the car. The Pi-side frame sender is not written yet (see `../pi/README.md`).
- Telemetry (`#` frames) is streamed back to the Pi every 100 ms.

## Tunables worth knowing

- DX_SPAN - camera half-width in px (160, i.e. a 320 px frame); sets how dx maps to move
  distance. `pi/sensors/camera.py` captures at 640 px wide, so either scale dx on the Pi or set
  this to 320.
- AVOID_MIN_AREA_R / AVOID_MIN_AREA_G - how big a pillar must look before the car acts.
- AVOID_SIDE_NEAR_MM / AVOID_SIDE_FAR_MM / SIDE_SAFE_MM - offset range and hard wall floor.
- FRONT_STOP_MM, AVOID_STEER_BOOST, AVOID_BACKUP_CM - the failsafe behaviour.
