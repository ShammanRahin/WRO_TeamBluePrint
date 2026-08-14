# obstacle_round - Obstacle Challenge firmware

`ObstacleRound.cpp` - the same navigation core as the open round, plus the Raspberry Pi vision
link, the offset-based pillar avoidance, and the front-proximity failsafe.

## What it does

Drives 3 laps while passing red pillars on the right and green pillars on the left, then stops.
Corner navigation is identical to the open round.

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
breaks at wall gaps and corners. See Decision #13.

## Front-proximity failsafe

Across every forward phase, if the front ToF reads below 200 mm the car reverses a short
distance holding its last steering angle, increases the swerve angle, and retries - escalating
until it clears, then restoring the base swerve. See Decision #14.

## Start + safety

- Start is the momentary button on PA5 (active-low).
- The STM32 is the safety master: a missing vision frame never stalls the car.
- Telemetry (`#` frames) is streamed back to the Pi every 100 ms.

## Tunables worth knowing

- DX_SPAN - camera half-width in px; sets how dx maps to move distance (set to 1 if the Pi
  sends a normalised offset).
- AVOID_MIN_AREA_R / AVOID_MIN_AREA_G - how big a pillar must look before the car acts.
- AVOID_SIDE_NEAR_MM / AVOID_SIDE_FAR_MM / SIDE_SAFE_MM - offset range and hard wall floor.
- FRONT_STOP_MM, AVOID_STEER_BOOST, AVOID_BACKUP_CM - the failsafe behaviour.
