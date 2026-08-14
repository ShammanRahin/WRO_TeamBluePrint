# open_round - Open Challenge firmware

`OpenRound.cpp` - the STM32-only navigation program. No Raspberry Pi, no camera; the Pi
harness is unplugged for this round so its absence is inspectable.

## What it does

Drives 3 laps (12 corners) of the walled track and stops in the start section, using only the
IMU, the three ToF sensors, the floor colour sensor, and the wheel encoder.

## Control flow

A non-blocking cooperative state machine - `loop()` services every sensor once per pass and
advances one step. No blocking waits or `delay()` in the driving path, so the car moves
continuously (turn -> shuffle -> realign -> straight).

| State | What happens |
|---|---|
| INIT | Latch the IMU heading zero, zero the encoder |
| DRIVE_TO_CORNER | Heading-hold PID; watch floor colour + walls for a corner |
| TURNING | Eased 90 degree arc, terminated on measured IMU heading; re-reference lane heading |
| LANE_CORRECT | Short measured lateral shuffle to re-centre, sized from the learned gap |
| FINAL_STRAIGHT | Drive the measured remaining distance and stop in the start section |
| FINISHED | Motor off, wheels centred |

## Key ideas

- Heading-hold PID: proportional on error, derivative on filtered yaw-rate, small integral,
  servo slew-limited with anti-windup. Acts only on a fresh IMU sample.
- Turn trigger = colour gate + wall event: a corner line arms the turn; it fires only when the
  front wall is close OR the inner-side ToF goes invalid. Colour is locked out from the start
  of a turn until the car is clear, so the two lines per corner cannot double-fire.
- Direction decode: the first corner line colour (orange = clockwise, blue = counter-clockwise)
  locks the turn direction and selects which side ToF is the inner wall.
- Distance bookkeeping: the encoder is zeroed only at start and at each turn completion.
- Auto-learned gap: the lateral correction reference is learned from the first corner.

## Tuning

Constants at the top of the file were set with the bench sketches in `../tools/`.
`SERVO_TRUE_STRAIGHT` is found by motion, not by eye.
