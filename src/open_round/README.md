# open_round - Open Challenge firmware

`OpenRound.cpp` - the STM32-only navigation program. No Raspberry Pi, no camera; the Pi
harness is unplugged for this round so its absence is inspectable.

## What it does

Drives 3 laps (12 corners) of the walled track and stops in the start section, using only the
BNO085 IMU, one front VL53L0X ToF (mux ch 3), the TCS34725 floor colour sensor (mux ch 4), and
the motor encoder (TIM5 on PA0/PA1).

Status LEDs: PB12 blinks during the start countdown and is solid while running, PB13 lights
over orange, PB14 over blue; all three solid = finished.

## Control flow

A non-blocking cooperative state machine - `loop()` services every sensor once per pass and
advances one step. No blocking waits or `delay()` in the driving path, so the car moves
continuously (turn -> shuffle -> realign -> straight).

| State | What happens |
|---|---|
| WAIT_START | 5 s countdown, then latch lane heading and zero the encoder. ⚠️ The start button (PB15) is **not read yet** — rule 9.11 requires it before competition |
| DRIVE_TO_CORNER | Heading-hold PID; watch floor colour + front wall for a corner |
| TURNING | Eased 90 degree arc, terminated on measured IMU heading; re-reference lane heading |
| LANE_CORRECT | Short measured lateral shuffle to re-centre, sized from the learned gap |
| FINAL_STRAIGHT | Drive the measured remaining distance and stop in the start section |
| RECOVER | Front wall ≤ 200 mm: reverse with mirrored steering until ≥ 350 mm (max 30 cm, 3 tries), then resume |
| FINISHED | Motor off, wheels centred |

IMU zeroing and sensor bring-up happen in `setup()`; there is no separate INIT state.

## Key ideas

- Heading-hold PID: structure has P, I (clamped) and D on filtered yaw-rate, servo
  slew-limited; as tuned only P is non-zero (`HEAD_KP = 2.0`). Acts only on a fresh IMU sample.
- Turn trigger = colour gate + wall event: a corner line arms the turn; it fires when the
  front wall is within 700 mm, or on colour alone if the ToF failed, or after 150 cm past the
  line as a backstop. There is no side ToF on this build. Colour is locked out for 50 cm after
  each turn, so the two lines per corner cannot double-fire.
- Direction decode: the first corner line colour (orange = clockwise, blue = counter-clockwise)
  locks the turn direction and selects which side ToF is the inner wall.
- Distance bookkeeping: the encoder is zeroed only at start and at each turn completion.
- Auto-learned gap: the lateral correction reference is learned from the first corner.

## Tuning

Constants at the top of the file (`TICKS_PER_CM = 14.853`, `SERVO_TRUE_STRAIGHT = 71.0`,
colour thresholds) were measured on the re-pinned car. The colour thresholds come from
the colour-capture version of `../tools/bench/tof_test.ino` (commit `a6f685e`; the file is now a three-ToF test). The calibration sketches in
`../tools/calibration/` still use the earlier pin map, so update `hardware_config.h` before
using them on this board. `SERVO_TRUE_STRAIGHT` is found by motion, not by eye.
