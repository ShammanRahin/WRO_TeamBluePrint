# Troubleshooting

Problems we have actually had on this car, and what fixed them. If you build
from these files you will hit most of these.

## The car reports a wall that is not there

**Symptom.** Front or side ToF shows a few hundred mm on open track. The car
turns early or refuses to drive.

**Cause.** The sensor is reading the white mat. The VL53L1X sees a cone, and any
nose-down rake puts the bottom of that cone on the floor. The floor is bright
and close, so it wins.

**Fix.** Signal-rate filtering — discard returns stronger than
`SIGNAL_MIN_MCPS`. See [CALIBRATION step 5](CALIBRATION.md#5-tof-floor-signal-threshold).
If filtering alone is not enough, the fix is mechanical: fit the printed
collimator snouts and the +2 degree wedge. Ours moved the first floor reflection
from 166 mm to 870 mm.

## Heading drifts over a run

**Symptom.** Laps one and two are clean, lap three clips a wall.

**Causes, in the order worth checking:**

1. **The IMU.** We had this with an MPU6050 and no amount of software fixed it.
   If your yaw reading walks while the car sits still, replace the part. We use
   a BNO085.
2. **Turn bias.** A consistent 1 degree of overshoot per corner is 12 degrees by
   the end. Run [step 6](CALIBRATION.md#6-ninety-degree-turns) and look at
   whether the mean error has a sign.
3. **Asymmetric steering.** Left and right turn errors differing means
   `SERVO_TRUE_STRAIGHT` is wrong. Back to [step 4](CALIBRATION.md#4-true-straight-servo-angle).

## The car turns twice at one corner

**Symptom.** Immediately after a turn it declares another corner and spins.

**Cause.** Coming out of the turn the colour sensor sweeps back across the same
pair of lines.

**Fix.** `POST_CORNER_LOCKOUT_CM`, currently 50 cm, during which colour is
ignored. If it still happens, the lockout is too short for your exit geometry.

## After avoiding a pillar, the next corner goes the wrong way

**Symptom.** Clean with no pillars. With pillars, the car turns the wrong way at
the next corner.

**Cause.** The avoidance sub-machine was still running and held the steering
when the corner condition fired.

**Fix.** Corner turns take absolute priority over everything, avoidance
included. A missed pillar costs points; a missed corner costs the run.

## Distance readings drift over the session

**Symptom.** `TICKS_PER_CM` was right this morning and is wrong now.

**Causes.** A slipping encoder coupler — check the grub screw. Or you calibrated
without the battery in, and the loaded rolling radius is smaller than the
unloaded one. Or you calibrated on a different surface.

## The camera sees red where there is no red

**Symptom.** Phantom red pillars, often near the orange floor line or the
magenta parking walls.

**Cause.** Orange, red and magenta are close together in colour space, and under
the arena lights they get closer.

**Fix.** Re-tune on the real mat under the real lights, not in your workshop.
Raise `min_blob_area`. Check the blob is not sitting inside the floor-line
region before accepting it.

## Camera will not open

**Symptom.** `CameraThread` dies at startup.

**Cause.** Two processes want the camera. `dashboard.py` and `main.py` cannot
both hold it.

**Fix.** `robodash.service` declares `Conflicts=robot.service` for exactly this
reason. Stop one before starting the other.

## Servo browns out the MCU

**Symptom.** The STM32 resets when the steering makes a large fast movement.

**Cause.** The servo pulls an instantaneous stall spike that drags the shared
rail down.

**Fix.** A separate regulator for the servo rail. The power domains in
[`electrical/ELECTRICAL.md`](../electrical/ELECTRICAL.md) section 2 exist for
this. Also limit `SERVO_SLEW` — [step 3](CALIBRATION.md#3-steering-jerk).

## Intermittent faults that come and go with vibration

**Symptom.** Works on the bench, fails on the track. A sensor drops out over
bumps.

**Cause.** DuPont jumpers. They are not a connector, they are a friction fit
that shakes loose.

**Fix.** Crimped and latched JST-XH with strain relief, everywhere. No
breadboards on the vehicle.
