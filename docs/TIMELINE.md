# Engineering log

Dated record of what we built, what broke, and what we changed because of it.
Entries are written when the thing happens, not reconstructed afterwards.

The detailed rationale behind each architectural change lives in
[`DECISIONS.md`](../DECISIONS.md) as numbered ADRs; this page is the narrative
that connects them.

---

## 2026-07-12 — repository opened

First commit. Steering study begun the same day
([`journal/steering-study-2026-07-12.md`](../journal/steering-study-2026-07-12.md)).

## 2026-07 — steering geometry

Kinematic simulations for turning radius, swept envelope and parking
feasibility. See [`src/sim/`](../src/sim/) and
[`journal/day-01-geometry-and-steering.md`](../journal/day-01-geometry-and-steering.md).

**Finding: the textbook two-arc reverse park is not feasible for us.** At 35
degrees of lock inside a bay 1.5x the vehicle length, the symmetric trajectory
misses by 25.6 mm. The problem is scale-invariant — shortening the car shrinks
the bay proportionally, so a smaller car does not fix it. Replaced with an
iterative multi-point shuffle closed on IMU yaw.

## 2026-07-26 — plan revision

[`journal/day-02-plan-revision-2026-07-26.md`](../journal/day-02-plan-revision-2026-07-26.md).

**Centre-pivot steering scrapped.** Simulation had said a single central pivot
tolerated joint slop three times better, so that is what was built. On the car
it was wrong: rotating the whole front axle swept the outer tyre +/-30.1 mm fore
and aft, which is 36% of the 82.5 mm parking clearance, and raised the risk of a
body strike. Replaced with a dual-knuckle parallelogram tie-bar that locks the
knuckle centres.

**Steering encoder deleted.** The original design had an AS5600 on the steering
shaft to cancel servo backlash. Moving to the parallelogram removed the central
shaft it would have mounted to, and the analysis that followed showed closed-loop
steering angle is redundant anyway: every manoeuvre terminates on measured body
yaw, not on wheel angle. One sensor, one I2C address and some mass saved.

## 2026-07-28 — ToF floor crosstalk solved

The VL53L1X has a wide cone and no software region of interest. With 1.1 degrees
of nose-down rake, the bottom of the cone hit the white mat and the sensors
reported a wall at 166 mm that was not there.

Built [`electrical/collimator.py`](../electrical/collimator.py) to size an
optical baffle. 2.5 x 10 x 20 mm printed slot collimators plus a +2.0 degree
mechanical wedge pushed the first ground reflection from **166 mm to 870 mm**,
clearing the side walls at 442.5 mm with margin.

Also on this date: electrical revision to a two-board single-sided build,
decisions 23 through 28.

## 2026-08 — firmware

Open-round FSM working. Obstacle logic bolted on top of it unchanged, rather
than written fresh, so that a pillar bug can never break the round that already
works.

**Bug, mid-August:** clean runs with no pillars, but after an avoidance
manoeuvre the next corner was taken in the wrong direction. Fixed by giving the
corner-turn trigger absolute priority over everything including an in-progress
avoidance.

## 2026-08-29 to 09-05 — carrier PCB

Revisions A through C in EasyEDA. Final: 90 x 70 mm single-sided, Pi 5 stack,
mux moved off onto its own board. Full decision trail in `DECISIONS.md`.

## 2026-09-01 to 09-04 — perception stack

[`src/pi/`](../src/pi/). Python environment for the RPLIDAR C1 and the Pi
camera, threading skeleton, camera and lidar adapters, fusion loop, standalone
calibration dashboard, systemd unit.

Status: fusion runs and prints. It does not yet emit the `V,...` frame to the
MCU. **Open.**

## Rebuild for the Open Championship

The nationals car and the Hyderabad car are not the same vehicle.

- **Servo changed** from MG996R to **JX PS-1171MG**. Every steering constant
  measured on the old servo — `SERVO_TRUE_STRAIGHT`, both limits — is invalid
  until [step 4](CALIBRATION.md#4-true-straight-servo-angle) is re-run.
- **IMU: MPU6050 abandoned, BNO085 fitted.** The MPU6050's yaw drift was large
  enough to lose a 90 degree turn within three laps. Since the entire control
  philosophy is "heading is the truth", an IMU that drifts is not a component
  you can compensate for in software — it invalidates the premise. Replaced with
  a BNO085 running its own sensor fusion.
- **ToF count reduced** to two, front and rear protection, now that lateral
  position comes from the lane-gap measurement rather than from wall following.

---

## To be filled in

Samman to complete. These are known gaps, listed so they are not forgotten:

- [ ] Nationals result and anything that failed on the day
- [ ] Parts burned or destroyed, with dates
- [ ] When the RPLIDAR C1 and the Pi 5 actually arrived, and what the car ran
      before them
- [ ] Why the vision pipeline is still at "fusion prints to console"
- [ ] Everything from 2026-09-05 onward, including the current rebuild
- [ ] Test results: runs attempted, runs completed, failure modes seen
