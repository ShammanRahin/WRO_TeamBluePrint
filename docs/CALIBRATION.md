# Calibration

Every number this car drives on was measured, not guessed. This page is the
procedure, start to finish, written so that somebody who has never touched the
robot can pick it up and get the same numbers we did.

Work through the steps in order the first time. Later you can re-run any single
step on its own, but the dependencies below are real — step 2 is meaningless if
step 1 was wrong.

```
1  encoder ticks per revolution
2  ticks per centimetre          needs 1
3  steering jerk                 independent
4  true straight servo angle     needs 2
5  ToF floor signal threshold    independent
6  ninety degree turns           needs 4
7  heading correction gain       needs 6
8  floor colour thresholds       independent
```

## Before you start

| | |
|---|---|
| Board | WeAct BlackPill, STM32F411CEU6 |
| Framework | Arduino core for STM32 (STM32duino) |
| Upload | DFU over USB, or ST-Link |
| Serial | 115200 baud |
| Libraries | `SparkFun_BNO08x_Arduino_Library`, Pololu `VL53L1X`, `Adafruit_TCS34725`, built-in `Servo`, `SPI`, `Wire` |

Two ways to run these:

- **The eight standalone sketches** in [`src/tools/calibration/`](../src/tools/calibration/) —
  `01_encoder_ticks_per_rev.cpp` through `08_floor_colour_thresholds.cpp`.
  One job each, heavily commented, more diagnostic output. Read these to
  understand what a step is doing.
- **[`CalibrationSuite.cpp`](../src/tools/calibration/CalibrationSuite.cpp)** — all
  eight behind a serial menu. Flash once, type a number. This is the one to
  have on the board in a pit with no table.

Both share [`hardware_config.h`](../src/tools/calibration/hardware_config.h).
Pins live there and nowhere else. Anything marked `VERIFY` in that file has not
been confirmed against the current PCB yet.

**Everything must be at competition weight.** Battery in, lid on, mast fitted.
The rolling radius of a loaded tyre is smaller than an unloaded one, and every
distance number in this document depends on it.

---

## 1. Encoder ticks per wheel revolution

**Gives you:** counts produced by one full turn of the drive wheel.

**Why not read the datasheet:** because the datasheet describes the encoder, not
your car. Gearbox backlash, a coupler that slips a few degrees under load, and
quadrature counting both edges of both channels all move the real figure.

**Rig:** flat hard floor. A pen mark on the tyre sidewall and a matching mark on
the chassis right beside it.

**Procedure:** line the marks up, press the button to zero, turn the wheel
forward by hand exactly one revolution until the marks meet again, press the
button. Repeat until you have 100 samples. Turn slowly and evenly and never
back up to correct an overshoot — a backed-up sample is a bad sample, discard
it.

**Reading it:** the sketch prints a running mean and standard deviation after
every sample. The standard deviation should settle inside about one percent of
the mean. If it keeps growing you are overshooting the mark, or the encoder is
dropping counts — check the coupler grub screw and the encoder wiring before
you collect another eighty samples of noise.

---

## 2. Ticks per centimetre

**Gives you:** `TICKS_PER_CM`, currently **31.933**.

**Why it takes two measurements.** A caliper across the wheel gives a first
guess:

```
ticks_per_cm  =  ticks_per_rev / (pi * wheel_diameter_cm)
```

That guess is always slightly high, because the loaded rolling radius is smaller
than the free radius — the tyre squashes under the car's weight and the contact
patch flattens out. How much smaller depends on the tyre compound, the mass and
the floor. There is no way to calculate it. You have to drive it.

**Why a regression and not a division.** Every run has a fixed error at each
end: the car accelerating off the line, and the car coasting the last few
millimetres after the motor cuts. Divide one distance by one tick count and that
fixed error is baked into your answer. Fit a straight line through several
different run lengths and the fixed error falls into the intercept, leaving the
slope clean. The slope is the number you want.

**Rig:** 2 m or more of the surface you will actually compete on. Carpet and
competition mat give different answers. Masking tape at 25, 50, 75, 100, 150
and 200 cm.

**Procedure:** line the rear axle up with the start line, press start, let it
roll, press start to stop it near a target line. Measure where the rear axle
*actually* finished, not where you aimed, and type that into the serial monitor.
Three passes over all six distances is eighteen points.

**Reading it:** R² should be above 0.999. Lower than that and either one
distance reading is wrong or the wheels are slipping. A large intercept means a
consistent sighting bias — you are reading the tape at an angle.

---

## 3. Steering jerk

**Gives you:** `SERVO_SLEW`, currently **2.5** degrees per control cycle.

**What jerk is.** Position, velocity, acceleration, jerk — the third derivative.
Taken on the heading axis here:

```
yaw rate   (deg/s)    straight off the gyro
yaw accel  (deg/s2)   rate of change of yaw rate
yaw jerk   (deg/s3)   rate of change of yaw accel
```

**Why it matters.** A jerk spike is the car snapping into a turn. That snap
breaks traction at the rear, scrubs the tyres, and makes the encoder over-read
because the wheels are turning further than the car is travelling. It also puts
a torque spike through the servo horn, which is how plastic gears strip. Limit
how fast the servo is allowed to move and you trade a little turn-in speed for a
large drop in jerk, and the car starts behaving the same way twice.

**Rig:** 2 m of clear floor, the car drives a continuous circle. Battery at
competition charge — jerk scales with speed and speed scales with pack voltage.

**Procedure:** set `SLEW_LIMIT` at the top of the sketch, flash, press start.
The car rolls straight for 800 ms so it is actually moving, then commands 25
degrees of steer and holds it while logging. Run the same steer angle at
`SLEW_LIMIT` of 0 (no limit, worst case), 1.0, 2.5 and 5.0.

**Reading it:** plot peak jerk against slew limit. You want the knee — the point
past which slowing the servo further stops buying you anything. Watch the
settling time printed at the end too. Past about 300 ms your corner entry goes
sloppy and you are giving back more than the jerk reduction is worth.

---

## 4. True straight servo angle

**Gives you:** `SERVO_TRUE_STRAIGHT`.

**Why it is not 90.** The servo's mechanical centre, which spline tooth the horn
landed on, the tie-bar length and both knuckle stops all stack up. Zero steer
lands wherever that stack puts it. On the nationals car it was **69 degrees**.

You cannot eyeball this. Half a degree of steer is invisible across a workbench
and puts the car 9 cm off line over a metre.

> The current build runs a **JX PS-1171MG** where the nationals car had an
> **MG996R**. Different servo, different spline, different centre. The 69.0 in
> `hardware_config.h` is from the old car and **must be re-measured**.

**How the sketch finds it.** Sweep candidate angles either side of your best
guess. Drive a fixed distance at each one and record how much heading the IMU
accumulated. A car going perfectly straight accumulates zero. Least absolute
drift wins. Several passes per angle, because floor texture and push-off add
noise to any single run.

**Rig:** 2 m straight, clear, on competition surface. Room to catch it.

**Reading it:** the summary should be a V — a clear minimum with bigger errors
either side. Flat or noisy means `RUN_CM` is too short, raise it. A minimum
sitting at the edge of the sweep means `CENTRE_GUESS` was off; move it and go
again.

---

## 5. ToF floor signal threshold

**Gives you:** `SIGNAL_MIN_MCPS` (**4.0**) and `TOF_MAX_VALID_MM` (**1300**).

**The problem.** The WRO mat is white vinyl and highly reflective. The walls are
matte black and barely reflective at all. The VL53L1X sees a cone, not a dot, so
when the car noses down even slightly the bottom of that cone clips the floor.
The floor throws back a bright return from close range; the wall throws back a
weak one from further away. The sensor reports the bright one, and you get a
wall that is not there.

Distance alone cannot separate the two. **Signal strength can** — and it works
the opposite way round to instinct. A real wall return is *weak*. A floor return
is *strong*. So we set a ceiling: anything stronger than the threshold is floor,
and gets thrown away.

**Rig:** the car at its finished ride height and rake. Not a bench — the whole
point of this measurement is the angle the sensor sits at once it is bolted to
the car. Plus a piece of the real matte black wall material and a tape measure.

**Procedure:** log 30 s of clear floor with no wall in range. Then log 30 s each
with the wall at 200, 400, 600, 800, 1000 and 1200 mm. Paste the CSV into a
spreadsheet.

**Reading it:** plot signal rate per condition. Two clouds that barely touch —
floor high, wall low. Put the threshold in the gap, nearer the wall cloud, so
you never discard a real wall. The furthest distance at which the wall still
returns usably is your `TOF_MAX_VALID_MM`.

If the clouds **overlap**, the fix is mechanical, not numerical. Add a
collimator snout or tilt the sensor up. See
[`electrical/collimator.py`](../electrical/collimator.py) — that solver is what
took our first ground reflection from 166 mm out to 870 mm.

---

## 6. Ninety degree turns

**Gives you:** `TURN_KP`, `TURN_MAX_STEER`, `TURN_MIN_STEER`, `TURN_KV`,
`TURN_STOP_DEG`, `TURN_MIN_PWM`, `TURN_MAX_PWM`.

**How the turn works.** Not for a fixed time, not for a fixed distance. Both of
those lie the moment a wheel slips. The car turns until the IMU says the heading
has moved 90 degrees, and not one degree before.

```
error  =  target_heading - current_heading
steer  =  clamp(TURN_KP * |error|,  TURN_MIN_STEER, TURN_MAX_STEER)
pwm    =  clamp(TURN_KV * |error|,  TURN_MIN_PWM,   TURN_MAX_PWM)
done   when |error| < TURN_STOP_DEG
```

Big error, big steering and more speed. Small error, less of both — so the car
eases into the exit instead of snapping onto it and overshooting.

**There is no settle step, deliberately.** The turn hands straight over to
heading hold, which absorbs the last fraction of a degree while the car is
already driving away down the next straight. A settle delay just burns track
time to arrive at the same heading.

**Rig:** 1.5 m square of open floor.

**Procedure:** press start per sample, thirty samples, alternating left and
right (`ALTERNATE` does this for you).

**Reading it:**

| What you see | What it means |
|---|---|
| mean near zero, sd under ~1.5 deg | done |
| mean consistently one sign | systematic over- or undershoot; lower `TURN_KP` if overshooting |
| large sd, scattered both ways | `TURN_MIN_STEER` or `TURN_MIN_PWM` too low and the car stalls out at the end of the turn |
| left mean and right mean differ | mechanical, not software. Steering is not symmetric about `SERVO_TRUE_STRAIGHT`. Back to step 4 |

This one compounds. A 1 degree bias per corner is 12 degrees by the end of three
laps, and 12 degrees is a wall.

---

## 7. Heading correction gain

**Gives you:** `HEAD_KP` (**2.0**), and confirmation that `HEAD_KD` and
`HEAD_KI` stay at zero.

**What heading hold does.** Between corners the car has exactly one job: hold
the heading it left the last corner on. It does not follow a wall. It does not
look at the camera.

```
error  =  target_heading - current_heading
servo  =  SERVO_TRUE_STRAIGHT + HEAD_KP * error,  slew limited
```

A P controller and nothing more.

- **KI is zero** because there is no steady disturbance worth integrating away.
  If the car drifts constantly then `SERVO_TRUE_STRAIGHT` is wrong, and an
  integrator would just hide that from you.
- **KD is zero** because the yaw signal is already filtered
  (`YAW_FILT_ALPHA = 0.35`) and differentiating filtered noise buys nothing at
  0.7 m/s.

**How to tune KP.** Raise it until the car visibly snakes — a weave down the
straight with a period of roughly half a second — then back off to about 60
percent of the gain that first produced it.

Too low and the car takes the entire straight to come back on line after a
corner. Too high and it snakes, which costs distance accuracy, because the
encoder counts the zigzag and not the straight line.

**Rig:** the longest straight you can find, 3 m plus.

**Reading it:** table KP against RMS error and zero-crossing count. RMS error
falls as KP rises, then crossings start climbing. Take the KP just below that.
More than about six crossings over 3 m is snaking. Worth doing once more while
nudging the car sideways by hand mid-run — the disturbance recovery tells you
more than a clean run does.

---

## 8. Floor colour thresholds

**Gives you:** the cut points that turn a TCS34725 reading into ORANGE, BLUE or
NOTHING. Currently:

```
BLUE    if  %B > 36  and  %R < 24
ORANGE  if  %R > 35  and  %B < 27
```

**Why percentages, not raw counts.** Raw red, green and blue counts move with
room brightness, sensor ride height, and pack voltage — the sensor's own
illumination LED dims as the battery sags. All three of those change on
competition day.

What does not change is the **ratio** between channels. Orange mat is orange
whether it is brightly or dimly lit. So divide each channel by the total and
threshold on the percentage, and the same code works in the pit, under arena
lights, and at the end of a run on a tired pack.

Note the deliberate gap between the two rules — blue demands `%R < 24` while
orange only demands `%B < 27`. White mat sits in the middle, matches neither,
and correctly returns nothing.

**Rig:** the real mat. The sensor at final ride height with its hood fitted.
Ride height changes the answer, so a bench reading is worthless.

**Procedure:** 200 samples over white, then orange, then blue. Then do all three
again with the lighting changed — brighter, dimmer, and with somebody standing
over the car casting a shadow. Competition lighting is not your workshop
lighting.

**Reading it:** set each cut point about three standard deviations clear of the
population you are excluding. If orange and white `%R` overlap at three sigma,
your hood is leaking ambient light — fix the hood, do not fudge the number.

`COLOR_CONFIRM_MS = 6` is the other half of this: a colour must hold for 6 ms
before it counts. That kills single-sample noise and adds no meaningful lag at
0.7 m/s.

---

## Pillar colour, on the Pi

Separate from the floor sensor. Pillar colour is done in the camera pipeline on
the Raspberry Pi, tuned through the web dashboard rather than by reflashing:
you click the pillar in the live view, it takes a set of samples, and it
computes the threshold for you.

Run it with:

```bash
cd src/pi
python3 dashboard.py          # then open http://<pi-address>:5000
```

Save writes back to [`src/pi/config.json`](../src/pi/config.json) atomically, and
the robot reads that same file at startup. The detection functions in
[`src/pi/sensors/camera.py`](../src/pi/sensors/camera.py) are module-level and
shared by both the dashboard and the robot, so what you tune is exactly what
runs. Nothing is duplicated between the two paths.

> **Open item.** `config.json` and `dashboard.py` as committed both work in
> **HSV**. The click-to-sample tuning we actually use works in **Lab**. The Lab
> version is not in the repository yet. Until it is committed, this section
> describes the HSV path and the two will disagree.

---

## Current values, all in one place

| Constant | Value | From |
|---|---|---|
| `TICKS_PER_CM` | 31.933 | step 2 |
| `SERVO_TRUE_STRAIGHT` | 69.0 | step 4 — **re-measure for the JX servo** |
| `SERVO_MAX_LEFT` | 5.0 | step 4 |
| `SERVO_MAX_RIGHT` | 115.0 | step 4 |
| `SIGNAL_MIN_MCPS` | 4.0 | step 5 |
| `TOF_MAX_VALID_MM` | 1300 | step 5 |
| `TURN_KP` | 2.5 | step 6 |
| `TURN_MAX_STEER` | 55.0 | step 6 |
| `TURN_MIN_STEER` | 8.0 | step 6 |
| `TURN_KV` | 3.5 | step 6 |
| `TURN_MIN_PWM` / `TURN_MAX_PWM` | 100 / 130 | step 6 |
| `TURN_STOP_DEG` | 0.3 | step 6 |
| `HEAD_KP` / `KD` / `KI` | 2.0 / 0 / 0 | step 7 |
| `YAW_FILT_ALPHA` | 0.35 | step 7 |
| `SERVO_SLEW` | 2.5 | step 3 |
| floor BLUE | `%B > 36 && %R < 24` | step 8 |
| floor ORANGE | `%R > 35 && %B < 27` | step 8 |
| `COLOR_CONFIRM_MS` | 6 | step 8 |
