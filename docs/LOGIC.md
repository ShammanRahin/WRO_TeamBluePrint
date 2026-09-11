# How the car thinks

This is the whole control logic in plain language. No code required to follow
it. Where a constant is named it comes from
[`docs/CALIBRATION.md`](CALIBRATION.md), and the firmware it describes is
[`src/open_round/OpenRound.cpp`](../src/open_round/OpenRound.cpp) and
[`src/obstacle_round/ObstacleRound.cpp`](../src/obstacle_round/ObstacleRound.cpp).

---

## The one idea the whole car is built on

**The heading is the truth. Everything else is a hint.**

A wheel can slip. A tyre can scrub sideways through a turn. A distance sensor
can read the floor and call it a wall. A camera can lose a pillar to a shadow.
Every one of those has happened to us on a run.

What does not lie is the IMU's idea of which way the car is pointing. So the
car is built so that heading decides when a manoeuvre is finished, and
everything else only decides when one should *start*. A turn ends when the
heading has moved 90 degrees, never after a time or a distance. A straight is
held on heading, not by following a wall.

That single decision is why the car finishes three laps instead of drifting
into a wall on lap two.

---

## Two tiers, and why the split exists

```
            +-------------------------------------------+
            |  Raspberry Pi 5            ADVISORY tier   |
            |                                            |
            |  camera thread  ->  colour + bearing       |
            |  lidar thread   ->  360 ranges             |
            |  fusion loop    ->  obstacles with distance|
            +--------------------+-----------------------+
                                 |  UART 115200
                                 |  "V,<colour>,<dx>,<area>"
                                 v
            +-------------------------------------------+
            |  STM32F411CEU6             REAL-TIME tier  |
            |                                            |
            |  IMU heading, encoder odometry,            |
            |  ToF ranges, floor colour,                 |
            |  the state machine, servo, motor           |
            +-------------------------------------------+
```

The STM32 is the only thing that can move the car. It owns the motor, the servo
and every safety decision, and it runs a fixed cycle with no dynamic memory
allocation and no blocking calls.

The Pi only ever *suggests*. It sends a frame saying "I can see a red pillar,
this far off centre, this big". The STM32 reads that as advice and decides for
itself what to do about it.

**Why it is built this way:** if the Pi crashes, overheats, drops frames, or a
Python exception takes down the vision thread, the STM32 notices the frames have
gone stale — `visionFresh()` is false after 250 ms — and quietly carries on
driving the open-round logic. A dead perception stack costs you pillar points.
It does not cost you the run. The high-level stack physically cannot stall or
crash the vehicle.

---

## Startup and arming

1. Power on. The STM32 brings up the I2C mux, the two VL53L1X, the TCS34725 and
   the BNO085 over SPI. Anything that fails to answer is marked absent and the
   car keeps going without it.
2. **Gyro zeroing.** The car sits still and the current yaw is captured as
   `initialYawOffset`. From here on "heading" means degrees relative to however
   the car was pointing when it was armed, wrapped to -180..+180. This is why
   the car must be placed square in the start section and left alone during
   boot.
3. `STATE_WAIT_START`. Nothing moves until the button is pressed. One button,
   one job, as the rules require.

---

## Working out which way round the track goes

The direction is randomised, so the car has to discover it on the first corner
rather than be told.

The mat has an orange line and a blue line at every corner. Which one the car
crosses **first** tells it the direction of travel:

- orange first -> clockwise -> every corner is a right turn
- blue first -> counter-clockwise -> every corner is a left turn

Once locked, it stays locked for the whole run. Corners two through twelve never
re-ask the question. Every subsequent corner only looks for the colour it
already expects, which makes a stray reflection off the other line harmless.

---

## The main loop: `STATE_DRIVE_TO_CORNER`

The car drives straight, holding the heading it left the last corner on:

```
error  =  target_heading - current_heading
servo  =  SERVO_TRUE_STRAIGHT + HEAD_KP * error,   slew limited to SERVO_SLEW
```

Proportional only. No integral, no derivative — the reasoning is in
[CALIBRATION step 7](CALIBRATION.md#7-heading-correction-gain).

While it drives it is watching for three things.

**The corner.** A corner is declared when the expected floor colour is seen
**and** the front ToF says a wall is inside `FRONT_TURN_MM` (700 mm). Both
conditions, because either alone gives false positives — a colour smear on the
mat, or a ToF return off the floor.

**The lockout.** For `POST_CORNER_LOCKOUT_CM` (50 cm) after a corner, colour
detections are ignored. Coming out of a turn the sensor sweeps back across the
same pair of lines it just crossed, and without the lockout the car would
declare a second corner immediately and spin.

**A pillar**, on the obstacle round only. More on that below.

---

## The lane-gap correction — the part that is ours

This is the piece of this car we have not seen anyone else do, and it is worth
reading slowly.

### The problem

Over three laps the car drifts sideways. Turn slightly wide on corner three and
you spend the rest of the lap running closer to the outer wall, which puts the
corner-four entry in the wrong place, and the error compounds. Twelve corners is
plenty of distance to accumulate a wall strike.

The usual fix is to follow a side wall with a distance sensor. We tried it. It
fails on this mat for the same reason step 5 exists: the ToF picks up the white
floor and reports a wall that is not there, and a wall-follower fed a phantom
wall steers into the real one.

### The insight

The orange and blue lines at each corner are not parallel. **They fan out.** The
further from the inner wall you cross them, the further apart they are.

So the gap between them is a ruler. The car does not need to see a wall at all —
it can measure its own lateral position by how much track it covers between
crossing the first line and crossing the second.

That measurement is pure odometry. It uses the encoder, which is the one sensor
on this car that the white mat cannot fool.

### How it runs

1. **Corner one sets the reference.** The car records the encoder distance
   between the two line crossings and stores it as the gap it intends to hold
   for the rest of the race. Whatever line it happened to take on corner one
   becomes "correct" by definition — the car is not trying to find the racing
   line, it is trying to stop drifting off whatever line it started on.
2. **Every corner after measures the same gap.** Smaller gap than the reference
   means the car has moved closer to the inner wall. Larger means it has drifted
   out.
3. **The correction is proportional to the difference:**

```
gap_error   =  measured_gap - reference_gap
offset_deg  =  clamp(K_LAT_DEG_PER_CM * gap_error,  -MAX_LAT_OFFSET_DEG, +MAX_LAT_OFFSET_DEG)
```

With `K_LAT_DEG_PER_CM = 4` and the cap at 30 degrees. That offset is added to
the heading target and held for `CORRECTION_DISTANCE_CM` (25 cm) at
`CORRECTION_PWM` (70), then released — the car crabs back onto its original
line and resumes normal heading hold.

4. **A deadband stops it hunting.** Errors under `GAP_DEADBAND_CM` (2 cm) are
   ignored. Errors over `GAP_THRESHOLD_CM` (20 cm) are treated as a bad
   measurement rather than a real drift and discarded, because 20 cm of lateral
   movement in one lap means something else went wrong and over-correcting on a
   bad reading is worse than not correcting at all.

### Why it is better than wall-following

- It cannot be fooled by floor reflections, because it never looks at the floor
  for distance.
- It needs no extra hardware — the encoder and the colour sensor are already
  there for other reasons.
- It self-references. There is no absolute "correct distance from the wall" to
  measure or calibrate. Corner one defines the target, so the same code works on
  any mat and any start position.
- It corrects **once per corner**, at the moment the information arrives, rather
  than continuously fighting a noisy signal down the straight.

---

## `STATE_TURNING`

Heading-terminated, proportional, eased, no settle. The law and the reasoning
are in [CALIBRATION step 6](CALIBRATION.md#6-ninety-degree-turns).

Two things worth repeating here:

- The corner turn has **absolute priority**. If a turn condition fires while the
  car is halfway through a pillar avoidance, the turn wins and avoidance is
  abandoned. A missed pillar costs points. A missed corner costs the run.
- `TURN_STOP_DEG` is 0.3 degrees, but the car does not sit there trying to hit
  it. The moment the turn exits, heading hold takes over and removes the
  remainder while the car is already moving away.

On corner twelve the car enters `STATE_FINAL_STRAIGHT`, runs
`FINAL_STRAIGHT_CM` (100 cm) to put itself back in the start section, and brakes
to `STATE_FINISHED`.

---

## Pillar avoidance — obstacle round only

### What the Pi sends

One line per frame, at up to 30 Hz:

```
V,<colour>,<dx>,<area>
```

- `colour` — `R`, `G` or `N` for nothing
- `dx` — how far off centre the pillar is, in pixels, positive to the right,
  spanning `DX_SPAN` = 160 either way
- `area` — blob size in pixels, which stands in for "how close"

The STM32 engages only when the colour is right, `area` exceeds
`AVOID_MIN_AREA_R` / `AVOID_MIN_AREA_G` (1600), and the frame is under 250 ms
old. A stale frame is no frame.

### Offset-based, not fixed-swerve

The naive approach is "see red, steer right by a fixed amount". It fails,
because how far you need to move depends entirely on where the pillar already
is. A pillar dead centre needs a big move. A pillar already near the edge of the
frame needs almost none, and moving the full amount throws you into the wall.

So the target is computed from `dx`:

- pillar **centred** (`|dx|` near 0) -> move a lot -> aim to pass **near** the
  wall, `AVOID_SIDE_NEAR_MM` = 150
- pillar **already off to the side** -> move little -> aim to pass **far** from
  the wall, `AVOID_SIDE_FAR_MM` = 300

with a hard floor of `SIDE_SAFE_MM` = 100 that the car will never go inside
whatever the arithmetic says. Red passes on the right, green on the left, per
the rules.

### The six phases

| Phase | What happens |
|---|---|
| 1 | **Swerve out** to the computed offset. Exits when the offset is reached, the safety floor is hit, the cap is hit, or the pillar simply leaves the frame. The encoder distance travelled sideways is remembered. |
| 2 | **Straighten** onto the lane heading while holding the offset |
| 3 | **Hold** until the colour has been gone for `AVOID_RELEASE_MS` (150 ms) — debounced, so one dropped frame does not end the manoeuvre early |
| 4 | **Return** exactly the remembered displacement, so the move out and the move back are symmetric and the car ends up on the line it started on |
| 5 | **Realign** to lane heading, restore the base swerve angle, hand back to `STATE_DRIVE_TO_CORNER` |
| 6 | **Recover** — see below |

### The recovery phase

If the front ToF drops below `FRONT_STOP_MM` (200 mm) during any of phases 1 to
4, the car is about to clip something. It does not stop and it does not give up:

1. Reverse `AVOID_BACKUP_CM` (15 cm) on the steering angle it was last using, so
   it retraces its own path out rather than reversing into something new
2. Increase the swerve by `AVOID_STEER_BOOST` (8 degrees), capped at
   `AVOID_STEER_MAX` (44)
3. Go back to phase 1 and try again, harder

It must move forward `RECOVER_MIN_FWD_CM` (5 cm) between backups, which stops
the car from oscillating back and forth against an obstacle it cannot clear. The
boosted angle is reset once the pillar is crossed, so one awkward pillar does not
leave the car swerving wildly for the rest of the lap.

---

## The Pi perception stack

Three threads, one shared state object, deliberately simple.
Source: [`src/pi/`](../src/pi/).

### `worldstate.py` — the integration seam

One lock, two slots — the newest camera result and the newest lidar result.
Producers overwrite; nothing is queued. `snapshot()` copies two references out
under the lock and returns.

The critical section is two assignments long. All the heavy work — colour
conversion, contour finding, sector scanning — happens outside it. That is why a
slow camera frame cannot stall the lidar thread or the fusion loop.

Conventions are fixed once, in that file, and enforced everywhere: **distances
in mm, angles in degrees, 0 degrees is robot forward, positive is left, and "no
return" is `float('inf')` and never `None`.** Most integration bugs in a system
like this are unit and sign bugs, and the cure is to decide once and write it
down where nobody can miss it.

### `sensors/camera.py`

Detection lives in **module-level functions**, not inside the thread class. The
robot calls them with values loaded from `config.json`; the dashboard calls the
same functions with live slider values. There is exactly one code path, so what
you tune in the browser is bit-for-bit what runs on the car. Nothing is
duplicated and the two cannot drift apart.

The camera's job is **colour and bearing only**. Distance is left at infinity.

```
bearing = -((cx - width/2) / (width/2)) * (hfov / 2)
```

### `sensors/lidar.py`

The `rplidarc1` library is asyncio and uses a TaskGroup. Rather than force the
whole program to be async, that entire world is sealed inside one thread running
its own event loop. Three tasks: the library's scan producer, a consumer that
folds streaming points into a rolling 360-element array indexed by integer
degree, and a publisher that copies that array into shared state at 50 Hz.

Outside that file nothing in the program knows asyncio exists.

### `main.py` — fusion

At 30 Hz: take a snapshot, and for each camera obstacle look up the lidar range
at the same bearing (`sector_min` over `BEARING_MATCH_DEG` = 8 degrees either
side) to attach a real distance to it.

Camera gives colour and bearing well and distance badly. Lidar gives distance
perfectly and colour not at all. Together they give a coloured obstacle with a
real range, which is what the avoidance logic actually needs.

> **Current state.** The fusion loop prints to the console. It does not yet
> drive the MCU — the `V,...` frame is not being emitted from this stack yet.
> That is the open item on the perception tier.

### `dashboard.py`

Flask, MJPEG streams of raw / mask / overlay, live sliders, a lidar radar view,
and atomic save back to `config.json` (write to a temp file, then `os.replace`,
so a crash mid-save can never leave a half-written config on the car).

Runs as a systemd unit ([`robodash.service`](../src/pi/robodash.service)) with
`Conflicts=robot.service`, because only one process can hold the camera and it
is better to fail loudly at boot than to have two processes fight over
`/dev/video0` in the pit.

---

## What runs where

| | Open round | Obstacle round |
|---|---|---|
| STM32 firmware | `OpenRound.cpp` | `ObstacleRound.cpp` |
| Heading hold | yes | yes |
| Lane-gap correction | yes | yes |
| Pillar avoidance | no | yes |
| Raspberry Pi | not required | camera + lidar + fusion |
| Wireless | disabled | disabled |

Same navigation core in both. The obstacle firmware adds the UART vision link,
the avoidance sub-machine and the front-proximity failsafe, and changes nothing
about how the car drives between pillars.
