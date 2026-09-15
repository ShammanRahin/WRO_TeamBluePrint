# Field Troubleshooting Guide — Failure Modes & Verified Fixes

> A diagnostic manual documenting 10 critical hardware, electrical, optical, and kinematic faults experienced during testing, along with root causes and verified remedies.
> For a high-level system overview, return to the **[Master README](../README.md)**.

---

## Quick Diagnostic Index

| Fault Symptom | Primary Root Cause | Verified Field Remedy |
|:---|:---|:---|
| [1. False Wall Reports on Open Track](#1-the-car-reports-a-wall-that-is-not-there) | White floor reflection clipping ToF beam | Fit collimator snouts + $+2.0^\circ$ wedge; filter `SIGNAL_MIN_MCPS` |
| [2. Heading Drifts Over 3 Laps](#2-heading-drifts-over-a-run) | Gyro integration drift / turn overshoot bias | Swap to BNO085; calibrate `TURN_KP` and `SERVO_TRUE_STRAIGHT` |
| [3. Double Turn / 180° Spin at Corner](#3-the-car-turns-twice-at-one-corner) | Sensor re-crossing corner lines on exit | Increase `POST_CORNER_LOCKOUT_CM` to $50\text{ cm}$ |
| [4. Wrong-Way Turn After Avoidance](#4-after-avoiding-a-pillar-the-next-corner-goes-the-wrong-way) | Avoidance routine overriding corner trigger | Enforce absolute corner turn priority in FSM |
| [5. Wheel Squeal & Wide Corner Exits](#5-the-car-squeals-in-corners-and-runs-wide) | Equal-angle steering scrub | Upgrade to true 100% Ackermann linkage |
| [6. Odometry Drifts Across Sessions](#6-distance-readings-drift-over-the-session) | Loose coupler or unloaded calibration | Tighten motor grub screws; calibrate with battery and lid fitted |
| [7. Phantom Red Obstacles in Vision](#7-the-camera-sees-red-where-there-is-no-red) | Ambient light shifting orange lines into red | Re-tune color bounds in arena; raise `min_blob_area` |
| [8. Camera Device Busy / Fails to Open](#8-camera-will-not-open) | Two processes competing for `/dev/video0` | Enable systemd service conflict masking |
| [9. MCU Browns Out on Sharp Turns](#9-servo-browns-out-the-mcu) | Servo stall current pulling down logic rail | Isolate servo on dedicated 6.0V 3A buck regulator |
| [10. Intermittent Sensor Disconnects](#10-intermittent-faults-under-vibration) | DuPont friction connectors shaking loose | Replace all wiring with crimped, latched JST-XH harnesses |

---

## Detailed Failure Mode Analyses

### 1. The Car Reports a Wall That Is Not There
* **Symptom**: Front or side ToF distance suddenly drops to $150\text{–}300\text{ mm}$ while cruising down an open straight, triggering an early turn or emergency brake.
* **Root Cause**: The reflective white vinyl mat reflects 940 nm infrared light far more strongly than matte black perimeter walls. Any vehicle pitch or nose-down rake ($1.04^\circ$) dips the lower cone of the VL53L1X into the floor.
* **Fix**:
  1. Fit 3D-printed $2.5 \times 10 \times 20\text{ mm}$ slot collimator snouts and $+2.0^\circ$ upward wedges ([`docs/engineering_findings.md`](engineering_findings.md#1-floor-ir-crosstalk-and-optical-collimation)).
  2. In firmware, enforce photon rate filtering: discard returns where signal rate exceeds `SIGNAL_MIN_MCPS` ($4.0\text{ MCPS}$).

---

### 2. Heading Drifts Over a Run
* **Symptom**: Laps 1 and 2 are smooth and centered, but during Lap 3 the vehicle gradually veers into the outer or inner perimeter wall.
* **Root Cause Investigation**:
  1. **Sensor Quality**: Low-grade IMUs (such as raw MPU6050) suffer from thermal zero-rate drift that accumulates over 60 seconds. *Fix: Replace with an SPI-connected BNO085 running internal sensor fusion.*
  2. **Systematic Turn Bias**: A subtle $1.0^\circ$ overshoot per corner accumulates to $12.0^\circ$ of drift across 12 corners. *Fix: Run Step 6 of calibration and lower `TURN_KP` if the mean error has a consistent sign.*
  3. **Steering Asymmetry**: Left and right turns producing different mean exit errors indicate mechanical servo misalignment. *Fix: Re-run Step 4 calibration for `SERVO_TRUE_STRAIGHT`.*

---

### 3. The Car Turns Twice at One Corner
* **Symptom**: Immediately after completing a $90^\circ$ corner turn, the vehicle detects a line, executes an immediate second turn, and drives backward or into the wall.
* **Root Cause**: As the car swings out of a turn, the downward-facing TCS34725 sensor sweeps across the outer corner marker line it just cleared.
* **Fix**: Enforce an encoder-gated lockout (`POST_CORNER_LOCKOUT_CM = 50 cm`) during which all floor color triggers are suppressed.

---

### 4. After Avoiding a Pillar, the Next Corner Goes the Wrong Way
* **Symptom**: Runs without obstacles complete 12 corners flawlessly. When avoiding an obstacle on the straight leading into a corner, the car turns the opposite direction.
* **Root Cause**: The multi-phase avoidance sub-machine was still executing its realignment phase and held a steering offset when the corner trigger fired.
* **Fix**: Assign **absolute hierarchy priority** to corner turns. If a corner trigger fires, immediately terminate the avoidance state machine and execute the turn. Missing a pillar loses points; missing a corner ends the run.

---

### 5. The Car Squeals in Corners and Runs Wide
* **Symptom**: Audible tire scrub through corners, rubber scuff marks on the mat, wide corner exits, and inconsistent heading changes run-to-run.
* **Root Cause**: Parallelogram steering linkages force both wheels to identical steering angles. In a tight corner, the inner wheel needs a significantly tighter radius than the outer wheel; the difference forces the tires to scrub laterally.
* **Second-Order Effect**: Slipping tires cause the drive axle encoder to over-read, corrupting the lane-gap odometry correction.
* **Fix**: Upgrade to a true 100% Ackermann linkage where knuckle horn axes intersect at the center of the rear axle ([`docs/engineering_findings.md`](engineering_findings.md#3-evolution-of-three-steering-geometries)).

---

### 6. Distance Readings Drift Over the Session
* **Symptom**: `TICKS_PER_CM` was perfectly calibrated in the morning, but by afternoon straightaway distances are systematically off by several centimeters.
* **Root Cause**:
  1. Loose grub screws on the motor-to-axle gear coupler allowing microscopic shaft slip under acceleration.
  2. Performing calibration without the battery or body shell installed: tyre squash under real competition mass ($540\text{ g}$) alters the effective rolling radius.
* **Fix**: Apply blue threadlocker to all drive shaft grub screws and always calibrate at full competition mass.

---

### 7. The Camera Sees Red Where There Is No Red
* **Symptom**: The vision system registers phantom red pillars near the orange corner lines or magenta parking lot boundary limiters.
* **Root Cause**: In HSV color space, orange, red, and magenta sit in close proximity along the hue spectrum. Warm arena lighting or shadows shift the perceived hue into the red envelope.
* **Fix**: Calibrate color thresholds under actual arena lighting using the browser dashboard. Filter out contours that fall within known floor-level regions of interest, and increase `min_blob_area`.

---

### 8. Camera Will Not Open
* **Symptom**: `CameraThread` crashes with a device busy error or V4L2 resource allocation failure upon startup.
* **Root Cause**: Linux allows only one process to hold `/dev/video0`. If the calibration dashboard (`dashboard.py`) is running in the background, the autonomous runner (`main.py`) cannot access the camera.
* **Fix**: Define systemd service mutual exclusion:
  ```ini
  [Unit]
  Description=RoboDash Tuning Dashboard
  Conflicts=robot.service
  ```

---

### 9. Servo Browns Out the MCU
* **Symptom**: The STM32 microcontroller randomly resets during sudden steering transitions, dropping back into `STATE_INIT`.
* **Root Cause**: Rapid servo actuation draws instantaneous current spikes of up to $2.5\text{A}$, causing voltage sags on shared power rails.
* **Fix**:
  1. Power the steering servo from a dedicated $6.0\text{V}$, $3\text{A}$ buck regulator with a $470\,\mu\text{F}$ low-ESR capacitor at the servo plug.
  2. Enforce software slew rate limiting (`SERVO_SLEW = 2.5 deg/cycle`).

---

### 10. Intermittent Faults Under Vibration
* **Symptom**: Vehicle operates reliably on the bench but drops I2C communication or loses encoder counts when driving over track seams and vibration bumps.
* **Root Cause**: DuPont jumper wires depend on friction fits that loosen under chassis vibration.
* **Fix**: Mandate genuine crimped and latched JST-XH connectors with heat-shrink strain relief for all interconnects, and screw terminals with ferrules for power. Breadboards are strictly banned.
