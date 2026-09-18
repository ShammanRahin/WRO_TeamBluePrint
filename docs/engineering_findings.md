# Engineering Findings & Iterations — What Broke and What We Learned

> This document records the empirical discoveries, analytical failure proofs, and hardware design revisions made by **Team Blueprint** throughout the development of our WRO Future Engineers vehicle.
> For the high-level system overview, return to the **[Master README](../README.md)**.

---

## 1. Floor IR Crosstalk and Optical Collimation

<p align="center">
  <img src="../media/diagrams/tof-floor-crosstalk.svg" alt="ToF cone hitting the floor, before and after collimation" width="95%"/>
</p>

### The Problem
* **Optical Environment**: The WRO Future Engineers mat is high-reflectance white vinyl (Rule 13.2), while perimeter and interior corridor walls are low-reflectance matte black MDF (Rules 13.4, 13.6).
* **Optical Spread**: The VL53L0X has a fixed, uncollimated Field of View of $\approx 25^\circ$ (no programmable ROI). The VL53L1X used by the obstacle program has a similar default FoV.
* **Chassis Rake**: Due to front wheel diameter ($46\text{ mm}$) and rear wheel diameter ($50\text{ mm}$), the chassis sits with a natural nose-down rake:
  ```math
  \text{rake} = \arctan\left(\frac{(50 - 46)/2}{L}\right) = \arctan\left(\frac{2}{136.14}\right) \approx 0.84^\circ\text{ nose-down}
  ```
  (The July analysis used the then-measured $L = 110\text{ mm}$, giving $1.04^\circ$.)
* **Consequence**: The bottom of the sensor's infrared cone struck the reflective white floor at just $166\text{ mm}$. Because the white mat returned a vastly higher photon count than the distant black wall, the sensor locked onto the floor, reporting phantom walls and triggering false corner turns.

### The Analytical & Physical Solution
The mitigation was engineered in Python ([`electrical/collimator.py`](../electrical/collimator.py)) and validated on the mat:

1. **Printed Slot Collimator Baffles**:
   * 3D printed a narrow horizontal slit aperture: $2.5\text{ mm tall} \times 10\text{ mm wide} \times 20\text{ mm deep}$.
   * Narrows the vertical optical half-angle from $12.5^\circ$ down to:
     ```math
     \theta_v = \arctan\left(\frac{1.25}{20}\right) \approx 3.58^\circ
     ```
   * The slot is intentionally wide ($10\text{ mm}$) because the VL53 emitter and receiver apertures are spaced $2.5\text{ mm}$ apart; a round aperture would vignette the detector.
2. **Mechanical Upward Mounting Wedge ($+2.0^\circ$)**:
   * Cancels the $0.84^\circ$ chassis rake and adds upward tilt on top.
   * Net effective vertical half-angle relative to the floor:
     ```math
     \theta_{\text{eff}} = 3.58^\circ + 0.84^\circ - 2.0^\circ = 2.42^\circ
     ```
   * Pushes the first ground reflection out to:
     ```math
     d_{\text{floor}} = \frac{h}{\tan(\theta_{\text{eff}})} = \frac{40\text{ mm}}{\tan(2.42^\circ)} \approx \mathbf{947\text{ mm}}
     ```
     (`python electrical/collimator.py --wheelbase 136.14`; with the July $110\text{ mm}$ wheelbase the same solver gives $870\text{ mm}$).
   * Either way this clears the nominal corridor side walls ($442.5\text{ mm}$) with over $400\text{ mm}$ of margin.
3. **Signal Strength Filter** (obstacle program, VL53L1X):
   * The obstacle firmware accepts a reading only if `RangeStatus` is valid, the peak signal rate is **at least** `SIGNAL_MIN_MCPS` (4.0), and the range is under `TOF_MAX_VALID_MM` (1300 mm) — i.e. it discards *weak* returns.
   * ⚠️ This is the opposite of the reasoning in calibration step 5 ([`05_tof_signal_threshold.cpp`](../src/tools/calibration/05_tof_signal_threshold.cpp)), which argues that floor returns are *strong* and should be cut with a ceiling. The two need reconciling against real step-5 data. The open-round VL53L0X path has no signal filter at all.

---

## 2. Analytical Failure of Textbook Two-Arc Parallel Parking

<p align="center">
  <img src="../media/steering/7_park_feasibility.png" alt="Parking Feasibility Analysis" width="90%"/>
</p>

### The Problem
WRO rules specify a parking bay length of only $1.5 \times L_{\text{vehicle}}$ ($247.5\text{ mm}$ for our $165\text{ mm}$ chassis), bounded by two rigid magenta limiters ($200 \times 20 \times 100\text{ mm}$). Touching a limiter results in 0 parking points (Rule 9.24.7).

Textbook robotics literature prescribes a symmetric reverse two-arc trajectory:
1. Turn rearward at maximum lock into the bay.
2. Counter-steer at maximum lock to straighten along the curb.

### Mathematical Proof of Failure
We developed a rigid-body swept polygon solver ([`src/sim/park_feasibility.py`](../src/sim/park_feasibility.py)) sweeping the full vehicle body envelope across all feasible longitudinal entry positions.

| Steering Lock ($\delta$) | Turn Radius ($R$) | Ratio $R/L$ | Best Clearance to Limiter | Analytical Verdict |
|:---:|:---:|:---:|:---:|:---:|
| **$35^\circ$ (As-Built)** | **$157\text{ mm}$** | **0.95** | **$-25.6\text{ mm}$** | **COLLISION** |
| $40^\circ$ | $131\text{ mm}$ | 0.79 | $-10.8\text{ mm}$ | COLLISION |
| $45^\circ$ | $110\text{ mm}$ | 0.67 | $0.0\text{ mm}$ | Touches — no margin |
| $60^\circ$ | $64\text{ mm}$ | 0.38 | $+0.5\text{ mm}$ | Beyond the built linkage's lock |

> These rows were computed with the July wheelbase measurement ($L = 110\text{ mm}$, `--wheelbase 110`). The CAD wheelbase of $136.14\text{ mm}$ is rejected by the solver because it exceeds the $117\text{ mm}$ that fits a $165\text{ mm}$ body with $46/50\text{ mm}$ wheels — the body length, overhang or wheelbase figure needs reconciling. A longer wheelbase raises $R$ to $194.4\text{ mm}$ ($R/L \approx 1.18$), which moves further away from the $R/L \le 0.7$ a two-arc park needs, so the conclusion stands.

### Fundamental Insight
* **Scale-Invariance**: The available longitudinal clearance is $\Delta L = 0.5 \times L_{\text{vehicle}}$. Shortening the car also shortens the parking bay by exactly the same proportion.
* **Geometric Limit**: A symmetric two-arc park needs roughly $\frac{R}{L} \le 0.70$ (about $45^\circ$ lock or more), which cannot clear reliably on an open-loop steering chassis under real tyre deformation.

### Solution: Multi-Point Yaw-Closed Shuffle
We abandoned the two-arc manoeuvre in favour of an iterative **multi-point shuffle** closed on IMU heading: enter at an angle, then make small forward-reverse arcs while measuring true heading until the car is square to the wall.

> **Status:** planned. Neither firmware program contains a parking phase yet (`ObstacleRound.cpp` ends with "No parking").

---

## 3. Evolution of Three Steering Geometries

<p align="center">
  <img src="../media/diagrams/steering-evolution.svg" alt="Centre pivot, parallelogram and true Ackermann compared" width="100%"/>
</p>

### Attempt 1: Central Pivot Turntable
* **Concept**: A single central bearing pivot rotating the entire front axle beam.
* **Simulation Prediction**: A kinematic Monte-Carlo slop model plus a pymunk friction sim showed the single bearing joint tolerated build slop about $3\times$ better than multi-link mechanisms ([`journal/steering-study-2026-07-12.md`](../journal/steering-study-2026-07-12.md)).
* **Why It Failed on the Track**:
  Rotating the entire front beam translated the outer front tire forward and the inner tire rearward by:
  ```math
  \Delta x = \frac{W}{2} \cdot \sin(\delta) = \frac{105\text{ mm}}{2} \cdot \sin(35^\circ) \approx \pm 30.1\text{ mm}
  ```
  This longitudinal wheel sweep consumed **$36\%$ of the entire $82.5\text{ mm}$ parking slack** — clearance the car needs most exactly when it is turning.

### Attempt 2: Parallelogram (Tie-Bar) Steering
* **Concept**: Two independent steering knuckles coupled by a rigid parallel tie-bar, driven by a single digital servo.
* **Result**: Knuckle pivot centers remained fixed, eliminating the longitudinal tire sweep and fixing the swept envelope.
* **Why It Failed on the Track**:
  In any turn, the inner wheel follows a tighter turning radius than the outer wheel:
  ```math
  R_{\text{inner}} = \frac{L}{\tan\delta_{\text{inner}}} \quad < \quad R_{\text{outer}} = \frac{L}{\tan\delta_{\text{outer}}}
  ```
  A parallelogram forces both knuckles to the exact same angle ($\delta_{\text{inner}} = \delta_{\text{outer}}$). The angular difference between the ideal rolling vectors was forced into **continuous lateral tire scrub**:
  1. Audible tire squeal and rubber scuffing across corner turns.
  2. Inconsistent corner exit headings run-to-run.
  3. **Odometry Corruption**: A scrubbing tire slips, causing the drive axle encoder to over-read. Because our lane-gap correction relies on encoder distance between corner lines, parallelogram scrub corrupted the lateral correction loop.

### Attempt 3: Ackermann Linkage (As-Built)
* **CAD Ground Truth (Autodesk Fusion 360)**:
  * **Wheelbase ($L$)**: $136.14\text{ mm}$ (measured $136.139\text{ mm}$ axle-to-axle, $136.126\text{ mm}$ rim-to-rim).
  * **Kingpin Pivot Spacing ($K_w$)**: $80.59\text{ mm}$ (measured $80.589\text{ mm}$ between knuckle pivot pins).
  * **Extreme Track Width ($W$)**: $114.24\text{ mm}$ (measured $114.244\text{ mm}$ outer wheel to outer wheel).
  * **Ackermann Arm Angle**: Knuckle steering arms are angled inward at **$107.0^\circ$ / $73.0^\circ$** ($17.0^\circ$ inclination from longitudinal axis).
* **Kinematic Convergence Proof**:
  To achieve pure 100% Ackermann rolling where arm extension rays intersect at the center of the rear axle:
  ```math
  \alpha = \arctan\left(\frac{K_w / 2}{L}\right) = \arctan\left(\frac{80.589\text{ mm} / 2}{136.139\text{ mm}}\right) = \arctan(0.29597) = 16.49^\circ \approx 16.5^\circ
  ```
  The CAD design angle of **$17.0^\circ$** is within $0.5^\circ$ of this, so the arm lines meet just ahead of the rear axle centre — very close to full Ackermann.
* **Kinematic Result**:
  ```math
  \cot\delta_{\text{outer}} - \cot\delta_{\text{inner}} = \frac{K_w}{L}
  ```
  (the Ackermann condition uses the kingpin spacing $K_w$). Both front wheels then roll close to perpendicular to rays from a single instantaneous centre of rotation, so front-tyre scrub is largely removed and cornering is quieter and more repeatable. The solid rear axle still scrubs in corners (see Decision #14), but that does not bias the axle encoder. Minimum turn radius at $35^\circ$ lock is $R = \frac{L}{\tan(35^\circ)} = 194.4\text{ mm}$.

---

## 4. Elimination of Steering Angle Encoder (Feedforward + IMU)

### Original Plan
The initial architecture specified an AS5600 magnetic rotary encoder mounted on the steering pivot shaft to provide closed-loop angular feedback and cancel servo backlash.

### The Optimization Insight
When moving to two-knuckle steering there was no single pivot shaft left to put the encoder on. The analysis showed that **steering-angle feedback is not needed for navigation**:
1. All vehicle turns and maneuvers terminate strictly on **measured vehicle yaw displacement from the BNO085 IMU**, not on the steering wheel position.
2. Between corners, the heading controller only requires that the steering actuator be monotonic and repeatable—not calibrated to absolute perfection.

Dropping the AS5600 saved mass, mechanical complexity and an I²C device. The cost, recorded in Decision #16, is that the turn radius is nominal rather than measured — which matters for parking, not for the 12 corners.
