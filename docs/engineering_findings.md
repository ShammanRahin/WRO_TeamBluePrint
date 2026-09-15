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
* **Optical Spread**: Standard VL53L1X Time-of-Flight sensors possess an uncollimated vertical Field of View (FoV) of $\approx 25^\circ$.
* **Chassis Rake**: Due to front wheel diameter ($46\text{ mm}$) and rear wheel diameter ($50\text{ mm}$), the chassis sits with a natural nose-down rake:
  $$\text{rake} = \arctan\left(\frac{50 - 46}{2 \times 110}\right) \approx 1.04^\circ\text{ nose-down}$$
* **Consequence**: The bottom of the sensor's infrared cone struck the reflective white floor at just $166\text{ mm}$. Because the white mat returned a vastly higher photon count than the distant black wall, the sensor locked onto the floor, reporting phantom walls and triggering false corner turns.

### The Analytical & Physical Solution
The mitigation was engineered in Python ([`electrical/collimator.py`](../electrical/collimator.py)) and validated on the mat:

1. **Printed Slot Collimator Baffles**:
   * 3D printed a narrow horizontal slit aperture: $2.5\text{ mm tall} \times 10\text{ mm wide} \times 20\text{ mm deep}$.
   * Narrows the vertical optical half-angle from $12.5^\circ$ down to:
     $$\theta_v = \arctan\left(\frac{1.25}{20}\right) \approx 3.58^\circ$$
   * The slot is intentionally wide ($10\text{ mm}$) because the VL53 emitter and receiver apertures are spaced $2.5\text{ mm}$ apart; a round aperture would vignette the detector.
2. **Mechanical Upward Mounting Wedge ($+2.0^\circ$)**:
   * Offsets the $1.04^\circ$ chassis rake and provides an additional upward tilt.
   * Net effective vertical half-angle relative to the floor:
     $$\theta_{\text{eff}} = 3.58^\circ + 1.04^\circ - 2.0^\circ = 2.62^\circ$$
   * Pushes the first ground reflection threshold out to:
     $$d_{\text{floor}} = \frac{h}{\tan(\theta_{\text{eff}})} = \frac{40\text{ mm}}{\tan(2.62^\circ)} \approx \mathbf{870\text{ mm}}$$
   * This completely clears the nominal corridor side walls ($442.5\text{ mm}$) with over $400\text{ mm}$ of margin.
3. **Signal Strength Discard Filter**:
   * Floor returns exhibit high photon counts ($> 4.0\text{ MCPS}$), while true matte black wall returns exhibit lower counts. Firmware discards any signal above `SIGNAL_MIN_MCPS` as floor noise.

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
| $45^\circ$ | $110\text{ mm}$ | 0.67 | $+0.1\text{ mm}$ | Zero Practical Margin |
| $60^\circ$ | $64\text{ mm}$ | 0.38 | $+0.5\text{ mm}$ | Mechanically Impossible |

### Fundamental Insight
* **Scale-Invariance**: The available longitudinal clearance is $\Delta L = 0.5 \times L_{\text{vehicle}}$. Shortening the car also shortens the parking bay by exactly the same proportion.
* **Geometric Limit**: A symmetric two-arc park geometrically requires $\frac{R}{L} \le 0.70$ (requiring $> 45^\circ$ lock), which cannot clear reliably on an open-loop steering chassis under real tire deformation.

### Solution: Multi-Point Yaw-Closed Shuffle
We abandoned the two-arc maneuver in favor of an iterative **multi-point shuffle maneuver** closed on IMU heading. The vehicle enters at an angle, performs small forward-reverse arcs while measuring true heading and ToF wall proximity, and squares itself against the curb.

---

## 3. Evolution of Three Steering Geometries

<p align="center">
  <img src="../media/diagrams/steering-evolution.svg" alt="Centre pivot, parallelogram and true Ackermann compared" width="100%"/>
</p>

### Attempt 1: Central Pivot Turntable
* **Concept**: A single central bearing pivot rotating the entire front axle beam.
* **Simulation Prediction**: PyMunk simulations showed a single bearing joint tolerated manufacturing tolerances $3\times$ better than multi-link mechanisms.
* **Why It Failed on the Track**:
  Rotating the entire front beam translated the outer front tire forward and the inner tire rearward by:
  $$\Delta x = \frac{W}{2} \cdot \sin(\delta) = \frac{105\text{ mm}}{2} \cdot \sin(35^\circ) \approx \pm 30.1\text{ mm}$$
  This longitudinal wheel sweep consumed **$36\%$ of the entire $82.5\text{ mm}$ parking clearance**, causing the outer tire to clip boundary walls during tight turns.

### Attempt 2: Parallelogram (Tie-Bar) Steering
* **Concept**: Two independent steering knuckles coupled by a rigid parallel tie-bar, driven by a single digital servo.
* **Result**: Knuckle pivot centers remained fixed, eliminating the longitudinal tire sweep and fixing the swept envelope.
* **Why It Failed on the Track**:
  In any turn, the inner wheel follows a tighter turning radius than the outer wheel:
  $$R_{\text{inner}} = \frac{L}{\tan\delta_{\text{inner}}} \quad < \quad R_{\text{outer}} = \frac{L}{\tan\delta_{\text{outer}}}$$
  A parallelogram forces both knuckles to the exact same angle ($\delta_{\text{inner}} = \delta_{\text{outer}}$). The angular difference between the ideal rolling vectors was forced into **continuous lateral tire scrub**:
  1. Audible tire squeal and rubber scuffing across corner turns.
  2. Inconsistent corner exit headings run-to-run.
  3. **Odometry Corruption**: A scrubbing tire slips, causing the drive axle encoder to over-read. Because our lane-gap correction relies on encoder distance between corner lines, parallelogram scrub corrupted the lateral correction loop.

### Attempt 3: True 100% Ackermann Linkage (As-Built)
* **CAD Ground Truth (Autodesk Fusion 360)**:
  * **Wheelbase ($L$)**: $136.14\text{ mm}$ (measured $136.139\text{ mm}$ axle-to-axle, $136.126\text{ mm}$ rim-to-rim).
  * **Kingpin Pivot Spacing ($K_w$)**: $80.59\text{ mm}$ (measured $80.589\text{ mm}$ between knuckle pivot pins).
  * **Extreme Track Width ($W$)**: $114.24\text{ mm}$ (measured $114.244\text{ mm}$ outer wheel to outer wheel).
  * **Ackermann Arm Angle**: Knuckle steering arms are angled inward at **$107.0^\circ$ / $73.0^\circ$** ($17.0^\circ$ inclination from longitudinal axis).
* **Kinematic Convergence Proof**:
  To achieve pure 100% Ackermann rolling where arm extension rays intersect at the center of the rear axle:
  $$\alpha = \arctan\left(\frac{K_w / 2}{L}\right) = \arctan\left(\frac{80.589\text{ mm} / 2}{136.139\text{ mm}}\right) = \arctan(0.29597) = 16.49^\circ \approx 16.5^\circ$$
  The CAD design angle of **$17.0^\circ$** matches the theoretical Ackermann convergence directly at the rear axle center to within $0.5^\circ$.
* **Kinematic Result**:
  $$\cot\delta_{\text{outer}} - \cot\delta_{\text{inner}} = \frac{W}{L}$$
  Both front wheels roll perpendicular to rays emanating from a single instantaneous center of rotation. Wheel scrub is completely eliminated, cornering is silent and repeatable, and drive axle odometry remains perfectly unbiased. Minimum turn radius at $35^\circ$ lock is $R = \frac{L}{\tan(35^\circ)} = 194.4\text{ mm}$.

---

## 4. Elimination of Steering Angle Encoder (Feedforward + IMU)

### Original Plan
The initial architecture specified an AS5600 magnetic rotary encoder mounted on the steering pivot shaft to provide closed-loop angular feedback and cancel servo backlash.

### The Optimization Insight
When transitioning to dual-knuckle steering, mounting an on-axis magnetic encoder became mechanically complex. However, mathematical analysis revealed that **steering angle feedback is functionally redundant**:
1. All vehicle turns and maneuvers terminate strictly on **measured vehicle yaw displacement from the BNO085 IMU**, not on the steering wheel position.
2. Between corners, the heading controller only requires that the steering actuator be monotonic and repeatable—not calibrated to absolute perfection.

Eliminating the AS5600 saved mass, mechanical complexity, and an extra I2C bus address with zero degradation in navigational precision.
