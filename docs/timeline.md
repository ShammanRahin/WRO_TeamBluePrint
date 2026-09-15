# Engineering Log & Timeline — Development Chronicle

> A chronological log documenting the milestones, failures, iterations, and architectural pivots of **Team Blueprint** from initial concept to the Asia Pacific Championship.
> For a high-level system overview, return to the **[Master README](../README.md)**.

---

## 1. Chronological Engineering Log

### 2026-07-12 — Repository Opened & Kinematic Study
* Repository initialized under WRO Future Engineers 2026 guidelines.
* Began initial steering kinematics and parking feasibility studies ([`journal/steering-study-2026-07-12.md`](../journal/steering-study-2026-07-12.md)).
* Initial simulation ranked single central pivot steering as highest in mechanical slop tolerance.

### 2026-07 — Analytical Proof: Textbook 2-Arc Parallel Park Fails
* Implemented Python rigid-body swept polygon simulation ([`src/sim/park_feasibility.py`](../src/sim/park_feasibility.py)).
* **Key Finding**: In a standard bay ($1.5 \times L_{\text{vehicle}}$), a symmetric two-arc reverse park mathematically collides with the outer boundary limiter by **$25.6\text{ mm}$** at $35^\circ$ steering lock.
* Proved the problem is scale-invariant (shortening the vehicle shrinks the bay proportionally).
* Decision: Scrapped two-arc parking; adopted an iterative **multi-point shuffle maneuver** closed on IMU heading ([`DECISIONS.md: #21`](../DECISIONS.md#L72)).

### 2026-07-26 — Major Mechanical & Sensor Architecture Pivot
* Refer to [`journal/day-02-plan-revision-2026-07-26.md`](../journal/day-02-plan-revision-2026-07-26.md).
* **Center Pivot Scrapped**: Track testing showed rotating the whole axle swept the front wheels fore/aft by $\pm 30.1\text{ mm}$, eating $36\%$ of parking bay clearance. Transitioned to dual-knuckle steering.
* **Magnetic Steering Encoder Deleted**: With maneuvers terminating on gyro yaw, closed-loop steering angle feedback was proven mathematically redundant. Removed AS5600 encoder to reduce mass and bus complexity ([`DECISIONS.md: #16`](../DECISIONS.md#L51)).
* **5:1 Drive Gear Reduction Added**: Reduced 25GA motor speed from $1331\text{ RPM}$ to $266\text{ RPM}$ ($0.70\text{ m/s}$ top speed), multiplying odometry resolution to $0.175\text{ mm/count}$ and giving comfortable $28.6\text{ ms}$ sensor dwell over corner lines.

### 2026-07-28 — Floor IR Crosstalk Elimination
* Standard VL53L1X sensors detected the white vinyl mat at $166\text{ mm}$ instead of walls due to a $1.04^\circ$ chassis nose-down rake.
* Developed analytical collimator model ([`electrical/collimator.py`](../electrical/collimator.py)).
* Designed and 3D printed $2.5 \times 10 \times 20\text{ mm}$ slot collimator snouts and $+2.0^\circ$ mechanical wedges, extending ground reflection threshold to **$870\text{ mm}$**.
* Standardized electrical build on two stacked single-sided Dhaka-milled PCBs joined by JST-XH interconnects ([`DECISIONS.md: #25`](../DECISIONS.md#L84)).

### 2026-08 — Autonomous Firmware & Priority Resolution
* Validated non-blocking cooperative state machine for Open Challenge.
* Integrated obstacle avoidance state machine directly on top of the open-round navigation core.
* **Critical Bug Resolved**: When avoiding obstacles near corner entries, the vehicle occasionally turned in the wrong direction upon exiting the maneuver. Resolved by enforcing **absolute corner turn priority** over all avoidance states.

### 2026-08-29 to 09-05 — PCB Carrier Revisions
* Designed and milled Carrier Board revisions A through C in EasyEDA.
* Finalized $90 \times 70\text{ mm}$ single-sided board with bottom-side copper, Raspberry Pi 5 mounting standoffs, and dedicated star ground point.

### 2026-09-01 to 09-04 — Raspberry Pi Perception Stack
* Developed Python perception architecture in [`src/pi/`](../src/pi/).
* Implemented multi-threaded concurrency model (`worldstate.py`) isolating camera acquisition, asyncio LiDAR scanning, and sensor fusion.
* Created web-based calibration dashboard (`dashboard.py`) with atomic config serialization.

---

## 2. Rebuild for the Asia Pacific Championship (Hyderabad, India)

Following our victory at the WRO Bangladesh National Finals, the vehicle underwent a comprehensive rebuild for the international stage:

1. **Digital Metal-Gear Servo Upgrade**:
   * Upgraded from analog MG996R to high-speed digital **JX PS-1171MG**.
   * Re-calibrated steering center (`SERVO_TRUE_STRAIGHT = 69.0°`) and slew rate limiter.
2. **Sensor-Fused IMU (BNO085)**:
   * Replaced bench MPU6050 with an SPI-interfaced **BNO085** running internal quaternion sensor fusion.
   * Yaw drift completely eliminated over 3-lap runs.
3. **True 100% Ackermann Steering Linkage**:
   * Parallelogram steering caused tire scrub in tight corners, which squealed and biased the drive encoder odometry.
   * Re-engineered knuckles so steering arm projections intersect at the center of the rear axle, achieving zero tire scrub and pure odometry.
4. **Distance Sensor Optimization**:
   * With the lane-gap odometry correction holding lateral lane position, side wall-following ToFs were retired, reducing active ToFs to front and rear anti-collision units.
5. **RPLIDAR C1 + Pi 5 Integration**:
   * Upgraded high-level SBC regulator to 5.0V/5.1V 5A to power the Slamtec RPLIDAR C1 DTOF scanner concurrently with the camera pipeline.

---

## 3. Pre-Competition Verification Checklist

- [x] Autonomous direction detection (clockwise / counter-clockwise) via first-pair line crossing order.
- [x] 12-corner 3-lap run completion on competition mat with zero wall strikes.
- [x] Optical collimation verified under bright ambient arena lighting.
- [x] Camera-LiDAR fusion pipeline tested with live point cloud visualization.
- [ ] Finalize UART serial packet emitter in `src/pi/main.py` to stream live obstacle vectors to the STM32.
- [ ] Capture final competition-ready chassis photographs for `v-photos/` and `t-photos/`.
