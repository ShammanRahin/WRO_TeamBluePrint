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
* Proved the problem is scale-invariant (shortening the vehicle shrinks the bay proportionally). The simulation used the then-measured 110 mm wheelbase.
* Decision: Scrapped two-arc parking; adopted an iterative **multi-point shuffle maneuver** closed on IMU heading ([`DECISIONS.md` #21](../DECISIONS.md)).

### 2026-07-26 — Major Mechanical & Sensor Architecture Pivot
* Refer to [`journal/day-02-plan-revision-2026-07-26.md`](../journal/day-02-plan-revision-2026-07-26.md).
* **Center Pivot Scrapped**: Rotating the whole front beam swings each front wheel fore/aft by $\pm 30.1\text{ mm}$, eating $36\%$ of parking-bay slack. The car was built with parallelogram (two-knuckle) steering instead.
* **Magnetic Steering Encoder Deleted**: With manoeuvres terminating on gyro yaw, steering-angle feedback is not needed for navigation. Removed AS5600 encoder to reduce mass and bus complexity ([`DECISIONS.md` #16](../DECISIONS.md)).
* **5:1 Drive Gear Reduction Added**: Reduced 25GA motor speed from $1331\text{ RPM}$ to $266\text{ RPM}$ ($0.70\text{ m/s}$ top speed), multiplying odometry resolution by 5 and giving a comfortable $28.6\text{ ms}$ sensor dwell over corner lines.

### 2026-07-28 — Floor IR Crosstalk Elimination
* The bare VL53L0X sensors detected the white mat at $166\text{ mm}$ instead of walls, aggravated by a $\sim 1^\circ$ chassis nose-down rake.
* Developed analytical collimator model ([`electrical/collimator.py`](../electrical/collimator.py)).
* Designed and 3D printed $2.5 \times 10 \times 20\text{ mm}$ slot collimator snouts and $+2.0^\circ$ mechanical wedges, extending the first floor return to **$870\text{ mm}$** (at the 110 mm wheelbase then assumed; $\approx 947\text{ mm}$ at the 136.14 mm CAD wheelbase).
* Standardized electrical build on two stacked single-sided Dhaka-milled PCBs joined by JST-XH interconnects ([`DECISIONS.md` #25](../DECISIONS.md)).

### 2026-08 — Autonomous Firmware & Priority Resolution
* Validated non-blocking cooperative state machine for Open Challenge.
* Integrated obstacle avoidance state machine directly on top of the open-round navigation core.
* **Critical Bug Resolved**: When avoiding obstacles near corner entries, the vehicle occasionally turned in the wrong direction upon exiting the manoeuvre. Resolved by giving corners priority: avoidance is not started once a corner is armed and the wall is close.
* **2026-08-20 — WRO Bangladesh National Final: 1st place, Future Engineers.**

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
   * Stationary heading holds to about 0.01° on the bench (per the open-round firmware notes).
3. **Ackermann Steering Linkage**:
   * Parallelogram steering caused tire scrub in tight corners, which squealed and biased the drive encoder odometry.
   * Re-engineered knuckles (107°/73° arms) so steering-arm projections meet near the centre of the rear axle, largely removing front-tyre scrub.
4. **Distance Sensor Reduction**:
   * With the lane-gap odometry correction holding lateral lane position, the side ToFs were dropped from the Open program. The current `OpenRound.cpp` uses a single front VL53L0X (mux ch 3) for turn confirmation and wall recovery. `ObstacleRound.cpp` still uses three VL53L1X (left, right, front).
5. **RPLIDAR C1 + Pi 5 Integration**:
   * Upgraded high-level SBC regulator to 5.1V 5A to power the Pi 5 and the Slamtec RPLIDAR C1 together.
6. **Carrier Re-Pin & Open-Round Firmware Update (2026-09-15 → 09-17)**:
   * `OpenRound.cpp` moved to the new carrier pin map (motor PA2/PA3, encoder TIM5 on PA0/PA1, IMU SPI on PA5–PA7, mux reset PB8, status LEDs PB12–PB14), with re-measured `TICKS_PER_CM` (14.853), `SERVO_TRUE_STRAIGHT` (71°) and colour thresholds.
   * Added `STATE_RECOVER`: back off when the front wall is under 200 mm, ignoring re-crossed floor lines.
   * Added the `tof_test.ino` bench sketch for colour-threshold capture; on 2026-09-18 it was replaced by a three-ToF test for the front VL53L0X (ch 3) and two new side VL53L0X (ch 5, ch 6), which the firmware does not use yet.
   * Still to do: port `ObstacleRound.cpp` and the calibration sketches to the new pin map; wire and read the start button.

---

## 3. Pre-Competition Verification Checklist

- [x] Autonomous direction detection (clockwise / counter-clockwise) from the first corner-line colour.
- [x] 12-corner 3-lap run completion on competition mat with zero wall strikes.
- [x] Optical collimation verified under bright ambient arena lighting.
- [x] Camera-LiDAR fusion pipeline tested with live point cloud visualization.
- [ ] Finalize UART serial packet emitter in `src/pi/main.py` to stream live obstacle vectors to the STM32.
- [ ] Wire the start button on the current carrier and make `OpenRound.cpp` wait for it (rule 9.11).
- [ ] Port `ObstacleRound.cpp` and `hardware_config.h` to the current pin map.
- [ ] Capture final competition-ready photographs for `v-photos/` and `t-photos/`.
- [ ] Upload CAD sources and STL files to `models/`.
- [ ] Regenerate `schemes/wiring_block_diagram.png` for the as-built electronics.
