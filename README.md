# Team Blueprint — WRO Future Engineers 2026

<div align="center">

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![WRO Category](https://img.shields.io/badge/WRO-Future%20Engineers%202026-brightgreen.svg)](#rule-compliance)
[![MCU](https://img.shields.io/badge/Master%20MCU-STM32F411CEU6-blue)](SPECSHEET.md)
[![Vision](https://img.shields.io/badge/Vision%20SBC-Raspberry%20Pi%204B-red)](src/vision/)
[![Design](https://img.shields.io/badge/Design%20Philosophy-Evidence--Driven-orange)](DECISIONS.md)

**An autonomous self-driving vehicle engineered from first principles by students from Bangladesh for the World Robot Olympiad (WRO) Future Engineers 2026 competition.**

[Overview](#overview) • [Specifications](#vehicle-specifications) • [Architecture](#system-architecture) • [Control & Navigation](#autonomous-navigation-strategy) • [Engineering Findings](#key-engineering-findings) • [Electronics](#electrical--pcb-design) • [Simulations](#simulation--reproduction) • [Repository Structure](#repository-structure) • [Team](#team--acknowledgments)

</div>

---

## Overview

**Team Blueprint** represents a rigorous, evidence-driven approach to autonomous miniature vehicle robotics. Rather than relying on heuristic trial-and-error, every subsystem of this vehicle—from kinematic linkage geometry and optical time-of-flight physics to multi-rail power isolation—is supported by numerical simulations, analytical models, and empirical testing logs.

This repository serves as the complete, transparent engineering logbook of the vehicle:
- **Design rationale**: Not only *what* was designed, but the quantitative *why*.
- **Empirical revisions**: Superseded designs and failed assumptions are retained with post-mortem analyses rather than wiped clean.
- **Source of truth documents**:
  - [`SPECSHEET.md`](SPECSHEET.md) — Comprehensive technical parameter limits, calibrated values, and pinouts.
  - [`DECISIONS.md`](DECISIONS.md) — Chronological architectural decision records (ADRs) with dated supersession blocks.
  - [`BOM.md`](BOM.md) — Component sourcing, inventory status, lead times, and unit costs.

---

## Vehicle Specifications

| Category | Parameter | Measured / Engineered Value | Rule Limit / Target | Notes |
|---|---|---|---|---|
| **Envelope** | Scored Footprint | **165 × 115 mm** | ≤ 300 × 200 mm | Ultra-compact design to maximize parking slack |
| | Height | **50 mm** (Open) / **~85 mm** (Obstacle) | ≤ 300 mm | Minimal CG height; camera mast modular |
| | Total Mass | **~420 g** (Open) / **~510 g** (Obstacle) | Unrestricted | Low-inertia vehicle for rapid deceleration |
| **Chassis** | Wheelbase ($L$) | **110 mm** | Measured | Optimized against turning radius |
| | Track Width ($W$) | **105 mm** (center-to-center) / **115 mm** (extreme) | — | 115 mm total outer width |
| | Wheel Diameter | **46 mm** (Front) / **50 mm** (Rear) | — | 1.1° natural forward rake |
| **Kinematics** | Steering Mechanism | **Parallelogram Tie-Bar** (Single Servo) | — | Replaced center-pivot to eliminate swept envelope growth |
| | Steering Range | **±35°** at wheel knuckles | — | Actuated via MG996R metal-gear servo |
| | Minimum Turning Radius | **157 mm** ($L / \tan 35^\circ$) | — | Centers inside standard 1000 mm driving lane |
| **Powertrain** | Primary Motor | **25GA DC Gearmotor** (12V) | Max 1 motor | Rule 11.5 compliant |
| | Reduction | **5:1 Spur Gear Final Drive** | — | High starting torque, eliminates stall cogging |
| | Drive Topology | **Solid rear axle** (No differential) | Max 1 driven axle | Maximizes straight-line odometry consistency |
| | Maximum Speed | **0.70 m/s** | — | Software throttled for predictable braking |
| **Sensors** | Odometry Resolution | **0.175 mm / count** | — | Quadrature optical/magnetic motor encoder |
| | Inertial Measurement | **MPU6050 / SPI 6-DoF IMU** | — | 1 kHz internal sampling for heading integration |
| | Distance Array | **4× VL53L0X Time-of-Flight (ToF)** | — | Equipped with custom 3D-printed optical collimators |
| | Ground Color Sensing | **TCS34725 RGB Sensor** | — | Downward-facing with isolated illumination hood |
| **Compute** | Low-Level Master | **STM32F411CEU6 (Black Pill)** | — | ARM Cortex-M4 @ 100 MHz, Hardware FPU, Real-Time Loop |
| | High-Level Vision | **Raspberry Pi 4B (2GB/4GB)** | — | Fisheye camera, OpenCV pillar classification (Obstacle only) |

### Rule Compliance Matrix

- **Mechanical (Rules 11.3, 11.5, 11.13)**: Exactly four wheels, single steering actuator, single drive motor coupled via a fixed gear reduction to a single solid axle.
- **RF & Telemetry (Rule 11.10)**: All wireless interfaces (Wi-Fi, Bluetooth) are disabled at the kernel boot level (`config.txt`) on the Raspberry Pi. Zero external communication during runs.
- **Power & Control Interface (Rules 9.10, 9.11)**: Dedicated master physical toggle switch for complete battery cut-off, plus a separate momentary push-button dedicated solely to initiating autonomous runs.

---

## System Architecture

The robot employs a **dual-tier heterogeneous compute hierarchy**:

```mermaid
graph TD
    subgraph Power ["Power Subsystem (Single Star Ground)"]
        BAT["2S LiPo / Li-Ion Battery"] --> REG6["6.0V 3A Buck (Servo Only)"]
        BAT --> REG5["5.0V 2A Buck (STM32 & Sensors)"]
        BAT --> REG51["5.1V 3A Buck (Pi 4B Harness)"]
        BAT --> DRV_PWR["Direct Battery Rail (BTS7960)"]
    end

    subgraph LowLevel ["Real-Time Tier (STM32F411CEU6)"]
        STM["STM32 Master Controller"]
        IMU["6-DoF IMU (Yaw Heading)"] -->|I2C / SPI| STM
        ENC["Encoder (Solid Axle)"] -->|Timer Quadrature| STM
        BTN["Start Pushbutton"] -->|GPIO EXTI| STM
        MUX["PCA9548A I2C Mux (3.3V)"] <-->|I2C Master| STM
        MUX --> TOF1["VL53L0X Front"]
        MUX --> TOF2["VL53L0X Left (L90)"]
        MUX --> TOF3["VL53L0X Right (R90)"]
        MUX --> COL["TCS34725 Floor Color"]
        
        STM -->|PWM 50Hz| SERVO["MG996R Steering Servo"]
        STM -->|PWM + EN| BTS["BTS7960 H-Bridge Driver"] --> MOTOR["25GA Drive Motor"]
    end

    subgraph HighLevel ["Vision Tier (Obstacle Round Only)"]
        PI["Raspberry Pi 4B"]
        CAM["160° Fisheye Camera"] -->|CSI / V4L2| PI
        PI -->|Checksummed Packet UART @ 115200| STM
    end

    style Power fill:#fff3e0,stroke:#f57c00,stroke-width:2px
    style LowLevel fill:#e3f2fd,stroke:#1976d2,stroke-width:2px
    style HighLevel fill:#fce4ec,stroke:#c2185b,stroke-width:2px
```

### Complete Electrical Schematic
A full physical connection diagram is illustrated below, mapping PCB pinouts, line terminations, and bus distributions:

![Wiring Block Diagram](schemes/wiring_block_diagram.png)

---

## Autonomous Navigation Strategy

### 1. Open Challenge (Deterministic Real-Time Firmware)
In the Open Challenge, the vehicle runs **exclusively on the STM32F411**, operating on a zero-drift, state-driven control cycle:

1. **Stationary Bias Calibration (Arming)**: Upon boot, stationary gyro offsets are auto-zeroed in flash. Conforms strictly with Rule 9.6 without requiring manual pit calibration.
2. **Centerline Wall Alignment**: Lateral ToF sensors ($L_{90}$ and $R_{90}$) balance distance against perimeter walls to establish the true track heading reference.
3. **Heading-Hold Cruise**: The vehicle drives straight using a proportional-derivative heading controller referenced to the gyro. Encoder odometry measures exact corridor displacement.
4. **Surface Marker Detection**: Downward-facing TCS34725 color sensor identifies the high-contrast orange and blue floor lines.
5. **Deterministic Direction Decoding**:
   - The first corner crossed has dual lines (Orange + Blue).
   - The detection order (`Orange → Blue` vs `Blue → Orange`) resolves the randomized race direction (Rule 9.3) dynamically.
6. **IMU-Terminated Turn Execution**:
   - The steering servo commands full ±35° lock.
   - **Crucial Design Rule**: The turn does *not* complete based on distance or time; it completes when integrated IMU yaw hits exactly 90.0°. Wheel slip and kinematic scrub cannot corrupt this threshold.
   - A temporal lockout mask prevents false line-trigger re-entry until the vehicle clears the intersection.
7. **Lap Counting & Finish**: After completing 12 consecutive 90° turns (3 full laps), the vehicle centers itself into the start sector and engages dynamic motor braking.

### 2. Obstacle Challenge (Vision-Assisted Navigation)
- **High-Level Computer Vision**: The Raspberry Pi runs a lightweight, multithreaded Python/C++ pipeline utilizing HSV thresholding, contour extraction, and bounding-box aspect ratio filtering to classify Red (pass right) and Green (pass left) pillars.
- **Safety Isolation Guarantee**: Vision coordinates are transmitted via fixed-size checksummed UART packets. The STM32 treats vision as an advisory input. If packets drop, freeze, or corrupt, the STM32 defaults instantly to its deterministic wall-following safety protocol. **The Raspberry Pi can never stall or crash the vehicle.**

---

## Key Engineering Findings

Through rigorous physical validation, four primary engineering hypotheses were challenged and redesigned:

### 1. Floor IR Crosstalk & Sensor Collimation
* **Problem**: The WRO mat is high-reflectance white vinyl (Rule 13.2), while perimeter walls are low-reflectance matte black (Rules 13.4, 13.6). Standard VL53L0X ToF sensors possess a 25° field of view without software-defined regions of interest. Chassis rake (1.1° nose-down) caused the sensors to trigger on the floor at 166 mm instead of detecting walls.
* **Solution**: Developed [`electrical/collimator.py`](electrical/collimator.py) to calculate optical snout baffles. 3D-printed narrow 2.5 × 10 × 20 mm slot collimators paired with +2.0° mechanical upward wedges pushed the first ground reflection threshold from 166 mm out to **870 mm**, completely clearing side walls at 442.5 mm.

### 2. Analytical Failure of Textbook Two-Arc Parallel Parking
* **Problem**: Kinematic analysis ([`src/sim/park_feasibility.py`](src/sim/park_feasibility.py)) proved that a standard symmetric reverse two-arc parking trajectory **fails by 25.6 mm** at 35° steering lock within WRO designated bay limits ($1.5 \times L$).
* **Insight**: The problem is scale-invariant—shortening the car shrinks the bay proportionally.
* **Solution**: Implemented an iterative **multi-point shuffle maneuver closed on IMU yaw feedback**, robust to open-loop steering backlash and varying floor friction.

### 3. Kinematic Center-Pivot vs. Parallelogram Linkage
* **Problem**: The team originally built a single central pivot front axle because simulations showed 3× higher tolerance to mechanical joint slop.
* **Empirical Flaw**: During physical runs, rotating the entire front axle swept the outer tire forward and backward by $\pm 30.1\text{ mm}$ ($(Track/2) \cdot \sin \delta$). This consumed **36% of the allowable 82.5 mm parking clearance**, increasing body envelope strike risk.
* **Solution**: Scrapped the center-pivot in favor of a dual-knuckle parallelogram tie-bar system, locking knuckle centers and stabilizing the vehicle envelope.

### 4. Feedforward Steering with IMU Closed-Loop
* **Problem**: Original blueprints called for an absolute magnetic rotary encoder (AS5600) on the steering shaft to eliminate servo gear backlash.
* **Insight**: Transitioning to the parallelogram mechanism removed the central steering shaft. Further analysis revealed that closed-loop steering angle is redundant because the high-level navigation loop terminates maneuvers on **measured body yaw rate and heading from the gyro**, not wheel angle. The magnetic encoder was eliminated, saving cost, mass, and I²C bus complexity.

---

## Electrical & PCB Design

To ensure resilience during high-vibration dynamic runs, the electronics follow strict avionics-style guidelines:

- **Custom Dhaka-Milled PCBs**: Fabricated using a single-sided isolation milling process with conservative 0.5 mm trace/space constraints:
  - **Board A (Upper)**: Houses STM32F411, BTS7960 gate interface, IMU, hardware UART, and primary power distribution.
  - **Board B (Lower)**: Dedicated 3.3V sensor aggregation plane containing the PCA9548A multiplexer and modular sensor headers.
- **Four Isolated Power Domains with Single Star Ground**:
  1. *Motor Rail*: Unregulated battery voltage routed via heavy-gauge copper directly to BTS7960 H-Bridge.
  2. *Servo Rail (6.0V)*: Dedicated 3A buck regulator. Prevents the 2.5A instantaneous stall spikes of the MG996R from causing MCU brownout resets.
  3. *Logic Rail (5.0V → 3.3V)*: Dedicated buck feeding STM32 and secondary low-dropout (LDO) regulator for sensors.
  4. *SBC Rail (5.1V)*: Dedicated high-current regulator harness for the Raspberry Pi 4B (physically disconnected during Open round).
- **Wiring & Interconnect Standards**:
  - **Zero jumper wires (DuPont) and zero breadboards** anywhere on the vehicle.
  - All wiring harness connections are crimped and latched using genuine JST-XH connectors with heat-shrink strain reliefs.
  - High-current paths use screw terminals with ferrules.

---

## Simulation & Reproduction

The mechanical and electrical models used to design this vehicle can be reproduced using standard Python scientific tools:

```bash
# Clone the repository
git clone https://github.com/ShammanRahin/WRO_TeamBluePrint.git
cd WRO_TeamBluePrint

# Install simulation dependencies
pip install numpy matplotlib pymunk

# 1. Optical collimator sizing simulation (Floor IR reflection vs slot geometry)
python electrical/collimator.py --plot

# 2. Geometric turning radius and park ratio sweep across wheelbases
python src/sim/geometry_sweep.py --wheelbase 110 --plot

# 3. Swept-body parallel parking envelope simulation
python src/sim/park_feasibility.py --wheelbase 110 --plot

# 4. Regenerate electrical wiring block diagrams
python electrical/make_block_diagram.py
```

---

## Repository Structure

```plaintext
├── BOM.md                       # Bill of Materials: components, part numbers, costs, and sourcing
├── DECISIONS.md                 # Architectural Decision Records (ADRs) with dated rationale
├── SPECSHEET.md                 # Comprehensive hardware specs, pinout tables, and calibrated limits
├── LICENSE                      # Open-source MIT License
├── README.md                    # Main documentation and system overview
│
├── electrical/                  # Schematics, PCB layouts, collimator solver, and wiring diagrams
├── journal/                     # Chronological engineering logs and milestone post-mortems
├── media/                       # Renderings, simulation output plots, and technical figures
├── models/                      # Parametric CAD models and 3D-printable STL/STEP files
├── schemes/                     # System block diagrams and interconnect schematics
├── src/
│   ├── sim/                     # Python kinematics, Monte-Carlo slop, and swept-envelope simulations
│   └── vision/                  # Raspberry Pi OpenCV detection scripts, config files, and calibrations
├── t-photos/                    # Team documentation photos (official and informal)
├── v-photos/                    # Vehicle close-up photographs from multiple perspectives
└── video/                       # Scored demonstration run footage and documentation links
```

---

## Team & Acknowledgments

### Team Blueprint
* **Samman Rahin Shanto** — Electrical System Design, Electronics & PCB Layout *(Islamic University of Technology - IUT)*
* **Syed Sholok** — Embedded Firmware (STM32), Low-Level Control Architecture & Kinematics *(Military Institute of Science and Technology - MIST)*
* **MD. Azmain Shak Rubayed** — CAD & Fabrication Engineer *(Northern University Bangladesh - NUB)*

### Institutional Support & Compliance
* Developed collaboratively across **Islamic University of Technology (IUT)**, **Military Institute of Science and Technology (MIST)**, and **Northern University Bangladesh (NUB)**.
* This repository is maintained in compliance with **WRO Future Engineers General Rule Chapter 7** and will remain publicly accessible.

---

## License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.
