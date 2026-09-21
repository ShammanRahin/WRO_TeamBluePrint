# models

Files used to produce the physical parts — 3D-printed and machined/cut pieces for **Team Blueprint** (WRO Future Engineers 2026).

## Available CAD & 3D Printable Models

| Model File | Part / Subsystem | Format | File Size | Description |
|---|---|---|---|---|
| [`ASMB.stl`](ASMB.stl) | Full Assembled Robot (Interactive 3D Preview) | STL (Binary) | ~10.3 MB | Complete assembled vehicle — **click to view in 3D directly on GitHub** |
| [`ASMB.step`](ASMB.step) | Master CAD Assembly | STEP AP214 | ~61.8 MB | Parametric engineering CAD assembly for Fusion 360 / SolidWorks |
| [`Front Bumper.stl`](Front%20Bumper.stl) | Front Bumper | STL (Binary) | ~822 KB | 3D print-ready mesh for front bumper and sensor mount |
| [`Front Bumper v45.stl`](Front%20Bumper%20v45.stl) | Front Bumper (v45 Revision) | STL (Binary) | ~1.42 MB | Latest revision of front bumper mesh |
| [`Base Plate.stl`](Base%20Plate.stl) | Main Chassis Baseplate | STL (Binary) | ~1.36 MB | 3D print-ready main vehicle chassis bottom plate |
| [`Middle Plate.stl`](Middle%20Plate.stl) | Mid-Deck Chassis Plate | STL (Binary) | ~1.22 MB | 3D print-ready middle tier electronics deck |
| [`LiDAR mount.stl`](LiDAR%20mount.stl) | Slamtec RPLIDAR C1 Mount | STL (Binary) | ~1.48 MB | 3D print-ready mast mount for 360° LiDAR scanner |
| [`Back TOF.stl`](Back%20TOF.stl) | Rear ToF Sensor Mount | STL (Binary) | ~722 KB | 3D print-ready collimated rear ToF sensor bracket |
| [`Tail .stl`](Tail%20.stl) | Rear Wing / Spoiler | STL (Binary) | ~3.03 MB | 3D print-ready rear wing structure ("TEAM BLUEPRINT") |
| [`Left Stearing Hand.stl`](Left%20Stearing%20Hand.stl) | Steering Knuckle (Left Hand) | STL (Binary) | ~3.96 MB | 3D print-ready Ackermann steering knuckle (left) |
| [`Right Stearing Hand.stl`](Right%20Stearing%20Hand.stl) | Steering Knuckle (Right Hand) | STL (Binary) | ~4.03 MB | 3D print-ready Ackermann steering knuckle (right) |
| [`LEGO Gear BOX.stl`](LEGO%20Gear%20BOX.stl) | Drive Gearbox Housing | STL (Binary) | ~2.54 MB | 3D print-ready gearbox enclosure for 5:1 reduction drive |
| [`LEGO Shaft.stl`](LEGO%20Shaft.stl) | Drive Axle Coupler / Shaft | STL (Binary) | ~468 KB | 3D print-ready drive shaft / axle coupler |
| [`Motor Mount.stl`](Motor%20Mount.stl) | 25GA Motor Mount | STL (Binary) | ~416 KB | 3D print-ready motor mounting bracket |
| [`Motor Mount .stl`](Motor%20Mount%20.stl) | 25GA Motor Mount (Reinforced) | STL (Binary) | ~1.88 MB | Reinforced motor bracket variant |
| [`Connector B22.stl`](Connector%20B22.stl) | Chassis Structural Connector | STL (Binary) | ~729 KB | 3D print-ready structural bracket |
| [`Left Front Wheel.stl`](Left%20Front%20Wheel.stl) | Front Wheel Hub (Left) | STL (Binary) | ~7.44 MB | 3D print-ready front left wheel hub with bearing recess |
| [`Right Front Wheel.stl`](Right%20Front%20Wheel.stl) | Front Wheel Hub (Right) | STL (Binary) | ~9.18 MB | 3D print-ready front right wheel hub with bearing recess |
| [`Right Rare Wheel.stl`](Right%20Rare%20Wheel.stl) | Rear Wheel Hub (Right) | STL (Binary) | ~6.31 MB | 3D print-ready driven rear wheel with axle interface |
| [`Left Rare Wheel New.stl`](Left%20Rare%20Wheel%20New.stl) | Rear Wheel Hub (Left) | STL (Binary) | ~6.31 MB | 3D print-ready driven rear wheel with axle interface (left) |
| [`Left BR.stl`](Left%20BR.stl) | Rear Axle Bearing Bracket (Left) | STL (Binary) | ~1.64 MB | 3D print-ready left axle bearing support block |
| [`Center BR.stl`](Center%20BR.stl) | Rear Axle Bearing Bracket (Center) | STL (Binary) | ~1.63 MB | 3D print-ready center axle bearing support block |
| [`Right BR.stl`](Right%20BR.stl) | Rear Axle Bearing Bracket (Right) | STL (Binary) | ~1.62 MB | 3D print-ready right axle bearing support block |
| [`Right Bearing Mount.stl`](Right%20Bearing%20Mount.stl) | Rear Axle Bearing Mount (Right) | STL (Binary) | ~1.07 MB | 3D print-ready axle bearing housing |

## What belongs here

- CAD source (Fusion 360 `.f3d` / STEP) for the chassis plate, steering linkage, wheels, sensor mounts, bumper, and camera mast.
- Print-ready STL exports for every printed part.
- A short note per part: material, layer height, and any orientation that matters (print settings are in [`../docs/build_guide.md`](../docs/build_guide.md#2-3d-printing-specifications)).

| Part | Process | Notes |
|---|---|---|
| Front Bumper | Print (PLA/PETG) | Full STEP assembly model + STLs included |
| Main Chassis & Mid-Deck | Print (PLA/PETG) | Base Plate + Middle Plate included |
| Rear Wing / Spoiler | Print (PLA/PETG) | "TEAM BLUEPRINT" spoiler structure (`Tail .stl`) |
| Sensor & LiDAR Mounts | Print (PETG/PLA+) | LiDAR mount + Back ToF bracket included |
| Steering Knuckles (Ackermann) | Print (PETG/PLA+) | Left & Right knuckle STLs included; 107° / 73° arms, ±35° travel |
| Drive Powertrain | Print (PLA/PETG) | 5:1 spur gear enclosure + Motor Mounts + Shaft included |
| Rear Axle Bearing Brackets | Print (PETG/PLA+) | Left, Center, Right BR & Bearing Mount STLs included |
| Front & Rear Wheels | Print (PLA/PETG) | Complete set of 4 wheels (Left/Right Front & Rear) included |
| ToF collimator snouts + 2° wedges | Print | 2.5 x 10 x 20 mm slot |
| Colour sensor hood | Print | Fixed to chassis |
| Camera / LiDAR mast | Print | ~90 mm, obstacle round |
