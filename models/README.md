# models

Files used to produce the physical parts — 3D-printed and machined/cut pieces for **Team Blueprint** (WRO Future Engineers 2026).

## Available CAD & 3D Printable Models

| Model File | Part / Subsystem | Format | File Size | Description |
|---|---|---|---|---|
| [`Front Bumper.step`](Front%20Bumper.step) | Front Bumper & Sensor Assembly | STEP AP214 | ~61.8 MB | Master CAD assembly for front crash protection, ToF sensor mounting & chassis interface |
| [`Front Bumper.stl`](Front%20Bumper.stl) | Front Bumper | STL (Binary) | ~822 KB | 3D print-ready mesh for front bumper and sensor mount |
| [`Front Bumper v45.stl`](Front%20Bumper%20v45.stl) | Front Bumper (v45 Revision) | STL (Binary) | ~822 KB | Latest revision of front bumper mesh |
| [`Left Stearing Hand.stl`](Left%20Stearing%20Hand.stl) | Steering Knuckle (Left Hand) | STL (Binary) | ~3.96 MB | 3D print-ready Ackermann steering knuckle (left) |
| [`Right Stearing Hand.stl`](Right%20Stearing%20Hand.stl) | Steering Knuckle (Right Hand) | STL (Binary) | ~4.03 MB | 3D print-ready Ackermann steering knuckle (right) |
| [`LEGO Gear BOX.stl`](LEGO%20Gear%20BOX.stl) | Drive Gearbox Housing | STL (Binary) | ~2.54 MB | 3D print-ready gearbox enclosure for 5:1 reduction drive |
| [`Right Front Wheel.stl`](Right%20Front%20Wheel.stl) | Front Wheel Hub (Right) | STL (Binary) | ~4.77 MB | 3D print-ready front wheel hub with bearing pocket |
| [`Right Rare Wheel.stl`](Right%20Rare%20Wheel.stl) | Rear Wheel Hub (Right) | STL (Binary) | ~6.31 MB | 3D print-ready driven rear wheel with axle interface |
| [`Left BR.stl`](Left%20BR.stl) | Rear Axle Bearing Bracket (Left) | STL (Binary) | ~1.64 MB | 3D print-ready left axle bearing support block |
| [`Center BR.stl`](Center%20BR.stl) | Rear Axle Bearing Bracket (Center) | STL (Binary) | ~1.63 MB | 3D print-ready center axle bearing support block |
| [`Right BR.stl`](Right%20BR.stl) | Rear Axle Bearing Bracket (Right) | STL (Binary) | ~1.62 MB | 3D print-ready right axle bearing support block |

## What belongs here

- CAD source (Fusion 360 `.f3d` / STEP) for the chassis plate, steering linkage, wheels, sensor mounts, bumper, and camera mast.
- Print-ready STL exports for every printed part.
- A short note per part: material, layer height, and any orientation that matters (print settings are in [`../docs/build_guide.md`](../docs/build_guide.md#2-3d-printing-specifications)).

| Part | Process | Notes |
|---|---|---|
| Front Bumper | Print (PLA/PETG) | Full STEP assembly model + STL included |
| Steering Knuckles (Ackermann) | Print (PETG/PLA+) | Left & Right knuckle STLs included; 107° / 73° arms, ±35° travel |
| Gearbox Housing | Print (PLA/PETG) | 5:1 spur gear enclosure |
| Rear Axle Bearing Brackets | Print (PETG/PLA+) | Left, Center, and Right bearing brackets included |
| Front & Rear Wheels | Print (PLA/PETG) | 46 mm front, 50 mm rear |
| Chassis plate | Print (PLA) | 130 x 80 mm |
| ToF collimator snouts + 2° wedges | Print | 2.5 x 10 x 20 mm slot |
| Colour sensor hood | Print | Fixed to chassis |
| Camera / LiDAR mast | Print | ~90 mm, obstacle round |
