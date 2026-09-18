# models

Files used to produce the physical parts - 3D-printed and machined/cut pieces.
**Not uploaded yet** — the CAD (Autodesk Fusion 360) and STL exports still need to be added;
WRO requires them.

## What belongs here

- CAD source (Fusion 360 `.f3d` / STEP) for the chassis plate, steering linkage, wheels, sensor
  mounts, and the camera mast.
- Print-ready STL exports for every printed part.
- A short note per part: material, layer height, and any orientation that matters
  (print settings are in [`../docs/build_guide.md`](../docs/build_guide.md#2-3d-printing-specifications)).

| Part | Process | Notes |
|---|---|---|
| Chassis plate | Print (PLA) | 130 x 80 mm |
| Front wheels x2 | Print | 46 mm |
| Rear wheels x2 | Print | 50 mm |
| Drive gears (5:1) | Print | 90% infill |
| Steering knuckles + tie-bar (Ackermann) | Print | 107° / 73° arms, +/-35 degrees travel |
| ToF collimator snouts + 2° wedges | Print | 2.5 x 10 x 20 mm slot |
| Colour sensor hood | Print | Fixed to chassis |
| Camera / LiDAR mast | Print | ~90 mm, obstacle round |
