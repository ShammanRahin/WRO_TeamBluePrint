# Day 00 — Project setup (2026-07-12)

## What happened
Initialised the WRO Future Engineers 2026 project structure in the local clone of
`ShammanRahin/WRO_TeamBluePrint`.

- Verified toolchain: git 2.47.1, PowerShell, Desktop Commander all working.
- Found no existing local clone (only an unzipped template + a loose CAD folder).
  Cloned the repo fresh to `F:\claude\WRO_TeamBluePrint`.
- Kept the WRO-standard template dirs (`t-photos, v-photos, video, schemes, src,
  models, other`) that judges expect.
- Added custom working dirs alongside them: `journal/`, `3D-models/`,
  `electrical/`, `media/` (`src/` already present).
- Created source-of-truth docs: `SPECSHEET.md`, `DECISIONS.md`, `BOM.md`.
- Kept the template `README.md` (to be rewritten to ≥5000 chars later per rubric).

## Decisions carried in (see DECISIONS.md)
Nav = gyro heading-hold with IMU-terminated 90° turns; STM32-only open round,
Pi + fisheye for obstacle round over checksummed UART; bell-crank steering with
AS5600 on the shaft, µs commanding; Sharp IR + sonar, ToF rejected, LIDAR deferred;
bearings everywhere, geometry-located parts, no jumpers/breadboard, star ground.

## Open questions blocking Day 1 (see full list below)
1. ⚠️ Hand-rotate encoder count: 180 counts per WHEEL rev or per MOTOR rev? — blocks odometry math.
2. ⚠️ BD national competition date — sets all three commit deadlines.
3. Servo / camera / sensors: owned vs to-buy + Daraz/RoboticsBD lead times — blocks BOM + ordering.
4. ⚠️ Corridor width in national rules (1 m or 0.6 m) — blocks geometry (R, lock, width).
5. Actual wheel diameter — blocks odometry + R.

## Next
Answer the five open questions with measured numbers, then lock Day-1 geometry
(wheelbase, lock, length, width, wheel dia) into SPECSHEET.md.

## Reminder to self
Design phase produces a justified, print-ready v1 — NOT a finished car.
Competition-level is earned by testing on a real mat in Sept–Oct.
Nothing gets pushed unreviewed.
