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

## Update (2026-07-12) — open questions answered
1. **Encoder:** working assumption 180 counts per WHEEL rev; Shanto to recount and correct. (placeholder)
2. **Competition:** October (national). Exact date still needed to pin commit deadlines.
3. **Parts:** MG90S servo (owned), VL53L1X ToF ×5 (owned), Raspberry Pi 4B 1 GB (owned), fisheye lens (owned). Sharp IR bought only if required. → BOM updated.
4. **Corridor width:** 1 m (national). → geometry unblocked.
5. **Wheel diameter:** free — 3D printer on hand, print any wheel. Treated as a design variable (start 45 mm).

**⚠️ Conflict logged:** VL53L1X ToF reverses Decision #7's "ToF rejected". Recorded as a team override with risk + mitigation in DECISIONS.md #7 and SPECSHEET §8. Must validate on the real mat; Sharp IR/sonar kept as documented fallback.

## Next
Lock Day-1 geometry (wheelbase, steering lock, length, width, starting wheel dia)
into SPECSHEET.md now that corridor = 1 m. Confirm encoder count. Get exact
competition date to set the three commit deadlines.

## Reminder to self
Design phase produces a justified, print-ready v1 — NOT a finished car.
Competition-level is earned by testing on a real mat in Sept–Oct.
Nothing gets pushed unreviewed.
