# Building this car from scratch

Order the parts, print the parts, build the boards, assemble, flash, calibrate.
If you follow this page and [`docs/CALIBRATION.md`](CALIBRATION.md) you should
end up with a car that behaves the same way ours does.

> Items marked **VERIFY** are not yet confirmed against the current build. The
> car is being rebuilt for the Open Championship and this page tracks the new
> build, not the nationals car.

## 0. What you are building

A four-wheeled vehicle, one steered axle at the front, one driven axle at the
rear, a real-time microcontroller doing all the driving and a single-board
computer doing all the seeing.

| Subsystem | Part |
|---|---|
| Real-time controller | WeAct BlackPill, STM32F411CEU6, Cortex-M4 at 100 MHz |
| Perception computer | Raspberry Pi 5, 8 GB |
| Drive motor | 12 V 25GA-370 gearmotor with hall encoder, 1330 rpm **VERIFY: output or motor rpm, and gearbox ratio** |
| Motor driver | BTS7960 module |
| Steering servo | JX PS-1171MG, 17 g digital metal gear |
| IMU | BNO085 over SPI |
| I2C mux | TCA9548A at 0x70 |
| Distance | 2x VL53L1X on mux channels 1 and 2 **VERIFY: which is front, which is rear** |
| Floor colour | TCS34725 on mux channel 3, downward facing with a light hood |
| Camera | fisheye lens on the Pi **VERIFY: `config.json` says 62 degree HFOV, which is not a fisheye figure** |
| Lidar | Slamtec RPLIDAR C1, 360 degree |
| Battery | LiPo **VERIFY: cell count, capacity, C rating** |

## 1. Printed parts

Printed on an **Ender 3 V3 SE** in **PLA**.

| Part | Infill | Notes |
|---|---|---|
| Chassis plate | 5% | ~130 x 105 mm |
| Body and brackets | 5% | strength is not the constraint, mass is |
| Gears | 90% | tooth root stress; do not print these light |
| Front wheels x2 | 5% | 46 mm |
| Rear wheels x2 | 5% | 50 mm |
| Steering knuckles and tie-bar | 5% | parallelogram linkage, +/-35 deg travel |
| ToF collimator snouts | 5% | see [step 5](CALIBRATION.md#5-tof-floor-signal-threshold) — these are structural, not decorative |
| Sensor mounts | 5% | fixed to the chassis, never to the steering |
| Camera mast | 5% | ~90 mm |

CAD source and STLs go in [`models/`](../models/). `*.f3d` and `*.f3z` were
previously excluded by `.gitignore`; that has been removed so the Fusion sources
are tracked.

## 2. Boards

Two single-sided boards, milled in-house by Azmain with a fibre laser and
etching.

- **Carrier**, 90 x 70 mm, bottom copper only, Raspberry Pi 5 stack holes,
  routed with FreeRouting. 0-ohm through-hole resistors used as jumpers wherever
  a crossing could not be avoided.
- **Mux board**, its own single-sided PCB, joined to the carrier by a 10-pin IDC
  box header, carrying the local JST-PH sensor ports.

Wiring standards, non-negotiable, because they are what stops a run dying to
vibration:

- **No DuPont jumpers and no breadboard anywhere on the vehicle**
- Every harness crimped and latched, JST-XH, with heat-shrink strain relief
- High-current paths on screw terminals with ferrules
- Motor B+ and B- go straight from the pack to the BTS7960 and never touch the
  carrier
- One master switch that breaks the logic rails; one separate momentary button
  that does nothing but start a run

Power domains and the star ground are documented in
[`electrical/ELECTRICAL.md`](../electrical/ELECTRICAL.md), section 2. The current
budget is section 3. Read both before you power anything.

## 3. Assembly order

1. **Steering first.** Build the parallelogram linkage on the bench and check it
   moves freely through its full travel before anything else goes on the
   chassis. Knuckle centres must stay fixed as the wheels turn — if the whole
   axle rotates you have built a centre-pivot, which we tried and abandoned
   (it swept the outer tyre +/-30 mm fore and aft and ate 36% of the parking
   clearance).
2. **Powertrain.** Motor, gear reduction, solid rear axle. No differential —
   a solid axle keeps straight-line odometry consistent, which the whole
   lane-gap correction depends on. Check the encoder coupler grub screw is
   properly tight; a slipping coupler is invisible until step 1 of calibration
   gives you a growing standard deviation.
3. **Electronics.** Carrier on, mux board on, sensors last. Bring the power
   rails up one at a time with the boards unpopulated and check each voltage
   before plugging anything in.
4. **Sensors.** ToF with collimators fitted and at final rake. Colour sensor at
   final ride height with its hood. Mast and camera. Lidar last so it is not in
   the way while you work.
5. **Bring-up order** is in `ELECTRICAL.md` section 10. Do not skip it.

## 4. Flash and calibrate

```bash
# STM32 firmware, PlatformIO
pio run -e blackpill_f411ce --target upload
```

Then work through [`docs/CALIBRATION.md`](CALIBRATION.md) in order. This is not
optional. Every constant in the firmware is specific to one physical car, and a
car built from these files with our numbers in it will not drive straight.

## 5. Raspberry Pi

See [`src/pi/README.md`](../src/pi/README.md).

## 6. Cost

**TODO.** Being priced from part links; will be published as an estimate with
local Dhaka sourcing noted separately, since local prices differ substantially
from listed international ones.
