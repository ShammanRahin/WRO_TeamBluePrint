#!/usr/bin/env python3
"""
VL53L0X collimator snout solver — WRO Future Engineers 2026, Team Blueprint.

WHY THIS EXISTS
---------------
The VL53L0X has a FIXED ~25 deg field of view and NO programmable ROI. (The VL53L1X
does; the L0X does not. See DECISIONS.md #7 correction.) So the beam cannot be narrowed
in firmware — it has to be narrowed mechanically.

That matters because of the field colours:
  * mat is WHITE                      (rule 13.2)
  * exterior + interior walls are BLACK on every visible face (rules 13.4, 13.6)

So the FALSE target (floor) reflects near-infrared BETTER than the TRUE target (wall).
A naive closest-return read locks onto the floor. The sensor is only trustworthy out to
the range at which the bottom edge of its cone first strikes the mat:

    d_floor = h / tan(theta_eff)
    theta_eff = theta_half + rake - wedge

  h          sensor optical-centre height above the mat
  theta_half vertical half-angle after the collimator = atan(slot_h/2 / slot_depth)
  rake       chassis nose-down angle, INHERITED by every chassis-mounted sensor
             = atan((rear_wheel_dia - front_wheel_dia) / 2 / wheelbase)
  wedge      printed upward tilt in the sensor mount (the correction)

CONSTRAINT: the beam axis must stay BELOW the 100 mm wall top (rules 13.3, 13.5) at
working range, or the sensor shoots over the wall and sees nothing.

Slot must be WIDE, not round: the L0X emitter and receiver apertures sit ~2.5 mm apart,
so a round hole vignettes the receiver. Narrow the VERTICAL axis only.

Re-run offset calibration after fitting a snout — the hood changes crosstalk.

USAGE
    python3 collimator.py
    python3 collimator.py --h 60 --wheelbase 110 --wedge 2.0
"""

import argparse
import math

WALL_TOP_MM = 100.0        # rules 13.3, 13.5
CORRIDOR_MM = 1000.0       # national corridor, confirmed 2026-07-12
CAR_WIDTH_MM = 115.0       # wheel extreme to wheel extreme (the scored width)
BARE_FOV_DEG = 25.0        # VL53L0X datasheet, fixed


def side_wall_distance(corridor=CORRIDOR_MM, car_width=CAR_WIDTH_MM):
    """Lateral distance from a centred car to each side wall."""
    return (corridor - car_width) / 2.0


def rake_deg(front_dia, rear_dia, wheelbase):
    """Chassis nose-down angle from front/rear wheel diameter mismatch."""
    if wheelbase <= 0:
        raise ValueError("wheelbase must be > 0")
    return math.degrees(math.atan(((rear_dia - front_dia) / 2.0) / wheelbase))


def theta_half_deg(slot_h, slot_depth):
    """Vertical half-angle after collimation. None => bare sensor."""
    if slot_h is None or slot_depth is None:
        return BARE_FOV_DEG / 2.0
    return math.degrees(math.atan((slot_h / 2.0) / slot_depth))


def d_floor(h, theta_eff_deg):
    """Range at which the cone's lower edge first strikes the mat."""
    if theta_eff_deg <= 0:
        return float("inf")
    return h / math.tan(math.radians(theta_eff_deg))


def axis_height(h, net_tilt_up_deg, distance):
    """Beam-axis height above mat at a given range. Must stay under WALL_TOP_MM."""
    return h + distance * math.tan(math.radians(net_tilt_up_deg))


def evaluate(h, slot_h, slot_depth, wedge, rake, target_range):
    th = theta_half_deg(slot_h, slot_depth)
    theta_eff = th + rake - wedge
    df = d_floor(h, theta_eff)
    net_up = wedge - rake
    ax = axis_height(h, net_up, target_range)
    return {
        "theta_half": th,
        "theta_eff": theta_eff,
        "d_floor": df,
        "margin": df - target_range,
        "axis_height": ax,
        "over_wall": ax >= WALL_TOP_MM,
        "ok": df > target_range and ax < WALL_TOP_MM,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--h", type=float, default=40.0,
                    help="sensor optical-centre height above mat, mm (default 40 "
                         "= as-built 50 mm car)")
    ap.add_argument("--slot-h", type=float, default=2.5, help="slot height, mm")
    ap.add_argument("--slot-w", type=float, default=10.0,
                    help="slot width, mm (wide, not round — see docstring)")
    ap.add_argument("--slot-depth", type=float, default=20.0, help="snout depth, mm")
    ap.add_argument("--wedge", type=float, default=2.0, help="mount wedge, deg up")
    ap.add_argument("--front-dia", type=float, default=46.0)
    ap.add_argument("--rear-dia", type=float, default=50.0)
    ap.add_argument("--wheelbase", type=float, default=None,
                    help="mm. FLAG: still unmeasured; default sweeps 100-117.")
    ap.add_argument("--plot", action="store_true", help="write plot to media/electrical/")
    args = ap.parse_args()

    target = side_wall_distance()

    print("=" * 74)
    print("VL53L0X COLLIMATOR SOLVER — WRO FE 2026 Team Blueprint")
    print("=" * 74)
    print(f"Corridor {CORRIDOR_MM:.0f} mm, car width {CAR_WIDTH_MM:.0f} mm "
          f"(wheel extremes)")
    print(f"--> side wall at {target:.1f} mm : this is what d_floor must exceed")
    print(f"Slot {args.slot_h} x {args.slot_w} x {args.slot_depth} mm "
          f"(h x w x depth), wedge +{args.wedge} deg")
    print()

    # --- rake -------------------------------------------------------------
    if args.wheelbase is None:
        wbs = [100.0, 110.0, 117.0]
        print("WHEELBASE NOT SUPPLIED — sweeping the feasible range "
              "(<= 165 - 23 - 25 = 117 mm)")
    else:
        wbs = [args.wheelbase]
    print(f"{'wheelbase':>10} | {'rake (deg)':>10}")
    print("-" * 24)
    rakes = []
    for wb in wbs:
        r = rake_deg(args.front_dia, args.rear_dia, wb)
        rakes.append(r)
        print(f"{wb:>10.0f} | {r:>10.2f}")
    rake = sum(rakes) / len(rakes)
    print(f"\nUsing rake = {rake:.2f} deg "
          f"(front {args.front_dia} mm vs rear {args.rear_dia} mm)\n")

    # --- configuration comparison ----------------------------------------
    configs = [
        ("bare VL53L0X (no snout, no wedge)", None, None, 0.0),
        ("snout only, no wedge", args.slot_h, args.slot_depth, 0.0),
        ("snout + wedge cancelling rake", args.slot_h, args.slot_depth, rake),
        (f"snout + {args.wedge} deg wedge  <-- SPEC", args.slot_h,
         args.slot_depth, args.wedge),
    ]
    print(f"{'configuration':<38} {'th_eff':>7} {'d_floor':>9} {'margin':>9} {'axis':>7}")
    print("-" * 74)
    for name, sh, sd, wg in configs:
        r = evaluate(args.h, sh, sd, wg, rake, target)
        flag = "OK " if r["ok"] else "FAIL"
        note = " OVER WALL" if r["over_wall"] else ""
        print(f"{name:<38} {r['theta_eff']:>6.2f}d {r['d_floor']:>8.0f}mm "
              f"{r['margin']:>+8.0f}mm {r['axis_height']:>6.0f}mm  {flag}{note}")

    # --- height growth ----------------------------------------------------
    print("\nMargin improves as the car grows (Pi 4B + BTS7960 force ~75-90 mm):")
    print(f"{'car height':>11} | {'h (optical)':>11} | {'d_floor':>9} | {'margin':>9}")
    print("-" * 50)
    for car_h in (50, 60, 75, 90):
        h_opt = car_h - 10.0
        r = evaluate(h_opt, args.slot_h, args.slot_depth, args.wedge, rake, target)
        print(f"{car_h:>9} mm | {h_opt:>9.0f} mm | {r['d_floor']:>7.0f} mm | "
              f"{r['margin']:>+7.0f} mm")

    # --- 30 deg pair ------------------------------------------------------
    print("\nFL30 / FR30 pair — slant range = gap / sin(30 deg) = 2 x gap "
          "(DECISIONS.md #19):")
    r = evaluate(args.h, args.slot_h, args.slot_depth, args.wedge, rake, target)
    print(f"{'gap':>8} | {'slant':>8} | vs d_floor {r['d_floor']:.0f} mm")
    print("-" * 40)
    for gap in (150, 250, 350, 442):
        slant = gap / math.sin(math.radians(30.0))
        print(f"{gap:>6} mm | {slant:>6.0f} mm | "
              f"{'OK' if slant < r['d_floor'] else 'FAIL'}")

    print("\nFLAG — bench-test before printing five mounts:")
    print("  * signal rate vs BLACK MDF at 442 mm, snout fitted (90 deg pair)")
    print("  * signal rate vs BLACK MDF at 300/700 mm at 60 deg incidence (30 deg pair)")
    print("  A 2.5 x 10 mm aperture throws away photons and the wall is already a poor")
    print("  NIR target. If marginal, order the Sharp fallback IMMEDIATELY (BOM.md).")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import os
        depths = [d / 2.0 for d in range(10, 81)]
        for wg, lbl in ((0.0, "no wedge"), (rake, "cancel rake"),
                        (args.wedge, f"+{args.wedge} deg wedge (SPEC)")):
            ys = [evaluate(args.h, args.slot_h, d, wg, rake, target)["d_floor"]
                  for d in depths]
            plt.plot(depths, ys, label=lbl)
        plt.axhline(target, color="r", ls="--",
                    label=f"side wall {target:.0f} mm (must clear)")
        plt.axvline(args.slot_depth, color="k", ls=":", lw=0.8)
        plt.xlabel("snout depth (mm)")
        plt.ylabel("floor first-return distance (mm)")
        plt.title(f"VL53L0X collimator — slot {args.slot_h} mm tall, "
                  f"h = {args.h:.0f} mm")
        plt.ylim(0, 2000)
        plt.legend()
        plt.grid(alpha=0.3)
        os.makedirs("media/electrical", exist_ok=True)
        out = "media/electrical/collimator_sweep.png"
        plt.savefig(out, dpi=130, bbox_inches="tight")
        print(f"\nplot -> {out}")


if __name__ == "__main__":
    main()
