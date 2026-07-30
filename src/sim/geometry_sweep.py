#!/usr/bin/env python3
"""
geometry_sweep.py — turn radius and park-feasibility ratio vs wheelbase and steering lock.

WHY THIS EXISTS
---------------
Decision #21 found the symmetric two-arc park fails by 25.6 mm at the as-built 35 deg
lock, with R/L = 0.95 against a requirement of roughly R/L <= 0.7.  Decision #28 briefly
raised the lock to 40 deg to fix that and was withdrawn the same day -- the linkage is
built and measured at 35 deg, and 40 deg still collides by 10.8 mm anyway.

This script exists to make that trade legible, since

    R = wheelbase / tan(lock)

The wheelbase was measured on 2026-07-28 at 110 mm.  105 mm is the TRACK (Decision #15),
not the wheelbase -- the two were confused more than once during design.

This script sweeps both axes so the answer can be read off the moment the calipers come
out, and so the sensitivity is visible: it shows how much the conclusion moves for a
5 mm measurement error.

The wheelbase now defaults to the measured 110 mm.  Pass --wheelbase to override, or
--sweep to see the sensitivity across the feasible range.

USAGE
-----
    python3 src/sim/geometry_sweep.py            # measured 110 mm
    python3 src/sim/geometry_sweep.py --sweep    # sensitivity across the range
    python3 src/sim/geometry_sweep.py --plot

REFERENCES
----------
    DECISIONS.md  #15 (steering, track vs wheelbase), #16 (open-loop), #21 (park), #28 (lock)
    src/sim/park_feasibility.py  — the swept-polygon check this feeds
"""

import argparse
import math
import sys

# --- Vehicle constants, as built -------------------------------------------------
CAR_LENGTH_MM = 165.0      # README section 2, scored footprint 165 x 115
TRACK_MM = 105.0           # centre-to-centre.  NOT the wheelbase.
WHEELBASE_MM = 110.0       # measured 2026-07-28, front axle centre to rear axle centre

# --- Targets ---------------------------------------------------------------------
R_BAND_MM = (120.0, 150.0)  # design target band for turn radius
R_OVER_L_TWO_ARC_MAX = 0.70  # Decision #21: symmetric two-arc needs roughly this or less

# Bay geometry is scale-invariant (Decision #21): bay = 1.5 x car length always,
# so slack = 0.5 x car length always.  Shortening the car cannot help.
PARK_SLACK_MM = 0.5 * CAR_LENGTH_MM


def turn_radius(wheelbase_mm: float, lock_deg: float) -> float:
    """Kinematic turn radius for equal-angle (parallelogram) steering."""
    if lock_deg <= 0.0 or lock_deg >= 90.0:
        raise ValueError("lock must be in (0, 90) degrees")
    return wheelbase_mm / math.tan(math.radians(lock_deg))


def verdict(r_mm: float) -> str:
    lo, hi = R_BAND_MM
    ratio = r_mm / CAR_LENGTH_MM
    in_band = "in band " if lo <= r_mm <= hi else "OUT     "
    two_arc = "two-arc plausible" if ratio <= R_OVER_L_TWO_ARC_MAX else "shuffle required"
    return f"{in_band} R/L={ratio:4.2f}  {two_arc}"


def sweep(wheelbases, locks) -> None:
    print()
    print("  Turn radius R = wheelbase / tan(lock)          car length L = "
          f"{CAR_LENGTH_MM:.0f} mm")
    print(f"  Target band {R_BAND_MM[0]:.0f}-{R_BAND_MM[1]:.0f} mm   "
          f"two-arc park needs R/L <= {R_OVER_L_TWO_ARC_MAX:.2f} "
          f"(Decision #21)")
    print()

    header = "  wheelbase |" + "".join(f"  {d:>5.0f} deg" for d in locks)
    print(header)
    print("  " + "-" * (len(header) - 2))
    for wb in wheelbases:
        row = f"  {wb:7.0f}   |"
        for d in locks:
            row += f"  {turn_radius(wb, d):7.1f} "
        print(row)

    print()
    print("  R/L ratio (same axes):")
    print()
    print(header)
    print("  " + "-" * (len(header) - 2))
    for wb in wheelbases:
        row = f"  {wb:7.0f}   |"
        for d in locks:
            row += f"  {turn_radius(wb, d) / CAR_LENGTH_MM:7.2f} "
        print(row)
    print()


def report_single(wheelbase_mm: float, locks) -> None:
    print()
    print(f"  Wheelbase {wheelbase_mm:.1f} mm   car length {CAR_LENGTH_MM:.0f} mm   "
          f"park slack {PARK_SLACK_MM:.1f} mm")
    print()
    for d in locks:
        r = turn_radius(wheelbase_mm, d)
        print(f"    lock {d:4.0f} deg   R = {r:6.1f} mm    {verdict(r)}")
    print()

    r35 = turn_radius(wheelbase_mm, 35.0)
    r40 = turn_radius(wheelbase_mm, 40.0)
    print(f"  Lock is FINAL at 35 deg -- R = {r35:.1f} mm.  Decision #28 briefly raised it")
    print(f"  to 40 deg (R = {r40:.1f} mm) and was withdrawn: the linkage is built at 35 deg,")
    print( "  and 40 deg still fails the two-arc park by 10.8 mm anyway.")
    print()
    print("  NOTE: this is the kinematic radius only.  Whether the park actually fits")
    print("  is decided by the swept body polygon against both magenta limiters --")
    print("  run park_feasibility.py with the same wheelbase.  Steering is open-loop")
    print("  (Decision #16), so treat R as nominal, never as measured.")
    print()


def plot(wheelbases, locks):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib not installed:  python3 -m pip install matplotlib")

    fig, ax = plt.subplots(figsize=(8, 5))
    for d in locks:
        ax.plot(wheelbases, [turn_radius(w, d) for w in wheelbases],
                marker="o", markersize=3, label=f"{d:.0f} deg lock")

    ax.axhspan(R_BAND_MM[0], R_BAND_MM[1], alpha=0.12, color="green",
               label=f"target {R_BAND_MM[0]:.0f}-{R_BAND_MM[1]:.0f} mm")
    ax.axhline(R_OVER_L_TWO_ARC_MAX * CAR_LENGTH_MM, ls="--", lw=1.2, color="red",
               label=f"two-arc limit R/L={R_OVER_L_TWO_ARC_MAX:.2f}")

    ax.set_xlabel("wheelbase (mm)  -- measured value is 110 mm")
    ax.set_ylabel("turn radius R (mm)")
    ax.set_title("Turn radius vs wheelbase and steering lock\n"
                 f"car length {CAR_LENGTH_MM:.0f} mm; track {TRACK_MM:.0f} mm is NOT the wheelbase")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()

    out = "media/steering/8_geometry_sweep.png"
    fig.savefig(out, dpi=150)
    print(f"  wrote {out}")


def main() -> None:
    p = argparse.ArgumentParser(
        description="Turn radius and park ratio vs wheelbase and steering lock.")
    p.add_argument("--wheelbase", type=float, default=WHEELBASE_MM,
                   help="wheelbase in mm, front axle centre to rear axle centre "
                        "(default: %(default)s, measured 2026-07-28)")
    p.add_argument("--sweep", action="store_true",
                   help="sweep the feasible wheelbase range instead of a single value")
    p.add_argument("--locks", type=float, nargs="+", default=[30.0, 35.0, 40.0, 45.0],
                   help="steering lock angles in degrees (default: 30 35 40 45)")
    p.add_argument("--plot", action="store_true", help="write the sweep plot")
    args = p.parse_args()

    wheelbases = [90.0, 95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0]

    if args.sweep:
        print()
        print(f"  Sensitivity sweep.  Measured wheelbase is {WHEELBASE_MM:.0f} mm.")
        print(f"  {TRACK_MM:.0f} mm is the TRACK (Decision #15).  Do not substitute it here.")
        sweep(wheelbases, args.locks)
    else:
        report_single(args.wheelbase, args.locks)

    if args.plot:
        plot(wheelbases, args.locks)


if __name__ == "__main__":
    main()
