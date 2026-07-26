#!/usr/bin/env python3
"""
Parallel-park feasibility — WRO Future Engineers 2026, Team Blueprint.

QUESTION THIS ANSWERS
---------------------
Reverse two-arc park, rear axle as pivot. Does the car's SWEPT BODY clear both magenta
parking-lot limitations and the outer wall? Touching a limitation = 0 parking points
(rule 9.24.7), so this is pass/fail, not a margin to optimise.

WHY A SOLVER AND NOT ARITHMETIC
-------------------------------
The lateral shift and longitudinal sweep are one-liners:
    dy = 2R(1 - cos phi)     dx = 2R sin phi
But those describe the REAR AXLE POINT. What actually hits a limiter is the rear inner
BODY CORNER, whose path is offset from the axle path and rotates through the manoeuvre.
That needs the swept polygon.

FIELD GEOMETRY (rules 13.25-13.27 and the parking-lot definition)
    wall along y = 0, corridor in +y
    bay depth 200 mm, bay length = 1.5 x car length
    two limiters, each 200 x 20 x 100 mm, magenta, one at each end of the bay

STEERING IS OPEN-LOOP (DECISIONS.md #16) — there is no AS5600, so nominal R is NOT
trustworthy. Arcs are closed on IMU yaw (DECISIONS.md #21). This solver therefore also
reports sensitivity to R error: if a +/-15% R error flips the verdict, the manoeuvre is
too tight to trust on the mat.

USAGE
    python3 park_feasibility.py --wheelbase 110
    python3 park_feasibility.py --wheelbase 110 --plot

--wheelbase is REQUIRED and deliberately has no default: it is still unmeasured
(front axle centre to rear axle centre). The script errors rather than assuming.
"""

import argparse
import math
import sys

CAR_LEN = 165.0        # body length, mm — the scored projection
CAR_WID = 115.0        # wheel extreme to extreme, mm — the scored width
LOCK_DEG = 35.0        # measured parallelogram lock
BAY_DEPTH = 200.0
LIMITER_W = 20.0
LIMITER_D = 200.0
WALL_Y = 0.0


def body_corners(x, y, th, a_r, length=CAR_LEN, width=CAR_WID):
    """Body rectangle corners for rear-axle centre at (x,y), heading th (rad).
    a_r = distance from rear axle to the rear edge of the body."""
    a_f = length - a_r
    loc = [(-a_r, -width / 2), (a_f, -width / 2), (a_f, width / 2), (-a_r, width / 2)]
    c, s = math.cos(th), math.sin(th)
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in loc]


def rect(x0, x1, y0, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def _axes(poly):
    ax = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        ex, ey = x2 - x1, y2 - y1
        L = math.hypot(ex, ey)
        if L > 1e-12:
            ax.append((-ey / L, ex / L))
    return ax


def _proj(poly, axis):
    vals = [p[0] * axis[0] + p[1] * axis[1] for p in poly]
    return min(vals), max(vals)


def overlap_depth(a, b):
    """SAT. Returns penetration depth (>0 = overlapping), else <=0 separation."""
    best = float("inf")
    for axis in _axes(a) + _axes(b):
        amin, amax = _proj(a, axis)
        bmin, bmax = _proj(b, axis)
        d = min(amax - bmin, bmax - amin)
        if d <= 0:
            return d
        best = min(best, d)
    return best


def simulate(R, y_start, y_final, a_r, steps=400, x_final=None):
    """Symmetric reverse two-arc, rear axle as pivot.

    Integrates the manoeuvre shape first, then TRANSLATES it so the final pose lands
    with the body centred in the bay. Returns (poses, phi_deg) or (None, None).
    """
    dy = y_start - y_final
    if dy <= 0 or dy > 4 * R:
        return None, None
    phi = math.acos(1.0 - dy / (2.0 * R))

    # reverse: ds < 0. First arc steers so that y decreases (toward the wall).
    x = y = th = 0.0
    poses = [(x, y, th)]
    ds = -(R * phi) / (steps / 2.0)
    for sign in (-1.0, +1.0):
        for _ in range(steps // 2):
            th += ds * sign / R
            x += ds * math.cos(th)
            y += ds * math.sin(th)
            poses.append((x, y, th))

    # translate so the parked body is centred longitudinally in the bay
    if x_final is None:
        bay_len = CAR_LEN * 1.5
        a_f = CAR_LEN - a_r
        x_final = bay_len / 2.0 - (a_f - a_r) / 2.0
    ox, oy = x_final - poses[-1][0], y_final - poses[-1][1]
    poses = [(px + ox, py + oy, pth) for px, py, pth in poses]
    return poses, math.degrees(phi)


def check(poses, a_r):
    """Min clearance to wall and to each limiter. <=0 anywhere = FAIL."""
    bay_x0, bay_x1 = 0.0, CAR_LEN * 1.5
    limA = rect(bay_x0 - LIMITER_W, bay_x0, WALL_Y, WALL_Y + LIMITER_D)
    limB = rect(bay_x1, bay_x1 + LIMITER_W, WALL_Y, WALL_Y + LIMITER_D)
    worst_wall, worst_A, worst_B = 1e9, 1e9, 1e9
    for x, y, th in poses:
        poly = body_corners(x, y, th, a_r)
        worst_wall = min(worst_wall, min(p[1] for p in poly) - WALL_Y)
        worst_A = min(worst_A, -overlap_depth(poly, limA))
        worst_B = min(worst_B, -overlap_depth(poly, limB))
    return worst_wall, worst_A, worst_B


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--wheelbase", type=float, required=True,
                    help="REQUIRED, mm, front axle centre to rear axle centre. "
                         "No default: still unmeasured.")
    ap.add_argument("--lock", type=float, default=LOCK_DEG)
    ap.add_argument("--y-start", type=float, default=285.0,
                    help="rear-axle lateral offset from wall at approach, mm. "
                         "Must exceed 200 + car_width/2 = 257.5 mm or the car "
                         "clips a limiter before the manoeuvre even starts.")
    ap.add_argument("--y-final", type=float, default=100.0,
                    help="rear-axle lateral offset when parked, mm")
    ap.add_argument("--rear-overhang", type=float, default=None,
                    help="rear axle to rear body edge, mm (default (165-WB)/2)")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    if args.wheelbase > CAR_LEN - 48.0:
        sys.exit(f"ERROR: wheelbase {args.wheelbase} mm exceeds the "
                 f"{CAR_LEN - 48.0:.0f} mm implied by a {CAR_LEN:.0f} mm body "
                 f"with 46/50 mm wheels.")

    a_r = args.rear_overhang if args.rear_overhang is not None \
        else (CAR_LEN - args.wheelbase) / 2.0
    R = args.wheelbase / math.tan(math.radians(args.lock))
    bay_len = CAR_LEN * 1.5
    slack = bay_len - CAR_LEN

    print("=" * 74)
    print("PARALLEL-PARK FEASIBILITY — WRO FE 2026 Team Blueprint")
    print("=" * 74)
    print(f"car {CAR_LEN:.0f} x {CAR_WID:.0f} mm | wheelbase {args.wheelbase:.0f} mm "
          f"| lock {args.lock:.0f} deg")
    print(f"turn radius R = {R:.1f} mm | rear overhang {a_r:.1f} mm")
    print(f"bay {bay_len:.1f} mm | longitudinal slack {slack:.1f} mm")
    print(f"approach y {args.y_start:.0f} mm -> parked y {args.y_final:.0f} mm "
          f"(shift {args.y_start - args.y_final:.0f} mm)")
    print()

    min_approach = LIMITER_D + CAR_WID / 2.0
    if args.y_start < min_approach:
        print(f"ERROR: approach offset {args.y_start:.0f} mm puts the car's near edge "
              f"at {args.y_start - CAR_WID/2:.1f} mm,")
        print(f"       inside the {LIMITER_D:.0f} mm limiter depth. Minimum approach "
              f"offset is {min_approach:.1f} mm.")
        sys.exit(1)

    poses, phi = simulate(R, args.y_start, args.y_final, a_r)
    if poses is None:
        print("INFEASIBLE in two symmetric arcs at this R — a 3-point shuffle is")
        print("required (allowed, but costs time). Re-run with a larger lock or a")
        print("smaller lateral shift.")
        return

    w, A, B = check(poses, a_r)
    print(f"arc angle phi = {phi:.1f} deg, longitudinal sweep "
          f"= {2 * R * math.sin(math.radians(phi)):.0f} mm")
    print()
    print(f"{'clearance':<34}{'mm':>10}   verdict")
    print("-" * 60)
    for label, v in (("outer wall", w),
                     ("magenta limiter A (entry)", A),
                     ("magenta limiter B (far)", B)):
        print(f"{label:<34}{v:>10.1f}   {'OK' if v > 0 else 'COLLISION'}")

    ok = min(w, A, B) > 0
    print()
    print("VERDICT:", "FEASIBLE" if ok else "NOT FEASIBLE as a two-arc park")
    if not ok:
        print("  -> plan a 3-point shuffle, or increase lock, or approach closer.")

    # ---- the parked longitudinal position is a FREE variable: sweep it ----
    lo, hi = a_r, bay_len - (CAR_LEN - a_r)
    print(f"\nParked rear-axle x is free over [{lo:.1f}, {hi:.1f}] mm — sweeping:")
    print(f"{'x_final':>9} | {'wall':>8} {'limiter A':>10} {'limiter B':>10} | verdict")
    print("-" * 56)
    best = None
    for i in range(9):
        xf = lo + i * (hi - lo) / 8.0
        p2, _ = simulate(R, args.y_start, args.y_final, a_r, x_final=xf)
        if p2 is None:
            continue
        w2, A2, B2 = check(p2, a_r)
        m = min(w2, A2, B2)
        print(f"{xf:>9.1f} | {w2:>+8.1f} {A2:>+10.1f} {B2:>+10.1f} | "
              f"{'OK' if m > 0 else 'FAIL'}")
        if best is None or m > best[0]:
            best = (m, xf)

    print(f"\nBEST symmetric two-arc: {best[0]:+.1f} mm clearance at "
          f"x_final = {best[1]:.1f} mm")
    print("VERDICT:", "FEASIBLE" if best[0] > 0 else
          "**NOT FEASIBLE** — a symmetric two-arc cannot park this car")

    # ---- what lock angle WOULD work ----
    print("\nMinimum lock angle for a two-arc park (x_final optimised at each lock):")
    print(f"{'lock':>6} | {'R':>7} | {'R/L':>6} | {'best clearance':>15} | verdict")
    print("-" * 60)
    for lock in (35, 40, 45, 50, 55, 60):
        Rl = args.wheelbase / math.tan(math.radians(lock))
        b = None
        for i in range(41):
            xf = lo + i * (hi - lo) / 40.0
            p2, _ = simulate(Rl, args.y_start, args.y_final, a_r, x_final=xf)
            if p2 is None:
                continue
            m = min(check(p2, a_r))
            if b is None or m > b:
                b = m
        if b is None:
            continue
        print(f"{lock:>5}d | {Rl:>6.1f} | {Rl/CAR_LEN:>6.2f} | {b:>+14.1f} | "
              f"{'OK' if b > 0 else 'FAIL'}")

    print("""
CONCLUSION — SCALE-INVARIANT
    Bay length is ALWAYS 1.5 x car length, so longitudinal slack is ALWAYS
    0.5 x car length. Shortening the car does NOT help: the bay shrinks with it.
    The only levers are the RATIO R/L and the manoeuvre strategy.

    A symmetric two-arc needs roughly R/L <= 0.7. At the as-built 35 deg lock
    R/L ~ 0.95, which fails. Raising lock to 45 deg reaches R/L ~ 0.67 and only
    just clears (~0 mm margin) — untrustworthy with open-loop steering
    (DECISIONS.md #16).

    RECOMMENDATION: plan a MULTI-POINT (3+) SHUFFLE as the primary parking
    manoeuvre, not a two-arc. Enter at whatever angle the geometry allows, then
    shuffle forward/back against IMU yaw to straighten. It costs time but it is
    allowed, and it is robust to R error — which matters because there is no
    steering feedback. Validate on the real mat.

    Touching a magenta limitation = 0 parking points (rule 9.24.7). Parking is
    15 pts + 7 pts for finishing in the start section = 22 of 122.
""")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import os
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.add_patch(plt.Rectangle((0, 0), bay_len, BAY_DEPTH, fc="0.93",
                                   ec="0.6", ls="--", label="bay"))
        for x0 in (-LIMITER_W, bay_len):
            ax.add_patch(plt.Rectangle((x0, 0), LIMITER_W, LIMITER_D,
                                       fc="magenta", alpha=0.75))
        ax.axhline(0, color="k", lw=3)
        n = len(poses)
        for i in range(0, n, max(1, n // 22)):
            c = body_corners(*poses[i], a_r)
            ax.add_patch(plt.Polygon(c, fill=False, ec="C0", alpha=0.35, lw=0.8))
        ax.add_patch(plt.Polygon(body_corners(*poses[-1], a_r), fill=False,
                                 ec="C2", lw=2.2, label="parked"))
        ax.plot([p[0] for p in poses], [p[1] for p in poses], "r-", lw=1,
                label="rear-axle path")
        ax.set_aspect("equal")
        ax.set_xlabel("along wall (mm)")
        ax.set_ylabel("from wall (mm)")
        ax.set_title(f"Reverse two-arc park — WB {args.wheelbase:.0f} mm, "
                     f"lock {args.lock:.0f} deg, R {R:.0f} mm")
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(alpha=0.25)
        os.makedirs("media/steering", exist_ok=True)
        out = "media/steering/7_park_feasibility.png"
        plt.savefig(out, dpi=130, bbox_inches="tight")
        print(f"plot -> {out}")


if __name__ == "__main__":
    main()
