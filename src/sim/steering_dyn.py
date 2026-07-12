"""
WRO FE 2026 - Steering dynamics validation (pymunk 2D rigid-body).
Purpose: check whether the Ackermann verdict from the kinematic study survives
REAL tyre dynamics. Top-down car; each tyre applies a lateral grip impulse capped
by friction (mu * load). When a mechanism demands more side force than the tyre
can give (scrub), the tyre SLIPS - exactly the effect the kinematic model idealised
away. We measure how much the final parked position wanders with surface friction
(mu) and joint slop. Low wander = robust.

TT  = turntable / single central pivot (both front wheels equal angle)
ACK = Ackermann bell-crank (inner/outer angles differ, ~0 scrub)
Units: mm, kg, s.
"""
import numpy as np, os, math
import pymunk
from pymunk import Vec2d
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
rng = np.random.default_rng(3)

L, T = 120.0, 95.0
CARL, CARW, M = 180.0, 100.0, 1.5
G = 9810.0                       # mm/s^2
LOCK = 40.0
OUT = r"F:\claude\WRO_TeamBluePrint\media\steering"

def ackermann_angles(dc):
    d = math.radians(dc)
    if abs(d) < 1e-9: return 0.0, 0.0
    R = L/math.tan(d)
    return math.degrees(math.atan2(L, R-T/2)), math.degrees(math.atan2(L, R+T/2))

# wheel local positions (x fwd, y left): FL, FR, RL, RR
WHEELS = [Vec2d(+L/2,+T/2), Vec2d(+L/2,-T/2), Vec2d(-L/2,+T/2), Vec2d(-L/2,-T/2)]

def steer_angles(mech, dc, slop):
    """Return 4 wheel steer angles (rad). slop = array of 4 per-wheel offsets (deg)."""
    if mech == "ACK":
        aL, aR = ackermann_angles(abs(dc)); s = np.sign(dc)
        base = [s*aL, s*aR, 0.0, 0.0]
    else:  # TT
        base = [dc, dc, 0.0, 0.0]
    return [math.radians(base[i] + slop[i]) for i in range(4)]

def run_once(mech, mu, slop, vt=150.0, dt=1/240):
    """Reverse two-arc park. Returns final (x,y,theta) of rear-axle centre."""
    space = pymunk.Space(); space.gravity = (0,0)
    moment = pymunk.moment_for_box(M, (CARL, CARW))
    car = pymunk.Body(M, moment); space.add(car)
    car.position = (0,0); car.angle = 0.0
    mshare = M/4.0
    jmax = mu*mshare*G*dt          # max lateral grip impulse per wheel per step
    jdrive_max = 0.5*mshare*G*dt   # drive/brake authority (rear)
    # phase durations sized for ~a full park (same command for both mechs)
    t_ph = 0.95
    plan = [(-LOCK, int(t_ph/dt)), (+LOCK, int(t_ph/dt))]
    for dc, nsteps in plan:
        ang = steer_angles(mech, dc, slop)
        for _ in range(nsteps):
            for i, wp in enumerate(WHEELS):
                world_p = car.local_to_world(wp)
                v = car.velocity_at_local_point(wp)
                th = car.angle + ang[i]
                f = Vec2d(math.cos(th), math.sin(th)); n = Vec2d(-math.sin(th), math.cos(th))
                v_lat = v.dot(n); v_fwd = v.dot(f)
                j_lat = float(np.clip(-mshare*v_lat, -jmax, jmax))
                imp = n*j_lat
                if i >= 2:  # rear wheels drive (reverse => vt applied as negative fwd)
                    j_dr = float(np.clip((-vt - v_fwd)*mshare, -jdrive_max, jdrive_max))
                    imp = imp + f*j_dr
                else:       # front rolling resistance (tiny)
                    j_rr = float(np.clip(-0.02*mshare*v_fwd, -jmax, jmax))
                    imp = imp + f*j_rr
                car.apply_impulse_at_world_point(imp, world_p)
            space.step(dt)
    rear = car.local_to_world(Vec2d(-L/2, 0.0))
    return rear.x, rear.y, car.angle

def slop_sample(mech, sig_ds):
    if mech == "TT":
        c = rng.normal(0, 0.15)          # one bearinged pivot, correlated
        return np.array([c, c, 0, 0])
    return np.array([rng.normal(0,sig_ds), rng.normal(0,sig_ds), 0, 0])  # per wheel

def study(mech, mus, sig_ds, n=25):
    """For each mu, run n parks; return mean curb-side lateral and its std (mm)."""
    means, stds = [], []
    for mu in mus:
        finals = []
        for _ in range(n):
            x,y,th = run_once(mech, mu, slop_sample(mech, sig_ds))
            finals.append(y - (T/2)*math.cos(th))   # curb-side wheel lateral
        finals = np.array(finals)
        means.append(finals.mean()); stds.append(finals.std())
    return np.array(means), np.array(stds)

# ---------------- run ----------------
mus = np.array([0.6,0.7,0.8,0.9,1.0,1.1,1.2])   # tyre-on-mat friction spread
SIG_DS = 0.5                                     # realistic built downstream slop (deg)

print("=== dynamic park: curb-side lateral vs surface friction mu ===")
res = {}
for mech in ["TT","ACK"]:
    m, s = study(mech, mus, SIG_DS)
    res[mech] = (m, s)
    print("  %s: lateral range over mu = %.1f mm (span), mean run-to-run std = %.2f mm"
          % (mech, m.max()-m.min(), s.mean()))

# sensitivity to surface = how much the landing shifts across the mu range
tt_span = res["TT"][0].max()-res["TT"][0].min()
ack_span = res["ACK"][0].max()-res["ACK"][0].min()
print("\nSurface-sensitivity (lower=better):  TT span=%.1f mm   ACK span=%.1f mm"%(tt_span,ack_span))
print("Run-to-run scatter (slop):           TT std=%.2f mm    ACK std=%.2f mm"
      %(res["TT"][1].mean(), res["ACK"][1].mean()))

# ---------------- plot ----------------
plt.figure(figsize=(6.5,4))
for mech,c in [("TT","C0"),("ACK","C1")]:
    m,s = res[mech]
    m0 = m - m.mean()      # centre each to compare shape/sensitivity
    plt.plot(mus, m0, "o-", color=c, label="%s (curb lateral, centred)"%mech)
    plt.fill_between(mus, m0-s, m0+s, color=c, alpha=0.18)
plt.xlabel("tyre-on-mat friction coefficient mu")
plt.ylabel("curb-side lateral vs its own mean (mm)")
plt.title("Dynamic park sensitivity to surface friction (band = run-to-run slop)")
plt.legend(); plt.grid(alpha=.3); plt.tight_layout()
plt.savefig(os.path.join(OUT,"4_dynamics.png"), dpi=110); plt.close()
print("\nplot saved:", os.path.join(OUT,"4_dynamics.png")); print("DONE")
