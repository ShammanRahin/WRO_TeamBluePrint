"""
WRO FE 2026 - solid rear axle vs open differential: quantify scrub + odometry.
Key geometric facts derived and checked numerically.
"""
import numpy as np, math, os
OUT = r"F:\claude\WRO_TeamBluePrint\media\steering"

track = 120.0        # mm
wheel_dia = 40.0     # mm
circ = math.pi*wheel_dia
enc_cpr = 180.0      # counts per wheel rev (assumption)
mm_per_count = circ/enc_cpr

# --- scrub: with a solid axle both wheels turn equally, so on a turn each rear
#     wheel slides. Sliding distance per wheel over a turn of angle theta is
#     independent of radius:  scrub = (track/2) * theta   (rad).
def scrub_per_wheel(theta_deg):
    return (track/2.0)*math.radians(theta_deg)

corner90 = scrub_per_wheel(90)
park_arc = scrub_per_wheel(55)     # one parallel-park arc ~55 deg
print("mm/count = %.3f  (enc %g cpr, wheel O%g)" % (mm_per_count, enc_cpr, wheel_dia))
print("\n=== SOLID AXLE scrub (per rear wheel, sliding distance) ===")
print("  per 90 deg corner : %.1f mm   (independent of turn radius)" % corner90)
print("  per lap (4 corners): %.1f mm" % (4*corner90))
print("  3-lap run          : %.1f mm" % (12*corner90))
print("  per parallel park  : %.1f mm  (two ~55 deg arcs)" % (2*park_arc))

# --- odometry bias: motor/encoder sits on the axle (solid) or diff carrier (diff).
#     Both rotate at the MEAN wheel speed = the vehicle-centre speed, so distance
#     odometry is UNBIASED for BOTH on a symmetric turn. Verify by integration.
def centre_vs_encoder(R, theta_deg, n=4000):
    th = np.linspace(0, math.radians(theta_deg), n); dth = th[1]-th[0]
    steps = n-1
    centre  = np.sum(np.full(steps, R*dth))                              # centre path
    encoder = np.sum(np.full(steps, 0.5*((R-track/2)+(R+track/2))*dth))  # axle/carrier
    return centre, encoder
for R in (128,200,300):
    ctr, enc = centre_vs_encoder(R,90)
    print("  R=%3d mm 90deg: centre path=%.1f mm  encoder-implied=%.1f mm  bias=%.2f mm"
          % (R, ctr, enc, enc-ctr))

# --- open diff downsides, quantified ---
print("\n=== OPEN DIFF trade-offs ===")
for bl in (0.5,1.0,2.0):
    print("  gear backlash %.1f deg -> distance jitter +/-%.2f mm per reversal (%.2f counts)"
          % (bl, (bl/360.0)*circ, (bl/360.0)*enc_cpr))
print("  torque bias: on low grip, all torque can go to ONE wheel -> that wheel spins,")
print("  drive+heading authority lost = a RANDOMNESS event (violates the no-randomness spec).")

# --- plot ---
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
fig,ax=plt.subplots(1,2,figsize=(12,4.4))
tks=np.arange(70,145,5)
ax[0].plot(tks,[(t/2)*math.radians(90) for t in tks],"o-")
ax[0].axvline(track,color="grey",ls=":"); ax[0].axhline(corner90,color="grey",ls=":")
ax[0].set_xlabel("track (mm)"); ax[0].set_ylabel("solid-axle scrub / wheel / 90deg corner (mm)")
ax[0].set_title("Scrub scales with track (=track/2 * angle)")
Rs=np.arange(90,401,20)
ax[1].plot(Rs,[corner90]*len(Rs),"s-",label="solid axle scrub (per 90deg)")
ax[1].plot(Rs,[0]*len(Rs),"^-",label="open diff scrub")
ax[1].set_xlabel("turn radius (mm)"); ax[1].set_ylabel("scrub per wheel per corner (mm)")
ax[1].set_title("Scrub is independent of turn radius"); ax[1].legend()
for a in ax: a.grid(alpha=.3)
plt.tight_layout(); plt.savefig(os.path.join(OUT,"6_rear_axle.png"),dpi=110); plt.close()
print("\nplot saved 6_rear_axle.png"); print("DONE")
