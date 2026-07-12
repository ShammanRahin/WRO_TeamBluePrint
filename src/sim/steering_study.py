"""
WRO FE 2026 - Steering mechanism robustness study
Compares three single-actuator steering architectures for a repeatable full
parallel park, under realistic joint slop, with AS5600 closed-loop calibration.

  TT  = Turntable / single central pivot (rigid axle, both wheels equal, 1 pivot)
  PAR = Parallel bell-crank (2 kingpin knuckles, equal angles, ~5 joints)
  ACK = Ackermann bell-crank (2 kingpin knuckles, Ackermann arms, ~5 joints)

Fair test: each mechanism is CALIBRATED (outer gyro+odometry+AS5600 loops null the
nominal bias), then stressed with Monte-Carlo joint slop. Metric = final lateral
parking error (curb-side wheels must end within 20 mm of the wall = "parallel").
Units: mm; deg at the API, rad internally.
"""
import numpy as np, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
rng = np.random.default_rng(7)

L, T = 120.0, 95.0            # wheelbase, front track (mm)
CARL = 180.0
BAY_DEPTH, BAY_LEN = 200.0, 1.5*180.0
LOCK, D_LAT = 40.0, 120.0     # full-lock centre angle (deg); lateral move into bay (mm)
SIG_TT, SIG_SENS = 0.15, 0.088   # deg: central-pivot backlash; AS5600 quantisation
OUT = r"F:\claude\WRO_TeamBluePrint\media\steering"

# ---------- geometry ----------
def ackermann_angles(dc):
    d = np.radians(dc)
    if abs(d) < 1e-9: return 0.0, 0.0
    R = L/np.tan(d)
    return np.degrees(np.arctan2(L, R-T/2)), np.degrees(np.arctan2(L, R+T/2))

def equal_angles(dc): return dc, dc
def wheel_angles(m, dc): return ackermann_angles(dc) if m=="ACK" else equal_angles(dc)

def realized_R(aL_deg, aR_deg):
    """Rear-axle-centre radius (mm) minimising front-wheel slip; + RMS scrub (deg)."""
    aL, aR = np.radians(aL_deg), np.radians(aR_deg)
    if abs(aL)<1e-6 and abs(aR)<1e-6: return 1e9, 0.0
    lo, hi = 20.0, 20000.0; gr=(np.sqrt(5)-1)/2
    f = lambda yc: (aL-np.arctan2(L,yc-T/2))**2 + (aR-np.arctan2(L,yc+T/2))**2
    c, d = hi-gr*(hi-lo), lo+gr*(hi-lo)
    for _ in range(70):
        if f(c) < f(d): hi=d
        else: lo=c
        c, d = hi-gr*(hi-lo), lo+gr*(hi-lo)
    yc=(lo+hi)/2
    return yc, np.degrees(np.sqrt(f(yc)/2))

# ---------- nominal radius + slop sensitivity (gradient) per mechanism ----------
def R_and_grad(m):
    aL, aR = wheel_angles(m, LOCK)
    R0, _ = realized_R(aL, aR)
    h = 0.01
    gL = (realized_R(aL+h, aR)[0] - realized_R(aL-h, aR)[0]) / (2*h)   # mm per deg
    gR = (realized_R(aL, aR+h)[0] - realized_R(aL, aR-h)[0]) / (2*h)
    return R0, gL, gR

# ---------- vectorized two-arc park (closed form) ----------
def two_arc_pose(R1, R2, s):
    """Final (x,y,theta) of a reverse two-arc park, arrays over samples.
    R1<0 (phase1 right/curb turn), R2>0 (phase2). ds=-s each phase."""
    R1=np.asarray(R1,float); R2=np.asarray(R2,float)
    dth1=-s/R1; x1=R1*np.sin(dth1); y1=R1*(1-np.cos(dth1)); th1=dth1
    dth2=-s/R2; f2=R2*np.sin(dth2); l2=R2*(1-np.cos(dth2))
    x2=x1+f2*np.cos(th1)-l2*np.sin(th1)
    y2=y1+f2*np.sin(th1)+l2*np.cos(th1)
    return x2, y2, th1+dth2

def curbside_y(y, th):   # curb-side (right) wheel line, accounts for heading
    return y - (T/2)*np.cos(th)

def calibrate(m):
    """Per-arc length s so the nominal park lands at D_LAT lateral, heading 0."""
    R0,_,_ = R_and_grad(m)
    phi = np.arccos(min(1.0, max(-1.0, 1 - D_LAT/(2*R0))))
    return R0*phi, R0

def montecarlo(m, sig_ds, n=200000):
    R0, gL, gR = R_and_grad(m)
    s, _ = calibrate(m)
    # reference (nominal) parked position
    xr,yr,thr = two_arc_pose(-R0, R0, s); ref = curbside_y(yr, thr)
    if m=="TT":
        sig=np.hypot(SIG_TT,SIG_SENS)
        c1=rng.normal(0,sig,n); c2=rng.normal(0,sig,n)   # correlated across L/R, per phase
        dR1=(gL+gR)*c1; dR2=(gL+gR)*c2
    else:
        sig=np.hypot(sig_ds,SIG_SENS)
        dR1=gL*rng.normal(0,sig,n)+gR*rng.normal(0,sig,n)
        dR2=gL*rng.normal(0,sig,n)+gR*rng.normal(0,sig,n)
    R1=-(R0+dR1); R2=(R0+dR2)
    x,y,th = two_arc_pose(R1,R2,s)
    err = curbside_y(y,th) - ref
    return err

def nominal_traj(m, steps=80):
    R0,_,_ = R_and_grad(m); s,_ = calibrate(m)
    pose=np.array([0.,0.,0.]); pts=[pose.copy()]
    for R in (-R0, R0):
        for _ in range(steps):
            x,y,th=pose; dth=(-s/steps)/R
            fwd=R*np.sin(dth); lat=R*(1-np.cos(dth))
            pose=np.array([x+fwd*np.cos(th)-lat*np.sin(th),
                           y+fwd*np.sin(th)+lat*np.cos(th), th+dth]); pts.append(pose.copy())
    return np.array(pts)

# ================= RUN =================
print("=== realized turning radius at %g deg lock (rear-axle centre) ===" % LOCK)
for m in ["TT","PAR","ACK"]:
    aL,aR=wheel_angles(m,LOCK); yc,sl=realized_R(aL,aR)
    print("  %-4s wheels L/R=%5.1f/%5.1f deg -> R=%6.1f mm  scrub=%.2f deg"%(m,aL,aR,yc,sl))

print("\n=== scrub (RMS front slip, deg) vs commanded centre angle ===")
angles=np.arange(5,46,5); scrub={"TT":[],"ACK":[]}
for a in angles:
    for m in ["TT","ACK"]:
        aL,aR=wheel_angles(m,a); scrub[m].append(realized_R(aL,aR)[1])
    print("  %2d deg: TT=%.2f  ACK=%.2f"%(a,scrub["TT"][-1],scrub["ACK"][-1]))

print("\n=== robustness: 95th-pct |lateral park error| (mm) ===")
sig=np.array([0.1,0.2,0.3,0.4,0.5,0.7,1.0,1.3,1.6,2.0])
tt95=np.percentile(np.abs(montecarlo("TT",0.0)),95)
print("  TT single pivot: %.2f mm (slop-independent)"%tt95)
ack95=[]; par95=[]
for sg in sig:
    ack95.append(np.percentile(np.abs(montecarlo("ACK",sg)),95))
    par95.append(np.percentile(np.abs(montecarlo("PAR",sg)),95))
    print("  sig=%.2f deg: ACK=%.2f  PAR=%.2f"%(sg,ack95[-1],par95[-1]))
ack95=np.array(ack95); par95=np.array(par95)
cross=None
for i in range(1,len(sig)):
    if (ack95[i]-tt95)*(ack95[i-1]-tt95)<=0:
        x0,x1=sig[i-1],sig[i]; y0,y1=ack95[i-1]-tt95,ack95[i]-tt95
        cross=x0-y0*(x1-x0)/(y1-y0); break
print("\nACK matches TT robustness at downstream slop ~ %s deg"%("%.2f"%cross if cross else ">2.0"))

# ---------- plots ----------
plt.figure(figsize=(6,4))
plt.plot(angles,scrub["TT"],"o-",label="Turntable / Parallel (equal angles)")
plt.plot(angles,scrub["ACK"],"s-",label="Ackermann bell-crank")
plt.xlabel("commanded centre steer (deg)"); plt.ylabel("RMS front-wheel scrub (deg)")
plt.title("Tyre scrub vs steering angle"); plt.legend(); plt.grid(alpha=.3)
plt.tight_layout(); plt.savefig(os.path.join(OUT,"1_scrub.png"),dpi=110); plt.close()

plt.figure(figsize=(7,3.6))
for m,ls in [("TT","-"),("ACK","--")]:
    tr=nominal_traj(m); plt.plot(tr[:,0],tr[:,1],ls,label="%s path"%m)
plt.axhline(-D_LAT,color="k",lw=.8,alpha=.4,label="curb line")
plt.xlabel("x (mm)"); plt.ylabel("y (mm)"); plt.title("Nominal reverse two-arc park (calibrated)")
plt.legend(); plt.grid(alpha=.3); plt.axis("equal"); plt.tight_layout()
plt.savefig(os.path.join(OUT,"2_park.png"),dpi=110); plt.close()

plt.figure(figsize=(6.5,4))
plt.axhline(tt95,color="C0",label="Turntable / single pivot (%.2f mm)"%tt95)
plt.plot(sig,ack95,"s-",color="C1",label="Ackermann bell-crank")
plt.plot(sig,par95,"^-",color="C2",label="Parallel bell-crank")
plt.axhline(20,color="red",lw=.8,ls="--"); plt.text(0.1,20.6,"20 mm 'parallel' limit",color="red")
if cross: plt.axvline(cross,color="grey",ls=":"); plt.text(cross+.02,plt.ylim()[1]*0.6,"crossover %.2f deg"%cross)
plt.xlabel("unobserved downstream joint slop, 1sigma (deg)")
plt.ylabel("95th-pct |lateral park error| (mm)")
plt.title("Parking robustness vs joint slop"); plt.legend(); plt.grid(alpha=.3)
plt.tight_layout(); plt.savefig(os.path.join(OUT,"3_robustness.png"),dpi=110); plt.close()
print("\nplots saved to",OUT); print("DONE")
