import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle, Polygon

OUT = "repo/media/diagrams/calibration/"
SURF="#FAFAF7"; CARD="#FFFFFF"; GRID="#E6E6DF"
INK="#2B2B28"; INK2="#6B6B63"; INK3="#96968C"
C1="#3B7DD8"; C2="#C2711B"; C3="#6B4FA8"; C4="#0E8C7A"
GOOD="#1F7A4D"; BAD="#B4452F"

plt.rcParams.update({
 "figure.facecolor":SURF,"axes.facecolor":CARD,"savefig.facecolor":SURF,
 "font.family":"DejaVu Sans","font.size":9,
 "axes.edgecolor":GRID,"axes.linewidth":0.9,"axes.labelcolor":INK2,"axes.labelsize":9,
 "axes.titlesize":10,"axes.titleweight":"bold","axes.titlecolor":INK,
 "xtick.color":INK3,"ytick.color":INK3,"xtick.labelsize":8.5,"ytick.labelsize":8.5,
 "grid.color":GRID,"grid.linewidth":0.7,"legend.frameon":False,"legend.fontsize":8.5,
 "lines.linewidth":2.0,"lines.markersize":5,
})
def strip(ax, grid="y"):
    for s in ("top","right"): ax.spines[s].set_visible(False)
    for s in ("left","bottom"): ax.spines[s].set_color(GRID)
    ax.grid(axis=grid, alpha=0.9); ax.set_axisbelow(True)

def head(fig, title, sub, note=None):
    """Title band at the top, note band at the bottom. Margins are reserved by
    canvas(), so nothing here can land on an axis label."""
    fig.text(0.045,0.965,title,fontsize=13.5,fontweight="bold",color=INK,va="top")
    fig.text(0.045,0.905,sub,fontsize=9.5,color=INK2,va="top")
    if note: fig.text(0.045,0.030,note,fontsize=8.3,color=INK3,va="bottom")

def punch(fig,text):
    """The one-line takeaway, in the bottom band above the grey note."""
    fig.text(0.5,0.090,text,ha="center",fontsize=9,color=INK,fontweight="bold")

def canvas(w,h,ncols=1,note=True,**gk):
    """Figure with space reserved above for the title block and below for the note."""
    fig,axes = plt.subplots(1,ncols,figsize=(w,h),**gk)
    fig.subplots_adjust(left=0.085, right=0.975,
                        top=1-(1.05/h), bottom=(1.45 if note else 0.72)/h,
                        wspace=0.30)
    return fig, axes

def save(fig,name):
    fig.savefig(OUT+name,format="svg")   # no tight bbox: the bands are deliberate
    plt.close(fig); print("  ",name)

ILL = "Illustrative shapes, not our measurements — this is how to read your own run. Replace with real data once collected."
rng = np.random.default_rng(7)

# ---------------------------------------------------------------- 01
fig,axes = canvas(10.4,5.0,2)
head(fig,"Step 1 · Encoder ticks per wheel revolution",
     "100 hand-turned revolutions. The mean is the answer; the spread tells you whether to believe it.",ILL)
g = rng.normal(1428,4.2,100)
ax=axes[0]; strip(ax)
ax.hist(g,bins=18,color=C1,edgecolor=CARD,linewidth=1.2)
ax.axvline(g.mean(),color=INK,lw=1.6)
ax.text(0.03,0.95,f"mean {g.mean():.1f}\nsd {g.std(ddof=1):.1f}  (0.3 % of mean)",
        transform=ax.transAxes,va="top",fontsize=9,color=INK,fontweight="bold")
ax.set_title("GOOD · tight, single peak",color=GOOD)
ax.set_xlabel("ticks per revolution"); ax.set_ylabel("samples")
b = np.concatenate([rng.normal(1428,4.2,55), rng.normal(1398,9,45)])
ax=axes[1]; strip(ax)
ax.hist(b,bins=18,color=BAD,edgecolor=CARD,linewidth=1.2)
ax.axvline(b.mean(),color=INK,lw=1.6,ls="--")
ax.text(0.03,0.95,"two peaks = the coupler is\nslipping under load.\nThe mean is meaningless here.",
        transform=ax.transAxes,va="top",fontsize=9,color=BAD,fontweight="bold")
ax.set_title("BAD · two peaks, sd keeps growing",color=BAD)
ax.set_xlabel("ticks per revolution")
save(fig,"01-encoder-ticks.svg")

# ---------------------------------------------------------------- 02
fig,axes = canvas(10.4,5.0,2,gridspec_kw={"width_ratios":[1.45,1]})
head(fig,"Step 2 · Ticks per centimetre",
     "Fit a line through several run lengths. The SLOPE is TICKS_PER_CM — the intercept absorbs the start/stop error.",ILL)
cm=np.array([25,50,75,100,150,200]*3,dtype=float)
cm=cm+rng.normal(0,0.6,cm.size)
ticks=31.933*cm+9+rng.normal(0,7,cm.size)
m,b0=np.polyfit(cm,ticks,1)
xs=np.linspace(0,210,10)
ax=axes[0]; strip(ax,"both")
ax.plot(xs,m*xs+b0,color=INK,lw=1.6,zorder=2,label=f"fit: {m:.3f}·cm + {b0:.1f}")
ax.scatter(cm,ticks,s=34,color=C1,edgecolor=CARD,linewidth=1.1,zorder=3,label="18 measured runs")
ax.set_xlabel("measured distance (cm)"); ax.set_ylabel("encoder ticks")
ax.legend(loc="upper left")
r=np.corrcoef(cm,ticks)[0,1]**2
ax.text(0.97,0.06,f"TICKS_PER_CM = {m:.3f}\nR² = {r:.5f}",transform=ax.transAxes,
        ha="right",fontsize=9.5,fontweight="bold",color=INK)
ax=axes[1]; strip(ax)
res=ticks-(m*cm+b0)
ax.axhline(0,color=INK3,lw=1)
ax.scatter(cm,res,s=30,color=C3,edgecolor=CARD,linewidth=1.1)
ax.set_title("residuals",color=INK2,fontsize=9.5)
ax.set_xlabel("distance (cm)"); ax.set_ylabel("ticks off the line")
ax.text(0.5,1.09,"no pattern = good.  A curve or a fan = wheels slipping.",transform=ax.transAxes,
        ha="center",fontsize=8.3,color=INK2)
save(fig,"02-ticks-per-cm.svg")

# ---------------------------------------------------------------- 03
fig,axes = canvas(10.4,5.0,2)
head(fig,"Step 3 · Steering jerk",
     "Jerk is the rate of change of yaw acceleration. Slew-limiting the servo trades a little turn-in speed for a lot of repeatability.",ILL)
t=np.linspace(0,0.9,400)
ax=axes[0]; strip(ax,"both")
for lim,c,lab in [(0.0,BAD,"no slew limit"),(2.5,C1,"slew 2.5 °/cycle")]:
    tau = 0.035 if lim==0 else 0.14
    rate = 70*(1-np.exp(-t/tau))
    jerk = np.gradient(np.gradient(rate,t),t)
    ax.plot(t,jerk/1000,color=c,label=lab)
ax.axhline(0,color=INK3,lw=0.8)
ax.set_xlabel("time from steering command (s)"); ax.set_ylabel("yaw jerk (×1000 °/s³)")
ax.legend(loc="upper right"); ax.set_title("the spike you are removing",color=INK2,fontsize=9.5)
ax=axes[1]; strip(ax,"both")
slew=np.array([0,0.5,1,2,2.5,4,6,8]); peak=np.array([58,34,21,11.5,9.4,7.9,7.3,7.0])
ax.plot(slew,peak,color=C2,marker="o",markeredgecolor=CARD,markeredgewidth=1.1)
ax.scatter([2.5],[9.4],s=150,facecolor="none",edgecolor=GOOD,linewidth=2,zorder=5)
ax.annotate("the knee — past here you are\ngiving up turn-in for nothing\nSERVO_SLEW = 2.5",xy=(2.5,9.4),
            xytext=(3.4,28),fontsize=8.5,color=GOOD,fontweight="bold",
            arrowprops=dict(arrowstyle="-",color=GOOD,lw=1.2))
ax.set_xlabel("servo slew limit (° per control cycle)"); ax.set_ylabel("peak |jerk| (×1000 °/s³)")
ax.set_title("pick the knee",color=INK2,fontsize=9.5)
save(fig,"03-steering-jerk.svg")

# ---------------------------------------------------------------- 04
fig,ax = canvas(8.8,5.1,1)
head(fig,"Step 4 · True straight servo angle",
     "Drive a fixed distance at each candidate angle and record accumulated heading. Straight means zero. The minimum wins.",
     "Illustrative shape. The 69.0° shown is the NATIONALS car — new servo and new Ackermann linkage mean this must be re-run.")
ang=np.arange(65,74,1.0)
drift=0.34+0.46*(ang-69.0)**2+rng.normal(0,0.05,ang.size)
strip(ax,"both")
ax.plot(ang,drift,color=C1,marker="o",markeredgecolor=CARD,markeredgewidth=1.2)
i=int(np.argmin(drift))
ax.scatter([ang[i]],[drift[i]],s=170,facecolor="none",edgecolor=GOOD,linewidth=2.2,zorder=5)
ax.annotate(f"SERVO_TRUE_STRAIGHT = {ang[i]:.0f}°",xy=(ang[i],drift[i]),xytext=(ang[i]+0.5,6),
            fontsize=10,fontweight="bold",color=GOOD,
            arrowprops=dict(arrowstyle="-",color=GOOD,lw=1.2))
ax.set_xlabel("commanded servo angle (°)"); ax.set_ylabel("mean |heading drift| over 150 cm (°)")
ax.text(0.5,1.035,"A clear V is what you want. Flat or noisy → run further. Minimum at the edge → your guess was off, re-centre the sweep.",
        transform=ax.transAxes,ha="center",fontsize=8.4,color=INK2)
save(fig,"04-true-straight.svg")

# ---------------------------------------------------------------- 05
fig,ax = canvas(9.6,5.3,1)
head(fig,"Step 5 · ToF floor signal threshold",
     "Signal strength separates floor from wall, and it works backwards from instinct: the FLOOR return is the strong one.",ILL)
strip(ax,"y")
groups=["floor\n(no wall)","wall\n200 mm","wall\n400 mm","wall\n600 mm","wall\n800 mm","wall\n1000 mm","wall\n1200 mm"]
mus=[11.5,3.1,2.6,2.2,1.8,1.4,1.05]; sds=[1.5,0.35,0.3,0.28,0.25,0.22,0.2]
for k,(mu,sd) in enumerate(zip(mus,sds)):
    v=rng.normal(mu,sd,90); x=k+rng.normal(0,0.055,90)
    col = BAD if k==0 else C1
    ax.scatter(x,v,s=9,color=col,alpha=0.5,edgecolor="none")
    ax.plot([k-0.26,k+0.26],[v.mean()]*2,color=INK,lw=1.8,zorder=4)
ax.axhline(4.0,color=GOOD,lw=1.8,ls="--")
ax.text(6.42,4.35,"SIGNAL_MIN_MCPS = 4.0",color=GOOD,fontsize=9.5,fontweight="bold",ha="right")
ax.text(6.42,9.2,"above the line → floor → DISCARD",color=BAD,fontsize=9,ha="right",fontweight="bold")
ax.text(6.42,1.9,"below the line → real wall → KEEP",color=C1,fontsize=9,ha="right",fontweight="bold")
ax.set_xticks(range(len(groups))); ax.set_xticklabels(groups)
ax.set_ylabel("peak signal rate (MCPS)")
ax.text(0.5,1.035,"Two clouds that barely touch. Put the threshold in the gap, nearer the wall side. If they overlap, fix the collimator — not the number.",
        transform=ax.transAxes,ha="center",fontsize=8.4,color=INK2)
save(fig,"05-tof-threshold.svg")

# ---------------------------------------------------------------- 06
fig,axes = canvas(10.4,5.6,2,sharey=True)
head(fig,"Step 6 · Ninety degree turns",
     "30 alternating turns. What you are reading is not the average error — it is whether left and right agree.",ILL)
bins=np.linspace(-5,5,26)
ax=axes[0]; strip(ax)
L=rng.normal(-0.15,0.9,60); R=rng.normal(0.1,0.9,60)
ax.hist(L,bins=bins,color=C1,alpha=0.85,edgecolor=CARD,linewidth=1.0,label="LEFT turns")
ax.hist(R,bins=bins,color=C3,alpha=0.7,edgecolor=CARD,linewidth=1.0,label="RIGHT turns")
ax.axvline(0,color=INK,lw=1.2)
ax.set_title("GOOD · both centred on zero, sd < 1.5°",color=GOOD)
ax.set_xlabel("final heading error (°)"); ax.set_ylabel("turns"); ax.legend(loc="upper left")
ax=axes[1]; strip(ax)
L2=rng.normal(-1.9,1.0,60); R2=rng.normal(1.6,1.0,60)
ax.hist(L2,bins=bins,color=C1,alpha=0.85,edgecolor=CARD,linewidth=1.0,label="LEFT turns")
ax.hist(R2,bins=bins,color=C3,alpha=0.7,edgecolor=CARD,linewidth=1.0,label="RIGHT turns")
ax.axvline(0,color=INK,lw=1.2)
ax.set_title("BAD · left and right disagree",color=BAD)
ax.set_xlabel("final heading error (°)"); ax.legend(loc="upper left")
ax.set_ylim(0,21)
ax.text(0.97,0.96,"This is MECHANICAL.\nSteering is not symmetric about\nSERVO_TRUE_STRAIGHT → back to step 4.",
        transform=ax.transAxes,ha="right",va="top",fontsize=8.6,color=BAD,fontweight="bold")
punch(fig,"A 1° bias per corner is 12° by the end of three laps, and 12° is a wall.")
save(fig,"06-turn-90.svg")

# ---------------------------------------------------------------- 07
fig,axes = canvas(10.4,5.0,2)
head(fig,"Step 7 · Heading correction gain",
     "Two separate plots, deliberately — never one chart with two y-axes. Take the KP just below where the weave starts.",ILL)
kp=np.array([0.5,1.0,1.5,2.0,2.5,3.0,4.0,5.0])
rms=np.array([4.1,2.3,1.4,0.95,0.82,0.78,0.80,0.95])
cross=np.array([1,1,2,3,5,9,16,27])
ax=axes[0]; strip(ax,"both")
ax.plot(kp,rms,color=C1,marker="o",markeredgecolor=CARD,markeredgewidth=1.1)
ax.scatter([2.0],[0.95],s=160,facecolor="none",edgecolor=GOOD,linewidth=2,zorder=5)
ax.set_xlabel("HEAD_KP"); ax.set_ylabel("RMS heading error over 3 m (°)")
ax.set_title("accuracy improves with gain",color=INK2,fontsize=9.5)
ax=axes[1]; strip(ax,"both")
ax.plot(kp,cross,color=C2,marker="o",markeredgecolor=CARD,markeredgewidth=1.1)
ax.axhline(6,color=BAD,lw=1.4,ls="--")
ax.text(5.0,7.2,"above 6 = snaking",color=BAD,fontsize=8.6,ha="right",fontweight="bold")
ax.scatter([2.0],[3],s=160,facecolor="none",edgecolor=GOOD,linewidth=2,zorder=5)
ax.annotate("HEAD_KP = 2.0",xy=(2.0,3),xytext=(2.4,18),fontsize=9.5,fontweight="bold",color=GOOD,
            arrowprops=dict(arrowstyle="-",color=GOOD,lw=1.2))
ax.set_xlabel("HEAD_KP"); ax.set_ylabel("zero crossings over 3 m")
ax.set_title("…until it starts to weave",color=INK2,fontsize=9.5)
punch(fig,"Snaking costs distance accuracy: the encoder counts the zigzag, not the straight line.")
save(fig,"07-heading-gain.svg")

# ---------------------------------------------------------------- 08
fig,ax = canvas(9.6,7.0,1)
head(fig,"Step 8 · Floor colour thresholds",
     "Percentages, not raw counts — the ratio between channels survives a dimming LED and changing arena light.",
     "Cluster positions illustrative. The shaded regions are the REAL thresholds compiled into the firmware.\nIf orange and white overlap in your data the light hood is leaking — fix the hood, not the number.")
strip(ax,"both")
ax.add_patch(Rectangle((0,36),24,64-36,facecolor=C1,alpha=0.12,zorder=0))
ax.add_patch(Rectangle((35,0),45,27,facecolor=C2,alpha=0.12,zorder=0))
ax.text(12,58,"BLUE\n%B > 36  and  %R < 24",color=C1,fontsize=9.5,fontweight="bold",ha="center")
ax.text(58,24,"ORANGE\n%R > 35  and  %B < 27",color=C2,fontsize=9.5,fontweight="bold",ha="center")
ax.text(52,44,"neither rule matches → NOTHING\nwhite mat lands here by design",color=INK2,fontsize=9,ha="center")
for (mr,mb,sd,c,lab) in [(15,47,2.0,C1,"blue line"),(48,14,2.4,C2,"orange line"),(32,32,1.8,INK3,"white mat")]:
    r=rng.normal(mr,sd,140); b=rng.normal(mb,sd,140)
    ax.scatter(r,b,s=13,color=c,alpha=0.55,edgecolor="none",label=lab)
ax.axvline(24,color=INK3,lw=0.9,ls=":"); ax.axvline(35,color=INK3,lw=0.9,ls=":")
ax.axhline(27,color=INK3,lw=0.9,ls=":"); ax.axhline(36,color=INK3,lw=0.9,ls=":")
ax.set_xlim(0,70); ax.set_ylim(0,64)
ax.set_xlabel("%R  —  red as a percentage of R+G+B")
ax.set_ylabel("%B  —  blue as a percentage of R+G+B")
ax.legend(loc="lower left",markerscale=1.6,bbox_to_anchor=(0.015,0.02))
punch(fig,"Set each cut point about 3 standard deviations clear of the cluster you are excluding.")
save(fig,"08-floor-colour.svg")

# ----------------------------------------------------------------------------
# Regenerate every figure on this page:
#     python3 media/diagrams/calibration/_generate.py
# (run from the repository root)
#
# The distributions here are ILLUSTRATIVE - they show what a correct result and
# a broken result look like, so you know what you are looking at before you have
# your own data. The thresholds and constants annotated on them are real, taken
# from the firmware. When real measurements exist, replace the synthetic arrays
# and re-run.
#
# Palette validated for colour-vision deficiency: #3B7DD8 #C2711B #6B4FA8 #0E8C7A
# on a #FAFAF7 surface - passes lightness band, chroma floor, CVD separation,
# normal-vision separation and contrast.
# ----------------------------------------------------------------------------
