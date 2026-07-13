"""
WRO FE 2026 - vehicle layout optimizer (objective: LOWEST CG / most stable).
Sweeps track / wheelbase / wheel-dia against: WRO rules (<=300x200 footprint,
<=300 tall, <=1.5 kg), turn-radius target, parking-bay fit, mass from BOM+PLA,
and a fisheye camera coverage constraint. Ranks by static tip-stability angle
atan((track/2)/CG_height) -- higher = more stable.
PyBullet unavailable on this machine (no wheel, no MSVC, no winget); this is the
analytic model. Dynamic park/tip already validated separately in pymunk.
Units: mm, g, deg.
"""
import numpy as np, itertools, math, os
OUT = r"F:\claude\WRO_TeamBluePrint\media\steering"

# ---- fixed component masses (g) and vertical placement rule z(design) ----
# z = height of the part's CG above ground (mm). Heavy parts low = low CG.
GROUND_CLEAR = 5.0     # chassis floor above ground
DECK_GAP = 5.0         # deck sits this far above wheel tops
LOCK = 45.0            # steer lock (deg)
BAY_DEPTH = 200.0      # parking bay depth (car width must fit with margin)
WALL_MARGIN = 20.0     # 2 cm each side 'parallel' target
CORRIDOR = 1000.0      # 1 m lane
CAM_HFOV = 160.0       # fisheye horizontal FOV (deg)

def masses_and_heights(WD, deck_z):
    floor = GROUND_CLEAR
    axle = WD/2.0
    # (name, mass_g, z_mm)  -- battery in FRONT of motor, both low
    comps = [
        ('battery',   80.0, floor+8),      # 2S LiPo ~80 g on the floor
        ('motor',     82.0, axle),         # GA25-370 at rear axle height
        ('servo',     55.0, deck_z+20),    # MG996R on deck (steering) -- HIGH
        ('pi4b',      46.0, deck_z+6),     # Raspberry Pi on deck
        ('stm32',     10.0, floor+6),
        ('tof_x5',    15.0, axle),         # 5x VL53L1X around perimeter
        ('wiring',    30.0, 20.0),
        ('bearings',  20.0, axle),
        ('camera',     5.0, None),         # z solved for coverage (set later)
    ]
    return comps, floor, axle

def pla_mass(L, W, WD):
    # printed structure: chassis shell + 4 wheels, from solid volume * PLA * infill
    PLA = 1.24e-3            # g/mm^3
    INFILL = 0.35           # effective solid fraction (walls+infill)
    chassis_vol = L*W*3.0 + 2*(L*10*20) + 2*(W*10*20)  # floor plate + rails
    wheel_vol = 4 * (math.pi*(WD/2)**2 * 12 * 0.5)      # 4 wheels, ~half solid
    return (chassis_vol+wheel_vol)*PLA*INFILL

def cam_height(body_top):
    # fisheye must see full corridor 1 car ahead AND clear the car's own stack.
    # 160 deg HFOV covers >1 m width even at <0.2 m, so height is set by NOT being
    # occluded by the body: mount just above the tallest component.
    return body_top + 8.0

def evaluate(T, WB, WD):
    """Return dict of metrics or None if infeasible."""
    wheel_w = 12.0
    W = T + wheel_w + 6                     # overall width (track + tyre + walls)
    front_over, rear_over = 22.0, 25.0
    L = WB + front_over + rear_over         # overall length
    deck_z = WD + DECK_GAP
    body_top = deck_z + 40                  # servo (40 tall) sets the top of stack
    H = cam_height(body_top) + 5            # camera mast tip ~ overall height
    # --- constraints ---
    if W > 200 or L > 300 or H > 300: return None       # footprint / height rule
    if W > BAY_DEPTH - 2*WALL_MARGIN: return None        # must park with 2 cm margin
    R = WB/math.tan(math.radians(LOCK))                  # turn radius (rear-axle centre)
    if not (110 <= R <= 150): return None                # target band
    # bay length = 1.5*L ; need slack (0.5*L). always true by rule, keep note
    # --- mass + CG ---
    comps, floor, axle = masses_and_heights(WD, deck_z)
    zc = cam_height(body_top)
    m = []; mz = []
    for name, mass, z in comps:
        if z is None: z = zc
        m.append(mass); mz.append(mass*z)
    pla = pla_mass(L, W, WD)
    m.append(pla); mz.append(pla*(deck_z*0.45))          # structure CG ~ mid
    M = sum(m); CG = sum(mz)/M
    if M > 1500: return None
    tip = math.degrees(math.atan((T/2)/CG))              # static lateral tip angle
    return dict(T=T, WB=WB, WD=WD, W=W, L=L, H=H, R=R, mass=M, CG=CG, tip=tip,
                cam_z=zc, pla=pla)

# ---- sweep ----
best = None; feas = []
for T in range(80,146,5):
    for WB in range(105,141,5):
        for WD in (38,40,42,45,48):
            r = evaluate(float(T), float(WB), float(WD))
            if r:
                feas.append(r)
                if best is None or r['tip'] > best['tip']: best = r
print('feasible designs:', len(feas))
b = best
print('\n=== RECOMMENDED (lowest CG / most stable) ===')
print('track=%.0f  wheelbase=%.0f  wheel_dia=%.0f mm' % (b['T'],b['WB'],b['WD']))
print('overall  L=%.0f  W=%.0f  H=%.0f mm   (limit 300x200x300)' % (b['L'],b['W'],b['H']))
print('turn radius R=%.0f mm   mass=%.0f g   CG height=%.1f mm   tip-stability=%.1f deg'
      % (b['R'],b['mass'],b['CG'],b['tip']))
print('camera mast height=%.0f mm (fisheye 160 deg clears the stack, covers 1 m lane)' % b['cam_z'])
print('PLA structure est=%.0f g' % b['pla'])

# ---- CG contribution breakdown for the recommended design ----
deck_z = b['WD']+DECK_GAP; body_top = deck_z+40
comps,_,axle = masses_and_heights(b['WD'], deck_z)
zc = cam_height(body_top)
print('\n=== CG contribution (what pulls CG up) ===')
rows=[]
for name,mass,z in comps:
    if z is None: z=zc
    rows.append((name,mass,z,mass*z))
rows.append(('PLA_struct', b['pla'], deck_z*0.45, b['pla']*deck_z*0.45))
tot=sum(r[3] for r in rows)
for name,mass,z,mz in sorted(rows,key=lambda r:-r[3]):
    print('  %-11s %5.0f g  @ z=%5.1f mm   %4.1f%% of CG moment'%(name,mass,z,100*mz/tot))

print('\n=== discrepancies / notes ===')
print('- SERVO dominates CG: MG996R on the deck at z=%.0f mm is the single biggest'%(deck_z+20))
print('  CG driver. Lowering it (flat-mount) or a lighter servo is the #1 stability win.')
print('- prev design "WRo final v5" footprint ~165 x 80 mm: very NARROW track -> higher')
print('  tip risk. Optimizer widens track to %.0f mm for stability within parking margin.'%b['T'])
print('- battery in front of motor (80 g, low) is GOOD for CG - keep it.')
print('- wheel_dia driven to %.0f mm (smaller = lower axle/CG) vs 45 mm start.'%b['WD'])

# ---- plots ----
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fig,ax=plt.subplots(1,2,figsize=(12,4.5))
names=[r[0] for r in sorted(rows,key=lambda r:-r[3])]; vals=[100*r[3]/tot for r in sorted(rows,key=lambda r:-r[3])]
ax[0].barh(names[::-1], vals[::-1], color='#c0603a'); ax[0].set_xlabel('% of CG moment')
ax[0].set_title('What raises the CG (recommended design)')
# tip stability vs track
Ts=sorted(set(r['T'] for r in feas))
tip_by_T=[max(r['tip'] for r in feas if r['T']==t) for t in Ts]
ax[1].plot(Ts, tip_by_T,'o-'); ax[1].axvline(b['T'],color='grey',ls=':')
ax[1].set_xlabel('track (mm)'); ax[1].set_ylabel('best tip-stability (deg)')
ax[1].set_title('Stability vs track (wider = more stable, until parking-width limit)')
plt.tight_layout(); plt.savefig(os.path.join(OUT,'5_layout_opt.png'),dpi=110); plt.close()
print('\nplot saved 5_layout_opt.png'); print('DONE')
