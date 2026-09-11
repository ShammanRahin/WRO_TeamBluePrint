# Original component illustrations - drawn, not sourced, so the repo carries no
# third-party image licences. Colour-coded by subsystem.
OUT = "repo/media/diagrams/components.svg"
SURF="#FAFAF7"; CARD="#FFFFFF"; EDGE="#E3E3DC"
INK="#2B2B28"; INK2="#6B6B63"; INK3="#96968C"
COMPUTE="#3B7DD8"; POWER="#C2711B"; SENSE="#6B4FA8"; PERCEPT="#0E8C7A"

W,H = 1000, 830
CW,CH = 238, 214
X0,Y0 = 22, 128
GX,GY = 6, 8

p=[]
def a(s): p.append(s)

def cell(cx,cy,accent,name,role,conn,note,body):
    a(f'<g transform="translate({cx},{cy})">')
    a(f'<rect x="0" y="0" width="{CW}" height="{CH}" rx="9" fill="{CARD}" stroke="{EDGE}"/>')
    a(f'<rect x="0" y="0" width="4" height="{CH}" rx="2" fill="{accent}"/>')
    a(f'<g transform="translate(18,20)">{body}</g>')
    a(f'<line x1="14" y1="{CH-78}" x2="{CW-14}" y2="{CH-78}" stroke="{EDGE}"/>')
    a(f'<text x="14" y="{CH-60}" font-size="12.5" font-weight="700" fill="{INK}">{name}</text>')
    a(f'<text x="14" y="{CH-43}" font-size="10.5" fill="{INK2}">{role}</text>')
    a(f'<text x="14" y="{CH-27}" font-size="10" fill="{INK3}">{conn}</text>')
    a(f'<text x="14" y="{CH-10}" font-size="9.5" fill="{accent}">{note}</text>')
    a('</g>')

def pcb(w,h,col,pins_top=0,pins_bot=0,chip=True):
    s=[f'<rect x="0" y="0" width="{w}" height="{h}" rx="4" fill="{col}" opacity="0.16" stroke="{col}" stroke-width="1.4"/>']
    for i in range(pins_top):
        s.append(f'<rect x="{8+i*7}" y="-3.5" width="4" height="7" rx="1" fill="{INK3}"/>')
    for i in range(pins_bot):
        s.append(f'<rect x="{8+i*7}" y="{h-3.5}" width="4" height="7" rx="1" fill="{INK3}"/>')
    if chip:
        s.append(f'<rect x="{w*0.30}" y="{h*0.28}" width="{w*0.40}" height="{h*0.44}" rx="2.5" fill="{INK}" opacity="0.82"/>')
        s.append(f'<circle cx="{w*0.35}" cy="{h*0.36}" r="2" fill="{CARD}"/>')
    return "".join(s)

# ---------------------------------------------------------------- 12 cells
cells=[]

cells.append((COMPUTE,"WeAct BlackPill","STM32F411CEU6 · Cortex-M4 100 MHz","every real-time decision, servo and motor","USB-C · 40 pins · hardware FPU",
  pcb(150,56,COMPUTE,20,20,True)
  + f'<rect x="6" y="16" width="16" height="24" rx="2" fill="{INK}" opacity="0.5"/>'))

cells.append((COMPUTE,"Raspberry Pi 5, 8 GB","Cortex-A76 quad @ 2.4 GHz","camera + lidar + fusion. Advisory only","UART to the STM32 · 115200 8N1",
  pcb(160,62,COMPUTE,26,0,True)
  + f'<rect x="120" y="14" width="30" height="16" rx="2" fill="{INK}" opacity="0.45"/>'
  + f'<rect x="120" y="36" width="30" height="16" rx="2" fill="{INK}" opacity="0.45"/>'))

cells.append((POWER,"BTS7960 driver module","43 A half-bridge pair","RPWM + LPWM from the STM32","B+/B− wired straight from the pack",
  pcb(140,58,POWER,8,8,False)
  + f'<rect x="14" y="12" width="40" height="34" rx="3" fill="{INK}" opacity="0.8"/>'
  + f'<rect x="62" y="12" width="40" height="34" rx="3" fill="{INK}" opacity="0.8"/>'
  + f'<rect x="110" y="10" width="22" height="38" rx="2" fill="{POWER}" opacity="0.5"/>'))

cells.append((POWER,"25GA-370 gearmotor","12 V, hall encoder on the shaft","single driven axle, no differential","quadrature into TIM3 · 0.175 mm/tick",
  f'<rect x="6" y="18" width="58" height="42" rx="7" fill="{POWER}" opacity="0.22" stroke="{POWER}" stroke-width="1.4"/>'
  + f'<rect x="64" y="24" width="34" height="30" rx="3" fill="{POWER}" opacity="0.35" stroke="{POWER}" stroke-width="1.2"/>'
  + f'<rect x="98" y="35" width="40" height="8" rx="4" fill="{INK3}"/>'
  + f'<circle cx="20" cy="39" r="9" fill="{POWER}" opacity="0.45"/>'))

cells.append((POWER,"JX PS-1171MG servo","17 g, digital, metal gear","steers the Ackermann linkage","own 6 V rail — stall spike isolated",
  f'<rect x="26" y="12" width="56" height="52" rx="3" fill="{POWER}" opacity="0.22" stroke="{POWER}" stroke-width="1.4"/>'
  + f'<rect x="6" y="20" width="20" height="10" rx="2" fill="{POWER}" opacity="0.4"/>'
  + f'<rect x="82" y="20" width="20" height="10" rx="2" fill="{POWER}" opacity="0.4"/>'
  + f'<circle cx="68" cy="28" r="12" fill="{POWER}" opacity="0.45"/>'
  + f'<rect x="64" y="4" width="8" height="26" rx="3" fill="{INK}" opacity="0.6"/>'))

cells.append((SENSE,"BNO085 IMU","9-DoF, on-chip sensor fusion","SPI. The heading is the truth","replaced an MPU6050 — too much drift",
  pcb(96,54,SENSE,12,12,True)))

cells.append((SENSE,"TCA9548A mux","8-channel I²C multiplexer, 0x70","three channels used: ToF, ToF, colour","own board · 10-pin IDC to the carrier",
  pcb(120,50,SENSE,16,16,True)))

cells.append((SENSE,"VL53L1X × 2","time-of-flight, 4 m, 940 nm","front and rear protection","printed collimator snout · +2° wedge",
  f'<rect x="10" y="16" width="54" height="42" rx="4" fill="{SENSE}" opacity="0.16" stroke="{SENSE}" stroke-width="1.4"/>'
  + f'<rect x="24" y="26" width="12" height="22" rx="2" fill="{INK}" opacity="0.8"/>'
  + f'<rect x="40" y="26" width="12" height="22" rx="2" fill="{INK}" opacity="0.8"/>'
  + f'<path d="M68,26 L112,14 L112,60 L68,48 Z" fill="{SENSE}" opacity="0.2"/>'
  + f'<rect x="64" y="30" width="8" height="14" rx="2" fill="{SENSE}" opacity="0.7"/>'))

cells.append((SENSE,"TCS34725","RGB + clear, with its own LED","reads the orange and blue corner lines","light hood · thresholds on %R and %B",
  f'<rect x="18" y="14" width="56" height="46" rx="4" fill="{SENSE}" opacity="0.16" stroke="{SENSE}" stroke-width="1.4"/>'
  + f'<circle cx="46" cy="33" r="9" fill="{INK}" opacity="0.75"/>'
  + f'<circle cx="30" cy="50" r="4" fill="{POWER}" opacity="0.8"/><circle cx="62" cy="50" r="4" fill="{POWER}" opacity="0.8"/>'
  + f'<path d="M28,64 L64,64 L70,80 L22,80 Z" fill="{INK}" opacity="0.14"/>'))

cells.append((PERCEPT,"Slamtec RPLIDAR C1","360° scanning, 12 m range","obstacle ranges for the fusion loop","USB · asyncio sealed in one thread",
  f'<ellipse cx="60" cy="26" rx="42" ry="13" fill="{PERCEPT}" opacity="0.28" stroke="{PERCEPT}" stroke-width="1.4"/>'
  + f'<path d="M18,26 L18,50 A42,13 0 0,0 102,50 L102,26" fill="{PERCEPT}" opacity="0.16" stroke="{PERCEPT}" stroke-width="1.4"/>'
  + f'<ellipse cx="60" cy="26" rx="18" ry="6" fill="{PERCEPT}" opacity="0.5"/>'
  + f'<path d="M104,30 A46,46 0 0,1 104,58" fill="none" stroke="{PERCEPT}" stroke-width="1.2" stroke-dasharray="3 3"/>'))

cells.append((PERCEPT,"Camera + wide lens","Picamera2, 640 × 480 at 30 Hz","pillar colour and bearing","distance stays ∞ — the lidar fills it",
  f'<rect x="20" y="16" width="52" height="44" rx="4" fill="{PERCEPT}" opacity="0.16" stroke="{PERCEPT}" stroke-width="1.4"/>'
  + f'<circle cx="46" cy="38" r="15" fill="{INK}" opacity="0.78"/><circle cx="46" cy="38" r="7" fill="{PERCEPT}" opacity="0.6"/>'
  + f'<path d="M76,22 L124,6 L124,70 L76,54 Z" fill="{PERCEPT}" opacity="0.14"/>'))

cells.append((POWER,"3S LiPo pack","11.1 V nominal","four rails, one star ground","one master switch breaks logic rails",
  f'<rect x="10" y="18" width="112" height="44" rx="6" fill="{POWER}" opacity="0.2" stroke="{POWER}" stroke-width="1.4"/>'
  + f'<rect x="22" y="26" width="28" height="28" rx="3" fill="{POWER}" opacity="0.4"/>'
  + f'<rect x="54" y="26" width="28" height="28" rx="3" fill="{POWER}" opacity="0.4"/>'
  + f'<rect x="86" y="26" width="28" height="28" rx="3" fill="{POWER}" opacity="0.4"/>'
  + f'<rect x="122" y="32" width="12" height="16" rx="2" fill="{INK}" opacity="0.55"/>'))

a(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="ui-sans-serif,-apple-system,Segoe UI,Helvetica,Arial,sans-serif">')
a(f'<rect width="{W}" height="{H}" fill="{SURF}"/>')
a(f'<text x="26" y="42" font-size="20" font-weight="700" fill="{INK}">What is on the car</text>')
a(f'<text x="26" y="65" font-size="13.5" fill="{INK2}">Twelve components. Colour marks the subsystem each one belongs to. Drawn for this document, so nothing here carries a third-party image licence.</text>')
for i,(lab,col) in enumerate([("real-time control",COMPUTE),("power and motion",POWER),("sensing",SENSE),("perception",PERCEPT)]):
    x=26+i*186
    a(f'<rect x="{x}" y="86" width="11" height="11" rx="2.5" fill="{col}"/>')
    a(f'<text x="{x+18}" y="96" font-size="11" fill="{INK2}">{lab}</text>')
for i,(accent,name,role,conn,note,body) in enumerate(cells):
    r,c = divmod(i,4)
    cell(X0+c*(CW+GX), Y0+r*(CH+GY), accent, name, role, conn, note, body)
a(f'<text x="26" y="{H-14}" font-size="10.5" fill="{INK3}">Part numbers, suppliers, unit costs and lead times are in BOM.md. Pin assignments are in src/tools/calibration/hardware_config.h.</text>')
a('</svg>')
open(OUT,"w").write("\n".join(p))
print("wrote",OUT)
