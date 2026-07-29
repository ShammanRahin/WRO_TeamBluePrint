# DESIGN RULES — single-sided, local Dhaka fab

**Date:** 2026-07-28 · Destination: `electrical/DESIGN_RULES.md`
Derived from the shop's published capability sheet (2026-07-28).

Both boards: **1 layer, FR-4, components on TOP, copper on BOTTOM, solder mask on bottom.**

---

## 1. Design philosophy for this process

Design to **nominal with margin**, not to the shop's absolute worst case.

Their stated hole tolerance is ±0.3 mm. Taken literally, a 1.0 mm hole could arrive at
0.7 mm and no 0.1″ header would fit anything. But their own note — *"if a specific drill
bit is unavailable, we'll use the next larger size"* — tells you the real variance is
**bit substitution upward**, not random error in both directions. So: size holes so that
the *next larger bit still works*, and give every pad enough annulus to survive it.

Design to the literal worst case and nothing at 2.54 mm pitch is manufacturable at all.

---

## 2. Rule table — enter these into EasyEDA Pro

| Rule | Value | Shop minimum | Why the margin |
|---|---|---|---|
| Signal trace width | **0.6 mm** | 0.5 mm | 20% over minimum |
| Power / rail trace | **1.5 mm** | — | Current + pad-lift resistance |
| Servo power run | **2.5 mm** | — | ~3.5 A at 1 oz; see `SCHEMATIC.md` §1.6 |
| Clearance (all) | **0.6 mm** | 0.5 mm | |
| Trace to board edge | **1.0 mm** | 0.5 mm | Outline tolerance is ±1 mm |
| Silk text height | **2.2 mm** | 2.03 mm (80 mil) | Arial Bold |

### Drill and pad table

| Part | Drill | Pad Ø | Nominal annulus |
|---|---|---|---|
| 0.1″ header (0.64 mm sq pin, 0.91 mm diagonal) | **1.0 mm** | **2.0 mm** | 0.50 mm |
| JST-XH 2.5 mm | **1.0 mm** | **1.9 mm** | 0.45 mm |
| Screw terminal 5.08 mm | **1.2 mm** | **2.5 mm** | 0.65 mm |
| Jumper wire end | **0.9 mm** | **1.8 mm** | 0.45 mm |
| M3 mounting hole | **3.57 mm** (9/64″) | — | see below |

**Mounting holes:** 3.18 mm (1/8″) is tempting but its worst case is 2.88 mm, which will
not pass an M3 screw. **3.57 mm (9/64″)** worst-cases to 3.27 mm and always clears. Both
bits are on their list.

### JST-PH is excluded

2.0 mm pitch → max pad 1.5 mm against a 1.0 mm drill = 0.25 mm annulus, on a board with
**no plated through-holes**. That pad will lift. **Buy XH only** (Decision #27). One
pitch, one crimp tool, and no two connectors on the robot that can be cross-plugged.

---

## 3. The constraint that shapes the whole layout

**No trace can pass between two adjacent connector pins.**

```
0.1" header:   2.54 mm pitch − 2.0 mm pad  = 0.54 mm gap available
One trace needs: 0.6 clearance + 0.6 trace + 0.6 clearance = 1.8 mm

                          0.54  <<  1.8      ✗
```

JST-XH is the same story (0.6 mm gap). So **every net routes around every connector,
never through a header field.**

Consequences:

1. **Placement is the entire design problem.** Once modules are placed, the routing is
   nearly determined. Spend your time on placement.
2. **Orient every module so its pins face the thing they connect to.** A module rotated
   180° can add five jumpers on its own.
3. **Routing channel pitch is 1.2 mm** (0.6 trace + 0.6 clearance). Budget board area
   accordingly — this is ~4× the density of a JLCPCB 6/6 mil board.

---

## 4. Jumper wire strategy

With no vias, every crossing becomes an insulated wire link on the **top** side, landing
in two drilled holes with bottom-side pads.

- **Target: ≤15 jumpers on Board A, ≤5 on Board B.** If you exceed that, the placement
  is wrong — move modules, don't add wires.
- Use **solid-core 22 AWG insulated** wire, not bare. Bare links short against module
  bodies sitting on top of them.
- Give every jumper a **silkscreen outline and a `JP` designator**. You will rebuild
  this board at least once; undocumented jumpers are how a working board becomes an
  unreproducible one — and Rubric Criterion 5 scores reproducibility directly.
- **Never jumper a power rail.** Route power as copper. A jumper carrying servo or
  motor current is a fire, not a glitch.
- Record the final count in the journal. "We got it to N jumpers by doing X" is exactly
  the iteration evidence Rubric Criterion 4 pays for.

---

## 5. Ground on a single layer

There is no ground plane. You get a **ground pour on the one copper layer**, which is
strictly worse and must be drawn deliberately.

1. Route all **signals first**.
2. Pour ground over everything remaining, bottom layer.
3. **Check the pour is actually continuous.** On a single layer, signal traces cut the
   pour into islands. Every island that isn't connected back to the star is a floating
   copper flag — an antenna, not a ground. Stitch islands with jumpers if you must.
4. **One star point** (`SCHEMATIC.md` §1.2). Motor, servo and logic grounds meet there
   and nowhere else.
5. Keep the ToF/I²C cable ground return away from the servo power run.

This is the weakest part of the design and you should expect to iterate on it after
the first mat tests. Note that honestly in the journal — acknowledged limitations score
better than pretending a 1-layer board has good EMI behaviour.

---

## 6. Board outline and mechanics

- **Rectangular only** (their constraint). No cutouts, no rounded corners.
- Outline tolerance **±1 mm** — so **the board cannot be located by its edges.**
  Locate the stack by **mounting holes**, which are drill-position accurate. This is
  Decision #10 (locate by geometry) applied to the PCB.
- Use **3 mounting holes minimum**, at identical coordinates on both boards so the stack
  aligns. Three, not four — three always sits flat.
- Board B (lower) and Board A (upper) on **M3 standoffs**. Height must clear the tallest
  component on Board B plus the JST cable bend radius.
- Keep the Black Pill's **USB edge overhanging** the outline for flashing access.
- Minimum board size is 1 × 1 inch; maximum 10 × 8 inch. Both boards land well inside.

> ⚠️ **Board A vs the chassis plate.** `BOM.md` records an 80 × 130 mm printed plate.
> Estimated Board A is ~100 × 80 mm at these rules — it fits if you orient it 80 mm
> across the plate width. Verify in Fusion **before** ordering, not after.

---

## 7. Silkscreen — it is documentation

Top silk is the only layer a human reads, and it is free. Include:

- Designator **and function** on every connector: `J4 SERVO`, `J7 IMU`, `J2 ENC`.
- **Pin 1 marked** on every connector, plus pin-1 signal name.
- Polarity on every electrolytic, orientation arrow on every module.
- Board name, revision, date: `BLUEPRINT MAIN v1.0 2026-07`.
- Every jumper outlined with a `JP` number.

Text is ≥2.2 mm Arial Bold, which is large — lay it out as you place, not at the end,
or it will not fit.

---

## 8. EasyEDA Pro → PDF export (the shop takes PDF, not Gerber)

Their requirement for a single-layer board with solder mask:

| File | Content | Mirrored? |
|---|---|---|
| Top Silk | Component-side silkscreen | No |
| Bottom Copper | The copper layer | **YES — flip horizontally** |
| Bottom Resist | Solder mask openings | **YES — flip horizontally** |

Zip all three and submit.

**Procedure:**

1. Run **DRC** with the §2 rules loaded. Zero violations before you export anything.
2. `File → Export → PDF`.
3. Export each layer as a **separate PDF**, one layer selected per file.
4. **Scale = 1:1.** No "fit to page", no "shrink to printable area". This is the single
   most common way a board arrives wrong.
5. Set **Mirror** on Bottom Copper and Bottom Resist.
6. Choose **black-and-white / monochrome**, not greyscale. Anti-aliased grey edges can
   be misinterpreted by their imaging step.
7. Include the board outline on every sheet so they can register the layers.

**Verify before sending — this takes five minutes and has saved entire board runs:**

- Print all three PDFs on paper at 100%.
- Measure a known dimension with calipers — the 2.54 mm header pitch across 10 pins
  should read **25.4 mm**. If it doesn't, the scale is wrong.
- Lay the printed bottom-copper sheet under a real 0.1″ header and a real JST-XH
  connector. Pins should land in pads.
- Confirm the mirrored sheets are actually mirrored: text on bottom copper should read
  **backwards** on the printout.

---

## 9. Test coupon — do this before the real boards

A 1 × 1 inch scrap board, submitted first. It validates every number in §2 against what
the shop *actually delivers* rather than what they advertise.

Put on it:

| Feature | Tests |
|---|---|
| 1× 8-pin 0.1″ header footprint | Do real pins fit? Is the annulus intact? |
| 1× JST-XH 4-pin footprint | Same, at 2.5 mm pitch |
| 1× M3 hole at 3.57 mm | Does an M3 screw pass? |
| Trace/gap ladder at 0.5 / 0.6 / 0.8 / 1.0 mm | Where does their process actually break? |
| 2 mm and 3 mm silk text | Is 2.2 mm really legible? |
| A 2.5 mm power trace | Width fidelity |
| Solder mask over half the traces | Registration accuracy |

Then **solder to it and pull the pads off with pliers.** You need to know how much force
lifts a pad on this process before you find out via an intermittent fault in September.

Cost: one board minimum order. Against the risk of etching two full boards to wrong
assumptions eight weeks before the competition, this is the cheapest insurance available.

---

## 10. Assembly rules (Decision #11 / #12 carried onto the PCB)

- **No Dupont anywhere.** JST-XH crimps or screw terminals only.
- **Strain relief on every cable** leaving a board — a zip-tie anchor hole next to each
  connector costs one drill hit. On a board with no plated holes, cable strain is the
  #1 pad-lift cause.
- **Twist the encoder A/B pair.** Twist the I²C SCL/SDA pair in the inter-board cable.
- Route the inter-board cable and the encoder cable **away from the motor leads and
  the servo run**.
- Threadlocker or nail polish on standoff threads (Decision #10).
- Solder every module header — do not rely on friction fit.

---

## 11. Checkpoint reviews

Per the agreed workflow, send at these three points:

| Checkpoint | Send | Reviewed for |
|---|---|---|
| **1 — Schematic** | PDF export of both sheets | Netlist vs `SCHEMATIC.md`, pin map vs `PINMAP.md`, missing decoupling, pull-ups, star ground |
| **2 — Placement** | Screenshot, modules placed, no routing | Jumper count implied by placement, module orientation, servo/motor run separation, board size vs chassis |
| **3 — Routing + DRC** | DRC report + the three export PDFs | Rule violations, ground pour continuity and islands, trace widths on power, mirror and 1:1 scale |
