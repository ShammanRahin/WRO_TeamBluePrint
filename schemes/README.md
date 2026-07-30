# Electromechanical diagrams

Wiring and connection diagrams for the vehicle.

| File | Shows |
|---|---|
| `stm32_wiring.svg` | **Full STM32F411 pin-level wiring** — every allocated pin, its peripheral, and what it connects to. Colour-coded by bus type. |
| `wiring_block_diagram.png` | System-level block diagram — power domains and major subsystems |

Pin-level detail, harness pinouts, the power budget and the bring-up order live in
[`../electrical/ELECTRICAL.md`](../electrical/ELECTRICAL.md). These diagrams are the
visual form of the same information.

## Regenerating

`wiring_block_diagram.png` is generated:

```bash
python3 electrical/make_block_diagram.py
```

`stm32_wiring.svg` is hand-maintained. If the pin map in `electrical/ELECTRICAL.md` §5
changes, update the SVG in the same commit — a wiring diagram that disagrees with the pin
map is worse than no diagram.
