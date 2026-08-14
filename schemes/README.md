# schemes

The electromechanical connection diagram(s): every electronic component and motor, and how they
connect. WRO asks for this as one or more JPEG/PNG/PDF schematics.

## What belongs here

- A single connection/wiring diagram showing the STM32, BTS7960 + motor, MG996R servo, BNO085,
  the TCA9548A multiplexer with the three VL53L1X and the TCS34725, the encoder, the Raspberry
  Pi + camera, and the four power rails meeting at one star ground.
- Export as PNG or PDF so it renders on GitHub and prints cleanly.

Keep the diagram in agreement with `../electrical/README.md` and `../SPECSHEET.md` - if a pin
moves, update all three.

Files: `wiring.png` (or .pdf) - the connection diagram (add)
