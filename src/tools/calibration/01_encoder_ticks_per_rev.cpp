// ============================================================================
// 01 - Encoder ticks per wheel revolution
// ============================================================================
// WHAT THIS GIVES YOU
//   The number of encoder counts the drive wheel produces in exactly one
//   full turn. Everything downstream - distance, lap length, the lane-gap
//   measurement - is built on this number, so get it right first.
//
// WHY WE DO IT THIS WAY
//   You cannot trust the motor's datasheet. Gearbox backlash, a slightly
//   slipping coupler and quadrature edge counting all shift the real figure.
//   So we measure it 100 times and look at the distribution. The mean is the
//   answer. The standard deviation tells you whether to believe the mean.
//
// WHAT YOU NEED
//   - The car, or just the powered rear axle, on a flat hard floor
//   - A marker pen. Put a single line on the tyre sidewall and a matching
//     line on the chassis right next to it, so you can see when the wheel has
//     come back round to exactly the same place
//   - Nothing else. This sketch never drives the motor; you turn the wheel
//     by hand.
//
// HOW TO RUN IT
//   1. Flash. Open the serial monitor at 115200.
//   2. Line the two marks up.
//   3. Press the start button. The counter zeroes.
//   4. Turn the wheel forward by hand, exactly one revolution, until the
//      marks line up again. Go slowly and evenly. Do not overshoot and back
//      up - a backed-up sample is a bad sample.
//   5. Press the button again. The sample is recorded and printed.
//   6. Repeat 100 times. It takes about fifteen minutes. Put music on.
//
// READING THE RESULT
//   After every sample the sketch prints the running mean and sd. Watch the
//   sd: it should settle inside about 1 percent of the mean. If it keeps
//   growing, you are either overshooting the mark or the encoder is missing
//   counts - check the coupler grub screw and the encoder wiring before you
//   carry on collecting numbers.
//
//   Write the final mean into your notes. Step 02 uses it.
// ============================================================================

#include <Wire.h>
#include "hardware_config.h"

Stats ticksPerRev;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  encoderBegin();
  pinMode(START_BTN_PIN, INPUT_PULLUP);

  Serial.println();
  Serial.println("=== 01 - encoder ticks per wheel revolution ===");
  Serial.println("Line up your marks, press the button, turn one full");
  Serial.println("revolution forward, press the button again.");
  Serial.println("Target: 100 samples.");
  Serial.println();
}

void loop() {
  waitForButton("Marks lined up? Press start to zero the counter.");
  zeroEncoder();
  Serial.println("  ... counting. Turn ONE revolution, then press start.");

  waitForButton("");
  long counts = readEncoder();

  if (counts <= 0) {
    Serial.print("  REJECTED: got ");
    Serial.print(counts);
    Serial.println(" counts. Did you turn it backwards? Sample discarded.");
    return;
  }

  ticksPerRev.add((double)counts);
  Serial.print("  sample ");
  Serial.print(ticksPerRev.n);
  Serial.print(": ");
  Serial.print(counts);
  Serial.println(" counts");
  ticksPerRev.print("  running:", " ticks/rev");

  if (ticksPerRev.n >= 100) {
    Serial.println();
    Serial.println("=== 100 samples collected ===");
    ticksPerRev.print("FINAL:", " ticks/rev");
    Serial.println("Record the mean. Move on to sketch 02.");
    Serial.println();
  }
}
