// ============================================================================
// 02 - Ticks per centimetre (distance calibration)
// ============================================================================
// WHAT THIS GIVES YOU
//   TICKS_PER_CM. Currently 31.933 in the firmware.
//
// WHY TWO STEPS AND NOT ONE
//   A caliper reading of the wheel gives you a first guess:
//       ticks_per_cm = ticks_per_rev / (pi * wheel_diameter_cm)
//   That guess is always slightly wrong, because the loaded rolling radius is
//   smaller than the free diameter - the tyre squashes under the car's weight
//   and the contact patch flattens. How much smaller depends on the tyre, the
//   mass and the floor. You cannot calculate it, you have to drive it.
//
//   So: caliper for the estimate, then a measured run for the truth, then fit
//   a straight line through several run lengths. The slope of that line is
//   ticks per cm. Fitting a line rather than dividing one number by another
//   cancels the fixed error at the start and end of each run - the bit where
//   the car is accelerating and the bit where it coasts to a stop.
//
// WHAT YOU NEED
//   - A tape measure and a flat run of at least 2 m of the surface you will
//     actually compete on. Carpet and mat give different answers.
//   - Masking tape to mark the start line and each target distance
//   - The car fully assembled and at competition weight. Battery in. Every
//     gram changes the rolling radius.
//
// HOW TO RUN IT
//   1. Flash. Serial monitor at 115200.
//   2. Mark a start line. Mark target lines at 25, 50, 75, 100, 150 and 200 cm.
//   3. Line the car's rear axle up with the start line. Press the button.
//      The car drives forward slowly and stops when you press the button again.
//   4. Stop it at each target line in turn. Measure where the rear axle
//      actually ended up - not where you aimed - and type that number into
//      the serial monitor, in cm, then press enter.
//   5. Do the whole set three times. Eighteen points total.
//
// READING THE RESULT
//   The sketch does the least-squares fit as you go and prints the slope.
//   The slope is TICKS_PER_CM. The intercept should be small; a large
//   intercept means you have a consistent measuring bias, usually from
//   sighting the axle against the tape at an angle.
//
//   R-squared should be above 0.999. If it is lower, one of your distance
//   readings is wrong, or the wheels are slipping.
// ============================================================================

#include <Wire.h>
#include "hardware_config.h"

const int CAL_SPEED = 110;   // slow enough not to slip, fast enough to move

// running sums for the least-squares fit of ticks = slope * cm + intercept
double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
long   nPoints = 0;

void report() {
  if (nPoints < 2) return;
  double denom = nPoints * sxx - sx * sx;
  if (fabs(denom) < 1e-9) return;
  double slope = (nPoints * sxy - sx * sy) / denom;
  double intercept = (sy - slope * sx) / nPoints;
  double r = (nPoints * sxy - sx * sy) /
             sqrt((nPoints * sxx - sx * sx) * (nPoints * syy - sy * sy));

  Serial.println();
  Serial.print("  points=");    Serial.print(nPoints);
  Serial.print("  TICKS_PER_CM = "); Serial.print(slope, 4);
  Serial.print("  intercept = ");    Serial.print(intercept, 2);
  Serial.print(" ticks  R2 = ");     Serial.println(r * r, 6);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  encoderBegin();
  motorBegin();
  setMotorSpeed(0);
  pinMode(START_BTN_PIN, INPUT_PULLUP);

  Serial.println();
  Serial.println("=== 02 - ticks per centimetre ===");
  Serial.println("Press start to roll. Press start again to stop.");
  Serial.println("Then type the MEASURED distance in cm and press enter.");
  Serial.println();
}

void loop() {
  waitForButton("Car on the start line? Press start to roll.");
  zeroEncoder();
  setMotorSpeed(CAL_SPEED);

  // roll until the button is pressed again
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  while (digitalRead(START_BTN_PIN) == HIGH) delay(2);
  setMotorSpeed(0);
  while (digitalRead(START_BTN_PIN) == LOW) delay(5);

  long counts = readEncoder();
  Serial.print("  stopped at ");
  Serial.print(counts);
  Serial.println(" ticks.");
  Serial.print("  measured distance in cm > ");

  // wait for a typed number
  while (!Serial.available()) delay(10);
  double cm = Serial.parseFloat();
  while (Serial.available()) Serial.read();   // flush the newline

  if (cm <= 0.5 || counts <= 0) {
    Serial.println("  REJECTED - nonsense input, sample discarded.");
    return;
  }
  Serial.println(cm, 2);

  sx  += cm;        sy  += (double)counts;
  sxx += cm * cm;   sxy += cm * (double)counts;
  syy += (double)counts * (double)counts;
  nPoints++;
  report();
}
