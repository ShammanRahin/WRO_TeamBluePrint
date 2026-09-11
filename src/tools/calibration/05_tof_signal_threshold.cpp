// ============================================================================
// 05 - ToF floor signal threshold
// ============================================================================
// WHAT THIS GIVES YOU
//   SIGNAL_MIN_MCPS. The return signal strength below which a VL53L1X reading
//   is the mat, not a wall, and must be thrown away.
//
// THE PROBLEM THIS SOLVES
//   The WRO mat is white vinyl and highly reflective. The walls are matte
//   black and barely reflective at all. The sensor has a cone of view, not a
//   laser dot, so when the car noses down even slightly the bottom of that
//   cone clips the floor. The floor sends back a big bright return from close
//   range; the wall sends back a weak one from further away. The sensor picks
//   the bright one and reports a wall that is not there.
//
//   Distance alone cannot separate them. Signal strength can. A real wall
//   return is weak. A floor return is strong. So we set a ceiling on signal
//   rate, and any reading above it is discarded as floor.
//
//   Yes, it is backwards from the usual instinct - here a STRONG signal is
//   the bad one.
//
// WHAT YOU NEED
//   - The car in its finished ride height and rake. Do not do this on a
//     bench, the whole point is the angle the sensor sits at on the car.
//   - A piece of the actual matte black wall material
//   - A tape measure
//
// HOW TO RUN IT
//   1. Flash. Serial monitor at 115200.
//   2. Point the sensor at clear floor with NO wall in range. Let it log for
//      30 seconds. This is your floor-only population.
//   3. Put the wall in front at 200 mm. Log 30 s. Then 400, 600, 800, 1000,
//      1200 mm. These are your wall populations.
//   4. Copy the CSV out of the serial monitor into a spreadsheet.
//
// READING THE RESULT
//   Plot signal rate on the y axis, one column per condition. You will see
//   two clouds that barely touch: floor returns high, wall returns low.
//   SIGNAL_MIN_MCPS goes in the gap, nearer the wall cloud than the floor one
//   so you never throw away a real wall.
//
//   The current value is 4.0. Also note the furthest distance at which the
//   wall still gives a usable return - that is your TOF_MAX_VALID_MM, now
//   1300.
//
//   If the two clouds overlap, the fix is mechanical not numerical: add a
//   collimator snout or tilt the sensor up. See electrical/collimator.py.
// ============================================================================

#include <Wire.h>
#include <VL53L1X.h>
#include "hardware_config.h"

VL53L1X tofA, tofB;
bool okA = false, okB = false;

bool startSensor(VL53L1X &s, uint8_t ch, const char *name) {
  tcaselect(ch);
  s.setTimeout(200);
  if (!s.init()) { Serial.print("  no sensor on channel "); Serial.println(ch); return false; }
  s.setDistanceMode(VL53L1X::Long);
  s.setMeasurementTimingBudget(50000);
  s.startContinuous(50);
  Serial.print("  "); Serial.print(name); Serial.print(" up on channel "); Serial.println(ch);
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Wire.begin();
  Wire.setClock(400000);

  Serial.println();
  Serial.println("=== 05 - ToF floor signal threshold ===");
  okA = startSensor(tofA, CH_TOF_A, "ToF A");
  okB = startSensor(tofB, CH_TOF_B, "ToF B");
  Serial.println();
  Serial.println("ms,sensor,range_mm,signal_mcps,ambient_mcps,status");
}

void logOne(VL53L1X &s, bool present, uint8_t ch, const char *name) {
  if (!present) return;
  tcaselect(ch);
  if (!s.dataReady()) return;
  s.read(false);
  Serial.print(millis());                       Serial.print(",");
  Serial.print(name);                           Serial.print(",");
  Serial.print(s.ranging_data.range_mm);        Serial.print(",");
  Serial.print(s.ranging_data.peak_signal_count_rate_MCPS, 3); Serial.print(",");
  Serial.print(s.ranging_data.ambient_count_rate_MCPS, 3);     Serial.print(",");
  Serial.println(VL53L1X::rangeStatusToString(s.ranging_data.range_status));
}

void loop() {
  logOne(tofA, okA, CH_TOF_A, "A");
  logOne(tofB, okB, CH_TOF_B, "B");
  delay(20);
}
