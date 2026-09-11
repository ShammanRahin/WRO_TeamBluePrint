// ============================================================================
// 08 - Floor colour thresholds
// ============================================================================
// WHAT THIS GIVES YOU
//   The four cut points that turn a TCS34725 reading into ORANGE, BLUE or
//   NOTHING. Current firmware:
//
//       BLUE   if  %B > 36  and  %R < 24
//       ORANGE if  %R > 35  and  %B < 27
//
// WHY PERCENTAGES AND NOT RAW VALUES
//   Raw red, green and blue counts change with how bright the room is, how
//   far the sensor sits off the mat, and how charged the battery is (the
//   sensor's own LED dims as the pack sags). All three of those move on
//   competition day.
//
//   What does not move is the RATIO between the channels. Orange floor is
//   orange whether it is lit brightly or dimly. So we divide each channel by
//   the total and threshold on the percentage. The same code then works in
//   the pit, under the arena lights, and at the end of a run on a tired pack.
//
//   There is a deliberate gap between the two rules - blue needs %R under 24
//   while orange only needs %B under 27 - so that white mat, which sits in
//   the middle, matches neither and returns NOTHING.
//
// WHAT YOU NEED
//   - The actual competition mat, or the closest thing you have
//   - The sensor mounted at its final ride height, with its hood on. Ride
//     height changes the answer, so a bench reading is useless.
//
// HOW TO RUN IT
//   1. Flash. Serial monitor at 115200.
//   2. Roll the car so the sensor sits over WHITE mat. Press start. It takes
//      SAMPLES readings and prints the distribution.
//   3. Repeat over ORANGE. Then over BLUE.
//   4. Do all three again with the room lights changed - brighter, dimmer,
//      and with someone standing over the car casting a shadow. Competition
//      lighting is not your workshop lighting.
//
// READING THE RESULT
//   For each surface you get mean and sd of %R, %G and %B. Set each cut point
//   roughly three standard deviations clear of the population you are trying
//   to exclude. If the orange and white %R populations overlap at three sd,
//   your hood is leaking ambient light - fix the hood, do not fudge the
//   number.
//
//   COLOR_CONFIRM_MS is the other half of this. A colour has to hold for
//   6 ms before it counts, which kills single-sample noise without adding
//   meaningful lag at 0.7 m/s.
// ============================================================================

#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "hardware_config.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS,
                                          TCS34725_GAIN_4X);
const int SAMPLES = 200;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Wire.begin();
  tcaselect(CH_COLOUR);
  if (!tcs.begin()) {
    Serial.print("TCS34725 not found on mux channel ");
    Serial.println(CH_COLOUR);
    while (1) delay(100);
  }
  Serial.println();
  Serial.println("=== 08 - floor colour thresholds ===");
  Serial.println("  put the sensor over one surface, press start, hold still");
}

void loop() {
  waitForButton("Sensor over the surface? Press start.");

  Stats pr, pg, pb, clear;
  for (int i = 0; i < SAMPLES; i++) {
    uint16_t r, g, b, c;
    tcaselect(CH_COLOUR);
    tcs.getRawData(&r, &g, &b, &c);
    float total = (float)r + g + b;
    if (total <= 0) continue;
    pr.add(100.0 * r / total);
    pg.add(100.0 * g / total);
    pb.add(100.0 * b / total);
    clear.add(c);
    delay(5);
  }

  Serial.println();
  pr.print("  %R", " %");
  pg.print("  %G", " %");
  pb.print("  %B", " %");
  clear.print("  clear", " counts");

  Serial.print("  3-sigma band  %R [");
  Serial.print(pr.mean - 3 * pr.sd(), 1); Serial.print(" .. ");
  Serial.print(pr.mean + 3 * pr.sd(), 1); Serial.print("]   %B [");
  Serial.print(pb.mean - 3 * pb.sd(), 1); Serial.print(" .. ");
  Serial.print(pb.mean + 3 * pb.sd(), 1); Serial.println("]");
  Serial.println("  Record this line against the surface name, then do the next surface.");
  Serial.println();
}
