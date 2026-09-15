#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>

// ============================================================
// SENSOR CHECK - colour threshold calibration + front ToF readout
//
// Standalone bench sketch. No motor, no servo, no IMU, no FSM.
// Streams live ToF + colour, and can capture each surface to work
// out where the ORANGE / BLUE thresholds should sit.
//
// IMPORTANT: this reads the TCS34725 with the exact same direct
// register burst the firmware uses (NOT tcs.getRawData), so the
// numbers here are the numbers classifyColor() will see.
//
// SERIAL COMMANDS (115200, send a single letter + Enter)
//   o   capture ORANGE  - hold the sensor over an orange line
//   b   capture BLUE    - hold the sensor over a blue line
//   w   capture WHITE   - hold the sensor over the plain mat
//   r   report + suggested thresholds to paste into openround.ino
//   l   toggle the live stream on/off
//   ?   this help
// ============================================================

#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70
#define TOF_CH      3      // front VL53L0X
#define TCS_CH      4      // TCS34725

// ---- thresholds currently live in openround.ino ----
float DARK_TOTAL_FLOOR = 100.0f;   // total < this -> COLOR_NONE
float BLUE_PB_FLOOR    = 36.0f;    // BLUE   needs pB >
float BLUE_PR_CAP      = 28.0f;    // BLUE   needs pR <
float ORANGE_PR_FLOOR  = 36.0f;    // ORANGE needs pR >
float ORANGE_PB_CAP    = 28.0f;    // ORANGE needs pB <
const float WHITE_TOL  = 7.0f;     // diagnostic only: |p - 33.3| < tol on all three

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);
VL53L0X lox;
bool tcsOk = false, tofOk = false;
bool liveStream = true;

// ============================================================
// I2C PLUMBING
// ============================================================
void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void resetTCA() {
  pinMode(TCA_RST_PIN, OUTPUT);
  digitalWrite(TCA_RST_PIN, LOW);
  delay(10);
  digitalWrite(TCA_RST_PIN, HIGH);
  delay(10);
}

// Same burst read the firmware uses - command | auto-increment | CDATAL
void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  r = g = b = c = 0;
  if (!tcsOk) return;
  tcaselect(TCS_CH);
  Wire.beginTransmission(0x29);
  Wire.write(0x80 | 0x20 | 0x14);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x29, (uint8_t)8);
  if (Wire.available() < 8) return;
  c  = (uint16_t)Wire.read();  c |= (uint16_t)Wire.read() << 8;
  r  = (uint16_t)Wire.read();  r |= (uint16_t)Wire.read() << 8;
  g  = (uint16_t)Wire.read();  g |= (uint16_t)Wire.read() << 8;
  b  = (uint16_t)Wire.read();  b |= (uint16_t)Wire.read() << 8;
}

uint16_t readToF(bool &valid) {
  valid = false;
  if (!tofOk) return 0;
  tcaselect(TOF_CH);
  uint16_t d = lox.readRangeContinuousMillimeters();
  valid = (!lox.timeoutOccurred() && d > 0 && d < 1200);
  return d;
}

// ============================================================
// SAMPLE STATISTICS
// ============================================================
struct Stats {
  bool  have;
  int   n;
  float pRmin, pRmax, pRsum;
  float pGmin, pGmax, pGsum;
  float pBmin, pBmax, pBsum;
  float tMin,  tMax,  tSum;
};
Stats orangeS, blueS, whiteS;

void resetStats(Stats &s) {
  s.have = false; s.n = 0;
  s.pRmin = s.pGmin = s.pBmin = 1e9f;  s.tMin = 1e9f;
  s.pRmax = s.pGmax = s.pBmax = -1e9f; s.tMax = -1e9f;
  s.pRsum = s.pGsum = s.pBsum = s.tSum = 0.0f;
}

void addSample(Stats &s, float pR, float pG, float pB, float total) {
  if (pR < s.pRmin) s.pRmin = pR;   if (pR > s.pRmax) s.pRmax = pR;
  if (pG < s.pGmin) s.pGmin = pG;   if (pG > s.pGmax) s.pGmax = pG;
  if (pB < s.pBmin) s.pBmin = pB;   if (pB > s.pBmax) s.pBmax = pB;
  if (total < s.tMin) s.tMin = total; if (total > s.tMax) s.tMax = total;
  s.pRsum += pR; s.pGsum += pG; s.pBsum += pB; s.tSum += total;
  s.n++;
}

void printRange(const __FlashStringHelper *tag, float lo, float hi, float avg) {
  Serial.print(tag);
  Serial.print(lo, 1); Serial.print('-'); Serial.print(hi, 1);
  Serial.print(F(" avg ")); Serial.print(avg, 1);
}

void printStats(const __FlashStringHelper *name, Stats &s) {
  Serial.print(F("  ")); Serial.print(name);
  if (!s.n) { Serial.println(F("  -- not captured --")); return; }
  printRange(F("  pR "), s.pRmin, s.pRmax, s.pRsum / s.n);
  printRange(F(" | pG "), s.pGmin, s.pGmax, s.pGsum / s.n);
  printRange(F(" | pB "), s.pBmin, s.pBmax, s.pBsum / s.n);
  Serial.print(F(" | total ")); Serial.print((long)s.tMin);
  Serial.print('-');            Serial.print((long)s.tMax);
  Serial.print(F(" ("));        Serial.print(s.n); Serial.println(F(" samples)"));
}

void capture(Stats &s, const __FlashStringHelper *name) {
  resetStats(s);
  Serial.print(F("\nSampling ")); Serial.print(name);
  Serial.println(F(" for 1.5 s - hold the sensor still over the surface..."));
  unsigned long t0 = millis();
  while (millis() - t0 < 1500) {
    uint16_t r, g, b, c;
    readColor(r, g, b, c);
    float total = (float)r + (float)g + (float)b;
    if (total > 0.0f) {
      addSample(s, (r / total) * 100.0f, (g / total) * 100.0f, (b / total) * 100.0f, total);
    }
    delay(10);
  }
  s.have = (s.n > 0);
  printStats(name, s);
  if (s.have && s.tMin < DARK_TOTAL_FLOOR) {
    Serial.println(F("  !! some samples fell under DARK_TOTAL_FLOOR - sensor is too far or too dim"));
  }
}

// ============================================================
// THRESHOLD SUGGESTION
// One rule = one boundary between the target class and the two others.
// The suggestion sits halfway across the gap; a negative gap means the
// classes overlap and no threshold can separate them.
// ============================================================
void suggestRule(const __FlashStringHelper *label, float lowSide, float highSide) {
  Serial.print(F("  ")); Serial.print(label);
  float gap = highSide - lowSide;
  if (gap <= 0.0f) {
    Serial.print(F("OVERLAP by ")); Serial.print(-gap, 1);
    Serial.println(F(" pts - cannot separate, fix lighting/height first"));
  } else {
    Serial.print(F("suggest ")); Serial.print(0.5f * (lowSide + highSide), 1);
    Serial.print(F("   (clear gap ")); Serial.print(gap, 1); Serial.println(F(" pts)"));
  }
}

void report() {
  Serial.println(F("\n=============== THRESHOLD REPORT ==============="));
  printStats(F("ORANGE"), orangeS);
  printStats(F("BLUE  "), blueS);
  printStats(F("WHITE "), whiteS);

  if (!orangeS.have || !blueS.have || !whiteS.have) {
    Serial.println(F("\nCapture all three (o, b, w) before asking for suggestions."));
    Serial.println(F("================================================\n"));
    return;
  }

  Serial.println(F("\n-- suggested thresholds --"));
  // BLUE needs pB above everything else, and pR below everything else
  suggestRule(F("BLUE_PB_FLOOR    "), max(whiteS.pBmax, orangeS.pBmax), blueS.pBmin);
  suggestRule(F("BLUE_PR_CAP      "), blueS.pRmax, min(whiteS.pRmin, orangeS.pRmin));
  // ORANGE needs pR above everything else, and pB below everything else
  suggestRule(F("ORANGE_PR_FLOOR  "), max(whiteS.pRmax, blueS.pRmax), orangeS.pRmin);
  suggestRule(F("ORANGE_PB_CAP    "), orangeS.pBmax, min(whiteS.pBmin, blueS.pBmin));

  float lowestTotal = min(min(orangeS.tMin, blueS.tMin), whiteS.tMin);
  Serial.print(F("  DARK_TOTAL_FLOOR   suggest "));
  Serial.print((long)(lowestTotal * 0.5f));
  Serial.print(F("   (dimmest valid reading was ")); Serial.print((long)lowestTotal);
  Serial.println(F(")"));

  Serial.println(F("\n-- paste into classifyColor() in openround.ino --"));
  Serial.print(F("  if (total < "));
  Serial.print((long)(lowestTotal * 0.5f));
  Serial.println(F(".0f) return COLOR_NONE;"));
  Serial.print(F("  if (pB > "));
  Serial.print(0.5f * (max(whiteS.pBmax, orangeS.pBmax) + blueS.pBmin), 1);
  Serial.print(F("f && pR < "));
  Serial.print(0.5f * (blueS.pRmax + min(whiteS.pRmin, orangeS.pRmin)), 1);
  Serial.println(F("f) return COLOR_BLUE;"));
  Serial.print(F("  if (pR > "));
  Serial.print(0.5f * (max(whiteS.pRmax, blueS.pRmax) + orangeS.pRmin), 1);
  Serial.print(F("f && pB < "));
  Serial.print(0.5f * (orangeS.pBmax + min(whiteS.pBmin, blueS.pBmin)), 1);
  Serial.println(F("f) return COLOR_ORANGE;"));
  Serial.println(F("================================================\n"));
}

// ============================================================
// LIVE STREAM
// ============================================================
void printHelp() {
  Serial.println(F("\ncommands: o=orange  b=blue  w=white  r=report  l=live on/off  ?=help"));
}

void streamLine() {
  bool tofValid;
  uint16_t d = readToF(tofValid);

  uint16_t r, g, b, c;
  readColor(r, g, b, c);
  float total = (float)r + (float)g + (float)b;
  float pR = 0, pG = 0, pB = 0;
  if (total > 0.0f) {
    pR = (r / total) * 100.0f;
    pG = (g / total) * 100.0f;
    pB = (b / total) * 100.0f;
  }

  Serial.print(F("TOF "));
  if (tofValid) { Serial.print(d); Serial.print(F(" mm")); }
  else          { Serial.print(F("  OOR")); }

  Serial.print(F(" | R ")); Serial.print(r);
  Serial.print(F(" G "));   Serial.print(g);
  Serial.print(F(" B "));   Serial.print(b);
  Serial.print(F(" C "));   Serial.print(c);
  Serial.print(F(" | pR ")); Serial.print(pR, 1);
  Serial.print(F(" pG "));   Serial.print(pG, 1);
  Serial.print(F(" pB "));   Serial.print(pB, 1);
  Serial.print(F(" | tot ")); Serial.print((long)total);

  Serial.print(F(" -> "));
  if (total < DARK_TOTAL_FLOOR) {
    Serial.println(F("DARK (below total floor)"));
  } else if (pB > BLUE_PB_FLOOR && pR < BLUE_PR_CAP) {
    Serial.println(F("BLUE"));
  } else if (pR > ORANGE_PR_FLOOR && pB < ORANGE_PB_CAP) {
    Serial.println(F("ORANGE"));
  } else if (fabs(pR - 33.3f) < WHITE_TOL &&
             fabs(pG - 33.3f) < WHITE_TOL &&
             fabs(pB - 33.3f) < WHITE_TOL) {
    Serial.println(F("WHITE / neutral"));
  } else {
    Serial.println(F("NONE (no rule matched)"));
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  resetTCA();
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  Serial.println(F("\n--- SENSOR CHECK ---"));

  tcaselect(TOF_CH);
  lox.setBus(&Wire);
  lox.setTimeout(100);
  tofOk = lox.init();
  if (tofOk) {
    lox.setMeasurementTimingBudget(30000);
    lox.startContinuous(0);
  }
  Serial.println(tofOk ? F("ToF FRONT (CH3): READY") : F("ToF FRONT (CH3): FAILED"));

  tcaselect(TCS_CH);
  delay(10);
  tcsOk = tcs.begin();
  Serial.println(tcsOk ? F("Colour  (CH4): READY") : F("Colour  (CH4): FAILED"));

  resetStats(orangeS); resetStats(blueS); resetStats(whiteS);
  printHelp();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'o': case 'O': capture(orangeS, F("ORANGE")); break;
      case 'b': case 'B': capture(blueS,   F("BLUE  ")); break;
      case 'w': case 'W': capture(whiteS,  F("WHITE ")); break;
      case 'r': case 'R': report();  break;
      case 'l': case 'L':
        liveStream = !liveStream;
        Serial.println(liveStream ? F("live ON") : F("live OFF"));
        break;
      case '?': printHelp(); break;
      default: break;   // swallow newlines
    }
  }

  static unsigned long lastPrint = 0;
  if (liveStream && millis() - lastPrint >= 100) {
    lastPrint = millis();
    streamLine();
  }
}
