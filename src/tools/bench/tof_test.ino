#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ============================================================
// ToF PAIR TEST - front (mux CH3) + rear (mux CH4) VL53L0X
//
// Reads both sensors as fast as they deliver (no blocking wait - the same
// polled read the park-out sketch uses) and prints, 5 times a second:
//
//   F  87 mm  [min 85 max 90 avg 87.4 sd 1.2]  | R 412 mm [...]  | 49 Hz / 48 Hz
//
// USE IT TO FIND THE STOP THRESHOLD (PARK_SAFE_MM in park_out.ino, 50 mm):
//   1. Put a wall (a magenta parking block is best - same surface as the
//      real thing) 150 mm from the sensor. Send 'r' to reset the stats.
//   2. Move it closer in 10 mm steps, 'r' after each step, and watch avg
//      and sd. Also watch the "!" flags:
//        ! TIMEOUT   the sensor did not answer in time
//        ! 8190+     no target / out of range (reads as FAR in park_out)
//        ! <20       too close to trust
//   3. The threshold is the smallest distance where avg still tracks the
//      real distance (within ~10 mm) and sd stays under ~5 mm. Below that
//      the VL53L0X usually starts reading high or jumping - it can NOT be
//      trusted to stop the car there. Set PARK_SAFE_MM a little above it.
//   4. Also note what each sensor reads with nothing in front (sky, the
//      far wall): that is what "open" looks like.
//
// Serial commands (115200):  r = reset stats   f / b = only front / only rear
//                            a = both (default)
// ============================================================

#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70

const uint8_t FRONT_CH = 3;
const uint8_t REAR_CH  = 4;
const uint32_t TIMING_BUDGET_US = 20000;   // 20 ms = ~50 Hz per sensor (fastest reliable)

VL53L0X tofFront, tofRear;
bool    okFront = false, okRear = false;

void tcaselect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void resetTCA() {
  pinMode(TCA_RST_PIN, OUTPUT);
  digitalWrite(TCA_RST_PIN, LOW);  delay(10);
  digitalWrite(TCA_RST_PIN, HIGH); delay(10);
}

bool initTof(VL53L0X &s, uint8_t ch, const __FlashStringHelper *name) {
  tcaselect(ch);
  s.setBus(&Wire);
  s.setTimeout(100);
  if (!s.init()) {
    Serial.print(name); Serial.print(F(" (CH")); Serial.print(ch); Serial.println(F("): INIT FAILED"));
    return false;
  }
  s.setMeasurementTimingBudget(TIMING_BUDGET_US);
  s.startContinuous();
  Serial.print(name); Serial.print(F(" (CH")); Serial.print(ch); Serial.println(F("): READY"));
  return true;
}

// Non-blocking read: returns true and the range only when a new sample is
// ready (the same registers readRangeContinuousMillimeters() polls).
bool tofPoll(VL53L0X &s, uint8_t ch, uint16_t &mm) {
  tcaselect(ch);
  if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;
  mm = s.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  s.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  return true;
}

struct Stats {
  uint16_t last = 0, lo = 65535, hi = 0;
  uint32_t n = 0, nWindow = 0, farCount = 0, tooClose = 0;
  double   sum = 0, sum2 = 0;
  unsigned long lastSampleMs = 0;
  void reset() { lo = 65535; hi = 0; n = 0; sum = sum2 = 0; farCount = tooClose = 0; }
  void add(uint16_t v) {
    last = v; nWindow++; lastSampleMs = millis();
    if (v >= 8190) { farCount++; return; }        // no target
    if (v < 20) tooClose++;
    n++; sum += v; sum2 += (double)v * v;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
};
Stats stF, stR;
char  mode = 'a';

void printStats(const __FlashStringHelper *name, Stats &s, bool ok, float hz) {
  Serial.print(name);
  if (!ok) { Serial.print(F(" ---- (no sensor)            ")); return; }
  if (millis() - s.lastSampleMs > 250) { Serial.print(F(" ! TIMEOUT                   ")); return; }
  if (s.last >= 8190) Serial.print(F(" FAR "));
  else { Serial.print(' '); Serial.print(s.last); Serial.print(F(" mm")); }
  if (s.n) {
    double avg = s.sum / s.n, sd = sqrt(fmax(0.0, s.sum2 / s.n - avg * avg));
    Serial.print(F("  [min ")); Serial.print(s.lo);
    Serial.print(F(" max ")); Serial.print(s.hi);
    Serial.print(F(" avg ")); Serial.print(avg, 1);
    Serial.print(F(" sd ")); Serial.print(sd, 1); Serial.print(']');
  }
  if (s.farCount) { Serial.print(F(" ! 8190+ x")); Serial.print(s.farCount); }
  if (s.tooClose) { Serial.print(F(" ! <20 x")); Serial.print(s.tooClose); }
  Serial.print(F("  ")); Serial.print(hz, 0); Serial.print(F(" Hz"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  resetTCA();
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  Serial.println(F("\n--- ToF PAIR TEST (front CH3, rear CH4) ---"));
  okFront = initTof(tofFront, FRONT_CH, F("front"));
  okRear  = initTof(tofRear,  REAR_CH,  F("rear "));
  Serial.println(F("commands: r = reset stats, f / b / a = front / rear / both\n"));
}

unsigned long lastPrint = 0;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') { stF.reset(); stR.reset(); Serial.println(F("-- stats reset --")); }
    if (c == 'f' || c == 'b' || c == 'a') mode = c;
  }
  uint16_t mm;
  if (okFront && mode != 'b' && tofPoll(tofFront, FRONT_CH, mm)) stF.add(mm);
  if (okRear  && mode != 'f' && tofPoll(tofRear,  REAR_CH,  mm)) stR.add(mm);

  unsigned long now = millis();
  if (now - lastPrint >= 200) {
    float dt = (now - lastPrint) / 1000.0f;
    float hzF = stF.nWindow / dt, hzR = stR.nWindow / dt;
    stF.nWindow = stR.nWindow = 0;
    lastPrint = now;
    if (mode != 'b') printStats(F("F"), stF, okFront, hzF);
    if (mode == 'a') Serial.print(F("  |  "));
    if (mode != 'f') printStats(F("R"), stR, okRear, hzR);
    Serial.println();
  }
}
