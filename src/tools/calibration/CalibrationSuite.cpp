// ============================================================================
// CalibrationSuite - all eight calibrations in one flash
// ============================================================================
// Flash this once and drive the whole calibration session from the serial
// monitor. Type a number, press enter, follow the prompts.
//
// The eight standalone sketches in this folder are the reference versions.
// They are longer, better commented, and print more. Read those to understand
// what a step does. Use this one on competition day when you do not want to
// reflash six times in a pit with no table.
//
//   1  encoder ticks per wheel revolution
//   2  ticks per centimetre
//   3  steering jerk
//   4  true straight servo angle
//   5  ToF floor signal threshold
//   6  ninety degree turns
//   7  heading correction gain
//   8  floor colour thresholds
//   0  print the current constants and exit to the menu
//
// RUN THEM IN ORDER. Each step depends on the ones before it:
//   2 needs 1. 4 needs 2. 6 needs 4. 7 needs 6.
// Steps 3, 5 and 8 are independent and can be done any time.
//
// SAFETY: steps 2, 3, 4, 6 and 7 drive the motor. Put the car on the floor
// with a clear run in front of it, not on a bench where it will drive off
// the edge. Every one of them waits for the start button first.
// ============================================================================

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <VL53L1X.h>
#include "Adafruit_TCS34725.h"
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "hardware_config.h"

BNO08x  myIMU;
Servo   steer;
VL53L1X tofA, tofB;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS,
                                          TCS34725_GAIN_4X);
bool imuOk = false, tofAok = false, tofBok = false, tcsOk = false;

// ------------------------------------------------------------- helpers ----
float readYawDeg() {
  if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    float qi = myIMU.getQuatI(), qj = myIMU.getQuatJ();
    float qk = myIMU.getQuatK(), qr = myIMU.getQuatReal();
    return atan2(2.0f * (qi * qj + qr * qk),
                 qr * qr + qi * qi - qj * qj - qk * qk) * 57.2957795f;
  }
  return NAN;
}
float freshYaw() {
  float y = NAN; unsigned long t = millis();
  while (isnan(y) && millis() - t < 250) y = readYawDeg();
  return y;
}
float wrap180(float a) { return fmod(a + 540.0f, 360.0f) - 180.0f; }
bool  abortRequested() { return Serial.available() && Serial.read() == 'q'; }
void  stopAll() { setMotorSpeed(0); steer.write(SERVO_TRUE_STRAIGHT); }

// ------------------------------------------------------------- step 1 -----
void step1_ticksPerRev() {
  Serial.println("\n[1] ticks per wheel revolution. Button to zero, turn one");
  Serial.println("    revolution by hand, button again. 'q' + enter to stop.");
  Stats s;
  while (!abortRequested()) {
    waitForButton("    ready");
    zeroEncoder();
    waitForButton("    turned? press start");
    long c = readEncoder();
    if (c <= 0) { Serial.println("    rejected"); continue; }
    s.add(c);
    s.print("    running", " ticks/rev");
  }
  s.print("  FINAL", " ticks/rev");
}

// ------------------------------------------------------------- step 2 -----
void step2_ticksPerCm() {
  Serial.println("\n[2] ticks per cm. Button rolls, button stops, then type");
  Serial.println("    the measured cm. 'q' + enter to stop.");
  double sx=0, sy=0, sxx=0, sxy=0; long n=0;
  while (!abortRequested()) {
    waitForButton("    on the line");
    zeroEncoder(); setMotorSpeed(110);
    pinMode(START_BTN_PIN, INPUT_PULLUP);
    while (digitalRead(START_BTN_PIN) == HIGH) delay(2);
    setMotorSpeed(0);
    while (digitalRead(START_BTN_PIN) == LOW) delay(5);
    long c = readEncoder();
    Serial.print("    ticks="); Serial.print(c); Serial.print("  cm > ");
    while (!Serial.available()) delay(10);
    double cm = Serial.parseFloat();
    while (Serial.available()) Serial.read();
    if (cm <= 0.5) { Serial.println("rejected"); continue; }
    Serial.println(cm, 2);
    sx+=cm; sy+=c; sxx+=cm*cm; sxy+=cm*c; n++;
    if (n >= 2) {
      double slope = (n*sxy - sx*sy) / (n*sxx - sx*sx);
      Serial.print("    TICKS_PER_CM = "); Serial.println(slope, 4);
    }
  }
}

// ------------------------------------------------------------- step 3 -----
void step3_jerk(float slewLimit) {
  Serial.print("\n[3] steering jerk, slew limit "); Serial.println(slewLimit, 2);
  if (!imuOk) { Serial.println("    IMU missing, cannot run"); return; }
  waitForButton("    clear floor?");
  float servoNow = SERVO_TRUE_STRAIGHT, target = SERVO_TRUE_STRAIGHT + 25.0;
  steer.write(servoNow); setMotorSpeed(120); delay(800);

  float prevRate=0, prevAcc=0; bool haveR=false, haveA=false;
  double peak=0; unsigned long t0=millis(), lastUs=micros();
  while (millis() - t0 < 2500) {
    while (micros() - lastUs < 5000) { }
    float dt = (micros() - lastUs) / 1e6f; lastUs = micros();
    if (slewLimit <= 0) servoNow = target;
    else servoNow += constrain(target - servoNow, -slewLimit, slewLimit);
    steer.write(servoNow);
    if (!(myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED)) continue;
    float rate = myIMU.getGyroZ() * 57.2957795f;
    if (haveR) {
      float acc = (rate - prevRate) / dt;
      if (haveA) { float j = (acc - prevAcc) / dt; if (fabs(j) > peak) peak = fabs(j); }
      prevAcc = acc; haveA = true;
    }
    prevRate = rate; haveR = true;
  }
  stopAll();
  Serial.print("    peak |jerk| = "); Serial.print(peak, 0); Serial.println(" deg/s3");
}

// ------------------------------------------------------------- step 4 -----
void step4_trueStraight(float centre, float stepDeg, int each, int passes) {
  Serial.println("\n[4] true straight servo angle");
  if (!imuOk) { Serial.println("    IMU missing, cannot run"); return; }
  const int N = 2*each + 1;
  double sumAbs[21] = {0}; int runs[21] = {0};
  for (int p = 0; p < passes; p++) {
    for (int i = 0; i < N && i < 21; i++) {
      float a = centre + (i - each) * stepDeg;
      Serial.print("    angle "); Serial.println(a, 1);
      waitForButton("    on the line");
      steer.write(a); delay(300);
      float y0 = freshYaw(); zeroEncoder(); setMotorSpeed(120);
      long tgt = (long)(150.0 * TICKS_PER_CM);
      while (labs(readEncoder()) < tgt) readYawDeg();
      setMotorSpeed(0); delay(400);
      float y1 = freshYaw();
      if (isnan(y0) || isnan(y1)) { Serial.println("    dropout"); continue; }
      float d = fabs(wrap180(y1 - y0));
      sumAbs[i] += d; runs[i]++;
      Serial.print("    drift "); Serial.println(d, 2);
    }
  }
  int best = 0; double bestv = 1e9;
  for (int i = 0; i < N && i < 21; i++) {
    double m = runs[i] ? sumAbs[i]/runs[i] : 1e9;
    Serial.print("    "); Serial.print(centre + (i-each)*stepDeg, 1);
    Serial.print(" -> "); Serial.println(m, 3);
    if (m < bestv) { bestv = m; best = i; }
  }
  Serial.print("    SERVO_TRUE_STRAIGHT = ");
  Serial.println(centre + (best-each)*stepDeg, 1);
}

// ------------------------------------------------------------- step 5 -----
void step5_tofSignal() {
  Serial.println("\n[5] ToF signal logging. CSV below. 'q' + enter to stop.");
  Serial.println("    ms,sensor,range_mm,signal_mcps,ambient_mcps,status");
  while (!abortRequested()) {
    for (int k = 0; k < 2; k++) {
      VL53L1X &s = k ? tofB : tofA;
      if (!(k ? tofBok : tofAok)) continue;
      uint8_t ch = k ? CH_TOF_B : CH_TOF_A;
      tcaselect(ch);
      if (!s.dataReady()) continue;
      s.read(false);
      Serial.print(millis()); Serial.print(",");
      Serial.print(k ? "B" : "A"); Serial.print(",");
      Serial.print(s.ranging_data.range_mm); Serial.print(",");
      Serial.print(s.ranging_data.peak_signal_count_rate_MCPS, 3); Serial.print(",");
      Serial.print(s.ranging_data.ambient_count_rate_MCPS, 3); Serial.print(",");
      Serial.println(VL53L1X::rangeStatusToString(s.ranging_data.range_status));
    }
    delay(20);
  }
}

// ------------------------------------------------------------- step 6 -----
void step6_turn90() {
  Serial.println("\n[6] ninety degree turns, alternating. 'q' + enter to stop.");
  if (!imuOk) { Serial.println("    IMU missing, cannot run"); return; }
  const float KP=2.5, MAXS=55.0, MINS=8.0, KV=3.5, STOP=0.3;
  const int MAXP=130, MINP=100;
  Stats eL, eR; bool right = true;
  while (!abortRequested()) {
    waitForButton(right ? "    RIGHT - press start" : "    LEFT - press start");
    steer.write(SERVO_TRUE_STRAIGHT); setMotorSpeed(110); delay(400);
    float start = freshYaw();
    if (isnan(start)) { stopAll(); continue; }
    float target = wrap180(start + (right ? -90.0f : 90.0f));
    unsigned long t0 = millis();
    while (millis() - t0 < 4000) {
      float y = readYawDeg(); if (isnan(y)) continue;
      float err = wrap180(target - y);
      if (fabs(err) < STOP) break;
      float mag = fabs(err);
      float sv = constrain(KP*mag, MINS, MAXS);
      int pwm = (int)constrain(KV*mag, (float)MINP, (float)MAXP);
      steer.write(SERVO_TRUE_STRAIGHT + (err > 0 ? sv : -sv));
      setMotorSpeed(pwm);
    }
    stopAll(); delay(500);
    float fe = wrap180(target - freshYaw());
    Serial.print(right ? "    RIGHT err " : "    LEFT  err "); Serial.println(fe, 2);
    if (right) eR.add(fe); else eL.add(fe);
    right = !right;
  }
  eR.print("  RIGHT", " deg"); eL.print("  LEFT ", " deg");
}

// ------------------------------------------------------------- step 7 -----
void step7_headingGain(float kp) {
  Serial.print("\n[7] heading hold, KP = "); Serial.println(kp, 2);
  if (!imuOk) { Serial.println("    IMU missing, cannot run"); return; }
  waitForButton("    long straight? press start");
  float target = freshYaw(); if (isnan(target)) return;
  zeroEncoder(); setMotorSpeed(130);
  float servoNow = SERVO_TRUE_STRAIGHT, filt = 0, peak = 0, prev = 0;
  double sumSq = 0; long n = 0; int cross = 0;
  unsigned long lastUs = micros();
  long tgt = (long)(300.0 * TICKS_PER_CM);
  while (labs(readEncoder()) < tgt) {
    while (micros() - lastUs < 5000) { }
    lastUs = micros();
    float y = readYawDeg(); if (isnan(y)) continue;
    float err = wrap180(target - y);
    filt += 0.35f * (err - filt);
    float want = constrain(SERVO_TRUE_STRAIGHT + kp*filt, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
    servoNow += constrain(want - servoNow, -2.5f, 2.5f);
    steer.write(servoNow);
    sumSq += (double)err*err; n++;
    if (fabs(err) > peak) peak = fabs(err);
    if ((prev < 0 && err > 0) || (prev > 0 && err < 0)) cross++;
    prev = err;
  }
  stopAll();
  Serial.print("    rms "); Serial.print(n ? sqrt(sumSq/n) : 0.0, 3);
  Serial.print(" deg  peak "); Serial.print(peak, 2);
  Serial.print(" deg  crossings "); Serial.println(cross);
}

// ------------------------------------------------------------- step 8 -----
void step8_colour() {
  Serial.println("\n[8] floor colour. Button per surface. 'q' + enter to stop.");
  if (!tcsOk) { Serial.println("    TCS34725 missing, cannot run"); return; }
  while (!abortRequested()) {
    waitForButton("    over the surface? press start");
    Stats pr, pg, pb;
    for (int i = 0; i < 200; i++) {
      uint16_t r,g,b,c; tcaselect(CH_COLOUR); tcs.getRawData(&r,&g,&b,&c);
      float t = (float)r+g+b; if (t <= 0) continue;
      pr.add(100.0*r/t); pg.add(100.0*g/t); pb.add(100.0*b/t);
      delay(5);
    }
    pr.print("    %R", " %"); pg.print("    %G", " %"); pb.print("    %B", " %");
  }
}

// ---------------------------------------------------------------- menu ----
void printConstants() {
  Serial.println("\n--- constants currently compiled in ---");
  Serial.print("  TICKS_PER_CM        "); Serial.println(TICKS_PER_CM, 4);
  Serial.print("  SERVO_TRUE_STRAIGHT "); Serial.println(SERVO_TRUE_STRAIGHT, 1);
  Serial.print("  SERVO_MAX_LEFT      "); Serial.println(SERVO_MAX_LEFT, 1);
  Serial.print("  SERVO_MAX_RIGHT     "); Serial.println(SERVO_MAX_RIGHT, 1);
  Serial.print("  SIGNAL_MIN_MCPS     "); Serial.println(SIGNAL_MIN_MCPS, 2);
  Serial.print("  TOF_MAX_VALID_MM    "); Serial.println(TOF_MAX_VALID_MM);
  Serial.println("  floor colour: BLUE %B>36 %R<24 | ORANGE %R>35 %B<27");
}

void printMenu() {
  Serial.println();
  Serial.println("================ CALIBRATION SUITE ================");
  Serial.println("  1  encoder ticks per revolution");
  Serial.println("  2  ticks per centimetre");
  Serial.println("  3  steering jerk");
  Serial.println("  4  true straight servo angle");
  Serial.println("  5  ToF floor signal threshold");
  Serial.println("  6  ninety degree turns");
  Serial.println("  7  heading correction gain");
  Serial.println("  8  floor colour thresholds");
  Serial.println("  0  print current constants");
  Serial.println("  run them IN ORDER the first time.");
  Serial.print  ("  choice > ");
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000);
  Wire.begin(); Wire.setClock(400000); SPI.begin();
  encoderBegin(); motorBegin(); setMotorSpeed(0);
  steer.attach(SERVO_PIN); steer.write(SERVO_TRUE_STRAIGHT);
  pinMode(START_BTN_PIN, INPUT_PULLUP);

  Serial.println("\nbringing up sensors...");
  imuOk = myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN);
  if (imuOk) { myIMU.enableRotationVector(5000); myIMU.enableGyro(5000); }
  Serial.print("  BNO085     "); Serial.println(imuOk ? "ok" : "MISSING");

  tcaselect(CH_TOF_A); tofA.setTimeout(200);
  tofAok = tofA.init();
  if (tofAok) { tofA.setDistanceMode(VL53L1X::Long);
                tofA.setMeasurementTimingBudget(50000); tofA.startContinuous(50); }
  Serial.print("  ToF ch"); Serial.print(CH_TOF_A); Serial.print("     ");
  Serial.println(tofAok ? "ok" : "MISSING");

  tcaselect(CH_TOF_B); tofB.setTimeout(200);
  tofBok = tofB.init();
  if (tofBok) { tofB.setDistanceMode(VL53L1X::Long);
                tofB.setMeasurementTimingBudget(50000); tofB.startContinuous(50); }
  Serial.print("  ToF ch"); Serial.print(CH_TOF_B); Serial.print("     ");
  Serial.println(tofBok ? "ok" : "MISSING");

  tcaselect(CH_COLOUR);
  tcsOk = tcs.begin();
  Serial.print("  TCS34725   "); Serial.println(tcsOk ? "ok" : "MISSING");
  printConstants();
}

void loop() {
  printMenu();
  while (!Serial.available()) delay(20);
  int c = Serial.parseInt();
  while (Serial.available()) Serial.read();
  Serial.println(c);
  switch (c) {
    case 1: step1_ticksPerRev(); break;
    case 2: step2_ticksPerCm(); break;
    case 3: step3_jerk(2.5); break;
    case 4: step4_trueStraight(SERVO_TRUE_STRAIGHT, 1.0, 4, 3); break;
    case 5: step5_tofSignal(); break;
    case 6: step6_turn90(); break;
    case 7: step7_headingGain(2.0); break;
    case 8: step8_colour(); break;
    case 0: printConstants(); break;
    default: Serial.println("  not a step"); break;
  }
  stopAll();
}
