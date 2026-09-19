/* ============================================================
 * TRUE-STRAIGHT FINDER v2  -  BlackPill F411CE + BNO08x
 *
 * RUNS ON ITS OWN AT POWER-UP. No serial needed to start.
 * Connect the serial monitor after the car stops moving and
 * press 'z' to dump the results.
 *
 * PER ANGLE:
 *   set servo -> record yaw  (h0)
 *   drive forward legCm      -> record yaw (h1)
 *   DRIFT = h1 - h0                       <-- the measurement
 *   drive backward legCm to return to the start position
 *   (return yaw logged too, as a cross-check only)
 *
 * True straight = the angle where DRIFT crosses zero. A
 * least-squares fit across the sweep locates that crossing
 * between the 0.5 deg steps.
 *
 * PINS
 *   IMU SPI1 : SCK PA5 | MISO PA6 | MOSI PA7 | CS PA4 | INT PB0 | RST PB1
 *   Encoder  : ENC_A PA0, ENC_B PA1   (TIM5 AF2 - keeps TIM2 free for PWM)
 *   Motor    : RPWM PA2, LPWM PA3     (TIM2 via analogWrite)
 *   Servo    : PA8
 *
 * ENCODER SIGN: forward counts NEGATIVE on this car, so
 * ENC_SIGN flips it. readEnc() is positive-forward everywhere.
 *
 * SERIAL (115200, send newline)
 *   z              diagnostics + 30 cm out-and-back at the best angle
 *   s              re-run the sweep
 *   a <angle>      single test at one angle
 *   r <lo> <hi> <step>   sweep range   (default 80 100 0.5)
 *   d <cm>         leg distance        (default 40)
 *   p <pwm>        drive PWM           (default 120)
 *   n <count>      repeats per angle   (default 1)
 *   x              stop   |   h  help
 * ============================================================ */

#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// ------------------------------------------------------------
// PINS
// ------------------------------------------------------------
const int IMU_SCK = PA5, IMU_MISO = PA6, IMU_MOSI = PA7;
const int IMU_CS  = PA4, IMU_INT  = PB0, IMU_RST  = PB1;

const int RPWM_PIN   = PA2;
const int LPWM_PIN   = PA3;
const int DRV_EN_PIN = -1;         // set to a pin if your driver needs enable
const int SERVO_PIN  = PA8;

// ------------------------------------------------------------
// CALIBRATION
// ------------------------------------------------------------
const float TICKS_PER_CM = 31.933;
const int   ENC_SIGN     = -1;     // forward counts negative -> flip it
const float SERVO_GUARD_MIN = 60.0;
const float SERVO_GUARD_MAX = 120.0;

const unsigned long BOOT_DELAY_MS = 6000;   // time to place the car and step back

// ------------------------------------------------------------
// RUNTIME TUNABLES
// ------------------------------------------------------------
float sweepLo = 80.0, sweepHi = 100.0, sweepStep = 0.5;
float legCm   = 40.0;
int   testPwm = 120;
int   repeats = 1;
const float VERIFY_CM = 30.0;

// ------------------------------------------------------------
// RESULTS
// ------------------------------------------------------------
const int MAX_PTS = 64;
struct Pt {
  float angle;
  float drift;      // h1 - h0 over the forward leg  <-- the metric
  float ret;        // yaw change coming back (cross-check)
  float net;        // yaw at start position after the round trip
  float fwdCm, revCm;
  bool  valid;
};
Pt  pts[MAX_PTS];
int nPts = 0;

float bestAngle = 90, fitAngle = 90;
bool  haveBest = false, haveFit = false;
bool  sweepDone = false;

// ------------------------------------------------------------
SPIClass SPI_IMU(IMU_MOSI, IMU_MISO, IMU_SCK);
BNO08x   myIMU;
Servo    steeringServo;

bool  imuOK = false;
float yawOffset = 0.0;
float gHeading  = 0.0;
static TIM_HandleTypeDef htim5;

// print only when a host is actually attached, so an unopened
// USB CDC port can never stall the sweep
inline bool ser() { return (bool)Serial; }

// ============================================================
float wrapDeg(float a) {
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

void setMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { analogWrite(RPWM_PIN, speed); analogWrite(LPWM_PIN, 0); }
  else if (speed < 0) { analogWrite(RPWM_PIN, 0);     analogWrite(LPWM_PIN, -speed); }
  else                { analogWrite(RPWM_PIN, 0);     analogWrite(LPWM_PIN, 0); }
}

void setServo(float deg) {
  deg = constrain(deg, SERVO_GUARD_MIN, SERVO_GUARD_MAX);
  steeringServo.writeMicroseconds((int)((deg / 180.0) * 1000.0) + 1000);
}

// positive = forward, regardless of how the encoder is wired
long readEnc()    { return (long)(ENC_SIGN * (int16_t)TIM5->CNT); }
void zeroEnc()    { TIM5->CNT = 0; }
long absL(long v) { return v < 0 ? -v : v; }

// ============================================================
// IMU
// ============================================================
float readYawRaw() {
  float qI = myIMU.getQuatI(), qJ = myIMU.getQuatJ();
  float qK = myIMU.getQuatK(), qR = myIMU.getQuatReal();
  if (qI == 0 && qJ == 0 && qK == 0 && qR == 0) return 0;
  return atan2(2.0f * (qI * qJ + qR * qK),
               (qR * qR + qI * qI - qJ * qJ - qK * qK)) * (180.0 / PI);
}

void serviceIMU() {
  if (myIMU.wasReset()) myIMU.enableGameRotationVector();
  if (myIMU.getSensorEvent() &&
      myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    gHeading = fmod(readYawRaw() - yawOffset + 540.0, 360.0) - 180.0;
  }
}

float settleHeading(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) serviceIMU();
  return gHeading;
}

bool initIMU() {
  SPI_IMU.begin();
  uint32_t speeds[3] = {3000000, 1000000, 500000};
  for (int i = 0; i < 3; i++) {
    pinMode(IMU_RST, OUTPUT);
    digitalWrite(IMU_RST, LOW);  delay(20);
    digitalWrite(IMU_RST, HIGH); delay(150);
    if (myIMU.beginSPI(IMU_CS, IMU_INT, IMU_RST, speeds[i], SPI_IMU)) {
      delay(500);
      myIMU.enableGameRotationVector();
      delay(100);
      myIMU.getSensorEvent();
      unsigned long t0 = millis();
      while (millis() - t0 < 3000) {
        if (myIMU.wasReset()) myIMU.enableGameRotationVector();
        if (myIMU.getSensorEvent() &&
            myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
          yawOffset = readYawRaw();
          if (ser()) { Serial.print(F("[IMU] up at ")); Serial.print(speeds[i] / 1000);
                       Serial.println(F(" kHz, zeroed")); }
          return true;
        }
        delay(10);
      }
      return false;
    }
  }
  return false;
}

// ============================================================
// ENCODER - TIM5 on PA0/PA1 (AF2)
// ============================================================
void initEncoder() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &g);

  TIM_Encoder_InitTypeDef e = {0};
  htim5.Instance         = TIM5;
  htim5.Init.Prescaler   = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period      = 65535;
  e.EncoderMode  = TIM_ENCODERMODE_TI12;
  e.IC1Polarity  = TIM_ICPOLARITY_RISING;
  e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  e.IC2Polarity  = TIM_ICPOLARITY_RISING;
  e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim5, &e);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

// ============================================================
// ONE LEG. dir +1 forward / -1 back. false = aborted.
// ============================================================
bool driveLeg(float cm, int dir, float &endHeading, float &actualCm) {
  long target = (long)(cm * TICKS_PER_CM);
  long start  = readEnc();

  unsigned long t0 = millis();
  unsigned long timeout = (unsigned long)(cm * 200.0) + 4000;
  long lastSeen = 0;
  unsigned long lastMove = millis();

  setMotor(dir * testPwm);

  bool ok = true;
  while (true) {
    serviceIMU();
    long d = absL(readEnc() - start);
    if (d >= target) break;

    if (d != lastSeen) { lastSeen = d; lastMove = millis(); }

    if (Serial.available())          { Serial.read(); ok = false; break; }
    if (millis() - t0 > timeout)     { ok = false; if (ser()) Serial.println(F("  ABORT timeout")); break; }
    if (millis() - lastMove > 1500)  { ok = false; if (ser()) Serial.println(F("  ABORT stall")); break; }
  }

  setMotor(0);
  actualCm   = absL(readEnc() - start) / TICKS_PER_CM;
  endHeading = settleHeading(450);        // let the chassis stop rocking
  return ok;
}

// ============================================================
// ONE ANGLE
//   h0 = yaw with the servo set, before moving
//   forward -> h1 ;  DRIFT = h1 - h0
//   backward to the same spot -> h2 (cross-check only)
// ============================================================
bool testAngle(float angle, Pt &out) {
  out.angle = angle;
  out.valid = false;

  setServo(angle);
  delay(400);                             // servo + linkage settle

  float dSum = 0, rSum = 0, nSum = 0, fc = 0, rc = 0;

  for (int r = 0; r < repeats; r++) {
    float h0 = settleHeading(350);        // yaw BEFORE the forward leg
    float h1, h2;

    if (!driveLeg(legCm, +1, h1, fc)) { setMotor(0); return false; }
    float drift = wrapDeg(h1 - h0);       // <-- the measurement

    if (!driveLeg(legCm, -1, h2, rc)) { setMotor(0); return false; }

    dSum += drift;
    rSum += wrapDeg(h2 - h1);
    nSum += wrapDeg(h2 - h0);
  }

  out.drift = dSum / repeats;
  out.ret   = rSum / repeats;
  out.net   = nSum / repeats;
  out.fwdCm = fc;  out.revCm = rc;
  out.valid = true;

  if (ser()) {
    Serial.print(F("  ")); Serial.print(angle, 1);
    Serial.print(F(" deg -> drift ")); Serial.print(out.drift, 2);
    Serial.print(F("  (ret ")); Serial.print(out.ret, 2);
    Serial.print(F(", net ")); Serial.print(out.net, 2); Serial.println(F(")"));
  }
  return true;
}

// ============================================================
// ANALYSIS - fit drift vs angle, find the zero crossing
// ============================================================
void analyse() {
  if (nPts < 2) return;

  float best = 1e9;
  for (int i = 0; i < nPts; i++) {
    if (!pts[i].valid) continue;
    if (fabs(pts[i].drift) < best) { best = fabs(pts[i].drift); bestAngle = pts[i].angle; haveBest = true; }
  }

  float sx = 0, sy = 0, sxx = 0, sxy = 0; int n = 0;
  for (int i = 0; i < nPts; i++) {
    if (!pts[i].valid) continue;
    sx += pts[i].angle;  sy += pts[i].drift;
    sxx += pts[i].angle * pts[i].angle;
    sxy += pts[i].angle * pts[i].drift;
    n++;
  }
  float den = n * sxx - sx * sx;
  haveFit = false;
  if (n >= 3 && fabs(den) > 1e-6) {
    float m = (n * sxy - sx * sy) / den;
    float c = (sy - m * sx) / n;
    if (fabs(m) > 1e-6) {
      fitAngle = -c / m;
      if (fitAngle > sweepLo - 2 && fitAngle < sweepHi + 2) haveFit = true;
    }
  }
}

// ============================================================
void runSweep() {
  nPts = 0; haveBest = false; haveFit = false; sweepDone = false;

  int steps = (int)((sweepHi - sweepLo) / sweepStep + 0.5) + 1;
  if (steps > MAX_PTS) {
    if (ser()) { Serial.print(F("too many steps: ")); Serial.println(steps); }
    return;
  }

  if (ser()) {
    Serial.print(F("\n=== SWEEP "));   Serial.print(sweepLo, 1);
    Serial.print(F(".."));             Serial.print(sweepHi, 1);
    Serial.print(F(" step "));         Serial.print(sweepStep, 2);
    Serial.print(F(" | leg "));        Serial.print(legCm, 0);
    Serial.print(F("cm | pwm "));      Serial.print(testPwm);
    Serial.println(F(" ==="));
  }

  for (int i = 0; i < steps; i++) {
    float a = sweepLo + i * sweepStep;
    if (a > sweepHi + 0.001) break;
    if (!testAngle(a, pts[nPts])) {
      if (ser()) Serial.println(F("=== ABORTED ==="));
      setMotor(0);
      analyse();
      return;
    }
    nPts++;
  }

  setMotor(0);
  analyse();
  sweepDone = true;
  setServo(haveFit ? fitAngle : bestAngle);

  if (ser()) Serial.println(F("=== SWEEP DONE - press z ==="));
}

// ============================================================
void printDiagnostics() {
  Serial.println(F("\n============== DIAGNOSTICS =============="));
  if (nPts == 0) { Serial.println(F("no data")); return; }

  Serial.print(F("points: ")); Serial.print(nPts);
  Serial.print(F(" | leg ")); Serial.print(legCm, 0);
  Serial.print(F("cm | pwm ")); Serial.print(testPwm);
  Serial.print(F(" | reps ")); Serial.println(repeats);
  Serial.println();
  Serial.println(F("angle, drift_deg, return_deg, net_deg, fwd_cm, rev_cm"));

  for (int i = 0; i < nPts; i++) {
    Serial.print(pts[i].angle, 1);  Serial.print(F(", "));
    Serial.print(pts[i].drift, 2);  Serial.print(F(", "));
    Serial.print(pts[i].ret, 2);    Serial.print(F(", "));
    Serial.print(pts[i].net, 2);    Serial.print(F(", "));
    Serial.print(pts[i].fwdCm, 1);  Serial.print(F(", "));
    Serial.println(pts[i].revCm, 1);
  }

  Serial.println();
  if (haveBest) { Serial.print(F("min |drift| at : ")); Serial.print(bestAngle, 2);
                  Serial.print(F(" deg  (")); Serial.print(fabs(pts[0].drift) , 2); Serial.println(F(" deg shown in table)")); }
  if (haveFit)  { Serial.print(F("FITTED STRAIGHT: ")); Serial.println(fitAngle, 3);
                  Serial.println(F("  -> use as SERVO_TRUE_STRAIGHT")); }
  else          { Serial.println(F("fit failed - drift never crosses zero in this range."));
                  Serial.println(F("  re-range with r <lo> <hi> <step> so it brackets zero.")); }
  Serial.println(F("=========================================")); 
}

void runVerify() {
  float use = haveFit ? fitAngle : bestAngle;
  Serial.print(F("\n=== VERIFY ")); Serial.print(use, 2);
  Serial.print(F(" deg, ")); Serial.print(VERIFY_CM, 0);
  Serial.println(F(" cm out and back ==="));
  Serial.println(F("starting in 3 s - clear the path"));
  delay(3000);

  setServo(use);
  delay(500);

  float h0 = settleHeading(400);
  float h1, h2, c1, c2;

  if (!driveLeg(VERIFY_CM, +1, h1, c1)) { setMotor(0); return; }
  float drift = wrapDeg(h1 - h0);
  Serial.print(F("  forward ")); Serial.print(c1, 1);
  Serial.print(F(" cm -> DRIFT ")); Serial.print(drift, 3); Serial.println(F(" deg"));

  if (!driveLeg(VERIFY_CM, -1, h2, c2)) { setMotor(0); return; }
  Serial.print(F("  back    ")); Serial.print(c2, 1);
  Serial.print(F(" cm -> yaw ")); Serial.print(wrapDeg(h2 - h1), 3); Serial.println(F(" deg"));
  Serial.print(F("  net after round trip: ")); Serial.println(wrapDeg(h2 - h0), 3);

  Serial.print(F("  -> "));
  if (fabs(drift) < 0.5)      Serial.println(F("GOOD. This is true straight."));
  else if (fabs(drift) < 1.5) Serial.println(F("close - nudge with 'a' and retest."));
  else                        Serial.println(F("still biased - re-sweep narrower around here."));
  Serial.println(F("======================================"));
}

// ============================================================
// SERIAL
// ============================================================
char lineBuf[48]; uint8_t lineLen = 0;

void help() {
  Serial.println(F("\nz=results+verify  s=sweep  a<ang>  r<lo><hi><step>  d<cm>  p<pwm>  n<cnt>  x  h"));
  Serial.print(F("range ")); Serial.print(sweepLo, 1); Serial.print(F("-")); Serial.print(sweepHi, 1);
  Serial.print(F(" step ")); Serial.print(sweepStep, 2);
  Serial.print(F(" leg "));  Serial.print(legCm, 0);
  Serial.print(F("cm pwm ")); Serial.print(testPwm);
  Serial.print(F(" reps "));  Serial.println(repeats);
}

void handleLine(char *s) {
  while (*s == ' ') s++;
  char c = *s; char *arg = s + 1;
  switch (c) {
    case 'z': printDiagnostics(); runVerify(); break;
    case 's': runSweep(); break;
    case 'x': setMotor(0); Serial.println(F("stopped")); break;
    case 'h': help(); break;
    case 'a': {
      float a = atof(arg);
      if (a < SERVO_GUARD_MIN || a > SERVO_GUARD_MAX) { Serial.println(F("out of guard range")); break; }
      Pt p; Serial.println(F("[single] starting in 3 s")); delay(3000);
      testAngle(a, p); setMotor(0); break;
    }
    case 'r': {
      float lo = atof(strtok(arg, " "));
      char *t2 = strtok(NULL, " "), *t3 = strtok(NULL, " ");
      float hi = t2 ? atof(t2) : 0, st = t3 ? atof(t3) : 0;
      if (lo >= SERVO_GUARD_MIN && hi <= SERVO_GUARD_MAX && hi > lo && st > 0.05) {
        sweepLo = lo; sweepHi = hi; sweepStep = st; Serial.println(F("ok"));
      } else Serial.println(F("bad range"));
      help(); break;
    }
    case 'd': { float v = atof(arg); if (v >= 10 && v <= 200) legCm = v; help(); break; }
    case 'p': { int v = atoi(arg);   if (v >= 60 && v <= 255) testPwm = v; help(); break; }
    case 'n': { int v = atoi(arg);   if (v >= 1 && v <= 5)    repeats = v; help(); break; }
    default:  if (c) Serial.println(F("? press h"));
  }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineLen) { lineBuf[lineLen] = 0; handleLine(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = ch;
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  if (DRV_EN_PIN >= 0) { pinMode(DRV_EN_PIN, OUTPUT); digitalWrite(DRV_EN_PIN, HIGH); }
  setMotor(0);

  steeringServo.attach(SERVO_PIN, 1000, 2000);
  setServo((sweepLo + sweepHi) * 0.5);

  initEncoder();
  zeroEnc();

  delay(300);
  if (ser()) Serial.println(F("\nTRUE-STRAIGHT FINDER v2 - auto start"));

  imuOK = initIMU();
  if (!imuOK) {
    if (ser()) Serial.println(F("IMU FAILED - not moving"));
    return;
  }

  // countdown so you can set the car down and get clear
  unsigned long t0 = millis();
  int lastSec = -1;
  while (millis() - t0 < BOOT_DELAY_MS) {
    serviceIMU();
    int s = (BOOT_DELAY_MS - (millis() - t0)) / 1000;
    if (ser() && s != lastSec) { lastSec = s; Serial.print(F("starting in ")); Serial.println(s + 1); }
  }

  runSweep();
}

void loop() {
  serviceIMU();
  pollSerial();
}
