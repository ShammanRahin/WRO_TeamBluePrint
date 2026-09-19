/* ============================================================
 * TRUE-STRAIGHT FINDER  -  BlackPill F411CE + BNO08x
 *
 * Sweeps the steering servo, drives a fixed distance forward
 * then back at each angle, and measures the yaw change on each
 * leg. Reversing flips the sign of a steering error but NOT of
 * gyro drift, so the two legs separate cleanly:
 *
 *     steer  = (fwdDrift - revDrift) / 2   <- real steering bias
 *     bias   = (fwdDrift + revDrift) / 2   <- gyro drift + asymmetry
 *
 * True straight is where "steer" crosses zero. A least-squares
 * fit across the sweep finds that crossing to better than the
 * 0.5 deg step size.
 *
 * PINS
 *   IMU SPI1 : SCK PA5 | MISO PA6 | MOSI PA7 | CS PA4 | INT PB0 | RST PB1
 *   Encoder  : ENC_A PA0, ENC_B PA1   (TIM5, AF2)
 *   Motor    : RPWM PA2, LPWM PA3     (TIM2 via analogWrite)
 *   Servo    : PA8
 *
 * SERIAL COMMANDS (115200, send newline)
 *   s              start the sweep
 *   z              print diagnostics, then 30 cm out-and-back at best angle
 *   a <angle>      single test at one angle
 *   r <lo> <hi> <step>   set sweep range      (default 80 100 0.5)
 *   d <cm>         set leg distance           (default 40)
 *   p <pwm>        set drive PWM              (default 120)
 *   n <count>      repeats per angle          (default 1)
 *   x              abort / stop motors
 *   h              help
 *
 * Any keypress during motion aborts immediately.
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

const int RPWM_PIN  = PA2;
const int LPWM_PIN  = PA3;
const int DRV_EN_PIN = -1;      // set to a pin if your driver needs an enable

const int SERVO_PIN = PA8;

// ------------------------------------------------------------
// CALIBRATION
// ------------------------------------------------------------
const float TICKS_PER_CM = 31.933;
const float SERVO_GUARD_MIN = 60.0;    // refuse to command outside this
const float SERVO_GUARD_MAX = 120.0;

// ------------------------------------------------------------
// TUNABLE AT RUNTIME
// ------------------------------------------------------------
float sweepLo   = 80.0;
float sweepHi   = 100.0;
float sweepStep = 0.5;
float legCm     = 40.0;
int   testPwm   = 120;
int   repeats   = 1;

const float VERIFY_CM = 30.0;

// ------------------------------------------------------------
// RESULTS
// ------------------------------------------------------------
const int MAX_PTS = 64;
struct Pt {
  float angle;
  float fwd, rev;      // per-leg yaw change, deg
  float steer, bias;   // decomposed
  bool  valid;
};
Pt pts[MAX_PTS];
int nPts = 0;

float bestAngle = 0;
bool  haveBest  = false;
float fitAngle  = 0;
bool  haveFit   = false;

// ------------------------------------------------------------
// OBJECTS
// ------------------------------------------------------------
SPIClass SPI_IMU(IMU_MOSI, IMU_MISO, IMU_SCK);
BNO08x   myIMU;
Servo    steeringServo;

bool  imuOK = false;
float yawOffset = 0.0;
float gHeading  = 0.0;
bool  gFresh    = false;

static TIM_HandleTypeDef htim5;
volatile bool abortFlag = false;

// ============================================================
// BASIC HELPERS
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

void zeroEnc()        { TIM5->CNT = 0; }
long readEnc()        { return (int16_t)TIM5->CNT; }
long absL(long v)     { return v < 0 ? -v : v; }

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
  gFresh = false;
  if (myIMU.wasReset()) {
    Serial.println(F("!! IMU reset - reports re-enabled"));
    myIMU.enableGameRotationVector();
  }
  if (myIMU.getSensorEvent() &&
      myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    gHeading = fmod(readYawRaw() - yawOffset + 540.0, 360.0) - 180.0;
    gFresh = true;
  }
}

// settle in place, keep the IMU fed, return a stable heading
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
      Serial.print(F("[IMU] up at ")); Serial.print(speeds[i] / 1000);
      Serial.println(F(" kHz"));
      delay(500);
      myIMU.enableGameRotationVector();
      delay(100);
      myIMU.getSensorEvent();
      // capture the zero
      unsigned long t0 = millis();
      while (millis() - t0 < 3000) {
        if (myIMU.wasReset()) myIMU.enableGameRotationVector();
        if (myIMU.getSensorEvent() &&
            myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
          yawOffset = readYawRaw();
          Serial.print(F("[IMU] zeroed at ")); Serial.println(yawOffset, 2);
          return true;
        }
        delay(10);
      }
      Serial.println(F("[IMU] no event to zero"));
      return false;
    }
  }
  return false;
}

// ============================================================
// ENCODER on TIM5 (PA0/PA1, AF2) - keeps TIM2 free for PWM
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
  htim5.Init.Period      = 65535;          // wrap at 16 bits so the cast works
  e.EncoderMode  = TIM_ENCODERMODE_TI12;
  e.IC1Polarity  = TIM_ICPOLARITY_RISING;
  e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  e.IC2Polarity  = TIM_ICPOLARITY_RISING;
  e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim5, &e);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

// ============================================================
// ONE LEG OF TRAVEL
// dir = +1 forward, -1 reverse. Returns false if aborted.
// ============================================================
bool driveLeg(float cm, int dir, float &dHeading, float &actualCm) {
  long target = (long)(cm * TICKS_PER_CM);
  long start  = readEnc();

  float h0 = settleHeading(350);

  unsigned long t0 = millis();
  unsigned long timeout = (unsigned long)(cm * 200.0) + 4000;   // generous
  long lastTicks = start;
  unsigned long lastMove = millis();

  setMotor(dir * testPwm);

  bool ok = true;
  while (true) {
    serviceIMU();

    long d = absL(readEnc() - start);
    if (d >= target) break;

    if (d != absL(lastTicks - start)) { lastTicks = readEnc(); lastMove = millis(); }

    if (Serial.available()) { Serial.read(); ok = false;
      Serial.println(F("  ABORT: key pressed")); break; }
    if (millis() - t0 > timeout) { ok = false;
      Serial.println(F("  ABORT: timeout")); break; }
    if (millis() - lastMove > 1500) { ok = false;
      Serial.println(F("  ABORT: encoder stalled")); break; }
  }

  setMotor(0);
  actualCm = absL(readEnc() - start) / TICKS_PER_CM;

  float h1 = settleHeading(450);      // let the chassis stop rocking
  dHeading = wrapDeg(h1 - h0);
  return ok;
}

// ============================================================
// ONE ANGLE: out and back
// ============================================================
bool testAngle(float angle, Pt &out) {
  out.angle = angle;
  out.valid = false;

  setServo(angle);
  delay(400);                          // servo and linkage settle

  float fwdSum = 0, revSum = 0;
  for (int r = 0; r < repeats; r++) {
    float dF, dR, cF, cR;
    if (!driveLeg(legCm, +1, dF, cF)) { setMotor(0); return false; }
    if (!driveLeg(legCm, -1, dR, cR)) { setMotor(0); return false; }
    fwdSum += dF;
    revSum += dR;
  }

  out.fwd = fwdSum / repeats;
  out.rev = revSum / repeats;
  out.steer = (out.fwd - out.rev) * 0.5;
  out.bias  = (out.fwd + out.rev) * 0.5;
  out.valid = true;

  Serial.print(F("  angle ")); Serial.print(angle, 1);
  Serial.print(F("  fwd ")); Serial.print(out.fwd, 2);
  Serial.print(F("  rev ")); Serial.print(out.rev, 2);
  Serial.print(F("  steer ")); Serial.print(out.steer, 2);
  Serial.print(F("  bias ")); Serial.println(out.bias, 2);
  return true;
}

// ============================================================
// SWEEP
// ============================================================
void runSweep() {
  nPts = 0; haveBest = false; haveFit = false;

  int steps = (int)((sweepHi - sweepLo) / sweepStep + 0.5) + 1;
  if (steps > MAX_PTS) {
    Serial.print(F("Too many steps (")); Serial.print(steps);
    Serial.print(F("), max ")); Serial.println(MAX_PTS);
    return;
  }

  Serial.println(F("\n=== SWEEP START ==="));
  Serial.print(F("range ")); Serial.print(sweepLo, 1);
  Serial.print(F(" .. "));   Serial.print(sweepHi, 1);
  Serial.print(F(" step "));  Serial.print(sweepStep, 2);
  Serial.print(F(" | leg ")); Serial.print(legCm, 0);
  Serial.print(F("cm | pwm ")); Serial.print(testPwm);
  Serial.print(F(" | repeats ")); Serial.println(repeats);
  Serial.println(F("press any key to abort\n"));

  for (int i = 0; i < steps; i++) {
    float a = sweepLo + i * sweepStep;
    if (a > sweepHi + 0.001) break;

    Serial.print(F("[")); Serial.print(i + 1);
    Serial.print(F("/"));  Serial.print(steps); Serial.print(F("]"));

    if (!testAngle(a, pts[nPts])) {
      Serial.println(F("=== SWEEP ABORTED ==="));
      setServo((sweepLo + sweepHi) * 0.5);
      return;
    }
    nPts++;
  }

  setServo((sweepLo + sweepHi) * 0.5);
  setMotor(0);
  Serial.println(F("=== SWEEP DONE ===  press z for results"));
  analyse();
}

// ============================================================
// ANALYSIS: min |steer| plus a least-squares zero crossing
// ============================================================
void analyse() {
  if (nPts < 2) return;

  // smallest absolute steering component
  float best = 1e9;
  for (int i = 0; i < nPts; i++) {
    if (!pts[i].valid) continue;
    if (fabs(pts[i].steer) < best) { best = fabs(pts[i].steer); bestAngle = pts[i].angle; haveBest = true; }
  }

  // linear fit steer = m*angle + c, solve for steer = 0
  float sx = 0, sy = 0, sxx = 0, sxy = 0; int n = 0;
  for (int i = 0; i < nPts; i++) {
    if (!pts[i].valid) continue;
    sx += pts[i].angle; sy += pts[i].steer;
    sxx += pts[i].angle * pts[i].angle;
    sxy += pts[i].angle * pts[i].steer;
    n++;
  }
  float denom = n * sxx - sx * sx;
  if (n >= 3 && fabs(denom) > 1e-6) {
    float m = (n * sxy - sx * sy) / denom;
    float c = (sy - m * sx) / n;
    if (fabs(m) > 1e-6) {
      fitAngle = -c / m;
      if (fitAngle > sweepLo - 2 && fitAngle < sweepHi + 2) haveFit = true;
    }
  }
}

void printDiagnostics() {
  Serial.println(F("\n================ DIAGNOSTICS ================"));
  if (nPts == 0) { Serial.println(F("no data - run 's' first")); return; }

  Serial.println(F("angle, fwd_deg, rev_deg, steer_deg, bias_deg"));
  for (int i = 0; i < nPts; i++) {
    Serial.print(pts[i].angle, 1);  Serial.print(F(", "));
    Serial.print(pts[i].fwd, 2);    Serial.print(F(", "));
    Serial.print(pts[i].rev, 2);    Serial.print(F(", "));
    Serial.print(pts[i].steer, 2);  Serial.print(F(", "));
    Serial.println(pts[i].bias, 2);
  }

  Serial.println();
  if (haveBest) {
    Serial.print(F("best sampled angle : ")); Serial.println(bestAngle, 2);
  }
  if (haveFit) {
    Serial.print(F("fitted zero cross  : ")); Serial.println(fitAngle, 3);
    Serial.println(F("  -> use this as SERVO_TRUE_STRAIGHT"));
  } else {
    Serial.println(F("fit failed - steer values may be noisy or all one sign."));
    Serial.println(F("  widen the range so it brackets zero, or raise 'n'."));
  }

  // average bias tells you about gyro drift over one leg
  float bsum = 0; int bn = 0;
  for (int i = 0; i < nPts; i++) if (pts[i].valid) { bsum += pts[i].bias; bn++; }
  if (bn) {
    Serial.print(F("mean bias term     : ")); Serial.print(bsum / bn, 3);
    Serial.println(F(" deg/leg  (gyro drift + chassis asymmetry)"));
  }
  Serial.println(F("============================================="));
}

// ============================================================
// VERIFY: 30 cm out and back at the chosen angle
// ============================================================
void runVerify() {
  float use = haveFit ? fitAngle : (haveBest ? bestAngle : (sweepLo + sweepHi) * 0.5);

  Serial.print(F("\n=== VERIFY at ")); Serial.print(use, 2);
  Serial.print(F(" deg, ")); Serial.print(VERIFY_CM, 0);
  Serial.println(F(" cm out and back ==="));

  setServo(use);
  delay(500);

  float dF, dR, cF, cR;
  if (!driveLeg(VERIFY_CM, +1, dF, cF)) { setMotor(0); return; }
  Serial.print(F("  forward : ")); Serial.print(cF, 1);
  Serial.print(F(" cm, yaw ")); Serial.print(dF, 2); Serial.println(F(" deg"));

  if (!driveLeg(VERIFY_CM, -1, dR, cR)) { setMotor(0); return; }
  Serial.print(F("  reverse : ")); Serial.print(cR, 1);
  Serial.print(F(" cm, yaw ")); Serial.print(dR, 2); Serial.println(F(" deg"));

  float steer = (dF - dR) * 0.5;
  float bias  = (dF + dR) * 0.5;
  Serial.print(F("  steer component : ")); Serial.println(steer, 3);
  Serial.print(F("  bias  component : ")); Serial.println(bias, 3);
  Serial.print(F("  net heading err : ")); Serial.println(dF + dR, 3);

  Serial.print(F("  -> "));
  if (fabs(steer) < 0.5)      Serial.println(F("GOOD. This is your true straight."));
  else if (fabs(steer) < 1.5) Serial.println(F("close. Nudge and re-verify with 'a'."));
  else                        Serial.println(F("still biased. Re-sweep a narrower range around here."));
  Serial.println(F("======================================"));
}

// ============================================================
// SERIAL COMMAND PARSER
// ============================================================
char lineBuf[48];
uint8_t lineLen = 0;

void help() {
  Serial.println(F("\ns | z | a <ang> | r <lo> <hi> <step> | d <cm> | p <pwm> | n <cnt> | x | h"));
  Serial.print(F("now: range ")); Serial.print(sweepLo, 1);
  Serial.print(F("-"));           Serial.print(sweepHi, 1);
  Serial.print(F(" step "));      Serial.print(sweepStep, 2);
  Serial.print(F(" leg "));       Serial.print(legCm, 0);
  Serial.print(F("cm pwm "));     Serial.print(testPwm);
  Serial.print(F(" reps "));      Serial.println(repeats);
}

void handleLine(char *s) {
  while (*s == ' ') s++;
  char c = *s;
  char *arg = s + 1;

  switch (c) {
    case 's': runSweep(); break;
    case 'z': printDiagnostics(); runVerify(); break;
    case 'x': setMotor(0); Serial.println(F("stopped")); break;
    case 'h': help(); break;

    case 'a': {
      float a = atof(arg);
      if (a < SERVO_GUARD_MIN || a > SERVO_GUARD_MAX) { Serial.println(F("angle out of guard range")); break; }
      Pt p;
      Serial.print(F("[single]"));
      testAngle(a, p);
      setMotor(0);
      break;
    }
    case 'r': {
      float lo = atof(strtok(arg, " "));
      char *t2 = strtok(NULL, " "); char *t3 = strtok(NULL, " ");
      float hi = t2 ? atof(t2) : 0, st = t3 ? atof(t3) : 0;
      if (lo >= SERVO_GUARD_MIN && hi <= SERVO_GUARD_MAX && hi > lo && st > 0.05) {
        sweepLo = lo; sweepHi = hi; sweepStep = st;
        Serial.println(F("range set"));
      } else Serial.println(F("bad range"));
      help();
      break;
    }
    case 'd': { float v = atof(arg); if (v >= 10 && v <= 200) legCm = v; help(); break; }
    case 'p': { int v = atoi(arg);   if (v >= 60 && v <= 255) testPwm = v; help(); break; }
    case 'n': { int v = atoi(arg);   if (v >= 1 && v <= 5)    repeats = v; help(); break; }

    default: if (c) Serial.println(F("? press h"));
  }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (lineLen) { lineBuf[lineLen] = 0; handleLine(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = ch;
    }
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  if (DRV_EN_PIN >= 0) { pinMode(DRV_EN_PIN, OUTPUT); digitalWrite(DRV_EN_PIN, HIGH); }
  setMotor(0);

  steeringServo.attach(SERVO_PIN, 1000, 2000);
  setServo((sweepLo + sweepHi) * 0.5);

  initEncoder();
  zeroEnc();

  Serial.println(F("\n=========================================="));
  Serial.println(F(" TRUE-STRAIGHT FINDER"));
  Serial.println(F("=========================================="));

  imuOK = initIMU();
  if (!imuOK) {
    Serial.println(F("IMU FAILED - cannot measure drift. Fix that first."));
    return;
  }

  // encoder sanity: push the car by hand
  Serial.println(F("\nPush the car forward by hand a few cm..."));
  long e0 = readEnc();
  unsigned long t0 = millis();
  while (millis() - t0 < 4000) {
    serviceIMU();
    if (absL(readEnc() - e0) > (long)(2 * TICKS_PER_CM)) {
      Serial.print(F("encoder OK, sign = "));
      Serial.println((readEnc() - e0) > 0 ? F("POSITIVE forward") : F("NEGATIVE forward"));
      break;
    }
  }
  if (absL(readEnc() - e0) <= (long)(2 * TICKS_PER_CM))
    Serial.println(F("no encoder movement seen - check ENC_A/ENC_B on PA0/PA1"));

  zeroEnc();
  help();
  Serial.println(F("\nClear a straight runway, then press 's'."));
}

void loop() {
  serviceIMU();
  pollSerial();
}
