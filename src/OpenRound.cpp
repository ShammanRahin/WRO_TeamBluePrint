#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>
#include <VL53L1X.h>          // Pololu VL53L1X ToF library

// ============================================================
// STRATEGY  (read this first)
//
// Sensor map from your scan:  CH1 = LEFT, CH3 = RIGHT, CH4 = FRONT.
//
// This robot follows ONE wall - the inner island - at a fixed distance,
// and turns each time that wall gives way at a corner.
//
// INIT
//   Sample both side ToFs and zero the yaw. The tracked SIDE is decided
//   later by the first colour (orange->right, blue->left); the start
//   sample just supplies the distance to hold (lockedRefMM).
//
// WHY THE INNER (convex) WALL, NOT THE OUTER (concave) ONE:
//   The island corner is convex - it pokes toward the car - so at the
//   corner the side beam shoots PAST it into open space and the reading
//   goes invalid. The outer perimeter corner is concave - it wraps
//   around the car - so that wall never leaves the beam and never reads
//   invalid. Only the inner wall's bend fires a turn.
//
// TURN TRIGGER  (all three, every turn, debounced)
//   1. front > FRONT_CLEAR_MM             (clearance ahead)
//   2. the colour-chosen side ToF invalid (tracked wall gave way)
//   3. a colour line seen  (arms/latches; also locks direction on turn 1)
//
// AFTER EACH TURN  (STATE_APPROACH, before the IMU realign)
//   Drive until the locked side reads lockedRefMM again. While doing so,
//   if heading strays past HEADING_GUARD_DEG the car realigns heading
//   FIRST (a skewed car makes the side ToF read a long diagonal), then
//   resumes closing on the distance. Finish with a clean IMU realign.
//
// STRAIGHTS are held on the IMU alone (smooth). ToF signal filter:
// samples with return strength < SIGNAL_MIN_MCPS are discarded.
//
// FINAL DISTANCE
//   Record the distance from the start to the FIRST turn. The final
//   straight after the 12th turn is TRACK_SIDE_CM minus that, so the
//   car finishes back in the start section.
// ============================================================

#define TCA_ADDR 0x70

enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };

enum RobotState {
  STATE_INIT,
  STATE_DRIVE_TO_CORNER,
  STATE_TURNING,
  STATE_APPROACH,        // drive to locked distance, then realign
  STATE_FINAL_STRAIGHT,
  STATE_FINISHED
};

// ============================================================
// HARDWARE PINS & OBJECTS
// ============================================================
const int RPWM_PIN   = PB9;
const int LPWM_PIN   = PB8;
const int DRV_EN_PIN = PB1;
const int SERVO_PIN  = PA8;

const int IMU_CS_PIN  = PB0;
const int IMU_INT_PIN = PB13;
const int IMU_RST_PIN = PB14;

const float TICKS_PER_CM        = 31.933;
const float SERVO_TRUE_STRAIGHT = 69.0;
const float SERVO_MAX_LEFT      = 5.0;    // lower angle = LEFT
const float SERVO_MAX_RIGHT     = 115.0;  // higher angle = RIGHT
const int   BASE_SPEED          = 120;

SPIClass SPI_IMU(PB5, PB4, PB3);
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

bool    tcsConnected     = false;
uint8_t tcsChannel       = 0;
float   initialYawOffset = 0.0;

// ============================================================
// ToF SENSORS
// ============================================================
const uint8_t CH_LEFT  = 1;
const uint8_t CH_RIGHT = 3;
const uint8_t CH_FRONT = 4;

VL53L1X tofL, tofR, tofF;
bool tofLok = false, tofRok = false, tofFok = false;

uint16_t tofLeft = 9999, tofRight = 9999, tofFront = 9999;
bool tofLeftValid = false, tofRightValid = false, tofFrontValid = false;

const uint16_t TOF_MAX_VALID_MM = 1300;
const float    SIGNAL_MIN_MCPS  = 4.0;    // reject returns weaker than this

// ============================================================
// LOCKED-WALL FOLLOWING
//
// The tracked side is chosen by the FIRST colour, not by closeness:
//   ORANGE -> clockwise  -> island on the RIGHT -> watch RIGHT ToF
//   BLUE   -> CCW        -> island on the LEFT  -> watch LEFT  ToF
// i.e. lockedIsLeft = !clockwiseMode, set when the first colour reads.
// The reference distance is that side's distance measured at start.
// ============================================================
bool  lockedIsLeft = true;    // set from the first colour
float lockedRefMM  = 300.0;   // distance to hold from the tracked wall

// side wall distances sampled once at start (supply lockedRefMM)
float leftStartMM = 0.0, rightStartMM = 0.0;
bool  leftStartValid = false, rightStartValid = false;

const uint16_t FRONT_CLEAR_MM = 600;   // condition 1: clearance ahead
const int      TURN_CONFIRM_HITS = 3;  // debounce for the combined trigger

// approach-to-distance tuning
const float HEADING_GUARD_DEG    = 15.0; // realign if heading strays past this (10-20)
const int   APPROACH_PWM         = 70;
const float APPROACH_KP_DEG_PER_MM = 0.06;
const float APPROACH_MAX_DEG     = 25.0;
const float APPROACH_DEADBAND_MM = 20.0;
const float APPROACH_MAX_CM      = 70.0; // distance cap on the whole manoeuvre
const float APPROACH_KHEAD       = 1.2;  // heading-override gain
const float APPROACH_HEAD_MAX_DEG = 40.0;
const int   APPROACH_LOST_CAP    = 200;  // give up homing if locked wall invalid this long

// ============================================================
// RUN CONSTANTS
// ============================================================
const int   TARGET_CORNERS   = 12;
const float TRACK_SIDE_CM    = 300.0;   // full straight length; final = this - firstSeg
const float SEARCH_SAFETY_CM = 400.0;
const float POST_CORNER_LOCKOUT_CM = 50;
const float REALIGN_SAFETY_CM = 80.0;

// ============================================================
// FSM DATA
// ============================================================
RobotState currentState = STATE_INIT;

BlockColor lockedColor   = COLOR_NONE;
bool       clockwiseMode = true;
int        cornerCount   = 0;

float targetHeading = 0.0;
float laneHeading   = 0.0;

BlockColor lastFirstColor = COLOR_NONE;
float      firstSegmentCm = 0.0;
float      finalDistanceCm = TRACK_SIDE_CM;

// ============================================================
// ANGLE HELPER
// ============================================================
float wrapDeg(float angle) {
  while (angle > 180.0)  angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

// ============================================================
// MULTIPLEXER HELPER
// ============================================================
void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ============================================================
// MOTOR & STEERING
// ============================================================
void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { analogWrite(RPWM_PIN, speed); analogWrite(LPWM_PIN, 0); }
  else if (speed < 0) { analogWrite(RPWM_PIN, 0);     analogWrite(LPWM_PIN, -speed); }
  else                { analogWrite(RPWM_PIN, 0);     analogWrite(LPWM_PIN, 0); }
}

void setServoAngle(float angleDeg) {
  angleDeg = constrain(angleDeg, 5.0, 115.0);
  int pulseWidth = (int)((angleDeg / 180.0) * 1000.0) + 1000;
  steeringServo.writeMicroseconds(pulseWidth);
}

// ============================================================
// ENCODER
// ============================================================
void zeroEncoder() { TIM3->CNT = 0; }
long readEncoder() { return (int16_t)TIM3->CNT; }

// ============================================================
// IMU
// ============================================================
float readYaw() {
  float qI = myIMU.getQuatI(), qJ = myIMU.getQuatJ();
  float qK = myIMU.getQuatK(), qReal = myIMU.getQuatReal();
  if (qI == 0.0f && qJ == 0.0f && qK == 0.0f && qReal == 0.0f) return 0.0f;
  float yawRadians = atan2(2.0f * (qI * qJ + qReal * qK),
                           (qReal * qReal + qI * qI - qJ * qJ - qK * qK));
  return yawRadians * (180.0 / PI);
}

void zeroYaw() {
  Serial.println(F("Waiting for valid IMU data to set Zero..."));
  unsigned long timeout = millis();
  while (millis() - timeout < 3000) {
    if (myIMU.wasReset()) myIMU.enableGameRotationVector();
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      initialYawOffset = readYaw();
      Serial.print(F("Zero Yaw locked at: ")); Serial.println(initialYawOffset);
      return;
    }
    delay(10);
  }
  Serial.println(F("ERROR: Timed out waiting for IMU event!"));
}

float readHeading() {
  float currentYaw = readYaw();
  return fmod(currentYaw - initialYawOffset + 540.0, 360.0) - 180.0;
}

// ============================================================
// COLOUR SENSOR
// ============================================================
void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (tcsConnected) { tcaselect(tcsChannel); tcs.getRawData(&r, &g, &b, &c); }
  else              { r = 0; g = 0; b = 0; c = 0; }
}

BlockColor pendingColor    = COLOR_NONE;
int        consecutiveHits = 0;

void resetColorDetector() { pendingColor = COLOR_NONE; consecutiveHits = 0; }

BlockColor detectColor(BlockColor wantColor) {
  const int HITS_NEEDED = 2;
  uint16_t r, g, b, c;
  readColor(r, g, b, c);

  BlockColor rawColor = COLOR_NONE;
  float totalLight = r + g + b;
  if (totalLight > 0) {
    float pR = ((float)r / totalLight) * 100.0;
    float pB = ((float)b / totalLight) * 100.0;
    if (pB > 36.0 && pR < 24.0)      rawColor = COLOR_BLUE;
    else if (pR > 35.0 && pB < 27.0) rawColor = COLOR_ORANGE;
  }

  if (wantColor != COLOR_NONE && rawColor != wantColor) rawColor = COLOR_NONE;
  if (rawColor == COLOR_NONE) { resetColorDetector(); return COLOR_NONE; }

  if (rawColor == pendingColor) consecutiveHits++;
  else { pendingColor = rawColor; consecutiveHits = 1; }

  if (consecutiveHits >= HITS_NEEDED) { resetColorDetector(); return rawColor; }
  return COLOR_NONE;
}

// ============================================================
// ToF SERVICE (non-blocking, signal-filtered)
// ============================================================
void serviceOneToF(VL53L1X &s, bool present, uint8_t ch,
                   uint16_t &outMM, bool &outValid) {
  if (!present) { outValid = false; return; }
  tcaselect(ch);
  if (s.dataReady()) {
    uint16_t d = s.read(false);
    bool ok = (s.ranging_data.range_status == VL53L1X::RangeValid) &&
              (s.ranging_data.peak_signal_count_rate_MCPS >= SIGNAL_MIN_MCPS) &&
              (d > 0) && (d < TOF_MAX_VALID_MM);
    outValid = ok;
    if (ok) outMM = d;
  }
}

void updateToF() {
  serviceOneToF(tofF, tofFok, CH_FRONT, tofFront, tofFrontValid);
  serviceOneToF(tofL, tofLok, CH_LEFT,  tofLeft,  tofLeftValid);
  serviceOneToF(tofR, tofRok, CH_RIGHT, tofRight, tofRightValid);
}

// Current reading of the locked (tracked) wall.
bool lockedReading(uint16_t &mm) {
  if (lockedIsLeft) { mm = tofLeft;  return tofLeftValid; }
  else              { mm = tofRight; return tofRightValid; }
}

// Trigger condition helpers.
bool frontClearNow() { return (!tofFrontValid) || (tofFront > FRONT_CLEAR_MM); }

// Is the given side's wall currently invalid (gave way)?
bool sideInvalid(bool isLeft) { return isLeft ? (!tofLeftValid) : (!tofRightValid); }

// ============================================================
// INIT
// ============================================================
void initHardware() {
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);
  digitalWrite(DRV_EN_PIN, HIGH);
  setMotorSpeed(0);

  steeringServo.attach(SERVO_PIN, 1000, 2000);
  setServoAngle(SERVO_TRUE_STRAIGHT);

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  TIM_Encoder_InitTypeDef sConfig = {0};
  static TIM_HandleTypeDef htim3 = {0};
  htim3.Instance         = TIM3;
  htim3.Init.Prescaler   = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period      = 65535;
  sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i); delay(10);
    if (tcs.begin()) { tcsConnected = true; tcsChannel = i;
      Serial.print(F("TCS34725 on TCA channel ")); Serial.println(i); break; }
  }
  if (!tcsConnected) Serial.println(F("WARNING: TCS34725 not detected!"));

  tcaselect(CH_LEFT);
  tofL.setBus(&Wire); tofL.setTimeout(100);
  tofLok = tofL.init();
  if (tofLok) { tofL.setDistanceMode(VL53L1X::Short); tofL.setMeasurementTimingBudget(33000); tofL.startContinuous(33); }
  Serial.println(tofLok ? F("ToF LEFT  (CH1) ready") : F("ToF LEFT  (CH1) FAILED"));

  tcaselect(CH_RIGHT);
  tofR.setBus(&Wire); tofR.setTimeout(100);
  tofRok = tofR.init();
  if (tofRok) { tofR.setDistanceMode(VL53L1X::Short); tofR.setMeasurementTimingBudget(33000); tofR.startContinuous(33); }
  Serial.println(tofRok ? F("ToF RIGHT (CH3) ready") : F("ToF RIGHT (CH3) FAILED"));

  tcaselect(CH_FRONT);
  tofF.setBus(&Wire); tofF.setTimeout(100);
  tofFok = tofF.init();
  if (tofFok) { tofF.setDistanceMode(VL53L1X::Short); tofF.setMeasurementTimingBudget(33000); tofF.startContinuous(33); }
  Serial.println(tofFok ? F("ToF FRONT (CH4) ready") : F("ToF FRONT (CH4) FAILED"));

  SPI_IMU.begin();
  if (myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU)) {
    delay(500);
    myIMU.enableGameRotationVector();
    delay(100);
    myIMU.getSensorEvent();
    zeroYaw();
  }
}

// Sample both side walls at rest. The SIDE is not chosen here - that is
// decided by the first colour. This just records each side's start
// distance so the correct one can become lockedRefMM once we know which
// side is the inner wall.
void sampleSideWalls() {
  long sumL = 0, sumR = 0; int nL = 0, nR = 0;
  for (int i = 0; i < 20; i++) {
    updateToF();
    if (tofLeftValid)  { sumL += tofLeft;  nL++; }
    if (tofRightValid) { sumR += tofRight; nR++; }
    delay(10);
  }
  leftStartValid  = (nL > 0);
  rightStartValid = (nR > 0);
  leftStartMM  = nL ? (float)sumL / nL : 0.0;
  rightStartMM = nR ? (float)sumR / nR : 0.0;

  Serial.print(F("[START] L="));
  Serial.print(leftStartValid ? (int)leftStartMM : -1);
  Serial.print(F("mm  R="));
  Serial.print(rightStartValid ? (int)rightStartMM : -1);
  Serial.println(F("mm  (side chosen later by first colour)"));
}

// ============================================================
// HEADING PID (smooth, IMU-only)
// ============================================================
float         pidPrevError = 0.0;
unsigned long pidPrevTime  = 0;

void resetHeadingPid() { pidPrevError = 0.0; pidPrevTime = millis(); }

void updateHeadingPid(float heading) {
  const float localKp = 1.2, localKd = 0.3;
  if (myIMU.wasReset()) myIMU.enableGameRotationVector();
  if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    unsigned long now = millis();
    float dt = (now - pidPrevTime) / 1000.0;
    if (dt <= 0.0) dt = 0.001;
    float error  = wrapDeg(heading - readHeading());
    float dError = (error - pidPrevError) / dt;
    float correction = (localKp * error) + (localKd * dError);
    setServoAngle(SERVO_TRUE_STRAIGHT - correction);
    pidPrevError = error;
    pidPrevTime  = now;
  }
}

// ============================================================
// turnToHeading() - unchanged
// ============================================================
void turnToHeading(float targetGlobalHeading) {
  const int   TURN_PWM       = 90;
  const float STOP_THRESHOLD = 4.0;
  targetGlobalHeading = wrapDeg(targetGlobalHeading);
  float angleNeeded = wrapDeg(targetGlobalHeading - readHeading());

  if (angleNeeded > -STOP_THRESHOLD && angleNeeded < STOP_THRESHOLD) {
    setServoAngle(SERVO_TRUE_STRAIGHT);
    Serial.println(F(">>> Realign not needed."));
    return;
  }
  if (angleNeeded < 0) setServoAngle(SERVO_MAX_RIGHT);
  else                 setServoAngle(SERVO_MAX_LEFT);
  delay(150);

  zeroEncoder();
  long safetyTicks = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
  setMotorSpeed(TURN_PWM);
  while (true) {
    long ticksNow = readEncoder();
    if (ticksNow < 0) ticksNow = -ticksNow;
    if (ticksNow >= safetyTicks) { Serial.println(F(">>> WARN: realign cap.")); break; }
    if (myIMU.wasReset()) myIMU.enableGameRotationVector();
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      float errorRemaining = wrapDeg(targetGlobalHeading - readHeading());
      if (angleNeeded < 0 && errorRemaining >= -STOP_THRESHOLD) break;
      if (angleNeeded > 0 && errorRemaining <=  STOP_THRESHOLD) break;
    }
    delay(2);
  }
  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(300);
}

// Drive fixed distance holding heading (IMU-only).
void goStraight(float cm, float heading) {
  zeroEncoder();
  resetHeadingPid();
  long targetTicks = (long)(cm * TICKS_PER_CM);
  setMotorSpeed(BASE_SPEED);
  while (true) {
    long ticksNow = readEncoder();
    if (ticksNow < 0) ticksNow = -ticksNow;
    if (ticksNow >= targetTicks) break;
    updateHeadingPid(heading);
    delay(2);
  }
  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100);
}

// ============================================================
// turnDegrees() - 90 deg on IMU heading (unchanged mechanism)
// ============================================================
void turnDegrees(float degree, float fromHeading) {
  const int TURN_PWM = 90;
  float finalTargetHeading = wrapDeg(fromHeading - degree);
  if (degree > 0) setServoAngle(SERVO_MAX_RIGHT);
  else            setServoAngle(SERVO_MAX_LEFT);
  delay(150);
  setMotorSpeed(TURN_PWM);
  while (true) {
    if (myIMU.wasReset()) myIMU.enableGameRotationVector();
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      float errorRemaining = wrapDeg(finalTargetHeading - readHeading());
      if (degree > 0 && errorRemaining >= -2.0) break;
      if (degree < 0 && errorRemaining <=  2.0) break;
    }
    delay(2);
  }
  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100);
  Serial.print(F(">>> TURN COMPLETE. Heading: ")); Serial.println(readHeading());
}

// ============================================================
// APPROACH LOCKED DISTANCE  (runs after each turn, before final realign)
//
// Close the locked-side wall distance onto lockedRefMM. Every loop:
//   - if heading has strayed past HEADING_GUARD_DEG, OVERRIDE steering
//     to pull heading back first (a skewed car reads a long diagonal on
//     the side ToF, which would corrupt the homing);
//   - otherwise steer proportionally to (current - reference) toward or
//     away from the wall.
// Ends when the wall is within APPROACH_DEADBAND_MM, or a distance cap,
// then a clean IMU realign onto laneHeadingRef.
// ============================================================
void approachLockedDistance(float laneHeadingRef) {
  zeroEncoder();
  setMotorSpeed(APPROACH_PWM);
  int lostHits = 0;
  float sideSign = lockedIsLeft ? -1.0 : 1.0;   // left wall -> steer down(=left) to close

  while (true) {
    long ticksNow = readEncoder();
    if (ticksNow < 0) ticksNow = -ticksNow;
    if (ticksNow >= (long)(APPROACH_MAX_CM * TICKS_PER_CM)) break;

    updateToF();
    if (myIMU.wasReset()) myIMU.enableGameRotationVector();

    float hErr = wrapDeg(laneHeadingRef - readHeading());

    if (fabs(hErr) > HEADING_GUARD_DEG) {
      // realign heading first
      float corr = APPROACH_KHEAD * hErr;
      if (corr >  APPROACH_HEAD_MAX_DEG) corr =  APPROACH_HEAD_MAX_DEG;
      if (corr < -APPROACH_HEAD_MAX_DEG) corr = -APPROACH_HEAD_MAX_DEG;
      setServoAngle(SERVO_TRUE_STRAIGHT - corr);
    } else {
      uint16_t m; bool v = lockedReading(m);
      if (!v) {
        lostHits++;
        setServoAngle(SERVO_TRUE_STRAIGHT);
        if (lostHits > APPROACH_LOST_CAP) break;   // wall gone; stop homing
      } else {
        lostHits = 0;
        float err = (float)m - lockedRefMM;         // + = too far from wall
        if (fabs(err) < APPROACH_DEADBAND_MM) break; // reached target distance
        float steer = APPROACH_KP_DEG_PER_MM * fabs(err);
        if (steer > APPROACH_MAX_DEG) steer = APPROACH_MAX_DEG;
        float dir = (err > 0) ? 1.0 : -1.0;
        setServoAngle(SERVO_TRUE_STRAIGHT + sideSign * dir * steer);
      }
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100);

  // Clean IMU realign onto the lane heading; the straight then runs on IMU.
  turnToHeading(laneHeadingRef);
}

// ============================================================
// DRIVE TO CORNER  (IMU-only steering + 3-condition trigger)
//
// Straight is held on the IMU. ToF is read only to build the trigger.
// A turn fires when ALL THREE hold (debounced TURN_CONFIRM_HITS):
//   1. front > FRONT_CLEAR_MM
//   2. the colour-chosen side ToF is invalid (inner wall gave way)
//   3. a colour line has been seen this corner (armed/latched)
//
// The colour is a LATCH: a corner line is only under the sensor for a
// few ms, so once seen we hold "armed" and let front + side-invalid
// complete the trigger. On the first turn the armed colour also locks
// lap direction and therefore which side is watched.
//
// On corner 1 it records the start->first-turn distance.
// ============================================================
bool driveToCorner(float heading) {
  resetColorDetector();
  resetHeadingPid();
  zeroEncoder();
  setMotorSpeed(BASE_SPEED);

  bool firstTurn   = (lockedColor == COLOR_NONE);
  bool colorArmed  = false;
  bool curSideLeft = lockedIsLeft;   // known for turns 2..12; set on arm for turn 1

  long lockoutTicks = 0;
  if (cornerCount > 0) lockoutTicks = (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM);
  long safetyTicks = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);

  int confirmHits = 0;
  bool cornerHit = false;

  while (true) {
    long ticksNow = readEncoder();
    if (ticksNow < 0) ticksNow = -ticksNow;
    if (ticksNow >= safetyTicks) break;

    updateToF();
    updateHeadingPid(heading);   // smooth IMU steering (only steering)

    // Condition 3: arm on colour. Turn 1 accepts either colour and
    // uses it to lock direction + which side to watch; later turns look
    // for the already-locked colour.
    if (!colorArmed) {
      BlockColor c = detectColor(firstTurn ? COLOR_NONE : lockedColor);
      if (c != COLOR_NONE) {
        colorArmed = true;
        if (firstTurn) {
          lastFirstColor = c;
          curSideLeft = (c == COLOR_BLUE);   // orange->right, blue->left
        }
        Serial.print(F("[TRIG] colour armed: "));
        Serial.println(c == COLOR_ORANGE ? F("ORANGE") : F("BLUE"));
      }
    }

    if (ticksNow > lockoutTicks && colorArmed) {
      // Conditions 1 and 2.
      bool cond = frontClearNow() && sideInvalid(curSideLeft);
      if (cond) confirmHits++; else confirmHits = 0;

      if (confirmHits >= TURN_CONFIRM_HITS) {
        cornerHit = true;
        if (firstTurn) {
          firstSegmentCm  = (float)ticksNow / TICKS_PER_CM;
          finalDistanceCm = TRACK_SIDE_CM - firstSegmentCm;
          if (finalDistanceCm < 0) finalDistanceCm = 0;
          Serial.print(F("[DIST] start->turn1 = ")); Serial.print(firstSegmentCm);
          Serial.print(F(" cm  final = ")); Serial.print(finalDistanceCm); Serial.println(F(" cm"));
        }
        break;
      }
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  if (!cornerHit) return false;

  Serial.print(F("CORNER  front="));
  Serial.print(tofFrontValid ? tofFront : 9999);
  Serial.print(F("  side=")); Serial.print(curSideLeft ? F("LEFT") : F("RIGHT"));
  Serial.println(F(" invalid"));
  return true;
}

// ============================================================
// MAIN
// ============================================================
void setup() {
  Serial.begin(115200);
  initHardware();
  delay(1000);
  sampleSideWalls();     // record both side start distances (side chosen by colour)
  delay(1000);
}

void loop() {
  switch (currentState) {

    case STATE_INIT: {
      Serial.println(F("[FSM] INIT"));
      lockedColor   = COLOR_NONE;
      cornerCount   = 0;
      laneHeading   = readHeading();
      targetHeading = laneHeading;
      currentState  = STATE_DRIVE_TO_CORNER;
      break;
    }

    case STATE_DRIVE_TO_CORNER: {
      Serial.println(F("[FSM] DRIVE_TO_CORNER"));
      if (driveToCorner(targetHeading)) {
        if (lockedColor == COLOR_NONE) {
          lockedColor   = lastFirstColor;
          clockwiseMode = (lockedColor == COLOR_ORANGE);
          lockedIsLeft  = (lockedColor == COLOR_BLUE);   // orange->right, blue->left

          // reference distance = the tracked side's start reading
          if (lockedIsLeft) lockedRefMM = leftStartValid  ? leftStartMM  : 300.0;
          else              lockedRefMM = rightStartValid ? rightStartMM : 300.0;

          Serial.print(clockwiseMode
            ? F("[FSM] LOCKED ORANGE -> CW, watch RIGHT ToF")
            : F("[FSM] LOCKED BLUE -> CCW, watch LEFT ToF"));
          Serial.print(F("  refMM=")); Serial.println(lockedRefMM);
        }
        currentState = STATE_TURNING;
      } else {
        Serial.println(F("[FSM] WARN: no corner within safety distance, retrying"));
      }
      break;
    }

    case STATE_TURNING: {
      cornerCount++;
      Serial.print(F("[FSM] TURNING - corner "));
      Serial.print(cornerCount); Serial.print(F(" / ")); Serial.println(TARGET_CORNERS);

      float turnAmount = clockwiseMode ? 90.0 : -90.0;
      turnDegrees(turnAmount, laneHeading);
      laneHeading = wrapDeg(laneHeading - turnAmount);

      Serial.print(F("  laneHeading=")); Serial.print(laneHeading);
      Serial.print(F("  IMU=")); Serial.println(readHeading());

      if (cornerCount >= TARGET_CORNERS) currentState = STATE_FINAL_STRAIGHT;
      else                               currentState = STATE_APPROACH;
      break;
    }

    case STATE_APPROACH: {
      approachLockedDistance(laneHeading);   // go to locked distance, then realign
      targetHeading = laneHeading;           // straight held on IMU
      currentState  = STATE_DRIVE_TO_CORNER;
      break;
    }

    case STATE_FINAL_STRAIGHT: {
      Serial.print(F("[FSM] FINAL_STRAIGHT ")); Serial.print(finalDistanceCm); Serial.println(F("cm"));
      goStraight(finalDistanceCm, laneHeading);
      currentState = STATE_FINISHED;
      break;
    }

    case STATE_FINISHED: {
      Serial.println(F("[FSM] FINISHED"));
      setMotorSpeed(0);
      setServoAngle(SERVO_TRUE_STRAIGHT);
      while (true) { delay(1000); }
      break;
    }
  }
}
