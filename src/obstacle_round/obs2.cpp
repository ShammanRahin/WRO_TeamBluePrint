#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>
#include <VL53L1X.h>

// ============================================================
// OPEN ROUND CORE + PILLAR AVOIDANCE
//
// The open-round FSM is UNCHANGED. Everything it did, it still does:
// drive -> colour gate -> turn -> lane correct -> straight -> final.
//
// Added on top, and nothing else:
//
//   STATE_AVOID   Camera (Pi, "V,<colour>,<dx>,<area>") sees a pillar.
//                 RED   -> steer RIGHT slowly until the pillar sits at an
//                          offset in the frame (dx pushed to the far side)
//                 GREEN -> steer LEFT  slowly, same idea
//                 then realign onto laneHeading and drop straight back
//                 into the open-round FSM where it left off.
//
//   TURNING IS TOP PRIORITY. The corner conditions are evaluated in DRIVE
//   *and* in AVOID. If they fire, the pillar maneuver is abandoned on the
//   spot and the car turns. A pillar can never hold off a corner.
//
//   The colour gate also keeps running during AVOID, so a corner line
//   crossed while swerving is still latched - otherwise the swerve would
//   eat the line that decides the turn direction.
// ============================================================

#define TCA_ADDR 0x70
#define PiSerial Serial          // Pi link IS USART1 on this board

enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };

enum RobotState {
  STATE_INIT,
  STATE_WAIT_START,              // <-- added
  STATE_DRIVE_TO_CORNER,
  STATE_TURNING,
  STATE_LANE_CORRECT,
  STATE_AVOID,                   // <-- added
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

const int START_BTN_PIN = PA5;        // active-low: PA5 <-> GND, press = LOW

const int IMU_CS_PIN  = PB0;
const int IMU_INT_PIN = PB13;
const int IMU_RST_PIN = PB14;

const float TICKS_PER_CM        = 31.933;
const float SERVO_TRUE_STRAIGHT = 69.0;
const float SERVO_MAX_LEFT      = 5.0;
const float SERVO_MAX_RIGHT     = 115.0;
const int   BASE_SPEED          = 150;

SPIClass SPI_IMU(PB5, PB4, PB3);
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

bool    tcsConnected     = false;
uint8_t tcsChannel       = 0;
float   initialYawOffset = 0.0;

// ToF (VL53L1X) on the mux:  CH1 = LEFT, CH3 = RIGHT, CH4 = FRONT
const uint8_t CH_LEFT  = 1;
const uint8_t CH_RIGHT = 3;
const uint8_t CH_FRONT = 4;
VL53L1X  tofL, tofR, tofF;
bool     tofLok = false, tofRok = false, tofFok = false;
uint16_t tofLeft = 9999, tofRight = 9999, tofFront = 9999;
bool     tofLeftValid = false, tofRightValid = false, tofFrontValid = false;
const uint16_t TOF_MAX_VALID_MM = 1300;
const float    SIGNAL_MIN_MCPS  = 4.0;
const uint16_t FRONT_TURN_MM    = 700;
const unsigned long SIDE_INVALID_MS = 40;   // watched-side invalid this long = wall gave way

// ============================================================
// RUN CONSTANTS
// ============================================================
const int   TARGET_CORNERS    = 12;
const int   FINAL_STRAIGHT_CM = 100;
const float SEARCH_SAFETY_CM  = 400.0;

float firstSegmentCm      = 0.0;
float fullStartStraightCm = 0.0;
bool  haveFullStraight    = false;
float finalDistanceCm     = FINAL_STRAIGHT_CM;

// ============================================================
// LANE CORRECTION TUNING
// ============================================================
const float GAP_THRESHOLD_CM       = 20.0;
const float GAP_DEADBAND_CM        =  2.0;
const float K_LAT_DEG_PER_CM       =  4;
const float MAX_LAT_OFFSET_DEG     = 30.0;
const float CORRECTION_DISTANCE_CM = 25.0;
const float POST_CORNER_LOCKOUT_CM = 50;
const int   CORRECTION_SIGN        =   1;
const int   CORRECTION_PWM         =   70;
const float REALIGN_SAFETY_CM      = 80.0;

// ============================================================
// PILLAR AVOIDANCE TUNING            (the only new tuning block)
// ============================================================
const int32_t  AVOID_MIN_AREA    = 1600;  // pillar this big in frame -> engage
const int      AVOID_PWM         = 100;   // "slowly"
const int      DX_SIGN           = +1;    // +1 if dx grows to the RIGHT in your frame
                                          //    (flip to -1 if your Pi reports the other way)
// --- the offset the pillar should be driven to, and the PID that does it ---
// RED  -> we move RIGHT, so the pillar must end up at dx = -AVOID_TARGET_DX
// GREEN-> we move LEFT,  so the pillar must end up at dx = +AVOID_TARGET_DX
//   error = targetDx - dx        (green: 200 - dx,  red: -200 - dx)
//   servo = STRAIGHT - Kp*error  (error > 0 wants the pillar further right in
//                                 frame, which means steering LEFT)
const float    AVOID_TARGET_DX   = 200.0; // <-- the offset
const float    AVOID_DX_TOL      = 25.0;  // inside this = "at the offset"
const float    AVOID_KP          = 0.12;  // deg of steer per px of error
const float    AVOID_KD          = 0.02;  // damping on dx rate (px/s)
const float    AVOID_D_ALPHA     = 0.4;   // derivative low-pass
const float    AVOID_MAX_STEER   = 32.0;  // clamp either side of straight
const float    AVOID_SERVO_SLEW  = 3.0;   // deg per loop, keeps it smooth
const float    AVOID_MAX_YAW     = 35.0;  // never lean further than this off lane
const unsigned long AVOID_HOLD_MS = 120;  // must sit at the offset this long
const uint16_t AVOID_SIDE_MIN_MM = 120;   // never crowd the pass-side wall closer than this
const float    AVOID_MAX_OUT_CM  = 45.0;  // hard cap on how far we lean out
const unsigned long AVOID_LOST_MS = 200;  // pillar gone from frame this long = done

// ============================================================
// VISION  (from the Pi over Serial:  "V,<colour>,<dx>,<area>")
// ============================================================
struct Vision { char colour; int dx; long area; unsigned long stamp; };
Vision  vis = {'N', 0, 0, 0};
char    visBuf[48];
uint8_t visLen = 0;

void parseVisionLine(char *s) {
  if (s[0] != 'V' || s[1] != ',') return;
  char c = s[2];
  if (c != 'R' && c != 'G' && c != 'N') return;
  char *p1 = strchr(s + 3, ',');  if (!p1) return;
  char *p2 = strchr(p1 + 1, ','); if (!p2) return;
  vis.colour = c;
  vis.dx     = atoi(p1 + 1);
  vis.area   = atol(p2 + 1);
  vis.stamp  = millis();
}

void serviceVision() {
  while (PiSerial.available()) {
    char ch = PiSerial.read();
    if (ch == '\n' || ch == '\r') {
      if (visLen > 0) { visBuf[visLen] = 0; parseVisionLine(visBuf); visLen = 0; }
    } else if (visLen < sizeof(visBuf) - 1) {
      visBuf[visLen++] = ch;
    }
  }
}

bool visionFresh() { return (millis() - vis.stamp) < 250; }

bool pillarSeen() {              // a real pillar, big enough to act on
  if (!visionFresh()) return false;
  if (vis.colour != 'R' && vis.colour != 'G') return false;
  return vis.area >= AVOID_MIN_AREA;
}

// ============================================================
// FSM DATA
// ============================================================
RobotState currentState = STATE_INIT;
bool          entered   = false;
unsigned long phaseT0   = 0;

BlockColor lockedColor   = COLOR_NONE;
bool       clockwiseMode = true;
int        cornerCount   = 0;

float targetHeading = 0.0;
float laneHeading   = 0.0;

float      lastGapCm        = 0.0;
BlockColor lastFirstColor   = COLOR_NONE;
long       cornerFirstTicks = 0;
bool       gapMeasured      = false;

float gapRefCm  = GAP_THRESHOLD_CM;
bool  gapRefSet = false;

// cached sensor readings, refreshed every loop
bool          gImuFresh = false;
float         gHeading  = 0.0;
float         gYawRate  = 0.0;
float         gPrevH    = 0.0;
unsigned long gPrevHT   = 0;

// ============================================================
// HELPERS
// ============================================================
float wrapDeg(float angle) {
  while (angle > 180.0)  angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { analogWrite(RPWM_PIN, speed); analogWrite(LPWM_PIN, 0); }
  else if (speed < 0) { analogWrite(RPWM_PIN, 0);     analogWrite(LPWM_PIN, -speed); }
  else                { analogWrite(RPWM_PIN, 0);     analogWrite(LPWM_PIN, 0); }
}

void setServoAngle(float angleDeg) {
  angleDeg = constrain(angleDeg, 5.0, 115.0);
  steeringServo.writeMicroseconds((int)((angleDeg / 180.0) * 1000.0) + 1000);
}

void zeroEncoder() { TIM3->CNT = 0; }
long readEncoder() { return (int16_t)TIM3->CNT; }
long absEnc(long v) { return v < 0 ? -v : v; }

// ---- IMU ----
float readYaw() {
  float qI = myIMU.getQuatI(), qJ = myIMU.getQuatJ();
  float qK = myIMU.getQuatK(), qReal = myIMU.getQuatReal();
  if (qI == 0.0f && qJ == 0.0f && qK == 0.0f && qReal == 0.0f) return 0.0f;
  float yawRadians = atan2(2.0f * (qI * qJ + qReal * qK),
                           (qReal * qReal + qI * qI - qJ * qJ - qK * qK));
  return yawRadians * (180.0 / PI);
}
float readHeading() { return fmod(readYaw() - initialYawOffset + 540.0, 360.0) - 180.0; }

void zeroYaw() {   // startup-only blocking zero (car not running yet)
  Serial.println(F("Waiting for valid IMU data to set Zero..."));
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (myIMU.wasReset()) myIMU.enableGameRotationVector();
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      initialYawOffset = readYaw();
      Serial.print(F("Zero Yaw locked at: ")); Serial.println(initialYawOffset);
      return;
    }
    delay(10);
  }
  Serial.println(F("ERROR: no IMU event to zero!"));
}

// ---- colour (direct register read, NO delay) ----
void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (!tcsConnected) { r = 0; g = 0; b = 0; c = 0; return; }
  tcaselect(tcsChannel);
  Wire.beginTransmission(0x29);
  Wire.write(0x80 | 0x20 | 0x14);      // command | auto-increment | CDATAL
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x29, (uint8_t)8);
  if (Wire.available() < 8) { r = 0; g = 0; b = 0; c = 0; return; }
  c  =  (uint16_t)Wire.read();  c |= (uint16_t)Wire.read() << 8;
  r  =  (uint16_t)Wire.read();  r |= (uint16_t)Wire.read() << 8;
  g  =  (uint16_t)Wire.read();  g |= (uint16_t)Wire.read() << 8;
  b  =  (uint16_t)Wire.read();  b |= (uint16_t)Wire.read() << 8;
}

const unsigned long COLOR_CONFIRM_MS = 6;
BlockColor    pendingColor  = COLOR_NONE;
unsigned long pendingStart  = 0;

void resetColorDetector() { pendingColor = COLOR_NONE; pendingStart = 0; }

BlockColor otherColor(BlockColor c) {
  if (c == COLOR_ORANGE) return COLOR_BLUE;
  if (c == COLOR_BLUE)   return COLOR_ORANGE;
  return COLOR_NONE;
}

BlockColor detectColor(BlockColor wantColor) {
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
  if (rawColor != pendingColor) { pendingColor = rawColor; pendingStart = millis(); return COLOR_NONE; }
  if (millis() - pendingStart >= COLOR_CONFIRM_MS) { resetColorDetector(); return rawColor; }
  return COLOR_NONE;
}

// ---- ToF (non-blocking, signal-filtered) ----
void serviceOneToF(VL53L1X &s, bool present, uint8_t ch, uint16_t &mm, bool &valid) {
  if (!present) { valid = false; return; }
  tcaselect(ch);
  if (s.dataReady()) {
    uint16_t d = s.read(false);
    bool ok = (s.ranging_data.range_status == VL53L1X::RangeValid) &&
              (s.ranging_data.peak_signal_count_rate_MCPS >= SIGNAL_MIN_MCPS) &&
              (d > 0) && (d < TOF_MAX_VALID_MM);
    valid = ok;
    if (ok) mm = d;
  }
}

// ---- called once at the top of every loop ----
void serviceSensors() {
  gImuFresh = false;
  if (myIMU.wasReset()) myIMU.enableGameRotationVector();
  if (myIMU.getSensorEvent() &&
      myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    gImuFresh = true;
    float h = readHeading();
    unsigned long now = millis();
    float dt = (now - gPrevHT) / 1000.0;
    if (dt > 0.0) gYawRate = wrapDeg(h - gPrevH) / dt;
    gPrevH = h; gPrevHT = now;
    gHeading = h;
  }
  serviceOneToF(tofF, tofFok, CH_FRONT, tofFront, tofFrontValid);
  serviceOneToF(tofL, tofLok, CH_LEFT,  tofLeft,  tofLeftValid);
  serviceOneToF(tofR, tofRok, CH_RIGHT, tofRight, tofRightValid);
  serviceVision();                                  // <-- added
}

// ============================================================
// SYSTEM INITIALIZATION
// ============================================================
void initOneToF(VL53L1X &s, bool &ok, uint8_t ch, const __FlashStringHelper *name) {
  tcaselect(ch);
  s.setBus(&Wire);
  s.setTimeout(100);
  ok = s.init();
  if (ok) {
    s.setDistanceMode(VL53L1X::Short);
    s.setMeasurementTimingBudget(50000);
    s.startContinuous(50);
  }
  Serial.print(name); Serial.println(ok ? F(" ready") : F(" FAILED"));
}

void initHardware() {
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
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
    tcaselect(i);
    delay(10);
    if (tcs.begin()) { tcsConnected = true; tcsChannel = i;
      Serial.print(F("TCS34725 on TCA channel ")); Serial.println(i); break; }
  }
  if (!tcsConnected) Serial.println(F("WARNING: TCS34725 not detected!"));

  initOneToF(tofL, tofLok, CH_LEFT,  F("ToF LEFT  (CH1)"));
  initOneToF(tofR, tofRok, CH_RIGHT, F("ToF RIGHT (CH3)"));
  initOneToF(tofF, tofFok, CH_FRONT, F("ToF FRONT (CH4)"));

  SPI_IMU.begin();
  if (myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU)) {
    delay(500);
    myIMU.enableGameRotationVector();
    delay(100);
    myIMU.getSensorEvent();
    zeroYaw();
  }
}

// ============================================================
// HEADING PID  (Kp=2, cached sensor data, acts only on fresh sample)
// ============================================================
const float HEAD_KP        = 2.0;
const float HEAD_KD        = 0.0;
const float HEAD_KI        = 0.0;
const float YAW_FILT_ALPHA = 0.35;
const float SERVO_SLEW     = 2.5;
const float INTEGRAL_CLAMP = 300.0;

float         pidPrevError = 0.0;
unsigned long pidPrevTime  = 0;
float         pidIntegral  = 0.0;
float         yawFilt      = 0.0;
float         prevServoCmd = SERVO_TRUE_STRAIGHT;

void resetHeadingPid() {
  pidPrevError = 0.0;  pidPrevTime = millis();
  pidIntegral  = 0.0;  yawFilt = 0.0;
  prevServoCmd = SERVO_TRUE_STRAIGHT;
}

void updateHeadingPid(float heading) {
  if (!gImuFresh) return;
  unsigned long now = millis();
  yawFilt += YAW_FILT_ALPHA * (gYawRate - yawFilt);
  float dt = (now - pidPrevTime) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  float error = wrapDeg(heading - gHeading);
  pidIntegral += error * dt;
  pidIntegral  = constrain(pidIntegral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
  float correction = HEAD_KP * error + HEAD_KI * pidIntegral - HEAD_KD * yawFilt;
  float want = SERVO_TRUE_STRAIGHT - correction;
  float dcmd = constrain(want - prevServoCmd, -SERVO_SLEW, SERVO_SLEW);
  float cmd  = constrain(prevServoCmd + dcmd, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  if (cmd <= SERVO_MAX_LEFT || cmd >= SERVO_MAX_RIGHT) pidIntegral -= error * dt;
  setServoAngle(cmd);
  prevServoCmd = cmd;
  pidPrevError = error;
  pidPrevTime  = now;
}

// ============================================================
// EASED TURN LAW  (shared by corner turns and realign - NO settle)
// ============================================================
const float TURN_KP        = 2.5;
const float TURN_MAX_STEER = 55.0;
const float TURN_MIN_STEER = 8.0;
const float TURN_KV        = 3.5;
const int   TURN_MAX_PWM   = 130;
const int   TURN_MIN_PWM   = 100;
const float TURN_STOP_DEG  = 0.3;

long turnStartTicks = 0;
long turnCapTicks   = 0;

// One arc step toward an absolute heading. true when the arc is finished.
bool turnArcStep(float target) {
  if (absEnc(readEncoder() - turnStartTicks) >= turnCapTicks) return true;
  if (gImuFresh) {
    float err = wrapDeg(target - gHeading);
    if (fabs(err) < TURN_STOP_DEG) return true;
    float mag   = fabs(err);
    float steer = constrain(TURN_KP * mag, TURN_MIN_STEER, TURN_MAX_STEER);
    int   pwm   = (int)constrain(TURN_KV * mag, (float)TURN_MIN_PWM, (float)TURN_MAX_PWM);
    float servo = (err > 0) ? (SERVO_TRUE_STRAIGHT - steer) : (SERVO_TRUE_STRAIGHT + steer);
    setServoAngle(servo);
    setMotorSpeed(pwm);
  }
  return false;
}

// ============================================================
// STATE HELPERS + working vars
// ============================================================
void goState(RobotState s) { currentState = s; entered = false; }

BlockColor dcWantFirst;
bool       dcFirstTurn;
bool       dcColorArmed;
long       dcBaseTicks;      // encoder at this straight's start (local distance)
long       dcLockoutTicks;
long       dcSafetyTicks;
unsigned long sideInvalidStart = 0;   // when the watched side first read invalid

float      turnTarget;
float      turnAmount;
BlockColor turnPartner;
bool       turnGotSecond;

uint8_t    lcPhase;          // 2 = shuffle, 4 = realign
long       lcBaseTicks;
long       lcTargetTicks;
long       fsTargetTicks;

// ---- pillar avoidance working vars (new) ----
uint8_t       avoidPhase     = 0;    // 1 = drive the pillar to the offset, 2 = realign
char          avoidColour    = 'N';
float         avoidTargetDx  = 0.0;  // +AVOID_TARGET_DX for green, -for red
float         avoidServoCmd  = SERVO_TRUE_STRAIGHT;
float         avoidPrevErr   = 0.0;
float         avoidDFilt     = 0.0;
unsigned long avoidPidTime   = 0;
unsigned long avoidVisStamp  = 0;    // last vision frame the PID acted on
unsigned long avoidHoldStart = 0;
long          avoidRefTicks  = 0;
unsigned long avoidLostStart = 0;
bool          avoidResume    = false;  // re-entering DRIVE mid-straight

void finishLaneCorrect() {
  targetHeading = laneHeading;
  if (cornerCount >= TARGET_CORNERS) goState(STATE_FINAL_STRAIGHT);
  else                               goState(STATE_DRIVE_TO_CORNER);
}

// ============================================================
// COLOUR GATE  -  lifted verbatim out of driveStep so AVOID can run it too
// ============================================================
void serviceColorGate() {
  if (!dcColorArmed) {
    BlockColor c = detectColor(dcWantFirst);
    if (c != COLOR_NONE) {
      dcColorArmed = true;
      cornerFirstTicks = readEncoder();          // absolute from segment origin
      if (dcFirstTurn) lastFirstColor = c;
      sideInvalidStart = 0;                       // start the side watch fresh
      resetColorDetector();
      Serial.print(F("  colour gate armed: "));
      Serial.println(c == COLOR_ORANGE ? F("ORANGE") : F("BLUE"));
    }
    return;
  }

  if (!gapMeasured && detectColor(otherColor(lastFirstColor)) != COLOR_NONE) {
    long tr = absEnc(readEncoder() - cornerFirstTicks);
    lastGapCm = (float)tr / TICKS_PER_CM;
    gapMeasured = true;
    Serial.print(F("  partner line pre-turn, gap="));
    Serial.print(lastGapCm); Serial.println(F(" cm"));
  }
}

// ============================================================
// CORNER TRIGGER  -  the open-round conditions, unchanged, in one place
// so both DRIVE and AVOID can ask "is it time to turn?"
// ============================================================
bool cornerTurnDue() {
  if (!dcColorArmed) return false;              // same gate as the open round

  // clockwise (orange) -> island on the RIGHT -> watch RIGHT ToF
  // ccw       (blue)   -> island on the LEFT  -> watch LEFT  ToF
  bool expectedCW  = dcFirstTurn ? (lastFirstColor == COLOR_ORANGE) : clockwiseMode;
  bool sidePresent = expectedCW ? tofRok : tofLok;
  bool sideValid   = expectedCW ? tofRightValid : tofLeftValid;

  bool sideConfirmed = false;
  if (sidePresent && !sideValid) {              // watched wall reads invalid = gave way
    if (sideInvalidStart == 0) sideInvalidStart = millis();
    if (millis() - sideInvalidStart >= SIDE_INVALID_MS) sideConfirmed = true;
  } else {
    sideInvalidStart = 0;
  }

  bool frontClose = tofFrontValid && (tofFront <= FRONT_TURN_MM);
  bool noRange    = (!tofFok) && (!tofLok) && (!tofRok);

  if (!(frontClose || sideConfirmed || noRange)) return false;

  if (frontClose)         Serial.println(F("  turn: FRONT close"));
  else if (sideConfirmed) Serial.println(expectedCW ? F("  turn: RIGHT wall gone") : F("  turn: LEFT wall gone"));
  else                    Serial.println(F("  turn: colour-only (no range sensors)"));
  return true;
}

// the open-round lock + distance bookkeeping, unchanged
void commitCorner() {
  if (lockedColor == COLOR_NONE) {
    lockedColor   = lastFirstColor;
    clockwiseMode = (lockedColor == COLOR_ORANGE);
    Serial.println(clockwiseMode
      ? F("[FSM] LOCKED ORANGE -> CLOCKWISE (right turns)")
      : F("[FSM] LOCKED BLUE -> COUNTERCLOCKWISE (left turns)"));
  }

  // distance since segment origin (turn-complete / start) to the arm point
  float segCm = absEnc(cornerFirstTicks) / TICKS_PER_CM;
  if (cornerCount == 0) {
    firstSegmentCm = segCm;
    Serial.print(F("[DIST] A start->turn1 = ")); Serial.print(firstSegmentCm); Serial.println(F(" cm"));
  } else if (cornerCount == 4) {
    fullStartStraightCm = segCm; haveFullStraight = true;
  } else if (cornerCount == 8 && haveFullStraight) {
    fullStartStraightCm = 0.5 * (fullStartStraightCm + segCm);
  }
  if (haveFullStraight) {
    finalDistanceCm = fullStartStraightCm - firstSegmentCm;
    if (finalDistanceCm < 0) finalDistanceCm = 0;
    Serial.print(F("[DIST] L=")); Serial.print(fullStartStraightCm);
    Serial.print(F("  final(L-A)=")); Serial.print(finalDistanceCm); Serial.println(F(" cm"));
  }

  goState(STATE_TURNING);          // motor keeps rolling into the turn
}

// ============================================================
// STATE: DRIVE TO CORNER      (open round, plus the pillar hand-off)
// ============================================================
void driveStep() {
  if (!entered) {
    entered = true;
    if (avoidResume) {
      // coming back from a swerve mid-straight: keep the colour gate, the
      // gap, the segment origin and the lockout exactly as they were
      avoidResume = false;
      Serial.println(F("[FSM] DRIVE_TO_CORNER (resumed)"));
      resetHeadingPid();
      setMotorSpeed(BASE_SPEED);
    } else {
      Serial.println(F("[FSM] DRIVE_TO_CORNER"));
      dcWantFirst = lockedColor;
      dcFirstTurn = (lockedColor == COLOR_NONE);
      resetColorDetector();
      resetHeadingPid();
      dcBaseTicks = readEncoder();     // NO zero - keep the segment origin
      setMotorSpeed(BASE_SPEED);
      dcColorArmed   = false;
      gapMeasured    = false;
      sideInvalidStart = 0;
      dcLockoutTicks = (cornerCount > 0) ? (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM) : 0;
      dcSafetyTicks  = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);
    }
  }

  long straightTicks = absEnc(readEncoder() - dcBaseTicks);   // distance this straight
  if (straightTicks >= dcSafetyTicks) {
    Serial.println(F("[FSM] WARN: no line within safety distance, retrying"));
    entered = false;
    return;
  }

  updateHeadingPid(targetHeading);

  if (straightTicks <= dcLockoutTicks) return;

  serviceColorGate();

  // ---- TURNING IS TOP PRIORITY ----
  if (cornerTurnDue()) { commitCorner(); return; }

  // ---- otherwise, a pillar may be handled ----
  if (pillarSeen()) {
    avoidColour = vis.colour;
    // RED -> pass on the RIGHT, drive the pillar to the LEFT of frame
    // GREEN-> pass on the LEFT,  drive the pillar to the RIGHT of frame
    avoidTargetDx  = (avoidColour == 'R') ? -AVOID_TARGET_DX : +AVOID_TARGET_DX;
    avoidServoCmd  = SERVO_TRUE_STRAIGHT;
    avoidPrevErr   = avoidTargetDx - (float)(DX_SIGN * vis.dx);
    avoidDFilt     = 0.0;
    avoidPidTime   = millis();
    avoidVisStamp  = vis.stamp;
    avoidHoldStart = 0;
    avoidRefTicks  = readEncoder();
    avoidLostStart = 0;
    avoidPhase     = 1;
    Serial.print(F("[AVOID] "));
    Serial.print(avoidColour == 'R' ? F("RED -> move RIGHT") : F("GREEN -> move LEFT"));
    Serial.print(F("  dx=")); Serial.print(vis.dx);
    Serial.print(F("  target=")); Serial.println(avoidTargetDx);
    goState(STATE_AVOID);
    return;
  }
}

// ============================================================
// STATE: AVOID   (lean out slowly until the pillar is at the offset,
//                 then realign onto laneHeading and resume)
// ============================================================
void avoidStep() {
  if (!entered) {
    entered = true;
    avoidServoCmd = SERVO_TRUE_STRAIGHT;
    setServoAngle(avoidServoCmd);
    setMotorSpeed(AVOID_PWM);
  }

  serviceColorGate();                       // a line crossed here still counts

  // ---- TURNING IS TOP PRIORITY: drop the maneuver and take the corner ----
  if (cornerTurnDue()) {
    Serial.println(F("[AVOID] aborted - corner first"));
    avoidPhase = 0;
    resetHeadingPid();
    commitCorner();
    return;
  }

  if (avoidPhase == 1) {                    // PID drives the pillar to the offset
    setMotorSpeed(AVOID_PWM);

    bool haveTarget = visionFresh() && (vis.colour == avoidColour);
    float err = 0.0;

    if (haveTarget) {
      err = avoidTargetDx - (float)(DX_SIGN * vis.dx);   // e.g. green: 200 - dx

      if (vis.stamp != avoidVisStamp) {     // only step the PID on a NEW frame
        unsigned long now = millis();
        float dt = (now - avoidPidTime) / 1000.0;
        if (dt <= 0.0) dt = 0.001;
        float d = (err - avoidPrevErr) / dt;
        avoidDFilt += AVOID_D_ALPHA * (d - avoidDFilt);
        avoidPrevErr  = err;
        avoidPidTime  = now;
        avoidVisStamp = vis.stamp;

        // error > 0 wants the pillar further RIGHT in frame -> steer LEFT
        float steer = AVOID_KP * err + AVOID_KD * avoidDFilt;
        steer = constrain(steer, -AVOID_MAX_STEER, AVOID_MAX_STEER);
        float want = SERVO_TRUE_STRAIGHT - steer;

        // never lean further than AVOID_MAX_YAW off the lane heading
        float yawErr = wrapDeg(gHeading - laneHeading);
        if (yawErr >  AVOID_MAX_YAW && want < SERVO_TRUE_STRAIGHT) want = SERVO_TRUE_STRAIGHT;
        if (yawErr < -AVOID_MAX_YAW && want > SERVO_TRUE_STRAIGHT) want = SERVO_TRUE_STRAIGHT;

        float dcmd = constrain(want - avoidServoCmd, -AVOID_SERVO_SLEW, AVOID_SERVO_SLEW);
        avoidServoCmd = constrain(avoidServoCmd + dcmd, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
      }
      avoidLostStart = 0;
    } else {
      if (avoidLostStart == 0) avoidLostStart = millis();
    }

    setServoAngle(avoidServoCmd);

    // at the offset once the error has stayed inside tolerance long enough
    if (haveTarget && fabs(err) <= AVOID_DX_TOL) {
      if (avoidHoldStart == 0) avoidHoldStart = millis();
    } else {
      avoidHoldStart = 0;
    }
    bool atOffset = (avoidHoldStart != 0) && (millis() - avoidHoldStart >= AVOID_HOLD_MS);
    bool lost     = (avoidLostStart != 0) && (millis() - avoidLostStart >= AVOID_LOST_MS);

    // safety: don't crowd the wall we are leaning towards, and cap the lean
    uint16_t sideMm  = (avoidColour == 'R') ? tofRight      : tofLeft;
    bool     sideVal = (avoidColour == 'R') ? tofRightValid : tofLeftValid;
    bool wallClose = sideVal && (sideMm <= AVOID_SIDE_MIN_MM);
    bool capped    = absEnc(readEncoder() - avoidRefTicks) >= (long)(AVOID_MAX_OUT_CM * TICKS_PER_CM);

    if (atOffset || lost || wallClose || capped) {
      if (atOffset)        { Serial.print(F("[AVOID] at offset, err=")); Serial.println(err); }
      else if (lost)       Serial.println(F("[AVOID] pillar gone -> realign"));
      else if (wallClose)  Serial.println(F("[AVOID] wall close -> realign"));
      else                 Serial.println(F("[AVOID] lean capped -> realign"));
      turnStartTicks = readEncoder();
      turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
      avoidPhase = 2;
    }
    return;
  }

  // phase 2: eased realign back onto the lane heading, then straight back
  // into the open-round FSM
  if (turnArcStep(laneHeading)) {
    avoidPhase = 0;
    avoidResume = true;
    resetHeadingPid();
    setMotorSpeed(BASE_SPEED);
    Serial.println(F("[AVOID] done -> back to open round"));
    goState(STATE_DRIVE_TO_CORNER);
  }
}

// ============================================================
// STATE: TURNING  (eased 90 deg turn + mid-turn partner watch, NO settle)
// ============================================================
void turningStep() {
  if (!entered) {
    entered = true;
    cornerCount++;
    Serial.print(F("[FSM] TURNING - corner "));
    Serial.print(cornerCount); Serial.print(F(" / ")); Serial.println(TARGET_CORNERS);
    turnAmount    = clockwiseMode ? 90.0 : -90.0;
    turnTarget    = wrapDeg(laneHeading - turnAmount);
    turnPartner   = otherColor(lastFirstColor);
    resetColorDetector();
    turnGotSecond = gapMeasured;
    turnStartTicks = readEncoder();
    turnCapTicks   = (long)(120.0 * TICKS_PER_CM);
  }

  if (!turnGotSecond && detectColor(turnPartner) != COLOR_NONE) {
    long tr = absEnc(readEncoder() - cornerFirstTicks);
    lastGapCm = (float)tr / TICKS_PER_CM;
    turnGotSecond = true; gapMeasured = true;
    Serial.print(F("  partner line mid-turn, gap="));
    Serial.print(lastGapCm); Serial.println(F(" cm"));
  }

  if (turnArcStep(turnTarget)) {       // arc finished - proceed immediately
    if (!turnGotSecond) {
      lastGapCm = gapRefCm;
      Serial.println(F("  WARN: partner line missed, skipping correction"));
    }
    laneHeading = wrapDeg(laneHeading - turnAmount);
    if (cornerCount == 1 && !gapRefSet && gapMeasured) {
      gapRefCm  = lastGapCm; gapRefSet = true;
      Serial.print(F("[GAP] reference learned from turn 1 = "));
      Serial.print(gapRefCm); Serial.println(F(" cm"));
    }
    zeroEncoder();                     // segment origin = this turn corner
    avoidResume = false;               // a new straight always starts clean
    Serial.print(F("  lane heading now ")); Serial.println(laneHeading);
    goState(STATE_LANE_CORRECT);
  }
}

// ============================================================
// STATE: LANE CORRECT  (continuous shuffle -> realign, NO stops)
// ============================================================
void laneCorrectStep() {
  if (!entered) {
    entered = true;
    float delta = lastGapCm - gapRefCm;
    float mag = fabs(delta);
    if (mag < GAP_DEADBAND_CM) {
      Serial.println(F("  correction skipped (inside deadband)"));
      finishLaneCorrect();
      return;
    }
    float steerSigned = CORRECTION_SIGN * delta;
    if (clockwiseMode) steerSigned = -steerSigned;
    float offset = K_LAT_DEG_PER_CM * mag;
    if (offset > MAX_LAT_OFFSET_DEG) offset = MAX_LAT_OFFSET_DEG;
    float servo = (steerSigned > 0) ? (SERVO_TRUE_STRAIGHT - offset)
                                    : (SERVO_TRUE_STRAIGHT + offset);
    Serial.print(F("  delta=")); Serial.print(delta);
    Serial.print(F("  steer ")); Serial.print(steerSigned > 0 ? F("LEFT ") : F("RIGHT "));
    Serial.print(offset); Serial.println(F(" deg"));

    setServoAngle(servo);
    setMotorSpeed(CORRECTION_PWM);     // shuffle immediately (servo slews as it rolls)
    lcBaseTicks   = readEncoder();
    lcTargetTicks = (long)(CORRECTION_DISTANCE_CM * TICKS_PER_CM);
    lcPhase = 2;
    return;
  }

  if (lcPhase == 2) {                  // shuffle a fixed distance
    if (absEnc(readEncoder() - lcBaseTicks) >= lcTargetTicks) {
      turnStartTicks = readEncoder();
      turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
      lcPhase = 4;                     // realign (no stop)
    }
  } else {                             // lcPhase == 4: eased realign onto lane heading
    if (turnArcStep(laneHeading)) finishLaneCorrect();
  }
}

// ============================================================
// STATE: FINAL STRAIGHT
// (encoder still counts from turn 12, so readEncoder() already includes the
//  lane-correction travel - we drive until that total reaches L - A)
// ============================================================
void finalStraightStep() {
  if (!entered) {
    entered = true;
    Serial.print(F("[FSM] FINAL_STRAIGHT to L-A = "));
    Serial.print(finalDistanceCm); Serial.println(F("cm from turn 12"));
    resetHeadingPid();
    setMotorSpeed(BASE_SPEED);
    fsTargetTicks = (long)(finalDistanceCm * TICKS_PER_CM);
  }
  updateHeadingPid(laneHeading);
  if (absEnc(readEncoder()) >= fsTargetTicks) {
    setMotorSpeed(0);
    setServoAngle(SERVO_TRUE_STRAIGHT);
    goState(STATE_FINISHED);
  }
}

// ============================================================
// START BUTTON  (PA5 active-low, blocking wait - car not moving yet)
// ============================================================
void waitForStart() {
  Serial.println(F("[FSM] waiting for START button (PA5 -> GND)"));
  while (digitalRead(START_BTN_PIN) == HIGH) { delay(5); }   // wait for press (LOW)
  delay(30);                                                 // debounce
  while (digitalRead(START_BTN_PIN) == LOW)  { delay(5); }   // wait for release
  Serial.println(F("[FSM] START"));
}

// ============================================================
// MAIN
// ============================================================
void setup() {
  Serial.begin(115200);
  initHardware();
  delay(2000);
}

void loop() {
  serviceSensors();

  switch (currentState) {
    case STATE_INIT:
      Serial.println(F("[FSM] INIT"));
      lockedColor   = COLOR_NONE;
      cornerCount   = 0;
      avoidPhase    = 0;
      avoidResume   = false;
      laneHeading   = readHeading();
      targetHeading = laneHeading;
      zeroEncoder();                 // segment origin = start position
      setMotorSpeed(0);
      setServoAngle(SERVO_TRUE_STRAIGHT);
      goState(STATE_WAIT_START);
      break;

    case STATE_WAIT_START:
      waitForStart();
      laneHeading   = readHeading();   // re-zero the reference at the gun
      targetHeading = laneHeading;
      zeroEncoder();                   // segment origin = start position
      goState(STATE_DRIVE_TO_CORNER);
      break;

    case STATE_DRIVE_TO_CORNER: driveStep();         break;
    case STATE_TURNING:         turningStep();       break;
    case STATE_LANE_CORRECT:    laneCorrectStep();   break;
    case STATE_AVOID:           avoidStep();         break;
    case STATE_FINAL_STRAIGHT:  finalStraightStep(); break;

    case STATE_FINISHED:
      if (!entered) {
        entered = true;
        Serial.println(F("[FSM] FINISHED"));
        setMotorSpeed(0);
        setServoAngle(SERVO_TRUE_STRAIGHT);
      }
      break;
  }
}
#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>
#include <VL53L1X.h>

// ============================================================
// OPEN ROUND CORE + PILLAR AVOIDANCE
//
// The open-round FSM is UNCHANGED. Everything it did, it still does:
// drive -> colour gate -> turn -> lane correct -> straight -> final.
//
// Added on top, and nothing else:
//
//   STATE_AVOID   Camera (Pi, "V,<colour>,<dx>,<area>") sees a pillar.
//                 RED   -> steer RIGHT slowly until the pillar sits at an
//                          offset in the frame (dx pushed to the far side)
//                 GREEN -> steer LEFT  slowly, same idea
//                 then realign onto laneHeading and drop straight back
//                 into the open-round FSM where it left off.
//
//   TURNING IS TOP PRIORITY. The corner conditions are evaluated in DRIVE
//   *and* in AVOID. If they fire, the pillar maneuver is abandoned on the
//   spot and the car turns. A pillar can never hold off a corner.
//
//   The colour gate also keeps running during AVOID, so a corner line
//   crossed while swerving is still latched - otherwise the swerve would
//   eat the line that decides the turn direction.
// ============================================================

#define TCA_ADDR 0x70
#define PiSerial Serial          // Pi link IS USART1 on this board

enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };

enum RobotState {
  STATE_INIT,
  STATE_WAIT_START,              // <-- added
  STATE_DRIVE_TO_CORNER,
  STATE_TURNING,
  STATE_LANE_CORRECT,
  STATE_AVOID,                   // <-- added
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

const int START_BTN_PIN = PA5;        // active-low: PA5 <-> GND, press = LOW

const int IMU_CS_PIN  = PB0;
const int IMU_INT_PIN = PB13;
const int IMU_RST_PIN = PB14;

const float TICKS_PER_CM        = 31.933;
const float SERVO_TRUE_STRAIGHT = 69.0;
const float SERVO_MAX_LEFT      = 5.0;
const float SERVO_MAX_RIGHT     = 115.0;
const int   BASE_SPEED          = 150;

SPIClass SPI_IMU(PB5, PB4, PB3);
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

bool    tcsConnected     = false;
uint8_t tcsChannel       = 0;
float   initialYawOffset = 0.0;

// ToF (VL53L1X) on the mux:  CH1 = LEFT, CH3 = RIGHT, CH4 = FRONT
const uint8_t CH_LEFT  = 1;
const uint8_t CH_RIGHT = 3;
const uint8_t CH_FRONT = 4;
VL53L1X  tofL, tofR, tofF;
bool     tofLok = false, tofRok = false, tofFok = false;
uint16_t tofLeft = 9999, tofRight = 9999, tofFront = 9999;
bool     tofLeftValid = false, tofRightValid = false, tofFrontValid = false;
const uint16_t TOF_MAX_VALID_MM = 1300;
const float    SIGNAL_MIN_MCPS  = 4.0;
const uint16_t FRONT_TURN_MM    = 700;
const unsigned long SIDE_INVALID_MS = 40;   // watched-side invalid this long = wall gave way

// ============================================================
// RUN CONSTANTS
// ============================================================
const int   TARGET_CORNERS    = 12;
const int   FINAL_STRAIGHT_CM = 100;
const float SEARCH_SAFETY_CM  = 400.0;

float firstSegmentCm      = 0.0;
float fullStartStraightCm = 0.0;
bool  haveFullStraight    = false;
float finalDistanceCm     = FINAL_STRAIGHT_CM;

// ============================================================
// LANE CORRECTION TUNING
// ============================================================
const float GAP_THRESHOLD_CM       = 20.0;
const float GAP_DEADBAND_CM        =  2.0;
const float K_LAT_DEG_PER_CM       =  4;
const float MAX_LAT_OFFSET_DEG     = 30.0;
const float CORRECTION_DISTANCE_CM = 25.0;
const float POST_CORNER_LOCKOUT_CM = 50;
const int   CORRECTION_SIGN        =   1;
const int   CORRECTION_PWM         =   70;
const float REALIGN_SAFETY_CM      = 80.0;

// ============================================================
// PILLAR AVOIDANCE TUNING            (the only new tuning block)
// ============================================================
const int32_t  AVOID_MIN_AREA    = 1600;  // pillar this big in frame -> engage
const float    AVOID_STEER_DEG   = 20.0;  // how hard we lean off straight
const int      AVOID_PWM         = 100;   // "slowly"
const int      AVOID_OFFSET_DX   = 110;   // |dx| past this = pillar is at the offset
const int      DX_SIGN           = +1;    // +1 if dx grows to the RIGHT in your frame
                                          //    (flip to -1 if your Pi reports the other way)
const uint16_t AVOID_SIDE_MIN_MM = 120;   // never crowd the pass-side wall closer than this
const float    AVOID_MAX_OUT_CM  = 45.0;  // hard cap on how far we lean out
const unsigned long AVOID_LOST_MS = 200;  // pillar gone from frame this long = done

// ============================================================
// VISION  (from the Pi over Serial:  "V,<colour>,<dx>,<area>")
// ============================================================
struct Vision { char colour; int dx; long area; unsigned long stamp; };
Vision  vis = {'N', 0, 0, 0};
char    visBuf[48];
uint8_t visLen = 0;

void parseVisionLine(char *s) {
  if (s[0] != 'V' || s[1] != ',') return;
  char c = s[2];
  if (c != 'R' && c != 'G' && c != 'N') return;
  char *p1 = strchr(s + 3, ',');  if (!p1) return;
  char *p2 = strchr(p1 + 1, ','); if (!p2) return;
  vis.colour = c;
  vis.dx     = atoi(p1 + 1);
  vis.area   = atol(p2 + 1);
  vis.stamp  = millis();
}

void serviceVision() {
  while (PiSerial.available()) {
    char ch = PiSerial.read();
    if (ch == '\n' || ch == '\r') {
      if (visLen > 0) { visBuf[visLen] = 0; parseVisionLine(visBuf); visLen = 0; }
    } else if (visLen < sizeof(visBuf) - 1) {
      visBuf[visLen++] = ch;
    }
  }
}

bool visionFresh() { return (millis() - vis.stamp) < 250; }

bool pillarSeen() {              // a real pillar, big enough to act on
  if (!visionFresh()) return false;
  if (vis.colour != 'R' && vis.colour != 'G') return false;
  return vis.area >= AVOID_MIN_AREA;
}

// ============================================================
// FSM DATA
// ============================================================
RobotState currentState = STATE_INIT;
bool          entered   = false;
unsigned long phaseT0   = 0;

BlockColor lockedColor   = COLOR_NONE;
bool       clockwiseMode = true;
int        cornerCount   = 0;

float targetHeading = 0.0;
float laneHeading   = 0.0;

float      lastGapCm        = 0.0;
BlockColor lastFirstColor   = COLOR_NONE;
long       cornerFirstTicks = 0;
bool       gapMeasured      = false;

float gapRefCm  = GAP_THRESHOLD_CM;
bool  gapRefSet = false;

// cached sensor readings, refreshed every loop
bool          gImuFresh = false;
float         gHeading  = 0.0;
float         gYawRate  = 0.0;
float         gPrevH    = 0.0;
unsigned long gPrevHT   = 0;

// ============================================================
// HELPERS
// ============================================================
float wrapDeg(float angle) {
  while (angle > 180.0)  angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { analogWrite(RPWM_PIN, speed); analogWrite(LPWM_PIN, 0); }
  else if (speed < 0) { analogWrite(RPWM_PIN, 0);     analogWrite(LPWM_PIN, -speed); }
  else                { analogWrite(RPWM_PIN, 0);     analogWrite(LPWM_PIN, 0); }
}

void setServoAngle(float angleDeg) {
  angleDeg = constrain(angleDeg, 5.0, 115.0);
  steeringServo.writeMicroseconds((int)((angleDeg / 180.0) * 1000.0) + 1000);
}

void zeroEncoder() { TIM3->CNT = 0; }
long readEncoder() { return (int16_t)TIM3->CNT; }
long absEnc(long v) { return v < 0 ? -v : v; }

// ---- IMU ----
float readYaw() {
  float qI = myIMU.getQuatI(), qJ = myIMU.getQuatJ();
  float qK = myIMU.getQuatK(), qReal = myIMU.getQuatReal();
  if (qI == 0.0f && qJ == 0.0f && qK == 0.0f && qReal == 0.0f) return 0.0f;
  float yawRadians = atan2(2.0f * (qI * qJ + qReal * qK),
                           (qReal * qReal + qI * qI - qJ * qJ - qK * qK));
  return yawRadians * (180.0 / PI);
}
float readHeading() { return fmod(readYaw() - initialYawOffset + 540.0, 360.0) - 180.0; }

void zeroYaw() {   // startup-only blocking zero (car not running yet)
  Serial.println(F("Waiting for valid IMU data to set Zero..."));
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (myIMU.wasReset()) myIMU.enableGameRotationVector();
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      initialYawOffset = readYaw();
      Serial.print(F("Zero Yaw locked at: ")); Serial.println(initialYawOffset);
      return;
    }
    delay(10);
  }
  Serial.println(F("ERROR: no IMU event to zero!"));
}

// ---- colour (direct register read, NO delay) ----
void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (!tcsConnected) { r = 0; g = 0; b = 0; c = 0; return; }
  tcaselect(tcsChannel);
  Wire.beginTransmission(0x29);
  Wire.write(0x80 | 0x20 | 0x14);      // command | auto-increment | CDATAL
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x29, (uint8_t)8);
  if (Wire.available() < 8) { r = 0; g = 0; b = 0; c = 0; return; }
  c  =  (uint16_t)Wire.read();  c |= (uint16_t)Wire.read() << 8;
  r  =  (uint16_t)Wire.read();  r |= (uint16_t)Wire.read() << 8;
  g  =  (uint16_t)Wire.read();  g |= (uint16_t)Wire.read() << 8;
  b  =  (uint16_t)Wire.read();  b |= (uint16_t)Wire.read() << 8;
}

const unsigned long COLOR_CONFIRM_MS = 6;
BlockColor    pendingColor  = COLOR_NONE;
unsigned long pendingStart  = 0;

void resetColorDetector() { pendingColor = COLOR_NONE; pendingStart = 0; }

BlockColor otherColor(BlockColor c) {
  if (c == COLOR_ORANGE) return COLOR_BLUE;
  if (c == COLOR_BLUE)   return COLOR_ORANGE;
  return COLOR_NONE;
}

BlockColor detectColor(BlockColor wantColor) {
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
  if (rawColor != pendingColor) { pendingColor = rawColor; pendingStart = millis(); return COLOR_NONE; }
  if (millis() - pendingStart >= COLOR_CONFIRM_MS) { resetColorDetector(); return rawColor; }
  return COLOR_NONE;
}

// ---- ToF (non-blocking, signal-filtered) ----
void serviceOneToF(VL53L1X &s, bool present, uint8_t ch, uint16_t &mm, bool &valid) {
  if (!present) { valid = false; return; }
  tcaselect(ch);
  if (s.dataReady()) {
    uint16_t d = s.read(false);
    bool ok = (s.ranging_data.range_status == VL53L1X::RangeValid) &&
              (s.ranging_data.peak_signal_count_rate_MCPS >= SIGNAL_MIN_MCPS) &&
              (d > 0) && (d < TOF_MAX_VALID_MM);
    valid = ok;
    if (ok) mm = d;
  }
}

// ---- called once at the top of every loop ----
void serviceSensors() {
  gImuFresh = false;
  if (myIMU.wasReset()) myIMU.enableGameRotationVector();
  if (myIMU.getSensorEvent() &&
      myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    gImuFresh = true;
    float h = readHeading();
    unsigned long now = millis();
    float dt = (now - gPrevHT) / 1000.0;
    if (dt > 0.0) gYawRate = wrapDeg(h - gPrevH) / dt;
    gPrevH = h; gPrevHT = now;
    gHeading = h;
  }
  serviceOneToF(tofF, tofFok, CH_FRONT, tofFront, tofFrontValid);
  serviceOneToF(tofL, tofLok, CH_LEFT,  tofLeft,  tofLeftValid);
  serviceOneToF(tofR, tofRok, CH_RIGHT, tofRight, tofRightValid);
  serviceVision();                                  // <-- added
}

// ============================================================
// SYSTEM INITIALIZATION
// ============================================================
void initOneToF(VL53L1X &s, bool &ok, uint8_t ch, const __FlashStringHelper *name) {
  tcaselect(ch);
  s.setBus(&Wire);
  s.setTimeout(100);
  ok = s.init();
  if (ok) {
    s.setDistanceMode(VL53L1X::Short);
    s.setMeasurementTimingBudget(50000);
    s.startContinuous(50);
  }
  Serial.print(name); Serial.println(ok ? F(" ready") : F(" FAILED"));
}

void initHardware() {
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
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
    tcaselect(i);
    delay(10);
    if (tcs.begin()) { tcsConnected = true; tcsChannel = i;
      Serial.print(F("TCS34725 on TCA channel ")); Serial.println(i); break; }
  }
  if (!tcsConnected) Serial.println(F("WARNING: TCS34725 not detected!"));

  initOneToF(tofL, tofLok, CH_LEFT,  F("ToF LEFT  (CH1)"));
  initOneToF(tofR, tofRok, CH_RIGHT, F("ToF RIGHT (CH3)"));
  initOneToF(tofF, tofFok, CH_FRONT, F("ToF FRONT (CH4)"));

  SPI_IMU.begin();
  if (myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU)) {
    delay(500);
    myIMU.enableGameRotationVector();
    delay(100);
    myIMU.getSensorEvent();
    zeroYaw();
  }
}

// ============================================================
// HEADING PID  (Kp=2, cached sensor data, acts only on fresh sample)
// ============================================================
const float HEAD_KP        = 2.0;
const float HEAD_KD        = 0.0;
const float HEAD_KI        = 0.0;
const float YAW_FILT_ALPHA = 0.35;
const float SERVO_SLEW     = 2.5;
const float INTEGRAL_CLAMP = 300.0;

float         pidPrevError = 0.0;
unsigned long pidPrevTime  = 0;
float         pidIntegral  = 0.0;
float         yawFilt      = 0.0;
float         prevServoCmd = SERVO_TRUE_STRAIGHT;

void resetHeadingPid() {
  pidPrevError = 0.0;  pidPrevTime = millis();
  pidIntegral  = 0.0;  yawFilt = 0.0;
  prevServoCmd = SERVO_TRUE_STRAIGHT;
}

void updateHeadingPid(float heading) {
  if (!gImuFresh) return;
  unsigned long now = millis();
  yawFilt += YAW_FILT_ALPHA * (gYawRate - yawFilt);
  float dt = (now - pidPrevTime) / 1000.0;
  if (dt <= 0.0) dt = 0.001;
  float error = wrapDeg(heading - gHeading);
  pidIntegral += error * dt;
  pidIntegral  = constrain(pidIntegral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
  float correction = HEAD_KP * error + HEAD_KI * pidIntegral - HEAD_KD * yawFilt;
  float want = SERVO_TRUE_STRAIGHT - correction;
  float dcmd = constrain(want - prevServoCmd, -SERVO_SLEW, SERVO_SLEW);
  float cmd  = constrain(prevServoCmd + dcmd, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  if (cmd <= SERVO_MAX_LEFT || cmd >= SERVO_MAX_RIGHT) pidIntegral -= error * dt;
  setServoAngle(cmd);
  prevServoCmd = cmd;
  pidPrevError = error;
  pidPrevTime  = now;
}

// ============================================================
// EASED TURN LAW  (shared by corner turns and realign - NO settle)
// ============================================================
const float TURN_KP        = 2.5;
const float TURN_MAX_STEER = 55.0;
const float TURN_MIN_STEER = 8.0;
const float TURN_KV        = 3.5;
const int   TURN_MAX_PWM   = 130;
const int   TURN_MIN_PWM   = 100;
const float TURN_STOP_DEG  = 0.3;

long turnStartTicks = 0;
long turnCapTicks   = 0;

// One arc step toward an absolute heading. true when the arc is finished.
bool turnArcStep(float target) {
  if (absEnc(readEncoder() - turnStartTicks) >= turnCapTicks) return true;
  if (gImuFresh) {
    float err = wrapDeg(target - gHeading);
    if (fabs(err) < TURN_STOP_DEG) return true;
    float mag   = fabs(err);
    float steer = constrain(TURN_KP * mag, TURN_MIN_STEER, TURN_MAX_STEER);
    int   pwm   = (int)constrain(TURN_KV * mag, (float)TURN_MIN_PWM, (float)TURN_MAX_PWM);
    float servo = (err > 0) ? (SERVO_TRUE_STRAIGHT - steer) : (SERVO_TRUE_STRAIGHT + steer);
    setServoAngle(servo);
    setMotorSpeed(pwm);
  }
  return false;
}

// ============================================================
// STATE HELPERS + working vars
// ============================================================
void goState(RobotState s) { currentState = s; entered = false; }

BlockColor dcWantFirst;
bool       dcFirstTurn;
bool       dcColorArmed;
long       dcBaseTicks;      // encoder at this straight's start (local distance)
long       dcLockoutTicks;
long       dcSafetyTicks;
unsigned long sideInvalidStart = 0;   // when the watched side first read invalid

float      turnTarget;
float      turnAmount;
BlockColor turnPartner;
bool       turnGotSecond;

uint8_t    lcPhase;          // 2 = shuffle, 4 = realign
long       lcBaseTicks;
long       lcTargetTicks;
long       fsTargetTicks;

// ---- pillar avoidance working vars (new) ----
uint8_t       avoidPhase     = 0;    // 1 = lean out, 2 = realign
char          avoidColour    = 'N';
float         avoidServo     = SERVO_TRUE_STRAIGHT;
long          avoidRefTicks  = 0;
unsigned long avoidLostStart = 0;
bool          avoidResume    = false;  // re-entering DRIVE mid-straight

void finishLaneCorrect() {
  targetHeading = laneHeading;
  if (cornerCount >= TARGET_CORNERS) goState(STATE_FINAL_STRAIGHT);
  else                               goState(STATE_DRIVE_TO_CORNER);
}

// ============================================================
// COLOUR GATE  -  lifted verbatim out of driveStep so AVOID can run it too
// ============================================================
void serviceColorGate() {
  if (!dcColorArmed) {
    BlockColor c = detectColor(dcWantFirst);
    if (c != COLOR_NONE) {
      dcColorArmed = true;
      cornerFirstTicks = readEncoder();          // absolute from segment origin
      if (dcFirstTurn) lastFirstColor = c;
      sideInvalidStart = 0;                       // start the side watch fresh
      resetColorDetector();
      Serial.print(F("  colour gate armed: "));
      Serial.println(c == COLOR_ORANGE ? F("ORANGE") : F("BLUE"));
    }
    return;
  }

  if (!gapMeasured && detectColor(otherColor(lastFirstColor)) != COLOR_NONE) {
    long tr = absEnc(readEncoder() - cornerFirstTicks);
    lastGapCm = (float)tr / TICKS_PER_CM;
    gapMeasured = true;
    Serial.print(F("  partner line pre-turn, gap="));
    Serial.print(lastGapCm); Serial.println(F(" cm"));
  }
}

// ============================================================
// CORNER TRIGGER  -  the open-round conditions, unchanged, in one place
// so both DRIVE and AVOID can ask "is it time to turn?"
// ============================================================
bool cornerTurnDue() {
  if (!dcColorArmed) return false;              // same gate as the open round

  // clockwise (orange) -> island on the RIGHT -> watch RIGHT ToF
  // ccw       (blue)   -> island on the LEFT  -> watch LEFT  ToF
  bool expectedCW  = dcFirstTurn ? (lastFirstColor == COLOR_ORANGE) : clockwiseMode;
  bool sidePresent = expectedCW ? tofRok : tofLok;
  bool sideValid   = expectedCW ? tofRightValid : tofLeftValid;

  bool sideConfirmed = false;
  if (sidePresent && !sideValid) {              // watched wall reads invalid = gave way
    if (sideInvalidStart == 0) sideInvalidStart = millis();
    if (millis() - sideInvalidStart >= SIDE_INVALID_MS) sideConfirmed = true;
  } else {
    sideInvalidStart = 0;
  }

  bool frontClose = tofFrontValid && (tofFront <= FRONT_TURN_MM);
  bool noRange    = (!tofFok) && (!tofLok) && (!tofRok);

  if (!(frontClose || sideConfirmed || noRange)) return false;

  if (frontClose)         Serial.println(F("  turn: FRONT close"));
  else if (sideConfirmed) Serial.println(expectedCW ? F("  turn: RIGHT wall gone") : F("  turn: LEFT wall gone"));
  else                    Serial.println(F("  turn: colour-only (no range sensors)"));
  return true;
}

// the open-round lock + distance bookkeeping, unchanged
void commitCorner() {
  if (lockedColor == COLOR_NONE) {
    lockedColor   = lastFirstColor;
    clockwiseMode = (lockedColor == COLOR_ORANGE);
    Serial.println(clockwiseMode
      ? F("[FSM] LOCKED ORANGE -> CLOCKWISE (right turns)")
      : F("[FSM] LOCKED BLUE -> COUNTERCLOCKWISE (left turns)"));
  }

  // distance since segment origin (turn-complete / start) to the arm point
  float segCm = absEnc(cornerFirstTicks) / TICKS_PER_CM;
  if (cornerCount == 0) {
    firstSegmentCm = segCm;
    Serial.print(F("[DIST] A start->turn1 = ")); Serial.print(firstSegmentCm); Serial.println(F(" cm"));
  } else if (cornerCount == 4) {
    fullStartStraightCm = segCm; haveFullStraight = true;
  } else if (cornerCount == 8 && haveFullStraight) {
    fullStartStraightCm = 0.5 * (fullStartStraightCm + segCm);
  }
  if (haveFullStraight) {
    finalDistanceCm = fullStartStraightCm - firstSegmentCm;
    if (finalDistanceCm < 0) finalDistanceCm = 0;
    Serial.print(F("[DIST] L=")); Serial.print(fullStartStraightCm);
    Serial.print(F("  final(L-A)=")); Serial.print(finalDistanceCm); Serial.println(F(" cm"));
  }

  goState(STATE_TURNING);          // motor keeps rolling into the turn
}

// ============================================================
// STATE: DRIVE TO CORNER      (open round, plus the pillar hand-off)
// ============================================================
void driveStep() {
  if (!entered) {
    entered = true;
    if (avoidResume) {
      // coming back from a swerve mid-straight: keep the colour gate, the
      // gap, the segment origin and the lockout exactly as they were
      avoidResume = false;
      Serial.println(F("[FSM] DRIVE_TO_CORNER (resumed)"));
      resetHeadingPid();
      setMotorSpeed(BASE_SPEED);
    } else {
      Serial.println(F("[FSM] DRIVE_TO_CORNER"));
      dcWantFirst = lockedColor;
      dcFirstTurn = (lockedColor == COLOR_NONE);
      resetColorDetector();
      resetHeadingPid();
      dcBaseTicks = readEncoder();     // NO zero - keep the segment origin
      setMotorSpeed(BASE_SPEED);
      dcColorArmed   = false;
      gapMeasured    = false;
      sideInvalidStart = 0;
      dcLockoutTicks = (cornerCount > 0) ? (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM) : 0;
      dcSafetyTicks  = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);
    }
  }

  long straightTicks = absEnc(readEncoder() - dcBaseTicks);   // distance this straight
  if (straightTicks >= dcSafetyTicks) {
    Serial.println(F("[FSM] WARN: no line within safety distance, retrying"));
    entered = false;
    return;
  }

  updateHeadingPid(targetHeading);

  if (straightTicks <= dcLockoutTicks) return;

  serviceColorGate();

  // ---- TURNING IS TOP PRIORITY ----
  if (cornerTurnDue()) { commitCorner(); return; }

  // ---- otherwise, a pillar may be handled ----
  if (pillarSeen()) {
    avoidColour = vis.colour;
    // RED -> move RIGHT, GREEN -> move LEFT
    avoidServo  = (avoidColour == 'R') ? (SERVO_TRUE_STRAIGHT + AVOID_STEER_DEG)
                                       : (SERVO_TRUE_STRAIGHT - AVOID_STEER_DEG);
    avoidRefTicks  = readEncoder();
    avoidLostStart = 0;
    avoidPhase     = 1;
    Serial.print(F("[AVOID] "));
    Serial.print(avoidColour == 'R' ? F("RED -> move RIGHT") : F("GREEN -> move LEFT"));
    Serial.print(F("  dx=")); Serial.println(vis.dx);
    goState(STATE_AVOID);
    return;
  }
}

// ============================================================
// STATE: AVOID   (lean out slowly until the pillar is at the offset,
//                 then realign onto laneHeading and resume)
// ============================================================
void avoidStep() {
  if (!entered) {
    entered = true;
    setServoAngle(avoidServo);
    setMotorSpeed(AVOID_PWM);
  }

  serviceColorGate();                       // a line crossed here still counts

  // ---- TURNING IS TOP PRIORITY: drop the maneuver and take the corner ----
  if (cornerTurnDue()) {
    Serial.println(F("[AVOID] aborted - corner first"));
    avoidPhase = 0;
    resetHeadingPid();
    commitCorner();
    return;
  }

  if (avoidPhase == 1) {                    // lean out, slowly
    setServoAngle(avoidServo);
    setMotorSpeed(AVOID_PWM);

    // the pillar is "at the offset" once it has slid to the far side of the
    // frame:  RED (we went right) -> dx to the LEFT,  GREEN -> dx to the RIGHT
    bool atOffset = false;
    if (visionFresh() && vis.colour == avoidColour) {
      int dx = DX_SIGN * vis.dx;
      atOffset = (avoidColour == 'R') ? (dx <= -AVOID_OFFSET_DX)
                                      : (dx >=  AVOID_OFFSET_DX);
    }

    // pillar left the frame entirely
    if (!pillarSeen() || vis.colour != avoidColour) {
      if (avoidLostStart == 0) avoidLostStart = millis();
    } else {
      avoidLostStart = 0;
    }
    bool lost = (avoidLostStart != 0) && (millis() - avoidLostStart >= AVOID_LOST_MS);

    // safety: don't crowd the wall we are leaning towards, and cap the lean
    uint16_t sideMm  = (avoidColour == 'R') ? tofRight      : tofLeft;
    bool     sideVal = (avoidColour == 'R') ? tofRightValid : tofLeftValid;
    bool wallClose = sideVal && (sideMm <= AVOID_SIDE_MIN_MM);
    bool capped    = absEnc(readEncoder() - avoidRefTicks) >= (long)(AVOID_MAX_OUT_CM * TICKS_PER_CM);

    if (atOffset || lost || wallClose || capped) {
      if (atOffset)        Serial.println(F("[AVOID] pillar at offset -> realign"));
      else if (lost)       Serial.println(F("[AVOID] pillar gone -> realign"));
      else if (wallClose)  Serial.println(F("[AVOID] wall close -> realign"));
      else                 Serial.println(F("[AVOID] lean capped -> realign"));
      turnStartTicks = readEncoder();
      turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
      avoidPhase = 2;
    }
    return;
  }

  // phase 2: eased realign back onto the lane heading, then straight back
  // into the open-round FSM
  if (turnArcStep(laneHeading)) {
    avoidPhase = 0;
    avoidResume = true;
    resetHeadingPid();
    setMotorSpeed(BASE_SPEED);
    Serial.println(F("[AVOID] done -> back to open round"));
    goState(STATE_DRIVE_TO_CORNER);
  }
}

// ============================================================
// STATE: TURNING  (eased 90 deg turn + mid-turn partner watch, NO settle)
// ============================================================
void turningStep() {
  if (!entered) {
    entered = true;
    cornerCount++;
    Serial.print(F("[FSM] TURNING - corner "));
    Serial.print(cornerCount); Serial.print(F(" / ")); Serial.println(TARGET_CORNERS);
    turnAmount    = clockwiseMode ? 90.0 : -90.0;
    turnTarget    = wrapDeg(laneHeading - turnAmount);
    turnPartner   = otherColor(lastFirstColor);
    resetColorDetector();
    turnGotSecond = gapMeasured;
    turnStartTicks = readEncoder();
    turnCapTicks   = (long)(120.0 * TICKS_PER_CM);
  }

  if (!turnGotSecond && detectColor(turnPartner) != COLOR_NONE) {
    long tr = absEnc(readEncoder() - cornerFirstTicks);
    lastGapCm = (float)tr / TICKS_PER_CM;
    turnGotSecond = true; gapMeasured = true;
    Serial.print(F("  partner line mid-turn, gap="));
    Serial.print(lastGapCm); Serial.println(F(" cm"));
  }

  if (turnArcStep(turnTarget)) {       // arc finished - proceed immediately
    if (!turnGotSecond) {
      lastGapCm = gapRefCm;
      Serial.println(F("  WARN: partner line missed, skipping correction"));
    }
    laneHeading = wrapDeg(laneHeading - turnAmount);
    if (cornerCount == 1 && !gapRefSet && gapMeasured) {
      gapRefCm  = lastGapCm; gapRefSet = true;
      Serial.print(F("[GAP] reference learned from turn 1 = "));
      Serial.print(gapRefCm); Serial.println(F(" cm"));
    }
    zeroEncoder();                     // segment origin = this turn corner
    avoidResume = false;               // a new straight always starts clean
    Serial.print(F("  lane heading now ")); Serial.println(laneHeading);
    goState(STATE_LANE_CORRECT);
  }
}

// ============================================================
// STATE: LANE CORRECT  (continuous shuffle -> realign, NO stops)
// ============================================================
void laneCorrectStep() {
  if (!entered) {
    entered = true;
    float delta = lastGapCm - gapRefCm;
    float mag = fabs(delta);
    if (mag < GAP_DEADBAND_CM) {
      Serial.println(F("  correction skipped (inside deadband)"));
      finishLaneCorrect();
      return;
    }
    float steerSigned = CORRECTION_SIGN * delta;
    if (clockwiseMode) steerSigned = -steerSigned;
    float offset = K_LAT_DEG_PER_CM * mag;
    if (offset > MAX_LAT_OFFSET_DEG) offset = MAX_LAT_OFFSET_DEG;
    float servo = (steerSigned > 0) ? (SERVO_TRUE_STRAIGHT - offset)
                                    : (SERVO_TRUE_STRAIGHT + offset);
    Serial.print(F("  delta=")); Serial.print(delta);
    Serial.print(F("  steer ")); Serial.print(steerSigned > 0 ? F("LEFT ") : F("RIGHT "));
    Serial.print(offset); Serial.println(F(" deg"));

    setServoAngle(servo);
    setMotorSpeed(CORRECTION_PWM);     // shuffle immediately (servo slews as it rolls)
    lcBaseTicks   = readEncoder();
    lcTargetTicks = (long)(CORRECTION_DISTANCE_CM * TICKS_PER_CM);
    lcPhase = 2;
    return;
  }

  if (lcPhase == 2) {                  // shuffle a fixed distance
    if (absEnc(readEncoder() - lcBaseTicks) >= lcTargetTicks) {
      turnStartTicks = readEncoder();
      turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
      lcPhase = 4;                     // realign (no stop)
    }
  } else {                             // lcPhase == 4: eased realign onto lane heading
    if (turnArcStep(laneHeading)) finishLaneCorrect();
  }
}

// ============================================================
// STATE: FINAL STRAIGHT
// (encoder still counts from turn 12, so readEncoder() already includes the
//  lane-correction travel - we drive until that total reaches L - A)
// ============================================================
void finalStraightStep() {
  if (!entered) {
    entered = true;
    Serial.print(F("[FSM] FINAL_STRAIGHT to L-A = "));
    Serial.print(finalDistanceCm); Serial.println(F("cm from turn 12"));
    resetHeadingPid();
    setMotorSpeed(BASE_SPEED);
    fsTargetTicks = (long)(finalDistanceCm * TICKS_PER_CM);
  }
  updateHeadingPid(laneHeading);
  if (absEnc(readEncoder()) >= fsTargetTicks) {
    setMotorSpeed(0);
    setServoAngle(SERVO_TRUE_STRAIGHT);
    goState(STATE_FINISHED);
  }
}

// ============================================================
// START BUTTON  (PA5 active-low, blocking wait - car not moving yet)
// ============================================================
void waitForStart() {
  Serial.println(F("[FSM] waiting for START button (PA5 -> GND)"));
  while (digitalRead(START_BTN_PIN) == HIGH) { delay(5); }   // wait for press (LOW)
  delay(30);                                                 // debounce
  while (digitalRead(START_BTN_PIN) == LOW)  { delay(5); }   // wait for release
  Serial.println(F("[FSM] START"));
}

// ============================================================
// MAIN
// ============================================================
void setup() {
  Serial.begin(115200);
  initHardware();
  delay(2000);
}

void loop() {
  serviceSensors();

  switch (currentState) {
    case STATE_INIT:
      Serial.println(F("[FSM] INIT"));
      lockedColor   = COLOR_NONE;
      cornerCount   = 0;
      avoidPhase    = 0;
      avoidResume   = false;
      laneHeading   = readHeading();
      targetHeading = laneHeading;
      zeroEncoder();                 // segment origin = start position
      setMotorSpeed(0);
      setServoAngle(SERVO_TRUE_STRAIGHT);
      goState(STATE_WAIT_START);
      break;

    case STATE_WAIT_START:
      waitForStart();
      laneHeading   = readHeading();   // re-zero the reference at the gun
      targetHeading = laneHeading;
      zeroEncoder();                   // segment origin = start position
      goState(STATE_DRIVE_TO_CORNER);
      break;

    case STATE_DRIVE_TO_CORNER: driveStep();         break;
    case STATE_TURNING:         turningStep();       break;
    case STATE_LANE_CORRECT:    laneCorrectStep();   break;
    case STATE_AVOID:           avoidStep();         break;
    case STATE_FINAL_STRAIGHT:  finalStraightStep(); break;

    case STATE_FINISHED:
      if (!entered) {
        entered = true;
        Serial.println(F("[FSM] FINISHED"));
        setMotorSpeed(0);
        setServoAngle(SERVO_TRUE_STRAIGHT);
      }
      break;
  }
}
