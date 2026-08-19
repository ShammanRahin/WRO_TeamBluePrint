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
// OBSTACLE ROUND  -  clean build on the open-round navigation core.
//
//   NAV  : same non-blocking FSM as the open round. The FLOOR colour
//          sensor gates corner turns (orange/blue lines); turns fire
//          when the front wall is close OR the inner side wall gives way.
//
//   PILLARS (camera, over Serial from the Pi):
//          "V,<colour>,<dx>,<area>"  colour = R | G | N
//          OFFSET-BASED avoidance: the pillar's own position (dx) sets
//          how far we move; the pass-side ToF is only a SAFETY FLOOR,
//          never the target (that was the wall-following flaw). We swerve
//          to the computed offset, hold heading until the colour leaves
//          the frame, then come back the SAME remembered displacement.
//              GREEN -> pass on the LEFT   (steer left, watch LEFT  ToF)
//              RED   -> pass on the RIGHT  (steer right, watch RIGHT ToF)
//
//   FRONT FAILSAFE: while avoiding, if the front ToF < 200 mm we are
//          about to clip something -> reverse on the last steering angle,
//          temporarily increase the swerve angle, then push forward again.
//          After the pillar is cleared the swerve angle returns to base.
//
//   No parking. 3 laps = 12 corners.
// ============================================================
#define TCA_ADDR 0x70
#define PiSerial Serial          // Pi link IS USART1 on this board
enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };
enum RobotState {
  STATE_INIT,
  STATE_WAIT_START,
  STATE_DRIVE_TO_CORNER,
  STATE_TURNING,
  STATE_LANE_CORRECT,
  STATE_AVOID,
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
// PILLAR AVOIDANCE TUNING  (offset-based)
// ============================================================
const int32_t  AVOID_MIN_AREA_R     = 1600;   // RED   pillar this big -> engage
const int32_t  AVOID_MIN_AREA_G     = 1600;   // GREEN pillar this big -> engage
const float    AVOID_STEER_DEG      = 22.0;   // base swerve angle off straight
const int      AVOID_PWM            = 120;    // speed during the maneuver
const int      DX_SPAN              = 160;    // half frame width in px (dx range)
const uint16_t AVOID_SIDE_NEAR_MM   = 150;    // target from pass wall when pillar CENTRED
const uint16_t AVOID_SIDE_FAR_MM    = 300;    // target when pillar already off to the side
const uint16_t SIDE_SAFE_MM         = 100;    // hard floor: never closer than this to a wall
const uint16_t AVOID_OUT_CAP_CM     = 45;     // fallback swerve cap if wall not seen
const unsigned long AVOID_RELEASE_MS = 150;   // colour must be gone this long to release
// --- front failsafe (near-miss recovery) ---
const uint16_t FRONT_STOP_MM        = 200;    // front closer than this = about to hit
const float    AVOID_STEER_BOOST    = 8.0;    // extra swerve added each near-miss
const float    AVOID_STEER_MAX      = 44.0;   // cap on the boosted swerve
const uint16_t AVOID_BACKUP_CM      = 15;     // reverse distance during recovery
const int      AVOID_BACK_PWM       = 110;    // reverse speed
const float    RECOVER_MIN_FWD_CM   = 5.0;    // must move forward this far between backups
// ============================================================
// VISION (from the Pi over Serial, full-duplex)
// ============================================================
struct Vision { char colour; int dx; long area; unsigned long stamp; };
Vision  vis = {'N', 0, 0, 0};
char    visBuf[48];
uint8_t visLen = 0;
void parseVisionLine(char *s) {          // strict  "V,c,dx,area"
  if (s[0] != 'V' || s[1] != ',') return;
  char c = s[2];
  if (c != 'R' && c != 'G' && c != 'N') return;
  char *p1 = strchr(s + 3, ',');  if (!p1) return;   // after colour
  char *p2 = strchr(p1 + 1, ','); if (!p2) return;   // after dx
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
bool pillarEngaged() {                    // correct colour + big enough + fresh
  if (!visionFresh()) return false;
  if (vis.colour == 'R') return vis.area >= AVOID_MIN_AREA_R;
  if (vis.colour == 'G') return vis.area >= AVOID_MIN_AREA_G;
  return false;
}
// ============================================================
// HELPERS
// ============================================================
float lastServoDeg = SERVO_TRUE_STRAIGHT;   // last commanded servo (for failsafe reverse)
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
  lastServoDeg = angleDeg;
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
// ---- floor colour (direct register read, NO delay) ----
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
BlockColor rawFloorColor() {
  uint16_t r, g, b, c;
  readColor(r, g, b, c);
  float total = r + g + b;
  if (total <= 0) return COLOR_NONE;
  float pR = ((float)r / total) * 100.0;
  float pB = ((float)b / total) * 100.0;
  if (pB > 36.0 && pR < 24.0)      return COLOR_BLUE;
  if (pR > 35.0 && pB < 27.0)      return COLOR_ORANGE;
  return COLOR_NONE;
}
BlockColor detectColor(BlockColor wantColor) {
  BlockColor rawColor = rawFloorColor();
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
// cached sensor readings, refreshed every loop
bool          gImuFresh = false;
float         gHeading  = 0.0;
float         gYawRate  = 0.0;
float         gPrevH    = 0.0;
unsigned long gPrevHT   = 0;
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
  serviceVision();
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
// EASED TURN LAW  (shared by corner turns, realign, avoidance - NO settle)
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
bool turnArcStep(float target) {         // one arc step; true when finished
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
BlockColor dcWantFirst;
bool       dcFirstTurn;
bool       dcColorArmed;
long       dcBaseTicks;
long       dcLockoutTicks;
long       dcSafetyTicks;
unsigned long sideInvalidStart = 0;
float      turnTarget;
float      turnAmount;
BlockColor turnPartner;
bool       turnGotSecond;
uint8_t    lcPhase;
long       lcBaseTicks;
long       lcTargetTicks;
long       fsTargetTicks;
// ---- avoidance sub-FSM state ----
uint8_t  avoidPhase    = 0;   // 1 swerve,2 straighten,3 hold,4 return,5 realign,6 backup
char     avoidColour   = 'N';
float    avoidSteerDeg = AVOID_STEER_DEG;
float    avoidOutServo  = SERVO_TRUE_STRAIGHT;
float    avoidBackServo = SERVO_TRUE_STRAIGHT;
uint16_t avoidTargetMm  = AVOID_SIDE_NEAR_MM;
long     avoidRefTicks  = 0;
long     avoidOutTicks  = 0;
long     avoidFwdRef    = 0;
long     avoidBackupRef = 0;
float    backupServo    = SERVO_TRUE_STRAIGHT;
unsigned long avoidGoneStart = 0;
void goState(RobotState s) { currentState = s; entered = false; }
void finishLaneCorrect() {
  targetHeading = laneHeading;
  if (cornerCount >= TARGET_CORNERS) goState(STATE_FINAL_STRAIGHT);
  else                               goState(STATE_DRIVE_TO_CORNER);
}
// ============================================================
// OFFSET-BASED PILLAR AVOIDANCE
// ============================================================
uint16_t avoidTargetSideMm() {
  // pillar centred (|dx|->0)  => move a lot  => small target (NEAR wall)
  // pillar already off-side   => move little => large target (FAR wall)
  float k = fabs((float)vis.dx) / (float)DX_SPAN;
  k = constrain(k, 0.0f, 1.0f);
  float t = AVOID_SIDE_NEAR_MM + (AVOID_SIDE_FAR_MM - AVOID_SIDE_NEAR_MM) * k;
  if (t < SIDE_SAFE_MM) t = SIDE_SAFE_MM;
  return (uint16_t)t;
}
void avoidComputeServos() {
  bool green = (avoidColour == 'G');
  avoidOutServo  = green ? (SERVO_TRUE_STRAIGHT - avoidSteerDeg)   // GREEN -> swerve LEFT
                         : (SERVO_TRUE_STRAIGHT + avoidSteerDeg);  // RED   -> swerve RIGHT
  avoidBackServo = green ? (SERVO_TRUE_STRAIGHT + avoidSteerDeg)
                         : (SERVO_TRUE_STRAIGHT - avoidSteerDeg);
}
void startAvoid() {
  avoidColour   = vis.colour;
  avoidSteerDeg = AVOID_STEER_DEG;
  avoidComputeServos();
  avoidTargetMm  = avoidTargetSideMm();       // <-- the OFFSET, from the pillar's dx
  avoidRefTicks  = readEncoder();
  avoidFwdRef    = readEncoder();
  avoidGoneStart = 0;
  avoidPhase = 1;
  setServoAngle(avoidOutServo);
  setMotorSpeed(AVOID_PWM);
  Serial.print(F("[AVOID] "));
  Serial.print(avoidColour == 'G' ? F("GREEN->left") : F("RED->right"));
  Serial.print(F(" target=")); Serial.print(avoidTargetMm); Serial.println(F("mm"));
}
void avoidStep() {
  bool green = (avoidColour == 'G');
  // ---- FRONT FAILSAFE: about to clip -> reverse on last angle, boost, retry ----
  if (avoidPhase >= 1 && avoidPhase <= 4 &&
      tofFrontValid && tofFront < FRONT_STOP_MM &&
      absEnc(readEncoder() - avoidFwdRef) >= (long)(RECOVER_MIN_FWD_CM * TICKS_PER_CM)) {
    backupServo = lastServoDeg;
    setServoAngle(backupServo);
    setMotorSpeed(-AVOID_BACK_PWM);
    avoidBackupRef = readEncoder();
    avoidPhase = 6;
    Serial.print(F("[AVOID] front ")); Serial.print(tofFront); Serial.println(F("mm -> backing up"));
  }
  switch (avoidPhase) {
    case 1: {   // SWERVE to the computed offset; wall ToF is only a safety floor
      setServoAngle(avoidOutServo);
      setMotorSpeed(AVOID_PWM);
      uint16_t sMm  = green ? tofLeft      : tofRight;
      bool     sVal = green ? tofLeftValid : tofRightValid;
      long moved = absEnc(readEncoder() - avoidRefTicks);
      bool reached = sVal && (sMm <= avoidTargetMm);
      bool safety  = sVal && (sMm <= SIDE_SAFE_MM);
      bool capped  = moved >= (long)(AVOID_OUT_CAP_CM * TICKS_PER_CM);
      bool cleared = !pillarEngaged();          // pillar already left the frame
      if (reached || safety || capped || cleared) {
        avoidOutTicks  = moved;                 // remember for a symmetric return
        turnStartTicks = readEncoder();
        turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
        avoidPhase = 2;
        Serial.print(F("[AVOID] at offset, out=")); Serial.print(moved / TICKS_PER_CM); Serial.println(F("cm"));
      }
      break;
    }
    case 2:     // STRAIGHTEN onto lane heading (holding the offset)
      if (turnArcStep(laneHeading)) { avoidGoneStart = 0; avoidPhase = 3; }
      break;
    case 3: {   // HOLD until the colour clears the frame (debounced)
      updateHeadingPid(laneHeading);
      bool gone = !pillarEngaged();
      if (gone) { if (avoidGoneStart == 0) avoidGoneStart = millis(); }
      else        avoidGoneStart = 0;
      if (avoidGoneStart != 0 && millis() - avoidGoneStart >= AVOID_RELEASE_MS) {
        setServoAngle(avoidBackServo);
        setMotorSpeed(AVOID_PWM);
        avoidRefTicks = readEncoder();
        avoidPhase = 4;
        Serial.println(F("[AVOID] passed -> returning"));
      }
      break;
    }
    case 4:     // RETURN the SAME remembered displacement
      setServoAngle(avoidBackServo);
      setMotorSpeed(AVOID_PWM);
      if (absEnc(readEncoder() - avoidRefTicks) >= avoidOutTicks) {
        turnStartTicks = readEncoder();
        turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
        avoidPhase = 5;
      }
      break;
    case 5:     // REALIGN to centre and resume normal driving
      if (turnArcStep(laneHeading)) {
        avoidSteerDeg = AVOID_STEER_DEG;        // restore base swerve after crossing
        avoidPhase = 0;
        resetHeadingPid();
        setMotorSpeed(BASE_SPEED);
        Serial.println(F("[AVOID] done, back to centre"));
        goState(STATE_DRIVE_TO_CORNER);
      }
      break;
    case 6:     // BACKUP: reverse on last angle, then boost swerve and retry
      setServoAngle(backupServo);
      setMotorSpeed(-AVOID_BACK_PWM);
      if (absEnc(readEncoder() - avoidBackupRef) >= (long)(AVOID_BACKUP_CM * TICKS_PER_CM)) {
        avoidSteerDeg += AVOID_STEER_BOOST;
        if (avoidSteerDeg > AVOID_STEER_MAX) avoidSteerDeg = AVOID_STEER_MAX;
        avoidComputeServos();
        avoidRefTicks = readEncoder();
        avoidFwdRef   = readEncoder();
        setServoAngle(avoidOutServo);
        setMotorSpeed(AVOID_PWM);
        avoidPhase = 1;
        Serial.print(F("[AVOID] boosted swerve -> ")); Serial.println(avoidSteerDeg);
      }
      break;
  }
}
// ============================================================
// STATE: DRIVE TO CORNER
// ============================================================
void driveStep() {
  if (!entered) {
    entered = true;
    Serial.println(F("[FSM] DRIVE_TO_CORNER"));
    dcWantFirst = lockedColor;
    dcFirstTurn = (lockedColor == COLOR_NONE);
    resetColorDetector();
    resetHeadingPid();
    dcBaseTicks = readEncoder();
    setMotorSpeed(BASE_SPEED);
    dcColorArmed   = false;
    gapMeasured    = false;
    sideInvalidStart = 0;
    dcLockoutTicks = (cornerCount > 0) ? (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM) : 0;
    dcSafetyTicks  = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);
  }
  long straightTicks = absEnc(readEncoder() - dcBaseTicks);
  if (straightTicks >= dcSafetyTicks) {
    Serial.println(F("[FSM] WARN: no line within safety distance, retrying"));
    entered = false;
    return;
  }
  updateHeadingPid(targetHeading);
  if (straightTicks <= dcLockoutTicks) return;
  // ---- turn triggers (floor colour gate + wall) computed first ----
  bool expectedCW  = dcFirstTurn ? (lastFirstColor == COLOR_ORANGE) : clockwiseMode;
  bool sidePresent = expectedCW ? tofRok : tofLok;
  bool sideValid   = expectedCW ? tofRightValid : tofLeftValid;
  bool frontClose  = tofFrontValid && (tofFront <= FRONT_TURN_MM);
  // ---- OFFSET-BASED PILLAR AVOIDANCE (don't pre-empt a real corner) ----
  if (pillarEngaged() && !(frontClose && dcColorArmed)) {
    startAvoid();
    goState(STATE_AVOID);
    return;
  }
  if (!dcColorArmed) {
    BlockColor c = detectColor(dcWantFirst);
    if (c != COLOR_NONE) {
      dcColorArmed = true;
      cornerFirstTicks = readEncoder();
      if (dcFirstTurn) lastFirstColor = c;
      sideInvalidStart = 0;
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
  bool sideConfirmed = false;
  if (sidePresent && !sideValid) {
    if (sideInvalidStart == 0) sideInvalidStart = millis();
    if (millis() - sideInvalidStart >= SIDE_INVALID_MS) sideConfirmed = true;
  } else {
    sideInvalidStart = 0;
  }
  bool noRange = (!tofFok) && (!tofLok) && (!tofRok);
  if (frontClose || sideConfirmed || noRange) {
    if (frontClose)         Serial.println(F("  turn: FRONT close"));
    else if (sideConfirmed) Serial.println(expectedCW ? F("  turn: RIGHT wall gone") : F("  turn: LEFT wall gone"));
    else                    Serial.println(F("  turn: colour-only (no range sensors)"));
    if (lockedColor == COLOR_NONE) {
      lockedColor   = lastFirstColor;
      clockwiseMode = (lockedColor == COLOR_ORANGE);
      Serial.println(clockwiseMode
        ? F("[FSM] LOCKED ORANGE -> CLOCKWISE (right turns)")
        : F("[FSM] LOCKED BLUE -> COUNTERCLOCKWISE (left turns)"));
    }
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
    }
    goState(STATE_TURNING);
  }
}
// ============================================================
// STATE: TURNING
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
  }
  if (turnArcStep(turnTarget)) {
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
    zeroEncoder();
    goState(STATE_LANE_CORRECT);
  }
}
// ============================================================
// STATE: LANE CORRECT
// ============================================================
void laneCorrectStep() {
  if (!entered) {
    entered = true;
    float delta = lastGapCm - gapRefCm;
    float mag = fabs(delta);
    if (mag < GAP_DEADBAND_CM) {
      finishLaneCorrect();
      return;
    }
    float steerSigned = CORRECTION_SIGN * delta;
    if (clockwiseMode) steerSigned = -steerSigned;
    float offset = K_LAT_DEG_PER_CM * mag;
    if (offset > MAX_LAT_OFFSET_DEG) offset = MAX_LAT_OFFSET_DEG;
    float servo = (steerSigned > 0) ? (SERVO_TRUE_STRAIGHT - offset)
                                    : (SERVO_TRUE_STRAIGHT + offset);
    setServoAngle(servo);
    setMotorSpeed(CORRECTION_PWM);
    lcBaseTicks   = readEncoder();
    lcTargetTicks = (long)(CORRECTION_DISTANCE_CM * TICKS_PER_CM);
    lcPhase = 2;
    return;
  }
  if (lcPhase == 2) {
    if (absEnc(readEncoder() - lcBaseTicks) >= lcTargetTicks) {
      turnStartTicks = readEncoder();
      turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
      lcPhase = 4;
    }
  } else {
    if (turnArcStep(laneHeading)) finishLaneCorrect();
  }
}
// ============================================================
// STATE: FINAL STRAIGHT
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
// TELEMETRY  ->  Pi web page ("#" lines)
// ============================================================
const char *stateName(RobotState s) {
  switch (s) {
    case STATE_INIT:            return "INIT";
    case STATE_WAIT_START:      return "WAIT";
    case STATE_DRIVE_TO_CORNER: return "DRIVE";
    case STATE_TURNING:         return "TURN";
    case STATE_LANE_CORRECT:    return "LANE";
    case STATE_AVOID:           return "AVOID";
    case STATE_FINAL_STRAIGHT:  return "FINAL";
    case STATE_FINISHED:        return "DONE";
  }
  return "?";
}
unsigned long lastTelem = 0;
void telemetry() {
  if (millis() - lastTelem < 100) return;
  lastTelem = millis();
  BlockColor fc = rawFloorColor();
  PiSerial.print(F("# st=")); PiSerial.print(stateName(currentState));
  PiSerial.print(F(" av="));  PiSerial.print(avoidPhase);
  PiSerial.print(F(" cw="));  PiSerial.print(clockwiseMode ? 'R' : 'L');
  PiSerial.print(F(" sv="));  PiSerial.print((int)lastServoDeg);
  PiSerial.print(F(" hd="));  PiSerial.print(gHeading, 1);
  PiSerial.print(F(" F="));   PiSerial.print(tofFrontValid ? (int)tofFront : -1);
  PiSerial.print(F(" L="));   PiSerial.print(tofLeftValid  ? (int)tofLeft  : -1);
  PiSerial.print(F(" R="));   PiSerial.print(tofRightValid ? (int)tofRight : -1);
  PiSerial.print(F(" fl="));  PiSerial.print(fc == COLOR_ORANGE ? 'O' : fc == COLOR_BLUE ? 'B' : '-');
  PiSerial.print(F(" vis=")); PiSerial.print(visionFresh() ? vis.colour : 'x');
  PiSerial.print(F(" dx="));  PiSerial.print(vis.dx);
  PiSerial.print(F(" a="));   PiSerial.print(vis.area);
  PiSerial.print(F(" cn="));  PiSerial.print(cornerCount);
  PiSerial.println();
}
// ============================================================
// START BUTTON  (PA5 active-low, blocking wait - car not moving yet)
// ============================================================
void waitForStart() {
  Serial.println(F("[FSM] waiting for START button (PA5 -> GND)"));
  while (digitalRead(START_BTN_PIN) == HIGH) { delay(5); }   // wait for press (LOW)
  delay(30);
  while (digitalRead(START_BTN_PIN) == LOW)  { delay(5); }   // wait for release
  Serial.println(F("[FSM] START"));
}
// ============================================================
// MAIN
// ============================================================
void setup() {
  Serial.begin(115200);
  initHardware();
  delay(500);
}
void loop() {
  serviceSensors();
  switch (currentState) {
    case STATE_INIT:
      Serial.println(F("[FSM] INIT"));
      lockedColor   = COLOR_NONE;
      cornerCount   = 0;
      laneHeading   = readHeading();
      targetHeading = laneHeading;
      zeroEncoder();
      goState(STATE_WAIT_START);
      break;
    case STATE_WAIT_START:
      waitForStart();
      laneHeading   = readHeading();     // re-zero reference at the gun
      targetHeading = laneHeading;
      zeroEncoder();
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
  telemetry();
}
