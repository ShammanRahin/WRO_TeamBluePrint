#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>

// ============================================================
// OBSTACLE ROUND - NON-BLOCKING FIRMWARE
//
// Built from OpenRound.cpp (same FSM, pins, calibration, turn trigger,
// recovery, final straight) with the old obstacle-round logic
// (main.ino + tracker.py) folded into the driving:
//
//   old main.ino                      this file
//   ---------------------------------------------------------------
//   wall PD on ToF (right - left)  -> wall-centre PD on LiDAR R - L,
//                                     output is a heading offset that the
//                                     IMU heading PID tracks
//   vision PD, red -100 / green    -> same gains, same targets, same
//     +100 px, area blend 1000-1800   area blend
//   servo = 97 - blended (67..127) -> servo = 76.5 + blended (20..140)
//                                     sign flipped: on this linkage a
//                                     LOWER servo angle steers LEFT
//   sonar <50 cm hard steer        -> dropped
//   sonar <30 cm reverse (blocking)-> non-blocking RECOVER at 200 mm,
//                                     suppressed while the camera sees a
//                                     pillar (the pillar PD owns that)
//   magenta                        -> ignored (Pi never sends it)
//
// SERIAL FRAME (Pi -> STM32), one line per send, ~50 Hz:
//     left,front,right,rev,color,err,area,vseq\n
//   left/front/right  mm, 65535 = no return
//   rev               lidar revolution counter (unused here)
//   color             1 = red, 0 = green, 2 = none
//   err               pillar centre x - 320 (640 px frame), + = right
//   area              blob area in 320x240 detection pixels
//   vseq              camera frame counter - the vision PD only updates
//                     when this changes, so a camera frame resent in
//                     several serial lines can't spike the D term
// A 4-field open-round line (left,front,right,rev) still parses; vision
// then reads as "none".
//
// COMMANDS (Pi -> STM32), sent by the Start / Stop buttons on the Pi page:
//     S\n   START - only accepted while armed in WAIT_START (or FINISHED,
//            which re-arms and starts a fresh run) and the LiDAR is live
//     X\n   STOP  - motor off, steering centred, straight to FINISHED
//
// START SEQUENCE (no physical button)
//   power on -> wait for the first Pi frame (LED1 slow blink)
//            -> armed in WAIT_START (LED1 medium blink)
//            -> START from the page -> run
//
// PER CORNER
//   1. drive on IMU heading + wall centring + pillar PD
//   2. FIRST corner only: floor colour line arms the gate and sets
//      direction (orange = CW, blue = CCW)
//   3. turn fires when the turn-side distance reads above SIDE_OPEN_MM
//      for SIDE_OPEN_FRAMES consecutive NEW frames
//   4. 90 deg arc at TURN_LOCK_FRACTION of full lock (wider than the open
//      round), vision OFF during the arc, back to DRIVE right after
//      (vision ON again immediately, including the post-corner lockout)
//
// BENCH-VERIFIED (inherited from OpenRound.cpp)
//   motor    PA2 forward, PA3 reverse
//   encoder  TIM5 PA0/PA1, negated so forward counts up
//   IMU      BNO08x SPI1 ~100 Hz, clockwise = negative yaw
//   colour   TCS34725 CH4, white pR 47 / orange pR 69 / blue pB 27
//   servo    500-2500 us, straight 76.5, left stop 20, right stop 140
//            turning radius at full lock: left 27 cm, right 25 cm
// ============================================================

enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };

enum RobotState {
  STATE_WAIT_START,
  STATE_DRIVE_TO_CORNER,
  STATE_TURNING,
  STATE_FINAL_STRAIGHT,
  STATE_RECOVER,
  STATE_FINISHED
};

// ============================================================
// HARDWARE PINS & OBJECTS
// ============================================================
const int MOT_RPWM_PIN = PA2;     // forward  (TIM2_CH3; TIM5 is the encoder)
const int MOT_LPWM_PIN = PA3;     // reverse  (TIM2_CH4)
const int SERVO_PIN    = PA8;

const int IMU_CS_PIN  = PA4;
const int IMU_INT_PIN = PB0;
const int IMU_RST_PIN = PB1;

const int LED1_PIN = PB12;        // slow blink (500 ms) = waiting for Pi; medium blink (250 ms) = armed, waiting for START; solid = running; fast blink = lidar stale
const int LED2_PIN = PB13;        // lit while ORANGE is under the sensor
const int LED3_PIN = PB14;        // lit while BLUE is under the sensor

#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70
#define TCS_CH      4             // TCS34725

// ---- calibration ----
const float TICKS_PER_CM        = 14.853;

const int   SERVO_MIN_PULSE_US  = 500;
const int   SERVO_MAX_PULSE_US  = 2500;
const float SERVO_TRUE_STRAIGHT = 76.5;
const float SERVO_MAX_LEFT      = 20.0;   // left hard stop  (below straight steers LEFT)
const float SERVO_MAX_RIGHT     = 140.0;  // right hard stop (above straight steers RIGHT)
const float IMU_YAW_SIGN        = 1.0;    // clockwise reads negative

// Obstacle round: one constant PWM for driving, turning and reversing.
const int DRIVE_PWM = 60;


SPIClass SPI_IMU(PA7, PA6, PA5);  // MOSI, MISO, SCLK
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

bool  tcsOk = false;
float initialYawOffset = 0.0;

// ============================================================
// PI LINK   "left,front,right,rev,color,err,area,vseq\n"
// ============================================================
const unsigned long LIDAR_STALE_MS      = 200;
const unsigned long LIDAR_DEAD_MS       = 1000;
const uint16_t      LIDAR_MAX_VALID_MM  = 3500;   // mat diagonal
const uint16_t      LIDAR_FAR           = 9999;   // internal "nothing there"

// A beam with no return must read FAR, never near.
uint16_t lidarSanitize(long v) {
  if (v <= 0 || v > (long)LIDAR_MAX_VALID_MM) return LIDAR_FAR;
  return (uint16_t)v;
}

uint16_t      lidarL = LIDAR_FAR, lidarF = LIDAR_FAR, lidarR = LIDAR_FAR;
unsigned long lidarLastMs = 0;
bool          lidarStale  = true;
bool          lidarDead   = true;
uint32_t      lidarFrames = 0;
bool          lidarNewFrame = false;   // true only on the loop a frame was parsed

// vision fields, from the same line
const int VIS_RED   = 1;
const int VIS_GREEN = 0;
const int VIS_NONE  = 2;
int      visColor = VIS_NONE;
int      visErr   = 0;
long     visArea  = 0;
uint32_t visSeq   = 0;

char    lidarBuf[64];
uint8_t lidarLen = 0;

bool startRequested = false;
bool stopRequested  = false;

void parseLine() {
  // one-letter commands from the Pi page
  if (lidarBuf[0] == 'S' && lidarBuf[1] == '\0') { startRequested = true; return; }
  if (lidarBuf[0] == 'X' && lidarBuf[1] == '\0') { stopRequested  = true; return; }

  long f[8];
  uint8_t n = 0;
  char *p = lidarBuf;
  while (n < 8) {
    char *end;
    long v = strtol(p, &end, 10);
    if (end == p) break;             // no digits - malformed
    f[n++] = v;
    if (*end != ',') break;
    p = end + 1;
  }
  if (n < 3) return;                 // not even a lidar frame - drop

  lidarL = lidarSanitize(f[0]);
  lidarF = lidarSanitize(f[1]);
  lidarR = lidarSanitize(f[2]);

  if (n >= 8) {
    visColor = (f[4] == VIS_RED || f[4] == VIS_GREEN) ? (int)f[4] : VIS_NONE;
    visErr   = (int)f[5];
    visArea  = f[6];
    visSeq   = (uint32_t)f[7];
  } else {
    visColor = VIS_NONE;             // open-round feed: no camera
  }

  lidarLastMs   = millis();
  lidarStale    = false;
  lidarDead     = false;
  lidarFrames++;
  lidarNewFrame = true;
}

void serviceLidar() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lidarLen > 0) {
        lidarBuf[lidarLen] = '\0';
        parseLine();
        lidarLen = 0;
      }
    } else if (lidarLen < sizeof(lidarBuf) - 1) {
      lidarBuf[lidarLen++] = c;
    } else {
      lidarLen = 0;                 // overflow - drop and resync on newline
    }
  }
  unsigned long age = millis() - lidarLastMs;
  if (age > LIDAR_STALE_MS) lidarStale = true;
  if (age > LIDAR_DEAD_MS)  lidarDead  = true;
}

// ---- turn trigger tuning ----
const uint16_t SIDE_OPEN_MM     = 1500;  // turn side above this = inner wall gone
const uint8_t  SIDE_OPEN_FRAMES = 3;     // consecutive NEW frames before believing it

uint8_t sideOpenCount = 0;

// ---- wall recovery ----
const uint16_t WALL_PANIC_MM     = 200;
const uint16_t WALL_CLEAR_MM     = 350;
const int      RECOVER_PWM       = DRIVE_PWM;
const float    RECOVER_MAX_CM    = 30.0;
const int      RECOVER_MAX_TRIES = 3;

// ============================================================
// RUN CONSTANTS
// ============================================================
const int   TARGET_CORNERS      = 12;
const int   FINAL_STRAIGHT_CM   = 100;
const float SEARCH_SAFETY_CM    = 400.0;
const float POST_CORNER_LOCKOUT_CM = 50.0;

float firstSegmentCm      = 0.0;
float fullStartStraightCm = 0.0;
bool  haveFullStraight    = false;
float finalDistanceCm     = FINAL_STRAIGHT_CM;

// ============================================================
// FSM DATA
// ============================================================
bool fsmStarted = false;

RobotState    currentState = STATE_WAIT_START;
bool          entered = false;
unsigned long phaseT0 = 0;

BlockColor lockedColor   = COLOR_NONE;
bool       clockwiseMode = true;
int        cornerCount   = 0;

float targetHeading = 0.0;
float laneHeading   = 0.0;

BlockColor lastFirstColor = COLOR_NONE;

// cached IMU, refreshed every loop
bool          gImuFresh = false;
float         gHeading  = 0.0;
float         gYawRate  = 0.0;
float         gPrevH    = 0.0;
unsigned long gPrevHT   = 0;

// cached colour, refreshed every loop
BlockColor gRawColor = COLOR_NONE;

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

void resetTCA() {
  pinMode(TCA_RST_PIN, OUTPUT);
  digitalWrite(TCA_RST_PIN, LOW);
  delay(10);
  digitalWrite(TCA_RST_PIN, HIGH);
  delay(10);
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { analogWrite(MOT_RPWM_PIN, speed); analogWrite(MOT_LPWM_PIN, 0); }
  else if (speed < 0) { analogWrite(MOT_RPWM_PIN, 0);     analogWrite(MOT_LPWM_PIN, -speed); }
  else                { analogWrite(MOT_RPWM_PIN, 0);     analogWrite(MOT_LPWM_PIN, 0); }
}

float lastServoCmd = SERVO_TRUE_STRAIGHT;

void setServoAngle(float angleDeg) {
  angleDeg = constrain(angleDeg, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  lastServoCmd = angleDeg;
  int pulse = (int)((angleDeg / 180.0) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) + SERVO_MIN_PULSE_US;
  steeringServo.writeMicroseconds(pulse);
}

// TIM5 encoder, negated so driving forward counts up
void zeroEncoder() { TIM5->CNT = 0; }
long readEncoder() { return -(int32_t)TIM5->CNT; }
long absEnc(long v) { return v < 0 ? -v : v; }

bool turnIsClockwise() {
  return (lockedColor == COLOR_NONE) ? (lastFirstColor == COLOR_ORANGE) : clockwiseMode;
}
uint16_t turnSideMm() { return turnIsClockwise() ? lidarR : lidarL; }

// ---- IMU ----
float readYaw() {
  float qI = myIMU.getQuatI(), qJ = myIMU.getQuatJ();
  float qK = myIMU.getQuatK(), qReal = myIMU.getQuatReal();
  if (qI == 0.0f && qJ == 0.0f && qK == 0.0f && qReal == 0.0f) return 0.0f;
  float yawRadians = atan2(2.0f * (qI * qJ + qReal * qK),
                           (qReal * qReal + qI * qI - qJ * qJ - qK * qK));
  return yawRadians * (180.0 / PI);
}

float readHeading() {
  float h = fmod(readYaw() - initialYawOffset + 540.0, 360.0) - 180.0;
  return IMU_YAW_SIGN * h;
}

void zeroYaw() {
  Serial.println(F("# zeroing yaw"));
  unsigned long t = millis();
  while (millis() - t < 3000) {
    if (myIMU.wasReset()) myIMU.enableGameRotationVector();
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      initialYawOffset = readYaw();
      Serial.print(F("# zero yaw ")); Serial.println(initialYawOffset);
      return;
    }
    delay(10);
  }
  Serial.println(F("# ERROR no IMU event to zero"));
}

// ---- floor colour ----
void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (!tcsOk) { r = 0; g = 0; b = 0; c = 0; return; }
  tcaselect(TCS_CH);
  Wire.beginTransmission(0x29);
  Wire.write(0x80 | 0x20 | 0x14);      // command | auto-increment | CDATAL
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x29, (uint8_t)8);
  if (Wire.available() < 8) { r = 0; g = 0; b = 0; c = 0; return; }
  c  = (uint16_t)Wire.read();  c |= (uint16_t)Wire.read() << 8;
  r  = (uint16_t)Wire.read();  r |= (uint16_t)Wire.read() << 8;
  g  = (uint16_t)Wire.read();  g |= (uint16_t)Wire.read() << 8;
  b  = (uint16_t)Wire.read();  b |= (uint16_t)Wire.read() << 8;
}

BlockColor classifyColor() {
  uint16_t r, g, b, c;
  readColor(r, g, b, c);
  float total = (float)r + (float)g + (float)b;
  if (total < 100.0f) return COLOR_NONE;
  float pR = (r / total) * 100.0f;
  float pB = (b / total) * 100.0f;
  if (pR > 52.0f && pB < 18.0f) return COLOR_ORANGE;
  if (pB > 23.0f && pR < 40.0f) return COLOR_BLUE;
  return COLOR_NONE;
}

const unsigned long COLOR_CONFIRM_MS = 6;
BlockColor    pendingColor = COLOR_NONE;
unsigned long pendingStart = 0;

void resetColorDetector() { pendingColor = COLOR_NONE; pendingStart = 0; }

bool colorMuted    = false;
long colorMuteFrom = 0;

BlockColor detectColor(BlockColor wantColor) {
  if (colorMuted) { resetColorDetector(); return COLOR_NONE; }
  BlockColor rawColor = gRawColor;
  if (wantColor != COLOR_NONE && rawColor != wantColor) rawColor = COLOR_NONE;

  if (rawColor == COLOR_NONE)   { resetColorDetector(); return COLOR_NONE; }
  if (rawColor != pendingColor) { pendingColor = rawColor; pendingStart = millis(); return COLOR_NONE; }
  if (millis() - pendingStart >= COLOR_CONFIRM_MS) { resetColorDetector(); return rawColor; }
  return COLOR_NONE;
}

// ============================================================
// WALL CENTRING  (old main.ino wall PD, now on LiDAR)
// ============================================================
// err = right - left (mm), + = more room on the right = steer right.
// The output is a HEADING OFFSET, not a servo command: the heading PID
// tracks laneHeading - wallOffsetDeg, so centring and heading hold can't
// fight each other. Only trusted while BOTH sides see a wall inside
// WALL_VALID_MAX_MM (corridor is at most 1000 mm wide); in a corner, or
// with a side beam looking down the next straight, it fades to 0 and the
// car simply holds heading.
const float    WALL_KP          = 0.06;   // deg per mm       (200 mm off -> 12 deg)
const float    WALL_KD          = 0.004;  // deg per (mm/s)
const float    WALL_MAX_DEG     = 20.0;
const uint16_t WALL_VALID_MAX_MM = 1100;

float         wallOffsetDeg = 0.0;
float         wallPrevErr   = 0.0;
unsigned long wallPrevMs    = 0;
bool          wallHavePrev  = false;

void resetWallCentre() { wallOffsetDeg = 0.0; wallHavePrev = false; }

void updateWallCentre() {
  if (!lidarNewFrame) return;
  if (lidarStale || lidarL > WALL_VALID_MAX_MM || lidarR > WALL_VALID_MAX_MM) {
    resetWallCentre();
    return;
  }
  float err = (float)lidarR - (float)lidarL;
  unsigned long now = millis();
  float d = 0.0;
  if (wallHavePrev) {
    float dt = (now - wallPrevMs) / 1000.0;
    if (dt > 0.0) d = (err - wallPrevErr) / dt;
  }
  wallOffsetDeg = constrain(WALL_KP * err + WALL_KD * d, -WALL_MAX_DEG, WALL_MAX_DEG);
  wallPrevErr  = err;
  wallPrevMs   = now;
  wallHavePrev = true;
}

// ============================================================
// PILLAR PD  (old main.ino vision PD, unchanged gains)
// ============================================================
// Red   -> hold the pillar at -100 px (left of centre) -> pass on its RIGHT
// Green -> hold the pillar at +100 px (right of centre) -> pass on its LEFT
// visOffset is in servo degrees, + = steer RIGHT (pillar right of its
// target -> turn right -> it slides left in the image).
const int   VIS_RED_TARGET_PX   = -150;   // calibrated
const int   VIS_GREEN_TARGET_PX =  150;   // calibrated
const float VIS_KP              = 0.9;
const float VIS_KD              = 0.05;
const long  VIS_AREA_MIN        = 300;    // below this: vision weight 0
const long  VIS_AREA_START      = 1000;   // weight 0 here ...  (calibrated)
const long  VIS_AREA_FULL       = 5000;   // ... 1 here          (calibrated)

bool          visValid     = false;
float         visOffset    = 0.0;
float         visWeight    = 0.0;
int           visPrevErr   = 0;
int           visPrevColor = VIS_NONE;
unsigned long visPrevMs    = 0;
bool          visHavePrev  = false;
uint32_t      visLastSeq   = 0;

void resetVision() {
  visValid = false; visWeight = 0.0; visOffset = 0.0;
  visHavePrev = false; visPrevColor = VIS_NONE;
}

// A pillar is "seen" when the latest line carries red/green and it is fresh.
bool pillarSeen() { return !lidarStale && visColor != VIS_NONE; }

void updateVision() {
  if (!pillarSeen()) { resetVision(); return; }
  if (!lidarNewFrame) return;
  if (visValid && visSeq == visLastSeq) return;      // same camera frame resent
  visLastSeq = visSeq;

  int desired = (visColor == VIS_RED) ? VIS_RED_TARGET_PX : VIS_GREEN_TARGET_PX;
  int e = visErr - desired;
  unsigned long now = millis();

  float d = 0.0;
  if (visHavePrev && visPrevColor == visColor) {     // colour switch = new target, no D kick
    float dt = (now - visPrevMs) / 1000.0;
    if (dt > 0.0) d = VIS_KD * (e - visPrevErr) / dt;
  }
  visOffset = VIS_KP * e + d;

  if (visArea > VIS_AREA_MIN) {
    visWeight = (float)(visArea - VIS_AREA_START) / (float)(VIS_AREA_FULL - VIS_AREA_START);
    visWeight = constrain(visWeight, 0.0, 1.0);
  } else {
    visWeight = 0.0;
  }

  visPrevErr   = e;
  visPrevColor = visColor;
  visPrevMs    = now;
  visHavePrev  = true;
  visValid     = true;
}

// ---- called once at the top of every loop ----
void serviceSensors() {
  lidarNewFrame = false;
  serviceLidar();

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

  gRawColor = classifyColor();
  if (currentState != STATE_FINISHED) {
    digitalWrite(LED2_PIN, gRawColor == COLOR_ORANGE ? HIGH : LOW);
    digitalWrite(LED3_PIN, gRawColor == COLOR_BLUE   ? HIGH : LOW);
  }

  updateWallCentre();
  updateVision();
}

// ============================================================
// SYSTEM INITIALIZATION
// ============================================================
void initHardware() {
  pinMode(MOT_RPWM_PIN, OUTPUT);
  pinMode(MOT_LPWM_PIN, OUTPUT);
  setMotorSpeed(0);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);

  steeringServo.attach(SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  setServoAngle(SERVO_TRUE_STRAIGHT);

  // ---- TIM5 encoder on PA0 / PA1 (AF2) ----
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  TIM_Encoder_InitTypeDef sConfig = {0};
  static TIM_HandleTypeDef htim5 = {0};
  htim5.Instance         = TIM5;
  htim5.Init.Prescaler   = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period      = 0xFFFFFFFF;
  sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim5, &sConfig);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  // ---- I2C + colour ----
  resetTCA();
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  tcaselect(TCS_CH);
  delay(10);
  tcsOk = tcs.begin();
  Serial.println(tcsOk ? F("# colour CH4 READY") : F("# colour CH4 FAILED"));

  // ---- IMU over SPI1 ----
  SPI_IMU.begin();
  if (myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU)) {
    delay(500);
    myIMU.enableGameRotationVector();
    delay(100);
    myIMU.getSensorEvent();
    zeroYaw();
  } else {
    Serial.println(F("# ERROR IMU not found"));
  }
}

// ============================================================
// DRIVE STEERING  = heading PID (with wall-centre offset) blended with
//                   the pillar PD by pillar area - old main.ino blend
// ============================================================
const float HEAD_KP        = 2.0;
const float YAW_FILT_ALPHA = 0.35;
const float SERVO_SLEW     = 2.5;     // servo deg per IMU update (~100 Hz)
const float INTEGRAL_CLAMP = 300.0;
const float HEAD_KI        = 0.0;
const float HEAD_KD        = 0.0;

unsigned long pidPrevTime  = 0;
float         pidIntegral  = 0.0;
float         yawFilt      = 0.0;
float         prevServoCmd = SERVO_TRUE_STRAIGHT;

void resetHeadingPid() {
  pidPrevTime  = millis();
  pidIntegral  = 0.0;  yawFilt = 0.0;
  prevServoCmd = SERVO_TRUE_STRAIGHT;
}

// useWall / useVision let FINAL_STRAIGHT and DRIVE share one law.
void updateDriveSteer(float laneH, bool useWall, bool useVision) {
  if (!gImuFresh) return;
  unsigned long now = millis();
  yawFilt += YAW_FILT_ALPHA * (gYawRate - yawFilt);
  float dt = (now - pidPrevTime) / 1000.0;
  if (dt <= 0.0) dt = 0.001;

  // right-positive wall offset -> lower (more clockwise) target heading
  float target = useWall ? wrapDeg(laneH - wallOffsetDeg) : laneH;
  float error  = wrapDeg(target - gHeading);
  pidIntegral += error * dt;
  pidIntegral  = constrain(pidIntegral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
  float correction = HEAD_KP * error + HEAD_KI * pidIntegral - HEAD_KD * yawFilt;

  // both terms as servo offsets from straight, + = right
  float headOff = -correction;
  float w       = (useVision && visValid) ? visWeight : 0.0;
  float blended = (1.0 - w) * headOff + w * visOffset;

  float want = SERVO_TRUE_STRAIGHT + blended;
  float dcmd = constrain(want - prevServoCmd, -SERVO_SLEW, SERVO_SLEW);
  float cmd  = constrain(prevServoCmd + dcmd, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  if (cmd <= SERVO_MAX_LEFT || cmd >= SERVO_MAX_RIGHT) pidIntegral -= error * dt;
  setServoAngle(cmd);
  prevServoCmd = cmd;
  pidPrevTime  = now;
}

// ============================================================
// EASED TURN LAW - wider arc than the open round
// ============================================================
// Full lock is 27 cm (left) / 25 cm (right). The obstacle round needs
// room for pillars in the corner section, so the arc is capped at
// TURN_LOCK_FRACTION of each side's travel (0.70 -> roughly 35-38 cm
// radius; measure on the mat and tune this one number). The last
// TURN_MAX_STEER / TURN_KP degrees of heading error still ease off.
const float TURN_LOCK_FRACTION   = 0.70;
const float TURN_KP              = 2.5;
const float TURN_MAX_STEER_LEFT  = (SERVO_TRUE_STRAIGHT - SERVO_MAX_LEFT)  * TURN_LOCK_FRACTION;  // ~39.6
const float TURN_MAX_STEER_RIGHT = (SERVO_MAX_RIGHT - SERVO_TRUE_STRAIGHT) * TURN_LOCK_FRACTION;  // ~44.5
const float TURN_MIN_STEER       = 8.0;
const int   TURN_PWM             = DRIVE_PWM;
const float TURN_STOP_DEG        = 0.3;
const float TURN_CAP_CM          = 150.0;   // odometry backstop (wider arc = longer path)

long turnStartTicks = 0;
long turnCapTicks   = 0;

bool turnArcStep(float target) {
  if (absEnc(readEncoder() - turnStartTicks) >= turnCapTicks) return true;
  if (gImuFresh) {
    float err = wrapDeg(target - gHeading);
    if (fabs(err) < TURN_STOP_DEG) return true;
    float mag      = fabs(err);
    float maxSteer = (err > 0) ? TURN_MAX_STEER_LEFT : TURN_MAX_STEER_RIGHT;
    float steer    = constrain(TURN_KP * mag, TURN_MIN_STEER, maxSteer);
    float servo    = (err > 0) ? (SERVO_TRUE_STRAIGHT - steer) : (SERVO_TRUE_STRAIGHT + steer);
    setServoAngle(servo);
    setMotorSpeed(TURN_PWM);
  }
  return false;
}

// ============================================================
// STATE HELPERS + working vars
// ============================================================
void goState(RobotState s) { currentState = s; entered = false; }

BlockColor dcWantColor;
bool       dcColorArmed;
long       dcBaseTicks;
long       dcLockoutTicks;
long       dcSafetyTicks;

float turnTarget;
float turnAmount;

long    fsTargetTicks;

void finishCorner() {
  targetHeading = laneHeading;
  if (cornerCount >= TARGET_CORNERS) goState(STATE_FINAL_STRAIGHT);
  else                               goState(STATE_DRIVE_TO_CORNER);
}

// ============================================================
// STATE: RECOVER  (wall too close - back off, then resume)
// Same as the open round. Not entered while the camera sees a pillar -
// a short front reading is then the pillar, and the pillar PD handles it.
// ============================================================
RobotState recoverReturnState   = STATE_DRIVE_TO_CORNER;
bool       recoverReturnEntered = false;
long       recoverBaseTicks     = 0;
int        recoverTries         = 0;

void enterRecovery() {
  recoverReturnState   = currentState;
  recoverReturnEntered = entered;
  colorMuted    = true;
  colorMuteFrom = readEncoder();
  currentState  = STATE_RECOVER;
  entered = false;
}

void recoverStep() {
  if (!entered) {
    entered = true;
    setMotorSpeed(0);
    setServoAngle(2.0 * SERVO_TRUE_STRAIGHT - lastServoCmd);
    setMotorSpeed(-RECOVER_PWM);
    recoverBaseTicks = readEncoder();
    Serial.print(F("# RECOVER front=")); Serial.println(lidarF);
  }

  bool clear     = !lidarStale && (lidarF >= WALL_CLEAR_MM);
  bool backedFar = absEnc(readEncoder() - recoverBaseTicks)
                   >= (long)(RECOVER_MAX_CM * TICKS_PER_CM);
  if (!clear && !backedFar) return;

  setMotorSpeed(0);
  if (clear) { recoverTries = 0;  Serial.println(F("# recover clear")); }
  else       { recoverTries++;    Serial.print(F("# recover capped, try "));
               Serial.println(recoverTries); }

  currentState = recoverReturnState;
  entered      = recoverReturnEntered;
  resetHeadingPid();

  switch (recoverReturnState) {
    case STATE_DRIVE_TO_CORNER:
    case STATE_FINAL_STRAIGHT:
      setMotorSpeed(DRIVE_PWM);
      break;
    default:
      break;     // TURNING sets its own PWM each step
  }
}

// ============================================================
// STATE: WAIT START  (armed - waits for START from the Pi page)
// ============================================================
void waitStartStep() {
  if (!entered) {
    entered = true;
    startRequested = false;          // a START sent before arming is ignored
    setMotorSpeed(0);
    setServoAngle(SERVO_TRUE_STRAIGHT);
    Serial.println(F("# WAIT_START armed - press Start on the Pi page"));
  }
  digitalWrite(LED1_PIN, ((millis() / 250) & 1) ? HIGH : LOW);

  if (!startRequested) return;
  startRequested = false;

  if (lidarStale) {
    Serial.println(F("# START refused: lidar stale"));
    return;
  }

  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  lockedColor      = COLOR_NONE;
  lastFirstColor   = COLOR_NONE;
  clockwiseMode    = true;
  cornerCount      = 0;
  colorMuted       = false;
  recoverTries     = 0;
  firstSegmentCm   = 0.0;
  fullStartStraightCm = 0.0;
  haveFullStraight = false;
  finalDistanceCm  = FINAL_STRAIGHT_CM;
  laneHeading      = gHeading;
  targetHeading    = laneHeading;
  resetVision();
  zeroEncoder();
  Serial.println(F("# GO"));
  goState(STATE_DRIVE_TO_CORNER);
}

// ============================================================
// STATE: DRIVE TO CORNER
// Same corner logic as the open round; steering is now the blended
// heading + wall-centre + pillar law.
// ============================================================
void driveStep() {
  if (!entered) {
    entered = true;
    dcWantColor   = lockedColor;
    dcColorArmed  = false;
    sideOpenCount = 0;
    resetColorDetector();
    resetHeadingPid();
    resetWallCentre();
    dcBaseTicks = readEncoder();
    setMotorSpeed(DRIVE_PWM);
    Serial.println(F("# DRIVE"));

    dcLockoutTicks = (cornerCount > 0) ? (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM) : 0;
    dcSafetyTicks  = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);
  }

  long straightTicks = absEnc(readEncoder() - dcBaseTicks);
  if (straightTicks >= dcSafetyTicks) {
    Serial.println(F("# WARN no turn trigger within safety distance, retrying"));
    entered = false;
    return;
  }

  // Vision is live from the first tick after the arc, lockout included -
  // a pillar right after the corner is handled immediately.
  updateDriveSteer(targetHeading, true, true);

  if (absEnc(readEncoder()) <= dcLockoutTicks) return;

  bool directionLocked = (lockedColor != COLOR_NONE);

  // ---- 1. colour gate: first corner, or lidar-dead fallback ----
  if (!dcColorArmed && (!directionLocked || lidarDead)) {
    BlockColor c = detectColor(dcWantColor);
    if (c != COLOR_NONE) {
      dcColorArmed = true;
      if (!directionLocked) lastFirstColor = c;
      sideOpenCount = 0;
      resetColorDetector();
      Serial.print(F("# gate ")); Serial.print(c == COLOR_ORANGE ? F("ORANGE") : F("BLUE"));
      Serial.print(F(" side=")); Serial.println(turnSideMm());
    }
  }
  if (!directionLocked && !dcColorArmed) return;

  // ---- 2. turn confirm: turn side > SIDE_OPEN_MM on consecutive NEW frames ----
  uint16_t sideNow = turnSideMm();
  if (lidarStale) {
    sideOpenCount = 0;
  } else if (lidarNewFrame) {
    if (sideNow > SIDE_OPEN_MM) { if (sideOpenCount < 250) sideOpenCount++; }
    else                        sideOpenCount = 0;
  }

  bool sideOpen = (sideOpenCount >= SIDE_OPEN_FRAMES);

  if (sideOpen || (lidarDead && dcColorArmed)) {
    if (sideOpen) { Serial.print(F("# turn: side open ")); Serial.println(sideNow); }
    else          Serial.println(F("# turn: colour only (lidar dead)"));

    if (!directionLocked) {
      lockedColor   = lastFirstColor;
      clockwiseMode = (lockedColor == COLOR_ORANGE);
      Serial.println(clockwiseMode ? F("# LOCKED CW (orange)") : F("# LOCKED CCW (blue)"));
    }

    float segCm = absEnc(readEncoder()) / TICKS_PER_CM;
    if (cornerCount == 0) {
      firstSegmentCm = segCm;
      Serial.print(F("# A=")); Serial.println(firstSegmentCm);
    } else if (cornerCount == 4) {
      fullStartStraightCm = segCm; haveFullStraight = true;
    } else if (cornerCount == 8 && haveFullStraight) {
      fullStartStraightCm = 0.5 * (fullStartStraightCm + segCm);
    }
    if (haveFullStraight) {
      finalDistanceCm = fullStartStraightCm - firstSegmentCm;
      if (finalDistanceCm < 0) finalDistanceCm = 0;
      Serial.print(F("# L=")); Serial.print(fullStartStraightCm);
      Serial.print(F(" final=")); Serial.println(finalDistanceCm);
    }

    goState(STATE_TURNING);
  }
}

// ============================================================
// STATE: TURNING  (wider eased 90 deg arc, vision off)
// ============================================================
void turningStep() {
  if (!entered) {
    entered = true;
    cornerCount++;
    Serial.print(F("# TURN ")); Serial.print(cornerCount);
    Serial.print('/'); Serial.println(TARGET_CORNERS);
    turnAmount     = clockwiseMode ? 90.0 : -90.0;
    turnTarget     = wrapDeg(laneHeading - turnAmount);
    turnStartTicks = readEncoder();
    turnCapTicks   = (long)(TURN_CAP_CM * TICKS_PER_CM);
  }

  if (turnArcStep(turnTarget)) {
    laneHeading = wrapDeg(laneHeading - turnAmount);
    zeroEncoder();
    Serial.print(F("# lane heading ")); Serial.println(laneHeading);
    finishCorner();
  }
}

// ============================================================
// STATE: FINAL STRAIGHT  (drive L - A from the end of turn 12)
// Pillars can sit in the start section, so the full steering law runs.
// ============================================================
void finalStraightStep() {
  if (!entered) {
    entered = true;
    Serial.print(F("# FINAL_STRAIGHT ")); Serial.println(finalDistanceCm);
    resetHeadingPid();
    resetWallCentre();
    setMotorSpeed(DRIVE_PWM);
    fsTargetTicks = (long)(finalDistanceCm * TICKS_PER_CM);
  }
  updateDriveSteer(laneHeading, true, true);
  if (absEnc(readEncoder()) >= fsTargetTicks) {
    setMotorSpeed(0);
    setServoAngle(SERVO_TRUE_STRAIGHT);
    goState(STATE_FINISHED);
  }
}

// ============================================================
// MAIN
// ============================================================
void setup() {
  Serial.begin(115200);
  initHardware();
}

void loop() {
  serviceSensors();

  // ---- startup gate: nothing runs until the Pi's first frame ----
  if (!fsmStarted) {
    if (lidarFrames == 0) {
      digitalWrite(LED1_PIN, ((millis() / 500) & 1) ? HIGH : LOW);
      return;
    }
    fsmStarted = true;
    Serial.println(F("# first Pi frame received - FSM starting"));
  }

  // START only means something while armed or finished. Anything else -
  // a press mid-run, or the repeat copies of the press that started this
  // run - is discarded so it can't auto-restart the car when it finishes.
  if (currentState != STATE_WAIT_START && currentState != STATE_FINISHED)
    startRequested = false;

  // STOP from the page: works in every state.
  if (stopRequested) {
    stopRequested = false;
    if (currentState != STATE_FINISHED) {
      Serial.println(F("# STOP from Pi"));
      goState(STATE_FINISHED);
    }
  }

  if (colorMuted && currentState != STATE_RECOVER && readEncoder() >= colorMuteFrom) {
    colorMuted = false;
    Serial.println(F("# colour re-enabled"));
  }

  // Wall panic overlay. Skipped while the camera sees a pillar: the short
  // front reading is the pillar, and the pillar PD is steering round it.
  if (!lidarStale &&
      !pillarSeen() &&
      currentState != STATE_RECOVER &&
      currentState != STATE_WAIT_START &&
      currentState != STATE_FINISHED &&
      recoverTries < RECOVER_MAX_TRIES &&
      lidarF <= WALL_PANIC_MM) {
    enterRecovery();
  }

  if (currentState != STATE_WAIT_START && currentState != STATE_FINISHED) {
    digitalWrite(LED1_PIN, lidarStale ? (((millis() / 100) & 1) ? HIGH : LOW) : HIGH);
  }

  switch (currentState) {
    case STATE_WAIT_START:      waitStartStep();     break;
    case STATE_DRIVE_TO_CORNER: driveStep();         break;
    case STATE_TURNING:         turningStep();       break;
    case STATE_FINAL_STRAIGHT:  finalStraightStep(); break;
    case STATE_RECOVER:         recoverStep();       break;

    case STATE_FINISHED:
      if (!entered) {
        entered = true;
        Serial.println(F("# FINISHED"));
        startRequested = false;      // only a NEW press restarts
        setMotorSpeed(0);
        setServoAngle(SERVO_TRUE_STRAIGHT);
        digitalWrite(LED1_PIN, HIGH);
        digitalWrite(LED2_PIN, HIGH);
        digitalWrite(LED3_PIN, HIGH);
      }
      // START again from the page re-arms and runs a fresh 3 laps
      // (put the car back in the start section first).
      if (startRequested) {
        goState(STATE_WAIT_START);   // WAIT_START clears the flag on entry,
        waitStartStep();             // so arm first ...
        startRequested = true;       // ... then honour this press
      }
      break;
  }
}
