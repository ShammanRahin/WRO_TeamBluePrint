// ============================================================================
// ObstacleLap.cpp — WRO Future Engineers obstacle round, LAP ONLY.
// (No parallel parking, no magenta, no start button, no rear ToF.)
//
// This is OpenRound.cpp with ONE state added. Everything that was already
// calibrated on this car — servo trim and limits, TICKS_PER_CM, the heading
// PID, the eased 90 deg arc, the colour thresholds, the wall recovery — is
// unchanged and still runs the car. Do not port this to ObstacleExecutor.cpp:
// that file's pin map (PB9/PB8 motor, TIM3 encoder, IMU on PB5/4/3) is from an
// earlier build and does not match this hardware.
//
// SPLIT OF WORK
//   Pi      camera + lidar, pillar tracking, and the avoidance solve. It sends
//           a solved manoeuvre: "hold this absolute heading for this many mm".
//   STM32   this file: the state machine, the heading PID at IMU rate, the
//           turn arc, odometry, the watchdog. Every fast loop is here because
//           a 50 Hz link cannot close a heading loop.
//
// STATES
//   BOOT     nothing happens until the Pi says its lidar AND camera are up.
//   HEADING  hold the lane heading. Watch for a corner, a pillar, the finish.
//   AVOID    hold the heading the Pi solved, for the distance it asked for.
//   TURN90   eased 90 deg arc, terminated on IMU heading.
//   FINISH   stopped.
//   RECOVER  overlay: something is too close in front; back off and resume.
//
// LINK  (docs/OBSTACLE_LAP.md)
//   Pi  -> us   PERCEPT  16 bytes, sync AA 55, 50 Hz
//   us  -> Pi   TELEM    22 bytes, sync 55 AA, 50 Hz
//   The '#' log lines below share the port. 0xAA is not ASCII, so a log line
//   can never contain the TELEM sync word and the Pi separates the two cleanly.
//
// BENCH-VERIFIED (unchanged from the open round)
//   motor    PA2 forward, PA3 reverse
//   encoder  TIM5 PA0/PA1, negated so forward counts up
//   IMU      BNO08x SPI1 ~100 Hz, clockwise = negative yaw
//   colour   TCS34725 CH4, white pR 47 / orange pR 69 / blue pB 27
//   servo    500-2500 us, straight 76.5, left stop 20, right stop 140
//            turning radius at full lock: left 27 cm, right 25 cm
// ============================================================================

#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>

enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };

// Must match STATE_NAMES / ST_* in control/percept_link.py.
enum RobotState {
  STATE_BOOT = 0,
  STATE_HEADING,
  STATE_AVOID,
  STATE_TURN90,
  STATE_FINISH,
  STATE_RECOVER
};

// ============================================================================
// HARDWARE PINS & OBJECTS
// ============================================================================
const int MOT_RPWM_PIN = PA2;     // forward  (TIM2_CH3; TIM5 is the encoder)
const int MOT_LPWM_PIN = PA3;     // reverse  (TIM2_CH4)
const int SERVO_PIN    = PA8;

const int IMU_CS_PIN  = PA4;
const int IMU_INT_PIN = PB0;
const int IMU_RST_PIN = PB1;

const int LED1_PIN = PB12;   // slow blink = waiting on Pi; solid = running; fast = lidar stale
const int LED2_PIN = PB13;   // ORANGE under the floor sensor
const int LED3_PIN = PB14;   // BLUE under the floor sensor
const int BTN_START_PIN = PB15;   // not used this round

#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70
#define TCS_CH      4

// ---- calibration ----
const float TICKS_PER_CM        = 14.853;
const float TICKS_PER_MM        = TICKS_PER_CM / 10.0;
const float MM_PER_TICK         = 10.0 / TICKS_PER_CM;

const int   SERVO_MIN_PULSE_US  = 500;
const int   SERVO_MAX_PULSE_US  = 2500;
const float SERVO_TRUE_STRAIGHT = 76.5;
const float SERVO_MAX_LEFT      = 20.0;   // below straight steers LEFT
const float SERVO_MAX_RIGHT     = 140.0;  // above straight steers RIGHT
const float IMU_YAW_SIGN        = 1.0;

const int BASE_SPEED  = 70;    // straights
const int AVOID_SPEED = 60;    // threading a pillar: slower buys solve accuracy

const unsigned long START_DELAY_MS = 5000;

SPIClass SPI_IMU(PA7, PA6, PA5);  // MOSI, MISO, SCLK
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

bool  tcsOk = false;
float initialYawOffset = 0.0;

// ============================================================================
// LINK — PERCEPT in, TELEM out
// ============================================================================
const uint8_t PERCEPT_SYNC0 = 0xAA, PERCEPT_SYNC1 = 0x55;
const uint8_t TELEM_SYNC0   = 0x55, TELEM_SYNC1   = 0xAA;
const uint8_t PERCEPT_LEN = 16, TELEM_LEN = 22;

const uint8_t P_LIDAR_OK = 0x01;
const uint8_t P_CAM_OK   = 0x02;
const uint8_t P_AVOID_MASK = 0x0C, P_AVOID_SHIFT = 2;
const uint8_t P_GREEN    = 0x10;
const uint8_t P_HELLO    = 0x20;

const uint8_t AVOID_NONE = 0, AVOID_TRACK = 1, AVOID_COMMIT = 2;

const uint8_t S_RUNNING = 0x01, S_LIDAR_STALE = 0x02, S_LIDAR_DEAD = 0x04;
const uint8_t S_DIR_LOCKED = 0x08, S_CLOCKWISE = 0x10, S_IMU_OK = 0x20;
const uint8_t S_COLOUR_OK = 0x40, S_RECOVERING = 0x80;

// Two thresholds on purpose. STALE (short) just stops us acting on old
// distances. DEAD (long) is what disables the lidar corner trigger — a single
// dropped batch of frames must not be read as "lidar gone".
const unsigned long LIDAR_STALE_MS     = 200;
const unsigned long LIDAR_DEAD_MS      = 1000;
const unsigned long LINK_STALE_MS      = 250;    // Pi quiet: stop trusting TRACK
const uint16_t      LIDAR_MAX_VALID_MM = 3500;   // mat diagonal
const uint16_t      LIDAR_FAR          = 9999;   // internal "nothing there"
const uint32_t      TELEM_PERIOD_MS    = 20;     // 50 Hz

// A beam with no return must read FAR, never near. A dropout treated as 0 mm
// would latch the wall-panic recovery on permanently.
uint16_t lidarSanitize(uint16_t v) {
  if (v == 0 || v == 0xFFFF || v > LIDAR_MAX_VALID_MM) return LIDAR_FAR;
  return v;
}

uint16_t      lidarL = LIDAR_FAR, lidarF = LIDAR_FAR, lidarR = LIDAR_FAR;
uint8_t       lidarRev = 0, lastRev = 0;
// Two clocks, not one. lidarLastMs only advances on frames the Pi marked
// LIDAR_OK, so a frame carrying a committed leg but no ranges cannot clear the
// stale flag and make LIDAR_FAR side readings look like an open corner.
unsigned long lidarLastMs = 0, linkLastMs = 0;
bool          lidarStale = true, lidarDead = true, linkStale = true;
uint32_t      perceptFrames = 0;
uint8_t       rxSeq = 0;

// latest solved manoeuvre from the Pi
uint8_t pAction = AVOID_NONE;
bool    pGreen = false, pLidarOk = false, pCamOk = false, pHello = false;
float   pHeadingDeg = 0.0;
uint16_t pLegMm = 0;

uint8_t rxBuf[PERCEPT_LEN];
uint8_t rxLen = 0;

static inline uint8_t xor8(const uint8_t *p, uint8_t n) {
  uint8_t x = 0; while (n--) x ^= *p++;
  return x;
}

void applyPercept(const uint8_t *f) {
  rxSeq          = f[2];
  uint8_t flags  = f[3];
  pLidarOk       = flags & P_LIDAR_OK;
  pCamOk         = flags & P_CAM_OK;
  pHello         = flags & P_HELLO;
  pGreen         = flags & P_GREEN;
  pAction        = (flags & P_AVOID_MASK) >> P_AVOID_SHIFT;

  uint16_t l, fr, r, leg; int16_t head_dd;
  memcpy(&l,       f + 4,  2);
  memcpy(&fr,      f + 6,  2);
  memcpy(&r,       f + 8,  2);
  lidarRev = f[10];
  memcpy(&head_dd, f + 11, 2);
  memcpy(&leg,     f + 13, 2);

  pHeadingDeg = head_dd / 10.0;
  pLegMm      = leg;

  if (pLidarOk) {
    lidarL = lidarSanitize(l);
    lidarF = lidarSanitize(fr);
    lidarR = lidarSanitize(r);
    lidarLastMs = millis();
    lidarStale = lidarDead = false;
  }
  linkLastMs = millis();
  linkStale = false;
  perceptFrames++;
}

void serviceLink() {
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (rxLen == 0) { if (c == PERCEPT_SYNC0) rxBuf[rxLen++] = c; continue; }
    if (rxLen == 1) {
      if (c == PERCEPT_SYNC1) rxBuf[rxLen++] = c;
      else rxLen = (c == PERCEPT_SYNC0) ? 1 : 0;   // resync without losing a sync
      continue;
    }
    rxBuf[rxLen++] = c;
    if (rxLen == PERCEPT_LEN) {
      if (xor8(rxBuf + 2, PERCEPT_LEN - 3) == rxBuf[PERCEPT_LEN - 1]) applyPercept(rxBuf);
      rxLen = 0;                                   // bad checksum: drop silently
    }
  }
  unsigned long age = millis() - lidarLastMs;
  if (age > LIDAR_STALE_MS) lidarStale = true;
  if (age > LIDAR_DEAD_MS)  lidarDead  = true;
  if (millis() - linkLastMs > LINK_STALE_MS) linkStale = true;
}

// ============================================================================
// RUN CONSTANTS
// ============================================================================
const uint16_t SIDE_OPEN_MM     = 1500;  // turn side above this = inner wall gone
const uint8_t  SIDE_OPEN_REVS   = 3;     // consecutive lidar REVOLUTIONS, not frames

const uint16_t WALL_PANIC_MM     = 200;
const uint16_t WALL_CLEAR_MM     = 350;
const int      RECOVER_PWM       = 90;
const float    RECOVER_MAX_CM    = 30.0;
const int      RECOVER_MAX_TRIES = 3;

const int   TARGET_CORNERS         = 12;
const float SEARCH_SAFETY_CM       = 400.0;
const float POST_CORNER_LOCKOUT_CM = 50.0;
const float POST_AVOID_LOCKOUT_CM  = 30.0;

const float    AVOID_MAX_LEG_CM   = 150.0;   // backstop on a runaway leg
const uint32_t AVOID_TIMEOUT_MS   = 5000;
const uint16_t FINISH_TOL_MM      = 60;      // front-distance match tolerance
const float    FINAL_FALLBACK_CM  = 100.0;

// ============================================================================
// FSM DATA
// ============================================================================
bool fsmStarted = false;
unsigned long firstFrameMs = 0;
const unsigned long CAMERA_GRACE_MS = 15000;  // start without the camera after this

RobotState    currentState = STATE_BOOT;
bool          entered = false;
unsigned long phaseT0 = 0;

BlockColor lockedColor   = COLOR_NONE;
bool       clockwiseMode = true;
int        cornerCount   = 0;

float targetHeading = 0.0;
float laneHeading   = 0.0;
// The first corner's line decides the driving direction, and it can fall under
// the car in the middle of an avoid. Latching it at run level (not per-straight)
// means the direction still locks if that happens.
BlockColor lastFirstColor = COLOR_NONE;
bool       firstLineSeen  = false;

uint16_t frontAtStartMm = LIDAR_FAR;   // the finish line, measured not guessed

// cached sensors, refreshed every loop
bool          gImuFresh = false;
float         gHeading = 0.0, gYawRate = 0.0, gPrevH = 0.0;
unsigned long gPrevHT = 0;
BlockColor    gRawColor = COLOR_NONE;

// Odometry that NEVER resets. zeroEncoder() still zeroes TIM5 for the open
// round's per-segment measurements, so the Pi needs its own monotonic count.
long gOdoTicks = 0, gLastRawEnc = 0;
float gSpeedMmps = 0.0;

// ============================================================================
// HELPERS
// ============================================================================
float wrapDeg(float a) {
  while (a > 180.0)  a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

void tcaselect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR); Wire.write(1 << ch); Wire.endTransmission();
}

void resetTCA() {
  pinMode(TCA_RST_PIN, OUTPUT);
  digitalWrite(TCA_RST_PIN, LOW);  delay(10);
  digitalWrite(TCA_RST_PIN, HIGH); delay(10);
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
  int pulse = (int)((angleDeg / 180.0) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US))
              + SERVO_MIN_PULSE_US;
  steeringServo.writeMicroseconds(pulse);
}

long readEncoder() { return -(int32_t)TIM5->CNT; }
long absEnc(long v) { return v < 0 ? -v : v; }

void serviceOdo() {
  long raw = readEncoder();
  gOdoTicks += (raw - gLastRawEnc);
  gLastRawEnc = raw;
}
void zeroEncoder() { TIM5->CNT = 0; gLastRawEnc = 0; }

// Clockwise means the inner block is on the right, so the RIGHT side gives way
// at the corner. Counter-clockwise, the left.
bool turnIsClockwise() {
  return (lockedColor == COLOR_NONE) ? (lastFirstColor == COLOR_ORANGE) : clockwiseMode;
}
uint16_t turnSideMm() { return turnIsClockwise() ? lidarR : lidarL; }

// ---- IMU ----
float readYaw() {
  float qI = myIMU.getQuatI(), qJ = myIMU.getQuatJ();
  float qK = myIMU.getQuatK(), qReal = myIMU.getQuatReal();
  if (qI == 0.0f && qJ == 0.0f && qK == 0.0f && qReal == 0.0f) return 0.0f;
  return atan2(2.0f * (qI * qJ + qReal * qK),
               (qReal * qReal + qI * qI - qJ * qJ - qK * qK)) * (180.0 / PI);
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
  if (!tcsOk) { r = g = b = c = 0; return; }
  tcaselect(TCS_CH);
  Wire.beginTransmission(0x29);
  Wire.write(0x80 | 0x20 | 0x14);      // command | auto-increment | CDATAL
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x29, (uint8_t)8);
  if (Wire.available() < 8) { r = g = b = c = 0; return; }
  c  = (uint16_t)Wire.read();  c |= (uint16_t)Wire.read() << 8;
  r  = (uint16_t)Wire.read();  r |= (uint16_t)Wire.read() << 8;
  g  = (uint16_t)Wire.read();  g |= (uint16_t)Wire.read() << 8;
  b  = (uint16_t)Wire.read();  b |= (uint16_t)Wire.read() << 8;
}

// Calibrated on the mat: white pR 47 / pB 19, orange pR 69 / pB 11,
// blue pR 36 / pB 27. Orange is tested first.
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

// Backing away from a wall drags the sensor back over lines already used.
// Muting from the start of the reverse until we are forward of that point
// again means those re-crossings are ignored. gRawColor is NOT muted, so the
// LEDs still show what is underneath.
bool colorMuted = false;
long colorMuteFrom = 0;

BlockColor detectColor(BlockColor wantColor) {
  if (colorMuted) { resetColorDetector(); return COLOR_NONE; }
  BlockColor raw = gRawColor;
  if (wantColor != COLOR_NONE && raw != wantColor) raw = COLOR_NONE;
  if (raw == COLOR_NONE)   { resetColorDetector(); return COLOR_NONE; }
  if (raw != pendingColor) { pendingColor = raw; pendingStart = millis(); return COLOR_NONE; }
  if (millis() - pendingStart >= COLOR_CONFIRM_MS) { resetColorDetector(); return raw; }
  return COLOR_NONE;
}

void serviceSensors() {
  serviceLink();
  serviceOdo();

  gImuFresh = false;
  if (myIMU.wasReset()) myIMU.enableGameRotationVector();
  if (myIMU.getSensorEvent() &&
      myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    gImuFresh = true;
    float h = readHeading();
    unsigned long now = millis();
    float dt = (now - gPrevHT) / 1000.0;
    if (dt > 0.0) gYawRate = wrapDeg(h - gPrevH) / dt;
    gPrevH = h; gPrevHT = now; gHeading = h;
  }

  gRawColor = classifyColor();
  if (currentState != STATE_FINISH) {
    digitalWrite(LED2_PIN, gRawColor == COLOR_ORANGE ? HIGH : LOW);
    digitalWrite(LED3_PIN, gRawColor == COLOR_BLUE   ? HIGH : LOW);
  }
}

// ============================================================================
// TELEMETRY
// ============================================================================
long  avoidBaseTicks = 0, avoidLegTicks = 0;

void sendTelemetry() {
  static long prevOdo = 0;
  static uint32_t prevT = 0;
  uint32_t now = millis();
  if (prevT && now > prevT)
    gSpeedMmps = (gOdoTicks - prevOdo) * MM_PER_TICK * 1000.0 / (now - prevT);
  prevOdo = gOdoTicks; prevT = now;

  uint8_t f[TELEM_LEN];
  f[0] = TELEM_SYNC0; f[1] = TELEM_SYNC1;
  f[2] = rxSeq;

  uint8_t st = 0;
  if (fsmStarted && currentState != STATE_FINISH) st |= S_RUNNING;
  if (lidarStale)                st |= S_LIDAR_STALE;
  if (lidarDead)                 st |= S_LIDAR_DEAD;
  if (lockedColor != COLOR_NONE) st |= S_DIR_LOCKED;
  if (clockwiseMode)             st |= S_CLOCKWISE;
  if (gPrevHT != 0)              st |= S_IMU_OK;
  if (tcsOk)                     st |= S_COLOUR_OK;
  if (currentState == STATE_RECOVER) st |= S_RECOVERING;
  f[3] = st;

  int32_t  odoMm   = (int32_t)(gOdoTicks * MM_PER_TICK);
  int16_t  spd     = (int16_t)gSpeedMmps;
  int16_t  head_dd = (int16_t)(gHeading * 10.0);
  int16_t  yaw_dd  = (int16_t)(gYawRate * 10.0);
  uint16_t frontMm = (lidarF == LIDAR_FAR) ? 0xFFFF : lidarF;
  uint8_t  state   = (uint8_t)currentState;
  uint8_t  corners = (uint8_t)cornerCount;
  long     rem     = (currentState == STATE_AVOID)
                     ? (avoidLegTicks - (gOdoTicks - avoidBaseTicks)) : 0;
  uint16_t remMm   = (rem > 0) ? (uint16_t)min((long)0xFFFE, (long)(rem * MM_PER_TICK)) : 0;
  uint8_t  floorC  = (gRawColor == COLOR_ORANGE) ? 1 : (gRawColor == COLOR_BLUE ? 2 : 0);

  memcpy(f + 4,  &odoMm,   4);
  memcpy(f + 8,  &spd,     2);
  memcpy(f + 10, &head_dd, 2);
  memcpy(f + 12, &yaw_dd,  2);
  memcpy(f + 14, &frontMm, 2);
  f[16] = state;
  f[17] = corners;
  memcpy(f + 18, &remMm,   2);
  f[20] = floorC;
  f[21] = xor8(f + 2, TELEM_LEN - 3);

  Serial.write(f, TELEM_LEN);
}

// ============================================================================
// HEADING PID  (unchanged from the open round)
// ============================================================================
const float HEAD_KP        = 2.0;
const float YAW_FILT_ALPHA = 0.35;
const float SERVO_SLEW     = 2.5;
const float INTEGRAL_CLAMP = 300.0;
const float HEAD_KI        = 0.0;
const float HEAD_KD        = 0.0;

unsigned long pidPrevTime = 0;
float pidIntegral = 0.0, yawFilt = 0.0, prevServoCmd = SERVO_TRUE_STRAIGHT;

void resetHeadingPid() {
  pidPrevTime = millis();
  pidIntegral = 0.0; yawFilt = 0.0;
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
  pidPrevTime  = now;
}

// ============================================================================
// EASED TURN LAW  (unchanged from the open round)
// ============================================================================
const float TURN_KP              = 2.5;
const float TURN_MAX_STEER_LEFT  = SERVO_TRUE_STRAIGHT - SERVO_MAX_LEFT;   // 56.5
const float TURN_MAX_STEER_RIGHT = SERVO_MAX_RIGHT - SERVO_TRUE_STRAIGHT;  // 63.5
const float TURN_MIN_STEER       = 8.0;
const float TURN_KV              = 3.5;
const int   TURN_MAX_PWM         = 130;
const int   TURN_MIN_PWM         = 100;
const float TURN_STOP_DEG        = 0.3;

long turnStartTicks = 0, turnCapTicks = 0;

bool turnArcStep(float target) {
  if (absEnc(readEncoder() - turnStartTicks) >= turnCapTicks) return true;
  if (gImuFresh) {
    float err = wrapDeg(target - gHeading);
    if (fabs(err) < TURN_STOP_DEG) return true;
    float mag      = fabs(err);
    float maxSteer = (err > 0) ? TURN_MAX_STEER_LEFT : TURN_MAX_STEER_RIGHT;
    float steer    = constrain(TURN_KP * mag, TURN_MIN_STEER, maxSteer);
    int   pwm      = (int)constrain(TURN_KV * mag, (float)TURN_MIN_PWM, (float)TURN_MAX_PWM);
    setServoAngle((err > 0) ? (SERVO_TRUE_STRAIGHT - steer) : (SERVO_TRUE_STRAIGHT + steer));
    setMotorSpeed(pwm);
  }
  return false;
}

// ============================================================================
// STATE HELPERS + working vars
// ============================================================================
void goState(RobotState s) { currentState = s; entered = false; }

BlockColor dcWantColor;
bool       dcColorArmed;
long       dcBaseTicks, dcLockoutTicks, dcSafetyTicks;
uint8_t    sideOpenCount = 0;

float turnTarget, turnAmount;

long  avoidLockoutFrom = -1000000;
float avoidTargetHeading = 0.0;

// Open-round segment measurement, kept as the finish FALLBACK only. Avoidance
// zig-zags inflate encoder distance relative to straight-line distance, so the
// front-wall match below is the primary terminator.
float firstSegmentCm = 0.0, fullStartStraightCm = 0.0;
bool  haveFullStraight = false;
float finalDistanceCm = FINAL_FALLBACK_CM;
long  fsTargetTicks = (long)(FINAL_FALLBACK_CM * TICKS_PER_CM);   // never 0: 0 finishes instantly

// ============================================================================
// RECOVER  (overlay: interrupts a moving state and returns to it)
// ============================================================================
RobotState recoverReturnState = STATE_HEADING;
bool       recoverReturnEntered = false;
long       recoverBaseTicks = 0;
int        recoverTries = 0;

void enterRecovery() {
  recoverReturnState   = currentState;
  recoverReturnEntered = entered;
  colorMuted    = true;
  colorMuteFrom = gOdoTicks;
  currentState  = STATE_RECOVER;
  entered = false;
}

void recoverStep() {
  if (!entered) {
    entered = true;
    setMotorSpeed(0);
    // Counter-steer while backing: keeps rotating the car the way it was
    // already turning instead of retracing the arc it came in on.
    setServoAngle(2.0 * SERVO_TRUE_STRAIGHT - lastServoCmd);
    setMotorSpeed(-RECOVER_PWM);
    recoverBaseTicks = gOdoTicks;
    Serial.print(F("# RECOVER front=")); Serial.println(lidarF);
  }

  bool clear     = !lidarStale && (lidarF >= WALL_CLEAR_MM);
  bool backedFar = absEnc(gOdoTicks - recoverBaseTicks) >= (long)(RECOVER_MAX_CM * TICKS_PER_CM);
  if (!clear && !backedFar) return;   // stale => clear is false => the cap decides

  setMotorSpeed(0);
  if (clear) { recoverTries = 0; Serial.println(F("# recover clear")); }
  else       { recoverTries++;   Serial.print(F("# recover capped, try ")); Serial.println(recoverTries); }

  currentState = recoverReturnState;
  entered      = recoverReturnEntered;
  resetHeadingPid();
  if (currentState == STATE_HEADING)    setMotorSpeed(BASE_SPEED);
  else if (currentState == STATE_AVOID) setMotorSpeed(AVOID_SPEED);
  // TURN90 sets its own PWM each step.
}

// ============================================================================
// STATE: BOOT
//
// Nothing happens until the Pi has sent one well-formed PERCEPT frame saying
// its lidar AND camera are up. That is the whole handshake: the car cannot
// start moving while perception is still coming up, and it cannot silently run
// an obstacle round blind. If the camera never comes up but the lidar does, we
// start anyway after CAMERA_GRACE_MS — an open-round lap scores more than a
// car that never moves — and say so in the log.
// ============================================================================
void bootStep() {
  if (!entered) {
    entered = true;
    phaseT0 = 0;
    setMotorSpeed(0);
    setServoAngle(SERVO_TRUE_STRAIGHT);
    Serial.println(F("# BOOT waiting for Pi"));
  }

  if (phaseT0 == 0) {
    digitalWrite(LED1_PIN, ((millis() / 500) & 1) ? HIGH : LOW);
    bool ready = pHello ||
                 (pLidarOk && (millis() - firstFrameMs > CAMERA_GRACE_MS));
    if (!ready) return;
    if (!pCamOk) Serial.println(F("# WARN camera down - lap will run blind to pillars"));
    phaseT0 = millis();
    Serial.println(F("# Pi ready - 5s countdown"));
  }

  digitalWrite(LED1_PIN, ((millis() / 250) & 1) ? HIGH : LOW);
  if (millis() - phaseT0 < START_DELAY_MS) return;

  digitalWrite(LED1_PIN, HIGH);
  lockedColor   = COLOR_NONE;
  firstLineSeen = false;
  cornerCount   = 0;
  colorMuted   = false;
  recoverTries = 0;
  laneHeading   = gHeading;
  targetHeading = laneHeading;
  frontAtStartMm = lidarF;            // the finish line, measured here
  zeroEncoder();
  Serial.print(F("# GO  frontAtStart=")); Serial.println(frontAtStartMm);
  goState(STATE_HEADING);
}

// ============================================================================
// STATE: HEADING  (maintain heading — every other state converges back here)
//
// Priority, and why: corner beats pillar. Missing a pillar costs points;
// missing a corner ends the run. Avoidance is checked before the post-corner
// lockout because a pillar can sit immediately after a turn, but the lockout
// still gates the CORNER trigger so the corner just turned cannot re-fire.
// ============================================================================
void headingStep() {
  if (!entered) {
    entered = true;
    dcWantColor   = lockedColor;
    dcColorArmed  = (lockedColor == COLOR_NONE) ? firstLineSeen : false;
    sideOpenCount = 0;
    resetColorDetector();
    resetHeadingPid();
    dcBaseTicks = gOdoTicks;
    setMotorSpeed(BASE_SPEED);
    dcLockoutTicks = (cornerCount > 0) ? (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM) : 0;
    dcSafetyTicks  = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);
    Serial.println(F("# HEADING"));
  }

  updateHeadingPid(targetHeading);

  // ---- pillar: the Pi has solved a manoeuvre ----
  // Checked before everything else including the finish, because the last
  // section has pillars in it too and a hit there costs more than a late stop.
  if (pAction != AVOID_NONE && !linkStale) { goState(STATE_AVOID); return; }

  // ---- the finish: same distance from the front wall as where we started ----
  // The wall test is primary because it is geometric. The open round's L - A
  // encoder arithmetic assumes path length equals straight-line length, and
  // avoidance zig-zags break that assumption; it is kept only as the fallback.
  if (cornerCount >= TARGET_CORNERS) {
    long run = absEnc(readEncoder());
    bool byWall = (frontAtStartMm != LIDAR_FAR) && !lidarStale &&
                  (lidarF != LIDAR_FAR) && (run > (long)(10.0 * TICKS_PER_CM)) &&
                  (lidarF <= frontAtStartMm + FINISH_TOL_MM);
    bool byOdo  = run >= fsTargetTicks;
    if (byWall || byOdo) {
      Serial.print(F("# FINISH by ")); Serial.println(byWall ? F("wall") : F("odo"));
      goState(STATE_FINISH);
    }
    return;                            // no corners left, just the run-out
  }

  long straightTicks = absEnc(gOdoTicks - dcBaseTicks);
  if (straightTicks >= dcSafetyTicks) {
    Serial.println(F("# WARN no turn trigger within safety distance, retrying"));
    entered = false;
    return;
  }

  // Corner lockouts: one from the turn just made, one from the avoid just made.
  if (absEnc(readEncoder()) <= dcLockoutTicks) return;
  if (absEnc(gOdoTicks - avoidLockoutFrom) <= (long)(POST_AVOID_LOCKOUT_CM * TICKS_PER_CM)) return;

  bool directionLocked = (lockedColor != COLOR_NONE);

  // ---- colour gate: first corner, or the lidar-dead fallback ----
  if (!dcColorArmed && (!directionLocked || lidarDead)) {
    BlockColor c = detectColor(dcWantColor);
    if (c != COLOR_NONE) {
      dcColorArmed = true;
      if (!directionLocked) { lastFirstColor = c; firstLineSeen = true; }
      sideOpenCount = 0;
      resetColorDetector();
      Serial.print(F("# gate ")); Serial.print(c == COLOR_ORANGE ? F("ORANGE") : F("BLUE"));
      Serial.print(F(" side=")); Serial.println(turnSideMm());
    }
  }
  if (!directionLocked && !dcColorArmed) return;   // direction not known yet

  // ---- turn confirm: counted per lidar REVOLUTION ----
  // Each bearing gets at most one new sample per revolution (~10 Hz on the C1),
  // so counting per frame at 50 Hz would "confirm" on one measurement resent.
  uint16_t sideNow = turnSideMm();
  if (lidarStale) {
    sideOpenCount = 0;
  } else if (lidarRev != lastRev) {
    lastRev = lidarRev;
    if (sideNow > SIDE_OPEN_MM) { if (sideOpenCount < 250) sideOpenCount++; }
    else                        sideOpenCount = 0;
  }
  bool sideOpen = (sideOpenCount >= SIDE_OPEN_REVS);

  if (sideOpen || (lidarDead && dcColorArmed)) {
    Serial.print(sideOpen ? F("# turn: side open ") : F("# turn: colour only "));
    Serial.println(sideNow);

    if (!directionLocked) {
      lockedColor   = lastFirstColor;
      clockwiseMode = (lockedColor == COLOR_ORANGE);
      Serial.println(clockwiseMode ? F("# LOCKED CW (orange)") : F("# LOCKED CCW (blue)"));
    }

    // Fallback finish measurement only — see the note at the declaration.
    float segCm = absEnc(readEncoder()) / TICKS_PER_CM;
    if (cornerCount == 0)                            firstSegmentCm = segCm;
    else if (cornerCount == 4)                     { fullStartStraightCm = segCm; haveFullStraight = true; }
    else if (cornerCount == 8 && haveFullStraight)   fullStartStraightCm = 0.5 * (fullStartStraightCm + segCm);
    if (haveFullStraight) {
      finalDistanceCm = max(0.0f, fullStartStraightCm - firstSegmentCm);
      fsTargetTicks   = (long)(finalDistanceCm * TICKS_PER_CM);
    }

    goState(STATE_TURN90);
  }
}

// ============================================================================
// STATE: AVOID
//
// The Pi has already done the geometry. All this state does is hold the heading
// it was given for the distance it was given, then hand back to HEADING.
//
//   TRACK   the Pi is still re-solving. Take the new heading and RE-BASE the
//           odometry, because the leg it sends is always measured from now.
//           The leg therefore never expires while TRACK persists — by design;
//           the timeout below is the only thing that ends a stuck track.
//   COMMIT  frozen. Ignore further updates and run the leg out on odometry.
//           This is what carries the car past a pillar the camera can no
//           longer see, and it survives a lidar dropout for the same reason.
// ============================================================================
void avoidStep() {
  if (!entered) {
    entered = true;
    phaseT0 = millis();
    avoidTargetHeading = pHeadingDeg;
    avoidBaseTicks = gOdoTicks;
    avoidLegTicks  = (long)(pLegMm * TICKS_PER_MM);
    resetHeadingPid();
    setMotorSpeed(AVOID_SPEED);
    Serial.print(F("# AVOID ")); Serial.print(pGreen ? F("GREEN->left ") : F("RED->right "));
    Serial.print(avoidTargetHeading); Serial.print(F(" deg for ")); Serial.println(pLegMm);
  }

  if (pAction == AVOID_NONE) {                 // the Pi says we are clear
    Serial.println(F("# avoid released"));
    avoidLockoutFrom = gOdoTicks;
    targetHeading = laneHeading;
    goState(STATE_HEADING);
    return;
  }

  // A stale link means the last TRACK heading is not being refreshed any more.
  // Stop re-basing and let the leg expire on its own distance, rather than
  // holding a frozen heading indefinitely with no one watching.
  if (pAction == AVOID_TRACK && !linkStale) {
    avoidTargetHeading = pHeadingDeg;
    avoidLegTicks = (long)(pLegMm * TICKS_PER_MM);
    avoidBaseTicks = gOdoTicks;
  }

  updateHeadingPid(avoidTargetHeading);

  // Keep watching for the first corner line even mid-manoeuvre: if it passes
  // under the car during an avoid and we are not looking, the direction never
  // locks and HEADING stalls until the safety distance fires.
  if (lockedColor == COLOR_NONE && !firstLineSeen) {
    BlockColor c = detectColor(COLOR_NONE);
    if (c != COLOR_NONE) {
      lastFirstColor = c; firstLineSeen = true;
      Serial.println(F("# first line seen during avoid"));
    }
  }

  long travelled = absEnc(gOdoTicks - avoidBaseTicks);
  bool done    = travelled >= avoidLegTicks;
  bool capped  = travelled >= (long)(AVOID_MAX_LEG_CM * TICKS_PER_CM);
  bool timeout = (millis() - phaseT0) > AVOID_TIMEOUT_MS;

  if (done || capped || timeout) {
    if (capped)  Serial.println(F("# avoid capped"));
    if (timeout) Serial.println(F("# avoid timed out"));
    avoidLockoutFrom = gOdoTicks;
    targetHeading = laneHeading;
    goState(STATE_HEADING);
  }
}

// ============================================================================
// STATE: TURN90  (eased arc, no settle, no lane correction)
// ============================================================================
void turn90Step() {
  if (!entered) {
    entered = true;
    cornerCount++;
    Serial.print(F("# TURN ")); Serial.print(cornerCount);
    Serial.print('/'); Serial.println(TARGET_CORNERS);
    turnAmount     = clockwiseMode ? 90.0 : -90.0;
    turnTarget     = wrapDeg(laneHeading - turnAmount);
    turnStartTicks = readEncoder();
    turnCapTicks   = (long)(120.0 * TICKS_PER_CM);
  }

  if (turnArcStep(turnTarget)) {
    laneHeading   = wrapDeg(laneHeading - turnAmount);
    targetHeading = laneHeading;
    zeroEncoder();                 // segment origin = this turn corner
    Serial.print(F("# lane heading ")); Serial.println(laneHeading);
    goState(STATE_HEADING);
  }
}

// ============================================================================
// MAIN
// ============================================================================
void initHardware() {
  pinMode(MOT_RPWM_PIN, OUTPUT);
  pinMode(MOT_LPWM_PIN, OUTPUT);
  setMotorSpeed(0);

  pinMode(LED1_PIN, OUTPUT); pinMode(LED2_PIN, OUTPUT); pinMode(LED3_PIN, OUTPUT);
  pinMode(BTN_START_PIN, INPUT_PULLUP);
  digitalWrite(LED1_PIN, LOW); digitalWrite(LED2_PIN, LOW); digitalWrite(LED3_PIN, LOW);

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
  gLastRawEnc = readEncoder();

  // ---- I2C + colour ----
  resetTCA();
  Wire.setSCL(I2C_SCL); Wire.setSDA(I2C_SDA);
  Wire.begin(); Wire.setClock(400000);
  delay(100);
  tcaselect(TCS_CH); delay(10);
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

void setup() {
  Serial.begin(115200);
  initHardware();
}

void loop() {
  serviceSensors();

  static uint32_t lastTelem = 0;
  uint32_t now = millis();
  if (now - lastTelem >= TELEM_PERIOD_MS) { lastTelem = now; sendTelemetry(); }

  if (!fsmStarted) {
    if (perceptFrames == 0) {
      digitalWrite(LED1_PIN, ((now / 500) & 1) ? HIGH : LOW);
      return;
    }
    fsmStarted = true;
    firstFrameMs = now;
    Serial.println(F("# first PERCEPT frame received"));
  }

  // Release the colour mute once we are forward of where the reverse began.
  if (colorMuted && currentState != STATE_RECOVER && gOdoTicks >= colorMuteFrom) {
    colorMuted = false;
    Serial.println(F("# colour re-enabled"));
  }

  // Wall panic overlay. Disabled when the lidar is stale — there is no front
  // distance to act on, and a frozen value would latch it.
  if (!lidarStale &&
      currentState != STATE_RECOVER && currentState != STATE_BOOT &&
      currentState != STATE_FINISH &&
      recoverTries < RECOVER_MAX_TRIES &&
      lidarF <= WALL_PANIC_MM) {
    enterRecovery();
  }

  if (currentState != STATE_BOOT && currentState != STATE_FINISH) {
    digitalWrite(LED1_PIN, lidarStale ? (((now / 100) & 1) ? HIGH : LOW) : HIGH);
  }

  switch (currentState) {
    case STATE_BOOT:    bootStep();    break;
    case STATE_HEADING: headingStep(); break;
    case STATE_AVOID:   avoidStep();   break;
    case STATE_TURN90:  turn90Step();  break;
    case STATE_RECOVER: recoverStep(); break;

    case STATE_FINISH:
      if (!entered) {
        entered = true;
        Serial.println(F("# FINISHED"));
        setMotorSpeed(0);
        setServoAngle(SERVO_TRUE_STRAIGHT);
        digitalWrite(LED1_PIN, HIGH);
        digitalWrite(LED2_PIN, HIGH);
        digitalWrite(LED3_PIN, HIGH);
      }
      break;
  }
}
