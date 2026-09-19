#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>

// ============================================================
// OPEN ROUND - NON-BLOCKING FIRMWARE  (LiDAR revision)
//
// Distance now comes from a Raspberry Pi over USB serial instead of
// ToF sensors. The Pi sends one line per frame:  left,front,right
// in millimetres. That same Serial is the debug channel, so FSM logs
// go back to the Pi - prints are state-transition only, never per-loop.
//
// PER CORNER
//   1. drive straight on IMU heading, tracking a slow average of both
//      side distances
//   2. colour line arms the gate and freezes the turn-side average as
//      a baseline
//   3. turn fires when the turn-side distance JUMPS above that baseline
//      (the inner wall has given way), or the front closes to 450 mm,
//      or the distance backstop trips
//   4. eased 90 deg arc
//   5. lane correct: drive 40 cm so the side readings settle, then crab
//      toward left == right, then realign and hold heading
//
// WHY A JUMP AND NOT A FIXED THRESHOLD
// The corridor is 1000 mm or 600 mm, chosen per section by coin toss,
// and the car's lane within it varies. A car running wide in a 1000 mm
// section already reads ~1000 mm to the inside, so no fixed number can
// separate "wide corridor" from "corner". The step up can.
//
// DEGRADED MODE
// No LiDAR frame for 200 ms sets lidarStale: side-open, front failsafe
// and wall recovery all switch off, and turns fire on the colour gate
// alone. That is the same path the old firmware used when the ToF
// failed to init, so it is known-good rather than new code.
//
// STARTUP GATE
// loop() does nothing but poll Serial until the first well-formed frame
// arrives from the Pi (see fsmStarted / lidarFrames below). WAIT_START's
// countdown - and therefore every state after it - cannot begin before
// that, so the car never starts moving while the Pi's LiDAR script is
// still coming up or hasn't been launched yet.
//
// BENCH-VERIFIED
//   motor    PA2 forward, PA3 reverse
//   encoder  TIM5 PA0/PA1, negated so forward counts up
//   IMU      BNO08x SPI1 ~100 Hz, clockwise = negative yaw
//   colour   TCS34725 CH4, white pR 47 / orange pR 69 / blue pB 27
// ============================================================

enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };

enum RobotState {
  STATE_WAIT_START,
  STATE_DRIVE_TO_CORNER,
  STATE_TURNING,
  STATE_LANE_CORRECT,
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

const int LED1_PIN = PB12;        // solid = running; slow blink (500 ms) = waiting for first Pi frame; fast blink (100 ms) = lidar stale mid-run
const int LED2_PIN = PB13;        // lit while ORANGE is under the sensor
const int LED3_PIN = PB14;        // lit while BLUE is under the sensor
const int BTN_START_PIN = PB15;   // not wired yet - see START_DELAY_MS

#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70
#define TCS_CH      4             // TCS34725

// ---- calibration ----
const float TICKS_PER_CM        = 14.853;
const float SERVO_TRUE_STRAIGHT = 71.0;
const float SERVO_MAX_LEFT      = 1.0;    // servo BELOW straight steers LEFT
const float SERVO_MAX_RIGHT     = 150.0;  // servo ABOVE straight steers RIGHT
const float IMU_YAW_SIGN        = 1.0;    // clockwise reads negative

const int BASE_SPEED     = 70;
const int CORRECTION_PWM = 55;

const unsigned long START_DELAY_MS = 5000;

SPIClass SPI_IMU(PA7, PA6, PA5);  // MOSI, MISO, SCLK
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

bool  tcsOk = false;
float initialYawOffset = 0.0;

// ============================================================
// LiDAR OVER SERIAL   "left,front,right\n"  in mm
// ============================================================
// Two thresholds on purpose. STALE (short) just stops us acting on old
// distances. DEAD (long) is what switches the car to colour-only turning -
// a single dropped batch of frames must not be read as "lidar gone" and
// fire an instant turn mid-straight.
const unsigned long LIDAR_STALE_MS      = 200;
const unsigned long LIDAR_DEAD_MS       = 1000;
const uint16_t      LIDAR_MAX_VALID_MM  = 3500;   // mat diagonal
const uint16_t      LIDAR_FAR           = 9999;   // internal "nothing there"

// A beam with no return must read FAR, never near. If a dropout were
// treated as 0 mm the wall-panic recovery would latch on permanently.
uint16_t lidarSanitize(long v) {
  if (v <= 0 || v > (long)LIDAR_MAX_VALID_MM) return LIDAR_FAR;
  return (uint16_t)v;
}

uint16_t      lidarL = LIDAR_FAR, lidarF = LIDAR_FAR, lidarR = LIDAR_FAR;
unsigned long lidarLastMs = 0;
bool          lidarStale  = true;
bool          lidarDead   = true;
uint32_t      lidarFrames = 0;

char    lidarBuf[40];
uint8_t lidarLen = 0;

void serviceLidar() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lidarLen > 0) {
        lidarBuf[lidarLen] = '\0';
        char *c1 = strchr(lidarBuf, ',');
        char *c2 = c1 ? strchr(c1 + 1, ',') : NULL;
        if (c1 && c2) {
          *c1 = '\0'; *c2 = '\0';
          lidarL = lidarSanitize(atol(lidarBuf));
          lidarF = lidarSanitize(atol(c1 + 1));
          lidarR = lidarSanitize(atol(c2 + 1));
          lidarLastMs = millis();
          lidarStale  = false;
          lidarDead   = false;
          lidarFrames++;
        }
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
const uint16_t FRONT_FORCE_MM      = 450;   // failsafe: turn regardless
const uint16_t SIDE_OPEN_DELTA_MM  = 400;   // jump above baseline = wall gave way
const uint8_t  SIDE_OPEN_FRAMES    = 3;     // consecutive frames before believing it
const float    SIDE_EMA_ALPHA      = 0.05;

float    sideEmaL = 0, sideEmaR = 0;
bool     sideEmaInit  = false;
uint16_t sideBaseline = 0;
uint8_t  sideOpenCount = 0;

// ---- wall recovery ----
const uint16_t WALL_PANIC_MM     = 200;
const uint16_t WALL_CLEAR_MM     = 350;
const int      RECOVER_PWM       = 90;
const float    RECOVER_MAX_CM    = 30.0;
const int      RECOVER_MAX_TRIES = 3;

// ============================================================
// RUN CONSTANTS
// ============================================================
const int   TARGET_CORNERS      = 12;
const int   FINAL_STRAIGHT_CM   = 100;
const float SEARCH_SAFETY_CM    = 400.0;
const float GATE_TO_TURN_MAX_CM = 150.0;
const float POST_CORNER_LOCKOUT_CM = 50.0;

float firstSegmentCm      = 0.0;
float fullStartStraightCm = 0.0;
bool  haveFullStraight    = false;
float finalDistanceCm     = FINAL_STRAIGHT_CM;

// ============================================================
// LANE CORRECTION  (LiDAR centring - one manoeuvre per corner)
// ============================================================
const float LC_SETTLE_CM      = 40.0;   // drive this far before trusting the sides
const float LC_BAND_MM        = 60.0;   // close enough to centre
const float LC_K_DEG_PER_MM   = 0.15;
const float LC_MAX_OFFSET_DEG = 30.0;
const float LC_MAX_CRAB_CM    = 60.0;
const float REALIGN_SAFETY_CM = 80.0;

// ============================================================
// FSM DATA
// ============================================================
// The FSM (and its WAIT_START countdown) is held off entirely until the
// Pi has sent at least one well-formed frame - see the gate at the top
// of loop(). Everything below still initialises to its normal idle
// values; it just doesn't get ticked until fsmStarted flips true.
bool fsmStarted = false;

RobotState    currentState = STATE_WAIT_START;
bool          entered = false;
unsigned long phaseT0 = 0;

BlockColor lockedColor   = COLOR_NONE;
bool       clockwiseMode = true;
int        cornerCount   = 0;

float targetHeading = 0.0;
float laneHeading   = 0.0;

BlockColor lastFirstColor   = COLOR_NONE;
long       cornerFirstTicks = 0;

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
  steeringServo.writeMicroseconds((int)((angleDeg / 180.0) * 1000.0) + 1000);
}

// TIM5 encoder, negated so driving forward counts up
void zeroEncoder() { TIM5->CNT = 0; }
long readEncoder() { return -(int32_t)TIM5->CNT; }
long absEnc(long v) { return v < 0 ? -v : v; }

// Which way are we turning, and therefore which side should open?
// Clockwise means the inner block is on the right, so the RIGHT side
// gives way at the corner. Counter-clockwise, the left.
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

// ---- colour ----
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

// Colour mute. Backing away from a wall drags the sensor back over lines
// already used. Muting from the start of the reverse until the car is
// forward of that point again means those re-crossings are ignored.
// gRawColor is NOT muted, so the LEDs still show what is underneath.
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

// ---- called once at the top of every loop ----
void serviceSensors() {
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
  pinMode(BTN_START_PIN, INPUT_PULLUP);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);

  steeringServo.attach(SERVO_PIN, 1000, 2000);
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
// HEADING PID
// ============================================================
const float HEAD_KP        = 2.0;
const float YAW_FILT_ALPHA = 0.35;
const float SERVO_SLEW     = 2.5;
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

// ============================================================
// EASED TURN LAW - verbatim from the tuned turning90 sketch
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
long       dcBaseTicks;
long       dcLockoutTicks;
long       dcSafetyTicks;

float turnTarget;
float turnAmount;

uint8_t lcPhase;              // 1 = settle, 2 = crab, 4 = realign
long    lcBaseTicks;
long    lcTargetTicks;
bool    lcGateSeen  = false;  // gate line crossed during lane correction
long    lcGateTicks = 0;
long    fsTargetTicks;

void finishLaneCorrect() {
  targetHeading = laneHeading;
  if (cornerCount >= TARGET_CORNERS) goState(STATE_FINAL_STRAIGHT);
  else                               goState(STATE_DRIVE_TO_CORNER);
}

// ============================================================
// STATE: RECOVER  (wall too close - back off, then resume)
//
// Interrupts whatever was running and returns to it, so a turn that ran
// out of room finishes its arc. The saved `entered` flag comes back too,
// so the resumed state does not re-run setup and TURNING cannot
// double-count a corner. Reversing uses the MIRROR of the last steering
// command: counter-steer while backing keeps rotating the car the way it
// was already turning instead of retracing the arc.
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

  // No LiDAR means no way to know when we are clear - back off the full
  // cap and hand control back rather than reversing blind.
  bool clear     = !lidarStale && (lidarF >= WALL_CLEAR_MM);
  bool backedFar = absEnc(readEncoder() - recoverBaseTicks)
                   >= (long)(RECOVER_MAX_CM * TICKS_PER_CM);
  if (!clear && !backedFar) return;   // stale => clear is false => cap decides

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
      setMotorSpeed(BASE_SPEED);
      break;
    case STATE_LANE_CORRECT:
      // A crab that ended in a wall was too aggressive - drop the rest
      // of it and go straight to the realign.
      lcPhase        = 4;
      turnStartTicks = readEncoder();
      turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
      break;
    default:
      break;     // TURNING and the realign arc set their own PWM each step
  }
}

// ============================================================
// STATE: WAIT START  (button not wired - fixed 5 s countdown)
// ============================================================
void waitStartStep() {
  if (!entered) {
    entered = true;
    phaseT0 = millis();
    setMotorSpeed(0);
    setServoAngle(SERVO_TRUE_STRAIGHT);
    Serial.println(F("# WAIT_START 5s"));
  }
  digitalWrite(LED1_PIN, ((millis() / 250) & 1) ? HIGH : LOW);

  if (millis() - phaseT0 >= START_DELAY_MS) {
    digitalWrite(LED1_PIN, HIGH);
    lockedColor  = COLOR_NONE;
    cornerCount  = 0;
    lcGateSeen   = false;
    colorMuted   = false;
    recoverTries = 0;
    laneHeading   = gHeading;
    targetHeading = laneHeading;
    zeroEncoder();
    Serial.print(F("# GO  lidar="));
    Serial.println(lidarDead ? F("DEAD - colour-only mode") : F("live"));
    goState(STATE_DRIVE_TO_CORNER);
  }
}

// ============================================================
// STATE: DRIVE TO CORNER
// ============================================================
void driveStep() {
  if (!entered) {
    entered = true;
    dcWantFirst = lockedColor;
    dcFirstTurn = (lockedColor == COLOR_NONE);
    resetColorDetector();
    resetHeadingPid();
    dcBaseTicks = readEncoder();
    setMotorSpeed(BASE_SPEED);

    sideEmaInit   = false;
    sideOpenCount = 0;

    // A gate line crossed during lane correction carries over, so a corner
    // reached while still correcting is not thrown away.
    if (lcGateSeen) {
      dcColorArmed     = true;
      cornerFirstTicks = lcGateTicks;
      lcGateSeen       = false;
      sideBaseline     = turnSideMm();
      Serial.println(F("# DRIVE (pre-armed from lane correct)"));
    } else {
      dcColorArmed = false;
      Serial.println(F("# DRIVE"));
    }

    dcLockoutTicks = (cornerCount > 0) ? (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM) : 0;
    dcSafetyTicks  = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);
  }

  long straightTicks = absEnc(readEncoder() - dcBaseTicks);
  if (straightTicks >= dcSafetyTicks) {
    Serial.println(F("# WARN no line within safety distance, retrying"));
    entered = false;
    return;
  }

  updateHeadingPid(targetHeading);

  // Track both sides while running the straight. Averaging both means the
  // baseline is ready whichever way turn 1 ends up going.
  if (!lidarStale) {
    if (!sideEmaInit) {
      if (lidarL < LIDAR_FAR && lidarR < LIDAR_FAR) {
        sideEmaL = lidarL; sideEmaR = lidarR; sideEmaInit = true;
      }
    } else {
      if (lidarL < LIDAR_FAR) sideEmaL += SIDE_EMA_ALPHA * ((float)lidarL - sideEmaL);
      if (lidarR < LIDAR_FAR) sideEmaR += SIDE_EMA_ALPHA * ((float)lidarR - sideEmaR);
    }
  }

  // Lockout runs from the TURN CORNER (encoder zero), so distance covered
  // during lane correction counts toward it instead of stacking on top.
  if (!dcColorArmed && absEnc(readEncoder()) <= dcLockoutTicks) return;

  // ---- 1. arm on the colour line, freezing the side baseline ----
  if (!dcColorArmed) {
    BlockColor c = detectColor(dcWantFirst);
    if (c != COLOR_NONE) {
      dcColorArmed = true;
      cornerFirstTicks = readEncoder();
      if (dcFirstTurn) lastFirstColor = c;
      sideBaseline = sideEmaInit
                   ? (uint16_t)(turnIsClockwise() ? sideEmaR : sideEmaL)
                   : turnSideMm();
      sideOpenCount = 0;
      resetColorDetector();
      Serial.print(F("# gate ")); Serial.print(c == COLOR_ORANGE ? F("ORANGE") : F("BLUE"));
      Serial.print(F(" sideBase=")); Serial.println(sideBaseline);
    }
    return;
  }

  // ---- 2. turn confirm ----
  uint16_t sideNow = turnSideMm();
  if (!lidarStale && sideNow >= sideBaseline + SIDE_OPEN_DELTA_MM) {
    if (sideOpenCount < 250) sideOpenCount++;
  } else {
    sideOpenCount = 0;
  }

  bool sideOpen   = (sideOpenCount >= SIDE_OPEN_FRAMES);
  bool frontForce = !lidarStale && (lidarF <= FRONT_FORCE_MM);
  bool backstop   = ((float)absEnc(readEncoder() - cornerFirstTicks) / TICKS_PER_CM)
                    >= GATE_TO_TURN_MAX_CM;

  // lidarDead, not lidarStale: colour-only turning fires the instant the
  // gate arms, so a brief frame dropout must not be allowed to trigger it.
  if (sideOpen || frontForce || lidarDead || backstop) {
    if      (sideOpen)   { Serial.print(F("# turn: side open ")); Serial.println(sideNow); }
    else if (frontForce) { Serial.print(F("# turn: front ")); Serial.println(lidarF); }
    else if (lidarDead)  Serial.println(F("# turn: colour only (lidar dead)"));
    else                 Serial.println(F("# turn: distance backstop"));

    if (lockedColor == COLOR_NONE) {
      lockedColor   = lastFirstColor;
      clockwiseMode = (lockedColor == COLOR_ORANGE);
      Serial.println(clockwiseMode ? F("# LOCKED CW (orange)") : F("# LOCKED CCW (blue)"));
    }

    float segCm = absEnc(cornerFirstTicks) / TICKS_PER_CM;
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
// STATE: TURNING  (eased 90 deg arc, no settle)
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
    turnCapTicks   = (long)(120.0 * TICKS_PER_CM);
  }

  if (turnArcStep(turnTarget)) {
    laneHeading = wrapDeg(laneHeading - turnAmount);
    zeroEncoder();                 // segment origin = this turn corner
    Serial.print(F("# lane heading ")); Serial.println(laneHeading);
    goState(STATE_LANE_CORRECT);
  }
}

// ============================================================
// STATE: LANE CORRECT
//   phase 1  drive LC_SETTLE_CM so both side readings are real corridor
//            measurements and not corner artefacts
//   phase 2  crab toward left == right, closed loop on the error
//   phase 4  realign onto laneHeading and hand back to the PID
// One manoeuvre per corner - the straight is then held on IMU alone.
// ============================================================
void laneCorrectStep() {
  if (!entered) {
    entered = true;
    if (lidarDead) {
      Serial.println(F("# lane correct skipped (lidar dead)"));
      finishLaneCorrect();
      return;
    }
    resetHeadingPid();
    resetColorDetector();
    setMotorSpeed(BASE_SPEED);
    lcBaseTicks   = readEncoder();
    lcTargetTicks = (long)(LC_SETTLE_CM * TICKS_PER_CM);
    lcPhase = 1;
    Serial.println(F("# LANE_CORRECT settle"));
    return;
  }

  // Keep watching for the NEXT corner's gate line. Settle plus crab can
  // cover a metre, and ignoring a line crossed here would send the car
  // into DRIVE un-armed and straight past the corner. Same lockout from
  // the turn corner, so the lines just turned over cannot re-trigger.
  if (!lcGateSeen && lockedColor != COLOR_NONE &&
      absEnc(readEncoder()) >= (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM)) {
    if (detectColor(lockedColor) != COLOR_NONE) {
      lcGateSeen  = true;
      lcGateTicks = readEncoder();
      Serial.println(F("# gate seen during lane correct"));
    }
  }

  if (lcPhase == 1) {
    updateHeadingPid(laneHeading);
    if (absEnc(readEncoder() - lcBaseTicks) < lcTargetTicks) return;

    int err = (int)lidarR - (int)lidarL;      // >0 means room on the right
    Serial.print(F("# sides L=")); Serial.print(lidarL);
    Serial.print(F(" R=")); Serial.print(lidarR);
    Serial.print(F(" err=")); Serial.println(err);

    // Centring is meaningless if a side is looking down an open corridor.
    if (lidarL >= LIDAR_FAR || lidarR >= LIDAR_FAR || abs(err) < (int)LC_BAND_MM) {
      Serial.println(F("# no crab needed"));
      finishLaneCorrect();
      return;
    }
    lcBaseTicks   = readEncoder();
    lcTargetTicks = (long)(LC_MAX_CRAB_CM * TICKS_PER_CM);
    setMotorSpeed(CORRECTION_PWM);
    lcPhase = 2;
    return;
  }

  if (lcPhase == 2) {
    int   err    = (int)lidarR - (int)lidarL;
    float offset = LC_K_DEG_PER_MM * fabs((float)err);
    if (offset > LC_MAX_OFFSET_DEG) offset = LC_MAX_OFFSET_DEG;
    setServoAngle(err > 0 ? (SERVO_TRUE_STRAIGHT + offset)    // room right, steer right
                          : (SERVO_TRUE_STRAIGHT - offset));

    bool reached = abs(err) < (int)LC_BAND_MM;
    bool capped  = absEnc(readEncoder() - lcBaseTicks) >= lcTargetTicks;
    if (reached || capped || lidarStale) {
      Serial.print(F("# crab done err=")); Serial.println(err);
      turnStartTicks = readEncoder();
      turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
      lcPhase = 4;
    }
    return;
  }

  if (turnArcStep(laneHeading)) finishLaneCorrect();
}

// ============================================================
// STATE: FINAL STRAIGHT
// Encoder still counts from turn 12, so it already includes the lane
// correction travel - we drive until the total reaches L - A.
// ============================================================
void finalStraightStep() {
  if (!entered) {
    entered = true;
    Serial.print(F("# FINAL_STRAIGHT ")); Serial.println(finalDistanceCm);
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
// MAIN
// ============================================================
void setup() {
  Serial.begin(115200);
  initHardware();
}

void loop() {
  serviceSensors();

  // ---- startup gate ----
  // Nothing below this runs until the Pi has sent at least one
  // well-formed frame (lidarFrames > 0). Before that, we just keep
  // draining Serial and slow-blink LED1 to show we're waiting on the
  // Pi. This stops WAIT_START's countdown - and every state after it -
  // from starting while the Pi's LiDAR script isn't up yet.
  if (!fsmStarted) {
    if (lidarFrames == 0) {
      digitalWrite(LED1_PIN, ((millis() / 500) & 1) ? HIGH : LOW);
      return;
    }
    fsmStarted = true;
    Serial.println(F("# first LiDAR frame received - FSM starting"));
  }

  // Release the colour mute once we are forward of where the reverse began.
  if (colorMuted && currentState != STATE_RECOVER && readEncoder() >= colorMuteFrom) {
    colorMuted = false;
    Serial.println(F("# colour re-enabled"));
  }

  // Wall panic overlay: interrupts any moving state and resumes it.
  // Disabled when the LiDAR is stale - there is no front distance to act on.
  if (!lidarStale &&
      currentState != STATE_RECOVER &&
      currentState != STATE_WAIT_START &&
      currentState != STATE_FINISHED &&
      recoverTries < RECOVER_MAX_TRIES &&
      lidarF <= WALL_PANIC_MM) {
    enterRecovery();
  }

  // LED1: solid while running, blinking fast if the LiDAR feed is dead.
  if (currentState != STATE_WAIT_START && currentState != STATE_FINISHED) {
    digitalWrite(LED1_PIN, lidarStale ? (((millis() / 100) & 1) ? HIGH : LOW) : HIGH);
  }

  switch (currentState) {
    case STATE_WAIT_START:      waitStartStep();     break;
    case STATE_DRIVE_TO_CORNER: driveStep();         break;
    case STATE_TURNING:         turningStep();       break;
    case STATE_LANE_CORRECT:    laneCorrectStep();   break;
    case STATE_FINAL_STRAIGHT:  finalStraightStep(); break;
    case STATE_RECOVER:         recoverStep();       break;

    case STATE_FINISHED:
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