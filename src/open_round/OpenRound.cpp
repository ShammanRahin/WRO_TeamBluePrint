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
// PER CORNER  (LiDAR only - the colour sensor plays no part while the
// LiDAR is alive; it only lights LED2 / LED3)
//   1. drive straight on IMU heading
//   2. turn fires when a side beam reads above SIDE_OPEN_MM (1500) for
//      SIDE_OPEN_FRAMES consecutive NEW LiDAR frames (the inner wall has
//      given way). Until the direction is known BOTH sides are watched and
//      the side that opens first locks it: right = clockwise, left =
//      anticlockwise (both at once: the longer beam). After that only the
//      inner side is watched.
//   3. FALLBACK: the front beam below FRONT_TURN_MM (200) turns anyway
//      (direction not known yet: toward the longer side beam). Past the
//      post-corner lockout this replaces the wall-panic RECOVER on a straight.
//   4. eased 90 deg arc at full steering lock, then straight back to
//      DRIVE on the new heading (no lane correction - IMU only)
//
// WHY 1500 mm IS SAFE AS A FIXED NUMBER
// The widest corridor is 1000 mm, so the inside wall can never read
// more than ~1000 mm mid-straight. Past the end of the inner wall the
// side beam looks down the next straight, well beyond 1500 mm.
//
// STOPPING WHERE IT STARTED
// At START the front beam's distance to the wall ahead is remembered
// (median of the last 5 frames while standing). After turn 12 the car
// drives the start straight until the front beam is back near that value
// (encoder L - A only bounds it), then ALIGNS: creeps forward / backward
// at ALIGN_PWM until the front is within FRONT_DEADBAND_MM of the start
// value on a settled (stopped) reading. No usable front at START (beyond
// the LiDAR's 3.5 m, or LiDAR dead) -> the old encoder L - A stop.
//
// DEGRADED MODE
// No LiDAR frame for 1000 ms sets lidarDead. Only then does the colour
// line trigger turns (and set the direction if it is still unknown:
// orange = clockwise) - with no LiDAR there is no side or front beam, and
// the car would drive into the outer wall.
//
// STARTUP SEQUENCE
//   1. Boot: onboard LED (PC13) slow-blinks. loop() only polls Serial -
//      the start button is ignored until the Pi's first well-formed
//      LiDAR frame arrives.
//   2. First frame received: onboard LED goes SOLID. The FSM enters
//      WAIT_START and waits for the start button on PB12.
//   3. Button pressed (debounced): car starts driving immediately.
// The button must be seen RELEASED before a press counts, so a button
// held (or shorted) at power-up can never launch the car.
//
// BUTTON (PB12) DURING AND AFTER A RUN
//   running  -> press: PAUSE (motor off, everything frozen where it is)
//   paused   -> press: RESUME exactly where it stopped (mid-straight,
//               mid-turn or mid-recovery)
//   finished -> press: a fresh 3-lap run (put the car back in the start
//               section first)
// Presses closer than BTN_LOCKOUT_MS are ignored (bounce, double taps).
//   status LED: slow blink 250 ms while paused
//
// STATUS LED (onboard PC13, active LOW)
//   slow blink 500 ms  waiting for the Pi's first LiDAR frame
//   solid              Pi connected, LiDAR live (ready / running / finished)
//   fast blink 100 ms  LiDAR frames stale (Pi dropped out)
//
// BENCH-VERIFIED
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
  STATE_FINISHED,
  STATE_PAUSED               // button pause: resumes the state it interrupted
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

const int STATUS_LED_PIN = PC13;  // Black Pill onboard LED, active LOW - see STATUS LED above
const int LED2_PIN = PB13;        // lit while ORANGE is under the sensor
const int LED3_PIN = PB14;        // lit while BLUE is under the sensor
const int BTN_START_PIN = PB12;   // start button to GND, internal pull-up (pressed = LOW)

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

const int BASE_SPEED     = 70;
const int CORRECTION_PWM = 55;

const unsigned long BTN_DEBOUNCE_MS = 30;
const unsigned long BTN_LOCKOUT_MS  = 400;   // ignore a second press this soon after one

SPIClass SPI_IMU(PA7, PA6, PA5);  // MOSI, MISO, SCLK
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

bool  tcsOk = false;
float initialYawOffset = 0.0;

// ---- onboard status LED (PC13 sinks current: LOW = on) ----
void statusLed(bool on) { digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH); }
void statusBlink(unsigned long periodMs) { statusLed((millis() / periodMs) & 1); }

// ---- start button, non-blocking debounce ----
// btnStable starts as LOW ("pressed") on purpose: the first debounced
// edge we can see is therefore a RELEASE, and only the press after that
// returns true. A button held down at boot cannot start the car.
bool          btnLastRaw  = LOW;
bool          btnStable   = LOW;
unsigned long btnChangeMs = 0;

unsigned long btnLastPressMs = 0;
bool          btnPressedOnce = false;
bool          btnPress       = false;  // set by loop() once per pass: a new press happened

bool startButtonPressed() {          // true once, on the debounced press edge
  bool raw = digitalRead(BTN_START_PIN);
  if (raw != btnLastRaw) { btnLastRaw = raw; btnChangeMs = millis(); }
  if (raw != btnStable && millis() - btnChangeMs >= BTN_DEBOUNCE_MS) {
    btnStable = raw;
    if (btnStable == LOW) {
      if (btnPressedOnce && millis() - btnLastPressMs < BTN_LOCKOUT_MS) return false;
      btnPressedOnce = true;
      btnLastPressMs = millis();
      return true;
    }
  }
  return false;
}

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
bool          lidarNewFrame = false;   // true only on the loop a frame was parsed

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
          lidarNewFrame = true;
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
const uint16_t SIDE_OPEN_MM     = 1500;  // turn side above this = inner wall gone
const uint8_t  SIDE_OPEN_FRAMES = 3;     // consecutive NEW frames before believing it
const uint16_t FRONT_TURN_MM    = 200;   // fallback: front closer than this = turn anyway

uint8_t sideOpenCount = 0;               // left + right counts are kept separately:
uint8_t sideOpenL = 0, sideOpenR = 0;    //   the first to open locks the direction

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
const float POST_CORNER_LOCKOUT_CM = 50.0;

// ---- stop at the start position (front beam) ----
const uint16_t FRONT_DEADBAND_MM  = 20;     // done when |front - start front| <= this
const uint16_t FRONT_SLOW_MM      = 150;    // hand over to ALIGN this far before the target
const int      ALIGN_PWM          = 50;     // creep speed both ways (raise if the car won't move)
const unsigned long ALIGN_SETTLE_MS  = 300; // stopped this long before a reading counts (~3 revs)
const unsigned long ALIGN_TIMEOUT_MS = 6000;// give up aligning, stop where it is
const float    FRONT_WINDOW_CM    = 60.0;   // front stop only once encoder >= L - A - this
const float    FRONT_BACKSTOP_CM  = 60.0;   //   and never drive past L - A + this blind

uint16_t frontHist[5] = { LIDAR_FAR, LIDAR_FAR, LIDAR_FAR, LIDAR_FAR, LIDAR_FAR };
uint8_t  frontHistIdx = 0;
uint16_t startFrontMm = LIDAR_FAR;          // LIDAR_FAR = unknown -> encoder stop

uint16_t frontMedian() {
  uint16_t v[5];
  for (int i = 0; i < 5; i++) v[i] = frontHist[i];
  for (int i = 1; i < 5; i++)                       // insertion sort, 5 values
    for (int j = i; j > 0 && v[j] < v[j - 1]; j--) { uint16_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t; }
  return v[2];
}

float firstSegmentCm      = 0.0;
float fullStartStraightCm = 0.0;
bool  haveFullStraight    = false;
float finalDistanceCm     = FINAL_STRAIGHT_CM;

// ============================================================
// FSM DATA
// ============================================================
// The FSM (and its WAIT_START button wait) is held off entirely until the
// Pi has sent at least one well-formed frame - see the gate at the top
// of loop(). Everything below still initialises to its normal idle
// values; it just doesn't get ticked until fsmStarted flips true.
bool fsmStarted = false;

RobotState    currentState = STATE_WAIT_START;
bool          entered = false;
unsigned long phaseT0 = 0;

bool       dirLocked     = false;      // direction known (first corner turned)
bool       clockwiseMode = true;
int        cornerCount   = 0;

float targetHeading = 0.0;
float laneHeading   = 0.0;


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

// Which way are we turning, and therefore which side should open?
// Clockwise means the inner block is on the right, so the RIGHT side
// gives way at the corner. Counter-clockwise, the left.
bool turnIsClockwise() { return clockwiseMode; }   // valid once dirLocked
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
  lidarNewFrame = false;
  serviceLidar();
  if (lidarNewFrame) { frontHist[frontHistIdx] = lidarF; frontHistIdx = (frontHistIdx + 1) % 5; }

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

  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(BTN_START_PIN, INPUT_PULLUP);
  statusLed(false);
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
// EASED TURN LAW - tuned turning90 sketch, steer cap = full lock
// ============================================================
// Steering saturates at each side's hard stop, so the arc runs at full
// lock (27 cm left / 25 cm right) and only eases off in the last
// TURN_MAX_STEER_x / TURN_KP degrees of heading error.
const float TURN_KP              = 2.5;
const float TURN_MAX_STEER_LEFT  = SERVO_TRUE_STRAIGHT - SERVO_MAX_LEFT;   // 56.5
const float TURN_MAX_STEER_RIGHT = SERVO_MAX_RIGHT - SERVO_TRUE_STRAIGHT;  // 63.5
const float TURN_MIN_STEER       = 8.0;
const float TURN_KV              = 3.5;
const int   TURN_MAX_PWM         = 130;
const int   TURN_MIN_PWM         = 100;
const float TURN_STOP_DEG        = 0.3;

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
    int   pwm      = (int)constrain(TURN_KV * mag, (float)TURN_MIN_PWM, (float)TURN_MAX_PWM);
    float servo    = (err > 0) ? (SERVO_TRUE_STRAIGHT - steer) : (SERVO_TRUE_STRAIGHT + steer);
    setServoAngle(servo);
    setMotorSpeed(pwm);
  }
  return false;
}

// ============================================================
// STATE HELPERS + working vars
// ============================================================
void goState(RobotState s) { currentState = s; entered = false; }

BlockColor dcWantColor;
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
    default:
      break;     // TURNING sets its own PWM each step
  }
}

// ============================================================
// STATE: WAIT START  (Pi connected - wait for the PB12 button)
// ============================================================
// A fresh 3-lap run from where the car stands (WAIT_START, or FINISHED).
void startRun() {
  dirLocked    = false;
  cornerCount  = 0;
  colorMuted   = false;
  recoverTries = 0;
  firstSegmentCm      = 0.0;         // lap bookkeeping from a previous run
  fullStartStraightCm = 0.0;         //   must not leak into this one
  haveFullStraight    = false;
  finalDistanceCm     = FINAL_STRAIGHT_CM;
  laneHeading   = gHeading;
  targetHeading = laneHeading;
  startFrontMm  = lidarStale ? LIDAR_FAR : frontMedian();   // LIDAR_FAR if > 3.5 m / no return
  Serial.print(F("# start front "));
  if (startFrontMm < LIDAR_FAR) Serial.println(startFrontMm);
  else                          Serial.println(F("unknown - will stop on the encoder (L - A)"));
  zeroEncoder();
  Serial.print(F("# GO  lidar="));
  Serial.println(lidarDead ? F("DEAD - colour-only mode") : F("live"));
  goState(STATE_DRIVE_TO_CORNER);
}

// ============================================================
// BUTTON PAUSE / RESUME
// Like RECOVER, PAUSED remembers the state it interrupted and that
// state's `entered` flag, so RESUME continues the same straight, arc or
// recovery instead of restarting it. The servo is left where it was.
// ============================================================
RobotState pauseReturnState   = STATE_DRIVE_TO_CORNER;
bool       pauseReturnEntered = false;

void pauseRun() {
  pauseReturnState   = currentState;
  pauseReturnEntered = entered;
  setMotorSpeed(0);
  currentState = STATE_PAUSED;
  entered      = true;
  Serial.println(F("# PAUSED (button) - press again to resume"));
}

void resumeRun() {
  currentState = pauseReturnState;
  entered      = pauseReturnEntered;
  resetHeadingPid();
  sideOpenCount = 0;                 // re-confirm a corner on fresh frames
  sideOpenL = sideOpenR = 0;
  switch (currentState) {
    case STATE_DRIVE_TO_CORNER:
    case STATE_FINAL_STRAIGHT: setMotorSpeed(BASE_SPEED);   break;
    case STATE_RECOVER:        setMotorSpeed(-RECOVER_PWM); break;   // keeps its mirrored steering
    default:                   break;                                // TURNING sets its own PWM
  }
  Serial.println(F("# RESUMED (button)"));
}

void waitStartStep() {
  if (!entered) {
    entered = true;
    phaseT0 = millis();
    setMotorSpeed(0);
    setServoAngle(SERVO_TRUE_STRAIGHT);
    Serial.println(F("# WAIT_START - press button on PB12"));
  }

  if (btnPress) {
    btnPress = false;
    startRun();
  }
}

// ============================================================
// STATE: DRIVE TO CORNER
//
// LiDAR only. A side beam > SIDE_OPEN_MM on SIDE_OPEN_FRAMES new frames
// triggers the turn - before the direction is known both sides are
// watched and the first to open locks it, after that only the inner side.
// Fallback: front < FRONT_TURN_MM turns anyway. The colour line is used
// only when the LiDAR is dead (degraded mode).
// ============================================================
// Past the post-corner lockout, a close front on a straight is a corner
// we missed: turn (below) instead of the wall-panic RECOVER in loop().
bool driveTurnArmed() {
  return currentState == STATE_DRIVE_TO_CORNER && entered && !lidarStale &&
         absEnc(readEncoder()) > dcLockoutTicks;
}

void driveStep() {
  if (!entered) {
    entered = true;
    dcWantColor   = dirLocked ? (clockwiseMode ? COLOR_ORANGE : COLOR_BLUE) : COLOR_NONE;
    sideOpenCount = 0;
    sideOpenL = sideOpenR = 0;
    resetColorDetector();
    resetHeadingPid();
    dcBaseTicks = readEncoder();
    setMotorSpeed(BASE_SPEED);
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

  updateHeadingPid(targetHeading);

  // Lockout runs from the turn corner (encoder zeroed at the end of the
  // arc), so the corner just turned cannot trigger another turn.
  if (absEnc(readEncoder()) <= dcLockoutTicks) return;

  bool trigger = false;

  if (lidarDead) {
    // ---- degraded mode: the colour line alone ----
    BlockColor c = detectColor(dcWantColor);
    if (c != COLOR_NONE) {
      if (!dirLocked) {
        dirLocked     = true;
        clockwiseMode = (c == COLOR_ORANGE);
        Serial.println(clockwiseMode ? F("# LOCKED CW (orange, lidar dead)")
                                     : F("# LOCKED CCW (blue, lidar dead)"));
      }
      Serial.println(F("# turn: colour only (lidar dead)"));
      trigger = true;
    }
  } else if (lidarStale) {
    sideOpenL = sideOpenR = 0;         // old distances: act on nothing
  } else if (lidarNewFrame) {
    // ---- side open: counted per NEW frame, not per loop ----
    bool watchL = !dirLocked || !clockwiseMode;
    bool watchR = !dirLocked ||  clockwiseMode;
    if (watchL && lidarL > SIDE_OPEN_MM) { if (sideOpenL < 250) sideOpenL++; } else sideOpenL = 0;
    if (watchR && lidarR > SIDE_OPEN_MM) { if (sideOpenR < 250) sideOpenR++; } else sideOpenR = 0;
    bool openL = sideOpenL >= SIDE_OPEN_FRAMES;
    bool openR = sideOpenR >= SIDE_OPEN_FRAMES;
    bool frontClose = lidarF < FRONT_TURN_MM;

    if (openL || openR || frontClose) {
      if (!dirLocked) {
        dirLocked = true;
        if (openL && openR)    clockwiseMode = (lidarR >= lidarL);   // both: the longer beam
        else if (openL || openR) clockwiseMode = openR;              // the side that opened
        else                   clockwiseMode = (lidarR >= lidarL);   // front fallback: roomier side
        Serial.print(clockwiseMode ? F("# LOCKED CW (right side) L=") : F("# LOCKED CCW (left side) L="));
        Serial.print(lidarL); Serial.print(F(" R=")); Serial.println(lidarR);
      }
      if (openL || openR) { Serial.print(F("# turn: side open ")); Serial.println(turnSideMm()); }
      else                { Serial.print(F("# turn: FRONT fallback, front=")); Serial.println(lidarF); }
      trigger = true;
    }
  }

  if (trigger) {
    // Segments are measured to the turn trigger point (same reference
    // for A and L, so final = L - A still lands on the start position).
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
// STATE: TURNING  (eased 90 deg arc, no settle, no lane correction)
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
    finishCorner();
  }
}

// ============================================================
// STATE: FINAL STRAIGHT
// Encoder counts from the end of turn 12 - drive until it reaches L - A.
// ============================================================
// Reverse heading hold (P only): reversing, the wheels act the other way,
// so the correction flips sign.
void reverseHeadingHold(float heading) {
  if (!gImuFresh) return;
  float error = wrapDeg(heading - gHeading);                 // + = nose must go left
  setServoAngle(constrain(SERVO_TRUE_STRAIGHT + HEAD_KP * error, SERVO_MAX_LEFT, SERVO_MAX_RIGHT));
}

enum FinalPhase { FS_DRIVE, FS_ALIGN };
FinalPhase    fsPhase;
long          fsMinTicks, fsMaxTicks;
unsigned long fsAlignT0, fsStillSince;
int           fsDir;                  // ALIGN: +1 forward, -1 back, 0 stopped (settling)

void finishFinal(const __FlashStringHelper *why) {
  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  Serial.print(F("# STOP ")); Serial.print(why);
  Serial.print(F(" front=")); Serial.print(lidarF);
  Serial.print(F(" start front=")); Serial.print(startFrontMm);
  Serial.print(F(" driven cm=")); Serial.println(absEnc(readEncoder()) / TICKS_PER_CM);
  goState(STATE_FINISHED);
}

void finalStraightStep() {
  bool useFront = startFrontMm < LIDAR_FAR;
  if (!entered) {
    entered = true;
    Serial.print(F("# FINAL_STRAIGHT ")); Serial.print(finalDistanceCm);
    Serial.println(useFront ? F(" cm (encoder bound), stop on the front beam")
                            : F(" cm on the encoder (no start front)"));
    resetHeadingPid();
    setMotorSpeed(BASE_SPEED);
    fsTargetTicks = (long)(finalDistanceCm * TICKS_PER_CM);
    // front stop allowed once near L - A; if L was never measured, anywhere
    fsMinTicks = haveFullStraight ? (long)(fmax(0.0, finalDistanceCm - FRONT_WINDOW_CM) * TICKS_PER_CM) : 0;
    fsMaxTicks = haveFullStraight ? (long)((finalDistanceCm + FRONT_BACKSTOP_CM) * TICKS_PER_CM)
                                  : (long)(300.0 * TICKS_PER_CM);
    fsPhase = FS_DRIVE;
  }
  long driven = absEnc(readEncoder());

  if (!useFront) {                                  // the old encoder stop
    updateHeadingPid(laneHeading);
    if (driven >= fsTargetTicks) finishFinal(F("(encoder L - A)"));
    return;
  }

  if (fsPhase == FS_DRIVE) {
    updateHeadingPid(laneHeading);
    bool nearTarget = !lidarStale && lidarF <= startFrontMm + FRONT_SLOW_MM;
    if ((nearTarget && driven >= fsMinTicks) || driven >= fsMaxTicks) {
      if (!nearTarget) Serial.println(F("# final: encoder backstop reached, aligning on the front beam"));
      setMotorSpeed(0);
      fsPhase = FS_ALIGN; fsDir = 0;
      fsAlignT0 = fsStillSince = millis();
      Serial.println(F("# ALIGN"));
    }
    return;
  }

  // ---- FS_ALIGN: creep forward / backward until the front matches ----
  if (millis() - fsAlignT0 > ALIGN_TIMEOUT_MS) { finishFinal(F("(align timeout)")); return; }
  if (lidarStale) { setMotorSpeed(0); fsDir = 0; fsStillSince = millis(); return; }
  int err = (int)lidarF - (int)startFrontMm;       // + = still too far from the wall: forward
  if (fsDir == 0) {                                 // stopped: wait for a settled reading
    setMotorSpeed(0);
    if (millis() - fsStillSince < ALIGN_SETTLE_MS) return;
    if (abs(err) <= FRONT_DEADBAND_MM) { finishFinal(F("(front matched)")); return; }
    fsDir = (err > 0) ? 1 : -1;
    resetHeadingPid();
  }
  // moving: stop as soon as the reading crosses into the deadband (or past it)
  if ((fsDir > 0 && err <= FRONT_DEADBAND_MM) || (fsDir < 0 && err >= -(int)FRONT_DEADBAND_MM)) {
    setMotorSpeed(0);
    fsDir = 0;
    fsStillSince = millis();
    return;
  }
  if (fsDir > 0) { updateHeadingPid(laneHeading);   setMotorSpeed(ALIGN_PWM);  }
  else           { reverseHeadingHold(laneHeading); setMotorSpeed(-ALIGN_PWM); }
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
  // draining Serial and slow-blink the onboard LED. The start button is
  // not read here, so it can't launch the car before the Pi is up.
  if (!fsmStarted) {
    if (lidarFrames == 0) {
      statusBlink(500);
      return;
    }
    fsmStarted = true;
    statusLed(true);
    Serial.println(F("# first LiDAR frame received - FSM starting"));
  }

  // ---- button: start (WAIT_START), pause / resume (running), restart (FINISHED) ----
  btnPress = startButtonPressed();
  if (btnPress) {
    if (currentState == STATE_PAUSED) {
      btnPress = false;
      resumeRun();
    } else if (currentState == STATE_FINISHED) {
      btnPress = false;
      Serial.println(F("# RESTART (button)"));
      startRun();
    } else if (currentState != STATE_WAIT_START) {
      btnPress = false;
      pauseRun();
    }                                // WAIT_START: waitStartStep() consumes it
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
      currentState != STATE_PAUSED &&
      !driveTurnArmed() &&                 // a straight turns at the front fallback instead
      recoverTries < RECOVER_MAX_TRIES &&
      lidarF <= WALL_PANIC_MM) {
    enterRecovery();
  }

  // Onboard LED: solid while the Pi is feeding frames (ready or running),
  // fast blink if the feed goes stale. Always solid once FINISHED.
  if (currentState == STATE_FINISHED) statusLed(true);
  else if (currentState == STATE_PAUSED) statusBlink(250);
  else if (lidarStale)                statusBlink(100);
  else                                statusLed(true);

  switch (currentState) {
    case STATE_WAIT_START:      waitStartStep();     break;
    case STATE_DRIVE_TO_CORNER: driveStep();         break;
    case STATE_TURNING:         turningStep();       break;
    case STATE_FINAL_STRAIGHT:  finalStraightStep(); break;
    case STATE_RECOVER:         recoverStep();       break;
    case STATE_PAUSED:          setMotorSpeed(0);    break;   // until the next press

    case STATE_FINISHED:
      if (!entered) {
        entered = true;
        Serial.println(F("# FINISHED"));
        setMotorSpeed(0);
        setServoAngle(SERVO_TRUE_STRAIGHT);
        statusLed(true);
        digitalWrite(LED2_PIN, HIGH);
        digitalWrite(LED3_PIN, HIGH);
      }
      break;
  }
}
