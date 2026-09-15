#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>

// ============================================================
// OPEN ROUND - NON-BLOCKING FIRMWARE (new hardware revision)
//
// FSM logic is unchanged from the working build: loop() services every
// sensor once per pass at max speed, then advances a cooperative state
// machine. No blocking while-loops, no delay() in the run path. The car
// flows turn -> shuffle -> realign -> straight without stopping.
//
// WHAT CHANGED vs the old board
//   motor      PB8/PB9  -> PA2 (RPWM) / PA3 (LPWM), no DRV_EN (hard-tied)
//   encoder    TIM3 PA6/PA7 -> TIM5 PA0/PA1 (32-bit), PA6/PA7 now SPI
//   IMU SPI    PB5/PB4/PB3 -> PA7/PA6/PA5, CS PA4, INT PB0, RST PB1
//   I2C        PB7/PB6 (unchanged), TCA9548 hardware reset added on PB8
//   front ToF  VL53L1X CH4 -> VL53L0X CH3
//   colour     auto-scan -> fixed CH4, new thresholds
//   side ToF   REMOVED - not on this build
//   servo      centre 69 -> 71, travel 5..115 -> 1..150
//   ticks/cm   31.933 -> 14.853 (2x motor, new gearing)
//   speeds     straight-line PWM halved to match the old ground speed
//   start      button PB15 not wired -> fixed 5 s countdown
//
// TURN TRIGGER: with no side ToF the corner is confirmed by the colour
// line gate plus the front wall closing to FRONT_TURN_MM. A distance
// backstop fires the turn if the front sensor never confirms.
//
// Distance bookkeeping: the encoder is zeroed only at START and at each
// TURN COMPLETION, so it measures "distance since the last turn" through
// the lane correction; A, L and the final straight share that reference.
// ============================================================

enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };

enum RobotState {
  STATE_WAIT_START,
  STATE_DRIVE_TO_CORNER,
  STATE_TURNING,
  STATE_LANE_CORRECT,
  STATE_FINAL_STRAIGHT,
  STATE_FINISHED
};

// ============================================================
// HARDWARE PINS & OBJECTS
// ============================================================
const int MOT_RPWM_PIN = PA2;     // TIM2_CH3 (TIM5 is the encoder, no clash)
const int MOT_LPWM_PIN = PA3;     // TIM2_CH4
const int SERVO_PIN    = PA8;

const int IMU_CS_PIN  = PA4;
const int IMU_INT_PIN = PB0;
const int IMU_RST_PIN = PB1;

const int LED1_PIN = PB12;        // blinks during countdown, solid while running
const int LED2_PIN = PB13;        // lit while ORANGE is under the sensor
const int LED3_PIN = PB14;        // lit while BLUE is under the sensor
                                  // all three solid = FINISHED
const int BTN_START_PIN = PB15;   // not wired yet - see START_DELAY_MS

#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70
#define TOF_CH      3             // front VL53L0X
#define TCS_CH      4             // TCS34725

// ---- calibration ----
const float TICKS_PER_CM        = 14.853;   // 248.8 ticks/rev / (5.2 cm * PI)
const float SERVO_TRUE_STRAIGHT = 71.0;
const float SERVO_MAX_LEFT      = 1.0;
const float SERVO_MAX_RIGHT     = 150.0;

// ---- speeds: halved from the old build because the motor is 2x RPM ----
const int BASE_SPEED      = 70;   // was 150
const int CORRECTION_PWM  = 55;   // was 70; shuffle speed

const unsigned long START_DELAY_MS = 5000;

SPIClass SPI_IMU(PA7, PA6, PA5);  // MOSI, MISO, SCLK
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);
VL53L0X lox;

bool  tcsOk = false;
bool  tofOk = false;
float initialYawOffset = 0.0;

// ---- front ToF state ----
uint16_t tofFront = 9999;
bool     tofFrontValid = false;
const uint16_t TOF_MAX_VALID_MM = 1200;   // VL53L0X practical ceiling
const uint16_t FRONT_TURN_MM    = 700;

// ============================================================
// RUN CONSTANTS
// ============================================================
const int   TARGET_CORNERS    = 12;
const int   FINAL_STRAIGHT_CM = 100;
const float SEARCH_SAFETY_CM  = 400.0;
const float GATE_TO_TURN_MAX_CM = 150.0;  // backstop: turn even if front ToF never confirms

float firstSegmentCm      = 0.0;
float fullStartStraightCm = 0.0;
bool  haveFullStraight    = false;
float finalDistanceCm     = FINAL_STRAIGHT_CM;

// ============================================================
// LANE CORRECTION TUNING
// ============================================================
const float GAP_THRESHOLD_CM       = 20.0;
const float GAP_DEADBAND_CM        =  2.0;
const float K_LAT_DEG_PER_CM       =  4.0;
const float MAX_LAT_OFFSET_DEG     = 30.0;
const float CORRECTION_DISTANCE_CM = 25.0;
const float POST_CORNER_LOCKOUT_CM = 50.0;
const int   CORRECTION_SIGN        =   1;
const float REALIGN_SAFETY_CM      = 80.0;

// ============================================================
// FSM DATA
// ============================================================
RobotState    currentState = STATE_WAIT_START;
bool          entered = false;
unsigned long phaseT0 = 0;

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

// cached IMU, refreshed every loop
bool          gImuFresh = false;
float         gHeading  = 0.0;
float         gYawRate  = 0.0;
float         gPrevH    = 0.0;
unsigned long gPrevHT   = 0;

// cached colour, refreshed every loop - drives the status LEDs and the FSM
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
  digitalWrite(TCA_RST_PIN, LOW);    // active LOW
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

void setServoAngle(float angleDeg) {
  angleDeg = constrain(angleDeg, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  steeringServo.writeMicroseconds((int)((angleDeg / 180.0) * 1000.0) + 1000);
}

// ---- TIM5 encoder (32-bit counter on PA0/PA1) ----
void zeroEncoder() { TIM5->CNT = 0; }
long readEncoder() { return (int32_t)TIM5->CNT; }
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

// ---- colour: direct register read on CH4, NO delay ----
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

const unsigned long COLOR_CONFIRM_MS = 6;
BlockColor    pendingColor = COLOR_NONE;
unsigned long pendingStart = 0;

void resetColorDetector() { pendingColor = COLOR_NONE; pendingStart = 0; }

BlockColor otherColor(BlockColor c) {
  if (c == COLOR_ORANGE) return COLOR_BLUE;
  if (c == COLOR_BLUE)   return COLOR_ORANGE;
  return COLOR_NONE;
}

// Raw classification, thresholds from the bench test sketch.
// Run once per loop by serviceSensors() and cached in gRawColor.
BlockColor classifyColor() {
  uint16_t r, g, b, c;
  readColor(r, g, b, c);
  float total = (float)r + (float)g + (float)b;
  if (total < 100.0f) return COLOR_NONE;       // below this it is dark / no mat
  float pR = (r / total) * 100.0f;
  float pB = (b / total) * 100.0f;
  if (pB > 36.0f && pR < 28.0f) return COLOR_BLUE;
  if (pR > 36.0f && pB < 28.0f) return COLOR_ORANGE;
  return COLOR_NONE;
}

// Debounced, optionally filtered to one colour. Reads this loop's cached value.
BlockColor detectColor(BlockColor wantColor) {
  BlockColor rawColor = gRawColor;
  if (wantColor != COLOR_NONE && rawColor != wantColor) rawColor = COLOR_NONE;

  if (rawColor == COLOR_NONE)   { resetColorDetector(); return COLOR_NONE; }
  if (rawColor != pendingColor) { pendingColor = rawColor; pendingStart = millis(); return COLOR_NONE; }
  if (millis() - pendingStart >= COLOR_CONFIRM_MS) { resetColorDetector(); return rawColor; }
  return COLOR_NONE;
}

// ---- front ToF: poll the interrupt flag, never block ----
void serviceFrontToF() {
  if (!tofOk) { tofFrontValid = false; return; }
  tcaselect(TOF_CH);
  if (lox.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t d = lox.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    lox.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    tofFrontValid = (d > 0 && d < TOF_MAX_VALID_MM);
    if (tofFrontValid) tofFront = d;
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
  serviceFrontToF();

  // one colour read per loop, shared by the FSM and the LEDs
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
  setMotorSpeed(0);                 // BTS7960 ENs are hard-tied on the carrier

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(BTN_START_PIN, INPUT_PULLUP);   // reserved, not read yet
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
  htim5.Init.Period      = 0xFFFFFFFF;     // 32-bit counter
  sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim5, &sConfig);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  // ---- I2C bus + mux ----
  resetTCA();
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  tcaselect(TOF_CH);
  lox.setBus(&Wire);
  lox.setTimeout(100);
  tofOk = lox.init();
  if (tofOk) {
    lox.setMeasurementTimingBudget(30000);
    lox.startContinuous(0);
  }
  Serial.println(tofOk ? F("ToF FRONT (CH3): READY") : F("ToF FRONT (CH3): FAILED"));

  tcaselect(TCS_CH);
  delay(10);
  tcsOk = tcs.begin();
  Serial.println(tcsOk ? F("Colour (CH4): READY") : F("Colour (CH4): FAILED"));

  // ---- IMU over SPI1 ----
  SPI_IMU.begin();
  if (myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU)) {
    delay(500);
    myIMU.enableGameRotationVector();
    delay(100);
    myIMU.getSensorEvent();
    zeroYaw();
  } else {
    Serial.println(F("ERROR: IMU not found!"));
  }
}

// ============================================================
// HEADING PID  (cached sensor data, acts only on a fresh sample)
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
// EASED TURN LAW - verbatim from the tuned turning90 sketch.
// Already validated on the 2x motor, so these PWMs are NOT halved.
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
long       dcBaseTicks;       // encoder at this straight's start
long       dcLockoutTicks;
long       dcSafetyTicks;

float      turnTarget;
float      turnAmount;
BlockColor turnPartner;
bool       turnGotSecond;

uint8_t    lcPhase;           // 2 = shuffle, 4 = realign
long       lcBaseTicks;
long       lcTargetTicks;
long       fsTargetTicks;

void finishLaneCorrect() {
  targetHeading = laneHeading;
  if (cornerCount >= TARGET_CORNERS) goState(STATE_FINAL_STRAIGHT);
  else                               goState(STATE_DRIVE_TO_CORNER);
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
    Serial.println(F("[FSM] WAIT_START - 5 s"));
  }
  digitalWrite(LED1_PIN, ((millis() / 250) & 1) ? HIGH : LOW);

  if (millis() - phaseT0 >= START_DELAY_MS) {
    digitalWrite(LED1_PIN, HIGH);
    lockedColor   = COLOR_NONE;
    cornerCount   = 0;
    laneHeading   = gHeading;
    targetHeading = laneHeading;
    zeroEncoder();                 // segment origin = start position
    Serial.println(F("[FSM] GO"));
    goState(STATE_DRIVE_TO_CORNER);
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
    dcBaseTicks = readEncoder();   // NO zero - keep the segment origin
    setMotorSpeed(BASE_SPEED);
    dcColorArmed = false;
    gapMeasured  = false;
    dcLockoutTicks = (cornerCount > 0) ? (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM) : 0;
    dcSafetyTicks  = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);
  }

  long straightTicks = absEnc(readEncoder() - dcBaseTicks);   // distance this straight
  if (straightTicks >= dcSafetyTicks) {
    Serial.println(F("[FSM] WARN: no line within safety distance, retrying"));
    entered = false;
    return;
  }

  updateHeadingPid(targetHeading);

  if (straightTicks <= dcLockoutTicks) return;

  // ---- 1. arm on the first colour line ----
  if (!dcColorArmed) {
    BlockColor c = detectColor(dcWantFirst);
    if (c != COLOR_NONE) {
      dcColorArmed = true;
      cornerFirstTicks = readEncoder();        // absolute from segment origin
      if (dcFirstTurn) lastFirstColor = c;
      resetColorDetector();
      Serial.print(F("  colour gate armed: "));
      Serial.println(c == COLOR_ORANGE ? F("ORANGE") : F("BLUE"));
    }
    return;
  }

  // ---- 2. opportunistic partner-line gap measurement ----
  if (!gapMeasured && detectColor(otherColor(lastFirstColor)) != COLOR_NONE) {
    lastGapCm = (float)absEnc(readEncoder() - cornerFirstTicks) / TICKS_PER_CM;
    gapMeasured = true;
    Serial.print(F("  partner line pre-turn, gap="));
    Serial.print(lastGapCm); Serial.println(F(" cm"));
  }

  // ---- 3. turn confirm: front wall close (or a backstop) ----
  float gateRunCm = (float)absEnc(readEncoder() - cornerFirstTicks) / TICKS_PER_CM;
  bool frontClose = tofFrontValid && (tofFront <= FRONT_TURN_MM);
  bool noRange    = !tofOk;
  bool backstop   = gateRunCm >= GATE_TO_TURN_MAX_CM;

  if (frontClose || noRange || backstop) {
    if      (frontClose) Serial.println(F("  turn: FRONT close"));
    else if (noRange)    Serial.println(F("  turn: colour-only (no front ToF)"));
    else                 Serial.println(F("  turn: distance backstop"));

    if (lockedColor == COLOR_NONE) {
      lockedColor   = lastFirstColor;
      clockwiseMode = (lockedColor == COLOR_ORANGE);
      Serial.println(clockwiseMode
        ? F("[FSM] LOCKED ORANGE -> CLOCKWISE (right turns)")
        : F("[FSM] LOCKED BLUE -> COUNTERCLOCKWISE (left turns)"));
    }

    // distance from segment origin (start / last turn) to the arm point
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

    goState(STATE_TURNING);        // motor keeps rolling into the turn
  }
}

// ============================================================
// STATE: TURNING  (eased 90 deg arc + mid-turn partner watch, NO settle)
// ============================================================
void turningStep() {
  if (!entered) {
    entered = true;
    cornerCount++;
    Serial.print(F("[FSM] TURNING - corner "));
    Serial.print(cornerCount); Serial.print(F(" / ")); Serial.println(TARGET_CORNERS);
    turnAmount     = clockwiseMode ? 90.0 : -90.0;
    turnTarget     = wrapDeg(laneHeading - turnAmount);
    turnPartner    = otherColor(lastFirstColor);
    resetColorDetector();
    turnGotSecond  = gapMeasured;
    turnStartTicks = readEncoder();
    turnCapTicks   = (long)(120.0 * TICKS_PER_CM);
  }

  if (!turnGotSecond && detectColor(turnPartner) != COLOR_NONE) {
    lastGapCm = (float)absEnc(readEncoder() - cornerFirstTicks) / TICKS_PER_CM;
    turnGotSecond = true; gapMeasured = true;
    Serial.print(F("  partner line mid-turn, gap="));
    Serial.print(lastGapCm); Serial.println(F(" cm"));
  }

  if (turnArcStep(turnTarget)) {   // arc finished - proceed immediately
    if (!turnGotSecond) {
      lastGapCm = gapRefCm;
      Serial.println(F("  WARN: partner line missed, skipping correction"));
    }
    laneHeading = wrapDeg(laneHeading - turnAmount);
    if (cornerCount == 1 && !gapRefSet && gapMeasured) {
      gapRefCm = lastGapCm; gapRefSet = true;
      Serial.print(F("[GAP] reference learned from turn 1 = "));
      Serial.print(gapRefCm); Serial.println(F(" cm"));
    }
    zeroEncoder();                 // segment origin = this turn corner
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
    float mag   = fabs(delta);
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
    setMotorSpeed(CORRECTION_PWM); // shuffle immediately (servo slews as it rolls)
    lcBaseTicks   = readEncoder();
    lcTargetTicks = (long)(CORRECTION_DISTANCE_CM * TICKS_PER_CM);
    lcPhase = 2;
    return;
  }

  if (lcPhase == 2) {              // shuffle a fixed distance
    if (absEnc(readEncoder() - lcBaseTicks) >= lcTargetTicks) {
      turnStartTicks = readEncoder();
      turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
      lcPhase = 4;                 // realign (no stop)
    }
  } else {                         // lcPhase == 4: eased realign onto lane heading
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
    Serial.print(finalDistanceCm); Serial.println(F(" cm from turn 12"));
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

  switch (currentState) {
    case STATE_WAIT_START:      waitStartStep();     break;
    case STATE_DRIVE_TO_CORNER: driveStep();         break;
    case STATE_TURNING:         turningStep();       break;
    case STATE_LANE_CORRECT:    laneCorrectStep();   break;
    case STATE_FINAL_STRAIGHT:  finalStraightStep(); break;

    case STATE_FINISHED:
      if (!entered) {
        entered = true;
        Serial.println(F("[FSM] FINISHED"));
        setMotorSpeed(0);
        setServoAngle(SERVO_TRUE_STRAIGHT);
        digitalWrite(LED1_PIN, HIGH);   // all three solid = finished
        digitalWrite(LED2_PIN, HIGH);
        digitalWrite(LED3_PIN, HIGH);
      }
      break;
  }
}
