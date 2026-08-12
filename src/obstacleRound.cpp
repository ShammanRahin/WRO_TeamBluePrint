#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>
#include <VL53L1X.h>

// ============================================================
// NON-BLOCKING BUILD
//
// loop() runs once per iteration at max speed: it services ALL sensors
// (IMU, front ToF; colour on demand) each pass and then advances a
// cooperative state machine. There are NO blocking while-loops and NO
// delay() calls in the run path - every former blocking wait (turn
// settle, servo-reach, post-move) is a millis() phase timer, and the
// colour confirm is a TIME window (not a loop-iteration count) so it
// behaves identically no matter how fast the loop spins.
//
// The control math (Kp=2 straight controller, eased turn law, gap
// correction, return-to-start odometry, colour+front turn trigger) is
// unchanged from the tuned blocking version.
// ============================================================

#define TCA_ADDR 0x70

enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };

enum RobotState {
  STATE_INIT,
  STATE_DRIVE_TO_CORNER,
  STATE_TURNING,
  STATE_LANE_CORRECT,
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
const float SERVO_MAX_LEFT      = 5.0;
const float SERVO_MAX_RIGHT     = 115.0;
const int   BASE_SPEED          = 120;

SPIClass SPI_IMU(PB5, PB4, PB3);
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

bool    tcsConnected     = false;
uint8_t tcsChannel       = 0;
float   initialYawOffset = 0.0;

// front ToF (VL53L1X) on the mux
const uint8_t CH_FRONT = 4;
VL53L1X  tofF;
bool     tofFok = false;
uint16_t tofFront = 9999;
bool     tofFrontValid = false;
const uint16_t TOF_MAX_VALID_MM = 1300;
const float    SIGNAL_MIN_MCPS  = 4.0;
const uint16_t FRONT_TURN_MM    = 700;

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
const float K_LAT_DEG_PER_CM       =  1.5;
const float MAX_LAT_OFFSET_DEG     = 30.0;
const float CORRECTION_DISTANCE_CM = 25.0;
const float POST_CORNER_LOCKOUT_CM = 50;
const int   CORRECTION_SIGN        =   1;
const int   CORRECTION_PWM         =   70;
const float REALIGN_SAFETY_CM      = 80.0;
const unsigned long SERVO_REACH_MS = 150;   // was delay(150) before a shuffle
const unsigned long POST_MOVE_MS   = 100;   // was delay(100) after a shuffle

// ============================================================
// FSM DATA
// ============================================================
RobotState currentState = STATE_INIT;
bool          entered   = false;    // false right after a state change
unsigned long phaseT0   = 0;        // generic phase timer start

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

// ---- cached sensor readings, refreshed every loop ----
bool          gImuFresh = false;
float         gHeading  = 0.0;
float         gYawRate  = 0.0;
float         gPrevH    = 0.0;
unsigned long gPrevHT   = 0;

// ============================================================
// ANGLE HELPER
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

// ============================================================
// MOTOR / STEERING / ENCODER
// ============================================================
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

float readHeading() {
  return fmod(readYaw() - initialYawOffset + 540.0, 360.0) - 180.0;
}

// Startup-only blocking zero (fine - the car is not running yet).
void zeroYaw() {
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

// ============================================================
// COLOUR SENSOR  (time-based confirm)
// ============================================================
// Direct register read - NO delay. Adafruit's getRawData() blocks for the
// integration time on every call, which would re-introduce blocking. The
// TCS34725 integrates continuously once begin() enables it, so the C/R/G/B
// data registers always hold the latest completed reading; we just read them.
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

const unsigned long COLOR_CONFIRM_MS = 6;   // was "2 consecutive reads" (~5 ms)
BlockColor    pendingColor  = COLOR_NONE;
unsigned long pendingStart  = 0;

void resetColorDetector() { pendingColor = COLOR_NONE; pendingStart = 0; }

BlockColor otherColor(BlockColor c) {
  if (c == COLOR_ORANGE) return COLOR_BLUE;
  if (c == COLOR_BLUE)   return COLOR_ORANGE;
  return COLOR_NONE;
}

// Returns wantColor (or either if wantColor==NONE) once it has been seen
// continuously for COLOR_CONFIRM_MS. Independent of loop rate.
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

  if (rawColor != pendingColor) {          // first sighting of this colour
    pendingColor = rawColor;
    pendingStart = millis();
    return COLOR_NONE;
  }
  if (millis() - pendingStart >= COLOR_CONFIRM_MS) {   // held long enough
    resetColorDetector();
    return rawColor;
  }
  return COLOR_NONE;
}

// ============================================================
// FRONT ToF (non-blocking, signal-filtered)
// ============================================================
void serviceFrontToF() {
  if (!tofFok) { tofFrontValid = false; return; }
  tcaselect(CH_FRONT);
  if (tofF.dataReady()) {
    uint16_t d = tofF.read(false);
    bool ok = (tofF.ranging_data.range_status == VL53L1X::RangeValid) &&
              (tofF.ranging_data.peak_signal_count_rate_MCPS >= SIGNAL_MIN_MCPS) &&
              (d > 0) && (d < TOF_MAX_VALID_MM);
    tofFrontValid = ok;
    if (ok) tofFront = d;
  }
}

// ============================================================
// SENSOR SERVICE  (called once at the top of every loop)
// ============================================================
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
}

// ============================================================
// SYSTEM INITIALIZATION
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
    tcaselect(i);
    delay(10);
    if (tcs.begin()) { tcsConnected = true; tcsChannel = i;
      Serial.print(F("TCS34725 on TCA channel ")); Serial.println(i); break; }
  }
  if (!tcsConnected) Serial.println(F("WARNING: TCS34725 not detected!"));

  tcaselect(CH_FRONT);
  tofF.setBus(&Wire);
  tofF.setTimeout(100);
  tofFok = tofF.init();
  if (tofFok) {
    tofF.setDistanceMode(VL53L1X::Short);
    tofF.setMeasurementTimingBudget(33000);
    tofF.startContinuous(33);
  }
  Serial.println(tofFok ? F("Front ToF (CH4) ready") : F("Front ToF (CH4) FAILED"));

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
// HEADING PID  (bench-tuned Kp=2; uses cached sensor data, acts only on
// a fresh IMU sample so its dt is the IMU report interval regardless of
// loop speed)
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
  if (!gImuFresh) return;                 // only act on a fresh sample
  unsigned long now = millis();

  yawFilt += YAW_FILT_ALPHA * (gYawRate - yawFilt);   // D on filtered rate

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
// EASED TURN LAW  (shared by corner turns and realign)
// ============================================================
const float TURN_KP        = 2.5;
const float TURN_MAX_STEER = 55.0;
const float TURN_MIN_STEER = 8.0;
const float TURN_KV        = 3.5;
const int   TURN_MAX_PWM   = 100;
const int   TURN_MIN_PWM   = 65;
const float TURN_STOP_DEG  = 0.3;
const unsigned long TURN_SETTLE_MS = 300;

long turnStartTicks = 0;
long turnCapTicks   = 0;

// One arc step toward an absolute heading. Returns true when the arc is
// finished (heading reached or distance cap). Sets servo/motor while running.
bool turnArcStep(float target) {
  if (absEnc(readEncoder() - turnStartTicks) >= turnCapTicks) return true;
  if (gImuFresh) {
    float err = wrapDeg(target - gHeading);
    if (fabs(err) < TURN_STOP_DEG) return true;
    float mag   = fabs(err);
    float steer = constrain(TURN_KP * mag, TURN_MIN_STEER, TURN_MAX_STEER);
    int   pwm   = (int)constrain(TURN_KV * mag, (float)TURN_MIN_PWM, (float)TURN_MAX_PWM);
    // err>0 => need heading UP => turn LEFT => servo below centre
    float servo = (err > 0) ? (SERVO_TRUE_STRAIGHT - steer) : (SERVO_TRUE_STRAIGHT + steer);
    setServoAngle(servo);
    setMotorSpeed(pwm);
  }
  return false;
}

// ============================================================
// STATE HELPERS
// ============================================================
void goState(RobotState s) { currentState = s; entered = false; }
bool waited(unsigned long ms) { return (millis() - phaseT0) >= ms; }

// ---- per-state working vars ----
BlockColor dcWantFirst;
bool       dcFirstTurn;
bool       dcColorArmed;
long       dcLockoutTicks;
long       dcSafetyTicks;

uint8_t    turnPhase;          // 0 = arc, 1 = settle
float      turnTarget;
float      turnAmount;
BlockColor turnPartner;
bool       turnGotSecond;

uint8_t    lcPhase;            // 1 servo-reach, 2 shuffle, 3 post, 4 realign-arc, 5 realign-settle
long       lcTargetTicks;
long       fsTargetTicks;

void finishLaneCorrect() {
  targetHeading = laneHeading;
  if (cornerCount >= TARGET_CORNERS) goState(STATE_FINAL_STRAIGHT);
  else                               goState(STATE_DRIVE_TO_CORNER);
}

// ============================================================
// STATE: DRIVE TO CORNER   (straight hold + colour/front trigger)
// ============================================================
void driveStep() {
  if (!entered) {
    entered = true;
    Serial.println(F("[FSM] DRIVE_TO_CORNER"));
    dcWantFirst = lockedColor;
    dcFirstTurn = (lockedColor == COLOR_NONE);
    resetColorDetector();
    resetHeadingPid();
    zeroEncoder();
    setMotorSpeed(BASE_SPEED);
    dcColorArmed   = false;
    gapMeasured    = false;
    dcLockoutTicks = (cornerCount > 0) ? (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM) : 0;
    dcSafetyTicks  = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);
  }

  long ticksNow = absEnc(readEncoder());
  if (ticksNow >= dcSafetyTicks) {
    Serial.println(F("[FSM] WARN: no line within safety distance, retrying"));
    entered = false;   // restart the search
    return;
  }

  updateHeadingPid(targetHeading);   // smooth straight, acts on fresh IMU only

  if (ticksNow <= dcLockoutTicks) return;

  if (!dcColorArmed) {
    BlockColor c = detectColor(dcWantFirst);
    if (c != COLOR_NONE) {
      dcColorArmed = true;
      cornerFirstTicks = readEncoder();
      if (dcFirstTurn) lastFirstColor = c;
      resetColorDetector();
      Serial.print(F("  colour gate armed: "));
      Serial.println(c == COLOR_ORANGE ? F("ORANGE") : F("BLUE"));
    }
    return;
  }

  // armed: watch partner line for the gap (may cross before the turn)
  if (!gapMeasured && detectColor(otherColor(lastFirstColor)) != COLOR_NONE) {
    long tr = absEnc(readEncoder() - cornerFirstTicks);
    lastGapCm = (float)tr / TICKS_PER_CM;
    gapMeasured = true;
    Serial.print(F("  partner line pre-turn, gap="));
    Serial.print(lastGapCm); Serial.println(F(" cm"));
  }

  // confirm: front wall close (or no front sensor -> colour-only)
  bool frontClose = tofFrontValid && (tofFront <= FRONT_TURN_MM);
  if (frontClose || !tofFok) {
    if (!tofFok) Serial.println(F("  (no front sensor: colour-only trigger)"));
    setMotorSpeed(0);

    // lock lap direction on the first corner
    if (lockedColor == COLOR_NONE) {
      lockedColor   = lastFirstColor;
      clockwiseMode = (lockedColor == COLOR_ORANGE);
      Serial.println(clockwiseMode
        ? F("[FSM] LOCKED ORANGE -> CLOCKWISE (right turns)")
        : F("[FSM] LOCKED BLUE -> COUNTERCLOCKWISE (left turns)"));
    }

    // record return-to-start distances
    float segCm = (float)cornerFirstTicks / TICKS_PER_CM;
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

    goState(STATE_TURNING);
  }
}

// ============================================================
// STATE: TURNING  (eased 90 deg corner turn + mid-turn partner watch)
// ============================================================
void turningStep() {
  if (!entered) {
    entered = true;
    cornerCount++;
    Serial.print(F("[FSM] TURNING - corner "));
    Serial.print(cornerCount); Serial.print(F(" / ")); Serial.println(TARGET_CORNERS);

    turnAmount    = clockwiseMode ? 90.0 : -90.0;
    turnTarget    = wrapDeg(laneHeading - turnAmount);   // right(+) decreases heading
    turnPartner   = otherColor(lastFirstColor);
    resetColorDetector();
    turnGotSecond = gapMeasured;
    turnStartTicks = readEncoder();
    turnCapTicks   = (long)(120.0 * TICKS_PER_CM);
    turnPhase = 0;
  }

  if (turnPhase == 0) {                 // ---- arc ----
    // watch the corner's partner line while arcing
    if (!turnGotSecond && detectColor(turnPartner) != COLOR_NONE) {
      long tr = absEnc(readEncoder() - cornerFirstTicks);
      lastGapCm = (float)tr / TICKS_PER_CM;
      turnGotSecond = true;
      gapMeasured   = true;
      Serial.print(F("  partner line mid-turn, gap="));
      Serial.print(lastGapCm); Serial.println(F(" cm"));
    }
    if (turnArcStep(turnTarget)) {       // arc finished
      setMotorSpeed(0);
      setServoAngle(SERVO_TRUE_STRAIGHT);
      phaseT0 = millis();
      turnPhase = 1;
    }
  } else {                              // ---- settle ----
    if (waited(TURN_SETTLE_MS)) {
      if (!turnGotSecond) {
        lastGapCm = gapRefCm;            // reference -> lane correction is a no-op
        Serial.println(F("  WARN: partner line missed, skipping correction"));
      }
      laneHeading = wrapDeg(laneHeading - turnAmount);

      if (cornerCount == 1 && !gapRefSet && gapMeasured) {
        gapRefCm  = lastGapCm;
        gapRefSet = true;
        Serial.print(F("[GAP] reference learned from turn 1 = "));
        Serial.print(gapRefCm); Serial.println(F(" cm"));
      }
      Serial.print(F("  lane heading now ")); Serial.print(laneHeading);
      Serial.print(F(", IMU ")); Serial.println(readHeading());
      goState(STATE_LANE_CORRECT);
    }
  }
}

// ============================================================
// STATE: LANE CORRECT  (shuffle then realign, or skip if centred)
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

    zeroEncoder();
    setServoAngle(servo);
    lcTargetTicks = (long)(CORRECTION_DISTANCE_CM * TICKS_PER_CM);
    phaseT0 = millis();
    lcPhase = 1;                          // servo-reach wait
    return;
  }

  switch (lcPhase) {
    case 1:                              // let the servo reach the angle
      if (waited(SERVO_REACH_MS)) {
        zeroEncoder();
        setMotorSpeed(CORRECTION_PWM);
        lcPhase = 2;
      }
      break;

    case 2:                              // shuffle a fixed distance
      if (absEnc(readEncoder()) >= lcTargetTicks) {
        setMotorSpeed(0);
        setServoAngle(SERVO_TRUE_STRAIGHT);
        phaseT0 = millis();
        lcPhase = 3;
      }
      break;

    case 3:                              // brief post-move settle, then realign
      if (waited(POST_MOVE_MS)) {
        if (fabs(wrapDeg(laneHeading - readHeading())) < TURN_STOP_DEG) {
          Serial.println(F(">>> Realign not needed."));
          finishLaneCorrect();
        } else {
          turnStartTicks = readEncoder();
          turnCapTicks   = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);
          lcPhase = 4;                    // realign arc
        }
      }
      break;

    case 4:                              // eased realign toward lane heading
      if (turnArcStep(laneHeading)) {
        setMotorSpeed(0);
        setServoAngle(SERVO_TRUE_STRAIGHT);
        phaseT0 = millis();
        lcPhase = 5;
      }
      break;

    case 5:                              // realign settle
      if (waited(TURN_SETTLE_MS)) {
        Serial.print(F(">>> Realigned. Heading ")); Serial.println(readHeading());
        finishLaneCorrect();
      }
      break;
  }
}

// ============================================================
// STATE: FINAL STRAIGHT
// ============================================================
void finalStraightStep() {
  if (!entered) {
    entered = true;
    Serial.print(F("[FSM] FINAL_STRAIGHT "));
    Serial.print(finalDistanceCm); Serial.println(F("cm (L - A)"));
    zeroEncoder();
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
  delay(2000);
}

void loop() {
  serviceSensors();   // IMU + front ToF fresh every pass, at full loop speed

  switch (currentState) {
    case STATE_INIT:
      Serial.println(F("[FSM] INIT"));
      lockedColor   = COLOR_NONE;
      cornerCount   = 0;
      laneHeading   = readHeading();
      targetHeading = laneHeading;
      goState(STATE_DRIVE_TO_CORNER);
      break;

    case STATE_DRIVE_TO_CORNER: driveStep();        break;
    case STATE_TURNING:         turningStep();      break;
    case STATE_LANE_CORRECT:    laneCorrectStep();  break;
    case STATE_FINAL_STRAIGHT:  finalStraightStep();break;

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
