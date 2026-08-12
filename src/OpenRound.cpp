#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>
#include <VL53L1X.h>          // === NEW === Pololu VL53L1X (front ToF only)

// ============================================================
// I2C MULTIPLEXER & ENUMS
// ============================================================
#define TCA_ADDR 0x70

enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };

// ------------------------------------------------------------
// HOW THIS RUN WORKS  (unchanged colour logic + two ToF additions)
//
// The colour lane-correction below is on the FLOOR, so the short
// 100 mm walls do not affect it. Two things are added on top:
//
//   * TURN TRIGGER = colour AND front, colour as the gate. A colour
//     line must be seen first (arms the turn and, on corner 1, locks
//     direction). Only then does the front wall closing within
//     FRONT_TURN_MM actually fire the turn. Neither fires alone, so a
//     stray colour read or a spurious front reading cannot turn the car.
//     The partner line is watched from the moment colour arms, so the
//     gap correction still works whether the partner is crossed before
//     or during the turn. (If the front sensor is absent it degrades to
//     colour-only so the car still runs.)
//
//   * RETURN-TO-START distance: record the start->first-turn distance
//     A, measure the full start-straight length L when the car re-
//     crosses it on lap 2, and drive (L - A) after the 12th turn so the
//     car finishes back on its start spot. The 12th corner now runs the
//     lane correction BEFORE the final straight so that final straight
//     begins from the same reference as the measured lap-2 straight.
//
// Original notes (still true):
// At every corner there are TWO lines (orange + blue) fanning out from
// the inner square. First line seen decides direction (orange->CW,
// blue->CCW). The partner line is caught mid-turn to measure the gap =
// how far out in the lane we are; a fixed sideways shuffle undoes the
// drift, then we realign to the dead-reckoned lane heading.
// ------------------------------------------------------------

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

bool    tcsConnected    = false;
uint8_t tcsChannel      = 0;
float   initialYawOffset = 0.0;

// === NEW === front ToF (VL53L1X) on the same mux
const uint8_t CH_FRONT = 4;                 // your scan: CH4 = FRONT
VL53L1X  tofF;
bool     tofFok = false;
uint16_t tofFront = 9999;
bool     tofFrontValid = false;
const uint16_t TOF_MAX_VALID_MM   = 1300;   // Short-mode ceiling
const float    SIGNAL_MIN_MCPS    = 4.0;    // reject weak returns (100 mm wall miss)
const uint16_t FRONT_TURN_MM      = 350;    // colour gates the turn; THIS front distance fires it

// ============================================================
// RUN CONSTANTS
// ============================================================
const int   TARGET_CORNERS   = 12;
const int   FINAL_STRAIGHT_CM = 100;    // fallback only; real value is measured
const float SEARCH_SAFETY_CM = 400.0;

// === NEW === return-to-start odometry
float firstSegmentCm      = 0.0;    // A: start -> first turn
float fullStartStraightCm = 0.0;    // L: full length of the start straight (lap 2)
bool  haveFullStraight    = false;
float finalDistanceCm     = FINAL_STRAIGHT_CM;  // = L - A once known

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

// ============================================================
// FSM DATA
// ============================================================
RobotState currentState = STATE_INIT;

BlockColor lockedColor   = COLOR_NONE;
bool       clockwiseMode = true;
int        cornerCount   = 0;

float targetHeading = 0.0;
float laneHeading   = 0.0;

float      lastGapCm       = 0.0;
BlockColor lastFirstColor  = COLOR_NONE;
long       cornerFirstTicks = 0;
bool       gapMeasured     = false;   // set once the partner line is caught this corner

// "Centred" reference gap. GAP_THRESHOLD_CM is only the fallback; the real
// value is learned from the gap measured at the FIRST corner (where the car
// starts from a known-good lateral position) and used for corners 2..12.
float      gapRefCm        = GAP_THRESHOLD_CM;
bool       gapRefSet       = false;

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
// 1. MOTOR & STEERING FUNCTIONS
// ============================================================
void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(RPWM_PIN, speed);
    analogWrite(LPWM_PIN, 0);
  } else if (speed < 0) {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, -speed);
  } else {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
  }
}

void setServoAngle(float angleDeg) {
  angleDeg = constrain(angleDeg, 5.0, 115.0);
  int pulseWidth = (int)((angleDeg / 180.0) * 1000.0) + 1000;
  steeringServo.writeMicroseconds(pulseWidth);
}

// ============================================================
// 2. ENCODER FUNCTIONS
// ============================================================
void zeroEncoder() {
  TIM3->CNT = 0;
}

long readEncoder() {
  return (int16_t)TIM3->CNT;
}

// ============================================================
// 3. IMU (YAW & HEADING) FUNCTIONS
// ============================================================
float readYaw() {
  float qI    = myIMU.getQuatI();
  float qJ    = myIMU.getQuatJ();
  float qK    = myIMU.getQuatK();
  float qReal = myIMU.getQuatReal();

  if (qI == 0.0f && qJ == 0.0f && qK == 0.0f && qReal == 0.0f) {
    return 0.0f;
  }

  float yawRadians = atan2(2.0f * (qI * qJ + qReal * qK),
                           (qReal * qReal + qI * qI - qJ * qJ - qK * qK));
  return yawRadians * (180.0 / PI);
}

void zeroYaw() {
  Serial.println(F("Waiting for valid IMU data to set Zero..."));
  unsigned long timeout = millis();
  while (millis() - timeout < 3000) {
    if (myIMU.wasReset()) {
      myIMU.enableGameRotationVector();
    }
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      initialYawOffset = readYaw();
      Serial.print(F("Zero Yaw successfully locked at: "));
      Serial.println(initialYawOffset);
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
// 4. COLOUR SENSOR
// ============================================================
void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (tcsConnected) {
    tcaselect(tcsChannel);
    tcs.getRawData(&r, &g, &b, &c);
  } else {
    r = 0; g = 0; b = 0; c = 0;
  }
}

BlockColor pendingColor    = COLOR_NONE;
int        consecutiveHits = 0;

void resetColorDetector() {
  pendingColor    = COLOR_NONE;
  consecutiveHits = 0;
}

BlockColor otherColor(BlockColor c) {
  if (c == COLOR_ORANGE) return COLOR_BLUE;
  if (c == COLOR_BLUE)   return COLOR_ORANGE;
  return COLOR_NONE;
}

BlockColor detectColor(BlockColor wantColor) {
  const int HITS_NEEDED = 2;
  uint16_t r, g, b, c;
  readColor(r, g, b, c);

  BlockColor rawColor  = COLOR_NONE;
  float      totalLight = r + g + b;

  if (totalLight > 0) {
    float pR = ((float)r / totalLight) * 100.0;
    float pB = ((float)b / totalLight) * 100.0;
    if (pB > 36.0 && pR < 24.0) {
      rawColor = COLOR_BLUE;
    } else if (pR > 35.0 && pB < 27.0) {
      rawColor = COLOR_ORANGE;
    }
  }

  if (wantColor != COLOR_NONE && rawColor != wantColor) {
    rawColor = COLOR_NONE;
  }

  if (rawColor == COLOR_NONE) {
    resetColorDetector();
    return COLOR_NONE;
  }

  if (rawColor == pendingColor) {
    consecutiveHits++;
  } else {
    pendingColor    = rawColor;
    consecutiveHits = 1;
  }

  if (consecutiveHits >= HITS_NEEDED) {
    resetColorDetector();
    return rawColor;
  }
  return COLOR_NONE;
}

// ============================================================
// === NEW === FRONT ToF SERVICE (non-blocking, signal-filtered)
//
// Selects the front channel and reads only if a fresh sample is ready,
// keeping it only when RangeValid, signal >= SIGNAL_MIN_MCPS, and in
// range. Weak returns (the 100 mm wall missed at distance) are dropped.
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
  htim3.Instance           = TIM3;
  htim3.Init.Prescaler     = 0;
  htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;
  htim3.Init.Period        = 65535;
  sConfig.EncoderMode      = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity      = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection     = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Polarity      = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection     = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  Wire.setClock(400000);   // === NEW === VL53L1X + TCS both fine at 400 kHz
  delay(100);

  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);
    delay(10);
    if (tcs.begin()) {
      tcsConnected = true;
      tcsChannel   = i;
      Serial.print(F("TCS34725 connected on TCA channel "));
      Serial.println(i);
      break;
    }
  }
  if (!tcsConnected) {
    Serial.println(F("WARNING: TCS34725 not detected through TCA9548A!"));
  }

  // === NEW === front ToF init (Short mode, 33 ms budget)
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
// HEADING PID  (bench-tuned: Kp=2, D on filtered yaw-rate, integral,
// servo slew limit + anti-windup. Kd/Ki are 0 - Kp alone gave RMS ~0.4)
// ============================================================
const float HEAD_KP        = 2.0;    // straight-line proportional gain
const float HEAD_KD        = 0.0;    // on filtered yaw-rate
const float HEAD_KI        = 0.0;    // standing-error trim (0 = off)
const float YAW_FILT_ALPHA = 0.35;   // low-pass on the yaw-rate D term
const float SERVO_SLEW     = 2.5;    // max servo change per tick (deg) - anti-thrash
const float INTEGRAL_CLAMP = 300.0;

float         pidPrevError  = 0.0;
unsigned long pidPrevTime   = 0;
float         pidIntegral   = 0.0;
float         yawFilt       = 0.0;
float         prevServoCmd  = SERVO_TRUE_STRAIGHT;
float         prevHeadingR  = 0.0;
unsigned long prevRateTime  = 0;

void resetHeadingPid() {
  pidPrevError = 0.0;  pidPrevTime = millis();
  pidIntegral  = 0.0;  yawFilt = 0.0;
  prevServoCmd = SERVO_TRUE_STRAIGHT;
  prevHeadingR = readHeading();  prevRateTime = millis();
}

void updateHeadingPid(float heading) {
  if (myIMU.wasReset()) myIMU.enableGameRotationVector();

  if (myIMU.getSensorEvent() &&
      myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    unsigned long now = millis();
    float h = readHeading();

    // filtered yaw-rate for the D term (D on measurement, not on error)
    float dtR = (now - prevRateTime) / 1000.0;
    float yawRate = (dtR > 0.0) ? wrapDeg(h - prevHeadingR) / dtR : 0.0;
    prevHeadingR = h;  prevRateTime = now;
    yawFilt += YAW_FILT_ALPHA * (yawRate - yawFilt);

    float dt = (now - pidPrevTime) / 1000.0;
    if (dt <= 0.0) dt = 0.001;

    float error = wrapDeg(heading - h);
    pidIntegral += error * dt;
    pidIntegral  = constrain(pidIntegral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

    float correction = HEAD_KP * error + HEAD_KI * pidIntegral - HEAD_KD * yawFilt;
    float want = SERVO_TRUE_STRAIGHT - correction;
    float dcmd = constrain(want - prevServoCmd, -SERVO_SLEW, SERVO_SLEW);   // slew limit
    float cmd  = constrain(prevServoCmd + dcmd, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
    if (cmd <= SERVO_MAX_LEFT || cmd >= SERVO_MAX_RIGHT) pidIntegral -= error * dt; // anti-windup

    setServoAngle(cmd);
    prevServoCmd = cmd;
    pidPrevError = error;
    pidPrevTime  = now;
  }
}

// ============================================================
// NAVIGATION FUNCTIONS
// ============================================================
// Bench-tuned turn law (sub-0.2 deg landings). Steering eases with the
// remaining angle and speed drops near the target, so the car
// decelerates into the heading instead of coasting past.
const float TURN_KP        = 2.5;    // servo deg per deg of heading error
const float TURN_MAX_STEER = 55.0;   // near full lock when far
const float TURN_MIN_STEER = 8.0;    // keep some authority near target
const float TURN_KV        = 3.5;    // pwm per deg of error
const int   TURN_MAX_PWM   = 100;
const int   TURN_MIN_PWM   = 65;
const float TURN_STOP_DEG  = 0.3;    // sub-degree stop
const int   TURN_SETTLE_MS = 300;

// Turn (either direction) to an absolute heading, easing in. Used by the
// lane correction, so it must resolve direction from the IMU error, not
// from lap direction.
void turnToHeading(float targetGlobalHeading) {
  targetGlobalHeading = wrapDeg(targetGlobalHeading);

  if (fabs(wrapDeg(targetGlobalHeading - readHeading())) < TURN_STOP_DEG) {
    setServoAngle(SERVO_TRUE_STRAIGHT);
    Serial.println(F(">>> Realign not needed."));
    return;
  }

  zeroEncoder();
  long safetyTicks = (long)(REALIGN_SAFETY_CM * TICKS_PER_CM);

  while (true) {
    long ticksNow = readEncoder();
    if (ticksNow < 0) ticksNow = -ticksNow;
    if (ticksNow >= safetyTicks) { Serial.println(F(">>> WARN: realign cap")); break; }

    if (myIMU.wasReset()) myIMU.enableGameRotationVector();
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      float err = wrapDeg(targetGlobalHeading - readHeading());
      if (fabs(err) < TURN_STOP_DEG) break;
      float mag   = fabs(err);
      float steer = constrain(TURN_KP * mag, TURN_MIN_STEER, TURN_MAX_STEER);
      int   pwm   = (int)constrain(TURN_KV * mag, (float)TURN_MIN_PWM, (float)TURN_MAX_PWM);
      // err>0 => need heading UP => turn LEFT => servo below centre
      float servo = (err > 0) ? (SERVO_TRUE_STRAIGHT - steer) : (SERVO_TRUE_STRAIGHT + steer);
      setServoAngle(servo);
      setMotorSpeed(pwm);
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(TURN_SETTLE_MS);
  Serial.print(F(">>> Realigned. Heading now: "));
  Serial.println(readHeading());
}

// Drive a fixed distance in cm while holding a heading.
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

// The 90 deg corner turn, now on the eased turn law. Terminates on IMU
// heading (sub-degree), and still watches for the partner colour line
// mid-turn to measure the lane gap. The encoder is NOT zeroed here - it
// keeps counting from driveToCorner's zero so (readEncoder-cornerFirstTicks)
// remains the distance since the first line. Turn distance for the safety
// cap is tracked separately from startTicks.
void turnDegrees(float degree, float fromHeading) {
  BlockColor wantSecond = otherColor(lastFirstColor);
  resetColorDetector();
  bool gotSecond = gapMeasured;   // keep a pre-turn gap if already caught

  float finalTargetHeading = wrapDeg(fromHeading - degree);  // right(+) decreases heading

  long startTicks = readEncoder();
  long safetyTicks = (long)(120.0 * TICKS_PER_CM);   // generous cap on a 90 deg arc

  while (true) {
    long moved = readEncoder() - startTicks;
    if (moved < 0) moved = -moved;
    if (moved >= safetyTicks) { Serial.println(F("  WARN: turn distance cap")); break; }

    if (myIMU.wasReset()) myIMU.enableGameRotationVector();
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      float err = wrapDeg(finalTargetHeading - readHeading());
      if (fabs(err) < TURN_STOP_DEG) break;
      float mag   = fabs(err);
      float steer = constrain(TURN_KP * mag, TURN_MIN_STEER, TURN_MAX_STEER);
      int   pwm   = (int)constrain(TURN_KV * mag, (float)TURN_MIN_PWM, (float)TURN_MAX_PWM);
      float servo = (err > 0) ? (SERVO_TRUE_STRAIGHT - steer) : (SERVO_TRUE_STRAIGHT + steer);
      setServoAngle(servo);
      setMotorSpeed(pwm);
    }

    // watch the corner's partner line while arcing through the turn
    if (!gotSecond && detectColor(wantSecond) != COLOR_NONE) {
      long travelled = readEncoder() - cornerFirstTicks;
      if (travelled < 0) travelled = -travelled;
      lastGapCm = (float)travelled / TICKS_PER_CM;
      gotSecond = true;
      gapMeasured = true;
      Serial.print(F("  partner line mid-turn, gap="));
      Serial.print(lastGapCm); Serial.println(F(" cm"));
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(TURN_SETTLE_MS);

  if (!gotSecond) {
    lastGapCm = gapRefCm;   // == reference -> lane correction becomes a no-op this corner
    Serial.println(F("  WARN: partner line missed, skipping correction"));
  }
  Serial.print(F(">>> TURN COMPLETE. Heading: "));
  Serial.println(readHeading());
}

// ============================================================
// CORNER MEASUREMENT  (turn trigger = colour AND front, colour is the gate)
//
// A corner needs BOTH:
//   GATE   : a colour line seen first  (corner 1: any colour, and it
//            locks direction; later: the locked colour). Arming records
//            cornerFirstTicks - the zero point for the gap measurement.
//   CONFIRM: after arming, the front wall closing within FRONT_TURN_MM
//            fires the turn. (If no front sensor is present, it degrades
//            to colour-only so the car still runs.)
//
// Between arming and firing, the partner line is watched too, so the gap
// correction works whether the partner is crossed on the straight or
// later mid-turn (turnDegrees respects gapMeasured).
//
// Records return-to-start distances:
//   corner 1  -> firstSegmentCm (A)
//   corner 5  -> fullStartStraightCm (L), refined on corner 9
// ============================================================
bool driveToCorner(float heading) {
  BlockColor wantFirst = lockedColor;

  resetColorDetector();
  resetHeadingPid();
  zeroEncoder();
  setMotorSpeed(BASE_SPEED);

  bool firstTurn = (lockedColor == COLOR_NONE);

  long lockoutTicks = 0;
  if (cornerCount > 0) {
    lockoutTicks = (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM);
  }
  long safetyTicks = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);

  bool colorArmed = false;
  gapMeasured = false;
  bool cornerHit = false;

  while (true) {
    long ticksNow = readEncoder();
    if (ticksNow < 0) ticksNow = -ticksNow;
    if (ticksNow >= safetyTicks) break;

    updateHeadingPid(heading);
    serviceFrontToF();

    if (ticksNow > lockoutTicks) {
      if (!colorArmed) {
        // GATE: a colour line must be seen first.
        BlockColor c = detectColor(wantFirst);
        if (c != COLOR_NONE) {
          colorArmed = true;
          cornerFirstTicks = readEncoder();
          if (firstTurn) lastFirstColor = c;
          resetColorDetector();       // now hunt the partner line
          Serial.print(F("  colour gate armed: "));
          Serial.println(c == COLOR_ORANGE ? F("ORANGE") : F("BLUE"));
        }
      } else {
        // Watch the partner line for the gap (may cross before the turn).
        if (!gapMeasured && detectColor(otherColor(lastFirstColor)) != COLOR_NONE) {
          long tr = readEncoder() - cornerFirstTicks;
          if (tr < 0) tr = -tr;
          lastGapCm = (float)tr / TICKS_PER_CM;
          gapMeasured = true;
          Serial.print(F("  partner line pre-turn, gap="));
          Serial.print(lastGapCm); Serial.println(F(" cm"));
        }
        // CONFIRM: front wall close (or no front sensor -> colour-only).
        bool frontClose = tofFrontValid && (tofFront <= FRONT_TURN_MM);
        if (frontClose || !tofFok) {
          cornerHit = true;
          if (!tofFok) Serial.println(F("  (no front sensor: colour-only trigger)"));
          break;
        }
      }
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);

  if (!cornerHit) return false;

  // record return-to-start distances (this straight's length to arm point)
  float segCm = (float)cornerFirstTicks / TICKS_PER_CM;
  if (cornerCount == 0) {
    firstSegmentCm = segCm;
    Serial.print(F("[DIST] A start->turn1 = ")); Serial.print(firstSegmentCm); Serial.println(F(" cm"));
  } else if (cornerCount == 4) {
    fullStartStraightCm = segCm; haveFullStraight = true;
  } else if (cornerCount == 8 && haveFullStraight) {
    fullStartStraightCm = 0.5 * (fullStartStraightCm + segCm);   // refine
  }
  if (haveFullStraight) {
    finalDistanceCm = fullStartStraightCm - firstSegmentCm;
    if (finalDistanceCm < 0) finalDistanceCm = 0;
    Serial.print(F("[DIST] L=")); Serial.print(fullStartStraightCm);
    Serial.print(F("  final(L-A)=")); Serial.print(finalDistanceCm); Serial.println(F(" cm"));
  }

  return true;
}

// ============================================================
// LANE CORRECTION  (unchanged)
// ============================================================
void doLaneCorrection(float laneHeadingRef) {
  float delta = lastGapCm - gapRefCm;
  float mag = delta;
  if (mag < 0) mag = -mag;

  if (mag < GAP_DEADBAND_CM) {
    Serial.println(F("  correction skipped (inside deadband)"));
    return;
  }

  float steerSigned = CORRECTION_SIGN * delta;
  if (clockwiseMode) steerSigned = -steerSigned;

  float offset = K_LAT_DEG_PER_CM * mag;
  if (offset > MAX_LAT_OFFSET_DEG) offset = MAX_LAT_OFFSET_DEG;

  float servo;
  if (steerSigned > 0) {
    servo = SERVO_TRUE_STRAIGHT - offset;
  } else {
    servo = SERVO_TRUE_STRAIGHT + offset;
  }

  Serial.print(F("  delta="));
  Serial.print(delta);
  Serial.print(F("  steer "));
  if (steerSigned > 0) Serial.print(F("LEFT "));
  else                 Serial.print(F("RIGHT "));
  Serial.print(offset);
  Serial.println(F(" deg"));

  zeroEncoder();
  long targetTicks = (long)(CORRECTION_DISTANCE_CM * TICKS_PER_CM);
  setServoAngle(servo);
  delay(150);
  setMotorSpeed(CORRECTION_PWM);

  while (true) {
    long ticksNow = readEncoder();
    if (ticksNow < 0) ticksNow = -ticksNow;
    if (ticksNow >= targetTicks) break;
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100);

  turnToHeading(laneHeadingRef);
}

// ============================================================
// MAIN SETUP & LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  initHardware();
  delay(2000);
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
          Serial.println(clockwiseMode
            ? F("[FSM] LOCKED ORANGE -> CLOCKWISE (right turns)")
            : F("[FSM] LOCKED BLUE -> COUNTERCLOCKWISE (left turns)"));
        }
        currentState = STATE_TURNING;
      } else {
        Serial.println(F("[FSM] WARN: no line within safety distance, retrying"));
      }
      break;
    }

    case STATE_TURNING: {
      cornerCount++;
      Serial.print(F("[FSM] TURNING - corner "));
      Serial.print(cornerCount);
      Serial.print(F(" / "));
      Serial.println(TARGET_CORNERS);

      float turnAmount = clockwiseMode ? 90.0 : -90.0;
      turnDegrees(turnAmount, laneHeading);
      laneHeading = wrapDeg(laneHeading - turnAmount);

      // Learn the "centred" gap from the FIRST corner and hold it for the
      // rest. Only trust a real measurement (partner line actually caught).
      if (cornerCount == 1 && !gapRefSet && gapMeasured) {
        gapRefCm  = lastGapCm;
        gapRefSet = true;
        Serial.print(F("[GAP] reference learned from turn 1 = "));
        Serial.print(gapRefCm); Serial.println(F(" cm"));
      }

      Serial.print(F("  lane heading is now "));
      Serial.print(laneHeading);
      Serial.print(F(", IMU says "));
      Serial.println(readHeading());

      // === NEW === always run the lane correction (even after turn 12)
      // so the final straight starts from the same reference as the
      // measured lap-2 straight. The LANE_CORRECT state decides whether
      // the next state is another straight or the final straight.
      currentState = STATE_LANE_CORRECT;
      break;
    }

    case STATE_LANE_CORRECT: {
      doLaneCorrection(laneHeading);
      targetHeading = laneHeading;

      if (cornerCount >= TARGET_CORNERS) currentState = STATE_FINAL_STRAIGHT;  // === NEW ===
      else                               currentState = STATE_DRIVE_TO_CORNER;
      break;
    }

    case STATE_FINAL_STRAIGHT: {
      // === NEW === drive the measured (L - A) instead of a fixed 100 cm
      Serial.print(F("[FSM] FINAL_STRAIGHT "));
      Serial.print(finalDistanceCm);
      Serial.println(F("cm  (L - A)"));
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
