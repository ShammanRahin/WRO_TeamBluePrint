#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>

// ============================================================
// I2C MULTIPLEXER & ENUMS
// ============================================================
#define TCA_ADDR 0x70

enum BlockColor { COLOR_NONE, COLOR_ORANGE, COLOR_BLUE };

// ------------------------------------------------------------
// HOW THIS RUN WORKS
//
// At every corner of the mat there are TWO lines, one orange and
// one blue. They fan out from the inner square like spokes, so
// they are CLOSE together near the inner square and FAR apart
// near the outer wall.
//
// 1. Drive straight. The first line we ever see decides direction:
//      ORANGE first -> clockwise      -> turn RIGHT every corner
//      BLUE   first -> counterclockwise -> turn LEFT  every corner
//
// 2. Keep driving and cross the OTHER line of that same corner.
//    The distance between the two crossings tells us how far out
//    in the lane we are:
//      small gap = we have drifted IN toward the inner square
//      large gap = we are OUT near the outer wall
//
// 3. Turn 90 degrees, then save the heading we actually settled at.
//
// 4. Steer sideways for a fixed distance to undo the drift, then
//    turn back onto that saved heading. Now we are back in the
//    middle of the lane AND pointing the right way.
//
// 5. After 12 corners (3 laps), drive 100cm straight and stop.
//
// WHY THIS IS NEEDED: every 90 degree turn ends slightly imperfect,
// so the robot creeps toward the inner square a little more each
// lap. The heading PID does NOT fix this - it keeps the robot
// pointing the right way, but has no idea it is in the wrong PLACE.
// ------------------------------------------------------------

// ------------------------------------------------------------
// FSM STATES
//
// DRIVE_TO_CORNER  drive straight until both lines of a corner
//                  have been crossed and the gap measured
// TURNING          turn 90 degrees, then save the heading we
//                  actually settled at
// LANE_CORRECT     shuffle sideways to undo the drift, then
//                  realign to that saved heading
// FINAL_STRAIGHT   after 12 corners, drive 100cm and stop
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
const int RPWM_PIN   = PB9;   // Motor Forward
const int LPWM_PIN   = PB8;   // Motor Reverse
const int DRV_EN_PIN = PB1;   // Motor Enable
const int SERVO_PIN  = PA8;   // Steering Servo

const int IMU_CS_PIN  = PB0;
const int IMU_INT_PIN = PB13;
const int IMU_RST_PIN = PB14;

// Encoder and servo constants
const float TICKS_PER_CM        = 31.933; // ~958 ticks / 30 cm
const float SERVO_TRUE_STRAIGHT = 69.0;   // Center steering angle
const float SERVO_MAX_LEFT      = 5.0;    // Lower angle = LEFT
const float SERVO_MAX_RIGHT     = 115.0;  // Higher angle = RIGHT
const int   BASE_SPEED          = 120;    // Motor PWM cruising speed (1 to 255)

SPIClass SPI_IMU(PB5, PB4, PB3);
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

bool    tcsConnected    = false;
uint8_t tcsChannel      = 0;    // Multiplexer channel where TCS34725 is located
float   initialYawOffset = 0.0; // Stores the "Zero" angle

// ============================================================
// RUN CONSTANTS
// ============================================================
const int   TARGET_CORNERS   = 12;      // 3 laps x 4 corners
const int   FINAL_STRAIGHT_CM = 100;    // final straight after the last turn
const float SEARCH_SAFETY_CM = 400.0;   // never drive forever without a line

// ============================================================
// LANE CORRECTION TUNING
// Every one of these is meant to be changed on the field.
// ============================================================
const float GAP_THRESHOLD_CM       = 25.0;  // MUST calibrate on the real mat
const float GAP_DEADBAND_CM        =  2.0;  // ignore corrections smaller than this
const float K_LAT_DEG_PER_CM       =  1.5;  // servo degrees per cm of error
const float MAX_LAT_OFFSET_DEG     = 30.0;  // clamp (servo limit is 46 either way)
const float CORRECTION_DISTANCE_CM = 20.0;  // fixed travel during a correction
const float SECOND_LINE_MAX_CM     = 60.0;  // give up waiting for the partner line
const float POST_CORNER_LOCKOUT_CM = 30.0;  // ignore lines just after leaving a corner
const int   CORRECTION_SIGN        =   +1;  // flip to -1 if corrections go backwards
const int   CORRECTION_PWM         =   90;  // slower than BASE_SPEED, more control

// ============================================================
// FSM DATA
// ============================================================
RobotState currentState = STATE_INIT;

BlockColor lockedColor   = COLOR_NONE;  // colour that won the very first corner
bool       clockwiseMode = true;        // true = orange first = right turns
int        cornerCount   = 0;           // corners completed so far

float targetHeading = 0.0;   // heading held while driving a straight
float laneHeading   = 0.0;   // heading saved right after a 90 degree turn

// Results of the most recent corner. Plain globals, no structs.
float      lastGapCm      = 0.0;        // distance between the two lines
BlockColor lastFirstColor = COLOR_NONE; // which colour we crossed first

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
  }
  else if (speed < 0) {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, -speed);
  }
  else {
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
//
// NOTE: never write abs(readEncoder()). Arduino's abs() is a
// macro that evaluates its argument TWICE, so that would read the
// hardware timer twice and can give an inconsistent answer.
// Always store the reading in a long first, then negate by hand.
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

// Heading in the range -180..180. LEFT is increasing, RIGHT is decreasing.
float readHeading() {
  float currentYaw = readYaw();
  return fmod(currentYaw - initialYawOffset + 540.0, 360.0) - 180.0;
}

// ============================================================
// 4. COLOUR SENSOR
// ============================================================

void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (tcsConnected) {
    tcaselect(tcsChannel);   // make sure the mux is routed to our sensor
    tcs.getRawData(&r, &g, &b, &c);
  } else {
    r = 0; g = 0; b = 0; c = 0;
  }
}

// ------------------------------------------------------------
// COLOUR DETECTION
//
// The old version had a 2 second cooldown after every detection.
// That made it impossible to see the second line at a corner, so
// no gap could ever be measured.
//
// Instead we now "arm" the detector to one specific colour. It
// throws away everything else. Because we arm it to the OTHER
// colour when looking for the second line, it is impossible to
// re-trigger on the same physical line, so no cooldown is needed.
// ------------------------------------------------------------

BlockColor pendingColor    = COLOR_NONE;  // colour we are currently counting
int        consecutiveHits = 0;           // how many reads in a row we saw it

// Wipe the counters. Call before starting to look for a new line.
void resetColorDetector() {
  pendingColor    = COLOR_NONE;
  consecutiveHits = 0;
}

// Returns the partner colour of a corner pair.
BlockColor otherColor(BlockColor c) {
  if (c == COLOR_ORANGE) return COLOR_BLUE;
  if (c == COLOR_BLUE)   return COLOR_ORANGE;
  return COLOR_NONE;
}

// Read the sensor once.
//   wantColor == COLOR_NONE  -> accept either orange or blue
//   wantColor == COLOR_BLUE  -> accept ONLY blue, ignore orange completely
// Returns COLOR_NONE until the wanted colour is seen twice in a row.
BlockColor detectColor(BlockColor wantColor) {
  const int HITS_NEEDED = 2;   // 2 reads at 2.4ms integration = about 5ms

  uint16_t r, g, b, c;
  readColor(r, g, b, c);

  BlockColor rawColor  = COLOR_NONE;
  float      totalLight = r + g + b;

  if (totalLight > 0) {
    float pR = ((float)r / totalLight) * 100.0;
    float pB = ((float)b / totalLight) * 100.0;

    // Calibrated at 2.4ms integration / 16X gain:
    //   Blue object:   R% ~19, B% ~42
    //   Orange object: R% ~40, B% ~23
    //   Bare floor:    R% ~26-30, B% ~30
    if (pB > 36.0 && pR < 24.0) {
      rawColor = COLOR_BLUE;
    } else if (pR > 35.0 && pB < 27.0) {
      rawColor = COLOR_ORANGE;
    }
  }

  // Throw away any colour we are not currently armed for.
  if (wantColor != COLOR_NONE && rawColor != wantColor) {
    rawColor = COLOR_NONE;
  }

  // Nothing valid this read: the streak is broken.
  if (rawColor == COLOR_NONE) {
    resetColorDetector();
    return COLOR_NONE;
  }

  // Count how many reads in a row gave the same colour.
  if (rawColor == pendingColor) {
    consecutiveHits++;
  } else {
    pendingColor    = rawColor;
    consecutiveHits = 1;
  }

  // Enough in a row: report it and clear the counters.
  if (consecutiveHits >= HITS_NEEDED) {
    resetColorDetector();
    return rawColor;
  }

  return COLOR_NONE;
}

// ============================================================
// SYSTEM INITIALIZATION
// ============================================================
void initHardware() {
  // ---- Motors ----
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);
  digitalWrite(DRV_EN_PIN, HIGH);
  setMotorSpeed(0);

  // ---- Servo ----
  steeringServo.attach(SERVO_PIN, 1000, 2000);
  setServoAngle(SERVO_TRUE_STRAIGHT);

  // ---- Encoder (Timer 3) ----
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
  // static: this handle must outlive initHardware(). It used to be a
  // plain local, which went out of scope the moment this function
  // returned. It only worked because HAL_TIM_Encoder_Start() had
  // already written the peripheral registers directly.
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

  // ---- Wire setup for the TCA9548A multiplexer ----
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  delay(100);

  // ---- Find the TCS34725 across the mux channels ----
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

  // ---- IMU ----
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
// HEADING PID
//
// One step of "hold this compass heading". Call it over and over
// inside a driving loop. It only acts when the IMU has fresh data.
// This used to be copy-pasted into two different drive functions.
// ============================================================
float         pidPrevError = 0.0;
unsigned long pidPrevTime  = 0;

void resetHeadingPid() {
  pidPrevError = 0.0;
  pidPrevTime  = millis();
}

void updateHeadingPid(float heading) {
  const float localKp = 1.2;
  const float localKd = 0.3;

  if (myIMU.wasReset()) {
    myIMU.enableGameRotationVector();
  }

  if (myIMU.getSensorEvent() &&
      myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {

    unsigned long now = millis();
    float dt = (now - pidPrevTime) / 1000.0;
    if (dt <= 0.0) dt = 0.001;   // never divide by zero

    // How far off the wanted heading are we?
    float error = heading - readHeading();
    while (error > 180.0)  error -= 360.0;   // wrap to -180..180
    while (error < -180.0) error += 360.0;

    float dError     = (error - pidPrevError) / dt;
    float correction = (localKp * error) + (localKd * dError);

    // Lower servo angle steers LEFT, higher steers RIGHT.
    setServoAngle(SERVO_TRUE_STRAIGHT - correction);

    pidPrevError = error;
    pidPrevTime  = now;
  }
}

// ============================================================
// NAVIGATION FUNCTIONS
// ============================================================

// Turn until the IMU says we are pointing at targetGlobalHeading.
// The motor creeps forward while turning - this robot cannot turn
// on the spot.
void turnToHeading(float targetGlobalHeading) {
  const int   TURN_PWM       = 90;
  const float STOP_THRESHOLD = 4.0;

  while (targetGlobalHeading > 180.0)  targetGlobalHeading -= 360.0;
  while (targetGlobalHeading < -180.0) targetGlobalHeading += 360.0;

  float currentHeading = readHeading();
  float angleNeeded    = targetGlobalHeading - currentHeading;

  while (angleNeeded > 180.0)  angleNeeded -= 360.0;
  while (angleNeeded < -180.0) angleNeeded += 360.0;

  // Negative angle means we must turn RIGHT.
  if (angleNeeded < 0) {
    setServoAngle(SERVO_MAX_RIGHT);
  } else {
    setServoAngle(SERVO_MAX_LEFT);
  }

  delay(150);
  setMotorSpeed(TURN_PWM);

  while (true) {
    if (myIMU.wasReset()) {
      myIMU.enableGameRotationVector();
    }

    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      currentHeading = readHeading();

      float errorRemaining = targetGlobalHeading - currentHeading;
      while (errorRemaining > 180.0)  errorRemaining -= 360.0;
      while (errorRemaining < -180.0) errorRemaining += 360.0;

      if (angleNeeded < 0 && errorRemaining >= -STOP_THRESHOLD) break;
      if (angleNeeded > 0 && errorRemaining <=  STOP_THRESHOLD) break;
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(300);

  Serial.print(F(">>> Realigned. Heading now: "));
  Serial.println(readHeading());
}

// Drive a fixed distance in cm while holding a heading.
void goStraight(int cm, float heading) {
  zeroEncoder();
  resetHeadingPid();

  long targetTicks = (long)(cm * TICKS_PER_CM);

  setMotorSpeed(BASE_SPEED);

  while (true) {
    long ticksNow = readEncoder();          // read ONCE, never inside abs()
    if (ticksNow < 0) ticksNow = -ticksNow;
    if (ticksNow >= targetTicks) break;

    updateHeadingPid(heading);
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100);
}

// Turn by a relative amount. Positive = RIGHT, negative = LEFT.
void turnDegrees(float degree) {
  const int TURN_PWM = 90;

  float startHeading       = readHeading();
  float finalTargetHeading = startHeading - degree;   // right = decreasing heading

  while (finalTargetHeading > 180.0)  finalTargetHeading -= 360.0;
  while (finalTargetHeading < -180.0) finalTargetHeading += 360.0;

  if (degree > 0) {
    setServoAngle(SERVO_MAX_RIGHT);
  } else {
    setServoAngle(SERVO_MAX_LEFT);
  }

  delay(150);
  setMotorSpeed(TURN_PWM);

  while (true) {
    if (myIMU.wasReset()) {
      myIMU.enableGameRotationVector();
    }

    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      float currentHeading = readHeading();

      float errorRemaining = finalTargetHeading - currentHeading;
      while (errorRemaining > 180.0)  errorRemaining -= 360.0;
      while (errorRemaining < -180.0) errorRemaining += 360.0;

      if (degree > 0 && errorRemaining >= -2.0) break;
      if (degree < 0 && errorRemaining <=  2.0) break;
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100);

  Serial.print(F(">>> TURN COMPLETE. Resting heading: "));
  Serial.println(readHeading());
}

// ============================================================
// CORNER MEASUREMENT
//
// Drives straight until BOTH lines of a corner have been crossed,
// and records how far apart they were.
//
// Results come back in globals (kept simple, no structs):
//   lastGapCm      - distance between the two lines
//   lastFirstColor - which colour we crossed first
//
// Returns true if a corner was crossed, false if we drove the full
// safety distance without seeing anything.
// ============================================================
bool driveToCorner(float heading) {
  // On the very first corner nothing is locked yet, so lockedColor
  // is COLOR_NONE and we accept either colour. After that only the
  // locked colour is allowed to start a corner.
  BlockColor wantFirst = lockedColor;

  resetColorDetector();
  resetHeadingPid();
  zeroEncoder();
  setMotorSpeed(BASE_SPEED);

  long lockoutTicks = (long)(POST_CORNER_LOCKOUT_CM * TICKS_PER_CM);
  long safetyTicks  = (long)(SEARCH_SAFETY_CM * TICKS_PER_CM);

  // ---------- PHASE A: find the first line ----------
  BlockColor firstSeen  = COLOR_NONE;
  long       firstTicks = 0;

  while (true) {
    long ticksNow = readEncoder();          // read ONCE, never inside abs()
    if (ticksNow < 0) ticksNow = -ticksNow;
    if (ticksNow >= safetyTicks) break;     // safety cap, give up

    updateHeadingPid(heading);

    // Ignore lines until we are well clear of the corner we just left.
    if (ticksNow > lockoutTicks) {
      firstSeen = detectColor(wantFirst);
      if (firstSeen != COLOR_NONE) {
        firstTicks = readEncoder();
        break;
      }
    }
    delay(2);
  }

  if (firstSeen == COLOR_NONE) {
    setMotorSpeed(0);
    setServoAngle(SERVO_TRUE_STRAIGHT);
    return false;
  }

  // ---------- PHASE B: find the partner line ----------
  BlockColor wantSecond     = otherColor(firstSeen);
  long       maxSecondTicks = (long)(SECOND_LINE_MAX_CM * TICKS_PER_CM);
  bool       gotSecond      = false;

  resetColorDetector();

  while (true) {
    long travelled = readEncoder() - firstTicks;   // read ONCE
    if (travelled < 0) travelled = -travelled;
    if (travelled >= maxSecondTicks) break;        // partner line missed

    updateHeadingPid(heading);

    if (detectColor(wantSecond) != COLOR_NONE) {
      lastGapCm = (float)travelled / TICKS_PER_CM;
      gotSecond = true;
      break;
    }
    delay(2);
  }

  // If the partner line was missed, pretend the gap was exactly on
  // target. That makes the correction zero, and we still turn.
  // Never stall here - we have already passed the corner marker,
  // so not turning means driving into the wall.
  if (!gotSecond) {
    lastGapCm = GAP_THRESHOLD_CM;
    Serial.println(F("  WARN: partner line missed, skipping correction"));
  }

  lastFirstColor = firstSeen;

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);

  Serial.print(F("CORNER  first="));
  if (firstSeen == COLOR_ORANGE) Serial.print(F("ORANGE"));
  else                           Serial.print(F("BLUE"));
  Serial.print(F("  gap="));
  Serial.print(lastGapCm);
  Serial.println(F(" cm"));

  return true;
}

// ============================================================
// LANE CORRECTION
//
// Called right after a 90 degree turn. laneHeading is the heading
// the robot settled at after that turn - the straight line we want
// to end up parallel to.
//
// The robot steers off to one side for a fixed distance, then uses
// the IMU to turn back onto laneHeading. Net result: it has moved
// sideways across the lane but is pointing the same way as before.
//
// Which way to steer depends on which way round the mat we are
// going, because the inner square is on a different side each way:
//   CLOCKWISE         inner square on the RIGHT
//   COUNTERCLOCKWISE  inner square on the LEFT
// A single fixed direction would fix the drift one way round the
// mat and drive straight into the wall the other way.
// ============================================================
void doLaneCorrection(float laneHeadingRef) {
  float delta = lastGapCm - GAP_THRESHOLD_CM;

  // Size of the error, always positive.
  float mag = delta;
  if (mag < 0) mag = -mag;

  if (mag < GAP_DEADBAND_CM) {
    Serial.println(F("  correction skipped (inside deadband)"));
    return;
  }

  // Positive result means steer LEFT, negative means steer RIGHT.
  float steerSigned = CORRECTION_SIGN * delta;
  if (clockwiseMode) steerSigned = -steerSigned;

  // Bigger error = harder steering. Travel distance stays fixed.
  float offset = K_LAT_DEG_PER_CM * mag;
  if (offset > MAX_LAT_OFFSET_DEG) offset = MAX_LAT_OFFSET_DEG;

  float servo;
  if (steerSigned > 0) {
    servo = SERVO_TRUE_STRAIGHT - offset;   // lower angle = LEFT
  } else {
    servo = SERVO_TRUE_STRAIGHT + offset;   // higher angle = RIGHT
  }

  Serial.print(F("  delta="));
  Serial.print(delta);
  Serial.print(F("  steer "));
  if (steerSigned > 0) Serial.print(F("LEFT "));
  else                 Serial.print(F("RIGHT "));
  Serial.print(offset);
  Serial.println(F(" deg"));

  // ---- open loop: hold that steering for a fixed distance ----
  zeroEncoder();
  long targetTicks = (long)(CORRECTION_DISTANCE_CM * TICKS_PER_CM);

  setServoAngle(servo);
  delay(150);                      // let the servo actually get there
  setMotorSpeed(CORRECTION_PWM);

  while (true) {
    long ticksNow = readEncoder();  // read ONCE, never inside abs()
    if (ticksNow < 0) ticksNow = -ticksNow;
    if (ticksNow >= targetTicks) break;
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100);

  // ---- closed loop: turn back onto the heading we saved ----
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
      targetHeading = readHeading();   // about 0, zeroYaw already ran
      currentState  = STATE_DRIVE_TO_CORNER;
      break;
    }

    case STATE_DRIVE_TO_CORNER: {
      Serial.println(F("[FSM] DRIVE_TO_CORNER"));

      if (driveToCorner(targetHeading)) {
        // The first corner of the run decides which way round we go.
        if (lockedColor == COLOR_NONE) {
          lockedColor   = lastFirstColor;
          clockwiseMode = (lockedColor == COLOR_ORANGE);
          if (clockwiseMode) {
            Serial.println(F("[FSM] LOCKED ORANGE -> CLOCKWISE (right turns)"));
          } else {
            Serial.println(F("[FSM] LOCKED BLUE -> COUNTERCLOCKWISE (left turns)"));
          }
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

      turnDegrees(clockwiseMode ? 90.0 : -90.0);

      // Save where we actually ended up, not where we aimed.
      laneHeading = readHeading();

      if (cornerCount >= TARGET_CORNERS) {
        currentState = STATE_FINAL_STRAIGHT;
      } else {
        currentState = STATE_LANE_CORRECT;
      }
      break;
    }

    case STATE_LANE_CORRECT: {
      doLaneCorrection(laneHeading);
      targetHeading = readHeading();   // trust where we actually are
      currentState  = STATE_DRIVE_TO_CORNER;
      break;
    }

    case STATE_FINAL_STRAIGHT: {
      Serial.print(F("[FSM] FINAL_STRAIGHT "));
      Serial.print(FINAL_STRAIGHT_CM);
      Serial.println(F("cm"));

      goStraight(FINAL_STRAIGHT_CM, laneHeading);
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
