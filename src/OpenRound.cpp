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
// FSM STATES
//
// Flow implemented:
//   1. Robot drives straight, watching for the FIRST color hit
//      (either orange or blue) -> STATE_SEARCH_FIRST_COLOR
//   2. Whichever color is seen FIRST "locks" the run:
//        - ORANGE first  -> clockwise mode   -> turn RIGHT 90 deg each hit
//        - BLUE first    -> counterclockwise -> turn LEFT  90 deg each hit
//      Once locked, the other color is ignored completely (filtered
//      out at the sensor level, not just at the FSM level).
//   3. Every time the locked color is detected again, the robot turns
//      90 degrees in the locked direction (STATE_TURNING), then goes
//      back to driving straight looking for the next hit
//      (STATE_DRIVE_TO_BLOCK).
//   4. After the locked color has been detected 12 times total, the
//      robot drives straight 100cm and stops (STATE_FINAL_STRAIGHT ->
//      STATE_FINISHED).
// ------------------------------------------------------------
enum RobotState {
  STATE_INIT,
  STATE_SEARCH_FIRST_COLOR,   // driving straight, first hit of either color locks the mode
  STATE_TURNING,              // executes one locked-direction 90 deg turn
  STATE_DRIVE_TO_BLOCK,       // driving straight, waiting for next locked-color hit
  STATE_FINAL_STRAIGHT,       // all 12 turns done -> final 100cm run
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
const int   BASE_SPEED          = 120;    // Motor PWM cruising speed (1 to 255)

// PID Control Parameters
float Kp = 0.7; // Steering responsiveness
float Kd = 0.5; // Steering damping

SPIClass SPI_IMU(PB5, PB4, PB3);
Servo steeringServo;
BNO08x myIMU;
// Replace the old initialization with this one:
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);

bool tcsConnected = false;
uint8_t tcsChannel = 0; // Multiplexer channel where TCS34725 is located
float initialYawOffset = 0.0; // Stores the "Zero" angle

// ============================================================
// FSM DATA
// ============================================================
RobotState currentState = STATE_INIT;

BlockColor lockedColor       = COLOR_NONE; // the color that "won" the first detection
bool       clockwiseMode     = true;       // true = orange locked (turn right), false = blue locked (turn left)
int        detectionCount    = 0;          // how many times the locked color has been seen
const int  TARGET_DETECTIONS = 12;         // stop turning after this many hits
const int  FINAL_STRAIGHT_CM = 100;        // final straight-line distance after last turn
const float SEARCH_SAFETY_CM = 400.0;      // safety cap so the robot never drives forever w/o a hit

float targetHeading = 0.0; // running absolute heading target the drive loop tries to hold

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
    analogWrite(LPWM_PIN, abs(speed));
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
  float qI = myIMU.getQuatI();
  float qJ = myIMU.getQuatJ();
  float qK = myIMU.getQuatK();
  float qReal = myIMU.getQuatReal();

  if (qI == 0.0f && qJ == 0.0f && qK == 0.0f && qReal == 0.0f) {
    return 0.0f;
  }

  float yawRadians = atan2(2.0f * (qI * qJ + qReal * qK),
                           (qReal * qReal + qI * qI - qJ * qJ - qK * qK));
  return yawRadians * (180.0 / PI);
}

void zeroYaw() {
  Serial.println("Waiting for valid IMU data to set Zero...");

  unsigned long timeout = millis();
  while (millis() - timeout < 3000) {
    if (myIMU.wasReset()) {
      myIMU.enableGameRotationVector();
    }

    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      initialYawOffset = readYaw();
      Serial.print("Zero Yaw successfully locked at: ");
      Serial.println(initialYawOffset);
      return;
    }
    delay(10);
  }
  Serial.println("ERROR: Timed out waiting for IMU event!");
}

float readHeading() {
  float currentYaw = readYaw();
  return fmod(currentYaw - initialYawOffset + 540.0, 360.0) - 180.0;
}

// ============================================================
// 4. COLOR SENSOR FUNCTIONS
// ============================================================

void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (tcsConnected) {
    tcaselect(tcsChannel); // Ensure multiplexer route is active
    tcs.getRawData(&r, &g, &b, &c);
  } else {
    r = 0; g = 0; b = 0; c = 0;
  }
}

/*
 * detectColorDebounced()
 * Calibrated against tested Blue, Orange, and Floor raw profiles.
 *
 * filterColor: if not COLOR_NONE, any raw reading that isn't this
 * exact color is treated as "nothing detected" -- it is discarded
 * BEFORE it ever touches the debounce state machine. This is what
 * makes the locked color truly ignore the other color: a stray
 * blue reading can't reset/steal the orange debounce counters
 * (and vice versa) once a color has been locked in.
 */
BlockColor detectColorDebounced(BlockColor filterColor = COLOR_NONE) {
  static BlockColor pendingColor = COLOR_NONE;
  static int consecutiveHits = 0;
  static unsigned long lastDetectionTime = 0;

  // Reduced to 2 hits for high-speed detection (~5ms total)
  const int HITS_NEEDED = 2;              
  const unsigned long COOLDOWN_MS = 2000; 

  if (millis() - lastDetectionTime < COOLDOWN_MS) {
    return COLOR_NONE;
  }

  uint16_t r, g, b, c;
  readColor(r, g, b, c);

  BlockColor rawColor = COLOR_NONE;
  float totalLight = r + g + b;

  if (totalLight > 0) {
    float pR = ((float)r / totalLight) * 100.0;
    float pB = ((float)b / totalLight) * 100.0;

    // ============================================================
    // CALIBRATED COLOR THRESHOLDS (2.4ms / 16X Gain)
    // Blue Object:  R% ~19%, B% ~42%
    // Orange Object: R% ~40%, B% ~23%
    // Floor (Ref):   R% ~26-30%, B% ~30%
    // ============================================================

    // Blue Check
    if (pB > 36.0 && pR < 24.0) {
      rawColor = COLOR_BLUE;
    }
    // Orange Check
    else if (pR > 35.0 && pB < 27.0) {
      rawColor = COLOR_ORANGE;
    }
  }

  // Reject anything that isn't the color we currently care about.
  if (filterColor != COLOR_NONE && rawColor != COLOR_NONE && rawColor != filterColor) {
    rawColor = COLOR_NONE;
  }

  // Debounce Filtering
  if (rawColor != COLOR_NONE) {
    if (rawColor == pendingColor) {
      consecutiveHits++;
    } else {
      pendingColor = rawColor;
      consecutiveHits = 1;
    }

    if (consecutiveHits >= HITS_NEEDED) {
      lastDetectionTime = millis();
      consecutiveHits = 0;
      pendingColor = COLOR_NONE;
      return rawColor;
    }
  }
  else {
    consecutiveHits = 0;
    pendingColor = COLOR_NONE;
  }

  return COLOR_NONE;
}

// ============================================================
// SYSTEM INITIALIZATION
// ============================================================
void initHardware() {
  // Motors
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);
  digitalWrite(DRV_EN_PIN, HIGH);
  setMotorSpeed(0);

  // Servo
  steeringServo.attach(SERVO_PIN, 1000, 2000);
  setServoAngle(69.0);

  // Encoder (Timer 3)
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_HandleTypeDef htim3 = {0};
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // Wire Setup for TCA9548A Multiplexer
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  delay(100);

  // Locate TCS34725 across Multiplexer Channels
  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);
    delay(10);
    if (tcs.begin()) {
      tcsConnected = true;
      tcsChannel = i;
      Serial.print("TCS34725 connected on TCA channel ");
      Serial.println(i);
      break;
    }
  }

  if (!tcsConnected) {
    Serial.println("WARNING: TCS34725 not detected through TCA9548A!");
  }

  // IMU
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
// NAVIGATION FUNCTIONS
// ============================================================

void turnToHeading(float targetGlobalHeading) {
  const float SERVO_TRUE_STRAIGHT = 69.0;
  const float SERVO_MAX_RIGHT     = 115.0;
  const float SERVO_MAX_LEFT      = 5.0;

  const int   TURN_PWM            = 90;
  const float STOP_THRESHOLD      = 4.0;

  while (targetGlobalHeading > 180.0)  targetGlobalHeading -= 360.0;
  while (targetGlobalHeading < -180.0) targetGlobalHeading += 360.0;

  float currentHeading = readHeading();
  float angleNeeded = targetGlobalHeading - currentHeading;

  while (angleNeeded > 180.0)  angleNeeded -= 360.0;
  while (angleNeeded < -180.0) angleNeeded += 360.0;

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

      if (angleNeeded < 0 && errorRemaining >= -STOP_THRESHOLD) {
        break;
      }
      else if (angleNeeded > 0 && errorRemaining <= STOP_THRESHOLD) {
        break;
      }
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);

  delay(300);
  Serial.print(">>> TURN COMPLETE. Final Resting Heading: ");
  Serial.println(readHeading());
}

void goStraight(int cm, float heading) {
  const float TICKS_PER_CM        = 31.933;
  const float SERVO_TRUE_STRAIGHT = 69.0;
  const int   BASE_SPEED          = 120;

  float localKp = 1.2;
  float localKd = 0.3;

  zeroEncoder();
  long targetTicks = (long)(cm * TICKS_PER_CM);

  float prevError = 0.0;
  unsigned long prevTime = millis();

  setMotorSpeed(BASE_SPEED);

  while (abs(readEncoder()) < targetTicks) {
    if (myIMU.wasReset()) {
      myIMU.enableGameRotationVector();
    }

    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      unsigned long currentTime = millis();
      float dt = (currentTime - prevTime) / 1000.0;
      if (dt <= 0.0) dt = 0.001;

      float currentHeading = readHeading();
      float error = heading - currentHeading;

      while (error > 180.0)  error -= 360.0;
      while (error < -180.0) error += 360.0;

      float dError = (error - prevError) / dt;
      float correction = (localKp * error) + (localKd * dError);

      float newServoAngle = SERVO_TRUE_STRAIGHT - correction;
      setServoAngle(newServoAngle);

      prevError = error;
      prevTime = currentTime;
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100);
}

/*
 * driveUntilColorDetected()
 *
 * Drives straight, holding `heading`, while continuously watching the
 * color sensor (filtered by `filterColor` -- pass COLOR_NONE during the
 * initial search so either color can win, or pass the locked color
 * afterwards so the other color is completely ignored).
 *
 * Returns the color that was detected, or COLOR_NONE if the safety
 * distance cap (`maxCm`) was reached without any hit (fail-safe so the
 * robot never drives forever if a block is missed / sensor glitches).
 */
BlockColor driveUntilColorDetected(float heading, BlockColor filterColor, float maxCm) {
  const float localKp = 1.2;
  const float localKd = 0.3;

  zeroEncoder();
  long safetyTicks = (long)(maxCm * TICKS_PER_CM);

  float prevError = 0.0;
  unsigned long prevTime = millis();

  setMotorSpeed(BASE_SPEED);

  while (abs(readEncoder()) < safetyTicks) {
    if (myIMU.wasReset()) {
      myIMU.enableGameRotationVector();
    }

    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      unsigned long currentTime = millis();
      float dt = (currentTime - prevTime) / 1000.0;
      if (dt <= 0.0) dt = 0.001;

      float currentHeading = readHeading();
      float error = heading - currentHeading;

      while (error > 180.0)  error -= 360.0;
      while (error < -180.0) error += 360.0;

      float dError = (error - prevError) / dt;
      float correction = (localKp * error) + (localKd * dError);

      float newServoAngle = SERVO_TRUE_STRAIGHT - correction;
      setServoAngle(newServoAngle);

      prevError = error;
      prevTime = currentTime;
    }

    BlockColor detected = detectColorDebounced(filterColor);
    if (detected != COLOR_NONE) {
      setMotorSpeed(0);
      setServoAngle(SERVO_TRUE_STRAIGHT);
      return detected;
    }

    delay(2);
  }

  // Safety cap reached without a detection.
  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  return COLOR_NONE;
}

void turnDegrees(float degree) {
  const float SERVO_TRUE_STRAIGHT = 69.0;
  const float SERVO_MAX_RIGHT     = 115.0;
  const float SERVO_MAX_LEFT      = 5.0;
  const int   TURN_PWM            = 90;

  float startHeading = readHeading();
  float finalTargetHeading = startHeading - degree;

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

      if (degree > 0 && errorRemaining >= -2.0) {
        break;
      }
      else if (degree < 0 && errorRemaining <= 2.0) {
        break;
      }
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100);
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
  // Finite State Machine
  switch (currentState) {

    case STATE_INIT: {
      Serial.println(F("[FSM] STATE: INIT"));
      lockedColor = COLOR_NONE;
      detectionCount = 0;
      targetHeading = readHeading(); // ~0.0, since zeroYaw() already ran in initHardware()
      currentState = STATE_SEARCH_FIRST_COLOR;
      break;
    }

    case STATE_SEARCH_FIRST_COLOR: {
      Serial.println(F("[FSM] STATE: SEARCH_FIRST_COLOR -> driving, watching for either color..."));

      // No filter yet -- either orange or blue can win the lock.
      BlockColor found = driveUntilColorDetected(targetHeading, COLOR_NONE, SEARCH_SAFETY_CM);

      if (found == COLOR_ORANGE) {
        lockedColor = COLOR_ORANGE;
        clockwiseMode = true; // orange first -> clockwise mode -> turn RIGHT
        Serial.println(F("[FSM] LOCKED ON ORANGE -> Clockwise mode (right turns). Blue is now ignored."));
        currentState = STATE_TURNING;
      }
      else if (found == COLOR_BLUE) {
        lockedColor = COLOR_BLUE;
        clockwiseMode = false; // blue first -> counterclockwise mode -> turn LEFT
        Serial.println(F("[FSM] LOCKED ON BLUE -> Counterclockwise mode (left turns). Orange is now ignored."));
        currentState = STATE_TURNING;
      }
      else {
        // Safety cap reached with no detection at all -- retry the search.
        Serial.println(F("[FSM] WARNING: no color found within safety distance. Retrying search."));
      }
      break;
    }

    case STATE_TURNING: {
      detectionCount++;
      Serial.print(F("[FSM] STATE: TURNING -> Detection #"));
      Serial.print(detectionCount);
      Serial.print(F(" / "));
      Serial.println(TARGET_DETECTIONS);

      float turnAmount = clockwiseMode ? 90.0 : -90.0; // +90 = right, -90 = left
      turnDegrees(turnAmount);

      // Trust the settled, actually-achieved heading rather than
      // re-deriving it, so small turn overshoot/undershoot doesn't
      // accumulate error in the straight-line PID target.
      targetHeading = readHeading();

      if (detectionCount >= TARGET_DETECTIONS) {
        Serial.println(F("[FSM] Target detection count reached."));
        currentState = STATE_FINAL_STRAIGHT;
      } else {
        currentState = STATE_DRIVE_TO_BLOCK;
      }
      break;
    }

    case STATE_DRIVE_TO_BLOCK: {
      Serial.println(F("[FSM] STATE: DRIVE_TO_BLOCK -> driving, watching only for locked color..."));

      // Filtered by lockedColor -- the other color is fully ignored
      // at the sensor level and can't interfere with debounce state.
      BlockColor found = driveUntilColorDetected(targetHeading, lockedColor, SEARCH_SAFETY_CM);

      if (found == lockedColor) {
        currentState = STATE_TURNING;
      } else {
        // Safety cap reached without seeing the locked color again.
        Serial.println(F("[FSM] WARNING: safety cap reached without a new detection. Retrying."));
      }
      break;
    }

    case STATE_FINAL_STRAIGHT: {
      Serial.print(F("[FSM] STATE: FINAL_STRAIGHT -> driving "));
      Serial.print(FINAL_STRAIGHT_CM);
      Serial.println(F("cm and stopping."));

      goStraight(FINAL_STRAIGHT_CM, targetHeading);
      currentState = STATE_FINISHED;
      break;
    }

    case STATE_FINISHED: {
      Serial.println(F("[FSM] STATE: FINISHED -> Run complete!"));
      setMotorSpeed(0);
      setServoAngle(SERVO_TRUE_STRAIGHT);
      while (true) { delay(1000); }
      break;
    }
  }
}
