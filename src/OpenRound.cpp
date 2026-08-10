
#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>

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
//encoder and servo 
 const float TICKS_PER_CM        = 31.933; // ~958 ticks / 30 cm
  const float SERVO_TRUE_STRAIGHT = 69.0;   // Center steering angle
  const int   BASE_SPEED          = 120;    // Motor PWM cruising speed (1 to 255)
  // 1. Constants & Tuning Parameters
  float Kp = .7; // Steering responsiveness (increase if it's slow to correct)
  float Kd = 0.5; // Steering damping (increase if it wobbles/oscillates)

SPIClass SPI_IMU(PB5, PB4, PB3);
Servo steeringServo;
BNO08x myIMU;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);

bool tcsConnected = false;
float initialYawOffset = 0.0; // Stores the "Zero" angle

// ============================================================
// 1. MOTOR & STEERING FUNCTIONS
// ============================================================

/*
 * setMotorSpeed(int speed)
 * Controls both direction and speed of the main drive motor.
 * - Positive values (1 to 255): Drive Forward
 * - Negative values (-1 to -255): Drive Backward
 * - Zero (0): Coast/Stop
 */
void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255); // Prevent invalid PWM values
  
  if (speed > 0) {
    // Forward
    analogWrite(RPWM_PIN, speed);
    analogWrite(LPWM_PIN, 0);
  } 
  else if (speed < 0) {
    // Backward
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, abs(speed));
  } 
  else {
    // Stop
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
  }
}

/*
 * setServoAngle(float angleDeg)
 * Steers the front wheels. 
 * - 69.0 is your calibrated true straight.
 * - Higher values turn Right, lower values turn Left.
 */
void setServoAngle(float angleDeg) {
  // Clamp to prevent physical damage to the steering rack
  angleDeg = constrain(angleDeg, 5.0, 115.0); 
  
  // Map angle (0-180) to servo microsecond pulses (1000-2000)
  int pulseWidth = (int)((angleDeg / 180.0) * 1000.0) + 1000;
  steeringServo.writeMicroseconds(pulseWidth);
}

// ============================================================
// 2. ENCODER FUNCTIONS
// ============================================================

/*
 * zeroEncoder()
 * Resets the wheel encoder distance counter back to 0. 
 * Call this right before starting a new movement segment.
 */
void zeroEncoder() {
  TIM3->CNT = 0;
}

/*
 * readEncoder()
 * Returns the current tick count from the encoder.
 * - Moving forward increases the count.
 * - Moving backward decreases the count (can go negative).
 */
long readEncoder() {
  return (int16_t)TIM3->CNT;
}

// ============================================================
// 3. IMU (YAW & HEADING) FUNCTIONS
// ============================================================

/*
 * readYaw()
 * Gets the raw, absolute Yaw angle directly from the IMU.
 * (Usually you will use readHeading() instead of this).
 */
float readYaw() {
  float qI = myIMU.getQuatI();
  float qJ = myIMU.getQuatJ();
  float qK = myIMU.getQuatK();
  float qReal = myIMU.getQuatReal();

  // If quaternions haven't received valid data yet, return 0
  if (qI == 0.0f && qJ == 0.0f && qK == 0.0f && qReal == 0.0f) {
    return 0.0f;
  }

  float yawRadians = atan2(2.0f * (qI * qJ + qReal * qK),
                           (qReal * qReal + qI * qI - qJ * qJ - qK * qK));
  return yawRadians * (180.0 / PI);
}
/*
 * zeroYaw()
 * Takes the current orientation of the car and sets it as the new 0.0 degree line.
 * Call this once when the car is placed on the track.
 */
void zeroYaw() {
  Serial.println("Waiting for valid IMU data to set Zero...");
  
  // Wait up to 3 seconds for the first valid Game Rotation report
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

/*
 * readHeading()
 * Returns the car's current angle relative to your "Zero" line.
 * - Returns a value between -180.0 and +180.0.
 * - 0.0 means the car is perfectly straight.
 * - Positive means it drifted right, Negative means it drifted left.
 */
float readHeading() {
  float currentYaw = readYaw();
  // Math to find shortest difference between current angle and zero point
  return fmod(currentYaw - initialYawOffset + 540.0, 360.0) - 180.0;
}

// ============================================================
// 4. COLOR SENSOR FUNCTION
// ============================================================

/*
 * readColor(r, g, b, c)
 * Reads Red, Green, Blue, and Clear (brightness) values.
 * Usage: 
 *   uint16_t r, g, b, c;
 *   readColor(r, g, b, c);
 */
void readColor(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &c) {
  if (tcsConnected) {
    tcs.getRawData(&r, &g, &b, &c);
  } else {
    r = 0; g = 0; b = 0; c = 0; // Return zeroes if unplugged
  }
}


// ============================================================
// SYSTEM INITIALIZATION (Don't change this block)
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
  setServoAngle(69.0); // Center steering

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

  // Color Sensor
  if (tcs.begin()) {
    tcsConnected = true;
  }

  // IMU
  SPI_IMU.begin();
  if (myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU)) {
    delay(500);
    myIMU.enableGameRotationVector();
    delay(100);
    
    // Auto-zero the IMU on boot
    myIMU.getSensorEvent();
    zeroYaw();
  }
}
// ============================================================
// 1. TURN TO ABSOLUTE GLOBAL HEADING
// ============================================================
void turnToHeading(float targetGlobalHeading) {
  // 1. Steering & Speed Constants
  const float SERVO_TRUE_STRAIGHT = 69.0;
  const float SERVO_MAX_RIGHT     = 115.0; 
  const float SERVO_MAX_LEFT      = 5.0;  
  
  // ============================================================
  // TUNING PARAMETERS
  // ============================================================
  const int   TURN_PWM            = 90;   // Turning power
  const float STOP_THRESHOLD      = 4.0;  // Cut power early (in degrees) to account for coasting

  // Wrap target angle within -180.0 to +180.0
  while (targetGlobalHeading > 180.0)  targetGlobalHeading -= 360.0;
  while (targetGlobalHeading < -180.0) targetGlobalHeading += 360.0;

  float currentHeading = readHeading();
  float angleNeeded = targetGlobalHeading - currentHeading;

  // Wrap angleNeeded
  while (angleNeeded > 180.0)  angleNeeded -= 360.0;
  while (angleNeeded < -180.0) angleNeeded += 360.0;

  // Steer towards target: Negative difference = Turn Right, Positive = Turn Left
  if (angleNeeded < 0) {
    setServoAngle(SERVO_MAX_RIGHT); // Steer Right
  } else {
    setServoAngle(SERVO_MAX_LEFT);  // Steer Left
  }

  delay(150); // Servo response time
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

      // Telemetry output to track turning accuracy
      Serial.print("Target: "); Serial.print(targetGlobalHeading);
      Serial.print(" | Current: "); Serial.print(currentHeading);
      Serial.print(" | Error: "); Serial.println(errorRemaining);

      // Early Cutoff Trigger:
      // Right turn (angleNeeded < 0): errorRemaining approaches 0 from the negative side
      if (angleNeeded < 0 && errorRemaining >= -STOP_THRESHOLD) {
        break; 
      } 
      // Left turn (angleNeeded > 0): errorRemaining approaches 0 from the positive side
      else if (angleNeeded > 0 && errorRemaining <= STOP_THRESHOLD) {
        break; 
      }
    }
    delay(2);
  }

  // Cut motor power and straighten front wheels
  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  
  // Pause to let chassis settle and print final resting heading
  delay(300);
  Serial.print(">>> TURN COMPLETE. Final Resting Heading: ");
  Serial.println(readHeading());
}

// ============================================================
// 2. GO STRAIGHT ON EXPLICIT GLOBAL HEADING
// ============================================================
void goStraight(int cm, float targetHeading) {
  const float TICKS_PER_CM        = 31.933;
  const float SERVO_TRUE_STRAIGHT = 69.0;
  const int   BASE_SPEED          = 120; // Cruising speed
  
  // ============================================================
  // TUNING PARAMETERS
  // ============================================================
  float Kp = 1.2; // Start here (adjust in Step 2)
  float Kd = 0.3; // Start here (adjust in Step 3)

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
      float error = targetHeading - currentHeading;

      // Wrap error between -180 and +180
      while (error > 180.0)  error -= 360.0;
      while (error < -180.0) error += 360.0;

      float dError = (error - prevError) / dt;
      float correction = (Kp * error) + (Kd * dError);

      // SUBTRACT correction to ensure proper steering direction
      float newServoAngle = SERVO_TRUE_STRAIGHT - correction;
      setServoAngle(newServoAngle);

      // Debug output (Open Serial Plotter at 115200 baud to visualize)
      Serial.print("Heading:"); Serial.print(currentHeading);
      Serial.print(",Target:"); Serial.print(targetHeading);
      Serial.print(",Error:"); Serial.print(error);
      Serial.print(",Servo:"); Serial.println(newServoAngle);

      prevError = error;
      prevTime = currentTime;
    }
    delay(2);
  }

  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100);
}
void turnDegrees(float degree) {
  // 1. Steering & Speed Constants
  const float SERVO_TRUE_STRAIGHT = 69.0;
  const float SERVO_MAX_RIGHT     = 115.0; 
  const float SERVO_MAX_LEFT      = 5.0;  
  const int   TURN_PWM            = 90;

  // 2. Calculate target heading
  float startHeading = readHeading();
  
  // Turning Right (+) decreases IMU angle (-), turning Left (-) increases IMU angle (+)
  float targetHeading = startHeading - degree;

  // Wrap targetHeading within -180.0 to +180.0 range
  while (targetHeading > 180.0)  targetHeading -= 360.0;
  while (targetHeading < -180.0) targetHeading += 360.0;

  // 3. Set wheel direction (+ is Right, - is Left)
  if (degree > 0) {
    setServoAngle(SERVO_MAX_RIGHT); // Steer Right
  } else {
    setServoAngle(SERVO_MAX_LEFT);  // Steer Left
  }

  // FIX 1: Give physical servo time to move wheels BEFORE motor starts
  delay(150);

  // 4. Run motor at fixed turning speed
  setMotorSpeed(TURN_PWM);

  // 5. Turn execution loop
  while (true) {
    if (myIMU.wasReset()) {
      myIMU.enableGameRotationVector();
    }

    // FIX 2: Only process when a fresh Game Rotation report arrives
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      
      float currentHeading = readHeading();

      // Calculate remaining signed angle error
      float errorRemaining = targetHeading - currentHeading;
      while (errorRemaining > 180.0)  errorRemaining -= 360.0;
      while (errorRemaining < -180.0) errorRemaining += 360.0;

      // FIX 3: Direction-aware stop condition (prevents infinite spin on overshoot)
      if (degree > 0 && errorRemaining >= -2.0) {
        break; // Right turn reached target (or slightly overshot)
      } 
      else if (degree < 0 && errorRemaining <= 2.0) {
        break; // Left turn reached target (or slightly overshot)
      }
    }
    
    delay(2); // Short yield delay for CPU stability
  }

  // 6. Cut motor & reset steering straight
  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  delay(100); // Brief pause to let chassis settle
}

// ============================================================
// YOUR LOGIC GOES HERE
// ============================================================
// ============================================================
// FSM STATES & SQUARE TRAJECTORY DATA
// ============================================================
enum RobotState {
  STATE_INIT,
  STATE_DRIVE,
  STATE_TURN,
  STATE_SETTLE,
  STATE_FINISHED
};

RobotState currentState = STATE_INIT;

// The 4 global absolute headings for a clockwise square:
const float SQUARE_HEADINGS[4] = {0.0, -90.0, 180.0, 90.0};
int currentSide = 0; // Tracks which of the 4 sides we are on

void setup() {
  Serial.begin(115200);
  initHardware();
  delay(2000); 
}

void loop() {
  // 1. MUST KEEP THIS ALIVE in loop() for fresh IMU data
  if (myIMU.wasReset()) {
    myIMU.enableGameRotationVector();
  }
  myIMU.getSensorEvent();

  // 2. FSM LOGIC
  switch (currentState) {

    case STATE_INIT:
      Serial.println(F("[FSM] STATE: INIT -> Zeroing Yaw to Global 0.0"));
      zeroYaw(); // Set current physical position as 0.0 degrees
      delay(1000); // Give you a second to step back
      currentState = STATE_DRIVE;
      break;

    case STATE_DRIVE:
      Serial.print(F("[FSM] STATE: DRIVE -> Side "));
      Serial.print(currentSide + 1);
      Serial.print(F(" at Heading "));
      Serial.println(SQUARE_HEADINGS[currentSide]);
      
      // Drive 100cm locked to the current side's global heading
      goStraight(100, SQUARE_HEADINGS[currentSide]);
      
      currentState = STATE_TURN;
      break;

    case STATE_TURN:
      currentSide++; // Move to the next side

      // Did we just finish the 4th side?
      if (currentSide >= 4) {
        Serial.println(F("[FSM] Returning to start orientation..."));
        turnToHeading(0.0); // Re-align to original forward direction
        currentState = STATE_FINISHED;
      } 
      else {
        Serial.print(F("[FSM] STATE: TURN -> Heading to "));
        Serial.println(SQUARE_HEADINGS[currentSide]);
        
        // Turn to the next absolute heading
        turnToHeading(SQUARE_HEADINGS[currentSide]);
        currentState = STATE_SETTLE;
      }
      break;

    case STATE_SETTLE:
      // A brief pause ensures chassis momentum completely stops 
      // before the forward PID loop violently takes over again
      delay(500);
      currentState = STATE_DRIVE;
      break;

    case STATE_FINISHED:
      Serial.println(F("[FSM] STATE: FINISHED -> Square Complete!"));
      setMotorSpeed(0);
      setServoAngle(69.0);
      // Wait forever
      while(true) { delay(1000); }
      break;
  }
}
