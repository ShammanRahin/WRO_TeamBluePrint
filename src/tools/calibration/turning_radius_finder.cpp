#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// ============================================================
// CALIBRATION TOOL: ENCODER & TURNING RADIUS
// Uses exact mechanical limits: Min = 20.0, Max = 130.0, Straight = 76.5
// ============================================================

// --- Hardware Pins ---
const int MOT_RPWM_PIN = PA2;
const int MOT_LPWM_PIN = PA3;
const int SERVO_PIN    = PA8;

const int IMU_CS_PIN  = PA4;
const int IMU_INT_PIN = PB0;
const int IMU_RST_PIN = PB1;

// --- PWM Pulse Width Constraints ---
const int MIN_PULSE_US = 500;
const int MAX_PULSE_US = 2500;

// --- Physical Limits (Measured) ---
const float SERVO_TRUE_STRAIGHT = 76.5;
const float SERVO_MIN_ANGLE     = 20.0;  // Left hard stop
const float SERVO_MAX_ANGLE     = 140.0; // Right hard stop

// Test Speed & Encoder Settings
float TICKS_PER_CM = 14; 
const int TEST_SPEED = 90;

SPIClass SPI_IMU(PA7, PA6, PA5);
Servo steeringServo;
BNO08x myIMU;

enum Mode { MODE_MENU, MODE_ENCODER, MODE_RADIUS_TEST };
Mode currentMode = MODE_MENU;

unsigned long lastPrintMs = 0;
float prevYaw = 0.0;
float accumulatedYaw = 0.0;
bool isTestingRadius = false;

// ============================================================
// HARDWARE HELPERS
// ============================================================
void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { analogWrite(MOT_RPWM_PIN, speed); analogWrite(MOT_LPWM_PIN, 0); }
  else if (speed < 0) { analogWrite(MOT_RPWM_PIN, 0);      analogWrite(MOT_LPWM_PIN, -speed); }
  else                { analogWrite(MOT_RPWM_PIN, 0);      analogWrite(MOT_LPWM_PIN, 0); }
}

void setServoAngle(float angleDeg) {
  // Clamp strictly between measured physical stops
  angleDeg = constrain(angleDeg, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  
  int pulseWidth = (int)((angleDeg / 180.0) * (MAX_PULSE_US - MIN_PULSE_US)) + MIN_PULSE_US;
  steeringServo.writeMicroseconds(pulseWidth);
}

void zeroEncoder() { TIM5->CNT = 0; }
long readEncoder() { return -(int32_t)TIM5->CNT; }

float readYaw() {
  float qI = myIMU.getQuatI(), qJ = myIMU.getQuatJ();
  float qK = myIMU.getQuatK(), qReal = myIMU.getQuatReal();
  if (qI == 0.0f && qJ == 0.0f && qK == 0.0f && qReal == 0.0f) return 0.0f;
  float yawRadians = atan2(2.0f * (qI * qJ + qReal * qK),
                           (qReal * qReal + qI * qI - qJ * qJ - qK * qK));
  return yawRadians * (180.0 / PI);
}

float wrapDeg(float angle) {
  while (angle > 180.0)  angle -= 360.0;
  while (angle < -180.0) angle += 360.0;
  return angle;
}

void printMenu() {
  Serial.println("\n=================================");
  Serial.println("  ROBOT CALIBRATION TOOL");
  Serial.println("=================================");
  Serial.println("Send a number via Serial Monitor:");
  Serial.println("[1] Free-Spin Encoder Count Test");
  Serial.println("[2] Measure Right Turning Radius (At 130.0 deg max right)");
  Serial.println("[3] Measure Left Turning Radius  (At  40.0 deg max left)");
  Serial.println("[0] Stop Motors and Return to Menu");
  Serial.println("=================================\n");
  currentMode = MODE_MENU;
}

// ============================================================
// INITIALIZATION
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  pinMode(MOT_RPWM_PIN, OUTPUT);
  pinMode(MOT_LPWM_PIN, OUTPUT);
  setMotorSpeed(0);

  steeringServo.attach(SERVO_PIN, MIN_PULSE_US, MAX_PULSE_US);
  setServoAngle(SERVO_TRUE_STRAIGHT);

  // Initialize TIM5 Encoder
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

  // Initialize IMU
  SPI_IMU.begin();
  if (myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU)) {
    myIMU.enableGameRotationVector();
    delay(200);
  } else {
    Serial.println("! WARNING: IMU not detected.");
  }

  printMenu();
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '1') {
      Serial.println("\n--- ENCODER TEST MODE ---");
      Serial.println("Rotate wheel 10 times manually to calculate ticks-per-rev.");
      zeroEncoder();
      currentMode = MODE_ENCODER;
    } 
    else if (cmd == '2' || cmd == '3') {
      float targetAngle = (cmd == '2') ? SERVO_MAX_ANGLE : SERVO_MIN_ANGLE;
      
      Serial.print("\n--- MEASURING ");
      Serial.print(cmd == '2' ? "RIGHT" : "LEFT");
      Serial.print(" TURNING RADIUS AT ");
      Serial.print(targetAngle, 1);
      Serial.println(" DEG ---");
      
      setServoAngle(targetAngle);
      delay(500); 
      
      zeroEncoder();
      myIMU.getSensorEvent();
      prevYaw = readYaw();
      accumulatedYaw = 0.0;
      
      setMotorSpeed(TEST_SPEED);
      isTestingRadius = true;
      currentMode = MODE_RADIUS_TEST;
    }
    else if (cmd == '0') {
      setMotorSpeed(0);
      setServoAngle(SERVO_TRUE_STRAIGHT);
      isTestingRadius = false;
      printMenu();
    }
  }

  if (currentMode == MODE_ENCODER) {
    if (millis() - lastPrintMs > 200) {
      Serial.print("Current Ticks: ");
      Serial.println(readEncoder());
      lastPrintMs = millis();
    }
  } 
  else if (currentMode == MODE_RADIUS_TEST && isTestingRadius) {
    if (myIMU.wasReset()) myIMU.enableGameRotationVector();
    
    if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
      float currentYaw = readYaw();
      float deltaYaw = wrapDeg(currentYaw - prevYaw);
      accumulatedYaw += deltaYaw;
      prevYaw = currentYaw;

      if (abs(accumulatedYaw) >= 360.0) {
        setMotorSpeed(0);
        setServoAngle(SERVO_TRUE_STRAIGHT);
        isTestingRadius = false;
        
        long totalTicks = abs(readEncoder());
        float circumferenceCm = totalTicks / TICKS_PER_CM;
        float radiusCm = circumferenceCm / (2.0 * PI);

        Serial.println("\n--- TEST COMPLETE ---");
        Serial.print("Total Ticks for 360 degrees: "); Serial.println(totalTicks);
        Serial.print("Circumference Traveled:      "); Serial.print(circumferenceCm); Serial.println(" cm");
        Serial.print("Calculated Turning Radius:   "); Serial.print(radiusCm); Serial.println(" cm");
        Serial.println("---------------------");
        
        delay(1000);
        printMenu();
      }
    }
  }
}
