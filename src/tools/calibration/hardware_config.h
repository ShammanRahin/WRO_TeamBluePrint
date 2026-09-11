// ============================================================================
// hardware_config.h - one place for every pin and every calibration constant
// ============================================================================
//
// Every sketch in this folder includes this file. If you move a wire, you
// change it HERE and nowhere else.
//
// Anything marked VERIFY is not confirmed against the current PCB yet. Check
// it with a multimeter before you trust a calibration run that depends on it.
//
// Board:      WeAct BlackPill, STM32F411CEU6, Arduino core for STM32
// Upload:     DFU over USB (hold BOOT0, tap NRST, release BOOT0) or ST-Link
// Serial:     115200 baud on the USB CDC port
// ============================================================================

#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------- motor ----
// VERIFY: the bench tester and the round firmware disagree on these two.
// Whichever pair you confirm, fix it in BOTH places before competition.
const int RPWM_PIN   = PB9;    // VERIFY - RearWheelDriveTester.cpp says PB8
const int LPWM_PIN   = PB8;    // VERIFY - RearWheelDriveTester.cpp says PB9
const int DRV_EN_PIN = PB1;    // BTS7960 enable, tied high in firmware

// ---------------------------------------------------------------- servo ----
// Current build uses a JX PS-1171MG 17 g digital servo.
// The nationals car used an MG996R; the angles below were found on THAT car,
// so step 04 must be re-run for the JX before you trust them.
const int SERVO_PIN = PA8;
const float SERVO_TRUE_STRAIGHT = 69.0;   // RE-MEASURE for the JX servo
const float SERVO_MAX_LEFT      = 5.0;    // RE-MEASURE
const float SERVO_MAX_RIGHT     = 115.0;  // RE-MEASURE

// ------------------------------------------------------------- encoder ----
// Quadrature encoder on the drive motor, read by TIM3 in hardware.
// PA6 = TIM3_CH1, PA7 = TIM3_CH2. No pin defines needed; TIM3 owns them.
const float TICKS_PER_CM = 31.933;        // output of step 02

// ----------------------------------------------------------------- IMU -----
// BNO085 over SPI. An MPU6050 was tried first and abandoned: yaw drift was
// large enough to lose a 90 degree turn inside three laps.
const int IMU_CS_PIN  = PB0;
const int IMU_INT_PIN = PB13;
const int IMU_RST_PIN = PB14;

// ---------------------------------------------------- I2C mux + sensors ----
// TCA9548A 8-channel I2C multiplexer at 0x70.
// (PCA9548A appears in some older docs. Same part for our purposes - pin and
//  register compatible. We standardise on the TCA name.)
#define TCA_ADDR 0x70

// Current build: two VL53L1X for front and rear protection, one TCS34725
// looking down at the mat.
// VERIFY: which physical sensor sits on channel 1 and which on channel 2.
const uint8_t CH_TOF_A  = 1;   // VERIFY - front or rear?
const uint8_t CH_TOF_B  = 2;   // VERIFY - the other one
const uint8_t CH_COLOUR = 3;   // TCS34725, downward facing

// A reading is thrown away if the return signal is weaker than this. Below
// this level the sensor is reading the mat, not a wall. Output of step 05.
const float    SIGNAL_MIN_MCPS  = 4.0;
const uint16_t TOF_MAX_VALID_MM = 1300;

// -------------------------------------------------------------- button -----
const int START_BTN_PIN = PA5;   // active low: pressed = LOW

// ============================================================================
// Small helpers every sketch uses. Header-only on purpose - one file to copy.
// ============================================================================

// Select one channel on the mux. Call before touching any sensor behind it.
inline void tcaselect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Running mean and standard deviation, Welford's method.
// Used everywhere because every calibration here is "take N samples, look at
// the distribution, keep the mean". If the standard deviation is large, the
// mean is not trustworthy and you have a mechanical problem, not a maths one.
struct Stats {
  long  n = 0;
  double mean = 0.0, m2 = 0.0;
  double minv = 1e18, maxv = -1e18;

  void add(double x) {
    n++;
    double d = x - mean;
    mean += d / n;
    m2   += d * (x - mean);
    if (x < minv) minv = x;
    if (x > maxv) maxv = x;
  }
  double sd() const { return n > 1 ? sqrt(m2 / (n - 1)) : 0.0; }
  void print(const char *label, const char *unit) {
    Serial.print(label);
    Serial.print("  n="); Serial.print(n);
    Serial.print("  mean="); Serial.print(mean, 4); Serial.print(unit);
    Serial.print("  sd="); Serial.print(sd(), 4);
    Serial.print("  min="); Serial.print(minv, 3);
    Serial.print("  max="); Serial.println(maxv, 3);
  }
};

// Block until the start button is pressed and released. Every sketch uses
// this so you can get your hands out of the way before the car moves.
inline void waitForButton(const char *prompt) {
  Serial.println(prompt);
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  while (digitalRead(START_BTN_PIN) == HIGH) delay(5);
  while (digitalRead(START_BTN_PIN) == LOW)  delay(5);
  delay(150);   // let the chassis stop rocking from your finger
}

// Read the TIM3 hardware quadrature counter.
inline long readEncoder() { return (int16_t)TIM3->CNT; }
inline void zeroEncoder() { TIM3->CNT = 0; }

// Start TIM3 in encoder mode on PA6/PA7. Call once in setup().
inline void encoderBegin() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &g);
  TIM3->SMCR = 0x03;    // encoder mode 3: count on both TI1 and TI2 edges
  TIM3->CCMR1 = 0x0101; // both channels mapped to their inputs
  TIM3->CCER = 0x0011;  // both channels enabled
  TIM3->ARR = 0xFFFF;
  TIM3->CNT = 0;
  TIM3->CR1 = 0x01;     // go
}

// Drive the rear wheels. speed is -255..255, negative is reverse.
inline void motorBegin() {
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);
  digitalWrite(DRV_EN_PIN, HIGH);
}
inline void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { analogWrite(RPWM_PIN, speed); analogWrite(LPWM_PIN, 0); }
  else            { analogWrite(RPWM_PIN, 0); analogWrite(LPWM_PIN, -speed); }
}
