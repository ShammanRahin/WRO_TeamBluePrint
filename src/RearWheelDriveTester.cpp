#include <Arduino.h>

// Define pins based on your wiring diagram
const int RPWM_PIN = PB8; // TIM4_CH3
const int LPWM_PIN = PB9; // TIM4_CH4
const int DRV_EN_PIN = PB1; // Motor enable pin

void setup() {
  // Configure motor control pins as outputs
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT);

  // Enable the BTS7960 driver (Drive PB1 HIGH)
  digitalWrite(DRV_EN_PIN, HIGH);
}

void loop() {
  // 1. Spin Forward for 2 seconds (PWM on RPWM, LPWM LOW)
  analogWrite(RPWM_PIN, 128); // Speed 0 - 255 (approx 50% duty cycle)
  analogWrite(LPWM_PIN, 0);
  delay(2000);

  // 2. Stop for 1 second
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  delay(1000);

  // 3. Spin Reverse for 2 seconds (RPWM LOW, PWM on LPWM)
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 128); // Speed 0 - 255 (approx 50% duty cycle)
  delay(2000);

  // 4. Stop for 1 second
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
  delay(1000);
}
