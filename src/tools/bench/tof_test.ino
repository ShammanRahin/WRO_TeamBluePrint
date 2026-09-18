#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ============================================================
// SIMPLE ToF READ - one VL53L0X behind the TCA9548A mux
// Blocking continuous read, prints mm once per 100 ms.
// Change TOF_CH to test a different mux channel.
// ============================================================

#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70

#define TOF_CH      3          // 3 = front, 5/6 = sides

VL53L0X sensor;

// ------------------------------------------------------------
void tcaselect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void resetTCA() {
  pinMode(TCA_RST_PIN, OUTPUT);
  digitalWrite(TCA_RST_PIN, LOW);
  delay(10);
  digitalWrite(TCA_RST_PIN, HIGH);
  delay(10);
}

// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  resetTCA();                  // pulse the mux reset line
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  Serial.println(F("\n--- SIMPLE ToF READ ---"));

  tcaselect(TOF_CH);
  sensor.setBus(&Wire);
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.print(F("ToF on CH"));
    Serial.print(TOF_CH);
    Serial.println(F(": INIT FAILED - check wiring / channel"));
    while (1) { delay(1000); }
  }

  sensor.setMeasurementTimingBudget(30000);
  sensor.startContinuous();
  Serial.print(F("ToF on CH"));
  Serial.print(TOF_CH);
  Serial.println(F(": READY\n"));
}

// ------------------------------------------------------------
void loop() {
  tcaselect(TOF_CH);
  uint16_t mm = sensor.readRangeContinuousMillimeters();

  Serial.print(mm);
  Serial.print(F(" mm"));
  if (sensor.timeoutOccurred()) Serial.print(F("   TIMEOUT"));
  Serial.println();

  delay(100);
}
