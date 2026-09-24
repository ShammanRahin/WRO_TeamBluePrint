#include <Wire.h>
#include <VL53L0X.h>

// Simple ToF reader: front VL53L0X on mux CH3, rear on mux CH4.
// Prints both distances in mm, ~20 times a second. Serial 115200.
// "----" = sensor not found, "FAR" = nothing in range, "TIMEOUT" = no answer.

#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70

VL53L0X tofFront;
VL53L0X tofRear;
bool okFront = false;
bool okRear  = false;

void tcaselect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  pinMode(TCA_RST_PIN, OUTPUT);
  digitalWrite(TCA_RST_PIN, LOW);  delay(10);
  digitalWrite(TCA_RST_PIN, HIGH); delay(10);

  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  tcaselect(3);
  tofFront.setTimeout(100);
  okFront = tofFront.init();
  if (okFront) { tofFront.setMeasurementTimingBudget(20000); tofFront.startContinuous(); }

  tcaselect(4);
  tofRear.setTimeout(100);
  okRear = tofRear.init();
  if (okRear) { tofRear.setMeasurementTimingBudget(20000); tofRear.startContinuous(); }

  Serial.print("front (CH3): "); Serial.println(okFront ? "OK" : "NOT FOUND");
  Serial.print("rear  (CH4): "); Serial.println(okRear  ? "OK" : "NOT FOUND");
}

void printReading(bool ok, VL53L0X &s, uint8_t ch) {
  if (!ok) { Serial.print("----"); return; }
  tcaselect(ch);
  uint16_t mm = s.readRangeContinuousMillimeters();
  if (s.timeoutOccurred()) Serial.print("TIMEOUT");
  else if (mm >= 8190)     Serial.print("FAR");
  else                     { Serial.print(mm); Serial.print(" mm"); }
}

void loop() {
  Serial.print("F: ");
  printReading(okFront, tofFront, 3);
  Serial.print("\tR: ");
  printReading(okRear, tofRear, 4);
  Serial.println();
  delay(50);
}
