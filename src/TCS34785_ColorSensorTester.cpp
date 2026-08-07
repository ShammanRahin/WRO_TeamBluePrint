#include <Wire.h>

#define TCA_ADDR 0x70
#define TCS_ADDR 0x29

// Your selection function
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // Set explicit I2C pins for STM32 Blackpill
  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();
  delay(100);

  Serial.println("\n--- TCA9548A & TCS34725 Finder ---");

  // Verify Multiplexer is present at 0x70
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ERROR: TCA9548A (0x70) not found! Check SCL/SDA wiring.");
    while (1);
  }
  Serial.println("TCA9548A found at 0x70.");

  // Scan channels 0-7 to locate the TCS34725
  bool found = false;
  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);
    delay(10);

    Wire.beginTransmission(TCS_ADDR);
    if (Wire.endTransmission() == 0) {
      Serial.print("SUCCESS: TCS34725 found on Channel ");
      Serial.println(i);

      // Initialize TCS34725
      Wire.beginTransmission(TCS_ADDR);
      Wire.write(0x80 | 0x00); // Enable Register
      Wire.write(0x01);        // Power ON
      Wire.endTransmission();
      delay(3);

      Wire.beginTransmission(TCS_ADDR);
      Wire.write(0x80 | 0x00);
      Wire.write(0x03);        // Power ON + RGBC Enable
      Wire.endTransmission();

      Wire.beginTransmission(TCS_ADDR);
      Wire.write(0x80 | 0x01); // ATIME (Integration time ~50ms)
      Wire.write(0xEB);
      Wire.endTransmission();

      Wire.beginTransmission(TCS_ADDR);
      Wire.write(0x80 | 0x0F); // Control (Gain 4x)
      Wire.write(0x01);
      Wire.endTransmission();

      found = true;
      break; 
    }
  }

  if (!found) {
    Serial.println("ERROR: TCS34725 not found on any channel!");
    while (1);
  }
}

void loop() {
  // Loop through channels to read the sensor wherever it's located
  for (uint8_t i = 0; i < 8; i++) {
    tcaselect(i);
    Wire.beginTransmission(TCS_ADDR);
    if (Wire.endTransmission() == 0) {
      // Read 8 bytes starting from CDATAL (0x14)
      Wire.beginTransmission(TCS_ADDR);
      Wire.write(0x80 | 0x14);
      Wire.endTransmission();

      Wire.requestFrom((uint8_t)TCS_ADDR, (uint8_t)8);
      if (Wire.available() >= 8) {
        uint16_t clear = Wire.read() | (Wire.read() << 8);
        uint16_t red   = Wire.read() | (Wire.read() << 8);
        uint16_t green = Wire.read() | (Wire.read() << 8);
        uint16_t blue  = Wire.read() | (Wire.read() << 8);

        Serial.print("Channel "); Serial.print(i);
        Serial.print(" | Clear: "); Serial.print(clear);
        Serial.print(" Red: "); Serial.print(red);
        Serial.print(" Green: "); Serial.print(green);
        Serial.print(" Blue: "); Serial.println(blue);
      }
    }
  }
  delay(500);
}
