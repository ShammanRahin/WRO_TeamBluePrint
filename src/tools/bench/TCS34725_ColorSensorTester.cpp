#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ============================================================
// MUX SCAN - finds which TCA9548A channel each sensor is on.
// Every 3 s it scans channels 0-7 (the mux has no channel 8) and the bare
// bus (all channels off), prints every I2C address it finds, and names
// what sits at 0x29:
//   VL53L0X  = ToF (model ID 0xEE)
//   TCS34725 = the old colour sensor (same 0x29 address, ID 0x44 / 0x4D)
// Every VL53L0X found is started and its distance printed in between scans.
// A sensor that shows up on EVERY channel is wired to the main bus,
// not through the mux.
// ============================================================

#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70

VL53L0X tof[8];
bool tofOn[8];

void tcaselect(int ch) {                 // ch < 0 = all channels off
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(ch < 0 ? 0 : (1 << ch));
  Wire.endTransmission();
}

bool present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

int readReg8(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

void scanChannel(int ch) {
  tcaselect(ch);
  delay(2);
  if (ch < 0) Serial.print(F("bus : "));
  else { Serial.print(F("CH")); Serial.print(ch); Serial.print(F("  : ")); }
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (a == TCA_ADDR) continue;         // the mux itself
    if (!present(a)) continue;
    found++;
    Serial.print(F("0x")); if (a < 16) Serial.print('0'); Serial.print(a, HEX);
    if (a == 0x29) {
      int vlId  = readReg8(0x29, 0xC0);          // VL53L0X model ID
      int tcsId = readReg8(0x29, 0x80 | 0x12);   // TCS34725 ID register
      if (vlId == 0xEE)                     Serial.print(F(" VL53L0X"));
      else if (tcsId == 0x44 || tcsId == 0x4D) Serial.print(F(" TCS34725 (colour)"));
      else                                  Serial.print(F(" unknown"));
      if (vlId == 0xEE && ch >= 0 && !tofOn[ch]) {
        tof[ch].setBus(&Wire);
        tof[ch].setTimeout(500);
        if (tof[ch].init()) {
          tof[ch].setMeasurementTimingBudget(30000);
          tof[ch].startContinuous();
          tofOn[ch] = true;
          Serial.print(F(" started"));
        } else {
          Serial.print(F(" init FAILED"));
        }
      }
    }
    Serial.print(F("  "));
  }
  if (!found) Serial.print(F("nothing"));
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  pinMode(TCA_RST_PIN, OUTPUT);
  digitalWrite(TCA_RST_PIN, LOW);  delay(10);
  digitalWrite(TCA_RST_PIN, HIGH); delay(10);

  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  Wire.setClock(100000);                 // slow and safe for scanning
  delay(100);

  Serial.println(F("\n--- MUX SCAN ---"));
  if (!present(TCA_ADDR)) Serial.println(F("!! TCA9548A not found at 0x70 - check SDA/SCL/power"));
}

unsigned long lastScan = 0;

void loop() {
  if (lastScan == 0 || millis() - lastScan >= 3000) {
    lastScan = millis();
    Serial.println(F("---- scan ----"));
    scanChannel(-1);
    for (int ch = 0; ch < 8; ch++) scanChannel(ch);
    Serial.println(F("--------------"));
  }

  bool any = false;
  for (int ch = 0; ch < 8; ch++) {
    if (!tofOn[ch]) continue;
    any = true;
    tcaselect(ch);
    uint16_t mm = tof[ch].readRangeContinuousMillimeters();
    Serial.print(F("CH")); Serial.print(ch); Serial.print(F(": "));
    Serial.print(mm); Serial.print(F(" mm"));
    if (tof[ch].timeoutOccurred()) Serial.print(F(" TIMEOUT"));
    Serial.print(F("   "));
  }
  if (any) Serial.println();
  delay(200);
}
