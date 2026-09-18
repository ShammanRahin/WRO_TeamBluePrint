#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// ============================================================
// THREE-ToF TEST - front CH3 plus the two new side sensors
//
// All three are VL53L0X on the TCA9548A.
//   CH3 = front (already in the firmware)
//   CH5 = side A   <- identify these two by waving a hand in front
//   CH6 = side B      of one and watching which column moves
//
// Reads are NON-BLOCKING: it polls each sensor's interrupt flag the
// same way the firmware does, so three sensors sharing one bus do not
// stall the loop. The reported loop Hz is what the FSM would get.
//
// WHAT TO CHECK
//   1. all three say READY at boot
//   2. cover one sensor at a time - only that column should change
//   3. hold a wall at ~200, ~350, ~700 mm and confirm the numbers agree
//      with a tape measure (those are WALL_PANIC, WALL_CLEAR, FRONT_TURN)
//   4. point a side sensor down an open straight - it should read OOR,
//      not a random short value. That is what "wall gave way" relies on.
//   5. loop Hz should stay in the hundreds
// ============================================================

#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70

#define CH_FRONT 3
#define CH_SIDE_A 5
#define CH_SIDE_B 6

const uint16_t TOF_MAX_VALID_MM = 1200;   // VL53L0X practical ceiling

// thresholds the firmware uses, shown inline so you can eyeball them
const uint16_t WALL_PANIC_MM  = 200;
const uint16_t WALL_CLEAR_MM  = 350;
const uint16_t FRONT_TURN_MM  = 700;

struct Tof {
  VL53L0X  dev;
  uint8_t  ch;
  bool     ok;
  uint16_t mm;
  bool     valid;
  uint32_t reads;
  const char *name;
};

Tof tofF = { VL53L0X(), CH_FRONT,  false, 9999, false, 0, "FRONT/CH3" };
Tof tofA = { VL53L0X(), CH_SIDE_A, false, 9999, false, 0, "SIDE/CH5 " };
Tof tofB = { VL53L0X(), CH_SIDE_B, false, 9999, false, 0, "SIDE/CH6 " };

uint32_t nLoop = 0;

// ============================================================
// I2C PLUMBING
// ============================================================
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

// ============================================================
// SENSORS
// ============================================================
void initTof(Tof &t) {
  tcaselect(t.ch);
  delay(5);
  t.dev.setBus(&Wire);
  t.dev.setTimeout(100);
  t.ok = t.dev.init();
  if (t.ok) {
    t.dev.setMeasurementTimingBudget(30000);
    t.dev.startContinuous(0);
  }
  Serial.print(t.name);
  Serial.println(t.ok ? F(": READY") : F(": FAILED"));
}

// Poll the interrupt flag - never blocks waiting for a conversion.
void serviceTof(Tof &t) {
  if (!t.ok) { t.valid = false; return; }
  tcaselect(t.ch);
  if (t.dev.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t d = t.dev.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    t.dev.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    t.reads++;
    t.valid = (d > 0 && d < TOF_MAX_VALID_MM);
    if (t.valid) t.mm = d;
  }
}

void printTof(Tof &t, bool markThresholds) {
  Serial.print(' '); Serial.print(t.name); Serial.print(' ');
  if (!t.ok) { Serial.print(F("  ----")); return; }
  if (!t.valid) { Serial.print(F("   OOR")); return; }

  if (t.mm < 1000) Serial.print(' ');
  if (t.mm < 100)  Serial.print(' ');
  Serial.print(t.mm); Serial.print(F("mm"));

  if (markThresholds) {
    if      (t.mm <= WALL_PANIC_MM) Serial.print(F(" [PANIC]"));
    else if (t.mm <= WALL_CLEAR_MM) Serial.print(F(" [<clear]"));
    else if (t.mm <= FRONT_TURN_MM) Serial.print(F(" [TURN]"));
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  resetTCA();
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

  Serial.println(F("\n--- THREE-ToF TEST ---"));
  Serial.println(F("mux 0x70   CH3 front   CH5 side A   CH6 side B"));

  initTof(tofF);
  initTof(tofA);
  initTof(tofB);

  Serial.println(F("\nwave a hand in front of one sensor to identify it\n"));
}

void loop() {
  serviceTof(tofF);
  serviceTof(tofA);
  serviceTof(tofB);
  nLoop++;

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    float secs = (millis() - lastPrint) / 1000.0;

    printTof(tofF, true);
    Serial.print(F("  |"));
    printTof(tofA, false);
    Serial.print(F("  |"));
    printTof(tofB, false);

    Serial.print(F("   | loop ")); Serial.print(nLoop / secs, 0);
    Serial.print(F("Hz  rates "));
    Serial.print(tofF.reads / secs, 0); Serial.print('/');
    Serial.print(tofA.reads / secs, 0); Serial.print('/');
    Serial.print(tofB.reads / secs, 0); Serial.println(F("Hz"));

    nLoop = tofF.reads = tofA.reads = tofB.reads = 0;
    lastPrint = millis();
  }
}
