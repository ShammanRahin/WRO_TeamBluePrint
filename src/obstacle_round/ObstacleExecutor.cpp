// ============================================================================
// ObstacleExecutor.cpp — STM32 side of the Pi-driven obstacle round.
//
// This is NOT ObstacleRound.cpp. That firmware owns its own state machine and
// drives the car itself. THIS firmware owns no strategy at all: the Pi runs the
// FSM and sends DriveCommands; the STM32 executes them and reports back.
//
//   Pi  -> us    DRIVE  11 bytes @ ~30 Hz   steering angle, speed, mode
//   us  -> Pi    TELEM  22 bytes @  50 Hz   heading, odometry, ToF, floor, button
//
// What lives HERE and must never move to the Pi:
//   * servo trim, direction and end limits      (mechanical facts)
//   * degrees -> microseconds                   (mechanical facts)
//   * closed-loop speed PID on the encoder      (needs 200 Hz, Pi link is 30)
//   * heading-hold P loop on the IMU            (needs IMU rate, same reason)
//   * the watchdog                              (safety must not depend on the Pi)
//
// Protocol spec: docs/PI_STM32_PROTOCOL.md
// Board: WeAct BlackPill STM32F411CEU6, Arduino core. Upload by DFU.
// ============================================================================

#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <Adafruit_TCS34725.h>
#include <VL53L1X.h>

#define PiSerial Serial          // native USB CDC -> /dev/ttyACM0 on the Pi

// ============================================================================
// PINS — taken from src/tools/calibration/hardware_config.h. Change there and
// here together, or better: include that header once the build system allows.
// ============================================================================
const int RPWM_PIN      = PB9;
const int LPWM_PIN      = PB8;   // VERIFY: RearWheelDriveTester.cpp swaps these
const int DRV_EN_PIN    = PB1;
const int SERVO_PIN     = PA8;
const int START_BTN_PIN = PA5;   // active low
const int IMU_CS_PIN    = PB0;
const int IMU_INT_PIN   = PB13;
const int IMU_RST_PIN   = PB14;

#define TCA_ADDR 0x70
const uint8_t CH_LEFT  = 1;      // VL53L1X
const uint8_t CH_RIGHT = 3;
const uint8_t CH_FRONT = 4;
// TCS34725 channel is discovered by scanning at boot (see initHardware)

// ============================================================================
// CALIBRATION — the numbers that make the Pi's units mean something
// ============================================================================

// ---- odometry -------------------------------------------------------------
// MEASURED on the current car via 02_ticks_per_cm.cpp: 248.8 ticks/rev over a
// 5.2 cm wheel. This is the same constant OpenRound.cpp runs on, and it is the
// authoritative one.
//
// Do NOT copy the 31.933 in ObstacleRound.cpp / hardware_config.h — that came
// from an earlier build with different gearing and is stale. If you ever see
// the two disagree again, this one (and OpenRound.cpp) is right.
//
// Everything the Pi commands is scaled by this: speed targets, distance_mm
// telemetry, and every odometry-terminated leg (parking shuffle, avoidance
// return, corner backstops).
const float TICKS_PER_CM  = 14.853;
const float MM_PER_TICK   = 10.0f / TICKS_PER_CM;     // = 0.673 mm

// ---- steering: road-wheel degrees -> servo degrees ------------------------
// The Pi speaks road-wheel angle, + = LEFT, limited to +-STEER_LOCK_DEG.
// The servo is asymmetric about straight, so left and right get their own
// gain. Both gains are NEGATIVE here: this servo's angle DECREASES as the road
// wheels go left. That is a fact about this linkage, not a sign error.
//
//   left  gain = (SERVO_MAX_LEFT  - TRUE_STRAIGHT) / (+STEER_LOCK)
//   right gain = (SERVO_MAX_RIGHT - TRUE_STRAIGHT) / (-STEER_LOCK)
//
// RE-MEASURE all three for the JX PS-1171MG (step 04_true_straight_servo.cpp).
// The values below were found on the MG996R car.
const float SERVO_TRUE_STRAIGHT = 69.0f;
const float SERVO_MAX_LEFT      = 5.0f;
const float SERVO_MAX_RIGHT     = 115.0f;
const float STEER_LOCK_DEG      = 35.0f;    // road wheel, SPECSHEET s3, FINAL

// ---- speed PID (200 Hz, Decision #22: closed loop is mandatory) -----------
// Start here, tune on the mat. Kff carries most of the load; the PID only
// trims. A pure-PID tune with no feedforward will feel sluggish and overshoot.
const float SPEED_KFF = 255.0f / 700.0f;   // duty per mm/s, from TOP_SPEED 700
const float SPEED_KP  = 0.25f;
const float SPEED_KI  = 0.60f;
const float SPEED_KD  = 0.0f;
const float SPEED_I_CLAMP = 120.0f;        // anti-windup, in duty counts
const uint32_t SPEED_PID_HZ = 200;

// ---- heading hold (P only; matches the tuned open-round behaviour) --------
const float HEAD_KP        = 1.6f;         // road-wheel deg per deg of error
const float HEAD_MAX_DEG   = 25.0f;        // don't full-lock on a heading error
const float SERVO_SLEW_DEG = 2.5f;         // per servo update, anti-scrub

// ---- link + safety --------------------------------------------------------
const uint32_t TELEM_HZ        = 50;
const uint32_t WATCHDOG_MS     = 250;      // no DRIVE frame this long -> cut motor
const uint16_t TOF_MAX_VALID_MM = 1300;
const float    SIGNAL_MIN_MCPS  = 4.0f;
const uint16_t TOF_INVALID      = 0xFFFF;

// ============================================================================
// WIRE PROTOCOL
// ============================================================================
const uint8_t DRIVE_SYNC0 = 0xAA, DRIVE_SYNC1 = 0x55;
const uint8_t TELEM_SYNC0 = 0x55, TELEM_SYNC1 = 0xAA;
const uint8_t DRIVE_LEN = 11, TELEM_LEN = 22;

// DRIVE flags
const uint8_t F_ENABLE      = 0x01;
const uint8_t F_CLOSED_LOOP = 0x02;
const uint8_t F_MODE_MASK   = 0x0C;
const uint8_t F_MODE_SHIFT  = 2;
// steer modes
const uint8_t MODE_DIRECT = 0, MODE_HEADING_HOLD = 1, MODE_STOP = 2;

// TELEM status bits
const uint8_t S_ENABLED     = 0x01;
const uint8_t S_WATCHDOG    = 0x02;
const uint8_t S_BUTTON      = 0x04;
const uint8_t S_CLOSED_LOOP = 0x08;
const uint8_t S_IMU_OK      = 0x10;
const uint8_t S_TOF_OK      = 0x20;
const uint8_t S_COLOUR_OK   = 0x40;

// floor colour codes
const uint8_t FLOOR_NONE = 0, FLOOR_ORANGE = 1, FLOOR_BLUE = 2;

// ============================================================================
// STATE
// ============================================================================
SPIClass SPI_IMU(PB5, PB4, PB3);      // MOSI, MISO, SCK
Servo    steeringServo;
BNO08x   myIMU;
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);
VL53L1X  tofL, tofR, tofF;

bool  tofLok = false, tofRok = false, tofFok = false;
bool  tcsConnected = false;   uint8_t tcsChannel = 0;
bool  imuOk = false;

// --- command from the Pi ---
struct DriveCmd {
  uint8_t  seq = 0;
  bool     enable = false;
  bool     closedLoop = true;
  uint8_t  mode = MODE_STOP;
  float    steerDeg = 0.0f;      // road wheel, + = left
  float    speedMmps = 0.0f;
  float    headingDeg = 0.0f;
  uint32_t stamp = 0;            // millis() when accepted
} cmd;

// --- measured ---
float   gHeading = 0.0f, gYawRate = 0.0f, initialYawOffset = 0.0f;
float   gPrevHeading = 0.0f;
long    gEncTotal = 0;           // accumulated ticks, survives TIM3 wrap
int16_t gLastRaw = 0;
float   gSpeedMmps = 0.0f;
bool    gButtonLatched = false;
bool    gWatchdogTripped = false;

uint16_t tofFrontMm = TOF_INVALID, tofLeftMm = TOF_INVALID, tofRightMm = TOF_INVALID;
uint8_t  gFloor = FLOOR_NONE;

float pidI = 0.0f, pidPrevErr = 0.0f;
float servoCmdDeg = SERVO_TRUE_STRAIGHT;

// ============================================================================
// LOW-LEVEL HELPERS
// ============================================================================
static inline uint8_t xor8(const uint8_t *p, uint8_t n) {
  uint8_t x = 0; while (n--) x ^= *p++; return x;
}
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static inline float wrapDeg(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}
void tcaselect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR); Wire.write(1 << ch); Wire.endTransmission();
}

// ---- motor: raw duty, -255..255 ----
void setMotorDuty(int duty) {
  duty = constrain(duty, -255, 255);
  if (duty > 0)      { analogWrite(RPWM_PIN, duty);  analogWrite(LPWM_PIN, 0); }
  else if (duty < 0) { analogWrite(RPWM_PIN, 0);     analogWrite(LPWM_PIN, -duty); }
  else               { analogWrite(RPWM_PIN, 0);     analogWrite(LPWM_PIN, 0); }
}

// ---- steering: THE mapping. road-wheel degrees -> servo -> microseconds ----
// This function is the entire reason the Pi never sees a servo. Everything
// mechanical about steering is contained in these few lines.
void setSteerDeg(float roadDeg) {
  roadDeg = clampf(roadDeg, -STEER_LOCK_DEG, STEER_LOCK_DEG);

  float servoDeg;
  if (roadDeg >= 0.0f) {          // left
    float gain = (SERVO_MAX_LEFT - SERVO_TRUE_STRAIGHT) / STEER_LOCK_DEG;
    servoDeg = SERVO_TRUE_STRAIGHT + roadDeg * gain;
  } else {                        // right
    float gain = (SERVO_MAX_RIGHT - SERVO_TRUE_STRAIGHT) / (-STEER_LOCK_DEG);
    servoDeg = SERVO_TRUE_STRAIGHT + roadDeg * gain;
  }

  // slew limit: protects the tyres and the BEC from step commands
  float d = servoDeg - servoCmdDeg;
  if (d >  SERVO_SLEW_DEG) servoDeg = servoCmdDeg + SERVO_SLEW_DEG;
  if (d < -SERVO_SLEW_DEG) servoDeg = servoCmdDeg - SERVO_SLEW_DEG;

  float lo = min(SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  float hi = max(SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  servoCmdDeg = clampf(servoDeg, lo, hi);

  steeringServo.writeMicroseconds((int)((servoCmdDeg / 180.0f) * 1000.0f) + 1000);
}

// ---- encoder ----
void encoderBegin() {
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7; g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_HIGH; g.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &g);
  TIM3->SMCR = 0x03; TIM3->CCMR1 = 0x0101; TIM3->CCER = 0x0011;
  TIM3->ARR = 0xFFFF; TIM3->CNT = 0; TIM3->CR1 = 0x01;
}
// TIM3 is 16-bit and wraps. Accumulate deltas so odometry is unbounded.
void serviceEncoder() {
  int16_t raw = (int16_t)TIM3->CNT;
  gEncTotal += (int16_t)(raw - gLastRaw);
  gLastRaw = raw;
}

// ---- IMU ----
float readYawRaw() {
  float qI = myIMU.getQuatI(), qJ = myIMU.getQuatJ();
  float qK = myIMU.getQuatK(), qR = myIMU.getQuatReal();
  if (qI == 0 && qJ == 0 && qK == 0 && qR == 0) return 0.0f;
  return atan2(2.0f * (qI * qJ + qR * qK),
               (qR * qR + qI * qI - qJ * qJ - qK * qK)) * (180.0f / PI);
}

// ============================================================================
// SENSOR SERVICE
// ============================================================================
void serviceOneToF(VL53L1X &s, bool present, uint8_t ch, uint16_t &mm) {
  if (!present) { mm = TOF_INVALID; return; }
  tcaselect(ch);
  if (!s.dataReady()) return;
  uint16_t r = s.read(false);
  bool ok = (s.ranging_data.range_status == VL53L1X::RangeValid) &&
            (s.ranging_data.peak_signal_count_rate_MCPS >= SIGNAL_MIN_MCPS) &&
            (r > 0) && (r <= TOF_MAX_VALID_MM);
  mm = ok ? r : TOF_INVALID;
}

// Floor colour. Single-sample with hysteresis — SPECSHEET s5 is explicit that
// a median filter smears the 28.6 ms line event away. Do not "improve" this
// by filtering it.
void serviceFloor() {
  if (!tcsConnected) { gFloor = FLOOR_NONE; return; }
  tcaselect(tcsChannel);
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  if (c < 50) { gFloor = FLOOR_NONE; return; }
  float rn = (float)r / c, bn = (float)b / c;

  // TODO(colour): thresholds from 08_floor_colour_thresholds.cpp.
  // Orange: high red fraction. Blue: high blue fraction. Mat: neither.
  if      (rn > 0.42f && bn < 0.25f) gFloor = FLOOR_ORANGE;
  else if (bn > 0.38f && rn < 0.30f) gFloor = FLOOR_BLUE;
  else                               gFloor = FLOOR_NONE;
}

// ============================================================================
// LINK — receive DRIVE
// ============================================================================
uint8_t rxBuf[DRIVE_LEN]; uint8_t rxLen = 0;

void applyDriveFrame(const uint8_t *f) {
  // f points at a validated 11-byte frame
  cmd.seq        = f[2];
  uint8_t flags  = f[3];
  cmd.enable     = flags & F_ENABLE;
  cmd.closedLoop = flags & F_CLOSED_LOOP;
  cmd.mode       = (flags & F_MODE_MASK) >> F_MODE_SHIFT;
  int16_t steer_dd, speed, head_dd;
  memcpy(&steer_dd, f + 4, 2);
  memcpy(&speed,    f + 6, 2);
  memcpy(&head_dd,  f + 8, 2);
  cmd.steerDeg   = steer_dd / 10.0f;
  cmd.speedMmps  = (float)speed;
  cmd.headingDeg = head_dd / 10.0f;
  cmd.stamp      = millis();
  gWatchdogTripped = false;
}

void serviceLink() {
  while (PiSerial.available()) {
    uint8_t ch = PiSerial.read();
    if (rxLen == 0) { if (ch == DRIVE_SYNC0) rxBuf[rxLen++] = ch; continue; }
    if (rxLen == 1) {
      if (ch == DRIVE_SYNC1) rxBuf[rxLen++] = ch;
      else rxLen = (ch == DRIVE_SYNC0) ? 1 : 0;   // resync, don't drop a sync
      continue;
    }
    rxBuf[rxLen++] = ch;
    if (rxLen == DRIVE_LEN) {
      if (xor8(rxBuf + 2, DRIVE_LEN - 3) == rxBuf[DRIVE_LEN - 1]) applyDriveFrame(rxBuf);
      rxLen = 0;                                   // bad checksum = silently drop
    }
  }
}

// ============================================================================
// LINK — send TELEM
// ============================================================================
void sendTelemetry() {
  uint8_t f[TELEM_LEN];
  f[0] = TELEM_SYNC0; f[1] = TELEM_SYNC1;
  f[2] = cmd.seq;

  uint8_t st = 0;
  if (cmd.enable && !gWatchdogTripped) st |= S_ENABLED;
  if (gWatchdogTripped)                st |= S_WATCHDOG;
  if (gButtonLatched)                  st |= S_BUTTON;
  if (cmd.closedLoop)                  st |= S_CLOSED_LOOP;
  if (imuOk)                           st |= S_IMU_OK;
  if (tofFok || tofLok || tofRok)      st |= S_TOF_OK;
  if (tcsConnected)                    st |= S_COLOUR_OK;
  f[3] = st;

  int32_t distMm  = (int32_t)(gEncTotal * MM_PER_TICK);
  int16_t spd     = (int16_t)gSpeedMmps;
  int16_t head_dd = (int16_t)(gHeading * 10.0f);
  int16_t yaw_dd  = (int16_t)(gYawRate * 10.0f);

  memcpy(f + 4,  &distMm,  4);
  memcpy(f + 8,  &spd,     2);
  memcpy(f + 10, &head_dd, 2);
  memcpy(f + 12, &yaw_dd,  2);
  memcpy(f + 14, &tofFrontMm, 2);
  memcpy(f + 16, &tofLeftMm,  2);
  memcpy(f + 18, &tofRightMm, 2);
  f[20] = gFloor;
  f[21] = xor8(f + 2, TELEM_LEN - 3);

  PiSerial.write(f, TELEM_LEN);
}

// ============================================================================
// CONTROL
// ============================================================================

// Speed PID, 200 Hz. Feedforward + PI. Output is motor duty.
int speedControl(float targetMmps, float dt) {
  if (!cmd.closedLoop) {
    // open loop: treat the request as a fraction of top speed. Lets you drive
    // the whole stack before the PID is tuned.
    return (int)clampf(targetMmps * SPEED_KFF, -255.0f, 255.0f);
  }
  float err = targetMmps - gSpeedMmps;
  pidI += err * dt;
  pidI = clampf(pidI, -SPEED_I_CLAMP / max(SPEED_KI, 0.001f),
                       SPEED_I_CLAMP / max(SPEED_KI, 0.001f));
  float d = (dt > 0.0f) ? (err - pidPrevErr) / dt : 0.0f;
  pidPrevErr = err;

  float out = targetMmps * SPEED_KFF + SPEED_KP * err + SPEED_KI * pidI + SPEED_KD * d;
  return (int)clampf(out, -255.0f, 255.0f);
}

// Decide the road-wheel angle for this tick from the commanded mode.
float steeringForMode() {
  if (cmd.mode == MODE_HEADING_HOLD) {
    // Close the heading loop HERE, at IMU rate, not over the 30 Hz Pi link.
    float err = wrapDeg(cmd.headingDeg - gHeading);
    return clampf(HEAD_KP * err, -HEAD_MAX_DEG, HEAD_MAX_DEG);
  }
  return cmd.steerDeg;                 // MODE_DIRECT
}

// ============================================================================
// SETUP
// ============================================================================
void initHardware() {
  pinMode(RPWM_PIN, OUTPUT); pinMode(LPWM_PIN, OUTPUT);
  pinMode(DRV_EN_PIN, OUTPUT); digitalWrite(DRV_EN_PIN, HIGH);
  setMotorDuty(0);

  pinMode(START_BTN_PIN, INPUT_PULLUP);

  steeringServo.attach(SERVO_PIN);
  setSteerDeg(0.0f);

  encoderBegin();

  Wire.setSDA(PB7); Wire.setSCL(PB6); Wire.begin(); Wire.setClock(400000);
  delay(100);

  for (uint8_t i = 0; i < 8; i++) {            // find the TCS34725
    tcaselect(i); delay(10);
    if (tcs.begin()) { tcsConnected = true; tcsChannel = i; break; }
  }

  // ToF
  struct { VL53L1X *s; bool *ok; uint8_t ch; } tofs[] = {
    {&tofL, &tofLok, CH_LEFT}, {&tofR, &tofRok, CH_RIGHT}, {&tofF, &tofFok, CH_FRONT}
  };
  for (auto &t : tofs) {
    tcaselect(t.ch); delay(10);
    t.s->setTimeout(100);
    if (t.s->init()) {
      t.s->setDistanceMode(VL53L1X::Medium);
      t.s->setMeasurementTimingBudget(33000);
      t.s->startContinuous(33);
      *t.ok = true;
    }
  }

  // IMU
  SPI_IMU.begin();
  if (myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU)) {
    delay(500);
    myIMU.enableGameRotationVector();
    delay(100);
    imuOk = true;
    // Zero the gyro while stationary. Rule 9.6 guarantees the car is placed
    // switched off, so this is firmware self-calibration, not team calibration
    // (rule 9.9 satisfied).
    uint32_t t0 = millis();
    while (millis() - t0 < 1500) {
      if (myIMU.getSensorEvent()) initialYawOffset = readYawRaw();
      delay(10);
    }
    gPrevHeading = 0.0f;
  }
}

void setup() {
  PiSerial.begin(115200);
  initHardware();
}

// ============================================================================
// LOOP — cooperative, non-blocking. Nothing here may delay().
// ============================================================================
uint32_t lastPid = 0, lastTelem = 0, lastSensor = 0;

void loop() {
  uint32_t now = millis();

  serviceLink();
  serviceEncoder();

  // ---- start button: latch on first press, report forever after ----
  if (!gButtonLatched && digitalRead(START_BTN_PIN) == LOW) {
    delay(0);                                   // no debounce needed: latched
    gButtonLatched = true;
  }

  // ---- IMU, as fast as it produces ----
  if (imuOk && myIMU.getSensorEvent()) {
    float h = wrapDeg(readYawRaw() - initialYawOffset);
    gYawRate = wrapDeg(h - gPrevHeading) * 1000.0f / max((uint32_t)1, now - lastSensor);
    gPrevHeading = h;
    gHeading = h;
  }

  // ---- slower sensors, 33 Hz ----
  if (now - lastSensor >= 30) {
    lastSensor = now;
    serviceOneToF(tofF, tofFok, CH_FRONT, tofFrontMm);
    serviceOneToF(tofL, tofLok, CH_LEFT,  tofLeftMm);
    serviceOneToF(tofR, tofRok, CH_RIGHT, tofRightMm);
    serviceFloor();
  }

  // ---- watchdog: the Pi went quiet -> stop. Safety never depends on the Pi.
  if (now - cmd.stamp > WATCHDOG_MS) {
    gWatchdogTripped = true;
    cmd.enable = false;
  }

  // ---- control, 200 Hz ----
  if (now - lastPid >= (1000 / SPEED_PID_HZ)) {
    float dt = (now - lastPid) / 1000.0f;
    lastPid = now;

    // measured speed from the encoder delta over this interval
    static long prevTicks = 0;
    long ticks = gEncTotal;
    if (dt > 0.0f) gSpeedMmps = (ticks - prevTicks) * MM_PER_TICK / dt;
    prevTicks = ticks;

    bool stopped = (!cmd.enable) || gWatchdogTripped || (cmd.mode == MODE_STOP);
    if (stopped) {
      setMotorDuty(0);
      setSteerDeg(0.0f);
      pidI = 0.0f; pidPrevErr = 0.0f;           // no windup while stopped
    } else {
      setSteerDeg(steeringForMode());
      setMotorDuty(speedControl(cmd.speedMmps, dt));
    }
  }

  // ---- telemetry, 50 Hz ----
  if (now - lastTelem >= (1000 / TELEM_HZ)) {
    lastTelem = now;
    sendTelemetry();
  }
}
