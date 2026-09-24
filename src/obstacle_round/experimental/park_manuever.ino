#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include <VL53L0X.h>

// ============================================================
// PARK OUT - standalone test  (same car, same pins as OpenRound.ino)
//
// PLACE THE CAR: parallel to the outer wall, pushed to the BACK of the lot
// (rear bumper at the rear block) and out toward the track side (60+ mm
// between its side and the wall). That gives the pivot the most room; a
// car centred in the lot usually aborts (safely) instead of getting out.
//
// One press of the PB12 button runs:
//   1. DECIDE   open side = the longer LiDAR side (median of 5 Pi frames).
//               The ToF + LiDAR readings also place the two parking blocks
//               and the wall in a small map around the car (the body model).
//   2. PIVOT    turn 90 deg toward the open side with full-lock shuffles:
//               FORWARD wheels toward the open side, REVERSE wheels away,
//               until the IMU says 90 deg.
//   3. EXIT     straight forward, IMU holding the 90 deg heading, until the
//               FRONT ToF reads EXIT_FRONT_MM (200) off the inner wall.
//   4. REALIGN  one full-lock REVERSE arc back to 0 deg (the start
//               heading). If a ToF or the body model stops it early, it
//               finishes with shuffles. Ends facing the original way.
// Turns are full lock all the way; the car slows in the last degrees
// instead, so it stops on the heading without overshooting.
//
// A shuffle leg ends at the FIRST of:
//   - front ToF (forward leg) / rear ToF (reverse leg) < PARK_SAFE_MM
//   - body model: encoder + IMU move the car's 210 x 114 rectangle through
//     the map; any part of it closing on a block within CORNER_MARGIN_MM or
//     on the wall within WALL_MARGIN_MM. This is what guards the CORNERS -
//     a straight-ahead ToF cannot see the front corner swing into the end
//     of a block (the host sim hit exactly that without it).
//   - LEG_CAP_CM
// Legs run on encoder SPEED (fast with room, 60 mm/s near a block), and
// both "near" tests add speed x BRAKE_S for the coast after the stop.
// The wheels only swing to the other lock once the car has stopped.
//
// Host sim (210 mm car, 1.5x lot, parked at the back, 65-80 mm off the
// wall, turning radius 220-320, slow/fast braking, both directions,
// LiDAR +-15 mm and encoder +-2% errors): no contact in any run, ~96% get
// out; the rest stop with an ABORT line.
//
// PI: run openRound.py (same wire as the open round: "left,front,right,rev").
//     Nothing happens until its first frame arrives.
// BUTTON (as OpenRound.ino): ready -> START; running -> PAUSE; paused ->
//        RESUME; done / aborted -> run again
// LED PC13 (as OpenRound.ino): slow blink (500 ms) = waiting for the Pi,
//        solid = ready / running / done, 250 ms blink = paused,
//        fast blink (100 ms) = Pi feed stale, very fast (60 ms) = aborted
// Serial log: '#' lines per leg: why it ended, ToF, body gap, and how far
// the car coasted after the stop (tune BRAKE_S from that).
// If a ToF is down (failed at boot or stops answering): FRONT -> the LiDAR
// front minus LIDAR_TO_FRONT_MM stands in; REAR -> reverse legs are blind,
// so each is capped at half a wheel turn (REAR_BLIND_CAP_CM). The log says
// "# front/rear ToF DOWN" when it switches.
// FIRST: run tof_pair_test.ino and set TOF_MIN_MM / PARK_SAFE_MM and the
// *_TOF_INSET_MM values; measure REAR_AXLE_MM and CAR_HALF_W_MM.
// ============================================================

// ---- pins (as OpenRound.ino) ----
const int MOT_RPWM_PIN = PA2, MOT_LPWM_PIN = PA3, SERVO_PIN = PA8;
const int IMU_CS_PIN = PA4, IMU_INT_PIN = PB0, IMU_RST_PIN = PB1;
const int STATUS_LED_PIN = PC13, BTN_PIN = PB12;
#define I2C_SCL     PB6
#define I2C_SDA     PB7
#define TCA_RST_PIN PB8
#define TCA_ADDR    0x70
const uint8_t FRONT_TOF_CH = 3;
const uint8_t REAR_TOF_CH  = 4;

// ---- calibration (as OpenRound.ino) ----
const float TICKS_PER_CM        = 14.853;
const int   SERVO_MIN_PULSE_US  = 500, SERVO_MAX_PULSE_US = 2500;
const float SERVO_TRUE_STRAIGHT = 76.5;   // obstacle firmware measured 79.5 on this car - check
const float SERVO_MAX_LEFT      = 20.0;   // below straight steers LEFT
const float SERVO_MAX_RIGHT     = 140.0;
const float IMU_YAW_SIGN        = 1.0;    // clockwise reads negative (left turn = +)

// ---- park-out tuning ----
const uint16_t PARK_SAFE_MM     = 25;     // front / rear ToF closer than this ends a leg. 50 does not
                                          // fit: the lot is 1.5 x 210 = 315 mm, only 105 mm spare in
                                          // total. The body model does the real guarding in the lot.
const uint16_t EXIT_FRONT_MM    = 200;    // EXIT: drive out until the FRONT ToF reads this (inner wall).
                                          // In a 1000 mm corridor that leaves ~590 mm behind the car -
                                          // room for the one reverse arc (it swings back ~1 turning radius)
const float    PARK_TOL_DEG     = 3.0;    // heading reached within this
const float    PARK_ACCEPT_DEG  = 8.0;    //   ... or within this if the legs get boxed in
const float    LEG_V_PER_DEG    = 15.0;   // near the target heading: speed <= this x degrees left
const float    LEG_CAP_CM       = 35.0;   // no single shuffle leg longer than this
const float    ARC_CAP_CM       = 70.0;   //   ... except the REALIGN reverse arc (90 deg at a ~260 mm
                                          //   turning radius is ~41 cm; 70 covers up to ~450 mm radius)
const float    EXIT_CAP_CM      = 80.0;   // EXIT backstop (front ToF sees nothing: VL53L0X reaches ~1.2 m)
const unsigned long SWING_MS    = 250;    // stopped while the wheels swing to the other lock
const float    STOPPED_MMPS     = 15.0;   // below this the car counts as stopped (before the swing)
const int      MAX_LEGS         = 24;     // per turn
const int      MAX_STUCK_LEGS   = 3;      // legs in a row that turned < 1 deg -> abort
const uint16_t OPEN_MIN_MM      = 400;    // the open side must read at least this
const uint16_t OPEN_MARGIN_MM   = 200;    //   and this much more than the other side
const uint16_t WALL_SIDE_MAX_MM = 350;    // one side has no LiDAR return: the other must read under
                                          // this (it is the wall) or over OPEN_MIN_MM (it is the track)
const int      DECIDE_REVS      = 5;      // LiDAR revolutions sampled to pick the open side
const unsigned long TOF_STALE_MS = 150;   // no new ToF sample this long = blind
// shuffle legs run on SPEED (encoder), not a fixed PWM, so they stop the same on any battery:
const float    LEG_VMIN_MMPS    = 60.0;   // speed right next to a wall
const float    LEG_VMAX_MMPS    = 180.0;  // speed with room
const float    LEG_V_PER_MM     = 3.0;    // target speed = this x mm of room left (clamped to the above)
const int      LEG_PWM_START    = 35;     // PWM a leg starts from (just under what moves the car)
const int      LEG_PWM_MAX      = 110;
const float    LEG_KI           = 0.04;   // PWM added per 20 ms per mm/s of speed error
const float    BRAKE_S          = 0.15;   // stop early by speed x this (coast + ToF sample age), seconds.
                                          // Only the START value: after every stop the car measures how far
                                          // it really coasted and learns it (gBrakeS, up to BRAKE_MAX_S)
const float    BRAKE_MAX_S      = 0.40;
const unsigned long IMU_DEAD_MS = 300;    // no heading update this long = IMU dead: never move
float gBrakeS = BRAKE_S;                  // learned: seconds of coast per mm/s at the stop

// ---- car + lot geometry (the body model that guards the CORNERS the ToFs cannot see) ----
const float CAR_LEN_MM        = 210.0;    // bumper to bumper
const float CAR_HALF_W_MM     = 57.0;     // half the widest part of the body
const float REAR_AXLE_MM      = 60.0;     // rear bumper to the rear axle (the car pivots about it)
const float FRONT_TOF_INSET_MM = 0.0;     // how far each ToF sits behind its bumper
const float REAR_TOF_INSET_MM  = 0.0;
const float LOT_DEPTH_MM      = 200.0;    // parking blocks stick out this far from the outer wall
const float BLOCK_THICK_MM    = 20.0;
const float CORNER_MARGIN_MM  = 10.0;     // body model: stop a leg this close to a block
const float WALL_MARGIN_MM    = 20.0;     //   ... or this close to the outer wall (placed by the LiDAR,
                                          //   which is only good to ~+-15 mm - the blocks use the ToFs)
const uint16_t TOF_MIN_MM     = 40;       // ToF readings below this are not trusted: the model treats
                                          // that block as touching the bumper (tof_pair_test finds it)

// ---- fallbacks when a ToF is down (failed at boot, or stops answering mid-run) ----
const float LIDAR_TO_FRONT_MM   = 105.0;  // FRONT ToF down -> front = LiDAR front - this. MEASURE it:
                                          // LiDAR centre to the front bumper. Too SMALL = the car thinks
                                          // it has more room than it has (105 = LiDAR mid-car, 210 car)
const float LIDAR_EXTRA_BRAKE_S = 0.10;   // the LiDAR front is 10 Hz: up to 100 ms old -> stop earlier
const float TICKS_PER_WHEEL_REV = 240.0;  // encoder ticks per full wheel turn (measured)
const float REAR_BLIND_CAP_CM   = 0.5f * TICKS_PER_WHEEL_REV / TICKS_PER_CM;   // REAR ToF down ->
                                          // every reverse leg is at most HALF a wheel turn (120 ticks = 8.1 cm)

SPIClass SPI_IMU(PA7, PA6, PA5);
Servo    steeringServo;
BNO08x   myIMU;
VL53L0X  tofFront, tofRear;
float    initialYawOffset = 0.0;
float    startHeading = 0.0, outHeading = 0.0;
int      openRot = 0;                    // +1 open on the left, -1 on the right
// (declared up here so the Arduino IDE's auto-generated prototypes can see it)
enum ParkState { P_WAIT_PI, P_READY, P_DECIDE, P_PIVOT, P_EXIT, P_REALIGN, P_PAUSED, P_DONE, P_ABORT };
ParkState st = P_WAIT_PI;

float wrapDeg(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

// ---------------- motor / servo / encoder ----------------
void setMotorSpeed(int s) {
  s = constrain(s, -255, 255);
  if (s > 0)      { analogWrite(MOT_RPWM_PIN, s); analogWrite(MOT_LPWM_PIN, 0); }
  else if (s < 0) { analogWrite(MOT_RPWM_PIN, 0); analogWrite(MOT_LPWM_PIN, -s); }
  else            { analogWrite(MOT_RPWM_PIN, 0); analogWrite(MOT_LPWM_PIN, 0); }
}
void setServoAngle(float a) {
  a = constrain(a, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
  steeringServo.writeMicroseconds((int)((a / 180.0) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) + SERVO_MIN_PULSE_US);
}
long readEncoder() { return -(int32_t)TIM5->CNT; }
long absEnc(long v) { return v < 0 ? -v : v; }

// car speed from the encoder, mm/s (absolute), over the last ~40 ms
float gSpeedMmps = 0; long spdTicks0 = 0; unsigned long spdMs0 = 0;
void serviceSpeed() {
  unsigned long now = millis();
  if (now - spdMs0 < 40) return;
  long t = readEncoder();
  gSpeedMmps = absEnc(t - spdTicks0) / TICKS_PER_CM * 10.0f * 1000.0f / (now - spdMs0);
  spdTicks0 = t; spdMs0 = now;
}

// ---------------- status LED + button ----------------
void statusLed(bool on) { digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH); }
void statusBlink(unsigned long p) { statusLed((millis() / p) & 1); }

bool btnLastRaw = LOW, btnStable = LOW, btnFired = false;   // must be seen released first
unsigned long btnChangeMs = 0, btnLastPress = 0;
bool buttonPressed() {
  bool raw = digitalRead(BTN_PIN);
  if (raw != btnLastRaw) { btnLastRaw = raw; btnChangeMs = millis(); }
  if (raw != btnStable && millis() - btnChangeMs >= 30) {
    btnStable = raw;
    if (btnStable == LOW && (!btnFired || millis() - btnLastPress >= 400)) {
      btnFired = true; btnLastPress = millis();
      return true;
    }
  }
  return false;
}

// ---------------- LiDAR lines from the Pi (openRound.py) ----------------
// Run openRound.py on the Pi. It sends, 50 times a second:
//     left,front,right,rev\n        e.g. "412,1873,655,1234\n"
//   mm at 90 / 0 / 270 deg; 65535 = no valid return = NO EVIDENCE (never
//   "open"); rev = LiDAR revolution counter - each bearing gets one new
//   sample per rev (~10 Hz), the other frames resend it. The Pi sends
//   nothing until its LiDAR is really scanning (the startup gate below),
//   and it prints our '#' lines on its console.
// Left and right pick the open side at the start; front stands in for the
// front ToF if that one is down.
const uint16_t LIDAR_FAR = 9999;          // internal: no valid return / out of range
uint16_t lidarL = LIDAR_FAR, lidarF = LIDAR_FAR, lidarR = LIDAR_FAR;
long     lidarRev = -1;
unsigned long lidarLastMs = 0;
uint32_t lidarFrames = 0;
bool lidarNewRev = false;                 // this pass brought a new revolution
char lidarBuf[48]; uint8_t lidarLen = 0;
uint16_t lSan(long v) { return (v <= 0 || v > 3500) ? LIDAR_FAR : (uint16_t)v; }   // 65535 -> FAR

void serviceLidar() {
  lidarNewRev = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lidarLen) {
        lidarBuf[lidarLen] = '\0';
        char *c1 = strchr(lidarBuf, ',');
        char *c2 = c1 ? strchr(c1 + 1, ',') : NULL;
        char *c3 = c2 ? strchr(c2 + 1, ',') : NULL;
        if (c1 && c2) {
          lidarL = lSan(atol(lidarBuf));
          lidarF = lSan(atol(c1 + 1));
          lidarR = lSan(atol(c2 + 1));
          long rev = c3 ? atol(c3 + 1) : lidarRev + 1;   // no rev field: every line is new
          if (rev != lidarRev) { lidarRev = rev; lidarNewRev = true; }
          lidarLastMs = millis(); lidarFrames++;
        }
        lidarLen = 0;
      }
    } else if (lidarLen < sizeof(lidarBuf) - 1) lidarBuf[lidarLen++] = c;
    else lidarLen = 0;
  }
}
bool lidarStale() { return lidarFrames == 0 || millis() - lidarLastMs > 200; }    // LED hint only
bool lidarDead()  { return lidarFrames == 0 || millis() - lidarLastMs > 1000; }   // as OpenRound.ino

// ---------------- ToF, non-blocking ----------------
void tcaselect(uint8_t ch) { Wire.beginTransmission(TCA_ADDR); Wire.write(1 << ch); Wire.endTransmission(); }
void resetTCA() { pinMode(TCA_RST_PIN, OUTPUT); digitalWrite(TCA_RST_PIN, LOW); delay(10); digitalWrite(TCA_RST_PIN, HIGH); delay(10); }

const uint16_t TOF_FAR = 9999;
uint16_t frontMm = TOF_FAR, rearMm = TOF_FAR;
unsigned long frontMs = 0, rearMs = 0;
bool frontOk = false, rearOk = false;

bool initTof(VL53L0X &s, uint8_t ch) {
  tcaselect(ch);
  s.setBus(&Wire);
  s.setTimeout(100);
  if (!s.init()) return false;
  s.setMeasurementTimingBudget(30000);           // ~33 Hz, as the team tof_test.ino (20 ms read FAR on the front)
  s.startContinuous();
  return true;
}
void pollTof(VL53L0X &s, uint8_t ch, uint16_t &mm, unsigned long &t) {
  tcaselect(ch);
  if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;   // not ready yet
  uint16_t v = s.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  s.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  mm = (v == 0 || v >= 8190) ? TOF_FAR : v;       // 8190/8191 = no target
  t = millis();
}
bool frontBlind() { return !frontOk || millis() - frontMs > TOF_STALE_MS; }
bool rearBlind()  { return !rearOk  || millis() - rearMs  > TOF_STALE_MS; }

// Front distance, bumper to obstacle: the front ToF, else the LiDAR front.
// false only when BOTH are down. brakeS = how early to stop at the current speed.
bool frontDist(uint16_t &mm, float &brakeS, bool &fromLidar) {
  brakeS = gBrakeS; fromLidar = false;
  if (!frontBlind()) { mm = frontMm; return true; }
  if (lidarStale())  { mm = TOF_FAR; return false; }
  fromLidar = true; brakeS = gBrakeS + LIDAR_EXTRA_BRAKE_S;
  mm = lidarF >= LIDAR_FAR ? TOF_FAR : (uint16_t)fmaxf(0.0f, lidarF - LIDAR_TO_FRONT_MM);
  return true;
}
bool frontDist(uint16_t &mm, float &brakeS) { bool l; return frontDist(mm, brakeS, l); }

// say so in the log whenever a ToF drops out or comes back
int8_t frontMode = -1, rearMode = -1;
void noteSensorModes() {
  int8_t f = frontBlind() ? 1 : 0, r = rearBlind() ? 1 : 0;
  if (f != frontMode) {
    frontMode = f;
    if (f) { Serial.print(F("# front ToF DOWN - using the LiDAR front minus ")); Serial.print(LIDAR_TO_FRONT_MM, 0); Serial.println(F(" mm")); }
    else Serial.println(F("# front ToF ok"));
  }
  if (r != rearMode) {
    rearMode = r;
    if (r) { Serial.print(F("# rear ToF DOWN - every reverse leg capped at ")); Serial.print(REAR_BLIND_CAP_CM, 1); Serial.println(F(" cm (half a wheel turn)")); }
    else Serial.println(F("# rear ToF ok"));
  }
}

// ---------------- IMU ----------------
bool  gImuFresh = false;
bool  imuOk = false;                      // found at boot
unsigned long gImuLastMs = 0;             // last heading update
bool  imuDead() { return !imuOk || millis() - gImuLastMs > IMU_DEAD_MS; }
float gHeading = 0.0;
float readYaw() {
  float qI = myIMU.getQuatI(), qJ = myIMU.getQuatJ(), qK = myIMU.getQuatK(), qR = myIMU.getQuatReal();
  if (qI == 0 && qJ == 0 && qK == 0 && qR == 0) return 0;
  return atan2(2.0f * (qI * qJ + qR * qK), (qR * qR + qI * qI - qJ * qJ - qK * qK)) * (180.0 / PI);
}
void serviceImu() {
  gImuFresh = false;
  if (myIMU.wasReset()) myIMU.enableGameRotationVector();
  if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    gImuFresh = true; gImuLastMs = millis();
    gHeading = IMU_YAW_SIGN * (fmod(readYaw() - initialYawOffset + 540.0, 360.0) - 180.0);
  }
}

// leg speed controller: target speed from the room left, PWM integrates the error
float legPwm = 0; unsigned long legPwmMs = 0;
int legSpeedPwm(float room, float degToGo = 1e6f) {   // room = mm left before a stop
  float vt = fminf(room * LEG_V_PER_MM, degToGo * LEG_V_PER_DEG);   // slow near a wall / the heading
  vt = constrain(vt, LEG_VMIN_MMPS, LEG_VMAX_MMPS);
  if (millis() - legPwmMs >= 20) {
    legPwmMs = millis();
    legPwm = constrain(legPwm + LEG_KI * (vt - gSpeedMmps), 0.0f, (float)LEG_PWM_MAX);
  }
  return (int)legPwm;
}

// ---------------- body model ----------------
// Frame fixed at DECIDE: x along the start heading, y toward the OPEN side,
// origin = rear axle at the start.  The encoder + IMU move the car's
// rectangle in it; the two parking blocks and the outer wall were placed
// from the ToF / LiDAR readings taken while the car was still straight.
float px = 0, py = 0;                       // rear axle
long  poseTicks = 0;
bool  modelOn = false;
float faceF = 1e6, faceR = -1e6, wallY = -1e6, blockEndY = -1e6;

float carPsi() { return openRot * wrapDeg(gHeading - startHeading) * (PI / 180.0f); }   // + = toward open side
void servicePose() {
  long t = readEncoder();
  float ds = (t - poseTicks) / TICKS_PER_CM * 10.0f;   // mm, + forward
  poseTicks = t;
  if (!modelOn) return;
  float psi = carPsi();
  px += ds * cosf(psi); py += ds * sinf(psi);
}
float rectDist(float x, float y, float x0, float x1, float y0, float y1) {   // <0 inside
  if (x > x0 && x < x1 && y > y0 && y < y1)                     // inside: minus the depth
    return -fminf(fminf(x - x0, x1 - x), fminf(y - y0, y1 - y));
  float dx = fmaxf(fmaxf(x0 - x, 0), x - x1), dy = fmaxf(fmaxf(y0 - y, 0), y - y1);
  return sqrtf(dx * dx + dy * dy);
}
// gap between the body outline and each obstacle, mm (<0 = touching):
// g[0] outer wall (already less its extra margin), g[1] front block, g[2] rear block
const int NOBS = 3;
void bodyGaps(float *g) {
  for (int i = 0; i < NOBS; i++) g[i] = 1e6;
  if (!modelOn) return;
  float psi = carPsi(), c = cosf(psi), sn = sinf(psi);
  const float xf = CAR_LEN_MM - REAR_AXLE_MM, xr = -REAR_AXLE_MM, w = CAR_HALF_W_MM;
  for (int e = 0; e < 4; e++) for (int k = 0; k <= 12; k++) {
    float u = k / 12.0f, bx, by;
    if (e < 2) { bx = xr + u * (xf - xr); by = e ? w : -w; }
    else       { bx = e == 2 ? xf : xr;   by = -w + u * 2 * w; }
    float x = px + bx * c - by * sn, y = py + bx * sn + by * c;
    g[0] = fminf(g[0], y - wallY - (WALL_MARGIN_MM - CORNER_MARGIN_MM));   // same stop test, wider margin
    g[1] = fminf(g[1], rectDist(x, y, faceF, faceF + BLOCK_THICK_MM, wallY, blockEndY));
    g[2] = fminf(g[2], rectDist(x, y, faceR - BLOCK_THICK_MM, faceR, wallY, blockEndY));
  }
}
float bodyClearance() { float g[NOBS]; bodyGaps(g); return fminf(g[0], fminf(g[1], g[2])); }
void placeModel(uint16_t closedSideMm, uint16_t frontMm, uint16_t rearMm, bool frontFromLidar) {
  px = py = 0; poseTicks = readEncoder(); modelOn = true;
  float fGap = frontFromLidar ? (float)frontMm                            // LiDAR: already bumper-based
             : frontMm < TOF_MIN_MM ? 0.0f : frontMm - FRONT_TOF_INSET_MM;  // bumper to block
  float rGap = rearMm  < TOF_MIN_MM ? 0.0f : rearMm  - REAR_TOF_INSET_MM;
  faceF = frontMm < TOF_FAR ? (CAR_LEN_MM - REAR_AXLE_MM) + fmaxf(fGap, 0.0f) : 1e6;
  faceR = rearMm  < TOF_FAR ? -REAR_AXLE_MM - fmaxf(rGap, 0.0f) : -1e6;
  wallY = -(float)closedSideMm;               // the LiDAR sits on the centre line
  blockEndY = wallY + LOT_DEPTH_MM;
  Serial.print(F("# model: front block ")); Serial.print(faceF - (CAR_LEN_MM - REAR_AXLE_MM), 0);
  Serial.print(F(" mm ahead, rear block ")); Serial.print(-faceR - REAR_AXLE_MM, 0);
  Serial.print(F(" mm behind, wall ")); Serial.print(closedSideMm - CAR_HALF_W_MM, 0);
  Serial.println(F(" mm from the side"));
}

// ---------------- the multi-point turn ----------------
// Rotates to `target` in the sense `rot` (+1 = left / CCW, -1 = right).
// Forward legs steer TOWARD the turn, reverse legs steer AWAY from it:
// reversing with the wheels right swings the nose left, so both kinds of
// leg keep rotating the car the same way. A forward leg ends at the front
// ToF, a reverse leg at the rear ToF, any leg at LEG_CAP_CM.
enum LegPhase { LEG_MOVE, LEG_SETTLE, LEG_SWING };
struct Maneuver {
  const char *name;
  float target; int rot; int dir;       // dir: +1 forward leg, -1 reverse leg
  LegPhase phase; unsigned long t0;
  long legTicks0, stopTicks; float legHeading0, stopSpeed;
  int legs, stuck;
  float gLast[NOBS], gTrend[NOBS]; unsigned long gMs;
  float firstCapCm;                     // cap for leg 1 (the REALIGN arc), LEG_CAP_CM after
} mv;

float fullLock(bool left) { return left ? SERVO_TRUE_STRAIGHT - SERVO_MAX_LEFT : SERVO_MAX_RIGHT - SERVO_TRUE_STRAIGHT; }
bool legSteersLeft() { return (mv.rot > 0) == (mv.dir > 0); }        // fwd+left or rev+right turn
float degLeft() { return mv.rot * wrapDeg(mv.target - gHeading); }   // still to rotate, + = not there

// Stop, wait until the car has really stopped (swinging the wheels while it
// still rolls steers the nose into the block), then put them on the next lock.
void legSwing() {
  setMotorSpeed(0);
  mv.phase = LEG_SETTLE; mv.t0 = millis(); mv.stopTicks = readEncoder(); mv.stopSpeed = gSpeedMmps;
}
void legLock() {
  bool left = legSteersLeft();
  setServoAngle(left ? SERVO_TRUE_STRAIGHT - fullLock(true) : SERVO_TRUE_STRAIGHT + fullLock(false));
  mv.phase = LEG_SWING; mv.t0 = millis();
}

void startManeuver(const char *name, float target, int rot, int firstDir, float firstCapCm = LEG_CAP_CM) {
  mv.name = name; mv.target = wrapDeg(target); mv.rot = rot; mv.dir = firstDir; mv.firstCapCm = firstCapCm;
  mv.legs = 0; mv.stuck = 0;
  Serial.print(F("# ")); Serial.print(name); Serial.print(F(": turn "));
  Serial.print(rot > 0 ? F("LEFT") : F("RIGHT")); Serial.print(F(" to heading ")); Serial.print(mv.target, 1);
  Serial.print(F(" (now ")); Serial.print(gHeading, 1); Serial.print(F("), first leg "));
  Serial.println(firstDir > 0 ? F("FORWARD") : F("REVERSE"));
  legSwing();
}

// 0 running, 1 done, -1 failed
int maneuverStep() {
  float left = degLeft();
  if (left <= PARK_TOL_DEG) {
    setMotorSpeed(0);
    Serial.print(F("# ")); Serial.print(mv.name); Serial.print(F(" done in ")); Serial.print(mv.legs);
    Serial.print(F(" legs, heading ")); Serial.println(gHeading, 1);
    return 1;
  }
  if (mv.phase == LEG_SETTLE) {
    if (gSpeedMmps > STOPPED_MMPS && millis() - mv.t0 < 600) return 0;
    if (mv.legs > 0) {                                   // learn how the car coasts
      float coastMm = absEnc(readEncoder() - mv.stopTicks) / TICKS_PER_CM * 10.0f;
      if (mv.stopSpeed > 40.0f) {
        float t = coastMm / mv.stopSpeed;                // seconds of coast per mm/s
        gBrakeS = constrain(t > gBrakeS ? t : 0.7f * gBrakeS + 0.3f * t, BRAKE_S, BRAKE_MAX_S);   // up at once, down slowly
      }
      Serial.print(F("#   coasted ")); Serial.print(coastMm, 0); Serial.print(F(" mm from "));
      Serial.print(mv.stopSpeed, 0); Serial.print(F(" mm/s -> brake allowance ")); Serial.print(gBrakeS, 2); Serial.println(F(" s"));
    }
    legLock();
    return 0;
  }
  if (mv.phase == LEG_SWING) {
    if (millis() - mv.t0 < SWING_MS) return 0;
    mv.phase = LEG_MOVE; mv.legTicks0 = readEncoder(); mv.legHeading0 = gHeading;
    bodyGaps(mv.gLast); for (int i = 0; i < NOBS; i++) mv.gTrend[i] = 0; mv.gMs = millis();
    legPwm = LEG_PWM_START; legPwmMs = millis();
    mv.legs++;
  }

  // ---- LEG_MOVE ----
  bool fwd = mv.dir > 0;
  bool blind = false, capBlind = false;
  uint16_t guard = TOF_FAR; float brakeS = gBrakeS;
  float cap = (mv.legs == 1 ? mv.firstCapCm : LEG_CAP_CM);
  if (fwd)               blind = !frontDist(guard, brakeS);      // front ToF, else the LiDAR front
  else if (!rearBlind()) guard = rearMm;
  else { cap = fminf(cap, REAR_BLIND_CAP_CM); capBlind = true; } // no rear eyes: half a wheel turn
  float legCm = absEnc(readEncoder() - mv.legTicks0) / TICKS_PER_CM;
  // body model: the nearest obstacle this leg is moving TOWARD (each tracked on its own -
  // a block corner can close in fast while the wall, still nearer, is opening up)
  float g[NOBS]; bodyGaps(g);
  bool fresh = millis() - mv.gMs >= 20;
  float clr = 1e6;                                   // gap to the nearest closing obstacle
  for (int i = 0; i < NOBS; i++) {
    if (fresh) { mv.gTrend[i] = g[i] - mv.gLast[i]; mv.gLast[i] = g[i]; }
    if (mv.gTrend[i] < -0.2f) clr = fminf(clr, g[i]);
  }
  if (fresh) mv.gMs = millis();
  bool closing = clr < 1e5f;
  const __FlashStringHelper *why = NULL;
  if (blind)                     why = F("front ToF AND LiDAR front both down");
  else if (guard < PARK_SAFE_MM + gSpeedMmps * brakeS) why = fwd ? F("front distance") : F("rear ToF");
  else if (closing && clr < CORNER_MARGIN_MM + 1.4f * gSpeedMmps * gBrakeS) why = F("body model (corner)");
  else if (legCm >= cap)         why = capBlind ? F("half-wheel cap (rear ToF down)") : F("leg cap");

  if (why) {
    setMotorSpeed(0);
    float turned = mv.rot * wrapDeg(gHeading - mv.legHeading0);
    Serial.print(F("# ")); Serial.print(mv.name); Serial.print(F(" leg ")); Serial.print(mv.legs);
    Serial.print(fwd ? F(" FWD ") : F(" REV ")); Serial.print(legCm, 1); Serial.print(F(" cm, turned "));
    Serial.print(turned, 1); Serial.print(F(" deg, heading ")); Serial.print(gHeading, 1);
    Serial.print(F(", ended: ")); Serial.print(why);
    if (!blind && guard < TOF_FAR) { Serial.print(fwd ? F(", front ") : F(", rear ")); Serial.print(guard); Serial.print(F(" mm")); }
    Serial.print(F(", body gap ")); Serial.print(fminf(g[0], fminf(g[1], g[2])), 0); Serial.print(F(" mm"));
    Serial.println();
    if (blind) return -1;                                  // cannot see the wall: never move on blind
    mv.stuck = (turned < 1.0f) ? mv.stuck + 1 : 0;
    if (mv.stuck >= MAX_STUCK_LEGS && degLeft() <= PARK_ACCEPT_DEG) {
      Serial.print(F("# ")); Serial.print(mv.name); Serial.print(F(" boxed in but only "));
      Serial.print(degLeft(), 1); Serial.println(F(" deg off - accepted"));
      return 1;
    }
    if (mv.stuck >= MAX_STUCK_LEGS) { Serial.println(F("# ABORT: boxed in - legs are not turning the car")); return -1; }
    if (mv.legs >= MAX_LEGS)        { Serial.println(F("# ABORT: too many legs")); return -1; }
    mv.dir = -mv.dir;
    legSwing();
    return 0;
  }

  // steer: FULL lock the whole way - forward toward the turn, reverse away from it
  setServoAngle(legSteersLeft() ? SERVO_TRUE_STRAIGHT - fullLock(true) : SERVO_TRUE_STRAIGHT + fullLock(false));
  float room = fminf((float)guard - PARK_SAFE_MM, closing ? clr - CORNER_MARGIN_MM : 1e6f);
  setMotorSpeed(mv.dir * legSpeedPwm(room, left - PARK_TOL_DEG));
  return 0;
}

// ---------------- the park-out FSM ----------------
uint16_t medL[DECIDE_REVS], medR[DECIDE_REVS]; uint8_t medN = 0;
long  exitTicks0 = 0;
ParkState pausedFrom = P_READY;

// median of the VALID samples (LIDAR_FAR skipped); LIDAR_FAR if fewer than half are valid
uint16_t medianValid(uint16_t *v, int n) {
  uint16_t a[DECIDE_REVS]; int k = 0;
  for (int i = 0; i < n; i++) if (v[i] < LIDAR_FAR) a[k++] = v[i];
  if (k * 2 <= n) return LIDAR_FAR;
  for (int i = 1; i < k; i++) for (int j = i; j > 0 && a[j] < a[j - 1]; j--) { uint16_t t = a[j]; a[j] = a[j - 1]; a[j - 1] = t; }
  return a[k / 2];
}

void go(ParkState s) { st = s; }

void stopCar(const __FlashStringHelper *why, ParkState next) {
  setMotorSpeed(0);
  setServoAngle(SERVO_TRUE_STRAIGHT);
  Serial.print(F("# ")); Serial.println(why);
  go(next);
}

void setup() {
  Serial.begin(115200);
  pinMode(MOT_RPWM_PIN, OUTPUT); pinMode(MOT_LPWM_PIN, OUTPUT); setMotorSpeed(0);
  pinMode(STATUS_LED_PIN, OUTPUT); statusLed(false);
  pinMode(BTN_PIN, INPUT_PULLUP);
  steeringServo.attach(SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  setServoAngle(SERVO_TRUE_STRAIGHT);

  // TIM5 encoder on PA0 / PA1
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_TIM5_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_0 | GPIO_PIN_1; g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH; g.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &g);
  TIM_Encoder_InitTypeDef enc = {0};
  static TIM_HandleTypeDef htim5 = {0};
  htim5.Instance = TIM5; htim5.Init.Prescaler = 0; htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  enc.EncoderMode = TIM_ENCODERMODE_TI12;
  enc.IC1Polarity = TIM_ICPOLARITY_RISING; enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  enc.IC2Polarity = TIM_ICPOLARITY_RISING; enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim5, &enc);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  // I2C: mux + both ToF
  resetTCA();
  Wire.setSCL(I2C_SCL); Wire.setSDA(I2C_SDA); Wire.begin(); Wire.setClock(400000);
  delay(100);
  frontOk = initTof(tofFront, FRONT_TOF_CH);
  rearOk  = initTof(tofRear,  REAR_TOF_CH);
  Serial.print(F("# ToF front (CH3) ")); Serial.println(frontOk ? F("READY") : F("FAILED"));
  Serial.print(F("# ToF rear  (CH4) ")); Serial.println(rearOk  ? F("READY") : F("FAILED"));

  // IMU
  SPI_IMU.begin();
  if (myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU)) {
    delay(500);
    myIMU.enableGameRotationVector();
    delay(100);
    unsigned long t = millis();
    while (millis() - t < 3000) {
      if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
        initialYawOffset = readYaw(); break;
      }
      delay(10);
    }
    imuOk = true; gImuLastMs = millis();
    Serial.println(F("# IMU ready"));
  } else {
    Serial.println(F("# ERROR IMU not found - the car will NOT move (no heading)"));
  }
}

void loopBody();

// every 2 s while idle: is everything alive before you press?
unsigned long idleMs = 0;
void idleStatus() {
  if (millis() - idleMs < 2000) return;
  idleMs = millis();
  Serial.print(F("# idle: heading ")); Serial.print(gHeading, 1);
  Serial.print(imuDead() ? F(" (IMU DEAD)") : F(" (IMU ok)"));
  Serial.print(F("  ToF F=")); if (frontBlind()) Serial.print(F("--")); else Serial.print(frontMm);
  Serial.print(F(" R="));      if (rearBlind())  Serial.print(F("--")); else Serial.print(rearMm);
  Serial.print(F("  LiDAR L=")); Serial.print(lidarL); Serial.print(F(" F=")); Serial.print(lidarF);
  Serial.print(F(" R=")); Serial.println(lidarR);
}

// ---- loop timing: something blocking the loop (an I2C / SPI call stuck on
// its timeout) starves the Pi's serial feed. Report it, once a second.
const char *slowNames[] = { "LiDAR serial", "IMU (SPI)", "front ToF (I2C)", "rear ToF (I2C)", "rest of loop" };
unsigned long slowMax[5], loopMaxUs = 0, slowReportMs = 0, loopStartUs = 0;
inline void timeIt(int i, unsigned long t0) { unsigned long d = micros() - t0; if (d > slowMax[i]) slowMax[i] = d; }
void reportSlowLoop() {
  unsigned long d = micros() - loopStartUs;
  if (d > loopMaxUs) loopMaxUs = d;
  if (millis() - slowReportMs < 1000) return;
  slowReportMs = millis();
  if (loopMaxUs > 50000UL) {                      // a loop pass over 50 ms in the last second
    int w = 0; for (int i = 1; i < 5; i++) if (slowMax[i] > slowMax[w]) w = i;
    Serial.print(F("# SLOW LOOP: ")); Serial.print(loopMaxUs / 1000); Serial.print(F(" ms max, worst: "));
    Serial.print(slowNames[w]); Serial.print(' '); Serial.print(slowMax[w] / 1000); Serial.println(F(" ms"));
  }
  loopMaxUs = 0; for (int i = 0; i < 5; i++) slowMax[i] = 0;
}

void loop() {
  loopStartUs = micros();
  unsigned long t0 = micros(); serviceLidar(); timeIt(0, t0);
  t0 = micros(); serviceImu(); timeIt(1, t0);
  serviceSpeed();
  servicePose();
  t0 = micros(); if (frontOk) pollTof(tofFront, FRONT_TOF_CH, frontMm, frontMs); timeIt(2, t0);
  t0 = micros(); if (rearOk)  pollTof(tofRear,  REAR_TOF_CH,  rearMm,  rearMs);  timeIt(3, t0);
  t0 = micros();
  loopBody();
  timeIt(4, t0);
  reportSlowLoop();
}

void loopBody() {

  // ---- startup gate (as OpenRound.ino): nothing runs until the Pi's first frame ----
  if (st == P_WAIT_PI) {
    statusBlink(500);                                  // slow blink: waiting for the Pi
    if (lidarFrames > 0) {
      Serial.println(F("# first LiDAR frame received - press the button to park out"));
      go(P_READY);
    }
    return;
  }

  noteSensorModes();

  // ---- button (as OpenRound.ino): start / pause / resume / run again ----
  bool press = buttonPressed();
  bool moving = (st == P_DECIDE || st == P_PIVOT || st == P_EXIT || st == P_REALIGN);
  if (press) {
    if (st == P_PAUSED) {
      press = false;
      Serial.println(F("# RESUME (button)"));
      if (pausedFrom == P_PIVOT || pausedFrom == P_REALIGN) legSwing();   // re-lock, then carry on
      if (pausedFrom == P_DECIDE) medN = 0;
      go(pausedFrom);
    } else if (moving) {
      press = false;
      setMotorSpeed(0);
      Serial.println(F("# PAUSED (button) - press again to resume"));
      pausedFrom = st;
      go(P_PAUSED);
    }                                                  // READY / DONE / ABORT: the switch starts a run
  }

  // ---- no heading = no idea how far the car has turned: stop ----
  if ((st == P_DECIDE || st == P_PIVOT || st == P_EXIT || st == P_REALIGN) && imuDead()) {
    stopCar(F("ABORT: IMU stopped giving heading"), P_ABORT);
  }

  // ---- LED (as OpenRound.ino) ----
  if (st == P_DONE)          statusLed(true);          // solid: done
  else if (st == P_ABORT)    statusBlink(60);          // very fast: aborted (press = try again)
  else if (st == P_PAUSED)   statusBlink(250);         // paused
  else if (lidarStale())     statusBlink(100);         // Pi feed stale
  else                       statusLed(true);          // ready / running

  switch (st) {
    case P_WAIT_PI:
      break;

    case P_PAUSED:
      setMotorSpeed(0);
      break;

    case P_READY:
    case P_DONE:
    case P_ABORT:
      idleStatus();
      if (press) {
        if (imuDead()) { Serial.println(F("# NOT starting: IMU has no heading (see '# IMU ready' at boot)")); go(P_ABORT); break; }
        medN = 0; Serial.println(F("# PARK OUT start")); go(P_DECIDE);
      }
      break;

    case P_DECIDE:                                   // 1. which side is open?
      if (lidarDead()) {                             // 1 s of silence - short gaps just wait
        Serial.print(F("# no LiDAR line for ")); Serial.print(millis() - lidarLastMs);
        Serial.print(F(" ms (")); Serial.print(lidarFrames); Serial.println(F(" lines so far)"));
        stopCar(F("ABORT: no LiDAR frames from the Pi (is openRound.py running?)"), P_ABORT); break;
      }
      if (!lidarNewRev) break;                       // one sample per LiDAR revolution
      medL[medN] = lidarL; medR[medN] = lidarR;
      if (++medN < DECIDE_REVS) break;
      {
        uint16_t L = medianValid(medL, DECIDE_REVS), R = medianValid(medR, DECIDE_REVS);
        Serial.print(F("# sides L=")); if (L < LIDAR_FAR) Serial.print(L); else Serial.print(F("none"));
        Serial.print(F(" R="));        if (R < LIDAR_FAR) Serial.print(R); else Serial.print(F("none"));
        Serial.print(F("  ToF front=")); Serial.print(frontMm); Serial.print(F(" rear=")); Serial.println(rearMm);
        uint16_t lo;                                 // distance to the wall side (for the body model)
        if (L < LIDAR_FAR && R < LIDAR_FAR) {        // both sides seen: the longer one is open
          uint16_t hi = max(L, R); lo = min(L, R);
          if (hi < OPEN_MIN_MM || hi - lo < OPEN_MARGIN_MM) {
            stopCar(F("ABORT: cannot tell which side is open (both sides too alike)"), P_ABORT); break;
          }
          openRot = (L > R) ? 1 : -1;
        } else if (L < LIDAR_FAR || R < LIDAR_FAR) { // one side has no return: no evidence, so the
          uint16_t seen = (L < LIDAR_FAR) ? L : R;   // side that WAS seen has to settle it
          int seenRot   = (L < LIDAR_FAR) ? 1 : -1;
          if (seen <= WALL_SIDE_MAX_MM)      { openRot = -seenRot; lo = seen; }   // seen = the wall
          else if (seen >= OPEN_MIN_MM)      { openRot =  seenRot;                 // seen = the track;
            lo = (uint16_t)(CAR_HALF_W_MM + 40);                                  // wall distance unknown:
            Serial.println(F("# wall side has no LiDAR return - assuming the car is 40 mm off the wall"));
          } else { stopCar(F("ABORT: one LiDAR side has no return and the other is ambiguous"), P_ABORT); break; }
        } else {
          stopCar(F("ABORT: no LiDAR return on either side"), P_ABORT); break;
        }
        uint16_t fMm; float bS; bool fLidar;
        if (!frontDist(fMm, bS, fLidar)) { stopCar(F("ABORT: front ToF and LiDAR front both down"), P_ABORT); break; }
        uint16_t rMm = rearBlind() ? 0 : rearMm;   // no rear ToF: assume the rear block touches the bumper
        if (fLidar)      Serial.println(F("# front ToF down - lot placed from the LiDAR front"));
        if (rearBlind()) Serial.println(F("# rear ToF down - rear block assumed right behind the bumper"));
        if (fMm < PARK_SAFE_MM + 5 && rMm < PARK_SAFE_MM + 5) {
          // no room for a single leg: both ends are already inside the stop distance
          Serial.print(F("# lot too tight for PARK_SAFE_MM=")); Serial.print(PARK_SAFE_MM);
          Serial.println(F(" - lower it (see tof_pair_test for how low the sensors stay reliable)"));
          stopCar(F("ABORT: no room to move"), P_ABORT); break;
        }
        startHeading = gHeading;
        placeModel(lo, fMm, rMm, fLidar);
        if (rMm > 40 || lo - CAR_HALF_W_MM < 60)
          Serial.println(F("# tip: park at the BACK of the lot and out toward the track (60+ mm off the wall) - most room to turn"));
        outHeading = wrapDeg(startHeading + 90.0f * openRot);
        Serial.print(F("# open side: ")); Serial.println(openRot > 0 ? F("LEFT") : F("RIGHT"));
        startManeuver("PIVOT", outHeading, openRot, +1);        // 2. forward first
        go(P_PIVOT);
      }
      break;

    case P_PIVOT: {
      int r = maneuverStep();
      if (r < 0) { stopCar(F("ABORT in PIVOT"), P_ABORT); break; }
      if (r > 0) {
        exitTicks0 = readEncoder();
        setServoAngle(SERVO_TRUE_STRAIGHT);
        legPwm = LEG_PWM_START; legPwmMs = millis();
        Serial.print(F("# EXIT: forward until the front reads ")); Serial.print(EXIT_FRONT_MM); Serial.println(F(" mm"));
        go(P_EXIT);
      }
      break;
    }

    case P_EXIT: {                                    // 3. straight out, heading hold
      float outCm = absEnc(readEncoder() - exitTicks0) / TICKS_PER_CM;
      uint16_t fMm; float bS;
      if (!frontDist(fMm, bS)) { stopCar(F("ABORT: front ToF and LiDAR front both down in EXIT"), P_ABORT); break; }
      bool atWall = fMm < TOF_FAR && fMm <= EXIT_FRONT_MM + gSpeedMmps * bS;
      if (atWall || outCm >= EXIT_CAP_CM) {
        setMotorSpeed(0);
        Serial.print(F("# EXIT ended: "));
        Serial.print(atWall ? F("front at the inner wall") : F("distance cap (front saw nothing)"));
        Serial.print(F(" front=")); Serial.print(fMm);
        Serial.print(F(" out cm=")); Serial.println(outCm, 1);
        startManeuver("REALIGN", startHeading, -openRot, -1, ARC_CAP_CM);   // 4. one full-lock reverse arc
        go(P_REALIGN);
        break;
      }
      if (gImuFresh) {
        float err = wrapDeg(outHeading - gHeading);            // + = nose must go left
        setServoAngle(SERVO_TRUE_STRAIGHT - constrain(2.0f * err, -30.0f, 30.0f));
      }
      float room = fMm < TOF_FAR ? (float)fMm - EXIT_FRONT_MM : 1e6f;
      setMotorSpeed(legSpeedPwm(room));
      break;
    }

    case P_REALIGN: {
      int r = maneuverStep();
      if (r < 0) { stopCar(F("ABORT in REALIGN"), P_ABORT); break; }
      if (r > 0) {
        stopCar(F("PARK OUT DONE - facing the start direction"), P_DONE);
        modelOn = false;
        Serial.print(F("# heading ")); Serial.print(gHeading, 1);
        Serial.print(F(" (start ")); Serial.print(startHeading, 1); Serial.println(')');
      }
      break;
    }
  }
}
