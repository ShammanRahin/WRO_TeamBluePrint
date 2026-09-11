// ============================================================================
// 07 - Heading correction gain
// ============================================================================
// WHAT THIS GIVES YOU
//   HEAD_KP, and confirmation that HEAD_KD and HEAD_KI can stay at zero.
//   Also SERVO_SLEW, working with what sketch 03 told you about jerk.
//
// WHAT HEADING HOLD IS
//   Between corners the car has one job: hold the heading it left the last
//   corner on. It does not follow a wall and it does not look at the camera.
//   It reads yaw off the IMU, compares it to the heading it is supposed to be
//   on, and steers proportionally to the difference:
//
//       error = target_heading - current_heading
//       servo = SERVO_TRUE_STRAIGHT + HEAD_KP * error, slew limited
//
//   That is a P controller and nothing more. KI is zero because there is no
//   steady-state disturbance to integrate away - if the car is drifting
//   constantly, SERVO_TRUE_STRAIGHT is wrong and integrating just hides it.
//   KD is zero because the yaw signal is already filtered (YAW_FILT_ALPHA)
//   and differentiating filtered noise buys nothing at these speeds.
//
// HOW TO TUNE KP
//   Raise it until the car visibly weaves, then back off. Concretely: the
//   oscillation you are looking for is the car snaking down the straight
//   with a period of roughly half a second. Back off to about 60 percent of
//   the gain that first produced it.
//
//   Too low and the car takes the whole straight to come back on line after
//   a corner. Too high and it snakes, which costs you distance accuracy
//   because the encoder counts the zigzag, not the straight line.
//
// WHAT YOU NEED
//   - The longest straight run of competition surface you can find, 3 m plus
//
// HOW TO RUN IT
//   1. Set TEST_KP. Flash. Serial monitor at 115200.
//   2. Press start. The car drives RUN_CM holding its launch heading and
//      logs heading error the whole way.
//   3. It prints RMS error, peak error, and a crude oscillation count.
//   4. Change TEST_KP, reflash, repeat. Try 1.0, 1.5, 2.0, 3.0, 4.0.
//   5. Optional: nudge the car sideways mid-run with your hand and watch how
//      it recovers. That disturbance response tells you more than a clean run.
//
// READING THE RESULT
//   Make a table of KP against RMS error and oscillation count. RMS error
//   falls as KP rises, then the oscillation count starts climbing. Take the
//   KP just below where oscillations begin.
//
//   Current firmware value is 2.0.
// ============================================================================

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "hardware_config.h"

BNO08x myIMU;
Servo steer;

const float TEST_KP     = 2.0;    // the value under test
const float YAW_FILT_ALPHA = 0.35;
const float SERVO_SLEW  = 2.5;    // deg per cycle, from sketch 03
const float RUN_CM      = 300.0;
const int   RUN_PWM     = 130;
const unsigned long CYCLE_US = 5000;

float servoNow = SERVO_TRUE_STRAIGHT;

float readYawDeg() {
  if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    float qi = myIMU.getQuatI(), qj = myIMU.getQuatJ();
    float qk = myIMU.getQuatK(), qr = myIMU.getQuatReal();
    return atan2(2.0f * (qi * qj + qr * qk),
                 qr * qr + qi * qi - qj * qj - qk * qk) * 57.2957795f;
  }
  return NAN;
}
float freshYaw() {
  float y = NAN; unsigned long t = millis();
  while (isnan(y) && millis() - t < 200) y = readYawDeg();
  return y;
}
float wrap180(float a) { return fmod(a + 540.0f, 360.0f) - 180.0f; }

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Wire.begin(); SPI.begin();
  motorBegin(); encoderBegin();
  steer.attach(SERVO_PIN);
  steer.write(SERVO_TRUE_STRAIGHT);
  if (!myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN)) {
    Serial.println("IMU not found."); while (1) delay(100);
  }
  myIMU.enableRotationVector(5000);

  Serial.println();
  Serial.println("=== 07 - heading correction gain ===");
  Serial.print("  KP under test = "); Serial.println(TEST_KP, 2);
}

void loop() {
  waitForButton("Long clear straight? Press start.");

  float target = freshYaw();
  if (isnan(target)) { Serial.println("  IMU dropout"); return; }

  zeroEncoder();
  setMotorSpeed(RUN_PWM);
  servoNow = SERVO_TRUE_STRAIGHT;

  long ticksTarget = (long)(RUN_CM * TICKS_PER_CM);
  double sumSq = 0; long n = 0;
  float peak = 0, filt = 0, prevErr = 0;
  int crossings = 0;
  unsigned long lastUs = micros();

  Serial.println("  cm,error_deg,servo_deg");
  while (labs(readEncoder()) < ticksTarget) {
    while (micros() - lastUs < CYCLE_US) { }
    lastUs = micros();

    float yaw = readYawDeg();
    if (isnan(yaw)) continue;
    float err = wrap180(target - yaw);
    filt += YAW_FILT_ALPHA * (err - filt);

    float want = SERVO_TRUE_STRAIGHT + TEST_KP * filt;
    want = constrain(want, SERVO_MAX_LEFT, SERVO_MAX_RIGHT);
    float d = want - servoNow;
    servoNow += constrain(d, -SERVO_SLEW, SERVO_SLEW);
    steer.write(servoNow);

    sumSq += (double)err * err; n++;
    if (fabs(err) > peak) peak = fabs(err);
    if ((prevErr < 0 && err > 0) || (prevErr > 0 && err < 0)) crossings++;
    prevErr = err;

    if (n % 20 == 0) {
      Serial.print(readEncoder() / TICKS_PER_CM, 1); Serial.print(",");
      Serial.print(err, 2); Serial.print(",");
      Serial.println(servoNow, 1);
    }
  }
  setMotorSpeed(0);
  steer.write(SERVO_TRUE_STRAIGHT);

  Serial.println();
  Serial.print("  KP=");         Serial.print(TEST_KP, 2);
  Serial.print("  rms err = ");  Serial.print(n ? sqrt(sumSq / n) : 0.0, 3);
  Serial.print(" deg  peak = "); Serial.print(peak, 2);
  Serial.print(" deg  zero crossings = "); Serial.println(crossings);
  Serial.println("  More than about 6 crossings over 3 m means it is snaking.");
  Serial.println();
}
