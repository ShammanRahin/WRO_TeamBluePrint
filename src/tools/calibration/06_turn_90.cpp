// ============================================================================
// 06 - Ninety degree turns
// ============================================================================
// WHAT THIS GIVES YOU
//   The turn law constants: TURN_KP, TURN_MAX_STEER, TURN_MIN_STEER, TURN_KV,
//   TURN_STOP_DEG and the PWM band. Current values are in the table at the
//   bottom of docs/CALIBRATION.md.
//
// HOW THE TURN ACTUALLY WORKS
//   The car does not turn for a fixed time or a fixed distance. Both of those
//   lie to you the moment a wheel slips. It turns until the IMU says the
//   heading has moved 90 degrees, and not one degree before.
//
//   The law is proportional on heading error. Big error, big steering angle
//   and more speed; small error, less of both, so the car eases into the exit
//   instead of snapping onto it and overshooting:
//
//       error = target_heading - current_heading
//       steer = clamp(TURN_KP * |error|, TURN_MIN_STEER, TURN_MAX_STEER)
//       pwm   = clamp(TURN_KV * |error|, TURN_MIN_PWM,   TURN_MAX_PWM)
//       done when |error| < TURN_STOP_DEG
//
//   There is deliberately no settle delay. The turn hands straight over to
//   heading hold, which absorbs the last fraction of a degree while the car
//   is already driving away. A settle step just wastes track time.
//
// WHAT YOU NEED
//   - Open floor, at least 1.5 m square
//   - Nothing else. The car turns on the spot from a rolling start.
//
// HOW TO RUN IT
//   1. Flash. Serial monitor at 115200.
//   2. Press start. The car rolls forward briefly, turns 90 degrees, stops,
//      and prints the final heading error plus how long the turn took.
//   3. Press start again for the next sample. Do 30. Alternate LEFT and
//      RIGHT - set TURN_DIRECTION and reflash halfway through, or set
//      ALTERNATE true and it will swap automatically.
//
// READING THE RESULT
//   You want the mean error near zero and the standard deviation under about
//   1.5 degrees. Then:
//
//     mean error consistently positive or negative -> the car overshoots or
//       undershoots every time. Lower TURN_KP if it overshoots.
//     large sd, errors scattered both ways -> TURN_MIN_STEER or TURN_MIN_PWM
//       is too low and the car stalls out near the end of the turn, or the
//       floor is inconsistent.
//     left and right means differ -> mechanical. Your steering is not
//       symmetric about SERVO_TRUE_STRAIGHT. Go back to sketch 04.
//
//   Error accumulates. A 1 degree bias per corner is 12 degrees by the end of
//   three laps, and that is a wall.
// ============================================================================

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "hardware_config.h"

BNO08x myIMU;
Servo steer;

// --- the constants under test ---
const float TURN_KP        = 2.5;
const float TURN_MAX_STEER = 55.0;
const float TURN_MIN_STEER = 8.0;
const float TURN_KV        = 3.5;
const int   TURN_MAX_PWM   = 130;
const int   TURN_MIN_PWM   = 100;
const float TURN_STOP_DEG  = 0.3;

const bool  ALTERNATE      = true;   // swap left/right every sample
bool        turnRight      = true;
const int   ROLL_IN_PWM    = 110;
const int   ROLL_IN_MS     = 400;
const unsigned long TURN_TIMEOUT_MS = 4000;

Stats errLeft, errRight, durLeft, durRight;

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
  Serial.println("=== 06 - ninety degree turns ===");
  Serial.println("  press start for each sample, 30 samples recommended");
}

void loop() {
  waitForButton(turnRight ? "Press start - RIGHT turn" : "Press start - LEFT turn");

  steer.write(SERVO_TRUE_STRAIGHT);
  setMotorSpeed(ROLL_IN_PWM);
  delay(ROLL_IN_MS);

  float start = freshYaw();
  if (isnan(start)) { setMotorSpeed(0); Serial.println("  IMU dropout"); return; }
  float target = wrap180(start + (turnRight ? -90.0f : 90.0f));

  unsigned long t0 = millis();
  float err = 0;
  while (millis() - t0 < TURN_TIMEOUT_MS) {
    float yaw = readYawDeg();
    if (isnan(yaw)) continue;
    err = wrap180(target - yaw);
    if (fabs(err) < TURN_STOP_DEG) break;

    float mag = fabs(err);
    float s   = constrain(TURN_KP * mag, TURN_MIN_STEER, TURN_MAX_STEER);
    int   pwm = (int)constrain(TURN_KV * mag, (float)TURN_MIN_PWM, (float)TURN_MAX_PWM);
    steer.write(SERVO_TRUE_STRAIGHT + (err > 0 ? s : -s));
    setMotorSpeed(pwm);
  }
  unsigned long dur = millis() - t0;
  setMotorSpeed(0);
  steer.write(SERVO_TRUE_STRAIGHT);
  delay(500);

  float settled = freshYaw();
  float finalErr = wrap180(target - settled);

  Serial.print(turnRight ? "  RIGHT " : "  LEFT  ");
  Serial.print("final error "); Serial.print(finalErr, 2);
  Serial.print(" deg in ");     Serial.print(dur); Serial.println(" ms");

  if (turnRight) { errRight.add(finalErr); durRight.add(dur); }
  else           { errLeft.add(finalErr);  durLeft.add(dur);  }

  if ((errLeft.n + errRight.n) % 5 == 0) {
    Serial.println();
    errRight.print("  RIGHT err", " deg");
    errLeft.print ("  LEFT  err", " deg");
    durRight.print("  RIGHT dur", " ms");
    durLeft.print ("  LEFT  dur", " ms");
    Serial.println();
  }
  if (ALTERNATE) turnRight = !turnRight;
}
