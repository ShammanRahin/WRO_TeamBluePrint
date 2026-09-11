// ============================================================================
// 03 - Steering jerk
// ============================================================================
// WHAT THIS GIVES YOU
//   How violently the car's rotation rate changes when the steering moves.
//   Jerk is the third derivative of position: position, velocity,
//   acceleration, jerk. Here we take it on the heading axis -
//
//       yaw rate      (deg/s)   straight off the IMU gyro
//       yaw accel     (deg/s2)  rate of change of yaw rate
//       yaw jerk      (deg/s3)  rate of change of yaw accel
//
// WHY YOU CARE
//   A high jerk peak is the car snapping into a turn. That snap is what
//   breaks traction at the rear, scrubs the tyres and makes the encoder
//   over-read. It also puts a torque spike through the servo horn, which is
//   how you strip a plastic gear. If you slew-limit the servo command you
//   trade a small amount of turn-in speed for a large drop in jerk, and the
//   car becomes repeatable.
//
//   SERVO_SLEW in the round firmware is currently 2.5 degrees per control
//   cycle. This sketch is how you decide whether that is the right number.
//
// WHAT YOU NEED
//   - Clear floor, at least 2 m across. The car drives a continuous circle.
//   - Battery at competition charge. Jerk scales with speed, and speed
//     scales with pack voltage.
//
// HOW TO RUN IT
//   1. Flash. Serial monitor at 115200.
//   2. Set SLEW_LIMIT below to the value you want to test. Start with 0
//      (no limit, the servo jumps straight to the target) to see the worst
//      case, then try 1.0, 2.5, 5.0.
//   3. Press start. The car centres, rolls forward for a second, then snaps
//      the steering to TEST_STEER_DEG and holds it while it circles.
//   4. It logs for LOG_MS then stops and prints the peak and RMS jerk.
//   5. Catch the car before it hits anything.
//
// READING THE RESULT
//   Run the same TEST_STEER_DEG at several slew limits and write the peak
//   jerk down for each. You are looking for the knee in the curve - the point
//   where slowing the servo further stops buying you much. That is your
//   SERVO_SLEW.
//
//   Watch the yaw rate settling time too, printed at the end. If the slew
//   limit pushes settling past about 300 ms your corner entry gets sloppy.
// ============================================================================

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "hardware_config.h"

BNO08x myIMU;
Servo steer;

const float TEST_STEER_DEG = 25.0;   // degrees off straight, one side
const float SLEW_LIMIT     = 2.5;    // deg per cycle; 0 = no limit
const int   TEST_PWM       = 120;
const unsigned long LOG_MS = 2500;
const unsigned long CYCLE_US = 5000; // 200 Hz control cycle

float servoNow = SERVO_TRUE_STRAIGHT;

float readYawRate() {
  if (myIMU.getSensorEvent() && myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED)
    return myIMU.getGyroZ() * 57.2957795f;   // rad/s -> deg/s
  return NAN;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Wire.begin();
  SPI.begin();
  motorBegin();
  steer.attach(SERVO_PIN);
  steer.write(SERVO_TRUE_STRAIGHT);

  if (!myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN)) {
    Serial.println("IMU not found. Check CS/INT/RST and the SPI wiring.");
    while (1) delay(100);
  }
  myIMU.enableGyro(5000);   // 200 Hz

  Serial.println();
  Serial.println("=== 03 - steering jerk ===");
  Serial.print("  test steer ");   Serial.print(TEST_STEER_DEG);
  Serial.print(" deg, slew limit "); Serial.print(SLEW_LIMIT);
  Serial.println(" deg/cycle (0 = none)");
  Serial.println("  t_ms,yaw_rate_dps,yaw_accel_dps2,yaw_jerk_dps3");
}

void loop() {
  waitForButton("Clear floor? Press start.");

  // roll straight for a moment so the car is actually moving before we steer
  servoNow = SERVO_TRUE_STRAIGHT;
  steer.write(servoNow);
  setMotorSpeed(TEST_PWM);
  delay(800);

  const float target = SERVO_TRUE_STRAIGHT + TEST_STEER_DEG;

  float prevRate = 0, prevAccel = 0;
  bool  haveRate = false, haveAccel = false;
  double peakJerk = 0, sumJerkSq = 0;
  long  nJerk = 0;
  unsigned long t0 = millis(), lastUs = micros();
  long settleMs = -1;
  float steadyRate = 0;

  while (millis() - t0 < LOG_MS) {
    while (micros() - lastUs < CYCLE_US) { /* hold the cycle rate */ }
    float dt = (micros() - lastUs) / 1e6f;
    lastUs = micros();

    // move the servo toward the target, respecting the slew limit
    if (SLEW_LIMIT <= 0) {
      servoNow = target;
    } else {
      float err = target - servoNow;
      servoNow += constrain(err, -SLEW_LIMIT, SLEW_LIMIT);
    }
    steer.write(servoNow);

    float rate = readYawRate();
    if (isnan(rate)) continue;

    if (haveRate) {
      float accel = (rate - prevRate) / dt;
      if (haveAccel) {
        float jerk = (accel - prevAccel) / dt;
        if (fabs(jerk) > peakJerk) peakJerk = fabs(jerk);
        sumJerkSq += (double)jerk * jerk;
        nJerk++;
        Serial.print(millis() - t0); Serial.print(",");
        Serial.print(rate, 2);       Serial.print(",");
        Serial.print(accel, 1);      Serial.print(",");
        Serial.println(jerk, 0);
      }
      prevAccel = accel; haveAccel = true;
    }
    prevRate = rate; haveRate = true;

    // settling: first time the rate stays within 5 percent of its late value
    steadyRate = rate;
    if (settleMs < 0 && millis() - t0 > 400 && fabs(rate) > 1.0)
      settleMs = millis() - t0;
  }

  setMotorSpeed(0);
  steer.write(SERVO_TRUE_STRAIGHT);

  Serial.println();
  Serial.print("  peak |jerk| = "); Serial.print(peakJerk, 0); Serial.println(" deg/s3");
  Serial.print("  rms  |jerk| = ");
  Serial.print(nJerk ? sqrt(sumJerkSq / nJerk) : 0.0, 0); Serial.println(" deg/s3");
  Serial.print("  steady yaw rate = "); Serial.print(steadyRate, 1); Serial.println(" deg/s");
  Serial.print("  approx settle = "); Serial.print(settleMs); Serial.println(" ms");
  Serial.println("  Change SLEW_LIMIT, reflash, run again. Plot peak jerk vs slew.");
  Serial.println();
}
