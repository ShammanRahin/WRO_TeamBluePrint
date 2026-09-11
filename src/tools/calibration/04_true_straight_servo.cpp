// ============================================================================
// 04 - True straight servo angle
// ============================================================================
// WHAT THIS GIVES YOU
//   SERVO_TRUE_STRAIGHT. The servo angle at which the car actually drives in
//   a straight line.
//
// WHY IT IS NOT 90
//   A servo's mechanical centre, the horn's spline position, the tie-bar
//   length and the two knuckle stops all stack up. The angle that produces
//   zero steer is wherever those errors happen to land. On the nationals car
//   it was 69 degrees, nowhere near 90. You cannot guess it and you cannot
//   eyeball it - a half degree of steer is invisible to the eye and puts the
//   car 9 cm off line over 1 m.
//
//   IMPORTANT: the current build has a JX PS-1171MG where the nationals car
//   had an MG996R. Different servo, different spline, different centre. The
//   69.0 in hardware_config.h is from the old car. Re-run this.
//
// HOW IT WORKS
//   Sweep a set of candidate angles around your best guess. For each one,
//   drive a fixed distance and record how much heading the IMU accumulated.
//   A car driving perfectly straight accumulates zero. The angle with the
//   smallest absolute heading change is the true straight.
//
//   We do several passes per angle and average, because floor texture and
//   a slightly uneven push-off add noise to any single run.
//
// WHAT YOU NEED
//   - A straight clear run of at least 2 m of competition surface
//   - Room to catch the car at the end
//
// HOW TO RUN IT
//   1. Set CENTRE_GUESS to your best guess. Eyeball the wheels parallel and
//      read the angle off the servo tester, or just start at 90.
//   2. Flash. Serial monitor at 115200.
//   3. Press start. The car runs one angle, prints the heading error, and
//      waits. Carry it back to the start line, press start again.
//   4. It works through every angle, PASSES times each.
//   5. Read the winner off the summary table at the end.
//
// READING THE RESULT
//   The table prints mean heading drift per angle. The best angle should be
//   a clear minimum with larger errors either side - a V shape. If the table
//   is flat or noisy, your run distance is too short; raise RUN_CM.
//
//   If the minimum sits at the edge of the sweep, your CENTRE_GUESS was off.
//   Move it and run again.
// ============================================================================

#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "hardware_config.h"

BNO08x myIMU;
Servo steer;

const float CENTRE_GUESS = 69.0;   // your starting point
const float SWEEP_STEP   = 1.0;    // degrees between candidates
const int   SWEEP_EACH   = 4;      // candidates either side of the guess
const int   PASSES       = 3;      // runs per candidate
const float RUN_CM       = 150.0;
const int   RUN_PWM      = 120;

const int N_ANGLES = 2 * SWEEP_EACH + 1;
double sumAbsDrift[N_ANGLES];
int    nRuns[N_ANGLES];

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
  float y = NAN;
  unsigned long t = millis();
  while (isnan(y) && millis() - t < 200) y = readYawDeg();
  return y;
}
float wrap180(float a) { return fmod(a + 540.0f, 360.0f) - 180.0f; }

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Wire.begin(); SPI.begin();
  encoderBegin(); motorBegin();
  steer.attach(SERVO_PIN);
  steer.write(CENTRE_GUESS);

  if (!myIMU.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN)) {
    Serial.println("IMU not found.");
    while (1) delay(100);
  }
  myIMU.enableRotationVector(5000);
  for (int i = 0; i < N_ANGLES; i++) { sumAbsDrift[i] = 0; nRuns[i] = 0; }

  Serial.println();
  Serial.println("=== 04 - true straight servo angle ===");
  Serial.print("  sweeping "); Serial.print(CENTRE_GUESS - SWEEP_EACH * SWEEP_STEP);
  Serial.print(" .. ");        Serial.print(CENTRE_GUESS + SWEEP_EACH * SWEEP_STEP);
  Serial.print(" deg in steps of "); Serial.println(SWEEP_STEP);
}

void runOne(int idx) {
  float angle = CENTRE_GUESS + (idx - SWEEP_EACH) * SWEEP_STEP;
  Serial.print("  angle "); Serial.print(angle, 1); Serial.println(" deg");
  waitForButton("    car on the line, press start");

  steer.write(angle);
  delay(300);
  float y0 = freshYaw();
  zeroEncoder();
  setMotorSpeed(RUN_PWM);
  long target = (long)(RUN_CM * TICKS_PER_CM);
  while (labs(readEncoder()) < target) { readYawDeg(); }
  setMotorSpeed(0);
  delay(400);                       // let it stop rocking
  float y1 = freshYaw();

  if (isnan(y0) || isnan(y1)) { Serial.println("    IMU dropout, discarded"); return; }
  float drift = wrap180(y1 - y0);
  sumAbsDrift[idx] += fabs(drift);
  nRuns[idx]++;
  Serial.print("    drift "); Serial.print(drift, 2); Serial.println(" deg");
}

void loop() {
  for (int p = 0; p < PASSES; p++)
    for (int i = 0; i < N_ANGLES; i++)
      runOne(i);

  Serial.println();
  Serial.println("=== summary: mean |heading drift| per angle ===");
  int best = 0;
  for (int i = 0; i < N_ANGLES; i++) {
    float angle = CENTRE_GUESS + (i - SWEEP_EACH) * SWEEP_STEP;
    double mean = nRuns[i] ? sumAbsDrift[i] / nRuns[i] : 999;
    Serial.print("  "); Serial.print(angle, 1);
    Serial.print(" deg  ->  "); Serial.print(mean, 3); Serial.println(" deg drift");
    if (mean < (nRuns[best] ? sumAbsDrift[best] / nRuns[best] : 999)) best = i;
  }
  Serial.print("  SERVO_TRUE_STRAIGHT = ");
  Serial.println(CENTRE_GUESS + (best - SWEEP_EACH) * SWEEP_STEP, 1);
  while (1) delay(1000);
}
