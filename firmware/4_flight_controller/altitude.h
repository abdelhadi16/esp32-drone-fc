#pragma once
// Altitude estimation and altitude control.
//
// ESTIMATOR: third-order complementary filter (same idea as ArduPilot's old
// AP_InertialNav). The accelerometer gives fast, smooth motion but drifts; the
// barometer is slow and noisy but never drifts. The filter keeps the best of both
// and also learns the accelerometer's bias:
//     e     = baroAlt - altEst
//     bias += K3 * e * dt
//     vel  += (accV + bias + K2 * e) * dt
//     alt  += (vel + K1 * e) * dt
// with K1 = 3/tau, K2 = 3/tau^2, K3 = 1/tau^3  (all three poles at -1/tau).
//
// CONTROLLER (ALT HOLD and LAND): two loops in a row
//     altitude error  --(ALT_P)-->  wanted vertical speed
//     speed error     --(VEL_P, VEL_I)-->  throttle around the hover throttle

constexpr float ALT_TAU = 1.5f;      // s: bigger = trust the barometer less
constexpr float ALT_K1 = 3.0f / ALT_TAU;
constexpr float ALT_K2 = 3.0f / (ALT_TAU * ALT_TAU);
constexpr float ALT_K3 = 1.0f / (ALT_TAU * ALT_TAU * ALT_TAU);

float groundPressure = 0.0f;   // Pa at the take-off point (set when arming)
float baroAlt = 0.0f;          // m, raw barometer altitude above the take-off point
float altEst = 0.0f;           // m, fused altitude
float velEst = 0.0f;           // m/s, fused vertical speed (up = +)
float accBias = 0.0f;          // m/s^2, learned accelerometer error
float accVf = 0.0f;            // m/s^2, smoothed vertical acceleration
bool altValid = false;

float altTarget = 0.0f;        // m, ALT HOLD target
float velI = 0.0f;             // us, integral of the speed loop
float hoverThr = HOVER_THROTTLE_GUESS;          // us, base throttle of the vertical controller
float hoverThrLearned = HOVER_THROTTLE_GUESS;   // us, learned while flying level and steady
bool hoverLearnedValid = false;
float hoverLearnTime = 0.0f;

float pressureToAlt(float p) {
  if (groundPressure <= 0.0f) return 0.0f;
  return 44330.0f * (1.0f - powf(p / groundPressure, 0.190295f));
}

// Make the current position "0 m" (called when arming, and by the 'z' command)
void altZero() {
  if (baroPressurePa > 0.0f) groundPressure = baroPressurePa;
  baroAlt = 0.0f;
  altEst = 0.0f;
  velEst = 0.0f;
  altTarget = 0.0f;
}

void altUpdate(float dt, bool newBaroSample) {
  const float a = verticalAccel();
  accVf += (dt / (0.0318f + dt)) * (a - accVf);            // ~5 Hz low-pass against vibration
  if (!baroOk || groundPressure <= 0.0f) { altValid = false; velEst = 0.0f; return; }
  if (newBaroSample) baroAlt = pressureToAlt(baroPressurePa);
  const float e = baroAlt - altEst;
  accBias = clampf(accBias + ALT_K3 * e * dt, -2.0f, 2.0f);
  velEst += (accVf + accBias + ALT_K2 * e) * dt;
  altEst += (velEst + ALT_K1 * e) * dt;
  altValid = true;
}

// Start a vertical-control session (ALT HOLD or LAND) from a known hover throttle
void altCtrlReset(float hover) {
  hoverThr = clampf(hover, 1250.0f, 1800.0f);
  velI = 0.0f;
  altTarget = altEst;
}

// Speed loop: returns the throttle in us
float altCtrlUpdate(float velTarget, float dt) {
  const float velErr = velTarget - velEst;
  const float newI = clampf(velI + VEL_I * velErr * dt, -VEL_I_MAX, VEL_I_MAX);
  float thr = hoverThr + VEL_P * velErr + newI;
  const float tc = fmaxf(tiltCos(), 0.7f);                     // tilted = less lift: add some
  thr = 1000.0f + (thr - 1000.0f) / tc;
  const float lo = (float)MOTOR_IDLE + 30.0f, hi = (float)THROTTLE_MAX;
  // anti wind-up: only keep the new integral if the output is not saturated the same way
  if (!((thr >= hi && velErr > 0) || (thr <= lo && velErr < 0))) velI = newI;
  return clampf(thr, lo, hi);
}

// ALT HOLD: throttle stick near the middle = hold; above/below = climb/descend
float altHoldVelTarget() {
  const int off = (int)rc[CH_THR] - 1500;
  if (abs(off) > ALT_STICK_DEADBAND) {
    const float s = clampf((abs(off) - ALT_STICK_DEADBAND) / (float)(500 - ALT_STICK_DEADBAND), 0.0f, 1.0f);
    altTarget = altEst;                                        // follow while the pilot moves
    return off > 0 ? s * ALT_MAX_CLIMB : -s * ALT_MAX_DESCENT;
  }
  return clampf(ALT_P * (altTarget - altEst), -ALT_MAX_DESCENT, ALT_MAX_CLIMB);
}

float landVelTarget() {
  return -(altEst > LAND_SLOW_ALT ? LAND_SPEED_FAST : LAND_SPEED_SLOW);
}
