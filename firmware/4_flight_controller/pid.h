#pragma once
// Rate PID used for roll, pitch and yaw.
// Runs at a fixed 250 Hz, so I and D are "per loop" (no dt), like YMFC-AL.

struct Pid { float i; float lastErr; };
Pid pidRoll = {0, 0}, pidPitch = {0, 0}, pidYaw = {0, 0};

float pidRun(Pid &p, float err, float kp, float ki, float kd) {
  p.i = clampf(p.i + ki * err, -PID_MAX, PID_MAX);
  const float out = kp * err + p.i + kd * (err - p.lastErr);
  p.lastErr = err;
  return clampf(out, -PID_MAX, PID_MAX);
}

void pidResetAll() {
  pidRoll = {0, 0};
  pidPitch = {0, 0};
  pidYaw = {0, 0};
}
