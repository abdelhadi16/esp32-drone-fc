#pragma once
// MPU6050 (GY-521) + attitude estimation (complementary filter).
// Conventions after mapAxes():
//   roll  + = right side down      pitch + = nose up      yaw + = nose turning right

constexpr uint8_t MPU_ADDR = 0x68;
constexpr float GYRO_LSB = 65.5f;     // +-500 deg/s
constexpr float ACC_LSB  = 4096.0f;   // +-8 g

float gyroOff[3] = {0, 0, 0};
float rollRate = 0, pitchRate = 0, yawRate = 0;      // deg/s, raw
float rollRateF = 0, pitchRateF = 0, yawRateF = 0;   // deg/s, smoothed for the PID
float accF = 0, accL = 0, accU = 1;                  // g, drone frame: forward / left / up
float accRoll = 0, accPitch = 0;                     // deg, from the accelerometer only
float angleRoll = 0, anglePitch = 0;                 // deg, fused estimate
uint16_t imuErrors = 0;
bool imuOk = false;
// How much the accelerometer corrects the angles each loop. The main sketch uses a
// fast 0.02 while disarmed (no vibration: settle quickly) and 0.002 while flying.
float accTrust = 0.02f;

bool mpuInit() {
  if (i2cReadReg(MPU_ADDR, 0x75) < 0) return false;
  i2cWriteReg(MPU_ADDR, 0x6B, 0x01);   // wake up, clock = gyro PLL
  delay(50);
  i2cWriteReg(MPU_ADDR, 0x1A, 0x03);   // digital low-pass ~44 Hz (cuts motor vibration)
  i2cWriteReg(MPU_ADDR, 0x1B, 0x08);   // gyro  +-500 deg/s
  i2cWriteReg(MPU_ADDR, 0x1C, 0x10);   // accel +-8 g
  return true;
}

bool readRaw(int16_t r[6]) {          // ax ay az gx gy gz
  uint8_t b[14];
  if (!i2cReadBytes(MPU_ADDR, 0x3B, b, 14)) return false;
  r[0] = (int16_t)(b[0] << 8 | b[1]);
  r[1] = (int16_t)(b[2] << 8 | b[3]);
  r[2] = (int16_t)(b[4] << 8 | b[5]);
  r[3] = (int16_t)(b[8] << 8 | b[9]);
  r[4] = (int16_t)(b[10] << 8 | b[11]);
  r[5] = (int16_t)(b[12] << 8 | b[13]);
  return true;
}

// chip axes -> drone axes (forward / left / up). Same as sketch 3.
void mapAxes(float x, float y, float z, float &fwd, float &left, float &up) {
  switch (IMU_ROTATION) {
    case 90:  fwd =  y; left = -x; break;
    case 180: fwd = -x; left = -y; break;
    case 270: fwd = -y; left =  x; break;
    default:  fwd =  x; left =  y; break;
  }
  up = z;
  if (IMU_UPSIDE_DOWN) { left = -left; up = -up; }
}

bool readImu() {
  int16_t r[6];
  if (!readRaw(r)) return false;
  float gf, gl, gu;
  mapAxes((r[3] - gyroOff[0]) / GYRO_LSB, (r[4] - gyroOff[1]) / GYRO_LSB,
          (r[5] - gyroOff[2]) / GYRO_LSB, gf, gl, gu);
  rollRate  =  gf;
  pitchRate = -gl;
  yawRate   = -gu;
  mapAxes(r[0] / ACC_LSB, r[1] / ACC_LSB, r[2] / ACC_LSB, accF, accL, accU);
  accRoll  = atan2f(accL, accU) * RAD2DEG - ACC_ROLL_TRIM;
  accPitch = atan2f(accF, sqrtf(accL * accL + accU * accU)) * RAD2DEG - ACC_PITCH_TRIM;
  return true;
}

void calibrateGyro() {
  if (DEBUG) Serial.println(F("Gyro calibration: don't move the drone (8 s)..."));
  double sum[3] = {0, 0, 0};
  int n = 0;
  int16_t r[6];
  for (int i = 0; i < 2000; i++) {
    if (readRaw(r)) { sum[0] += r[3]; sum[1] += r[4]; sum[2] += r[5]; n++; }
    rcUpdate();
    delay(4);
  }
  if (n > 0) for (int k = 0; k < 3; k++) gyroOff[k] = sum[k] / n;
}

// Gyro + accelerometer fusion (complementary filter), called at 250 Hz.
// In flight accTrust = 0.002: the angle follows the gyro and is pulled towards the
// accelerometer with a time constant of 1 / (0.002 * 250 Hz) = 2 s.
void updateAttitude(float dt) {
  if (!readImu()) { if (imuErrors < 1000) imuErrors++; return; }
  imuErrors = 0;

  rollRateF  = 0.7f * rollRateF  + 0.3f * rollRate;
  pitchRateF = 0.7f * pitchRateF + 0.3f * pitchRate;
  yawRateF   = 0.7f * yawRateF   + 0.3f * yawRate;

  angleRoll  += rollRate * dt;
  anglePitch += pitchRate * dt;
  const float s = sinf(yawRate * dt * DEG2RAD);     // yawing swaps roll and pitch a little
  anglePitch -= angleRoll * s;
  angleRoll  += anglePitch * s;
  angleRoll  += accTrust * (accRoll  - angleRoll);
  anglePitch += accTrust * (accPitch - anglePitch);
}

// Vertical acceleration in m/s^2 (up = +), gravity removed. Used by the altitude estimator.
float verticalAccel() {
  const float sr = sinf(angleRoll * DEG2RAD),  cr = cosf(angleRoll * DEG2RAD);
  const float sp = sinf(anglePitch * DEG2RAD), cp = cosf(anglePitch * DEG2RAD);
  // "up" direction seen in the drone frame = (sin pitch, sin roll cos pitch, cos roll cos pitch)
  const float fUp = accF * sp + accL * sr * cp + accU * cr * cp;   // in g
  return (fUp - 1.0f) * GRAVITY;
}

float tiltCos() {   // cos(roll) * cos(pitch): 1 when level
  return cosf(angleRoll * DEG2RAD) * cosf(anglePitch * DEG2RAD);
}
