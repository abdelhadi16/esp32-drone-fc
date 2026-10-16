#pragma once
// Small helpers shared by every module.
#include <Wire.h>

constexpr float RAD2DEG = 57.29578f;
constexpr float DEG2RAD = 0.0174533f;
constexpr float GRAVITY = 9.80665f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---- I2C (MPU6050 and BMP280 share the bus) ----
bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

int i2cReadReg(uint8_t addr, uint8_t reg) {
  uint8_t v;
  return i2cReadBytes(addr, reg, &v, 1) ? v : -1;
}

bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
