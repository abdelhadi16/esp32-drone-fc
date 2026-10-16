#pragma once
#include <stdint.h>
#include <stddef.h>
// I2C bus model: devices answer through sim_i2c_* callbacks implemented by the harness.
bool sim_i2c_present(uint8_t addr);
void sim_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, int n);
void sim_i2c_read(uint8_t addr, uint8_t reg, uint8_t *out, int n);
struct TwoWire {
  uint8_t addr = 0, tx[16]; int txn = 0; uint8_t rx[32]; int rxn = 0, rxpos = 0; uint8_t regPtr[128] = {0};
  bool begin(int, int, uint32_t = 0) { return true; }
  void setTimeOut(uint16_t) {}
  void beginTransmission(uint8_t a) { addr = a; txn = 0; }
  size_t write(uint8_t b) { if (txn < 16) tx[txn++] = b; return 1; }
  uint8_t endTransmission(bool = true) {
    if (!sim_i2c_present(addr)) return 2;
    if (txn >= 1) regPtr[addr] = tx[0];
    if (txn >= 2) sim_i2c_write(addr, tx[0], tx + 1, txn - 1);
    return 0; }
  size_t requestFrom(uint8_t a, size_t n, bool = true) {
    rxn = rxpos = 0;
    if (!sim_i2c_present(a) || n > 32) return 0;
    sim_i2c_read(a, regPtr[a], rx, (int)n); rxn = (int)n; return n; }
  uint8_t requestFrom(uint8_t a, uint8_t n) { return (uint8_t)requestFrom(a, (size_t)n, true); }
  int read() { return rxpos < rxn ? rx[rxpos++] : -1; }
};
extern TwoWire Wire;
