#pragma once
// BMP280 barometer driver (no library needed). Address 0x76 or 0x77, found automatically.
// Compensation formulas: Bosch BMP280 datasheet, section 3.11.3 (64-bit integer version).
//
// Settings: pressure x8, temperature x1, IIR filter x4, normal mode, standby 0.5 ms
//   -> a new sample about every 23 ms (~44 Hz), noise around 5-10 cm.
// TIP: cover the sensor with a piece of open-cell foam: prop wash and light
//      both disturb the reading.

uint8_t bmpAddr = 0;            // 0 = not found
bool baroOk = false;
float baroPressurePa = 0.0f;    // latest pressure, Pa
float baroTempC = 0.0f;
uint8_t baroFails = 0;

static uint16_t digT1, digP1;
static int16_t  digT2, digT3, digP2, digP3, digP4, digP5, digP6, digP7, digP8, digP9;

bool baroRead() {
  if (!bmpAddr) return false;
  uint8_t b[6];
  if (!i2cReadBytes(bmpAddr, 0xF7, b, 6)) return false;
  const int32_t adcP = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
  const int32_t adcT = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);
  if (adcP == 0x80000 || adcT == 0x80000) return false;     // no measurement yet

  // temperature -> tFine
  int32_t v1 = ((((adcT >> 3) - ((int32_t)digT1 << 1))) * ((int32_t)digT2)) >> 11;
  int32_t v2 = (((((adcT >> 4) - ((int32_t)digT1)) * ((adcT >> 4) - ((int32_t)digT1))) >> 12) *
                ((int32_t)digT3)) >> 14;
  const int32_t tFine = v1 + v2;
  baroTempC = ((tFine * 5 + 128) >> 8) / 100.0f;

  // pressure (64-bit), result in Pa * 256
  int64_t p1 = (int64_t)tFine - 128000;
  int64_t p2 = p1 * p1 * (int64_t)digP6;
  p2 = p2 + ((p1 * (int64_t)digP5) << 17);
  p2 = p2 + (((int64_t)digP4) << 35);
  p1 = ((p1 * p1 * (int64_t)digP3) >> 8) + ((p1 * (int64_t)digP2) << 12);
  p1 = (((((int64_t)1) << 47) + p1)) * ((int64_t)digP1) >> 33;
  if (p1 == 0) return false;
  int64_t p = 1048576 - adcP;
  p = (((p << 31) - p2) * 3125) / p1;
  p1 = (((int64_t)digP9) * (p >> 13) * (p >> 13)) >> 25;
  p2 = (((int64_t)digP8) * p) >> 19;
  p = ((p + p1 + p2) >> 8) + (((int64_t)digP7) << 4);

  const float pa = (float)p / 256.0f;
  if (pa < 30000.0f || pa > 110000.0f) return false;          // nonsense -> ignore
  baroPressurePa = pa;
  return true;
}

bool baroInit() {
  bmpAddr = 0;
  const uint8_t addrs[2] = {0x76, 0x77};
  for (uint8_t i = 0; i < 2; i++) {
    const int id = i2cReadReg(addrs[i], 0xD0);
    if (id == 0x58 || id == 0x60) { bmpAddr = addrs[i]; break; }   // BMP280 / BME280
  }
  if (!bmpAddr) return false;

  i2cWriteReg(bmpAddr, 0xE0, 0xB6);    // soft reset -> sleep mode
  delay(10);
  uint8_t c[24];
  if (!i2cReadBytes(bmpAddr, 0x88, c, 24)) { bmpAddr = 0; return false; }
  digT1 = (uint16_t)(c[0] | (c[1] << 8));
  digT2 = (int16_t)(c[2] | (c[3] << 8));
  digT3 = (int16_t)(c[4] | (c[5] << 8));
  digP1 = (uint16_t)(c[6] | (c[7] << 8));
  digP2 = (int16_t)(c[8] | (c[9] << 8));
  digP3 = (int16_t)(c[10] | (c[11] << 8));
  digP4 = (int16_t)(c[12] | (c[13] << 8));
  digP5 = (int16_t)(c[14] | (c[15] << 8));
  digP6 = (int16_t)(c[16] | (c[17] << 8));
  digP7 = (int16_t)(c[18] | (c[19] << 8));
  digP8 = (int16_t)(c[20] | (c[21] << 8));
  digP9 = (int16_t)(c[22] | (c[23] << 8));

  i2cWriteReg(bmpAddr, 0xF5, 0x08);    // config: standby 0.5 ms, IIR filter x4 (write in sleep mode)
  i2cWriteReg(bmpAddr, 0xF4, 0x33);    // ctrl_meas: temp x1, pressure x8, normal mode
  delay(60);
  for (uint8_t i = 0; i < 10; i++) {   // wait for the first real sample
    if (baroRead()) return true;
    delay(25);
  }
  bmpAddr = 0;
  return false;
}

// Called at 50 Hz from the main loop. Returns true when a sample was read.
bool baroUpdate() {
  if (!baroOk) return false;
  if (baroRead()) { baroFails = 0; return true; }
  if (++baroFails > 25) baroOk = false;   // 0.5 s without data = sensor lost
  return false;
}
