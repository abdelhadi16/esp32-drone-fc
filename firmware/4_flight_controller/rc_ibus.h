#pragma once
// FlySky i-BUS receiver (FS-iA10B "i-BUS SERVO" port) on UART1, receive only.
// Frame: 0x20 0x40, 14 channels x 2 bytes (little-endian), 2-byte checksum.

constexpr uint32_t FAILSAFE_MS = 250;     // no valid frame for this long = radio lost

uint16_t rc[14] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500,
                   1500, 1500, 1500, 1500, 1500, 1500};
uint32_t lastFrameMs = 0;

void rcBegin() {
  Serial1.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);
}

void rcUpdate() {
  static uint8_t buf[32];
  static uint8_t idx = 0;
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (idx == 0) { if (b == 0x20) buf[idx++] = b; continue; }
    if (idx == 1) {
      if (b == 0x40) buf[idx++] = b;
      else if (b != 0x20) idx = 0;
      continue;
    }
    buf[idx++] = b;
    if (idx == 32) {
      idx = 0;
      uint16_t sum = 0xFFFF;
      for (uint8_t i = 0; i < 30; i++) sum -= buf[i];
      if (sum == (uint16_t)(buf[30] | (buf[31] << 8))) {
        for (uint8_t ch = 0; ch < 14; ch++) {
          uint16_t v = (buf[2 + ch * 2] | (buf[3 + ch * 2] << 8)) & 0x0FFF;
          rc[ch] = v < 1000 ? 1000 : (v > 2000 ? 2000 : v);
        }
        lastFrameMs = millis();
      }
    }
  }
}

bool radioOk() {
  return lastFrameMs != 0 && (millis() - lastFrameMs) < FAILSAFE_MS;
}

// Stick position -1.0 .. +1.0 with a small dead zone around the centre
float stick(uint16_t v) {
  int d = (int)v - 1500;
  if (abs(d) <= 8) return 0.0f;
  return clampf((d > 0 ? d - 8 : d + 8) / 492.0f, -1.0f, 1.0f);
}

// 3-position mode switch (SwC on CH6): ~1000 MANUAL, ~1500 ALT HOLD, ~2000 LAND
enum FlightMode : uint8_t { MODE_MANUAL = 0, MODE_ALT = 1, MODE_LAND = 2 };
const char *const MODE_NAMES[3] = {"MANUAL", "ALT HOLD", "LAND"};

FlightMode decodeModeSwitch(uint16_t v) {
  if (v < 1300) return MODE_MANUAL;
  if (v > 1700) return MODE_LAND;
  return MODE_ALT;
}
