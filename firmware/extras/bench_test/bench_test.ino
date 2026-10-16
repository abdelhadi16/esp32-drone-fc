/*
  ==============================================================
   DRONE BENCH TEST  —  ESP32-S3 flight controller board
  ==============================================================
   !!!  REMOVE ALL PROPELLERS BEFORE USING THIS SKETCH  !!!

   Serial Monitor: 115200 baud, line ending = "Newline"

   Commands (type the letter, press Enter):
     c      ESC throttle calibration (all 4 ESCs at once)
     1..4   spin ONE motor (M1..M4) for 2 s at test throttle
     a      spin M1 -> M2 -> M3 -> M4 one after another
     + / -  raise / lower test throttle (25 us steps, capped at 1400)
     r      live receiver monitor (FlySky i-BUS)
     s      I2C scan + sensor ID check (MPU6050 / MPU9250, BMP280)
     i      IMU check: gyro bias + live accel / gyro / angles
     v      battery voltage
     h      show menu
   Any key stops a running test. Motors always return to 1000 us.

   Pin map (from your PCB schematic):
     M1..M4  -> GPIO 10, 11, 12, 13   (CONN_ESC pins 1..4)
     I-BUS   -> GPIO 18               (CONN_RC pin 2)
     SDA/SCL -> GPIO 9 / GPIO 8
     VBAT    -> GPIO 1  (R3 10k / R4 3.3k divider, CONN_BATT)
*/

#include <Arduino.h>
#include <Wire.h>
#include "esp_log.h"

// ---------------- Pins ----------------
const uint8_t MOTOR_PINS[4] = {10, 11, 12, 13};
const int IBUS_RX_PIN = 18;
const int I2C_SDA_PIN = 9;
const int I2C_SCL_PIN = 8;
const int VBAT_PIN    = 1;

// ---------------- Battery divider ----------------
const float R_TOP    = 10000.0f;  // R3
const float R_BOTTOM = 3300.0f;   // R4  (set to 2700 if you change it)
float VBAT_CAL = 1.00f;           // tune = (multimeter volts) / (printed volts)

// ---------------- ESC signal ----------------
const uint32_t ESC_FREQ_HZ  = 50;    // standard servo PWM, accepted by every ESC
const uint8_t  ESC_RES_BITS = 14;    // max LEDC resolution on ESP32-S3
const uint16_t PWM_MIN  = 1000;      // us  = motor stopped
const uint16_t PWM_MAX  = 2000;      // us  = full throttle (calibration only)
const uint16_t TEST_MIN = 1050;
const uint16_t TEST_MAX = 1400;      // bench safety cap
uint16_t testThrottle   = 1150;
const uint32_t SPIN_MS  = 2000;

// ---------------- FlySky i-BUS ----------------
const uint8_t IBUS_CHANNELS = 14;
uint16_t rcChannels[IBUS_CHANNELS];
uint32_t lastIbusFrameMs = 0;
uint32_t ibusGoodFrames = 0, ibusBadFrames = 0;

// ---------------- Sensors ----------------
const uint8_t MPU_ADDR = 0x68;   // AD0 -> GND on your PCB
const uint8_t MAG_ADDR = 0x0C;   // compass inside an MPU9250 only (the MPU6050 has none)
uint8_t bmpAddr = 0x77;          // BMP280: 0x76 or 0x77, found automatically by 's'

// ==============================================================
//  ESC output
// ==============================================================
void escWriteUs(uint8_t motor, uint16_t us) {
  const uint32_t periodUs = 1000000UL / ESC_FREQ_HZ;
  const uint32_t maxDuty  = (1UL << ESC_RES_BITS) - 1;
  uint32_t duty = (uint32_t)us * maxDuty / periodUs;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PINS[motor], duty);
#else
  ledcWrite(motor, duty);            // channel number == motor index
#endif
}

void allMotors(uint16_t us) {
  for (uint8_t m = 0; m < 4; m++) escWriteUs(m, us);
}

void escInit() {
  for (uint8_t m = 0; m < 4; m++) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PINS[m], ESC_FREQ_HZ, ESC_RES_BITS);
#else
    ledcSetup(m, ESC_FREQ_HZ, ESC_RES_BITS);
    ledcAttachPin(MOTOR_PINS[m], m);
#endif
  }
  allMotors(PWM_MIN);
}

// ==============================================================
//  i-BUS parser  (32-byte frame: 0x20 0x40, 14 ch LE, checksum)
// ==============================================================
void ibusUpdate() {
  static uint8_t buf[32];
  static uint8_t idx = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (idx == 0) {                       // header byte 1
      if (b == 0x20) buf[idx++] = b;
      continue;
    }
    if (idx == 1) {                       // header byte 2
      if (b == 0x40) buf[idx++] = b;
      else if (b != 0x20) idx = 0;        // 0x20 again: stay in sync
      continue;
    }

    buf[idx++] = b;
    if (idx == 32) {
      idx = 0;
      uint16_t sum = 0xFFFF;
      for (uint8_t i = 0; i < 30; i++) sum -= buf[i];
      uint16_t rxSum = buf[30] | (buf[31] << 8);
      if (sum == rxSum) {
        for (uint8_t ch = 0; ch < IBUS_CHANNELS; ch++) {
          rcChannels[ch] = (buf[2 + ch * 2] | (buf[3 + ch * 2] << 8)) & 0x0FFF;
        }
        lastIbusFrameMs = millis();
        ibusGoodFrames++;
      } else {
        ibusBadFrames++;
      }
    }
  }
}

// ==============================================================
//  Serial helpers
// ==============================================================
void flushSerial() {
  delay(20);
  while (Serial.available()) Serial.read();
}

bool stopRequested() {
  if (Serial.available()) { flushSerial(); return true; }
  return false;
}

char waitForKey() {
  flushSerial();
  while (!Serial.available()) { ibusUpdate(); delay(5); }
  char c = Serial.read();
  flushSerial();
  return c;
}

// ==============================================================
//  I2C helpers
// ==============================================================
int readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readBytes(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) dst[i] = Wire.read();
  return true;
}

// ==============================================================
//  Battery
// ==============================================================
float readVbat() {
  uint32_t mv = 0;
  for (int i = 0; i < 16; i++) mv += analogReadMilliVolts(VBAT_PIN);
  mv /= 16;
  return (mv / 1000.0f) * (R_TOP + R_BOTTOM) / R_BOTTOM * VBAT_CAL;
}

void printBattery() {
  float v = readVbat();
  Serial.printf("\nBattery: %.2f V  (%.2f V/cell, 3S)\n", v, v / 3.0f);
  if (v < 5.0f)       Serial.println(F("  -> LiPo not detected (or CONN_BATT not wired)."));
  else if (v < 9.9f)  Serial.println(F("  -> TOO LOW (<3.3 V/cell). Stop and charge NOW."));
  else if (v < 10.8f) Serial.println(F("  -> Low (<3.6 V/cell). Charge before flying."));
  else                Serial.println(F("  -> OK."));
  Serial.println(F("  Compare with a multimeter; adjust VBAT_CAL if it's off."));
}

// ==============================================================
//  Motor tests
// ==============================================================
bool spinMotor(uint8_t m) {
  Serial.printf("\nM%u (GPIO %u) -> %u us for %lu ms   (any key = STOP)\n",
                m + 1, MOTOR_PINS[m], testThrottle, (unsigned long)SPIN_MS);

  for (uint16_t us = PWM_MIN; us <= testThrottle; us += 5) {   // soft ramp
    escWriteUs(m, us);
    delay(10);
    if (stopRequested()) { escWriteUs(m, PWM_MIN); Serial.println(F("  STOPPED")); return false; }
  }

  uint32_t t0 = millis();
  while (millis() - t0 < SPIN_MS) {
    if (stopRequested()) { escWriteUs(m, PWM_MIN); Serial.println(F("  STOPPED")); return false; }
    delay(5);
  }
  escWriteUs(m, PWM_MIN);
  Serial.println(F("  done -> note WHICH arm spun and its DIRECTION (seen from above)."));
  return true;
}

void spinAll() {
  for (uint8_t m = 0; m < 4; m++) {
    if (!spinMotor(m)) return;
    delay(800);
  }
}

void calibrateEscs() {
  Serial.println(F("\n=========== ESC CALIBRATION ==========="));
  Serial.println(F("PROPS OFF! Board powered by USB."));
  Serial.println(F("1) UNPLUG the LiPo, then press Enter   (q = cancel)"));
  if (waitForKey() == 'q') { Serial.println(F("Cancelled.")); return; }

  allMotors(PWM_MAX);
  Serial.println(F("2) Signal is now MAX (2000 us). PLUG IN the LiPo."));
  Serial.println(F("   When you hear the 'max' beeps (usually beep-beep),"));
  Serial.println(F("   press Enter RIGHT AWAY (waiting too long = programming mode)."));
  waitForKey();

  allMotors(PWM_MIN);
  Serial.println(F("3) Signal is now MIN (1000 us)."));
  Serial.println(F("   ESCs should beep the cell count (3) then a long beep = saved."));
  Serial.println(F("Done. Unplug + replug the LiPo, then test motors with 1..4."));
}

// ==============================================================
//  Receiver monitor
// ==============================================================
void receiverMonitor() {
  Serial.println(F("\nReceiver monitor -- move sticks & switches (any key = exit)"));
  Serial.println(F("Expect ~1000..2000 per channel, ~1500 at stick center."));
  flushSerial();
  uint32_t lastPrint = 0;
  while (!stopRequested()) {
    ibusUpdate();
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      if (lastIbusFrameMs == 0 || millis() - lastIbusFrameMs > 500) {
        Serial.printf("NO i-BUS SIGNAL   (good frames %lu, bad %lu)\n",
                      (unsigned long)ibusGoodFrames, (unsigned long)ibusBadFrames);
      } else {
        for (uint8_t i = 0; i < 10; i++) Serial.printf("CH%u:%4u ", i + 1, rcChannels[i]);
        Serial.println();
      }
    }
    delay(1);
  }
}

// ==============================================================
//  Sensors
// ==============================================================
void sensorCheck() {
  // Wake MPU and enable I2C bypass so the AK8963 magnetometer shows up at 0x0C
  writeReg(MPU_ADDR, 0x6B, 0x00);
  delay(50);
  writeReg(MPU_ADDR, 0x37, 0x02);
  delay(10);

  Serial.printf("\nI2C scan (SDA=GPIO%d, SCL=GPIO%d):\n", I2C_SDA_PIN, I2C_SCL_PIN);
  uint8_t found = 0;
  bool seen76 = false, seen77 = false;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  found 0x%02X\n", a);
      found++;
      if (a == 0x76) seen76 = true;
      if (a == 0x77) seen77 = true;
    }
  }
  if (seen76) bmpAddr = 0x76;
  else if (seen77) bmpAddr = 0x77;
  if (!found) Serial.println(F("  NOTHING found -> check 3V3, GND, SDA/SCL, module seating."));

  int who = readReg(MPU_ADDR, 0x75);
  Serial.print(F("MPU  @0x68 WHO_AM_I: "));
  if (who < 0) Serial.println(F("no answer"));
  else {
    Serial.printf("0x%02X -> ", who);
    switch (who) {
      case 0x71: Serial.println(F("MPU9250 (OK)")); break;
      case 0x73: Serial.println(F("MPU9255 (OK)")); break;
      case 0x70: Serial.println(F("MPU6500 = no magnetometer (common 'fake 9250'); gyro+accel still fine")); break;
      case 0x68: Serial.println(F("MPU6050 (OK)")); break;
      default:   Serial.println(F("unknown chip")); break;
    }
  }

  // Only an MPU9250/9255 has a compass (AK8963 at 0x0C). Don't probe for it otherwise.
  Serial.print(F("MAG  @0x0C:          "));
  if (who == 0x71 || who == 0x73) {
    int mag = readReg(MAG_ADDR, 0x00);
    if (mag < 0) Serial.println(F("no answer"));
    else Serial.printf("0x%02X -> %s\n", mag, mag == 0x48 ? "AK8963 (OK)" : "unexpected");
  } else {
    Serial.println(F("none - this chip has no compass (normal for an MPU6050)"));
  }

  int bmp = readReg(bmpAddr, 0xD0);
  Serial.printf("BMP  @0x%02X CHIP_ID:  ", bmpAddr);
  if (bmp < 0) Serial.println(F("no answer"));
  else {
    Serial.printf("0x%02X -> ", bmp);
    if (bmp == 0x58) Serial.println(F("BMP280 (OK)"));
    else if (bmp == 0x60) Serial.println(F("BME280 (OK, also has humidity)"));
    else Serial.println(F("unknown"));
  }
}

bool readImuRaw(int16_t a[3], int16_t g[3]) {
  uint8_t b[14];
  if (!readBytes(MPU_ADDR, 0x3B, b, 14)) return false;
  for (int i = 0; i < 3; i++) {
    a[i] = (int16_t)((b[i * 2] << 8) | b[i * 2 + 1]);
    g[i] = (int16_t)((b[8 + i * 2] << 8) | b[9 + i * 2]);
  }
  return true;
}

void imuLive() {
  if (readReg(MPU_ADDR, 0x75) < 0) { Serial.println(F("\nMPU not responding -- run 's' first.")); return; }

  writeReg(MPU_ADDR, 0x6B, 0x00);  // wake
  delay(100);
  writeReg(MPU_ADDR, 0x1A, 0x03);  // DLPF ~41 Hz
  writeReg(MPU_ADDR, 0x1B, 0x00);  // gyro  +-250 dps -> 131 LSB/dps
  writeReg(MPU_ADDR, 0x1C, 0x00);  // accel +-2 g    -> 16384 LSB/g

  Serial.println(F("\nKeep the drone STILL and FLAT... measuring gyro bias"));
  const int N = 500;
  float bias[3] = {0, 0, 0};
  int16_t a[3], g[3];
  for (int i = 0; i < N; i++) {
    if (readImuRaw(a, g)) for (int k = 0; k < 3; k++) bias[k] += g[k];
    delay(2);
  }
  for (int k = 0; k < 3; k++) bias[k] /= N;
  Serial.printf("Gyro bias (dps): X %.2f  Y %.2f  Z %.2f   <- copy these into your flight code\n",
                bias[0] / 131.0f, bias[1] / 131.0f, bias[2] / 131.0f);
  Serial.println(F("Flat: az ~ +1.00 g. Tilt nose down / right side down and note the signs."));
  Serial.println(F("(any key = exit)"));

  flushSerial();
  uint32_t lastPrint = 0;
  while (!stopRequested()) {
    if (millis() - lastPrint >= 100 && readImuRaw(a, g)) {
      lastPrint = millis();
      float ax = a[0] / 16384.0f, ay = a[1] / 16384.0f, az = a[2] / 16384.0f;
      float gx = (g[0] - bias[0]) / 131.0f, gy = (g[1] - bias[1]) / 131.0f, gz = (g[2] - bias[2]) / 131.0f;
      float roll  = atan2f(ay, az) * 57.2958f;
      float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
      Serial.printf("acc[g] %+5.2f %+5.2f %+5.2f | gyro[dps] %+7.1f %+7.1f %+7.1f | roll %+6.1f pitch %+6.1f\n",
                    ax, ay, az, gx, gy, gz, roll, pitch);
    }
    delay(1);
  }
}

// ==============================================================
//  Menu
// ==============================================================
void printMenu() {
  Serial.println(F("\n========== DRONE BENCH TEST =========="));
  Serial.println(F("   !!! PROPELLERS OFF !!!"));
  Serial.println(F(" c    ESC calibration (all ESCs)"));
  Serial.println(F(" 1-4  spin one motor"));
  Serial.println(F(" a    spin all motors in order"));
  Serial.println(F(" +/-  change test throttle"));
  Serial.println(F(" r    receiver (i-BUS) monitor"));
  Serial.println(F(" s    I2C scan + sensor IDs"));
  Serial.println(F(" i    IMU live + gyro bias"));
  Serial.println(F(" v    battery voltage"));
  Serial.println(F(" h    this menu"));
  Serial.printf(" test throttle = %u us\n", testThrottle);
}

void setup() {
  escInit();                                   // FIRST: hold ESCs at 1000 us
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);   // i-BUS, RX only
  esp_log_level_set("i2c.master", ESP_LOG_NONE);        // hide the driver's red "nack" lines
  esp_log_level_set("i2c", ESP_LOG_NONE);               // (this sketch prints its own results)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  analogReadResolution(12);
  delay(1500);
  printMenu();
}

void loop() {
  ibusUpdate();
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;
  flushSerial();

  switch (c) {
    case 'c': calibrateEscs(); break;
    case '1': case '2': case '3': case '4': spinMotor(c - '1'); break;
    case 'a': spinAll(); break;
    case '+':
      testThrottle += 25;
      if (testThrottle > TEST_MAX) testThrottle = TEST_MAX;
      Serial.printf("test throttle = %u us\n", testThrottle);
      break;
    case '-':
      testThrottle -= 25;
      if (testThrottle < TEST_MIN) testThrottle = TEST_MIN;
      Serial.printf("test throttle = %u us\n", testThrottle);
      break;
    case 'r': receiverMonitor(); break;
    case 's': sensorCheck(); break;
    case 'i': imuLive(); break;
    case 'v': printBattery(); break;
    case 'h': printMenu(); break;
    default:  Serial.println(F("? (h = menu)")); break;
  }
  allMotors(PWM_MIN);   // always end at motor-stop
}
