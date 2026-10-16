/*
  ==============================================================
   SKETCH 3 — MPU6050 TEST: orientation + level calibration
  ==============================================================
   Serial Monitor: 115200 baud, line ending "Newline".

   The MPU6050 (GY-521) plugs into U2 pins 1..8 (VCC GND SCL SDA XDA XCL AD0 INT).
   Pins 9-10 of U2 stay empty. AD0 is tied to GND -> I2C address 0x68.

   Commands:
     g   re-measure gyro offsets (keep the drone perfectly still)
     l   level calibration (drone on a LEVEL surface) -> prints the 2 trim values
     h   help / checklist

   The 3 checks (front of the drone = where M2/M4 are):
     1. Tilt the RIGHT side down   -> ROLL  becomes POSITIVE (+)
     2. Lift the NOSE up           -> PITCH becomes POSITIVE (+)
     3. Turn the nose to the RIGHT -> YAW RATE is POSITIVE (+) while turning
   If one is wrong, change IMU_ROTATION / IMU_UPSIDE_DOWN below, upload, test again.
   When all 3 pass, copy the SAME settings into sketch 4's config.h (section 1).
*/

#include <Arduino.h>
#include <Wire.h>
#include "esp_log.h"

// ===================== SETTINGS TO FIND =====================
// Direction the "X" arrow printed on the GY-521 points, seen from above:
//   0 = forward, 90 = right, 180 = backward, 270 = left
const int  IMU_ROTATION    = 0;
const bool IMU_UPSIDE_DOWN = false;   // true if the chip faces the ground
// =============================================================

const int I2C_SDA_PIN = 9;
const int I2C_SCL_PIN = 8;
const uint8_t MOTOR_PINS[4] = {10, 11, 12, 13};
const uint8_t MPU_ADDR = 0x68;

const float GYRO_LSB = 65.5f;     // +-500 deg/s range
const float RAD2DEG  = 57.29578f;
const float DEG2RAD  = 0.0174533f;
const uint32_t LOOP_US = 4000;    // 250 Hz, same as the flight code

float gyroOff[3] = {0, 0, 0};
float rollTrim = 0, pitchTrim = 0;
float angleRoll = 0, anglePitch = 0;
float rollRate = 0, pitchRate = 0, yawRate = 0;
float accRoll = 0, accPitch = 0;

// ---------------- I2C helpers ----------------
int readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRaw(int16_t r[6]) {             // ax ay az gx gy gz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  r[0] = (int16_t)(b[0] << 8 | b[1]);
  r[1] = (int16_t)(b[2] << 8 | b[3]);
  r[2] = (int16_t)(b[4] << 8 | b[5]);
  r[3] = (int16_t)(b[8] << 8 | b[9]);
  r[4] = (int16_t)(b[10] << 8 | b[11]);
  r[5] = (int16_t)(b[12] << 8 | b[13]);
  return true;
}

bool mpuInit() {
  int who = readReg(0x75);
  if (who < 0) return false;
  Serial.printf("MPU found, WHO_AM_I = 0x%02X %s\n", who,
                who == 0x68 ? "(MPU6050, OK)" : "(not 0x68: clone chip, usually still works)");
  writeReg(0x6B, 0x01);   // wake up, clock = gyro PLL
  delay(50);
  writeReg(0x1A, 0x03);   // digital low-pass filter ~44 Hz (cuts motor vibration)
  writeReg(0x1B, 0x08);   // gyro  +-500 deg/s
  writeReg(0x1C, 0x10);   // accel +-8 g
  return true;
}

// Turn chip axes into drone axes: forward / left / up
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

// Read the sensor and compute rates (deg/s) and accel angles (deg)
bool updateImu() {
  int16_t r[6];
  if (!readRaw(r)) return false;
  float gf, gl, gu, af, al, au;
  mapAxes((r[3] - gyroOff[0]) / GYRO_LSB, (r[4] - gyroOff[1]) / GYRO_LSB,
          (r[5] - gyroOff[2]) / GYRO_LSB, gf, gl, gu);
  rollRate  =  gf;     // + = right side going down
  pitchRate = -gl;     // + = nose going up
  yawRate   = -gu;     // + = nose turning right
  mapAxes(r[0], r[1], r[2], af, al, au);
  accRoll  = atan2f(al, au) * RAD2DEG - rollTrim;
  accPitch = atan2f(af, sqrtf(al * al + au * au)) * RAD2DEG - pitchTrim;
  return true;
}

void calibrateGyro() {
  Serial.println(F("Gyro calibration: keep the drone STILL for 4 seconds..."));
  double sum[3] = {0, 0, 0};
  int n = 0;
  int16_t r[6];
  for (int i = 0; i < 1000; i++) {
    if (readRaw(r)) { sum[0] += r[3]; sum[1] += r[4]; sum[2] += r[5]; n++; }
    delay(4);
  }
  if (n == 0) { Serial.println(F("  FAILED: no data from MPU")); return; }
  for (int k = 0; k < 3; k++) gyroOff[k] = sum[k] / n;
  Serial.printf("  done. Raw gyro offsets: %.0f %.0f %.0f\n", gyroOff[0], gyroOff[1], gyroOff[2]);
  updateImu();
  angleRoll = accRoll;
  anglePitch = accPitch;
}

void levelCalibration() {
  Serial.println(F("Level calibration: drone on a LEVEL surface, don't touch it (3 s)..."));
  float oldR = rollTrim, oldP = pitchTrim;
  rollTrim = pitchTrim = 0;
  double sr = 0, sp = 0;
  int n = 0;
  for (int i = 0; i < 750; i++) {
    if (updateImu()) { sr += accRoll; sp += accPitch; n++; }
    delay(4);
  }
  if (n == 0) { rollTrim = oldR; pitchTrim = oldP; Serial.println(F("  FAILED")); return; }
  rollTrim = sr / n;
  pitchTrim = sp / n;
  Serial.println(F("  done. In sketch 4's config.h, REPLACE the two ACC_..._TRIM lines with:"));
  Serial.printf("  #define ACC_ROLL_TRIM     %.2ff\n", rollTrim);
  Serial.printf("  #define ACC_PITCH_TRIM    %.2ff\n", pitchTrim);
  angleRoll = 0;
  anglePitch = 0;
}

void printHelp() {
  Serial.println(F("\n=========== SKETCH 3: MPU6050 TEST ==========="));
  Serial.printf("Settings: IMU_ROTATION %d, IMU_UPSIDE_DOWN %s  (same in sketch 4's config.h)\n",
                IMU_ROTATION, IMU_UPSIDE_DOWN ? "true" : "false");
  Serial.println(F("Checks (front = M2/M4 side):"));
  Serial.println(F("  1. Right side DOWN  -> ROLL  positive"));
  Serial.println(F("  2. Nose UP          -> PITCH positive"));
  Serial.println(F("  3. Turn nose RIGHT  -> YAW RATE positive (while turning)"));
  Serial.println(F("Commands: g = gyro calibration, l = level calibration, h = help"));
}

// ==============================================================
//  I2C check: runs when the MPU6050 doesn't answer.
//  Uses the idle bus voltages measured at start-up + a scan of every address.
// ==============================================================
const char* lineState(float v) {
  if (v > 2.6f) return "OK (pulled up)";
  if (v < 0.3f) return "STUCK LOW";
  return "HALF-WAY (pulled down by something)";
}

void i2cDoctor(float sdaV, float sclV) {
  Serial.println(F("\n=========== MPU6050 NOT FOUND: I2C CHECK ==========="));
  Serial.println(F("Bus voltage at start-up (both should be about 3.1-3.3 V):"));
  Serial.printf("  SDA (GPIO %d, U2 pin 4): %.2f V  %s\n", I2C_SDA_PIN, sdaV, lineState(sdaV));
  Serial.printf("  SCL (GPIO %d, U2 pin 3): %.2f V  %s\n", I2C_SCL_PIN, sclV, lineState(sclV));

  Serial.println(F("Scanning every I2C address..."));
  uint8_t found = 0;
  bool at69 = false;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      found++;
      Serial.printf("  device at 0x%02X", a);
      if (a == 0x68)                 Serial.print(F("  <- MPU6050, but it didn't answer a read: loose contact?"));
      else if (a == 0x69)          { Serial.print(F("  <- MPU6050 with AD0 HIGH")); at69 = true; }
      else if (a == 0x76 || a == 0x77) Serial.print(F("  <- BMP280 barometer"));
      Serial.println();
    }
  }
  if (!found) Serial.println(F("  nothing answers on the bus."));

  Serial.print(F("\nMOST LIKELY: "));
  if (sdaV < 0.3f || sclV < 0.3f)
    Serial.println(F("SDA or SCL is shorted to GND (solder bridge), or the module sits in the wrong holes."));
  else if (sdaV <= 2.6f || sclV <= 2.6f)
    Serial.println(F("the module has NO POWER (VCC/GND not connected) or is plugged in backwards."));
  else if (at69)
    Serial.println(F("AD0 is not tied to GND: check U2 pin 7 and its solder joint."));
  else if (!found)
    Serial.println(F("the module's pins don't touch the PCB (header not soldered) or the module is dead."));
  else
    Serial.println(F("a loose contact on SDA/SCL: re-solder U2 pins 3 and 4."));

  Serial.println(F("\nCheck, with the USB cable UNPLUGGED:"));
  Serial.println(F(" 1. The pin header is SOLDERED to the GY-521 board (it comes loose in the bag)."));
  Serial.println(F(" 2. Module in U2 pins 1-8, its VCC pin in the hole marked VCC; U2 pins 9-10 empty."));
  Serial.println(F(" 3. Beep test: module SDA <-> ESP32 pin 9, module SCL <-> ESP32 pin 8, GND <-> GND."));
  Serial.println(F("Then with USB plugged in:"));
  Serial.println(F(" 4. The small LED on the GY-521 is ON. No LED = no power."));
  Serial.println(F(" 5. Multimeter: module VCC to GND = about 3.3 V."));
  Serial.println(F(" 6. Still nothing? Connect the module straight to the ESP32 with 4 jumper wires"));
  Serial.println(F("    (VCC->3V3, GND->GND, SDA->9, SCL->8) and press RST."));
  Serial.println(F("    Found that way = the PCB is the problem. Still not found = the module is bad."));
  Serial.println(F("\nUnplug USB before touching wires. Press RST to test again after a fix."));
}

void setup() {
  // keep the ESCs at "off" while testing (50 Hz: every ESC accepts it)
  const uint32_t duty = 1000UL * ((1UL << 14) - 1) / 20000UL;  // 1000 us at 50 Hz
  for (uint8_t m = 0; m < 4; m++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PINS[m], 50, 14);
    ledcWrite(MOTOR_PINS[m], duty);
#else
    ledcSetup(m, 50, 14);
    ledcAttachPin(MOTOR_PINS[m], m);
    ledcWrite(m, duty);
#endif
  }

  Serial.begin(115200);
  delay(1500);

  // Hide the I2C driver's own red "nack" lines: this sketch explains problems itself.
  esp_log_level_set("i2c.master", ESP_LOG_NONE);
  esp_log_level_set("i2c", ESP_LOG_NONE);

  // Idle bus voltage, measured BEFORE the I2C driver takes the pins.
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  delay(10);
  const float sdaV = analogReadMilliVolts(I2C_SDA_PIN) / 1000.0f;
  const float sclV = analogReadMilliVolts(I2C_SCL_PIN) / 1000.0f;

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);

  if (!mpuInit()) {
    i2cDoctor(sdaV, sclV);
    while (true) delay(1000);
  }
  printHelp();
  calibrateGyro();
}

void loop() {
  static uint32_t loopTimer = micros();
  while (micros() - loopTimer < LOOP_US) { }
  float dt = (micros() - loopTimer) / 1e6f;
  loopTimer = micros();
  if (dt > 0.02f) dt = 0.02f;

  if (updateImu()) {
    // Same filter as the flight code: gyro for fast moves, accel to stop drift
    angleRoll  += rollRate * dt;
    anglePitch += pitchRate * dt;
    float s = sinf(yawRate * dt * DEG2RAD);
    anglePitch -= angleRoll * s;
    angleRoll  += anglePitch * s;
    angleRoll  = 0.998f * angleRoll  + 0.002f * accRoll;
    anglePitch = 0.998f * anglePitch + 0.002f * accPitch;
  }

  if (Serial.available()) {
    char c = Serial.read();
    delay(20);
    while (Serial.available()) Serial.read();
    if (c == 'g') calibrateGyro();
    else if (c == 'l') levelCalibration();
    else if (c == 'h') printHelp();
    loopTimer = micros();
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 150) {
    lastPrint = millis();
    Serial.printf("ROLL %+6.1f  PITCH %+6.1f deg  |  rates  roll %+7.1f  pitch %+7.1f  YAW %+7.1f deg/s\n",
                  angleRoll, anglePitch, rollRate, pitchRate, yawRate);
  }
}
