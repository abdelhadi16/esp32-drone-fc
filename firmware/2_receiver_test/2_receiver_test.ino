/*
  ==============================================================
   SKETCH 2 — RECEIVER TEST (FlySky FS-iA10B, i-BUS)
  ==============================================================
   Serial Monitor: 115200 baud.

   Wiring: receiver "i-BUS SERVO" port -> PCB H2 (CONN_RC)
       S (signal) -> H2 pin 2  (GPIO 18)
       +          -> H2 pin 1  (5V)
       -          -> H2 pin 3  (GND)

   What you should see, 10 times per second:
     ROLL PITCH THR YAW  = CH1..CH4, about 1000..2000, 1500 at centre
     CH5 (SwA)           = ARM (about 2000) or SAFE (about 1000)
     CH6 (SwC)           = MANUAL (about 1000), ALT HOLD (about 1500), LAND (about 2000)
     CH7..CH10           = other switches / knobs (not used)
   Failsafe check (guide Step 7.6): with SwA on ARM and SwC on MANUAL, switch the
   transmitter OFF. Within about 1 s you must see THR about 1000, CH5 still ARM
   and CH6 = LAND  (the flight controller then lands by itself).
   Motors are held at "off" the whole time (safe to run with the LiPo in).
   The ESCs get a 50 Hz "off" signal here, so they arm and stay quiet.
*/

#include <Arduino.h>

const int IBUS_RX_PIN = 18;
const uint8_t MOTOR_PINS[4] = {10, 11, 12, 13};

// ---------------- i-BUS ----------------
const uint8_t IBUS_CHANNELS = 14;
uint16_t rc[IBUS_CHANNELS];
uint32_t lastFrameMs = 0;
uint32_t goodFrames = 0, badFrames = 0;

// i-BUS frame: 0x20 0x40, 14 channels (2 bytes each, little-endian), checksum
void ibusUpdate() {
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
        for (uint8_t ch = 0; ch < IBUS_CHANNELS; ch++)
          rc[ch] = (buf[2 + ch * 2] | (buf[3 + ch * 2] << 8)) & 0x0FFF;
        lastFrameMs = millis();
        goodFrames++;
      } else {
        badFrames++;
      }
    }
  }
}

// Keep the ESCs quiet and safe: 1000 us at 50 Hz (a signal every ESC accepts)
void escHoldOff() {
  const uint32_t duty = 1000UL * ((1UL << 14) - 1) / 20000UL;  // 1000 us of a 20000 us (50 Hz) period
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
}

// Same thresholds as the flight controller (sketch 4)
const char* armSwitch(uint16_t v) { return v > 1700 ? "ARM" : "SAFE"; }
const char* modeSwitch(uint16_t v) {
  if (v < 1300) return "MANUAL";
  if (v > 1700) return "LAND";
  return "ALT HOLD";
}

void setup() {
  escHoldOff();
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, IBUS_RX_PIN, -1);
  delay(1500);
  Serial.println(F("\n=========== SKETCH 2: RECEIVER TEST ==========="));
  Serial.println(F("Turn the transmitter ON. Move each stick and switch."));
  Serial.println(F("CH5 must show ARM/SAFE with SwA, CH6 MANUAL/ALT HOLD/LAND with SwC."));
}

void loop() {
  ibusUpdate();

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 100) return;
  lastPrint = millis();

  if (lastFrameMs == 0 || millis() - lastFrameMs > 200) {
    Serial.printf("SIGNAL LOST   (good frames %lu, bad frames %lu)\n",
                  (unsigned long)goodFrames, (unsigned long)badFrames);
    return;
  }

  Serial.printf("ROLL %4u  PITCH %4u  THR %4u  YAW %4u | "
                "CH5 %4u %-4s | CH6 %4u %-8s | CH7 %4u  CH8 %4u  CH9 %4u  CH10 %4u\n",
                rc[0], rc[1], rc[2], rc[3],
                rc[4], armSwitch(rc[4]), rc[5], modeSwitch(rc[5]), rc[6], rc[7], rc[8], rc[9]);
}
