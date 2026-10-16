#pragma once
// Buzzer + RGB LED. Everything is non-blocking: beepUpdate() runs every loop
// (called from statusUpdate() in the main sketch, which also picks the LED colour).
//
// Buzzer (one-shots)                    Buzzer (repeating, most important wins)
//   ready to arm / log saved .. 1 long    landing ............ beep-beep (0.25 s)
//   armed, mode accepted ...... 1 short   battery CRITICAL ... fast chirps
//   disarmed .................. 2 short   battery LOW ........ short beep every second
//   refused (arm or mode) ..... 3 fast    lost drone ......... chirp every 2 s (after a crash /
//                                                              radio-loss landing, until re-armed)
//
// RGB LED: yellow = starting, green = ready (blinking = not ready), blue = armed MANUAL,
//          purple = ALT HOLD, orange blinking = LAND, yellow flashes = battery LOW,
//          red blinking = battery CRITICAL or MPU6050 missing, white = saving the log

enum BeepRepeat : uint8_t { REP_NONE = 0, REP_BATT_LOW, REP_BATT_CRIT, REP_LANDING, REP_LOST };
struct RepDef { uint16_t on, off; };
static const RepDef REP_DEFS[] = {{0, 0}, {80, 920}, {80, 220}, {250, 250}, {120, 1880}};

BeepRepeat beepRepeat = REP_NONE;
struct Shot { uint8_t remaining; uint16_t on, off; bool phaseOn; uint32_t t0; };
Shot shot = {0, 0, 0, false, 0};
bool buzzerIsOn = false;

void buzzerWrite(bool on) {
  if (BUZZER_PIN < 0 || on == buzzerIsOn) return;
  buzzerIsOn = on;
  if (BUZZER_PASSIVE) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(BUZZER_PIN, on ? 128 : 0);
#else
    ledcWrite(4, on ? 128 : 0);
#endif
  } else {
    digitalWrite(BUZZER_PIN, (on == BUZZER_ACTIVE_HIGH) ? HIGH : LOW);
  }
}

// Call AFTER escInit() (the motors must own LEDC channels 0-3)
void statusBegin() {
  if (BUZZER_PIN >= 0) {
    if (BUZZER_PASSIVE) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
      ledcAttach(BUZZER_PIN, 2700, 8);
#else
      ledcSetup(4, 2700, 8);
      ledcAttachPin(BUZZER_PIN, 4);
#endif
    } else {
      pinMode(BUZZER_PIN, OUTPUT);
    }
    buzzerIsOn = true;
    buzzerWrite(false);
  }
}

void beep(uint8_t n, uint16_t onMs = 80, uint16_t offMs = 120) {
  shot = {n, onMs, offMs, true, (uint32_t)millis()};
  buzzerWrite(true);
}

void beepUpdate(uint32_t now) {
  if (shot.remaining) {                       // one-shot beeps win over the repeating pattern
    if (shot.phaseOn && now - shot.t0 >= shot.on) {
      buzzerWrite(false); shot.phaseOn = false; shot.t0 = now; shot.remaining--;
    } else if (!shot.phaseOn && now - shot.t0 >= shot.off) {
      buzzerWrite(true); shot.phaseOn = true; shot.t0 = now;
    }
    return;
  }
  const RepDef &r = REP_DEFS[beepRepeat];
  if (r.on == 0) { buzzerWrite(false); return; }
  buzzerWrite((now % (r.on + r.off)) < r.on);
}

// ---------------- RGB LED ----------------
uint32_t ledColor = 0xFFFFFFFF;

void ledWrite(uint8_t r, uint8_t g, uint8_t b) {
  if (RGB_LED_PIN < 0) return;
  const uint32_t c = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
  if (c == ledColor) return;                  // only talk to the LED when the colour changes
  ledColor = c;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(RGB_LED_PIN, r, g, b);
#else
  neopixelWrite(RGB_LED_PIN, r, g, b);
#endif
}
