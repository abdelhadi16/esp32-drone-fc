/*
  ==============================================================
   SKETCH 1 — MOTOR TEST + ESC CALIBRATION   (version 2)
   ESP32-S3 flight controller board (your PCB)
  ==============================================================
   Serial Monitor: 115200 baud, line ending "Newline".

   What changed in version 2 (fix for "ESCs keep beeping, motors don't spin"):
     - The ESC signal now starts at 50 Hz, the standard servo signal that
       every ESC accepts. Use  f  to try faster signals later.
     - Calibration is timed by the code: you only plug in the LiPo at "NOW".
     - New tools:  b = what the beeps mean,  p = wiring check with a multimeter,
                   i = status,  o = lower the "off" pulse.
     - Stronger output drive on the 4 signal pins.

   Commands (type, then Enter):
     c      ESC calibration (all 4 ESCs at once)          -- PROPS OFF
     1..4   spin ONE motor (M1..M4) for 3 seconds
     a      spin ALL 4 motors together for 3 seconds
     s      spin M1 -> M2 -> M3 -> M4, one after another
     +  -   raise / lower the test throttle (steps of 25 us)
     f      change the ESC signal speed: 50 / 100 / 200 / 250 / 400 Hz
     o      change the "off" pulse: 1000 / 950 / 900 us
     p      wiring check (multimeter) for the 4 signal wires
     b      what the ESC beeps mean
     i      status (signal, battery)
     h      show the menu

   EMERGENCY STOP (any of these, at any time):
     - type ANY key + Enter in the Serial Monitor
     - press the BOOT button on the ESP32 board
     - unplug the LiPo (always works, even if the code is stuck)
   Motors also stop by themselves after 3 seconds.

   Motor layout (seen from above, front at the top):
        M4 (CW)    M2 (CCW)
             \  /
             /  \
        M3 (CCW)   M1 (CW)
*/

#include <Arduino.h>
#include "driver/gpio.h"

// ---------------- Pins (from your PCB) ----------------
const uint8_t MOTOR_PINS[4] = {10, 11, 12, 13};      // H6 pins 1..4
const char* const MOTOR_NAMES[4] = {"M1 rear-right  (should spin CW)",
                                    "M2 front-right (should spin CCW)",
                                    "M3 rear-left   (should spin CCW)",
                                    "M4 front-left  (should spin CW)"};
const int BOOT_BUTTON = 0;                           // BOOT button on the DevKit
const int VBAT_PIN    = 1;                           // H5 battery divider (R3 10k / R4 3.3k)
const float VBAT_DIVIDER = (10000.0f + 3300.0f) / 3300.0f;

// ---------------- ESC signal ----------------
const uint32_t FREQ_LIST[]  = {50, 100, 200, 250, 400};   // f cycles through these
const uint8_t  FREQ_COUNT   = sizeof(FREQ_LIST) / sizeof(FREQ_LIST[0]);
uint8_t        freqIndex    = 0;                          // start at 50 Hz
uint32_t       escFreq      = 50;
const uint8_t  ESC_RES_BITS = 14;

const uint16_t OFF_LIST[]   = {1000, 950, 900};           // o cycles through these
const uint8_t  OFF_COUNT    = sizeof(OFF_LIST) / sizeof(OFF_LIST[0]);
uint8_t        offIndex     = 0;
uint16_t       pwmOff       = 1000;                       // "motor off" pulse
const uint16_t CAL_MIN      = 1000;                       // calibration end points
const uint16_t CAL_MAX      = 2000;

// ---------------- Test limits ----------------
const uint16_t TEST_MIN = 1050;
const uint16_t TEST_MAX = 1400;      // hard cap for bench tests
uint16_t testThrottle   = 1200;      // start low
const uint32_t RUN_TIME_MS = 3000;   // auto-stop after this

bool attachOk[4]     = {false, false, false, false};
bool bootStopEnabled = true;         // switched off if the BOOT pin reads "pressed" at start-up
bool lipoSeen        = false;

// ==============================================================
//  ESC output
// ==============================================================
uint32_t usToDuty(uint16_t us, uint32_t freq) {
  const uint32_t periodUs = 1000000UL / freq;
  const uint32_t maxDuty  = (1UL << ESC_RES_BITS) - 1;
  return (uint32_t)us * maxDuty / periodUs;
}

void escWriteDuty(uint8_t m, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PINS[m], duty);
#else
  ledcWrite(m, duty);
#endif
}

void escWriteUs(uint8_t m, uint16_t us) {
  escWriteDuty(m, usToDuty(us, escFreq));
}

void allMotors(uint16_t us) {
  for (uint8_t m = 0; m < 4; m++) escWriteUs(m, us);
}

void allNoSignal() {                  // wire held LOW: the ESC sees "no signal" (motor stops)
  for (uint8_t m = 0; m < 4; m++) escWriteDuty(m, 0);
}

void escInit() {
  for (uint8_t m = 0; m < 4; m++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    attachOk[m] = ledcAttach(MOTOR_PINS[m], escFreq, ESC_RES_BITS);
#else
    attachOk[m] = ledcSetup(m, escFreq, ESC_RES_BITS) > 0;
    ledcAttachPin(MOTOR_PINS[m], m);
#endif
    gpio_set_drive_capability((gpio_num_t)MOTOR_PINS[m], GPIO_DRIVE_CAP_3);   // strongest output
  }
  allMotors(pwmOff);
}

// Change the signal speed safely: no pulses during the switch, so an ESC
// can never see an over-long pulse (which some read as full throttle).
bool setEscFrequency(uint32_t f) {
  allNoSignal();
  delay(30);
  bool ok = true;
  for (uint8_t m = 0; m < 4; m++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if (ledcChangeFrequency(MOTOR_PINS[m], f, ESC_RES_BITS) == 0) ok = false;
#else
    if (ledcSetup(m, f, ESC_RES_BITS) == 0) ok = false;
#endif
  }
  escFreq = f;
  allMotors(pwmOff);
  return ok;
}

// ==============================================================
//  Battery (only works if H5 is wired to the PDB; reads 0 V otherwise)
// ==============================================================
float readVbat() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(VBAT_PIN);
  return (mv / 8) / 1000.0f * VBAT_DIVIDER;
}

void lipoWatch() {
  static uint32_t last = 0;
  if (millis() - last < 300) return;
  last = millis();
  float v = readVbat();
  if (!lipoSeen && v > 6.0f) {
    lipoSeen = true;
    Serial.printf("\nLiPo connected (%.1f V). Listen: melody, 3 beeps, long beep = ESCs READY.\n", v);
    Serial.println(F("  Beeping that doesn't stop? Type b."));
  } else if (lipoSeen && v < 3.0f) {
    lipoSeen = false;
    Serial.println(F("\nLiPo disconnected."));
  }
}

// ==============================================================
//  Stop handling / keyboard
// ==============================================================
void flushSerial() {
  delay(20);
  while (Serial.available()) Serial.read();
}

bool bootPressed() {
  return bootStopEnabled && digitalRead(BOOT_BUTTON) == LOW;
}

// true if the user asked to stop (Serial key or BOOT button)
bool stopRequested() {
  if (Serial.available()) { flushSerial(); return true; }
  return bootPressed();
}

void emergencyStop() {
  allMotors(pwmOff);
  Serial.println(F("\n*** STOPPED: all motors off ***"));
  while (bootPressed()) delay(10);   // wait for button release
}

// Wait for a key (returns it), the BOOT button (returns 'B'),
// or a timeout (returns 0). timeoutMs = 0 means wait forever.
char waitKeyOrBoot(uint32_t timeoutMs) {
  flushSerial();
  while (bootPressed()) delay(10);   // ignore a button that is already held
  uint32_t t0 = millis();
  while (timeoutMs == 0 || millis() - t0 < timeoutMs) {
    if (Serial.available()) { char c = Serial.read(); flushSerial(); return c; }
    if (bootPressed()) { while (bootPressed()) delay(10); return 'B'; }
    delay(5);
  }
  return 0;
}

// ==============================================================
//  Spin the motors in 'mask' (bit 0 = M1 ... bit 3 = M4)
//  Soft ramp up, hold, then stop. Returns false if stopped early.
// ==============================================================
bool runMotors(uint8_t mask, uint32_t holdMs) {
  for (uint16_t us = pwmOff; us <= testThrottle; us += 5) {
    for (uint8_t m = 0; m < 4; m++) if (mask & (1 << m)) escWriteUs(m, us);
    delay(10);
    if (stopRequested()) { emergencyStop(); return false; }
  }
  uint32_t t0 = millis();
  while (millis() - t0 < holdMs) {
    if (stopRequested()) { emergencyStop(); return false; }
    delay(5);
  }
  allMotors(pwmOff);
  return true;
}

void notSpinningHint() {
  Serial.println(F("  Nothing turned? If the ESCs are still beeping, they are NOT armed:"));
  Serial.println(F("  type b to read the beeps. If they are silent, try + to raise the throttle."));
}

void spinOne(uint8_t m) {
  Serial.printf("\n%s  ->  %u us for %lu s at %lu Hz   (any key / BOOT = STOP)\n",
                MOTOR_NAMES[m], testThrottle, (unsigned long)(RUN_TIME_MS / 1000),
                (unsigned long)escFreq);
  if (runMotors(1 << m, RUN_TIME_MS)) {
    Serial.println(F("  done. Which arm spun? Which direction (from above)?"));
    notSpinningHint();
  }
}

void spinAll() {
  Serial.printf("\nALL 4 motors  ->  %u us for %lu s at %lu Hz   (any key / BOOT = STOP)\n",
                testThrottle, (unsigned long)(RUN_TIME_MS / 1000), (unsigned long)escFreq);
  if (runMotors(0x0F, RUN_TIME_MS)) {
    Serial.println(F("  done. Did all 4 start at the same time and sound the same?"));
    notSpinningHint();
  }
}

void spinSequence() {
  for (uint8_t m = 0; m < 4; m++) {
    Serial.printf("\n%s  ->  %u us\n", MOTOR_NAMES[m], testThrottle);
    if (!runMotors(1 << m, 2000)) return;
    uint32_t t0 = millis();
    while (millis() - t0 < 1000) {                     // pause, still listening
      if (stopRequested()) { emergencyStop(); return; }
      delay(5);
    }
  }
  Serial.println(F("\nSequence done."));
}

// ==============================================================
//  ESC calibration: teaches each ESC where 1000 us and 2000 us are.
//  The code does the timing: you plug in the LiPo at "NOW", and it drops
//  to STOP 3-4 s later (or as soon as you press Enter / BOOT on 'beep-beep').
// ==============================================================
void calibrateEscs() {
  Serial.println(F("\n=========== ESC CALIBRATION ==========="));
  Serial.println(F("Teaches every ESC: 1000 us = stop, 2000 us = full power. PROPS OFF!"));
  if (readVbat() > 6.0f) {
    Serial.println(F("The LiPo is plugged in (H5 sees it). UNPLUG it, then type c again."));
    return;
  }
  Serial.println(F("1) LiPo UNPLUGGED? Hold the LiPo plug ready. Press Enter to start. (q = cancel)"));
  if (waitKeyOrBoot(0) == 'q') { Serial.println(F("Cancelled.")); return; }

  allMotors(CAL_MAX);
  Serial.println(F("2) Signal is now FULL (2000 us). Plug in the LiPo when you read NOW:"));
  for (int i = 3; i >= 1; i--) {
    Serial.printf("   %d...\n", i);
    uint32_t t0 = millis();
    while (millis() - t0 < 1000) {
      if (Serial.available() && Serial.read() == 'q') {
        allMotors(pwmOff);
        flushSerial();
        Serial.println(F("Cancelled."));
        return;
      }
      delay(5);
    }
  }
  Serial.println(F("   >>> NOW: PLUG IN THE LIPO <<<"));
  flushSerial();

  // Drop to STOP: 3 s after H5 sees the LiPo, or 4 s after NOW if H5 isn't wired.
  // Enter or BOOT drops it immediately (use it when you hear 'beep-beep').
  uint32_t tGo = millis(), tPlug = 0;
  while (true) {
    uint32_t now = millis();
    if (tPlug == 0 && readVbat() > 6.0f) { tPlug = now; Serial.println(F("   LiPo detected.")); }
    if (tPlug != 0 && now - tPlug >= 3000) break;
    if (tPlug == 0 && now - tGo >= 4000) break;
    if (Serial.available() || bootPressed()) break;
    delay(10);
  }
  allMotors(CAL_MIN);                  // drop to STOP right away...
  flushSerial();                       // ...then tidy up the key / button
  while (bootPressed()) delay(10);

  Serial.println(F("3) Signal is now STOP (1000 us). What did you hear?"));
  Serial.println(F("   'beep-beep', then 3 short beeps + 1 long beep  -> calibration SAVED."));
  Serial.println(F("   only the normal start-up tones                -> the ESC missed FULL:"));
  Serial.println(F("        type c again and plug in exactly at NOW."));
  Serial.println(F("   a melody, then beep patterns that repeat     -> PROGRAMMING MODE:"));
  Serial.println(F("        unplug the LiPo, type c again, plug in a little LATER after NOW."));
  Serial.println(F("4) Unplug the LiPo, wait 5 s, plug it back in, listen for: melody, 3 beeps, long beep."));
}

// ==============================================================
//  Wiring check: one signal wire at a time gets a SAFE "motor off"
//  pulse (1000 us at 250 Hz); the others get nothing. A multimeter on
//  DC volts reads about 0.8 V on the active wire and 0 V on the others.
// ==============================================================
void pinCheck() {
  Serial.println(F("\n=========== WIRING CHECK (multimeter) ==========="));
  Serial.println(F("Best with the LiPo UNPLUGGED. Multimeter on DC volts (2 V or 20 V range)."));
  Serial.println(F("Black probe on H6 pin 5 (GND). Red probe on the pin named below."));
  Serial.println(F("Expect about 0.8 V on that pin, and 0 V on the other three."));
  Serial.println(F("Then measure at that ESC's plug too: white/yellow wire vs black/brown wire."));
  Serial.println(F("Enter = next wire, q = quit."));

  const uint32_t savedFreq = escFreq;
  setEscFrequency(250);
  for (uint8_t m = 0; m < 4; m++) {
    allNoSignal();
    escWriteUs(m, 1000);
    Serial.printf("\n  NOW ACTIVE: M%u  (GPIO %u, H6 pin %u)  -> expect ~0.8 V on H6 pin %u only\n",
                  m + 1, MOTOR_PINS[m], m + 1, m + 1);
    if (waitKeyOrBoot(0) == 'q') break;
  }
  setEscFrequency(savedFreq);
  Serial.println(F("\nWiring check finished. Signals are back to normal."));
  Serial.println(F("  0 V on the active pin  -> broken trace/solder joint between the ESP32 and H6."));
  Serial.println(F("  0.8 V on the wrong pin -> the plug or the wires are in the wrong order."));
  Serial.println(F("  Right at H6 but 0 V at the ESC plug -> plug reversed or bad crimp."));
  Serial.println(F("  Also check (LiPo unplugged, beep mode): H6 pin 5 <-> PDB minus must BEEP."));
}

// ==============================================================
//  Beep decoder (Hobbywing / XXD / SimonK-style ESCs)
// ==============================================================
void printBeeps() {
  Serial.println(F("\n=========== WHAT THE ESC BEEPS MEAN ==========="));
  Serial.println(F("Melody, then 3 short beeps, then 1 long beep, then SILENCE"));
  Serial.println(F("   -> NORMAL. Armed and ready (3 beeps = 3S battery). Motors will spin."));
  Serial.println(F("One beep every ~2 seconds, forever"));
  Serial.println(F("   -> NO VALID SIGNAL. Check the ESC signal wire on H6 and ESC ground on H6 pin 5,"));
  Serial.println(F("      run p (wiring check), try another speed with f, then unplug/replug the LiPo."));
  Serial.println(F("Fast beeps (about 4 per second), forever"));
  Serial.println(F("   -> THROTTLE NOT AT ZERO: the ESC thinks 'off' is too high."));
  Serial.println(F("      Run c (calibration). Or lower the off pulse with o, then replug the LiPo."));
  Serial.println(F("'beep-beep' pairs, about one pair per second"));
  Serial.println(F("   -> BATTERY VOLTAGE WRONG (LiPo too low, or the ESC's battery setting changed)."));
  Serial.println(F("      Measure the LiPo: a charged 3S reads 11.1-12.6 V. Charge it."));
  Serial.println(F("'beep-beep', then a melody, then beep patterns that repeat"));
  Serial.println(F("   -> PROGRAMMING MODE (the ESC saw full throttle too long at power-up)."));
  Serial.println(F("      Unplug the LiPo. Nothing is spinning; just redo c if you were calibrating."));
  Serial.println(F("No sound at all"));
  Serial.println(F("   -> No power to the ESC: check its thick red/black wires on the PDB."));
}

// ==============================================================
//  Menu / status
// ==============================================================
void printMenu() {
  Serial.println(F("\n=========== SKETCH 1: MOTOR TEST (v2) ==========="));
  Serial.println(F("  !!! PROPS OFF (until guide Step 6) !!!"));
  Serial.println(F("  c     ESC calibration"));
  Serial.println(F("  1-4   spin one motor          a  all 4 together"));
  Serial.println(F("  s     M1 -> M4 in sequence    + -  test throttle"));
  Serial.println(F("  f     ESC signal speed        o  off pulse"));
  Serial.println(F("  p     wiring check            b  what the beeps mean"));
  Serial.println(F("  i     status                  h  this menu"));
  Serial.println(F("  STOP: any key, BOOT button, or unplug the LiPo"));
}

void printStatus() {
  Serial.println(F("\n--- STATUS ---"));
  Serial.printf("ESC signal    : %lu Hz, off pulse %u us, test throttle %u us (max %u)\n",
                (unsigned long)escFreq, pwmOff, testThrottle, TEST_MAX);
  for (uint8_t m = 0; m < 4; m++)
    Serial.printf("  M%u -> GPIO %u : %s\n", m + 1, MOTOR_PINS[m],
                  attachOk[m] ? "signal running" : "LEDC SETUP FAILED (report this)");
  float v = readVbat();
  if (v > 6.0f) Serial.printf("LiPo (via H5) : %.2f V  (%.2f V per cell)\n", v, v / 3.0f);
  else          Serial.println(F("LiPo (via H5) : not seen (unplugged, or H5 not wired - that's OK)"));
  if (!bootStopEnabled) Serial.println(F("BOOT button   : reads 'pressed' at start-up -> BOOT stop disabled"));
}

void setup() {
  escInit();                          // FIRST: ESCs see "motor off" immediately
  pinMode(BOOT_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  Serial.begin(115200);
  delay(1500);
  bootStopEnabled = (digitalRead(BOOT_BUTTON) == HIGH);
  printMenu();
  printStatus();
  Serial.println(F("\nNow plug in the LiPo and LISTEN."));
  Serial.println(F("Normal = melody, 3 beeps, 1 long beep, then silence. Then type 1."));
  Serial.println(F("Beeping that never stops? Type b."));
}

void loop() {
  allMotors(pwmOff);
  lipoWatch();
  if (!Serial.available()) { delay(5); return; }

  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;
  flushSerial();

  switch (c) {
    case 'c': calibrateEscs(); break;
    case '1': case '2': case '3': case '4': spinOne(c - '1'); break;
    case 'a': spinAll(); break;
    case 's': spinSequence(); break;
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
    case 'f': {
      freqIndex = (freqIndex + 1) % FREQ_COUNT;
      bool ok = setEscFrequency(FREQ_LIST[freqIndex]);
      Serial.printf("ESC signal = %lu Hz%s\n", (unsigned long)escFreq, ok ? "" : "  (LEDC ERROR)");
      Serial.println(F("  Unplug the LiPo, plug it back in, listen, then test with 1-4."));
      Serial.println(F("  Note the FASTEST speed that arms and spins smoothly: sketch 4 needs it."));
      break;
    }
    case 'o':
      offIndex = (offIndex + 1) % OFF_COUNT;
      pwmOff = OFF_LIST[offIndex];
      allMotors(pwmOff);
      Serial.printf("off pulse = %u us. Unplug and replug the LiPo, then listen.\n", pwmOff);
      break;
    case 'p': pinCheck(); break;
    case 'b': printBeeps(); break;
    case 'i': printStatus(); break;
    case 'h': printMenu(); break;
    default:  Serial.println(F("? (h = menu)")); break;
  }
  allMotors(pwmOff);
}
