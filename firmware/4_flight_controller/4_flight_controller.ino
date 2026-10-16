/*
  ==============================================================
   SKETCH 4 (v3) - FLIGHT CONTROLLER
   ESP32-S3 + MPU6050 + BMP280 + FlySky i-BUS + 4 ESCs, Quad X
  ==============================================================
   Flight modes (SwC = CH6, check with sketch 2):
     ~1000  MANUAL    self-levelling, you control the throttle
     ~1500  ALT HOLD  the drone keeps its height by itself (barometer).
                      Throttle stick in the middle = hold, up/down = climb/descend.
     ~2000  LAND      comes down slowly and stops the motors on the ground

   ARM   : SwC on MANUAL, throttle fully DOWN, then SwA (CH5) to ARM (~2000).
   DISARM: SwA to SAFE (~1000) -> the motors stop at once, in every mode (kill switch).

   Automatic safety:
     radio lost in the air ............ LAND (on the ground: motors stop)
     battery critical in the air ...... LAND (you can still steer, not cancel)
     barometer lost in ALT HOLD ....... LAND
     tilt over 70 deg / MPU6050 lost .. motors stop
     throttle down on the ground 15 s . motors stop

   Settings: config.h       Documentation: docs/ in the repository
   Arduino IDE, Tools menu: Board "ESP32S3 Dev Module", Flash Size "16MB (128Mb)",
     Partition Scheme "16M Flash (3MB APP/9.9MB FATFS)", PSRAM "OPI PSRAM".

   Motor layout (seen from above, front at the top):
        M4 (CW)    M2 (CCW)
             \  /
             /  \
        M3 (CCW)   M1 (CW)
*/

#include <Arduino.h>
#include <Wire.h>
#include "esp_log.h"

#include "config.h"
#include "util.h"
#include "rc_ibus.h"
#include "motors.h"
#include "pid.h"
#include "imu_mpu6050.h"
#include "baro_bmp280.h"
#include "altitude.h"
#include "battery.h"
#include "status_beeper.h"
#include "flight_log.h"

#define FC_VERSION "3.0"

// ---------------- Constants ----------------
constexpr uint32_t LOOP_US          = 4000;    // 250 Hz control loop
constexpr uint16_t THR_ARM_MAX      = 1050;    // throttle stick must be below this to arm
constexpr float    CRASH_ANGLE      = 70.0f;   // deg
constexpr uint16_t AIRBORNE_THR     = 1350;    // throttle above this for 0.5 s = taking off
constexpr float    ALT_MIN_FOR_HOLD = 0.5f;    // m: ALT HOLD needs at least this height
constexpr float    GROUND_ALT       = 0.3f;    // m: below this the drone counts as "on the ground"
constexpr float    AUTO_DISARM_S    = 15.0f;   // throttle down on the ground this long -> disarm
constexpr uint32_t LOG_EVERY        = 250 / LOG_HZ;
static_assert(250 % LOG_HZ == 0, "LOG_HZ must divide 250: use 25, 50, 125 or 250");

// ---------------- Flight state ----------------
// Why the drone is landing. A higher number wins (battery locks the mode switch).
enum LandReason : uint8_t { LAND_NONE = 0, LAND_SWITCH, LAND_BARO, LAND_RADIO, LAND_BATTERY };
const char *const LAND_NAMES[] = {"-", "switch", "barometer lost", "radio lost", "battery critical"};

bool armed = false;
bool armSwitchWasDown = false;    // SwA must be seen on SAFE before an arm is accepted
FlightMode flightMode = MODE_MANUAL;
LandReason landReason = LAND_NONE;
FlightMode lastSwitchMode = MODE_MANUAL;   // last accepted SwC position
FlightMode swCandidate = MODE_MANUAL;      // SwC position waiting to be stable
uint32_t swCandSince = 0;
bool radioWasOk = true;
bool airborne = false;
float airTimer = 0, idleTimer = 0;
float lastThrOut = MOTOR_OFF;              // collective throttle sent to the mixer (us)
float wantRollLog = 0, wantPitchLog = 0;   // wanted angles, for the log
float blindThr = HOVER_THROTTLE_GUESS;     // LAND without barometer: fixed throttle
bool hoverSeeded = false;
BattState lastBattState = BATT_OK;
bool lostBeeper = false;                   // after a crash / radio loss: beep to find the drone
bool statusPrint = true;

// landing detector (LAND, or ALT HOLD with the throttle stick fully down)
float landTime = 0, landedTimer = 0, minThrTimer = 0, lowAltTimer = 0, blindTime = 0;
double landThrSum = 0;
uint32_t landThrN = 0;
bool groundContact = false;

// flight summary
uint32_t armTimeMs = 0, flightTimeMs = 0;
float maxAlt = 0, minCellV = 9.9f;

// ---------------- Flight notes ----------------
// Events during a flight are kept here and printed after landing
// (printing while armed could pause the control loop when USB is unplugged).
constexpr uint8_t MAX_NOTES = 20;
struct Note { uint32_t ms; char text[60]; };
Note notes[MAX_NOTES];
uint8_t noteCount = 0;

void note(const char *fmt, ...) {
  char buf[60];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (armed && noteCount < MAX_NOTES) {               // kept for the flight summary
    notes[noteCount].ms = millis() - armTimeMs;
    memcpy(notes[noteCount].text, buf, sizeof(buf));
    noteCount++;
  }
  if (!armed && DEBUG) Serial.println(buf);          // on the ground: print right away
}

void printFlightSummary() {
  if (!DEBUG || armTimeMs == 0) return;
  Serial.printf("\n---- Last flight: %.1f s", flightTimeMs / 1000.0f);
  if (maxAlt > 0.05f) Serial.printf(", highest %.1f m", maxAlt);
  if (minCellV < 9.0f) Serial.printf(", lowest %.2f V/cell under load, now %d%%", minCellV, battPct);
  if (CURRENT_PIN >= 0) Serial.printf(", %.0f mAh used", mAhUsed);
  Serial.println(F(" ----"));
  for (uint8_t i = 0; i < noteCount; i++)
    Serial.printf("  %7.1f s  %s\n", notes[i].ms / 1000.0f, notes[i].text);
  if (noteCount == MAX_NOTES) Serial.println(F("  (later events not kept)"));
}

// =================================================================
//  Arm / disarm
// =================================================================
// Returns why arming is not possible right now, or nullptr when ready.
const char *armProblem() {
  if (!radioOk())                                   return "no radio signal";
  if (!imuOk)                                       return "MPU6050 not found";
  if (rc[CH_THR] > THR_ARM_MAX)                     return "throttle not fully down";
  if (decodeModeSwitch(rc[CH_MODE]) != MODE_MANUAL) return "SwC (CH6) not on MANUAL";
  if (fabsf(accRoll) > 25 || fabsf(accPitch) > 25)  return "drone not level";
  if (battPresent && vCell < CELL_MIN_ARM_V)        return "battery too low, charge it";
  return nullptr;
}

void disarm(const char *why) {
  if (!armed) return;
  motorsOff();                       // first of all
  note("DISARMED: %s", why);         // still "armed" here -> kept for the summary
  armed = false;
  flightTimeMs = millis() - armTimeMs;
  pidResetAll();
  armSwitchWasDown = false;          // SwA must go back to SAFE before the next arm
  airborne = false;
  flightMode = MODE_MANUAL;
  landReason = LAND_NONE;
  logStop();
  beep(2);
  printFlightSummary();
}

void tryArm() {
  if (!radioOk()) return;
  if (rc[CH_ARM] <= 1700) { armSwitchWasDown = true; return; }
  if (!armSwitchWasDown) return;     // switch was already on ARM: wait until it is flipped
  armSwitchWasDown = false;          // the switch just went to ARM

  const char *problem = armProblem();
  if (problem) {
    note("NO ARM: %s. SwA to SAFE, fix it, try again.", problem);
    beep(3, 60, 60);
    return;
  }
  pidResetAll();
  altZero();                         // height 0 = here
  battResetFlight();
  lastBattState = BATT_OK;
  flightMode = MODE_MANUAL;
  landReason = LAND_NONE;
  lastSwitchMode = swCandidate = MODE_MANUAL;
  airborne = false;
  airTimer = idleTimer = 0;
  radioWasOk = true;
  lostBeeper = false;
  maxAlt = 0;
  minCellV = 9.9f;
  noteCount = 0;
  armTimeMs = millis();
  if (DEBUG) Serial.println(F("ARMED"));
  logStart();
  armed = true;
  beep(1);
}

// =================================================================
//  Modes and automatic landing
// =================================================================
// Throttle that should hold the drone in the air, when leaving MANUAL
float manualHoverBase() {
  return hoverLearnedValid ? hoverThrLearned : lastThrOut;
}

bool onGround() {
  return !airborne || (altValid && altEst < GROUND_ALT);
}

void resetLandDetector() {
  landTime = landedTimer = minThrTimer = lowAltTimer = blindTime = 0;
  landThrSum = 0;
  landThrN = 0;
  groundContact = false;
}

void startLand(LandReason why) {
  if (flightMode == MODE_LAND) {                     // already landing: maybe a stronger reason
    if (why > landReason) { landReason = why; note("landing reason: %s", LAND_NAMES[why]); }
    return;
  }
  if (onGround()) {                                  // nothing to land: just stop
    static const char *const ON_GROUND[] = {"-", "LAND switch on the ground", "barometer lost on the ground",
                                            "radio lost on the ground", "battery critical on the ground"};
    disarm(ON_GROUND[why]);
    return;
  }
  if (flightMode == MODE_ALT) {
    blindThr = hoverThr + velI - LAND_NOBARO_DROP;   // the speed loop keeps its state
  } else {
    const float base = manualHoverBase();
    altCtrlReset(base);
    blindThr = base - LAND_NOBARO_DROP;
  }
  flightMode = MODE_LAND;
  landReason = why;
  resetLandDetector();
  note("LAND (%s)%s", LAND_NAMES[why], altValid ? "" : " without barometer");
  beep(1);
}

void onModeSwitch(FlightMode want) {
  if (landReason == LAND_BATTERY) {                 // battery landing cannot be cancelled
    note("SwC ignored: battery landing");
    beep(3, 60, 60);
    return;
  }
  switch (want) {
    case MODE_MANUAL:
      if (flightMode == MODE_MANUAL) return;
      flightMode = MODE_MANUAL;
      landReason = LAND_NONE;
      note("MANUAL");
      beep(1);
      break;

    case MODE_ALT:
      if (!ALT_HOLD_ENABLED) { note("ALT HOLD refused: disabled in config.h"); beep(3, 60, 60); return; }
      if (!altValid)         { note("ALT HOLD refused: no barometer"); beep(3, 60, 60); return; }
      if (!airborne || altEst < ALT_MIN_FOR_HOLD) {
        note("ALT HOLD refused: climb above %.1f m first", ALT_MIN_FOR_HOLD);
        beep(3, 60, 60);
        return;
      }
      if (flightMode == MODE_LAND) altTarget = altEst;                     // keep the speed loop
      else altCtrlReset(hoverLearnedValid ? hoverThrLearned : lastThrOut);
      flightMode = MODE_ALT;
      landReason = LAND_NONE;
      note("ALT HOLD at %.1f m", altEst);
      beep(1);
      break;

    case MODE_LAND:
      startLand(LAND_SWITCH);
      break;
  }
}

// Hover throttle: the average throttle while flying level and steady.
// Used as the starting point when ALT HOLD or LAND begins.
void learnHover(float dt) {
  if (!airborne || flightMode == MODE_LAND || lastThrOut < THR_ARM_MAX) return;
  if (fabsf(angleRoll) > 15 || fabsf(anglePitch) > 15) return;
  if (altValid && fabsf(velEst) > 0.3f) return;
  const float thrLevel = 1000.0f + (lastThrOut - 1000.0f) * tiltCos();
  if (!hoverSeeded) { hoverThrLearned = thrLevel; hoverSeeded = true; }
  hoverThrLearned += (dt / 3.0f) * (thrLevel - hoverThrLearned);   // ~3 s average
  hoverLearnTime += dt;
  if (hoverLearnTime > 2.0f) hoverLearnedValid = true;
}

// Landing detector (idea from INAV): the controller pushes the throttle down
// but the drone does not move any more -> it is on the ground.
void landDetect(float dt) {
  landTime += dt;
  if (!altValid) {                                   // no barometer: fixed time
    blindTime += dt;
    if (blindTime >= LAND_NOBARO_TIME_S) disarm("landing finished (no barometer: timer)");
    return;
  }
  if (landTime > 1.0f) { landThrSum += lastThrOut; landThrN++; }
  const float avgThr = landThrN ? (float)(landThrSum / landThrN) : lastThrOut;
  const bool still = fabsf(velEst) < 0.25f && fabsf(rollRateF) < 20 &&
                     fabsf(pitchRateF) < 20 && fabsf(yawRateF) < 20;
  groundContact = landThrN > 0 && still && lastThrOut < avgThr - 20.0f;
  const bool pushedDown = landThrN > 0 && lastThrOut < avgThr - 40.0f;
  // probably touching the ground: lower the throttle faster so it does not skate around.
  // (in the air the drone would start to sink, "still" turns false and this stops)
  if (groundContact) velI = fmaxf(velI - 150.0f * dt, -VEL_I_MAX);

  landedTimer = (landTime > 2.0f && still && pushedDown) ? landedTimer + dt : 0.0f;
  minThrTimer = (lastThrOut <= MOTOR_IDLE + 31 && fabsf(velEst) < 0.5f) ? minThrTimer + dt : 0.0f;
  if (altEst < LAND_SLOW_ALT) lowAltTimer += dt;

  if (landedTimer >= 1.0f)               disarm("landed");
  else if (minThrTimer >= 3.0f)          disarm("landed (throttle at minimum)");
  else if (lowAltTimer >= LAND_TIMEOUT_S) disarm("landing timeout");
}

void updateAirborne(float dt) {
  const bool lifting = lastThrOut >= AIRBORNE_THR ||
                       (altValid && altEst > 0.7f && lastThrOut > MOTOR_IDLE + 100);
  airTimer = lifting ? airTimer + dt : 0.0f;
  if (!airborne && airTimer >= 0.5f) { airborne = true; note("airborne"); }

  // MANUAL with the throttle stick down: on the ground (it cannot fly at idle)
  idleTimer = (flightMode == MODE_MANUAL && rc[CH_THR] < THR_ARM_MAX) ? idleTimer + dt : 0.0f;
  if (airborne && idleTimer >= 1.0f) { airborne = false; note("on the ground"); }
  if (!airborne && idleTimer >= AUTO_DISARM_S) disarm("throttle down for 15 s");
}

// Everything that can change the flight mode or stop the motors. Runs every loop while armed.
void updateFlightState(float dt) {
  const bool radio = radioOk();
  const uint32_t now = millis();

  // 1) kill switch: works in every mode
  if (radio && rc[CH_ARM] <= 1700) { disarm("SwA to SAFE (kill switch)"); return; }

  // 2) crash or sensor failure
  if (fabsf(angleRoll) > CRASH_ANGLE || fabsf(anglePitch) > CRASH_ANGLE) {
    lostBeeper = airborne;
    disarm("crash: tilted more than 70 deg");
    return;
  }
  if (imuErrors > 20) { lostBeeper = airborne; disarm("MPU6050 stopped answering"); return; }

  // 3) in the air or on the ground?
  updateAirborne(dt);
  if (!armed) return;

  // 4) radio lost / back
  if (!radio && radioWasOk) {
    radioWasOk = false;
    if (airborne) lostBeeper = true;
    startLand(LAND_RADIO);                           // on the ground this disarms
    if (!armed) return;
  }
  if (radio && !radioWasOk) {
    radioWasOk = true;
    lastSwitchMode = swCandidate = decodeModeSwitch(rc[CH_MODE]);
    note("radio back: move SwC to take over");
  }

  // 5) battery
  if (battState != lastBattState) {
    lastBattState = battState;
    note("battery %s: %.2f V/cell (%d%%)", battState == BATT_CRITICAL ? "CRITICAL" : "LOW", vCell, battPct);
  }
  if (battPresent && battState == BATT_CRITICAL && airborne && landReason != LAND_BATTERY) {
    startLand(LAND_BATTERY);
    if (!armed) return;
  }

  // 6) barometer lost while holding height
  if (flightMode == MODE_ALT && !altValid) startLand(LAND_BARO);

  // 7) mode switch. MANUAL and LAND act at once; the middle (ALT HOLD) must be
  //    stable for 0.15 s (a quick flip from one end to the other passes the middle).
  if (radio) {
    const FlightMode sw = decodeModeSwitch(rc[CH_MODE]);
    if (sw != swCandidate) { swCandidate = sw; swCandSince = now; }
    const bool stable = (sw != MODE_ALT) || (now - swCandSince >= 150);
    if (sw != lastSwitchMode && stable) {
      lastSwitchMode = sw;
      onModeSwitch(sw);
      if (!armed) return;
    }
  }

  // 8) landing detector: in LAND, or in ALT HOLD with the throttle stick fully down
  const bool descending = flightMode == MODE_LAND ||
                          (flightMode == MODE_ALT && radio && rc[CH_THR] < 1100);
  if (descending) landDetect(dt);
  else resetLandDetector();
  if (!armed) return;

  // 9) learn the hover throttle
  learnHover(dt);
}

// =================================================================
//  Flight control: throttle + sticks -> PIDs -> motor mix
// =================================================================
void flightControl(float dt) {
  if (!armed) { motorsOff(); lastThrOut = MOTOR_OFF; return; }
  const bool radio = radioOk();

  // 1) collective throttle
  float thr;
  if (flightMode == MODE_MANUAL) {
    const uint16_t stickThr = constrain(rc[CH_THR], 1000, THROTTLE_MAX);
    if (stickThr < THR_ARM_MAX) {            // throttle down: idle, no corrections
      pidResetAll();
      motorsAll(MOTOR_IDLE);
      lastThrOut = MOTOR_IDLE;
      wantRollLog = wantPitchLog = 0;
      return;
    }
    thr = stickThr;
  } else if (altValid) {                     // ALT HOLD and LAND: the height controller decides
    thr = altCtrlUpdate(flightMode == MODE_ALT ? altHoldVelTarget() : landVelTarget(), dt);
    if (flightMode == MODE_LAND) blindThr = hoverThr + velI - LAND_NOBARO_DROP;   // in case the baro fails
  } else {                                   // LAND without barometer: a bit under hover
    thr = clampf(blindThr, MOTOR_IDLE + 30.0f, THROTTLE_MAX);
    thr = 1000.0f + (thr - 1000.0f) / fmaxf(tiltCos(), 0.7f);
  }
  lastThrOut = thr;

  // 2) sticks -> wanted angles and yaw rate (centred when the radio is lost)
  float sRoll = 0, sPitch = 0, sYaw = 0;
  if (radio) { sRoll = stick(rc[CH_ROLL]); sPitch = stick(rc[CH_PITCH]); sYaw = stick(rc[CH_YAW]); }
  const float wantRoll  =  sRoll  * MAX_ANGLE_DEG;   // right stick right   -> right side down
  const float wantPitch = -sPitch * MAX_ANGLE_DEG;   // right stick forward -> nose down
  const float wantYaw   =  sYaw   * MAX_YAW_RATE;    // left stick right    -> nose right
  wantRollLog = wantRoll;
  wantPitchLog = wantPitch;

  // 3) angle error -> wanted rotation speed (self-levelling)
  const float rollRateSet  = clampf(LEVEL_GAIN * (wantRoll  - angleRoll),  -250.0f, 250.0f);
  const float pitchRateSet = clampf(LEVEL_GAIN * (wantPitch - anglePitch), -250.0f, 250.0f);

  // 4) no I-term wind-up on the ground (it can flip the drone at take-off or touchdown)
  if (!airborne) {
    pidRoll.i = pidPitch.i = pidYaw.i = 0;
  } else if (groundContact) {
    pidRoll.i *= 0.95f; pidPitch.i *= 0.95f; pidYaw.i *= 0.95f;
  }

  // 5) rate PIDs and mixer
  const float outRoll  = pidRun(pidRoll,  rollRateSet  - rollRateF,  P_ROLL, I_ROLL, D_ROLL);
  const float outPitch = pidRun(pidPitch, pitchRateSet - pitchRateF, P_ROLL, I_ROLL, D_ROLL);
  const float outYaw   = pidRun(pidYaw,   wantYaw      - yawRateF,   P_YAW,  I_YAW,  D_YAW);
  motorsMix(thr, outRoll, outPitch, outYaw);
}

// =================================================================
//  Flight log record (50 times per second while armed)
// =================================================================
int16_t sat16(float v) { return (int16_t)lrintf(clampf(v, -32767.0f, 32767.0f)); }

void logRecord() {
  LogRec r;
  r.t_ms = millis();
  r.roll_cd = sat16(angleRoll * 100);
  r.pitch_cd = sat16(anglePitch * 100);
  r.rollSp_cd = sat16(wantRollLog * 100);
  r.pitchSp_cd = sat16(wantPitchLog * 100);
  r.yawRate_dd = sat16(yawRateF * 10);
  r.alt_cm = sat16(altEst * 100);
  r.altTgt_cm = sat16(flightMode == MODE_ALT ? altTarget * 100 : altEst * 100);
  r.baroAlt_cm = sat16(baroAlt * 100);
  r.vz_cms = sat16(velEst * 100);
  r.thrIn = rc[CH_THR];
  r.thrOut = (uint16_t)lastThrOut;
  for (uint8_t m = 0; m < 4; m++) r.m[m] = motorOut[m];
  r.vbat_mv = (uint16_t)(vbat * 1000);
  r.vrest_mv = (uint16_t)(vRest * 1000);
  r.amps_ca = sat16(battAmps * 100);
  r.battPct = battPct < 0 ? 0 : (uint8_t)battPct;
  r.spare = 0;
  r.mode = flightMode;
  uint8_t f = LOGF_ARMED;
  if (airborne) f |= LOGF_AIRBORNE;
  if (radioOk()) f |= LOGF_RADIO_OK;
  if (battState == BATT_LOW) f |= LOGF_BATT_LOW;
  if (battState == BATT_CRITICAL) f |= LOGF_BATT_CRIT;
  if (baroOk) f |= LOGF_BARO_OK;
  if (landReason == LAND_RADIO) f |= LOGF_LAND_RADIO;
  if (landReason == LAND_BATTERY) f |= LOGF_LAND_BATT;
  r.flags = f;
  logAdd(r);
}

// =================================================================
//  Buzzer + LED
// =================================================================
void statusUpdate(uint32_t now) {
  // repeating beeps, the most important first
  BeepRepeat rep = REP_NONE;
  if (armed && flightMode == MODE_LAND)                rep = REP_LANDING;
  else if (battPresent && battState == BATT_CRITICAL)  rep = REP_BATT_CRIT;
  else if (battPresent && battState == BATT_LOW)       rep = REP_BATT_LOW;
  else if (!armed && lostBeeper)                       rep = REP_LOST;
  beepRepeat = rep;
  beepUpdate(now);

  const bool blink = (now / 250) % 2;
  const bool battLow = battPresent && battState == BATT_LOW;
  if (!imuOk || (battPresent && battState == BATT_CRITICAL)) {
    ledWrite(blink ? 60 : 0, 0, 0);                           // red blink: problem
  } else if (armed) {
    if (flightMode == MODE_LAND)       ledWrite(blink ? 60 : 0, blink ? 20 : 0, 0);  // orange blink
    else if (battLow && blink)         ledWrite(50, 40, 0);   // yellow flashes: battery low
    else if (flightMode == MODE_ALT)   ledWrite(35, 0, 50);   // purple: ALT HOLD
    else                               ledWrite(0, 0, 60);    // blue: MANUAL
  } else {
    if (battLow && blink)              ledWrite(50, 40, 0);
    else if (armProblem())             ledWrite(0, blink ? 40 : 0, 0);   // green blink: not ready
    else                               ledWrite(0, 40, 0);    // green: ready to arm
  }
}

// =================================================================
//  Serial Monitor (115200 baud) - only while DISARMED
// =================================================================
void printHelp() {
  Serial.println(F("\nCommands (only while DISARMED; type, then Enter):"));
  Serial.println(F("  h    this help"));
  Serial.println(F("  i    info: sensors, battery, hover throttle, log storage, last flight"));
  Serial.println(F("  s    live status lines on/off"));
  Serial.println(F("  l    list saved flight logs"));
  Serial.println(F("  d    print the newest log as CSV   (d3 = log 3)"));
  Serial.println(F("  X    erase ALL logs"));
  Serial.println(F("  z    set height 0 here"));
  Serial.println(F("tools/flight_log.py downloads and plots the logs for you.\n"));
}

void printInfo() {
  Serial.printf("\n---- Flight controller v%s ----\n", FC_VERSION);
  Serial.printf("MPU6050 ........ %s\n", imuOk ? "OK" : "NOT FOUND (run sketch 3)");
  if (baroOk)
    Serial.printf("BMP280 ......... OK at 0x%02X, %.2f hPa, %.1f C, height %+.2f m\n",
                  bmpAddr, baroPressurePa / 100.0f, baroTempC, altEst);
  else
    Serial.printf("BMP280 ......... %s -> no ALT HOLD, landing without height control\n",
                  ALT_HOLD_ENABLED ? "NOT FOUND" : "off in config.h");
  if (battPresent)
    Serial.printf("Battery ........ %.2f V = %.2f V/cell, %d%% (%s), sag %.2f V per unit load\n", vbat, vCell,
                  battPct, battState == BATT_CRITICAL ? "CRITICAL" : battState == BATT_LOW ? "LOW" : "OK", sagK);
  else
    Serial.printf("Battery ........ no reading (%.2f V): LiPo unplugged, or check H5 / VBAT_CAL\n", vbat);
  Serial.printf("Hover throttle . %.0f us (%s)\n", hoverThrLearned,
                hoverLearnedValid ? "learned in flight" : "guess from config.h");
  Serial.printf("ESC signal ..... %d Hz, off %d us, idle %d us\n", ESC_FREQ_HZ, MOTOR_OFF, MOTOR_IDLE);
  if (logCap)
    Serial.printf("Log memory ..... %s, room for %.1f min at %d Hz\n",
                  logCap > 5000 ? "PSRAM" : "small RAM buffer (enable Tools > PSRAM > OPI PSRAM)",
                  logCap / (float)LOG_HZ / 60.0f, LOG_HZ);
  else
    Serial.println(F("Log memory ..... none (logging off)"));
  if (fsOk) {
    int newest, oldest, count;
    logScan(newest, oldest, count);
    Serial.printf("Log flash ...... %d saved, %lu KB free\n", count, (unsigned long)(FFat.freeBytes() / 1024));
  } else {
    Serial.println(F("Log flash ...... NOT AVAILABLE: Tools > Partition Scheme > 16M Flash (3MB APP/9.9MB FATFS)"));
  }
  printFlightSummary();
  Serial.println();
}

void printStatus() {
  const char *problem = armProblem();
  static const char *const SWC[3] = {"MANUAL", "ALT", "LAND"};
  char batt[24];
  if (battPresent) snprintf(batt, sizeof(batt), "%.2f V %d%%", vbat, battPct);
  else snprintf(batt, sizeof(batt), "no LiPo");
  Serial.printf("%s | roll %+5.1f pitch %+5.1f | THR %4u SwA %-4s SwC %-6s | h %+5.2f m | %s%s%s\n",
                problem ? "WAIT " : "READY", angleRoll, anglePitch, rc[CH_THR],
                rc[CH_ARM] > 1700 ? "ARM" : "SAFE", SWC[decodeModeSwitch(rc[CH_MODE])],
                altValid ? altEst : 0.0f, batt, problem ? "  <- " : "", problem ? problem : "");
}

void runCommand(const char *cmd) {
  switch (cmd[0]) {
    case 'h': case '?': printHelp(); break;
    case 'i': printInfo(); break;
    case 's':
      statusPrint = !statusPrint;
      Serial.printf("Live status %s\n", statusPrint ? "ON" : "OFF");
      break;
    case 'l': logList(); break;
    case 'd': logDump(cmd[1] ? atoi(cmd + 1) : -1); break;
    case 'X': logEraseAll(); break;
    case 'z': altZero(); Serial.println(F("Height set to 0 here.")); break;
    default: Serial.println(F("Unknown command. Type h for help."));
  }
}

void handleSerial() {
  static char line[16];
  static uint8_t len = 0;
  static uint32_t lastCharMs = 0;
  while (Serial.available()) {
    const char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (len) { line[len] = 0; runCommand(line); len = 0; }
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
  if (len && millis() - lastCharMs > 150) { line[len] = 0; runCommand(line); len = 0; }   // "No line ending"
}

void saveLogNow() {
  ledWrite(40, 40, 40);                                // white: saving, do not unplug
  if (DEBUG) Serial.println(F("Saving the flight log to flash... wait for the long beep."));
  if (!logSave() && DEBUG) Serial.println(F("Log NOT saved (type i)."));
  beep(1, 500, 100);                                   // long beep = ready again
}

// =================================================================
//  Setup / loop
// =================================================================
void setup() {
  escInit();                          // FIRST: the ESCs must see "off" right away
  statusBegin();
  ledWrite(40, 40, 0);                // yellow: starting
  Serial.begin(115200);
  rcBegin();
  analogReadResolution(12);
  esp_log_level_set("i2c.master", ESP_LOG_NONE);   // no driver error spam on the serial port
  esp_log_level_set("i2c", ESP_LOG_NONE);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  Wire.setTimeOut(3);
  delay(1000);

  if (DEBUG) Serial.printf("\n=========== SKETCH 4: FLIGHT CONTROLLER v%s ===========\n", FC_VERSION);
  imuOk = mpuInit();
  if (imuOk) {
    calibrateGyro();
    readImu();
    angleRoll = accRoll;
    anglePitch = accPitch;
  } else if (DEBUG) {
    Serial.println(F("MPU6050 NOT FOUND -> arming is blocked (run sketch 3)."));
  }

  baroOk = ALT_HOLD_ENABLED && baroInit();
  altZero();
  if (DEBUG) {
    if (baroOk) Serial.printf("BMP280 OK at 0x%02X (%.1f hPa)\n", bmpAddr, baroPressurePa / 100.0f);
    else Serial.println(F("BMP280 not used: no ALT HOLD, landing without height control."));
  }

  if (DEBUG) Serial.println(F("Log storage... (the very first start can take 30 s)"));
  logBegin();
  for (uint8_t i = 0; i < 10; i++) battUpdate(0.1f, false);

  if (DEBUG) {
    printInfo();
    Serial.println(F("Ready. To arm: SwC on MANUAL, throttle DOWN, then SwA to ARM.  Type h for help."));
  }
  beep(1, 500, 100);                  // long beep = ready
}

void loop() {
  static uint32_t loopTimer = micros();
  static uint32_t loopCount = 0;
  static float battDt = 0;

  // run at a steady 250 Hz
  while (micros() - loopTimer < LOOP_US) { }
  float dt = (micros() - loopTimer) / 1e6f;
  loopTimer = micros();
  if (dt > 0.02f) dt = 0.02f;
  loopCount++;
  const uint32_t now = millis();

  // sensors
  rcUpdate();
  accTrust = armed ? 0.002f : 0.02f;                             // see imu_mpu6050.h
  if (imuOk) updateAttitude(dt);
  const bool newBaro = (loopCount % 5 == 0) && baroUpdate();     // 50 Hz
  altUpdate(dt, newBaro);
  battDt += dt;
  if (loopCount % 25 == 0) { battUpdate(battDt, armed); battDt = 0; }   // 10 Hz

  // decisions and motors
  if (armed) updateFlightState(dt);
  else tryArm();
  flightControl(dt);

  if (armed) {
    if (altValid && altEst > maxAlt) maxAlt = altEst;
    if (battPresent && vCellRaw < minCellV) minCellV = vCellRaw;
    if (loopCount % LOG_EVERY == 0) logRecord();
  }
  statusUpdate(now);

  // only on the ground: save the log, Serial Monitor
  if (!armed) {
    if (logPendingSave && shot.remaining == 0) saveLogNow();
    handleSerial();
    if (DEBUG && statusPrint && loopCount % 125 == 0) printStatus();   // every 0.5 s
  }
}
