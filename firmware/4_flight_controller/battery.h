#pragma once
// Battery monitoring: voltage (H5 divider on GPIO 1), optional current sensor,
// percentage, and the LOW / CRITICAL states that trigger warnings and auto-landing.
//
// Voltage divider:  Vpin = Vbat * R4 / (R3 + R4)   ->   Vbat = Vpin * (R3 + R4) / R4
// With R3 = 10k and R4 = 3.3k, a full 3S (12.6 V) gives 3.13 V on the pin.
//
// SAG COMPENSATION (no current sensor needed)
// Under load a LiPo's voltage drops because of its internal resistance, so the raw
// voltage in flight says "40 %" for a battery that is really at 60 %. The drop is
// about proportional to the battery current, and the current is about proportional
// to the motor load  L = sum(u^3)  (u = motor command 0..1; motor power grows with
// speed cubed). Just before arming the battery is resting, so in the first seconds
// of flight we learn  K = (V_rest - V_now) / L  and then estimate the resting voltage
// as  V_rest = V_now + K * L.  Percentage and LOW / CRITICAL use this estimate.
// A hard floor on the raw voltage (CELL_ABS_MIN_V) still protects the battery.

enum BattState : uint8_t { BATT_OK = 0, BATT_LOW = 1, BATT_CRITICAL = 2 };

float vbat = 0.0f;          // V, measured (filtered), sags under load
float vRest = 0.0f;         // V, estimated resting voltage (sag compensated)
float vCell = 0.0f;         // V per cell, from vRest
float vCellRaw = 0.0f;      // V per cell, measured
float battAmps = 0.0f;      // A (0 without a current sensor)
float mAhUsed = 0.0f;
int battPct = -1;           // 0..100 from vRest, -1 = unknown
bool battPresent = false;   // false when only USB powers the board
BattState battState = BATT_OK;
float lowTimer = 0.0f, critTimer = 0.0f, okTimer = 0.0f;

float loadF = 0.0f;         // motor load, filtered like vbat
float sagK = 0.0f;          // V per unit of load (learned in flight)
float sagLearnT = 0.0f;     // s of learning in this flight
float vRest0 = 0.0f;        // V, resting voltage just before arming

// Resting voltage per cell -> charge (%). Typical LiPo curve.
static const float PCT_V[] = {3.30f, 3.50f, 3.61f, 3.69f, 3.71f, 3.73f, 3.75f, 3.77f, 3.79f, 3.80f, 3.82f,
                              3.84f, 3.85f, 3.87f, 3.91f, 3.95f, 3.98f, 4.02f, 4.08f, 4.11f, 4.15f, 4.20f};
static const uint8_t PCT_P[] = {0, 2, 5, 10, 15, 20, 25, 30, 35, 40, 45,
                                50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};

int lipoPercent(float cellV) {
  const int n = sizeof(PCT_V) / sizeof(PCT_V[0]);
  if (cellV <= PCT_V[0]) return 0;
  if (cellV >= PCT_V[n - 1]) return 100;
  for (int i = 1; i < n; i++) {
    if (cellV < PCT_V[i]) {
      const float f = (cellV - PCT_V[i - 1]) / (PCT_V[i] - PCT_V[i - 1]);
      return (int)(PCT_P[i - 1] + f * (PCT_P[i] - PCT_P[i - 1]) + 0.5f);
    }
  }
  return 100;
}

float motorLoad() {          // ~ battery current, in "motor units" (0..4)
  float s = 0.0f;
  for (uint8_t m = 0; m < 4; m++) {
    const float u = clampf((motorOut[m] - 1050.0f) / 950.0f, 0.0f, 1.0f);
    s += u * u * u;
  }
  return s;
}

// Called 10 times per second. learnSag = armed (the battery was resting just before).
void battUpdate(float dt, bool learnSag) {
  const float v = analogReadMilliVolts(VBAT_PIN) / 1000.0f * VBAT_DIVIDER * VBAT_CAL;
  vbat = (vbat < 1.0f) ? v : vbat + 0.2f * (v - vbat);
  loadF += 0.2f * (motorLoad() - loadF);              // same filter, so both lag together
  battPresent = vbat > 2.5f * BATTERY_CELLS;

  if (CURRENT_PIN >= 0) {
    const float a = (analogReadMilliVolts(CURRENT_PIN) / 1000.0f - CURRENT_OFFSET_V) * CURRENT_A_PER_V;
    battAmps += 0.3f * (a - battAmps);
    if (battAmps > 0.0f) mAhUsed += battAmps * 1000.0f * dt / 3600.0f;
  }
  if (!battPresent) { vRest = vCell = vCellRaw = 0.0f; battPct = -1; lowTimer = critTimer = okTimer = 0.0f; return; }

  // learn the sag during the first 10 s with the motors working (take-off, hover)
  if (learnSag && loadF > 0.25f && vRest0 > 0.0f && sagLearnT < 10.0f) {
    const float k = clampf((vRest0 - vbat) / loadF, 0.0f, 3.0f);
    sagK = (sagLearnT == 0.0f) ? k : sagK + 0.1f * (k - sagK);
    sagLearnT += dt;
  }
  // during the first 2 s of learning K is not ready yet: the battery cannot lose
  // much charge in 2 s, so keep the resting voltage measured before arming
  vRest = (learnSag && sagLearnT > 0.0f && sagLearnT < 2.0f) ? vRest0 : vbat + sagK * loadF;
  vCell = vRest / BATTERY_CELLS;
  vCellRaw = vbat / BATTERY_CELLS;
  battPct = lipoPercent(vCell);

  // A state must last 5 s (short dips on punch-outs are ignored). In flight it stays latched.
  // On the ground (disarmed) a healthy battery for 3 s clears it (fresh pack on the bench).
  lowTimer  = (vCell < CELL_WARN_V) ? lowTimer + dt : 0.0f;
  critTimer = (vCell < CELL_CRITICAL_V || vCellRaw < CELL_ABS_MIN_V) ? critTimer + dt : 0.0f;
  okTimer   = (!learnSag && vCell >= CELL_MIN_ARM_V) ? okTimer + dt : 0.0f;
  if (critTimer >= 5.0f) battState = BATT_CRITICAL;
  else if (lowTimer >= 5.0f && battState == BATT_OK) battState = BATT_LOW;
  else if (okTimer >= 3.0f) battState = BATT_OK;
}

void battResetFlight() {   // at arming (motors still off: the battery is resting)
  battState = BATT_OK;
  lowTimer = critTimer = okTimer = 0.0f;
  mAhUsed = 0.0f;
  vRest0 = vbat;
  sagLearnT = 0.0f;        // sagK keeps the last value until it is learned again
}
