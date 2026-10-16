// =====================================================================================
//  Software-in-the-loop simulator for the flight controller (firmware/4_flight_controller)
// =====================================================================================
//  The REAL flight-controller code is compiled on a PC together with:
//    - a quadcopter model (4 motors with lag, thrust ~ throttle^2, battery sag, ground,
//      ground effect, drag, a centre of gravity slightly off-centre),
//    - register-level models of the MPU6050 and the BMP280 (the barometer data is
//      produced by inverting Bosch's datasheet formulas, so the firmware's own integer
//      formulas are tested too), noise and slow barometer drift,
//    - FlySky i-BUS frames (normal, transmitter off, receiver failsafe values),
//    - a LiPo battery (resting voltage curve + internal resistance),
//    - an in-memory FFat file system, PSRAM, buzzer and RGB LED capture.
//  Each scenario flies the drone and checks the result (PASS / FAIL).
//
//  Build + run everything:   bash run_tests.sh
//  One scenario with a trace: g++ -O2 -std=gnu++17 -I. sim.cpp -o sim && ./sim 1 trace
//  (scenario numbers: see main() at the end; flight logs are written to out/)
#include "Arduino.h"
#include "Wire.h"
#include "FFat.h"
#include <random>
#include <vector>
#include <string>
#include <functional>
#include <algorithm>

uint64_t g_us = 0;
HS Serial, Serial1;
TwoWire Wire;
uint32_t g_duty[64], g_freq[64];
uint8_t g_res[64], g_chpin[16];
bool g_psram = true;
SimFs g_fs;
fs::F_Fat FFat;
void physics();
void sim_delay(uint32_t ms) { uint64_t end = g_us + (uint64_t)ms * 1000; while (g_us < end) { g_us += 500; physics(); } }

// ----------------------------------------------------------------- the flight controller
#include "../../firmware/4_flight_controller/4_flight_controller.ino"

// ----------------------------------------------------------------- world model
std::mt19937 rng(12345);
std::normal_distribution<double> N01(0, 1);
const double G = 9.80665;

struct Drone {
  double mass = 0.85, Tmax = 7.0, armL = 0.117, kq = 0.016, Ixx = 0.0075, Iyy = 0.0075, Izz = 0.013;
  double slopeRoll = 0, slopePitch = 0;            // ground slope, deg
  double z = 0, vz = 0, vx = 0, vy = 0, x = 0, y = 0;
  double phi = 0, th = 0, p = 0, q = 0, r = 0;     // deg, deg/s
  double eff[4] = {1000, 1000, 1000, 1000}, T[4] = {0, 0, 0, 0};
  bool ground = true;
  double axn = 0, ayn = 0, azup = 0;               // world accelerations (NED x,y ; up)
  double impactT = 0, impactAcc = 0;               // touchdown shock for the accelerometer
  double lastTouchdownV = 0; int touchdowns = 0; double maxTiltGround = 0;
  // battery (3S LiPo)
  double cap_mAh = 3300, soc = 1.0, Rint = 0.045, amps = 0, vbat = 12.6;
  double cgTorque = 0.004;                         // N*m: CG a bit off-centre -> needs I-term
} D;

double ocvCell(double soc) {                        // resting cell voltage vs state of charge
  static const double S[] = {0, .03, .06, .10, .15, .20, .30, .40, .50, .60, .70, .80, .90, 1.0};
  static const double V[] = {3.27, 3.50, 3.60, 3.67, 3.70, 3.72, 3.76, 3.80, 3.84, 3.88, 3.94, 4.01, 4.09, 4.19};
  if (soc <= 0) return 3.27;
  if (soc >= 1) return 4.19;
  for (int i = 1; i < 14; i++) if (soc < S[i]) return V[i - 1] + (soc - S[i - 1]) / (S[i] - S[i - 1]) * (V[i] - V[i - 1]);
  return 4.19;
}

double motorUs(int i) {
  int pin = 10 + i;
  if (!g_freq[pin] || !g_duty[pin]) return 0;
  double period = 1e6 / g_freq[pin];
  return g_duty[pin] * period / ((1 << g_res[pin]) - 1);
}

// ---------------- radio ----------------
uint16_t ch[14] = {1500, 1500, 1000, 1500, 1000, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
enum RadioMode { R_ON, R_OFF, R_FAILSAFE } radio = R_ON;
uint64_t lastFrameUs = 0;
void pushFrame() {
  uint16_t v[14]; memcpy(v, ch, sizeof(v));
  if (radio == R_FAILSAFE) { v[0] = v[1] = v[3] = 1500; v[2] = 1000; v[5] = 2000; }   // CH5 holds
  uint8_t f[32] = {0x20, 0x40};
  for (int c = 0; c < 14; c++) { f[2 + 2 * c] = v[c] & 0xFF; f[3 + 2 * c] = v[c] >> 8; }
  uint16_t s = 0xFFFF; for (int i = 0; i < 30; i++) s -= f[i];
  f[30] = s & 0xFF; f[31] = s >> 8;
  for (auto b : f) Serial1.q.push_back(b);
}

// ---------------- BMP280 model (datasheet floating-point formulas, inverted) ----------------
const uint16_t dT1 = 27504; const int16_t dT2 = 26435, dT3 = -1000;
const uint16_t dP1 = 36477; const int16_t dP2 = -10685, dP3 = 3024, dP4 = 2855, dP5 = 140, dP6 = -7, dP7 = 15500, dP8 = -14600, dP9 = 6000;
uint8_t bmpRegs[256];
bool bmpNormal = false, bmpFail = false, mpuFail = false;
double bmpIIR = -1, baroDrift = 0, tempC = 25.0;
uint64_t lastBaroUs = 0;
double compT(int32_t adcT, double &tfine) {
  double v1 = (adcT / 16384.0 - dT1 / 1024.0) * dT2;
  double v2 = (adcT / 131072.0 - dT1 / 8192.0) * (adcT / 131072.0 - dT1 / 8192.0) * dT3;
  tfine = (int32_t)(v1 + v2); return (v1 + v2) / 5120.0;
}
double compP(int32_t adcP, double tfine) {
  double v1 = tfine / 2.0 - 64000.0, v2 = v1 * v1 * dP6 / 32768.0;
  v2 = v2 + v1 * dP5 * 2.0; v2 = v2 / 4.0 + dP4 * 65536.0;
  v1 = (dP3 * v1 * v1 / 524288.0 + dP2 * v1) / 524288.0; v1 = (1.0 + v1 / 32768.0) * dP1;
  double p = 1048576.0 - adcP; p = (p - v2 / 4096.0) * 6250.0 / v1;
  v1 = dP9 * p * p / 2147483648.0; v2 = p * dP8 / 32768.0;
  return p + (v1 + v2 + dP7) / 16.0;
}
void bmpSetData(double pa, double tc) {
  int lo = 0, hi = (1 << 20) - 1; double tf = 0;
  while (lo < hi) { int m = (lo + hi) / 2; if (compT(m, tf) < tc) lo = m + 1; else hi = m; }
  int adcT = lo; compT(adcT, tf);
  lo = 0; hi = (1 << 20) - 1;
  while (lo < hi) { int m = (lo + hi) / 2; if (compP(m, tf) > pa) lo = m + 1; else hi = m; }
  int adcP = lo;
  bmpRegs[0xF7] = adcP >> 12; bmpRegs[0xF8] = (adcP >> 4) & 0xFF; bmpRegs[0xF9] = (adcP & 0xF) << 4;
  bmpRegs[0xFA] = adcT >> 12; bmpRegs[0xFB] = (adcT >> 4) & 0xFF; bmpRegs[0xFC] = (adcT & 0xF) << 4;
}
void bmpReset() {
  memset(bmpRegs, 0, sizeof(bmpRegs));
  bmpRegs[0xD0] = 0x58;
  const uint16_t cal[12] = {dT1, (uint16_t)dT2, (uint16_t)dT3, dP1, (uint16_t)dP2, (uint16_t)dP3, (uint16_t)dP4,
                            (uint16_t)dP5, (uint16_t)dP6, (uint16_t)dP7, (uint16_t)dP8, (uint16_t)dP9};
  for (int i = 0; i < 12; i++) { bmpRegs[0x88 + 2 * i] = cal[i] & 0xFF; bmpRegs[0x89 + 2 * i] = cal[i] >> 8; }
  bmpRegs[0xF7] = 0x80; bmpRegs[0xFA] = 0x80;                            // 0x80000 = no data yet
  bmpNormal = false; bmpIIR = -1;
}
double truePressure() {
  double Ttot = D.T[0] + D.T[1] + D.T[2] + D.T[3];
  double wash = -2.5 * Ttot / (D.mass * G);                              // prop wash on the sensor
  return 101325.0 * pow(1.0 - D.z / 44330.0, 5.255) + baroDrift + wash;
}

// ---------------- MPU6050 model ----------------
uint8_t mpuRegs[128];
uint8_t imuBuf[14];
void put16(uint8_t *b, double v) { int16_t s = (int16_t)fmax(-32768, fmin(32767, lround(v))); b[0] = (uint8_t)(s >> 8); b[1] = (uint8_t)(s & 0xFF); }
void mpuFill() {
  const double R = D.phi * M_PI / 180, Pp = D.th * M_PI / 180;
  const double cf = cos(R), sf = sin(R), ct = cos(Pp), st = sin(Pp);
  // specific force in NED: a - g
  double fx = D.axn, fy = D.ayn, fz = -D.azup - G;
  if (D.impactT > 0) fz -= D.impactAcc;                                   // touchdown shock (upward)
  const double bx = ct * fx - st * fz;
  const double by = st * sf * fx + cf * fy + ct * sf * fz;
  const double bz = st * cf * fx - sf * fy + ct * cf * fz;
  double af = bx / G, al = -by / G, au = -bz / G;                        // forward / left / up, in g
  const double accLsb = 16384.0 / (1 << ((mpuRegs[0x1C] >> 3) & 3));     // from the FC's own setting
  const double gyrLsb = 131.0 / (1 << ((mpuRegs[0x1B] >> 3) & 3));
  const double vib = 0.035;
  put16(imuBuf + 0, (af + vib * N01(rng)) * accLsb);
  put16(imuBuf + 2, (al + vib * N01(rng)) * accLsb);
  put16(imuBuf + 4, (au + vib * N01(rng)) * accLsb);
  put16(imuBuf + 6, 0);
  put16(imuBuf + 8, (D.p + 0.3 * N01(rng)) * gyrLsb + 37);
  put16(imuBuf + 10, (-D.q + 0.3 * N01(rng)) * gyrLsb - 52);
  put16(imuBuf + 12, (-D.r + 0.3 * N01(rng)) * gyrLsb + 11);
}

// ---------------- I2C bus ----------------
bool sim_i2c_present(uint8_t a) {
  if (a == 0x68) return !mpuFail;
  if (a == 0x77) return !bmpFail;
  return false;
}
void sim_i2c_write(uint8_t a, uint8_t reg, const uint8_t *d, int n) {
  if (a == 0x68) { for (int i = 0; i < n; i++) mpuRegs[(reg + i) & 127] = d[i]; return; }
  if (a == 0x77) {
    if (reg == 0xE0 && d[0] == 0xB6) { bmpReset(); return; }
    for (int i = 0; i < n; i++) bmpRegs[reg + i] = d[i];
    if (reg == 0xF4) bmpNormal = (d[0] & 3) == 3;
  }
}
void sim_i2c_read(uint8_t a, uint8_t reg, uint8_t *out, int n) {
  if (a == 0x68) {
    if (reg == 0x3B) { mpuFill(); memcpy(out, imuBuf, n < 14 ? n : 14); return; }
    for (int i = 0; i < n; i++) out[i] = (reg + i == 0x75) ? 0x68 : mpuRegs[(reg + i) & 127];
    return;
  }
  if (a == 0x77) for (int i = 0; i < n; i++) out[i] = bmpRegs[(reg + i) & 255];
}

// ---------------- ADC, buzzer, LED ----------------
uint32_t sim_adc_mv(uint8_t pin) {
  if (pin == 1) return (uint32_t)fmax(0, (D.vbat * 3300.0 / 13300.0) * 1000.0 + 6 * N01(rng));
  return 0;
}
struct Beep { double t0, t1; };
std::vector<Beep> beeps; bool buzOn = false; double buzSince = 0;
void sim_pinMode(uint8_t, uint8_t) {}
void sim_digitalWrite(uint8_t pin, uint8_t v) {
  if (pin != 17) return;
  double t = g_us / 1e6;
  if (v && !buzOn) { buzOn = true; buzSince = t; }
  else if (!v && buzOn) { buzOn = false; beeps.push_back({buzSince, t}); }
}
uint32_t ledNow = 0;
void sim_rgb(uint8_t, uint8_t r, uint8_t g, uint8_t b) { ledNow = (r << 16) | (g << 8) | b; }
int beepsBetween(double a, double b) { int n = 0; for (auto &x : beeps) if (x.t0 >= a && x.t0 < b) n++; return n; }

// ---------------- physics ----------------
bool baroDriftOn = true;
int motorDead = -1;
void physics() {
  while (true) {
    static uint64_t lastPhys = 0;
    if (lastPhys + 500 > g_us) break;
    lastPhys += 500;
    const double dt = 0.0005;
    // motors + battery
    double amps = 0.4, vf = (D.vbat / 12.6) * (D.vbat / 12.6);
    const double ge = 1.0 + 0.08 * fmax(0.0, 1.0 - D.z / 0.25);     // ground effect
    for (int i = 0; i < 4; i++) {
      double cmd = motorUs(i); if (cmd < 900) cmd = 1000;
      D.eff[i] += (cmd - D.eff[i]) * dt / 0.04;
      double u = fmin(1.0, fmax(0.0, (D.eff[i] - 1050.0) / 950.0));
      if (i == motorDead) u = 0;
      D.T[i] = D.Tmax * vf * u * u * ge;
      amps += 17.0 * u * u * u;
    }
    D.amps = amps;
    D.soc -= amps * dt / 3600.0 / (D.cap_mAh / 1000.0);
    D.vbat = 3 * ocvCell(D.soc) - amps * D.Rint;
    const double Ttot = D.T[0] + D.T[1] + D.T[2] + D.T[3];
    const double tr = (D.T[2] + D.T[3] - D.T[0] - D.T[1]) * D.armL + D.cgTorque;
    const double tp = (D.T[1] + D.T[3] - D.T[0] - D.T[2]) * D.armL;
    const double ty = D.kq * (D.T[1] + D.T[2] - D.T[0] - D.T[3]);
    const double R = D.phi * M_PI / 180, Pp = D.th * M_PI / 180;
    double azup = Ttot * cos(R) * cos(Pp) / D.mass - G - 0.3 * D.vz;
    if (D.ground) {
      D.axn = D.ayn = 0; D.vx = D.vy = 0;
      if (azup > 0.05) { D.ground = false; }                          // lift-off
      else {
        azup = 0; D.z = 0; D.vz = 0;
        // tipping: torque bigger than what the landing gear can hold
        const double hold = D.mass * G * 0.10;
        double ar = 0, ap = 0;
        if (fabs(tr) > hold) ar = (tr - copysign(hold, tr)) / D.Ixx * 180 / M_PI;
        if (fabs(tp) > hold) ap = (tp - copysign(hold, tp)) / D.Iyy * 180 / M_PI;
        D.p += ar * dt; D.q += ap * dt;
        if (ar == 0 && fabs(D.phi - D.slopeRoll) < 0.5) D.p = 0;
        if (ap == 0 && fabs(D.th - D.slopePitch) < 0.5) D.q = 0;
        D.phi += D.p * dt; D.th += D.q * dt;
        if (ar == 0 && D.p == 0) D.phi += (D.slopeRoll - D.phi) * 0.02;   // settle on the gear
        if (ap == 0 && D.q == 0) D.th += (D.slopePitch - D.th) * 0.02;
        D.r = 0;
        D.maxTiltGround = fmax(D.maxTiltGround, fmax(fabs(D.phi - D.slopeRoll), fabs(D.th - D.slopePitch)));
      }
    }
    if (!D.ground) {
      D.p += ((tr / D.Ixx) * 180 / M_PI - 0.6 * D.p) * dt;
      D.q += ((tp / D.Iyy) * 180 / M_PI - 0.6 * D.q) * dt;
      D.r += ((ty / D.Izz) * 180 / M_PI - 1.0 * D.r) * dt;
      D.phi += D.p * dt; D.th += D.q * dt;
      D.axn = -Ttot * sin(Pp) * cos(R) / D.mass - 0.35 * D.vx;
      D.ayn = Ttot * sin(R) / D.mass - 0.35 * D.vy;
      D.vx += D.axn * dt; D.vy += D.ayn * dt; D.x += D.vx * dt; D.y += D.vy * dt;
      D.vz += azup * dt; D.z += D.vz * dt;
      if (D.z <= 0 && D.vz < 0) {                                       // touchdown
        D.lastTouchdownV = D.vz; D.touchdowns++;
        D.impactAcc = fmin(30.0, -D.vz / 0.02); D.impactT = 0.02;
        D.z = 0; D.vz = 0; D.ground = true; azup = 0;
        if (fabs(D.phi - D.slopeRoll) < 30) D.p = 0;
        if (fabs(D.th - D.slopePitch) < 30) D.q = 0;
      }
    }
    D.azup = azup;
    if (D.impactT > 0) D.impactT -= dt;
    // barometer: new sample every 23 ms, IIR x4 like the chip
    if (bmpNormal && g_us - lastBaroUs >= 23000) {
      lastBaroUs = g_us;
      if (baroDriftOn) baroDrift += 0.04 * N01(rng) + 0.0002;           // slow random drift
      const double pm = truePressure() + 1.5 * N01(rng);
      bmpIIR = (bmpIIR < 0) ? pm : bmpIIR + (pm - bmpIIR) / 4.0;
      bmpSetData(bmpIIR, tempC);
    }
  }
  if (radio != R_OFF && g_us - lastFrameUs >= 7000) { lastFrameUs = g_us; pushFrame(); }
}

// ----------------------------------------------------------------- scenario helpers
bool traceOn = false; double traceEvery = 0.5, nextTrace = 0;
std::function<void()> pilot;                                          // optional automatic pilot
double maxAltErr = 0, altErrRef = 0; bool trackAltErr = false;
double minZ = 1e9, maxZ = -1e9;

void trace() {
  printf("  t=%7.2f %-8s arm=%d air=%d z=%6.2f est=%6.2f tgt=%6.2f vz=%+5.2f vest=%+5.2f thr=%4.0f hov=%4.0f vI=%+5.0f "
         "roll=%+5.1f pitch=%+5.1f V=%5.2f bat=%d\n",
         g_us / 1e6, MODE_NAMES[flightMode], armed, airborne, D.z, altEst, altTarget, D.vz, velEst, lastThrOut,
         hoverThrLearned, velI, D.phi, D.th, D.vbat, (int)battState);
}
void step() {
  loop(); physics();
  if (pilot) pilot();
  const double t = g_us / 1e6;
  if (trackAltErr && armed) maxAltErr = fmax(maxAltErr, fabs(D.z - altErrRef));
  if (armed) { minZ = fmin(minZ, D.z); maxZ = fmax(maxZ, D.z); }
  if (traceOn && t >= nextTrace) { nextTrace = t + traceEvery; trace(); }
}
void runFor(double s) { const double end = g_us / 1e6 + s; while (g_us / 1e6 < end) step(); }
bool runUntilCond(double maxS, std::function<bool()> c) {
  const double end = g_us / 1e6 + maxS;
  while (g_us / 1e6 < end) { step(); if (c()) return true; }
  return false;
}
int fails = 0;
void check(bool ok, const char *what, double v = NAN) {
  if (std::isnan(v)) printf("%s  %s\n", ok ? "PASS" : "FAIL", what);
  else printf("%s  %s  (%.3f)\n", ok ? "PASS" : "FAIL", what, v);
  if (!ok) fails++;
}
void boot() {
  bmpReset();
  setup();
  runFor(1.0);
}
void arm() { ch[2] = 1000; ch[5] = 1000; ch[4] = 1000; runFor(0.3); ch[4] = 2000; runFor(0.3); }
// "human" pilot in MANUAL: holds a height with the throttle stick
double pilotAlt = 0;
void manualPilot() { if (flightMode != MODE_MANUAL || !armed) return;
  double s = 1580 + 160 * (pilotAlt - D.z) - 120 * D.vz; ch[2] = (uint16_t)fmin(1800, fmax(1100, s)); }
void takeoffManual(double h) {
  pilotAlt = h; pilot = manualPilot;
  runUntilCond(15, [&] { return D.z > h - 0.15 && fabs(D.vz) < 0.3; });
  runFor(4.0);                                                        // hover: learns the hover throttle
}
void altHoldOn() { pilot = nullptr; ch[2] = 1500; ch[5] = 1500; runFor(0.4); }

void typeCmd(const char *cmd) {                                  // like typing in the Serial Monitor
  for (const char *c = cmd; *c; c++) Serial.q.push_back(*c);
  Serial.q.push_back('\n');
}
void dumpLog(const char *fname) {
  Serial.out.clear(); Serial.capture = true; bool q = Serial.quiet; Serial.quiet = true;
  typeCmd("d");
  runFor(0.2);
  Serial.capture = false; Serial.quiet = q;
  const std::string &o = Serial.out;
  size_t b = o.find("BEGIN_LOG"), e = o.find("END_OF_LOG");
  if (b == std::string::npos || e == std::string::npos) { printf("dumpLog: no log\n"); return; }
  FILE *f = fopen(fname, "w"); fwrite(o.data() + b, 1, e - b + 11, f); fclose(f);
}
// ----------------------------------------------------------------- scenarios
void s1_normal_flight() {
  printf("== S1: take-off MANUAL -> ALT HOLD -> climb -> LAND on 3 deg slope -> log dump\n");
  D.slopeRoll = 3.0; D.phi = 3.0;
  boot();
  check(imuOk && baroOk && fsOk && logCap > 40000, "boot: IMU, baro, flash, PSRAM log buffer");
  check(ledNow == 0x002800, "LED green = ready");
  check(fabs(baroPressurePa - truePressure()) < 6, "BMP280 integer formula matches datasheet float formula (Pa)",
        baroPressurePa - truePressure());
  arm(); check(armed, "armed with SwA"); runFor(0.1);
  check(ledNow == 0x00003C, "LED blue = armed MANUAL");
  takeoffManual(2.0);
  check(airborne && hoverLearnedValid, "airborne, hover throttle learned", hoverThrLearned);
  altHoldOn();
  check(flightMode == MODE_ALT, "ALT HOLD engaged");
  check(ledNow == 0x230032, "LED purple = ALT HOLD");
  altErrRef = D.z; maxAltErr = 0; trackAltErr = true;
  runFor(20); trackAltErr = false;
  check(maxAltErr < 0.5, "ALT HOLD 20 s: max height error < 0.5 m", maxAltErr);
  double z0 = D.z; ch[2] = 1900; runFor(3.0); ch[2] = 1500; runFor(4.0);
  check(D.z - z0 > 1.2 && D.z - z0 < 3.5, "stick up 3 s: climbed (m)", D.z - z0);
  altErrRef = D.z; maxAltErr = 0; trackAltErr = true; runFor(10); trackAltErr = false;
  check(maxAltErr < 0.5, "holds the new height 10 s (m)", maxAltErr);
  // LAND
  ch[5] = 2000; runFor(0.1);
  check(flightMode == MODE_LAND, "LAND engaged");
  double tLand = g_us / 1e6, maxDesc = 0; D.maxTiltGround = 0;
  double tTouch = 0;
  bool done = runUntilCond(40, [&] { maxDesc = fmax(maxDesc, -D.vz); if (!tTouch && D.ground) tTouch = g_us / 1e6; return !armed; });
  double tDisarm = g_us / 1e6;
  check(done, "landed and disarmed by itself", tDisarm - tLand);
  printf("      touchdown -> motors off: %.2f s\n", tDisarm - tTouch);
  check(maxDesc < 0.9, "descent speed never above 0.9 m/s", maxDesc);
  check(D.touchdowns >= 1 && D.lastTouchdownV > -0.5, "touchdown softer than 0.5 m/s", D.lastTouchdownV);
  check(D.maxTiltGround < 10, "no tipping on the 3 deg slope (deg)", D.maxTiltGround);
  runFor(3.0);
  check(!logPendingSave && g_fs.files.size() == 1, "log saved to flash");
  printf("----- serial commands h, i, l -----\n");
  typeCmd("h"); runFor(0.05);
  typeCmd("i"); runFor(0.05);
  typeCmd("l"); runFor(0.05);
  typeCmd("s"); runFor(0.05);
  printf("-----------------------------------\n");
  // dump through the serial commands
  Serial.capture = true; Serial.quiet = true;
  typeCmd("d");
  runFor(0.2);
  Serial.capture = false; Serial.quiet = false;
  const std::string &o = Serial.out;
  size_t b = o.find("BEGIN_LOG"), e = o.find("END_OF_LOG");
  check(b != std::string::npos && e != std::string::npos, "dump has BEGIN_LOG / END_OF_LOG");
  int rows = 0; for (size_t i = b; i < e; i++) if (o[i] == '\n') rows++;
  rows -= 2;                                                          // BEGIN line + CSV header
  printf("      CSV rows: %d (flight %.1f s at %d Hz)\n", rows, flightTimeMs / 1000.0, LOG_HZ);
  check(fabs(rows - flightTimeMs / 1000.0 * LOG_HZ) < 0.01 * flightTimeMs / 1000.0 * LOG_HZ + 3, "one row per 20 ms of flight");
  FILE *f = fopen("out/s1_log.csv", "w"); fwrite(o.data() + b, 1, e - b + 11, f); fclose(f);
}

void s2_battery_landing() {
  printf("== S2: battery runs low in ALT HOLD -> LOW warning -> CRITICAL -> forced LAND, SwC ignored\n");
  D.soc = 0.40;
  boot(); arm(); takeoffManual(3.0); altHoldOn();
  runFor(15); printf("      hovering: measured %.2f V/cell, estimated resting %.2f V/cell (%d%%), true resting %.2f, sag K %.2f\n",
                      vCellRaw, vCell, battPct, ocvCell(D.soc), sagK);
  check(fabs(vCell - ocvCell(D.soc)) < 0.06, "sag compensation: resting voltage within 0.06 V/cell", vCell - ocvCell(D.soc));
  bool low = runUntilCond(400, [] { return battState == BATT_LOW; });
  check(low, "battery LOW detected", vCell);
  printf("      LOW at true charge %.0f%%\n", D.soc * 100);
  double t0 = g_us / 1e6; runFor(3.0);
  check(beepsBetween(t0, t0 + 3) >= 2 && flightMode == MODE_ALT, "LOW: beeps every second, still flying ALT HOLD");
  bool crit = runUntilCond(400, [] { return flightMode == MODE_LAND; });
  check(crit && landReason == LAND_BATTERY, "CRITICAL -> automatic LAND", vCell);
  printf("      CRITICAL at true charge %.0f%% (%.0f mAh left)\n", D.soc * 100, D.soc * D.cap_mAh);
  ch[5] = 1000; runFor(0.5);
  check(flightMode == MODE_LAND, "SwC to MANUAL is ignored during a battery landing");
  bool done = runUntilCond(60, [] { return !armed; });
  check(done && D.z < 0.05, "landed and disarmed");
  printf("      landed with %.0f%% (%.0f mAh) left\n", D.soc * 100, D.soc * D.cap_mAh);
  ch[2] = 1000; ch[4] = 1000; ch[5] = 1000; runFor(3.0);
  printf("      resting after landing: %.2f V/cell\n", vCell);
  ch[4] = 2000; runFor(0.5);
  check(!armed && D.vbat / 3 < CELL_MIN_ARM_V, "cannot re-arm with the empty battery", vCell);
  D.soc = 1.0; runFor(5.0);                                            // swap in a fresh pack
  check(battState == BATT_OK && armProblem() == nullptr, "fresh pack on the ground clears the warning");
  runFor(1.0); dumpLog("out/s2_log.csv");
}

void s3_radio_loss_alt() {
  printf("== S3: radio frames stop in ALT HOLD at 4 m -> LAND -> disarm -> lost-drone beeps\n");
  boot(); arm(); takeoffManual(4.0); altHoldOn();
  radio = R_OFF; runFor(0.5);
  check(flightMode == MODE_LAND && landReason == LAND_RADIO, "radio lost -> LAND");
  bool done = runUntilCond(40, [] { return !armed; });
  check(done && D.z < 0.05 && D.lastTouchdownV > -0.6, "landed by itself", D.lastTouchdownV);
  double t0 = g_us / 1e6; runFor(6);
  check(beepsBetween(t0 + 2, t0 + 6) >= 2, "lost-drone beeper on after landing");
  radio = R_ON; runFor(1.0); dumpLog("out/s3_log.csv");
}

void s4_failsafe_frames_manual() {
  printf("== S4: receiver failsafe values (THR low, SwC LAND) while hovering in MANUAL at 3 m\n");
  boot(); arm(); takeoffManual(3.0);
  pilot = nullptr; radio = R_FAILSAFE;
  double vmax = 0; runUntilCond(3, [&] { vmax = fmax(vmax, -D.vz); return false; });
  check(flightMode == MODE_LAND, "failsafe frames -> LAND (switch)");
  check(vmax < 0.9, "no fall when the failsafe values arrive: max sink speed (m/s)", vmax);
  bool done = runUntilCond(40, [] { return !armed; });
  check(done && D.lastTouchdownV > -0.6, "landed and disarmed", D.lastTouchdownV);
}

void s5_kill_switch() {
  printf("== S5: kill switch in the air\n");
  boot(); arm(); takeoffManual(2.0);
  pilot = nullptr; ch[4] = 1000; runFor(0.02);
  check(!armed && motorUs(0) < 1010 && motorUs(3) < 1010, "SwA SAFE: motors off within one loop", motorUs(0));
}

void s6_radio_loss_ground() {
  printf("== S6: radio lost while armed on the ground\n");
  boot(); arm(); runFor(1.0);
  radio = R_OFF; bool d = runUntilCond(1.0, [] { return !armed; });
  check(d, "disarmed (radio lost on the ground)");
}

void s7_arming_refusals() {
  printf("== S7: arming refusals\n");
  boot();
  ch[2] = 1300; ch[4] = 1000; runFor(0.3); ch[4] = 2000; runFor(0.3);
  check(!armed, "throttle up -> refused");
  ch[2] = 1000; ch[4] = 1000; ch[5] = 1500; runFor(0.3); ch[4] = 2000; runFor(0.3);
  check(!armed, "SwC on ALT HOLD -> refused");
  ch[5] = 1000; runFor(0.3);
  check(!armed, "switch already on ARM: no arm until flipped");
  ch[4] = 1000; runFor(0.3); ch[4] = 2000; runFor(0.3);
  check(armed, "flip SwA again -> armed");
  ch[4] = 1000; runFor(0.5);
  D.soc = 0.02; runFor(1.0);
  ch[4] = 2000; runFor(0.3);
  check(!armed, "battery too low -> refused", vCell);
  D.soc = 0.9; runFor(2.0); ch[4] = 1000; runFor(0.3);
  D.slopeRoll = 30; runFor(1.0); ch[4] = 2000; runFor(0.3);
  check(!armed, "tilted 30 deg -> refused");
  D.slopeRoll = 0; runFor(1.0); ch[4] = 1000; runFor(0.3); radio = R_OFF; runFor(0.5); ch[4] = 2000; runFor(0.5);
  check(!armed, "no radio -> nothing happens");
}

void s8_long_hover() {
  printf("== S8: 3 minutes of ALT HOLD (noise, baro drift, battery sag): no false landing\n");
  boot(); arm(); takeoffManual(2.5); altHoldOn();
  double z0 = D.z, worst = 0, sumsq = 0; int n = 0; double thr0 = hoverThr + velI;
  runUntilCond(180, [&] { double e = D.z - z0; worst = fmax(worst, fabs(e)); sumsq += e * e; n++; return !armed; });
  check(armed && flightMode == MODE_ALT, "still flying after 180 s");
  printf("      height error: rms %.2f m, worst %.2f m; baro drift %.1f Pa (%.2f m)\n",
         sqrt(sumsq / n), worst, baroDrift, baroDrift / 12.0);
  check(worst < 1.2, "worst height error < 1.2 m (includes baro drift)", worst);
  printf("      throttle needed: start %.0f us, now %.0f us (battery %.2f V)\n", thr0, hoverThr + velI, D.vbat);
}

void s9_baro_failure() {
  printf("== S9: BMP280 stops answering in ALT HOLD at 3 m\n");
  boot(); arm(); takeoffManual(3.0); altHoldOn(); runFor(3);
  bmpFail = true; runFor(0.8);
  check(flightMode == MODE_LAND && !baroOk, "barometer lost -> LAND without barometer");
  double maxDesc = 0;
  bool done = runUntilCond(20, [&] { maxDesc = fmax(maxDesc, -D.vz); return !armed; });
  check(done, "motors stop after LAND_NOBARO_TIME_S");
  printf("      blind descent: max %.2f m/s, touchdown %.2f m/s\n", maxDesc, D.lastTouchdownV);
  check(maxDesc < 2.5, "blind descent slower than 2.5 m/s (firm landing)", maxDesc);
}

void s10_crash() {
  printf("== S10: crash (drone flips in the air)\n");
  boot(); arm(); takeoffManual(2.0);
  motorDead = 1;                                                       // M2 (front-right) stops
  double t0 = g_us / 1e6; bool d = runUntilCond(2.0, [] { return !armed; });
  printf("      motor M2 failed: tilt %.0f deg, motors cut after %.2f s\n", fmax(fabs(D.phi), fabs(D.th)), g_us / 1e6 - t0);
  check(d && fmax(fabs(D.phi), fabs(D.th)) < 100, "tilt > 70 deg -> motors stop");
  runFor(5);
  check(beepsBetween(g_us / 1e6 - 4, g_us / 1e6) >= 1, "lost-drone beeper");
}

void s11_auto_disarm() {
  printf("== S11: armed, throttle down on the ground for 15 s\n");
  boot(); arm();
  bool d = runUntilCond(20, [] { return !armed; });
  check(d && g_us / 1e6 > 25, "auto-disarm after 15 s", g_us / 1e6);
}

void s12_alt_stick_down_landing() {
  printf("== S12: ALT HOLD, throttle stick fully down -> descends and disarms on the ground\n");
  boot(); arm(); takeoffManual(3.0); altHoldOn();
  ch[2] = 1000;
  double maxDesc = 0;
  bool done = runUntilCond(30, [&] { maxDesc = fmax(maxDesc, -D.vz); return !armed; });
  check(done && D.z < 0.05, "landed and disarmed in ALT HOLD");
  check(maxDesc < 0.8, "descent <= ALT_MAX_DESCENT", maxDesc);
}

void s13_land_from_high() {
  printf("== S13: LAND from 15 m\n");
  boot(); arm(); takeoffManual(15.0);
  ch[5] = 2000; runFor(0.2);
  double tStart = g_us / 1e6, vFast = 0, vSlow = 0;
  bool done = runUntilCond(60, [&] {
    if (D.z > 4 && D.z < 10) vFast = -D.vz;
    if (D.z > 0.3 && D.z < 1.0) vSlow = -D.vz;
    return !armed; });
  check(done, "landed and disarmed", g_us / 1e6 - tStart);
  printf("      descent speed: %.2f m/s high up, %.2f m/s near the ground\n", vFast, vSlow);
  check(fabs(vFast - 0.6) < 0.2 && fabs(vSlow - 0.3) < 0.15, "fast then slow descent");
}

void s14_switch_flip() {
  printf("== S14: SwC flipped from MANUAL to LAND quickly (30 ms in the middle)\n");
  boot(); arm(); takeoffManual(2.0);
  int b0 = (int)beeps.size();
  pilot = nullptr; ch[2] = 1500;
  ch[5] = 1500; runFor(0.03); ch[5] = 2000; runFor(0.5);
  check(flightMode == MODE_LAND, "LAND engaged");
  check((int)beeps.size() - b0 <= 2, "no refusal beeps (ALT HOLD never tried)");
}

void s15_no_psram_no_fs() {
  printf("== S15: PSRAM off and no FAT partition -> RAM log, dump from RAM\n");
  g_psram = false; FFat.ok = false;
  boot();
  check(!fsOk && logCap > 0 && logCap < 2000, "small RAM log buffer", logCap);
  arm(); takeoffManual(1.5); ch[5] = 2000; runUntilCond(30, [] { return !armed; });
  Serial.capture = true; Serial.quiet = true;
  typeCmd("d");
  runFor(0.2); Serial.capture = false; Serial.quiet = false;
  check(Serial.out.find("BEGIN_LOG ram") != std::string::npos && Serial.out.find("END_OF_LOG") != std::string::npos,
        "dump of the RAM log works");
}

void s16_log_rotation() {
  printf("== S16: 23 short flights -> only LOG_MAX_FILES logs kept, oldest removed\n");
  boot();
  for (int i = 0; i < 23; i++) { arm(); runFor(1.0); ch[4] = 1000; runFor(1.5); }
  int newest, oldest, count; logScan(newest, oldest, count);
  check(count == LOG_MAX_FILES && newest == 23 && oldest == 4, "20 logs kept: 4..23", count);
  Serial.quiet = true; typeCmd("X"); runFor(0.2); Serial.quiet = false;
  logScan(newest, oldest, count);
  check(count == 0, "X erases all logs");
}

void s17_ground_modes() {
  printf("== S17: ALT HOLD refused on the ground; LAND on the ground = disarm\n");
  boot(); arm(); ch[2] = 1200; runFor(1.0);
  ch[5] = 1500; runFor(0.4);
  check(armed && flightMode == MODE_MANUAL, "ALT HOLD refused on the ground");
  ch[5] = 2000; runFor(0.1);
  check(!armed, "LAND on the ground -> motors stop");
}

int main(int argc, char **argv) {
  int s = argc > 1 ? atoi(argv[1]) : 1;
  traceOn = argc > 2;
  Serial.quiet = (argc > 3);
  switch (s) {
    case 1: s1_normal_flight(); break;
    case 2: s2_battery_landing(); break;
    case 3: s3_radio_loss_alt(); break;
    case 4: s4_failsafe_frames_manual(); break;
    case 5: s5_kill_switch(); break;
    case 6: s6_radio_loss_ground(); break;
    case 7: s7_arming_refusals(); break;
    case 8: s8_long_hover(); break;
    case 9: s9_baro_failure(); break;
    case 10: s10_crash(); break;
    case 11: s11_auto_disarm(); break;
    case 12: s12_alt_stick_down_landing(); break;
    case 13: s13_land_from_high(); break;
    case 14: s14_switch_flip(); break;
    case 15: s15_no_psram_no_fs(); break;
    case 16: s16_log_rotation(); break;
    case 17: s17_ground_modes(); break;
  }
  printf("%s (%d failed checks)\n", fails ? "SCENARIO FAILED" : "scenario ok", fails);
  return fails ? 1 : 0;
}
