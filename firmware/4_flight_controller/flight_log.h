#pragma once
// Flight log.
//  - During flight: one record (44 bytes) every 20 ms goes into PSRAM (RAM only:
//    writing to flash in flight would pause the CPU and disturb the control loop).
//  - After disarming: the records are saved to flash as /log_NNN.bin (FFat partition,
//    "16M Flash (3MB APP/9.9MB FATFS)"). Wait for the 3 beeps before unplugging.
//  - On the bench (USB, H1 unplugged): type 'l' to list, 'd' to dump the newest log
//    as CSV, 'd7' for log 7, 'X' to erase all. tools/flight_log.py does it for you.
#include "FFat.h"

struct __attribute__((packed)) LogRec {
  uint32_t t_ms;
  int16_t roll_cd, pitch_cd;        // 0.01 deg
  int16_t rollSp_cd, pitchSp_cd;    // 0.01 deg (wanted angles)
  int16_t yawRate_dd;               // 0.1 deg/s
  int16_t alt_cm, altTgt_cm, baroAlt_cm;
  int16_t vz_cms;                   // cm/s
  uint16_t thrIn, thrOut;           // us
  uint16_t m[4];                    // us
  uint16_t vbat_mv;                 // measured (sags under load)
  uint16_t vrest_mv;                // estimated resting voltage (sag compensated)
  int16_t amps_ca;                  // 0.01 A (current sensor, else 0)
  uint8_t battPct;                  // % from vrest
  uint8_t mode;                     // 0 MANUAL, 1 ALT HOLD, 2 LAND
  uint8_t flags;                    // see LOGF_*
  uint8_t spare;
};
static_assert(sizeof(LogRec) == 44, "LogRec must stay 44 bytes");
constexpr uint16_t LOG_VERSION = 2;

enum : uint8_t { LOGF_ARMED = 1, LOGF_AIRBORNE = 2, LOGF_RADIO_OK = 4, LOGF_BATT_LOW = 8,
                 LOGF_BATT_CRIT = 16, LOGF_BARO_OK = 32, LOGF_LAND_RADIO = 64, LOGF_LAND_BATT = 128 };

struct __attribute__((packed)) LogHeader {
  char magic[4];        // "DLOG"
  uint16_t version;     // LOG_VERSION
  uint16_t recSize;     // sizeof(LogRec)
  uint32_t count;
  uint16_t logHz;
  uint16_t reserved;
};

LogRec *logBuf = nullptr;
uint32_t logCap = 0, logCount = 0;
bool logActive = false;
bool fsOk = false;
bool logPendingSave = false;
uint32_t logStartMs = 0;

void logBegin() {
  if (!LOG_ENABLED) return;
  if (psramFound()) {
    logCap = (2UL * 1024 * 1024) / sizeof(LogRec);            // 2 MB = ~16 min at 50 Hz
    logBuf = (LogRec *)ps_malloc(logCap * sizeof(LogRec));
  }
  if (!logBuf) {
    logCap = (48UL * 1024) / sizeof(LogRec);                  // no PSRAM: ~22 s
    logBuf = (LogRec *)malloc(logCap * sizeof(LogRec));
  }
  if (!logBuf) logCap = 0;
  fsOk = FFat.begin(true);
}

void logStart() { logCount = 0; logStartMs = millis(); logActive = (logBuf != nullptr); logPendingSave = false; }
void logStop()  { if (logActive && logCount > 0) logPendingSave = fsOk; logActive = false; }
void logAdd(const LogRec &r) { if (logActive && logCount < logCap) logBuf[logCount++] = r; }

static int logIndexFromName(const char *name) {
  const char *p = strstr(name, "log_");
  return p ? atoi(p + 4) : -1;
}

// newest / oldest index and file count
static void logScan(int &newest, int &oldest, int &count) {
  newest = -1; oldest = 100000; count = 0;
  File root = FFat.open("/");
  if (!root) return;
  File f = root.openNextFile();
  while (f) {
    const int idx = logIndexFromName(f.name());
    if (idx >= 0) { count++; if (idx > newest) newest = idx; if (idx < oldest) oldest = idx; }
    f = root.openNextFile();
  }
}

static void logPath(int idx, char *out) { snprintf(out, 20, "/log_%03d.bin", idx); }

// Saves the RAM log to flash. Blocking (a few seconds) -> only call while DISARMED.
bool logSave() {
  logPendingSave = false;
  if (!fsOk || logCount == 0) return false;
  int newest, oldest, count;
  logScan(newest, oldest, count);
  const size_t need = sizeof(LogHeader) + logCount * sizeof(LogRec) + 8192;
  for (int guard = 0; guard < 100 && count > 0 &&
                      (count >= LOG_MAX_FILES || FFat.freeBytes() < need); guard++) {   // make room
    char p[20]; logPath(oldest, p); FFat.remove(p);
    logScan(newest, oldest, count);
  }
  const int idx = (newest < 0 ? 1 : newest + 1);
  char path[20]; logPath(idx, path);
  File f = FFat.open(path, FILE_WRITE);
  if (!f) return false;
  LogHeader h = {{'D', 'L', 'O', 'G'}, LOG_VERSION, (uint16_t)sizeof(LogRec), logCount, LOG_HZ, 0};
  f.write((const uint8_t *)&h, sizeof(h));
  const uint8_t *src = (const uint8_t *)logBuf;
  size_t left = logCount * sizeof(LogRec);
  while (left) {
    const size_t n = left > 8192 ? 8192 : left;
    if (f.write(src, n) != n) { f.close(); return false; }
    src += n; left -= n;
  }
  f.close();
  if (DEBUG) Serial.printf("Log saved: %s (%lu records, %.1f s)\n", path,
                           (unsigned long)logCount, logCount / (float)LOG_HZ);
  return true;
}

void logList() {
  if (!fsOk) { Serial.println(F("No flash file system (check Partition Scheme: ...FATFS).")); return; }
  Serial.println(F("Saved flight logs:"));
  File root = FFat.open("/");
  File f = root ? root.openNextFile() : File();
  int n = 0;
  while (f) {
    if (logIndexFromName(f.name()) >= 0) {
      Serial.printf("  %-14s %7lu bytes  %6.1f s\n", f.name(), (unsigned long)f.size(),
                    (f.size() - sizeof(LogHeader)) / (float)sizeof(LogRec) / LOG_HZ);
      n++;
    }
    f = root.openNextFile();
  }
  if (!n) Serial.println(F("  (none)"));
  Serial.printf("Flash: %lu KB free of %lu KB\n", (unsigned long)(FFat.freeBytes() / 1024),
                (unsigned long)(FFat.totalBytes() / 1024));
}

static void logPrintCsvHeader() {
  Serial.println(F("t_s,mode,armed,airborne,radio_ok,batt_low,batt_crit,baro_ok,land_radio,land_batt,"
                   "roll,pitch,roll_sp,pitch_sp,yaw_rate,alt,alt_target,baro_alt,vz,"
                   "thr_in,thr_out,m1,m2,m3,m4,vbat,vrest,batt_pct,current"));
}

static void logPrintCsvRow(const LogRec &r, uint32_t t0) {
  Serial.printf("%.3f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.2f,%.2f,"
                "%u,%u,%u,%u,%u,%u,%.2f,%.2f,%u,%.2f\n",
                (r.t_ms - t0) / 1000.0f, r.mode,
                (r.flags & LOGF_ARMED) ? 1 : 0, (r.flags & LOGF_AIRBORNE) ? 1 : 0,
                (r.flags & LOGF_RADIO_OK) ? 1 : 0, (r.flags & LOGF_BATT_LOW) ? 1 : 0,
                (r.flags & LOGF_BATT_CRIT) ? 1 : 0, (r.flags & LOGF_BARO_OK) ? 1 : 0,
                (r.flags & LOGF_LAND_RADIO) ? 1 : 0, (r.flags & LOGF_LAND_BATT) ? 1 : 0,
                r.roll_cd / 100.0f, r.pitch_cd / 100.0f, r.rollSp_cd / 100.0f, r.pitchSp_cd / 100.0f,
                r.yawRate_dd / 10.0f, r.alt_cm / 100.0f, r.altTgt_cm / 100.0f, r.baroAlt_cm / 100.0f,
                r.vz_cms / 100.0f, r.thrIn, r.thrOut, r.m[0], r.m[1], r.m[2], r.m[3],
                r.vbat_mv / 1000.0f, r.vrest_mv / 1000.0f, r.battPct, r.amps_ca / 100.0f);
}

// Prints a log as CSV between BEGIN_LOG and END_OF_LOG. idx < 0 = newest.
void logDump(int idx) {
  if (!fsOk) {                                 // no flash: dump what is still in RAM
    Serial.printf("BEGIN_LOG ram %lu\n", (unsigned long)logCount);
    logPrintCsvHeader();
    for (uint32_t i = 0; i < logCount; i++) logPrintCsvRow(logBuf[i], logBuf[0].t_ms);
    Serial.println(F("END_OF_LOG"));
    return;
  }
  if (idx < 0) { int oldest, count; logScan(idx, oldest, count); }
  if (idx < 0) { Serial.println(F("No saved log.")); return; }
  char path[20]; logPath(idx, path);
  File f = FFat.open(path, FILE_READ);
  LogHeader h;
  if (!f || f.read((uint8_t *)&h, sizeof(h)) != sizeof(h) || memcmp(h.magic, "DLOG", 4) != 0) {
    Serial.printf("Cannot read %s\n", path);
    return;
  }
  if (h.version != LOG_VERSION || h.recSize != sizeof(LogRec)) {
    Serial.printf("%s was written by another firmware version (v%u). Erase old logs with X.\n", path, h.version);
    return;
  }
  Serial.printf("BEGIN_LOG %s %lu\n", path, (unsigned long)h.count);
  logPrintCsvHeader();
  LogRec r;
  uint32_t t0 = 0;
  for (uint32_t i = 0; i < h.count; i++) {
    if (f.read((uint8_t *)&r, sizeof(r)) != sizeof(r)) break;
    if (i == 0) t0 = r.t_ms;
    logPrintCsvRow(r, t0);
  }
  f.close();
  Serial.println(F("END_OF_LOG"));
}

void logEraseAll() {
  if (!fsOk) return;
  int newest, oldest, count;
  logScan(newest, oldest, count);
  for (int guard = 0; guard < 1000 && count > 0; guard++) {
    char p[20]; logPath(oldest, p); FFat.remove(p);
    logScan(newest, oldest, count);
  }
  Serial.println(count ? F("Some logs could not be erased.") : F("All logs erased."));
}
