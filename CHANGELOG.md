# Changelog

## v3.0 – October 2026

Flight controller split into modules (`config.h` + one `.h` per job).

- **Altitude hold** (SwC middle): BMP280 driver with Bosch's integer compensation,
  3rd-order complementary filter (barometer + accelerometer, learned accelerometer bias),
  height → vertical speed → throttle cascade, learned hover throttle, tilt compensation,
  anti-windup, throttle-stick climb / descent.
- **Automatic landing** (SwC down, radio lost, battery critical, barometer lost): 0.6 then
  0.3 m/s, INAV-style landing detector, fast spool-down and I-term decay at touchdown,
  three safety nets, landing without barometer.
- **Battery management**: sag compensation without a current sensor, percentage,
  LOW (warning) and CRITICAL (auto-landing) with 5 s confirmation, absolute floor,
  arming check, optional current sensor.
- **Status**: non-blocking buzzer patterns on H3 (GPIO 17) and DevKit RGB LED.
- **Flight recorder**: 44-byte records at 50 Hz in PSRAM, saved to FFat after landing,
  20 files kept, CSV dump over USB; `tools/flight_log.py` (download, summary, plots).
- **Safety**: arming needs SwC on MANUAL and a charged battery; auto-disarm after 15 s
  at idle on the ground; I-terms held at zero until airborne; faster attitude settling
  while disarmed; no serial printing while armed (flight summary after landing).
- **Serial commands**: `h i s l d X z`.
- Sketch 2 shows ARM / SAFE and MANUAL / ALT HOLD / LAND; sketch 3 prints the level trims
  as `#define` lines ready for `config.h`.
- **Simulator** (`tools/sim`): 17 scenarios, both Arduino-ESP32 cores.

## v2 – October 2026

Single-file angle-mode flight controller; motor test with timed ESC calibration and
signal-speed test; receiver test; IMU test with I²C doctor; PDF build guide.
