# 9. AST Aerial Technology Cup (AST-ATC 1.0)

Organised by the AST Club of the National Higher School of Autonomous Systems
Technologies (Sidi Abdellah). Teams of 1–5 choose one (or more) of six technical topics.

## 9.1 Our topic: 4 – Embedded Systems

The specifications book lists three examples for Topic 4. This project covers all three:

| Topic 4 example (from the rules) | What we built |
|----------------------------------|---------------|
| "Development of an onboard control system for flight and sensors" | own PCB + own firmware on an ESP32-S3: 250 Hz loop, LEDC ESC output, i-BUS decoder, MPU6050 and BMP280 drivers written from the datasheets |
| "Sensor data fusion for stable autonomous flight" | complementary filter for the angles, 3rd-order complementary filter (barometer + accelerometer) for height and vertical speed, altitude hold |
| "Battery monitoring and power management system" | voltage measurement, **sag compensation without a current sensor**, percentage, LOW warning, CRITICAL automatic landing, arming check |

Plus things judges like to see: **autonomous landing** with ground detection, **failsafes**
(radio, battery, barometer, crash), a **flight recorder** with analysis tools, and a
**software-in-the-loop simulator** with 17 automatic test scenarios.

How it matches the rules:

| Rule | Status |
|------|--------|
| UAV entirely designed and built by the team, no ready-made drone | own frame assembly, own PCB, own firmware |
| open-source flight controllers allowed | we wrote our own (inspired by YMFC-AL, ArduPilot, INAV ideas, all credited) |
| fully or partially autonomous | partially: altitude hold, automatic landing, automatic failsafe landings |
| all sensing, processing and decisions onboard | everything runs on the ESP32-S3; the laptop is only used after the flight to read logs |
| safety | kill switch, arming checks, failsafes, crash cut-off, buzzer and LED status |

## 9.2 Demo plan (about 5 minutes)

1. **Poster / slide:** block diagram (docs README), PCB photo, loop timing.
2. **Power on:** long beep, green LED. Show a refused arm (throttle up → 3 beeps, reason).
3. **Take off in MANUAL**, hover at 1.5 m.
4. **ALT HOLD:** switch, take your hand off the throttle stick; the drone holds its height.
   Climb 1 m with the stick, hold again.
5. **LAND:** switch; the drone comes down, detects the ground and stops by itself.
6. (If the judges allow it) **failsafe:** ALT HOLD, transmitter off → automatic landing.
7. **Laptop:** `flight_log.py summary` + plot of the flight you just did: height vs target,
   battery measured vs estimated resting voltage.

Practise the whole demo at least five times with the same battery routine.

## 9.3 Numbers to put in the report

Measure them from your own logs (the simulator values are only a reference):

| Metric | How | Simulator |
|--------|-----|-----------|
| height hold error (RMS and worst) in ALT HOLD | `alt` − `alt_target` over 30 s | 0.16 m RMS, 0.31 m worst |
| descent speeds in LAND | `vz` above / below 1.5 m | 0.6 / 0.3 m/s |
| touchdown speed and time to motor stop | `vz` at touchdown, time to `armed` = 0 | 0.1 m/s, 1.1 s |
| battery estimate quality | `vrest` at the end vs voltage measured 1 min after landing | within 0.03 V/cell |
| flight time per pack, charge used | summary | – |
| loop rate | 250 Hz, fixed | – |

## 9.4 Report outline

1. Problem and goals (Topic 4).
2. Hardware: PCB, sensors, power, wiring (chapter 1).
3. Firmware architecture: modules, loop timing, state machine (chapter 2).
4. Sensor fusion: equations and plots (chapter 3).
5. Control: cascades, mixer, tuning method (chapter 4).
6. Battery management: sag compensation idea and results (chapter 5).
7. Autonomy and safety: ALT HOLD, LAND, landing detector, failsafes (chapter 6).
8. Verification: simulator scenarios, bench tests, flight logs (chapters 7, 8).
9. Results, limits, future work.
10. References ([resources](resources.md)).

## 9.5 If you have time left (ideas, in order of value)

- **Autonomous take-off** (Topic 1 example "autonomous takeoff, mission execution and
  landing"): a switch that climbs to 1.5 m, holds 10 s and lands. Built from the existing
  ALT HOLD and LAND code.
- **Laser rangefinder** (VL53L1X, I²C) for a precise height below ~3 m and a softer landing.
- **Current sensor** (chapter 5.5) for real mAh counting.
- **Telemetry** over ESP-NOW to a second ESP32 on the ground (display only; decisions stay
  onboard).

## 9.6 Practical and legal notes

- The specifications book says the event takes place "after mid-April 2026". Confirm the
  current date, the arena (indoor / outdoor) and the safety inspection with the organisers.
- **Drone rules in Algeria.** Presidential Decree 21-285 (13 July 2021) and an
  interministerial decree of 8 September 2025 regulate owning, building and flying drones.
  In 2026 the Ministry of National Defence asked all drone owners to register their drones
  with the National Centre for Unmanned Aircraft Systems (CNSAPB, deadline 30 April 2026),
  and reports say home-built or modified drones need an official technical report before
  approval. Ask the organisers how competitors' drones are registered or authorised, and
  until then only fly where you are allowed (for example during the event, under the
  organisers' supervision). This is not legal advice; check the official texts. Sources in
  [resources](resources.md#regulations-algeria).
