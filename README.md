# ESP32-S3 Quadcopter Flight Controller

A home-made flight controller for an F330 quadcopter: our own PCB with an
**ESP32-S3-DevKitC-1 (N16R8)**, an **MPU6050** and a **BMP280**, a FlySky **i-BUS**
receiver, and flight firmware written from scratch in Arduino C++.
Built for the **AST Aerial Technology Cup 1.0**, Topic 4: *Embedded Systems*.

> ⚠️ Experimental. Props off for every bench test. Fly low, over grass, away from people,
> with the kill switch ready. Check the local drone rules before flying outdoors
> ([docs/09](docs/09-competition.md#96-practical-and-legal-notes)).

## Features (firmware v3)

- **Self-levelling (angle) mode**, 250 Hz loop, cascaded angle → rate PID, Quad-X mixer
- **Altitude hold**: barometer + accelerometer fused by a 3rd-order complementary filter,
  height and vertical-speed cascade, learned hover throttle, tilt compensation
- **Automatic landing** with ground detection, controlled descent (0.6 → 0.3 m/s)
- **Failsafes**: radio lost → land, battery critical → land, barometer lost → land,
  crash cut-off, kill switch, arming checks, auto-disarm
- **Battery management without a current sensor**: sag-compensated resting voltage,
  percentage, LOW warning, CRITICAL auto-landing
- **Buzzer and RGB LED** status, non-blocking
- **Flight recorder**: 50 Hz into PSRAM during flight, saved to flash after landing,
  downloaded and plotted with a Python tool
- **Software-in-the-loop simulator**: the real firmware flies a virtual quadcopter in
  17 automatic test scenarios (`tools/sim`)

| Simulated flight: MANUAL → ALT HOLD → climb → LAND | Simulated battery landing |
|---|---|
| ![](docs/images/sim_flight_althold_land.png) | ![](docs/images/sim_battery_landing.png) |

## Repository layout

```
esp32-drone-fc/
├── firmware/
│   ├── 1_motor_test/          ESC arming + calibration, motor order and direction
│   ├── 2_receiver_test/       radio channels, switches, failsafe check
│   ├── 3_imu_test/            MPU6050 orientation and level calibration
│   ├── 4_flight_controller/   THE FLIGHT CODE (edit config.h only)
│   │   ├── 4_flight_controller.ino   modes, failsafes, landing, logging, serial
│   │   ├── config.h                  all user settings
│   │   ├── rc_ibus.h  motors.h  pid.h  imu_mpu6050.h  baro_bmp280.h
│   │   └── altitude.h battery.h status_beeper.h flight_log.h util.h
│   └── extras/bench_test/     all-in-one bench check
├── tools/
│   ├── flight_log.py          download / summarise / plot flight logs
│   ├── requirements.txt
│   └── sim/                   software-in-the-loop simulator + run_tests.sh
├── docs/                      documentation (start with docs/README.md) + PDF build guide
├── hardware/                  schematic, PCB view, pin map, known PCB issues
└── .github/                   automatic tests on every push, pull request template
```

## Quick start

1. Arduino IDE with the **esp32 by Espressif** boards. Tools: *ESP32S3 Dev Module*,
   *Flash Size 16MB*, **PSRAM: OPI PSRAM**, **Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS)**,
   *USB CDC On Boot: Disabled* (COM port).
2. Follow the build guide [`docs/Drone_Build_Guide.pdf`](docs/Drone_Build_Guide.pdf) with
   sketches 1 → 2 → 3 (motors, radio, IMU).
3. Put your values in [`config.h`](firmware/4_flight_controller/config.h), upload
   `4_flight_controller`, and follow the test plan in [docs/08](docs/08-testing-procedure.md).

| Control | Function |
|---------|----------|
| CH1–CH4 | roll, pitch, throttle, yaw (Mode 2) |
| **SwA → CH5** | ARM / SAFE (kill switch) |
| **SwC → CH6** | MANUAL / ALT HOLD / LAND |

To arm: SwC on MANUAL, throttle fully down, SwA to ARM. Details, beeps and LED colours:
[docs/02](docs/02-firmware-overview.md).

## Documentation

| | |
|-|-|
| [1. Hardware](docs/01-hardware.md) | [6. Altitude hold and landing](docs/06-altitude-hold-and-landing.md) |
| [2. Firmware overview](docs/02-firmware-overview.md) | [7. Flight logging](docs/07-flight-logging.md) |
| [3. Sensor fusion](docs/03-sensor-fusion.md) | [8. Testing procedure](docs/08-testing-procedure.md) |
| [4. Control](docs/04-control.md) | [9. Competition](docs/09-competition.md) |
| [5. Battery](docs/05-battery.md) | [10. GitHub workflow](docs/10-github-workflow.md) |
| [Resources: books, papers, courses](docs/resources.md) | [Changelog](CHANGELOG.md) |

## Tests

```bash
bash tools/sim/run_tests.sh          # needs only g++; ends with "ALL SCENARIOS PASSED"
```

GitHub Actions runs the simulator, the log tool and an Arduino build of every sketch on
each push and pull request.

## Credits

Ideas and methods from Joop Brokking's YMFC-AL (loop structure, per-loop PID), ArduPilot
(inertial height filter, failsafe behaviour), INAV (landing detector) and Bosch / InvenSense
datasheets. See [docs/resources.md](docs/resources.md).

## License

[MIT](LICENSE)
