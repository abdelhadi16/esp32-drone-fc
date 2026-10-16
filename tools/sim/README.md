# Software-in-the-loop simulator

`sim.cpp` compiles the **real** flight controller (`firmware/4_flight_controller`) on a PC
and connects it to simulated hardware:

| Simulated | How |
|-----------|-----|
| quadcopter | 4 motors (40 ms lag, thrust ∝ throttle², ground effect), 0.85 kg, F330 arm length, inertia, drag, ground contact and tipping, CG slightly off-centre |
| MPU6050 | register model (WHO_AM_I, config registers, 14-byte burst), noise, gyro offsets |
| BMP280 | register model; pressure → raw ADC by inverting Bosch's floating-point formulas, internal IIR ×4, 23 ms sample rate, noise, slow drift, prop-wash offset |
| radio | i-BUS frames every 7 ms; transmitter off; receiver failsafe values |
| battery | 3S LiPo resting-voltage curve, 45 mΩ internal resistance, current from motor load |
| Arduino-ESP32 | LEDC, ADC, Serial, PSRAM, FFat (in memory), RGB LED and buzzer capture |

`Arduino.h`, `Wire.h`, `FFat.h`, `esp_log.h` and `driver/gpio.h` in this folder replace the
real ESP32 headers.

## Run

```bash
bash run_tests.sh                     # all scenarios, cores 3.x and 2.x
g++ -O2 -std=gnu++17 -I. sim.cpp -o sim
./sim 1 trace                         # scenario 1 with a state trace every 0.5 s
```

Output goes to `out/` (one text file per scenario, simulated flight logs as CSV). Plot a
simulated log like a real one: `python ../flight_log.py plot out/s1_log.csv`.

## Scenarios

| # | Checks |
|---|--------|
| 1 | take-off in MANUAL, hover throttle learned, ALT HOLD ±0.5 m, stick climb, LAND on a 3° slope, soft touchdown, no tipping, log saved and dumped, LED colours, serial commands |
| 2 | battery runs down: sag compensation accuracy, LOW, CRITICAL → LAND, SwC locked, cannot re-arm |
| 3 | radio frames stop in ALT HOLD → LAND → disarm → lost-drone beeper |
| 4 | receiver failsafe values in MANUAL → LAND without a fall |
| 5 | kill switch in the air |
| 6 | radio lost on the ground → disarm |
| 7 | arming refusals (throttle, SwC, switch already on ARM, battery, tilt, no radio) |
| 8 | 3 minutes of ALT HOLD with noise, drift and battery sag: no false landing |
| 9 | barometer fails in ALT HOLD → landing without barometer |
| 10 | motor failure → crash detection → motors off |
| 11 | auto-disarm after 15 s at idle |
| 12 | ALT HOLD with the stick fully down lands and disarms |
| 13 | LAND from 15 m: 0.6 then 0.3 m/s |
| 14 | quick SwC flip MANUAL → LAND does not trigger ALT HOLD |
| 15 | no PSRAM, no FAT partition: RAM log still dumps |
| 16 | 23 flights: 20 logs kept, oldest deleted; `X` erases all |
| 17 | ALT HOLD refused on the ground; LAND on the ground stops the motors |

A model is never the real drone: the simulator checks the **logic** (modes, failsafes,
landing detection, logging) and gives realistic starting gains. Real flights decide the
final tuning.
