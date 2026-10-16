## What changed and why

<!-- e.g. "ALT HOLD: VEL_I 40 -> 30 because the drone breathed up and down (log 7)" -->

## How it was tested

- [ ] `bash tools/sim/run_tests.sh` -> ALL SCENARIOS PASSED
- [ ] compiled and uploaded with the Arduino IDE
- [ ] bench test with props off (docs/08, part C)
- [ ] flight test (attach the log plot from `tools/flight_log.py plot`)

## Safety check

- [ ] kill switch (SwA) still stops the motors at once
- [ ] arming refusals still work (throttle up, SwC not on MANUAL)
