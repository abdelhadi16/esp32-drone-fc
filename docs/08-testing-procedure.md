# 8. Testing procedure for version 3

Do the build guide (PDF) steps 1–12 first: motors, ESCs, receiver, MPU6050 and a good
MANUAL hover. Then add the version 3 features in this order. Tick each box; do not skip a
failed test.

**Rules for every test:** props **off** unless the step says "props on". Props-on tests
outdoors on grass, 5 m from people, kill switch (SwA) under your finger, a charged pack.

## A. Upload and settings

- [ ] Tools menu as in chapter 2.1 (PSRAM **OPI**, Partition **16M Flash (3MB APP/9.9MB FATFS)**).
- [ ] Copy your values into `config.h`: `IMU_ROTATION`, `IMU_UPSIDE_DOWN`, `ACC_*_TRIM`
      (sketch 3), `ESC_FREQ_HZ`, `MOTOR_OFF` (sketch 1), your PID values (guide step 12).
- [ ] Upload. The first start can take ~30 s (the log partition is formatted once).
- [ ] Serial Monitor: the info block shows MPU6050 OK, BMP280 OK at 0x76/0x77,
      "Log memory: PSRAM", "Log flash: 0 saved".

## B. Radio (props off)

- [ ] Transmitter: **Aux. channels → Channel 6 = SwC**.
- [ ] Sketch 2: CH5 shows `ARM` / `SAFE`, CH6 shows `MANUAL` / `ALT HOLD` / `LAND`.
- [ ] Failsafe values as in chapter 6.5; transmitter-off test in sketch 2 passes.

## C. Flight controller on the bench (props off, LiPo in, USB in, H1 unplugged)

- [ ] Status line says `READY` with SwA SAFE, SwC MANUAL, throttle down.
- [ ] **Battery:** `i` shows the voltage; multimeter → set `VBAT_CAL` (chapter 5.1).
- [ ] **Buzzer / LED:** long beep at start; LED green. Arm → 1 beep, LED blue.
      Disarm → 2 beeps; ~1 s later LED white then long beep (log saved).
- [ ] **Refusals (3 fast beeps + reason):** arm with throttle up; arm with SwC on ALT HOLD;
      arm with the drone tilted 45°.
- [ ] **Kill switch:** arm, throttle to ~1300 µs (motors spin), SwA SAFE → motors stop at once.
- [ ] **Modes on the ground:** arm, throttle a little up, SwC → ALT HOLD: refused (3 beeps).
      SwC → LAND: motors stop.
- [ ] **Radio loss:** arm, motors slow, transmitter off → motors stop within ~1 s.
- [ ] **Auto-disarm:** arm, leave the throttle down → motors stop after 15 s.
- [ ] **Log:** `l` lists the logs; `python tools/flight_log.py download` + `plot` work.

## D. First flights with version 3 (props on)

- [ ] MANUAL hover at 1 m for 30 s. Land, disarm. `i` → "Hover throttle … (learned in
      flight)". Download and plot the log: angles follow the wanted angles, motors close
      together, battery "estimated resting" smooth.
- [ ] ALT HOLD at 1.5–2 m for 10 s (chapter 6.1), back to MANUAL, land. Log: height
      within ±0.3 m of the target. Tune with chapter 4.4 if not.
- [ ] ALT HOLD climbs and descents with the throttle stick.
- [ ] LAND from 2 m: lands, motors stop by themselves within ~2 s.
- [ ] LAND from 5 m.
- [ ] ALT HOLD, throttle stick fully down: lands and stops.

## E. Failsafes in the air (props on, over grass, at 2–3 m)

- [ ] Radio: ALT HOLD at 3 m, switch the transmitter off. The drone lands and stops.
      After landing, the "find me" chirp every 2 s.
- [ ] Battery: fly a pack until LOW (beep every second); land normally. Note the charge in
      the log. Later, with a nearly empty pack at low height, let CRITICAL trigger the
      automatic landing once, so you know what it looks and sounds like.

## Before every flight

- [ ] Props tight, correct direction; battery strapped; no loose wires near props.
- [ ] Transmitter on first, SwA SAFE, SwC MANUAL, throttle down.
- [ ] Plug the LiPo, keep the drone still until the long beep (gyro calibration).
- [ ] LED green, no repeating beeps.

## After every flight

- [ ] SwA SAFE (2 beeps), wait for the long beep (log saved), then unplug the LiPo.
- [ ] Touch motors and ESCs: warm is fine, too hot to touch is not.
- [ ] Download the log, write down what you changed and what you saw.

## Software tests on a PC (no drone needed)

The real flight-controller code also runs in a simulator on your PC (`tools/sim`): 17
scenarios fly a virtual quadcopter and check take-off, ALT HOLD, LAND, battery landing,
radio loss, receiver failsafe, kill switch, crash, barometer failure, logging and more.
Run it after every code change:

```bash
bash tools/sim/run_tests.sh      # Linux, macOS, or Windows with WSL / Git Bash + g++
```

It must end with `ALL SCENARIOS PASSED`. GitHub runs it automatically on every push
(chapter 10).
