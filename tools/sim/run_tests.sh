#!/usr/bin/env bash
# Builds the simulator for both Arduino-ESP32 cores (2.x and 3.x code paths) and runs
# every scenario. Exit code 0 = all checks passed. Needs only g++ (Linux, macOS, WSL).
set -u
cd "$(dirname "$0")"
mkdir -p out
FAILED=0
for CORE in 3 2; do
  echo "== building for Arduino-ESP32 core ${CORE}.x"
  if ! g++ -O2 -std=gnu++17 -Wall -Wno-unused-parameter -Wno-unused-function \
        -DESP_ARDUINO_VERSION_MAJOR=${CORE} -I. sim.cpp -o out/sim_core${CORE}; then
    echo "BUILD FAILED"; exit 1
  fi
done
# all scenarios with the 3.x build, the main ones again with the 2.x build
for S in $(seq 1 17); do
  if out/sim_core3 "$S" > "out/scenario_${S}.txt" 2>&1; then r=ok; else r=FAILED; FAILED=1; fi
  printf "  %-70s %s\n" "$(grep -m1 '^== ' out/scenario_${S}.txt | cut -c4-)" "$r"
done
for S in 1 3 4; do
  if out/sim_core2 "$S" > "out/scenario_${S}_core2.txt" 2>&1; then r=ok; else r=FAILED; FAILED=1; fi
  printf "  %-70s %s\n" "(core 2.x) $(grep -m1 '^== ' out/scenario_${S}_core2.txt | cut -c4-)" "$r"
done
echo
grep -h "^FAIL" out/scenario_*.txt && echo
if [ "$FAILED" = 0 ]; then echo "ALL SCENARIOS PASSED ($(cat out/scenario_*.txt | grep -c '^PASS') checks)"; else echo "SOME SCENARIOS FAILED (see out/)"; fi
exit $FAILED
