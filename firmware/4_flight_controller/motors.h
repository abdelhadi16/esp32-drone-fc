#pragma once
// ESC output (LEDC hardware PWM) and the Quad-X motor mixer.
//
//   Motor layout seen from above, front at the top:
//        M4 (CW)    M2 (CCW)
//             \  /
//             /  \    (X frame)
//        M3 (CCW)   M1 (CW)
#include "driver/gpio.h"

constexpr uint8_t ESC_RES_BITS = 14;
uint16_t motorOut[4] = {MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF};

void escWriteUs(uint8_t m, uint16_t us) {
  const uint32_t periodUs = 1000000UL / ESC_FREQ_HZ;
  const uint32_t maxDuty  = (1UL << ESC_RES_BITS) - 1;
  const uint32_t duty = (uint32_t)us * maxDuty / periodUs;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PINS[m], duty);
#else
  ledcWrite(m, duty);
#endif
}

void motorsAll(uint16_t us) {
  for (uint8_t m = 0; m < 4; m++) { motorOut[m] = us; escWriteUs(m, us); }
}

void motorsOff() { motorsAll(MOTOR_OFF); }

// Must run FIRST in setup(): the ESCs see "off" right away, and the motors
// take LEDC channels 0-3 before anything else (the buzzer uses channel 4).
void escInit() {
  for (uint8_t m = 0; m < 4; m++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(MOTOR_PINS[m], ESC_FREQ_HZ, ESC_RES_BITS);
#else
    ledcSetup(m, ESC_FREQ_HZ, ESC_RES_BITS);
    ledcAttachPin(MOTOR_PINS[m], m);
#endif
    gpio_set_drive_capability((gpio_num_t)MOTOR_PINS[m], GPIO_DRIVE_CAP_3);
  }
  motorsOff();
}

// thr = collective throttle (us); roll/pitch/yaw = corrections from the PIDs
//   +roll  = more power on the LEFT motors  (M3, M4)
//   +pitch = more power on the FRONT motors (M2, M4)
//   +yaw   = more power on the CCW motors   (M2, M3)
void motorsMix(float thr, float roll, float pitch, float yaw) {
  const float mix[4] = {
    thr - roll - pitch - yaw,   // M1 rear-right  CW
    thr - roll + pitch + yaw,   // M2 front-right CCW
    thr + roll - pitch + yaw,   // M3 rear-left   CCW
    thr + roll + pitch - yaw    // M4 front-left  CW
  };
  for (uint8_t m = 0; m < 4; m++) {
    motorOut[m] = (uint16_t)clampf(mix[m], (float)MOTOR_IDLE, (float)MOTOR_MAX);
    escWriteUs(m, motorOut[m]);
  }
}
