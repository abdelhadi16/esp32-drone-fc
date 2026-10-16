#pragma once
// =====================================================================
//  USER SETTINGS  —  the only file you normally need to edit.
//  Everything is explained in docs/ (see docs/README.md for the index).
// =====================================================================

// ---------------------------------------------------------------------
//  1. From sketch 3 (guide Step 8): how the MPU6050 sits on the frame
// ---------------------------------------------------------------------
#define IMU_ROTATION      0          // X arrow on the GY-521: 0 forward, 90 right, 180 back, 270 left
#define IMU_UPSIDE_DOWN   false      // true only if the chip faces the ground
#define ACC_ROLL_TRIM     0.00f      // the two numbers sketch 3 printed after 'l'
#define ACC_PITCH_TRIM    0.00f

// ---------------------------------------------------------------------
//  2. From sketch 1 (guide Steps 2-5): ESC signal
// ---------------------------------------------------------------------
#define ESC_FREQ_HZ       250        // fastest speed that armed + ran smoothly ('f' in sketch 1)
#define MOTOR_OFF         1000       // "off" pulse that lets the ESCs arm ('o' in sketch 1)
#define MOTOR_IDLE        1100       // armed with throttle down: motors spin slowly
#define MOTOR_MAX         2000
#define THROTTLE_MAX      1800       // keeps room for corrections at full throttle

// ---------------------------------------------------------------------
//  3. Pins (your PCB)
// ---------------------------------------------------------------------
static const uint8_t MOTOR_PINS[4] = {10, 11, 12, 13};   // H6 pins 1..4 = M1..M4
#define IBUS_RX_PIN       18         // H2 pin 2
#define I2C_SDA_PIN       9          // MPU6050 + BMP280
#define I2C_SCL_PIN       8
#define VBAT_PIN          1          // H5: battery divider R3 10k / R4 3.3k
#define BUZZER_PIN        17         // H3 pin 3 (the unused GPS connector). -1 = no buzzer
#define BUZZER_ACTIVE_HIGH true      // "high level trigger" buzzer module
#define BUZZER_PASSIVE    false      // true = bare piezo that needs a tone
#define RGB_LED_PIN       48         // RGB LED on the DevKit (38 on some boards). -1 = off
#define CURRENT_PIN       -1         // optional current sensor output on an ADC1 pin (e.g. 2). -1 = none

// ---------------------------------------------------------------------
//  4. Radio channels (FS-i6X, i-BUS). Channel numbers start at 0 here.
// ---------------------------------------------------------------------
#define CH_ROLL           0          // CH1
#define CH_PITCH          1          // CH2
#define CH_THR            2          // CH3
#define CH_YAW            3          // CH4
#define CH_ARM            4          // CH5 = SwA : ~2000 ARM, ~1000 SAFE (= KILL switch)
#define CH_MODE           5          // CH6 = SwC : ~1000 MANUAL, ~1500 ALT HOLD, ~2000 LAND

// ---------------------------------------------------------------------
//  5. How the drone answers the sticks (MANUAL mode = self-levelling)
// ---------------------------------------------------------------------
constexpr float MAX_ANGLE_DEG  = 30.0f;   // full right stick = this tilt
constexpr float LEVEL_GAIN     = 5.0f;    // how hard it pulls back to level
constexpr float MAX_YAW_RATE   = 150.0f;  // deg/s at full yaw stick

// ---------------------------------------------------------------------
//  6. Attitude PID (start values for F330 + A2212 2200KV + 6" props)
// ---------------------------------------------------------------------
constexpr float P_ROLL  = 1.3f,  I_ROLL = 0.04f, D_ROLL = 18.0f;   // pitch uses the same
constexpr float P_YAW   = 4.0f,  I_YAW  = 0.02f, D_YAW  = 0.0f;
constexpr float PID_MAX = 400.0f;

// ---------------------------------------------------------------------
//  7. Battery management
// ---------------------------------------------------------------------
#define BATTERY_CELLS     3
constexpr float VBAT_DIVIDER     = (10000.0f + 3300.0f) / 3300.0f;   // (R3 + R4) / R4
constexpr float VBAT_CAL         = 1.00f;   // = multimeter volts / volts shown by the drone
// The thresholds use the RESTING voltage per cell (sag compensated, see battery.h):
constexpr float CELL_WARN_V      = 3.75f;   // ~25 % left, 5 s -> beep every second, yellow LED
constexpr float CELL_CRITICAL_V  = 3.65f;   // ~7 % left, 5 s  -> AUTOMATIC LANDING
constexpr float CELL_ABS_MIN_V   = 3.30f;   // measured under load, 5 s -> also AUTOMATIC LANDING
constexpr float CELL_MIN_ARM_V   = 3.75f;   // at rest: refuse to arm below this
constexpr float CURRENT_A_PER_V  = 0.0f;    // current sensor: amps per volt of output
constexpr float CURRENT_OFFSET_V = 0.0f;    // current sensor: output volts at 0 A

// ---------------------------------------------------------------------
//  8. Altitude hold (BMP280 barometer + accelerometer)
// ---------------------------------------------------------------------
#define ALT_HOLD_ENABLED  true
constexpr float ALT_P            = 0.8f;    // (m/s) of climb per metre of altitude error
constexpr float VEL_P            = 80.0f;   // throttle us per (m/s) of vertical speed error
constexpr float VEL_I            = 40.0f;   // throttle us per (m/s x s): fixes hover throttle error
constexpr float VEL_I_MAX        = 150.0f;  // limit of that correction (us)
constexpr float ALT_MAX_CLIMB    = 0.8f;    // m/s with the throttle stick fully up (ALT HOLD)
constexpr float ALT_MAX_DESCENT  = 0.6f;    // m/s with the throttle stick fully down
constexpr int   ALT_STICK_DEADBAND = 100;   // us around 1500 where the drone just holds height
constexpr float HOVER_THROTTLE_GUESS = 1500.0f;  // first guess; learned during flight

// ---------------------------------------------------------------------
//  9. Automatic landing
// ---------------------------------------------------------------------
constexpr float LAND_SPEED_FAST  = 0.6f;    // m/s while higher than LAND_SLOW_ALT
constexpr float LAND_SPEED_SLOW  = 0.3f;    // m/s for the last metres
constexpr float LAND_SLOW_ALT    = 1.5f;    // m above the take-off point
constexpr float LAND_TIMEOUT_S   = 30.0f;   // safety net: s below LAND_SLOW_ALT, then motors stop
constexpr float LAND_NOBARO_DROP = 40.0f;   // no barometer: throttle this far below hover...
constexpr float LAND_NOBARO_TIME_S = 20.0f; // ...for this long, then motors stop (firm landing!)

// ---------------------------------------------------------------------
// 10. Flight log (stored in PSRAM during flight, saved to flash after)
// ---------------------------------------------------------------------
#define LOG_ENABLED       true
#define LOG_HZ            50
#define LOG_MAX_FILES     20

// Print status / events on the Serial Monitor (never slows the loop in flight)
#define DEBUG             true
