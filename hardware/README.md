# Hardware files

| File | Content |
|------|---------|
| `schematic.png` | PCB schematic (ESP32-S3 DevKit carrier) |
| `pcb_3d_view.png` | 3D view of the PCB |

## Pin map used by the firmware

| Function | Connector / part | ESP32-S3 GPIO | `config.h` |
|----------|------------------|---------------|------------|
| ESC M1, M2, M3, M4 | H6 pins 1–4 (pin 5 GND) | 10, 11, 12, 13 | `MOTOR_PINS` |
| Receiver i-BUS | H2 pin 2 (1 = 5 V, 3 = GND) | 18 (UART1 RX) | `IBUS_RX_PIN` |
| I²C SCL / SDA (MPU6050 U2, BMP280 U3) | R1/R2 4.7 k pull-ups | 8 / 9 | `I2C_SCL_PIN` / `I2C_SDA_PIN` |
| Battery voltage | H5 → R3 10 k / R4 3.3 k | 1 (ADC1) | `VBAT_PIN`, `VBAT_DIVIDER` |
| Buzzer | H3 pin 3 (1 = 5 V, 4 = GND) | 17 | `BUZZER_PIN` |
| RGB LED | on the DevKit | 48 (38 on some boards) | `RGB_LED_PIN` |
| Board power in flight | H1 (5 V, 470 µF C1) | 5V0 | – |
| Not used | H4 (Pi UART) | 43 / 44 = UART0 | keep empty |

I²C addresses: MPU6050 0x68 (AD0 = GND), BMP280 0x76 or 0x77 (auto-detected; ours answers
at 0x77 although U3's SDO is tied to GND).

Full explanations, power rules and known issues: [docs/01-hardware.md](../docs/01-hardware.md).
