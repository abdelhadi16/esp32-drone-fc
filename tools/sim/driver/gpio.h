#pragma once
typedef enum { GPIO_NUM_0 = 0 } gpio_num_t;
typedef enum { GPIO_DRIVE_CAP_0 = 0, GPIO_DRIVE_CAP_1, GPIO_DRIVE_CAP_2, GPIO_DRIVE_CAP_3 } gpio_drive_cap_t;
typedef int esp_err_t;
inline esp_err_t gpio_set_drive_capability(gpio_num_t, gpio_drive_cap_t) { return 0; }
