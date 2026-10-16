#pragma once
// Host simulation of the small part of the Arduino-ESP32 API the flight controller uses.
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <deque>
#include <string>
#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 3
#endif
#define F(x) x
#define SERIAL_8N1 0x800001c
#define LOW 0
#define HIGH 1
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
extern uint64_t g_us;
inline unsigned long micros() { g_us += 5; return (unsigned long)g_us; }
inline unsigned long millis() { return (unsigned long)(g_us / 1000); }
void sim_delay(uint32_t ms);
inline void delay(uint32_t ms) { sim_delay(ms); }
struct HS {
  std::deque<uint8_t> q; bool quiet = false; bool capture = false; std::string out;
  void begin(unsigned long, uint32_t = 0, int8_t = -1, int8_t = -1) {}
  int available() { return (int)q.size(); }
  int read() { if (q.empty()) return -1; int b = q.front(); q.pop_front(); return b; }
  void emit(const char *s) {
    if (capture) out += s;
    if (!quiet) { static bool bol = true; for (const char *c = s; *c; c++) { if (bol) ::printf("[%8.3f] ", g_us / 1e6); ::putchar(*c); bol = (*c == '\n'); } }
  }
  size_t print(const char *s) { emit(s); return strlen(s); }
  size_t println(const char *s = "") { emit(s); emit("\n"); return strlen(s) + 1; }
  int printf(const char *f, ...) __attribute__((format(printf, 2, 3))) {
    char b[512]; va_list a; va_start(a, f); int n = vsnprintf(b, sizeof(b), f, a); va_end(a); emit(b); return n; }
};
extern HS Serial, Serial1;
void sim_pinMode(uint8_t, uint8_t); void sim_digitalWrite(uint8_t, uint8_t);
inline void pinMode(uint8_t p, uint8_t m) { sim_pinMode(p, m); }
inline void digitalWrite(uint8_t p, uint8_t v) { sim_digitalWrite(p, v); }
extern uint32_t g_duty[64], g_freq[64]; extern uint8_t g_res[64];
#if ESP_ARDUINO_VERSION_MAJOR >= 3
inline bool ledcAttach(uint8_t pin, uint32_t f, uint8_t r) { g_freq[pin] = f; g_res[pin] = r; return true; }
inline bool ledcWrite(uint8_t pin, uint32_t duty) { g_duty[pin] = duty; return true; }
void sim_rgb(uint8_t pin, uint8_t r, uint8_t g, uint8_t b);
inline void rgbLedWrite(uint8_t pin, uint8_t r, uint8_t g, uint8_t b) { sim_rgb(pin, r, g, b); }
#else
extern uint8_t g_chpin[16];
inline uint32_t ledcSetup(uint8_t ch, uint32_t f, uint8_t r) { g_freq[32 + ch] = f; g_res[32 + ch] = r; return f; }
inline void ledcAttachPin(uint8_t pin, uint8_t ch) { g_chpin[ch] = pin; g_freq[pin] = g_freq[32 + ch]; g_res[pin] = g_res[32 + ch]; }
inline void ledcWrite(uint8_t ch, uint32_t duty) { g_duty[g_chpin[ch]] = duty; }
void sim_rgb(uint8_t pin, uint8_t r, uint8_t g, uint8_t b);
inline void neopixelWrite(uint8_t pin, uint8_t r, uint8_t g, uint8_t b) { sim_rgb(pin, r, g, b); }
#endif
uint32_t sim_adc_mv(uint8_t pin);
inline uint32_t analogReadMilliVolts(uint8_t pin) { return sim_adc_mv(pin); }
inline void analogReadResolution(uint8_t) {}
extern bool g_psram;
inline bool psramFound() { return g_psram; }
inline void *ps_malloc(size_t n) { return g_psram ? malloc(n) : nullptr; }
#include "driver/gpio.h"
