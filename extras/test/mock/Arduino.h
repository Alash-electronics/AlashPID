// Имитация Arduino.h для тестов AlashPID: часы millis() и Print, который печатает в консоль и копит текст.
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>
class __FlashStringHelper;
#define F(x) (reinterpret_cast<const __FlashStringHelper *>(x))
#define PROGMEM
inline uint32_t g_ms = 1000;
inline uint32_t millis() { return g_ms; }
class Print {
 public:
  std::string text;
  void put(const char *s) { printf("%s", s); text += s; }
  void print(const char *s) { put(s); }
  void print(const __FlashStringHelper *s) { put((const char *)s); }
  void println(const __FlashStringHelper *s) { put((const char *)s); put("\n"); }
  void print(long v) { char b[24]; snprintf(b, sizeof b, "%ld", v); put(b); }
  void println(long v) { print(v); put("\n"); }
  void print(double v, int d = 2) { char b[40]; snprintf(b, sizeof b, "%.*f", d, v); put(b); }
  void println(double v, int d = 2) { print(v, d); put("\n"); }
  void println(const char *s = "") { put(s); put("\n"); }
};
