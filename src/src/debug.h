#ifndef DEBUG_H_
#define DEBUG_H_

#include <Arduino.h>
#include "targets.h"

#ifndef DEBUG_SERIAL

#if defined(PLATFORM_ESP32)
#define DEBUG_SERIAL Serial
#elif defined(TARGET_R9M_TX)
#define DEBUG_SERIAL Serial
#elif defined(PLATFORM_ESP8266) || defined(TARGET_R9M_RX)
#define DEBUG_SERIAL CrsfSerial
#endif

#endif // ifndef DEBUG_SERIAL

#ifdef DEBUG_SERIAL
// Debug enabled
#define DEBUG_PRINT(...) DEBUG_SERIAL.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) DEBUG_SERIAL.println(__VA_ARGS__)
#else
// Debug disabled
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

#endif // DEBUG_H_