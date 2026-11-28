#pragma once

#include <Arduino.h>

// デバッグ用のUSBシリアル出力。
//   DBG("書式", ...)            : 1行出力（先頭に [時刻ms] を付ける）
//   DBG_EVERY(間隔ms, "書式", ...) : 同じ場所からは間隔ms以上空けて出力（ループ内用）
//
// 0 にすると全てのデバッグ出力がコンパイルされなくなる（競技本番で切りたいとき）。
#ifndef DEBUG_PRINT_ENABLED
#define DEBUG_PRINT_ENABLED 1
#endif

namespace debuglog
{
// 1行の最大長。送信バッファにこれだけの空きがないときは出力を捨てて、走行を止めない。
constexpr int kMaxLineLength = 256;

inline bool canPrint()
{
    // シリアルモニタが開かれていない、または送信が詰まっているときは出力しない
    return Serial && Serial.availableForWrite() >= kMaxLineLength;
}
} // namespace debuglog

#if DEBUG_PRINT_ENABLED

#define DBG(...)                                        \
    do                                                  \
    {                                                   \
        if (debuglog::canPrint())                       \
        {                                               \
            Serial.printf("[%lu] ", millis());          \
            Serial.printf(__VA_ARGS__);                 \
            Serial.println();                           \
        }                                               \
    } while (0)

#define DBG_EVERY(intervalMs, ...)                                  \
    do                                                              \
    {                                                               \
        static unsigned long _dbgLastMs = 0;                        \
        if (millis() - _dbgLastMs >= (unsigned long)(intervalMs))   \
        {                                                           \
            _dbgLastMs = millis();                                  \
            DBG(__VA_ARGS__);                                       \
        }                                                           \
    } while (0)

#else

#define DBG(...) do {} while (0)
#define DBG_EVERY(intervalMs, ...) do {} while (0)

#endif
