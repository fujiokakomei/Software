#include <Arduino.h>

#include "upperVL53.hpp"

namespace
{
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kPrintIntervalMs = 100;
constexpr uint32_t kUsbSerialWaitMs = 2000;

uppervl53s::UpperVL53s upperVL53s(Serial8);
uint16_t distances[uppervl53s::UpperVL53s::NUM_SENSORS] = {0};
uint16_t previousDistances[uppervl53s::UpperVL53s::NUM_SENSORS] = {0};
bool hasPreviousDistances = false;

void printRawData(const uppervl53s::UpperVL53s::ReceiveDebugInfo& debugInfo)
{
    Serial.print(F("  received: bytes="));
    Serial.print(debugInfo.bytesRead);
    Serial.print(F(", valid frames="));
    Serial.print(debugInfo.validFrames);
    Serial.print(F(", CRC errors="));
    Serial.println(debugInfo.checksumErrors);

    Serial.print(F("  raw RX (hex): "));
    if (debugInfo.rawDataSize == 0)
    {
        Serial.println(F("(no data received)"));
        return;
    }

    for (size_t i = 0; i < debugInfo.rawDataSize; ++i)
    {
        if (i != 0)
        {
            Serial.print(' ');
        }
        if (debugInfo.rawData[i] < 0x10)
        {
            Serial.print('0');
        }
        Serial.print(debugInfo.rawData[i], HEX);
    }
    if (debugInfo.rawDataTruncated)
    {
        Serial.print(F(" ... (truncated)"));
    }
    Serial.println();
}

void printDistances(
    int availableBefore,
    int availableAfter,
    const uppervl53s::UpperVL53s::ReceiveDebugInfo& debugInfo)
{
    //Serial.print(F("[upper_vl53] t="));
    //Serial.print(millis());
    //Serial.print(F(" ms, Serial8 available="));
    //Serial.print(availableBefore);
    //Serial.print(F(" -> "));
    //Serial.print(availableAfter);
    //Serial.println(F(" bytes"));

    //printRawData(debugInfo);

    Serial.print(F("  distances (mm): "));
    for (size_t i = 0; i < uppervl53s::UpperVL53s::NUM_SENSORS; ++i)
    {
        if (i != 0)
        {
            Serial.print(F(", "));
        }
        Serial.print(i);
        Serial.print(':');
        Serial.print(distances[i]);
    }
    Serial.println();

    /*
    if (!hasPreviousDistances)
    {
        Serial.println(F("  changes: first sample"));
        memcpy(previousDistances, distances, sizeof(distances));
        hasPreviousDistances = true;
        return;
    }
    */
    /*
    Serial.print(F("  changes: "));
    bool changed = false;
    for (size_t i = 0; i < uppervl53s::UpperVL53s::NUM_SENSORS; ++i)
    {
        if (distances[i] == previousDistances[i])
        {
            continue;
        }

        if (changed)
        {
            Serial.print(F(", "));
        }
        Serial.print(i);
        Serial.print(':');
        Serial.print(previousDistances[i]);
        Serial.print(F("->"));
        Serial.print(distances[i]);
        changed = true;
    }
    if (!changed)
    {
        Serial.print(F("none"));
    }
    */
    Serial.println();

    memcpy(previousDistances, distances, sizeof(distances));
}
} // namespace

void setup()
{
    Serial.begin(kSerialBaud);
    Serial8.begin(kSerialBaud); // RP2040 からの上部 VL53 データ

    const uint32_t waitStart = millis();
    while (!Serial && millis() - waitStart < kUsbSerialWaitMs)
    {
    }

    Serial.println(F("Upper VL53 test started"));
    Serial.print(F("Debug UART: Serial8 @ "));
    Serial.print(kSerialBaud);
    Serial.println(F(" baud"));
    Serial.println(F("Output: elapsed time, pending RX bytes, all distances, and changes"));
}

void loop()
{
    // 有効なフレームを受信していない間は、直前の値（起動直後は 0）を表示する。
    const int availableBefore = Serial8.available();
    uppervl53s::UpperVL53s::ReceiveDebugInfo debugInfo;
    upperVL53s.getVL53s(distances, &debugInfo);
    const int availableAfter = Serial8.available();
    printDistances(availableBefore, availableAfter, debugInfo);
    delay(kPrintIntervalMs);
}
