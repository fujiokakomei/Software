#include <Arduino.h>

#include "upperVL53.hpp"

namespace
{
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kPrintIntervalMs = 100;

uppervl53s::UpperVL53s upperVL53s(Serial8);
uint16_t distances[uppervl53s::UpperVL53s::NUM_SENSORS] = {0};

void printDistances()
{
    for (size_t i = 0; i < uppervl53s::UpperVL53s::NUM_SENSORS; ++i)
    {
        if (i != 0)
        {
            Serial.print(',');
        }
        Serial.print(distances[i]);
    }
    Serial.println();
}
} // namespace

void setup()
{
    Serial.begin(kSerialBaud);
    Serial8.begin(kSerialBaud); // RP2040 からの上部 VL53 データ

    Serial.println(F("Upper VL53 test started"));
    Serial.println(F("CSV columns: sensor 0,1,2,3,4,5,6,7,8,9,10,11 (mm)"));
}

void loop()
{
    // 有効なフレームを受信していない間は、直前の値（起動直後は 0）を表示する。
    upperVL53s.getVL53s(distances);
    printDistances();
    delay(kPrintIntervalMs);
}
