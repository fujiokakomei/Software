#pragma once

#include <Arduino.h>

namespace uppervl53s
{
class UpperVL53s
{
private:
    HardwareSerial& _rp2040;

public:
    static constexpr int NUM_SENSORS = 12;
    uint16_t distance_data[NUM_SENSORS] = {0};
    const int EXPECTED_BYTES = NUM_SENSORS * 2; // 24バイト

    UpperVL53s(HardwareSerial& rp2040) : _rp2040(rp2040) {};

    void getVL53s(uint16_t distance_data[NUM_SENSORS]);
};
}