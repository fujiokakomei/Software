#pragma once

#include <Arduino.h>
#include "upperVL53.hpp"

namespace uppervl53s {class UpperVL53s;}

namespace walls
{
class Walls
{
private:
    // Control.cppの21~24行の値 + 20
    const float frontWallThreshold = 136.0f;
    const float backWallThreshold = 127.0f;
    const float rightWallThreshold = 131.7f;
    const float leftWallThreshold = 131.7f;

    const float frontSlopeThresholdUpperSensor = 254.5f; // Fusionでの計測値234.5 + 20

    uint16_t distance_data[12] = {0};
    uppervl53s::UpperVL53s& _vl53s;

public:
    explicit Walls(uppervl53s::UpperVL53s& vl53s) : _vl53s(vl53s) {};

    void readWall(std::array<bool, 4>& wall);
    // 上部VL53のみで判定するため、下り坂は検出できない。
    int readFront(); // 0:特に言うことなし、1:上り坂の可能性
};
}
