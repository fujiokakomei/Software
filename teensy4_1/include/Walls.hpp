#pragma once

#include <Arduino.h>
#include "upperVL53.hpp"
#include "Sensors.hpp"

namespace uppervl53s {class UpperVL53s;}
namespace sensors {class SensorManager;}

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

    const float frontSlopeThresholdLowerSensor = 130.5f; // Fusionでの計測値110.5 + 20
    const float frontSlopeThresholdUpperSensor = 254.5f; // Fusionでの計測値234.5 + 20

    const float frontDownSlopeThresholdLowerSensor =250.0f; // Fusionでのクソガバ計測値

    uint16_t distance_data[12] = {0};
    uppervl53s::UpperVL53s& _vl53s;

    sensors::SensorManager& _sensors;

public:
    Walls(uppervl53s::UpperVL53s& vl53s, 
        sensors::SensorManager& sensors) : _vl53s(vl53s), _sensors(sensors) {};

    void readWall(std::array<bool, 4>& wall);
    int readFront(); // 0:特に言うことなし、1:坂 (2以降に左右障害物を追加するかもしれない)
};
}