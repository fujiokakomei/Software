#pragma once

#include <Arduino.h>
#include <array>
#include "upperVL53.hpp"

namespace walls
{
class Walls
{
private:
    static constexpr int kFrontSensor = 0;
    static constexpr int kRightSensor = 3;
    static constexpr int kBackSensor = 6;
    static constexpr int kLeftSensor = 9;

    // Control.cppの21~24行の値 + 20 + 20
    const float frontWallThreshold = 156.0f;
    const float backWallThreshold = 147.0f;
    const float rightWallThreshold = 151.7f;
    const float leftWallThreshold = 151.7f;

    uint16_t distance_data[uppervl53s::UpperVL53s::NUM_SENSORS] = {0};
    uppervl53s::UpperVL53s& _vl53s;

public:
    explicit Walls(uppervl53s::UpperVL53s& vl53s) : _vl53s(vl53s) {};

    // 前・右・後・左（ロボット基準）の壁の有無
    void readWall(std::array<bool, 4>& wall);
    // 坂は走行中にジャイロの傾きで検出する（ControlRobot / SlopeDetector）。
};
}
