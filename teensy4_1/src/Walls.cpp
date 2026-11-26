#include "Walls.hpp"

namespace walls
{

void Walls::readWall(std::array<bool, 4>& wall)
{
    _vl53s.getVL53s(distance_data);

    // 上部VL53は30度刻みで 0:前、3:右、6:後、9:左（Control.cppと同じ並び）。
    wall[0] = distance_data[kFrontSensor] < frontWallThreshold;
    wall[1] = distance_data[kRightSensor] < rightWallThreshold;
    wall[2] = distance_data[kBackSensor] < backWallThreshold;
    wall[3] = distance_data[kLeftSensor] < leftWallThreshold;
}

}
