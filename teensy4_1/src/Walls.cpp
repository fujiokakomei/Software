#include "Walls.hpp"

namespace walls
{

void Walls::readWall(std::array<bool, 4>& wall)
{
    _vl53s.getVL53s(distance_data);

    if (distance_data[0] < frontWallThreshold)
    {
        wall[0] = true;
    }
    else
    {
        wall[0] = false;
    }

    if (distance_data[1] < rightWallThreshold)
    {
        wall[1] = true;
    }
    else
    {
        wall[1] = false;
    }

    if (distance_data[2] < backWallThreshold)
    {
        wall[2] = true;
    }
    else
    {
        wall[2] = false;
    }

    if (distance_data[3] < leftWallThreshold)
    {
        wall[3] = true;
    }
    else
    {
        wall[3] = false;
    }
}

int Walls::readFront()
{
    _vl53s.getVL53s(distance_data);
    float lowerFrontDistance = _sensors.readCenterFrontDistance();

    if (lowerFrontDistance > frontDownSlopeThresholdLowerSensor)
    {
        return 1;
    }

    if (distance_data[0] < frontSlopeThresholdUpperSensor && distance_data[0] > frontWallThreshold && lowerFrontDistance < frontSlopeThresholdLowerSensor)
    {
        return 1;
    }

    return 0;
}

}