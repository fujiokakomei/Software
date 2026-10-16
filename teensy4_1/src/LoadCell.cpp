#include "LoadCell.hpp"

#include <math.h>

namespace loadcells
{

namespace
{
constexpr int CalibrationSamples = 20;
constexpr int ReadSamples = 10;
constexpr float LoadThreshold = 10.0f;
constexpr float ScaleFactor = 1.0f; //必要に応じて実測値で調整
}

void LoadCellManager::setupLoadCells(int rightAnalogPin, int leftAnalogPin)
{
    rightPin = rightAnalogPin;
    leftPin = leftAnalogPin;

    pinMode(rightPin, INPUT);
    pinMode(leftPin, INPUT);

    rightOffset = readAveragedAnalog(rightPin, CalibrationSamples); //右のロードセルのゼロ点を設定
    leftOffset = readAveragedAnalog(leftPin, CalibrationSamples);   //左のロードセルのゼロ点を設定
}

int LoadCellManager::readLoadCell()
{
    const bool rightLoaded = readRightLoadCell() > LoadThreshold;
    const bool leftLoaded = readLeftLoadCell() > LoadThreshold;

    if (rightLoaded && leftLoaded)
    {
        return 3; //両方
    }
    if (rightLoaded)
    {
        return 1; //右
    }
    if (leftLoaded)
    {
        return 2; //左
    }
    return 0; //なし
}

float LoadCellManager::readRightLoadCell()
{
    return readCalibratedAnalog(rightPin, rightOffset);
}

float LoadCellManager::readLeftLoadCell()
{
    return readCalibratedAnalog(leftPin, leftOffset);
}

float LoadCellManager::readAveragedAnalog(int pin, int samples)
{
    if (pin < 0 || samples <= 0)
    {
        return 0.0f;
    }

    uint32_t total = 0;
    for (int i = 0; i < samples; ++i)
    {
        total += analogRead(pin);
        delayMicroseconds(100);
    }

    return static_cast<float>(total) / static_cast<float>(samples);
}

float LoadCellManager::readCalibratedAnalog(int pin, float offset)
{
    const float rawValue = readAveragedAnalog(pin, ReadSamples);
    return fabsf(rawValue - offset) / ScaleFactor;
}

} // namespace loadcells
