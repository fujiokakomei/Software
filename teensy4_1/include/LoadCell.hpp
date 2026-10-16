#pragma once

#include <Arduino.h>

namespace loadcells
{
class LoadCellManager
{
public:
    void setupLoadCells(int rightAnalogPin, int leftAnalogPin);
    int readLoadCell(); //0: なし、1: 右、2: 左、3: 両方
    float readRightLoadCell();
    float readLeftLoadCell();

private:
    float readAveragedAnalog(int pin, int samples);
    float readCalibratedAnalog(int pin, float offset);

    int rightPin = -1;
    int leftPin = -1;
    float rightOffset = 0.0f;
    float leftOffset = 0.0f;
};
}
