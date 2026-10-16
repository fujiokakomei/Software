#include <Arduino.h>

#include "LoadCell.hpp"

loadcells::LoadCellManager _loadcell;

int right = 24;
int left = 25;

float right_val;
float left_val;

void setup()
{
    _loadcell.setupLoadCells(right, left);
    Serial.begin(115200);
}

void loop()
{
    right_val = _loadcell.readRightLoadCell();
    left_val = _loadcell.readLeftLoadCell();
    Serial.print(right_val);
    Serial.print("   ");
    Serial.println(left_val);
    delay(100);
}