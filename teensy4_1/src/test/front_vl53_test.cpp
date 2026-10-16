#include <Arduino.h>

#include "Sensors.hpp"

int ledPinTCS = 40;
sensors::SensorManager sensorManager;

float right, center, left;

void setup()
{
    sensorManager.setupSensors(ledPinTCS);
    delay(500);

    Serial.begin(115200);
}

void loop()
{
    right = sensorManager.readRightFrontDistance();
    center = sensorManager.readCenterFrontDistance();
    left = sensorManager.readLeftFrontDistance();

    Serial.print(left);
    Serial.print(", ");
    Serial.print(center);
    Serial.print(", ");
    Serial.println(right);

    delay(100);
}