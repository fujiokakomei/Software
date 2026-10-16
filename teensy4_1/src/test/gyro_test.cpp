#include <Arduino.h>

#include "Sensors.hpp"

int ledPinTCS = 40;
sensors::SensorManager sensorManager;

float yawoffset, rolloffset, pitchoffset, x, y, z;

void setup()
{
    sensorManager.setupSensors(ledPinTCS);
    delay(500);

    Serial.begin(115200);
}

void loop()
{
    sensorManager.readOrientation(x, y, z);
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.println(z);

    delay(100);
}