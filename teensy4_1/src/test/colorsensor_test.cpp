#include <Arduino.h>

#include "Sensors.hpp"

int ledPinTCS = 40;
sensors::SensorManager sensorManager;

uint16_t r, g, b, c;
float colorTemp, lux;

sensors::ColorTile colorTile;

void setup()
{
    sensorManager.setupSensors(ledPinTCS);
    delay(500);

    digitalWrite(ledPinTCS, HIGH);
    delay(1000);
    digitalWrite(ledPinTCS, LOW);
    delay(1000);
    digitalWrite(ledPinTCS, HIGH);

    Serial.begin(115200);
}

void loop()
{
    sensorManager.readColor(r, g, b, c, colorTemp, lux);
    Serial.print(r);
    Serial.print(", ");
    Serial.print(g);
    Serial.print(", ");
    Serial.print(b);
    Serial.print(", ");
    Serial.print(c);
    Serial.print(", ");
    Serial.print(colorTemp);
    Serial.print(", ");
    Serial.println(lux);

    /*
    colorTile = sensors::detectColorTile(sensorManager);
    if (colorTile == sensors::ColorTile::None)
    {
        Serial.println("None");
    }
    else if (colorTile == sensors::ColorTile::Blue)
    {
        Serial.println("Blue");
    }
    else if (colorTile == sensors::ColorTile::Red)
    {
        Serial.println("Red");
    }
    else if (colorTile == sensors::ColorTile::Silver)
    {
        Serial.println("Silver");
    }
    else if (colorTile == sensors::ColorTile::Black)
    {
        Serial.println("Black");
    }
    */
}