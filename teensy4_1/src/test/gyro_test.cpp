#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

Adafruit_BNO055 bno(55, 0x28, &Wire1);

void setup()
{
    Serial.begin(115200);
    Serial.println("Serial begin");
    Wire1.begin();
    Wire1.setClock(400000);

    if (!bno.begin())
    {
        Serial.println("BNO055 initialization failed");
        while (true)
        {
            delay(10);
        }
    }

    bno.setExtCrystalUse(true);
    bno.setMode(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
    delay(100);
}

void loop()
{
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

    Serial.print(event.orientation.x);
    Serial.print(", ");
    Serial.print(event.orientation.y);
    Serial.print(", ");
    Serial.println(event.orientation.z);

    delay(100);
}