#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

namespace
{

constexpr int kTcsLedPin = 40;
constexpr unsigned long kSampleIntervalMs = 100;

Adafruit_TCS34725 tcs(
    TCS34725_INTEGRATIONTIME_50MS,
    TCS34725_GAIN_4X
);

void printSensorNotFound()
{
    Serial.println("TCS34725 was not found on Wire.");
    Serial.println("Check SDA, SCL, VIN, GND, and the I2C address.");
}

} // namespace

void setup()
{
    pinMode(kTcsLedPin, OUTPUT);
    digitalWrite(kTcsLedPin, HIGH);

    Serial.begin(115200);
    delay(500);
    Wire.begin();
    //Wire.setClock(400000);
    delay(500);

    if (!tcs.begin(TCS34725_ADDRESS, &Wire))
    {
        printSensorNotFound();
        digitalWrite(kTcsLedPin, LOW);
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("r,g,b,c,color_temp_k,lux");
}

void loop()
{
    uint16_t r = 0;
    uint16_t g = 0;
    uint16_t b = 0;
    uint16_t c = 0;
    tcs.getRawData(&r, &g, &b, &c);

    const uint16_t colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    const uint16_t lux = tcs.calculateLux(r, g, b);

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

    delay(kSampleIntervalMs);
}
