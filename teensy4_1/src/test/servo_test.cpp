#include <Arduino.h>

#include "RescueKits.hpp"

#include <Servo.h>

Servo servo;

int servo_pin = 23;
int led_pin = 29;

void setup()
{
    servo.attach(servo_pin);
    delay(100);

    servo.write(0);
    delay(500);
    servo.write(90);
    delay(500);
    servo.write(180);
    delay(500);

    for (int i = 0; i < 180; i++)
    {
        servo.write(i);
        delay(10);
    }
}

void loop(){}