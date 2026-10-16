#pragma once
#include <Arduino.h>
#include <Servo.h>

namespace rescuekits
{
class RescueKitManager
{
public:
    Servo servo;

    int ServoPin; //サーボモーターのピン
    int ledPin;        //LEDのピン

    int remainingKits; //残りキット数

    void setupRescueKits(int ServoPin, int ledPin, int rescuekits);
    void deployRescueKit(int number, bool is_led);
};
}