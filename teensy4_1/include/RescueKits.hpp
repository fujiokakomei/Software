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

    int idleAngle = 0;   //普段の角度
    int deployAngle = 0; //投下時の角度

    // idleAngle: 普段の角度、deployAngle: 投下時の角度
    void setupRescueKits(int ServoPin, int ledPin, int rescuekits, int idleAngle, int deployAngle);
    void deployRescueKit(int number, bool is_led);
};
}
