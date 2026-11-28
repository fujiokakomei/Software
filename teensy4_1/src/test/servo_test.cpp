// 投下機構の動作確認。起動後、右→左の順に1回ずつ投下動作をする。
//   pio run -e servo_test -t upload
#include <Arduino.h>

#include "RescueKits.hpp"

rescuekits::RescueKitManager rightKit;
rescuekits::RescueKitManager leftKit;

int rightKitPin = 22; // 普段135度、投下時180度
int leftKitPin = 23;  // 普段45度、投下時0度
int led_pin = 32;

void setup()
{
    rightKit.setupRescueKits(rightKitPin, led_pin, 1, 105, 180);
    leftKit.setupRescueKits(leftKitPin, led_pin, 1, 75, 0);
    delay(2000);

    rightKit.deployRescueKit(1, true);
    delay(1000);
    leftKit.deployRescueKit(1, true);
}

void loop(){}
