#include "Motor.hpp"

#include <Arduino.h>

namespace tb67
{

void ControlMotors::setupMotors(int _m1_1, int _m1_2, int _m2_1, int _m2_2, int _m3_1, int _m3_2, int _m4_1, int _m4_2)
{
    m1_1 = _m1_1;
    m1_2 = _m1_2;
    m2_1 = _m2_1;
    m2_2 = _m2_2;
    m3_1 = _m3_1;
    m3_2 = _m3_2;
    m4_1 = _m4_1;
    m4_2 = _m4_2;
    pinMode(m1_1, OUTPUT);
    pinMode(m1_2, OUTPUT);
    pinMode(m2_1, OUTPUT);
    pinMode(m2_2, OUTPUT);
    pinMode(m3_1, OUTPUT);
    pinMode(m3_2, OUTPUT);
    pinMode(m4_1, OUTPUT);
    pinMode(m4_2, OUTPUT);
    digitalWrite(m1_1, LOW);
    digitalWrite(m1_2, LOW);
    digitalWrite(m2_1, LOW);
    digitalWrite(m2_2, LOW);
    digitalWrite(m3_1, LOW);
    digitalWrite(m3_2, LOW);
    digitalWrite(m4_1, LOW);
    digitalWrite(m4_2, LOW);
}

void ControlMotors::moveForward(int speed)
{
    if (speed >= 0 && speed <= 255)
    {
        analogWrite(m1_1, speed);
        analogWrite(m1_2, 0);
        analogWrite(m2_1, speed);
        analogWrite(m2_2, 0);
        analogWrite(m3_1, speed);
        analogWrite(m3_2, 0);
        analogWrite(m4_1, speed);
        analogWrite(m4_2, 0);
    }
}

void ControlMotors::moveBackward(int speed)
{
    if (speed >= 0 && speed <= 255)
    {
        analogWrite(m1_1, 0);
        analogWrite(m1_2, speed);
        analogWrite(m2_1, 0);
        analogWrite(m2_2, speed);
        analogWrite(m3_1, 0);
        analogWrite(m3_2, speed);
        analogWrite(m4_1, 0);
        analogWrite(m4_2, speed);
    }
}

void ControlMotors::turnRight()
{
    digitalWrite(m1_1, LOW);
    digitalWrite(m1_2, HIGH);
    digitalWrite(m2_1, LOW);
    digitalWrite(m2_2, HIGH);
    digitalWrite(m3_1, HIGH);
    digitalWrite(m3_2, LOW);
    digitalWrite(m4_1, HIGH);
    digitalWrite(m4_2, LOW);
}

void ControlMotors::turnLeft()
{
    digitalWrite(m1_1, HIGH);
    digitalWrite(m1_2, LOW);
    digitalWrite(m2_1, HIGH);
    digitalWrite(m2_2, LOW);
    digitalWrite(m3_1, LOW);
    digitalWrite(m3_2, HIGH);
    digitalWrite(m4_1, LOW);
    digitalWrite(m4_2, HIGH);
}

void ControlMotors::stopRobot()
{
    digitalWrite(m1_1, LOW);
    digitalWrite(m1_2, LOW);
    digitalWrite(m2_1, LOW);
    digitalWrite(m2_2, LOW);
    digitalWrite(m3_1, LOW);
    digitalWrite(m3_2, LOW);
    digitalWrite(m4_1, LOW);
    digitalWrite(m4_2, LOW);
}

void ControlMotors::brakeRobot()
{
    digitalWrite(m1_1, HIGH);
    digitalWrite(m1_2, HIGH);
    digitalWrite(m2_1, HIGH);
    digitalWrite(m2_2, HIGH);
    digitalWrite(m3_1, HIGH);
    digitalWrite(m3_2, HIGH);
    digitalWrite(m4_1, HIGH);
    digitalWrite(m4_2, HIGH);
}

void ControlMotors::moveLeftSide(int speed) // 車体左側のモーター二つだけ制御関数
{
    if (speed >= 0 && speed <= 255)
    {
        analogWrite(m3_1, speed);
        analogWrite(m3_2, 0);
        analogWrite(m4_1, speed);
        analogWrite(m4_2, 0);
    }
    else if (speed >= -255)
    {
        analogWrite(m3_1, 0);
        analogWrite(m3_2, speed);
        analogWrite(m4_1, 0);
        analogWrite(m4_2, speed);
    }
}

void ControlMotors::moveRightSide(int speed) // 車体右側のモーター二つだけ制御関数
{
    if (speed >= 0 && speed <= 255)
    {
        analogWrite(m1_1, speed);
        analogWrite(m1_2, 0);
        analogWrite(m2_1, speed);
        analogWrite(m2_2, 0);
    }
    else if (speed >= -255)
    {
        analogWrite(m1_1, 0);
        analogWrite(m1_2, speed);
        analogWrite(m2_1, 0);
        analogWrite(m2_2, speed);
    }
}

} // namespace tb67
