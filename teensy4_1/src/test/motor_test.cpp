#include <Arduino.h>

#include "Motor.hpp"

tb67::ControlMotors _motors;

int m1_1 = 0;
int m1_2 = 1; //    M4------M1
int m2_1 = 2; //    |       |
int m2_2 = 3; //    |       |
int m3_1 = 4; //    |       |
int m3_2 = 5; //    M3------M2
int m4_1 = 6;
int m4_2 = 7;

void setup()
{
    _motors.setupMotors(m1_1, m1_2, m2_1, m2_2, m3_1, m3_2, m4_1, m4_2);
    delay(1000);

    for (int i = 100; i < 255; i++)
    {
        analogWrite(m1_1, i);
        analogWrite(m1_2, 0);
        delay(50);
    }
    for (int i = 100; i < 255; i++)
    {
        analogWrite(m1_1, 0);
        analogWrite(m1_2, i);
        delay(50);
    }
    analogWrite(m1_1, LOW);
    analogWrite(m1_2, LOW);


    for (int i = 100; i < 255; i++)
    {
        analogWrite(m2_1, i);
        analogWrite(m2_2, 0);
        delay(50);
    }
    for (int i = 100; i < 255; i++)
    {
        analogWrite(m2_1, 0);
        analogWrite(m2_2, i);
        delay(50);
    }
    analogWrite(m2_1, LOW);
    analogWrite(m2_2, LOW);


    for (int i = 100; i < 255; i++)
    {
        analogWrite(m3_1, i);
        analogWrite(m3_2, 0);
        delay(50);
    }
    for (int i = 100; i < 255; i++)
    {
        analogWrite(m3_1, 0);
        analogWrite(m3_2, i);
        delay(50);
    }
    analogWrite(m3_1, LOW);
    analogWrite(m3_2, LOW);


    for (int i = 100; i < 255; i++)
    {
        analogWrite(m4_1, i);
        analogWrite(m4_2, 0);
        delay(50);
    }
    for (int i = 100; i < 255; i++)
    {
        analogWrite(m4_1, 0);
        analogWrite(m4_2, i);
        delay(50);
    }
    analogWrite(m4_1, LOW);
    analogWrite(m4_2, LOW);


    _motors.moveForward(200);
    delay(1000);
    _motors.moveBackward(200);
    delay(1000);
    _motors.turnRight();
    delay(1000);
    _motors.turnLeft();
    delay(1000);
    _motors.brakeRobot();
    delay(100);
    _motors.moveRightSide(200);
    delay(1000);
    _motors.moveLeftSide(200);
    delay(1000);
    _motors.stopRobot();
}

void loop(){}