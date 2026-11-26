#include "Motor.hpp"

#include <Arduino.h>

namespace tb67
{

namespace
{

constexpr int kMaxPwm = 255;
// Teensyの8bit分解能では256を渡すと100%（常時HIGH）になる。
constexpr int kFullOnPwm = 256;

// 1つのモーターを駆動する。speed > 0 で機体が前進する向きに回す。
// motor_devブランチで確認した回転方向（_2側にPWMで前進）に合わせている。
// Teensy 4.x では analogWrite() した後のピンに digitalWrite() しても
// PWM出力のままになり停止しないことがあるため、すべて analogWrite() で出力する。
void driveMotor(int in1, int in2, int speed)
{
    speed = constrain(speed, -kMaxPwm, kMaxPwm);
    if (speed >= 0)
    {
        analogWrite(in1, 0);
        analogWrite(in2, speed);
    }
    else
    {
        analogWrite(in1, -speed);
        analogWrite(in2, 0);
    }
}

} // namespace

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
    stopRobot();
}

void ControlMotors::moveForward(int speed)
{
    if (speed >= 0 && speed <= kMaxPwm)
    {
        moveRightSide(speed);
        moveLeftSide(speed);
    }
}

void ControlMotors::moveBackward(int speed)
{
    if (speed >= 0 && speed <= kMaxPwm)
    {
        moveRightSide(-speed);
        moveLeftSide(-speed);
    }
}

void ControlMotors::turnRight() // 超信地旋回（右）
{
    moveRightSide(-kMaxPwm);
    moveLeftSide(kMaxPwm);
}

void ControlMotors::turnLeft() // 超信地旋回（左）
{
    moveRightSide(kMaxPwm);
    moveLeftSide(-kMaxPwm);
}

void ControlMotors::stopRobot() // 惰性で停止（全ピンLOW）
{
    analogWrite(m1_1, 0);
    analogWrite(m1_2, 0);
    analogWrite(m2_1, 0);
    analogWrite(m2_2, 0);
    analogWrite(m3_1, 0);
    analogWrite(m3_2, 0);
    analogWrite(m4_1, 0);
    analogWrite(m4_2, 0);
}

void ControlMotors::brakeRobot() // ブレーキ（全ピンHIGH）
{
    analogWrite(m1_1, kFullOnPwm);
    analogWrite(m1_2, kFullOnPwm);
    analogWrite(m2_1, kFullOnPwm);
    analogWrite(m2_2, kFullOnPwm);
    analogWrite(m3_1, kFullOnPwm);
    analogWrite(m3_2, kFullOnPwm);
    analogWrite(m4_1, kFullOnPwm);
    analogWrite(m4_2, kFullOnPwm);
}

void ControlMotors::moveLeftSide(int speed) // 車体左側のモーター二つだけ制御関数
{
    driveMotor(m3_1, m3_2, speed);
    driveMotor(m4_1, m4_2, speed);
}

void ControlMotors::moveRightSide(int speed) // 車体右側のモーター二つだけ制御関数
{
    driveMotor(m1_1, m1_2, speed);
    driveMotor(m2_1, m2_2, speed);
}

} // namespace tb67
