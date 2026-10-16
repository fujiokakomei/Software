#pragma once

namespace tb67
{

class ControlMotors
{
public:
    int m1_1;
    int m1_2; //    M4------M1
    int m2_1; //    |       |
    int m2_2; //    |       |
    int m3_1; //    |       |
    int m3_2; //    M3------M2
    int m4_1;
    int m4_2;

    void setupMotors(int _m1_1, int _m1_2, int _m2_1, int _m2_2, int _m3_1, int _m3_2, int _m4_1, int _m4_2);
    void moveForward(int speed);
    void moveBackward(int speed);
    void turnRight();
    void turnLeft();
    void stopRobot();
    void brakeRobot();

    void moveLeftSide(int speed);
    void moveRightSide(int speed);
};

} // namespace tb67
