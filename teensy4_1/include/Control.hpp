#pragma once

#include <Arduino.h>

namespace tb67 {class ControlMotors;}
namespace sensors {class SensorManager;}
namespace rescuekits {class RescueKitManager;}
namespace loadcells {class LoadCellManager;}
namespace uppervl53s {class UpperVL53s;}
namespace rcj_maze {class MazeMapper; struct FloorPosition;}

namespace controlrobot // Motor.cppとSensors.cppなどを使って細かい制御するやつ
{
class ControlRobot
{
private:
    tb67::ControlMotors& _motors;
    sensors::SensorManager& _sensors;

    HardwareSerial& _rightCam;
    HardwareSerial& _leftCam;
    //HardwareSerial& _rp2040;

    loadcells::LoadCellManager& _loadCells;

    uppervl53s::UpperVL53s& _vl53s;

    const float DELTA_T = 0.004;
    const float KP = 1.2;
    const float KI = 0.0;
    const float KD = 0.04;

    float _pidIntegral = 0.0f;
    float _pidPrevError = 0.0f;

    uint16_t distance_data[12] = {0};

public:
    rescuekits::RescueKitManager& _rightKits;
    rescuekits::RescueKitManager& _leftKits;

    ControlRobot(HardwareSerial& rightCam, 
        HardwareSerial& leftCam, 
        //HardwareSerial& rp2040,
        tb67::ControlMotors& motors, 
        sensors::SensorManager& sensors,
        rescuekits::RescueKitManager& rightKits,
        rescuekits::RescueKitManager& leftKits,
        loadcells::LoadCellManager& loadcell,
        uppervl53s::UpperVL53s& vl53s
    ) : _motors(motors), 
    _sensors(sensors), 
    _rightCam(rightCam), 
    _leftCam(leftCam), 
    //_rp2040(rp2040),
    _loadCells(loadcell),
    _vl53s(vl53s),
    _rightKits(rightKits), 
    _leftKits(leftKits) {}


    float convertSensorValue(uint16_t distanceMm, int sensorIndex); //斜めとかをsinで90度の時の角度に変換
    int isWall(); // 左右に壁があるか
    float wallFollowingOperation();

    float PID(float target_yaw);
    // タイムアウトまたは通信待ちタイムアウト時は-1を返す。
    int moveForwardOneBlock(int speed, float start_yaw,
        const rcj_maze::FloorPosition& startPosition,
        const rcj_maze::FloorPosition& goalPosition,
        rcj_maze::MazeMapper& mazeMapper); // 0:前進成功、1:前進成功＆チェックポイント、2:青、3:赤、4:黒（障害物回避などを追加予定）
    bool deployRescueKits(int q, String way);

    bool turnDegrees(float degree); // -180 ~ 180、LoPで中断したらfalse
    void avoidObstacle();
    /*
    Fusionでシミュレーションしたらこんな感じ↓
    1. 20mm後退
    2. 障害物ない側に15度超信地旋回
    3. 120mmもしくは、ない側のロードセルが反応するまで前進
    4. 正面を向く（15度旋回）
    5. 150mm前進
    6. ある側に15度旋回
    7. 110mm前進
    8. 正面を向く（15度旋回）
    9. 道中央に復帰関数(未実装)で道の中央に戻って
       ぶつかった前進関数のstartDistanceから300(mm)の倍数になるように前進（ちょっとだったら後退でもいい）
    */

    bool moveForwardMm(int speed, int range, float start_yaw);
    bool moveBackwardMm(int speed, int range, float start_yaw);

    bool climbSlope(int speed, float start_yaw);
    bool goDownSlope(int speed, float start_yaw); // 下部距離センサーの値が閾値より長かったら下り坂判定

    bool correctGyroYaw(int nowDir); // 後ろ下がって壁に機体押し付けて無理やりジャイロのyawのドリフトとかいうやつを修正する

};
} // namespace ControlRobot
