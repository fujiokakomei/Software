#include <Arduino.h>

#include "Control.hpp"
#include "Buttons.hpp"
#include "Motor.hpp"
#include "Sensors.hpp"
#include "RescueKits.hpp"
#include "LoadCell.hpp"
#include "upperVL53.hpp"
#include "Mapping.hpp"

#include <array>
#include <math.h>

namespace controlrobot
{

namespace
{

constexpr float kTileDistanceMm = 300.0f;
constexpr float kReturnToleranceMm = 20.0f;
constexpr unsigned long kReturnTimeoutMs = 3000;

// その方向に壁があり、ロボットがタイルの中央にいるとしたときの前後左右距離センサーの値
constexpr float kFrontDistanceMm = 116.0f;
constexpr float kBackDistanceMm = 107.0f;
constexpr float kRightDistanceMm = 111.7f;
constexpr float kLeftDistanceMm = 111.7f;
// ロボットがタイルの中央にいるとしてその方向に壁があるかどうか
constexpr float frontWallThreshold = 156.0f;
constexpr float backWallThreshold = 147.0f;
constexpr float rightWallThreshold = 151.7f;
constexpr float leftWallThreshold = 151.7f;
constexpr unsigned long kTurnTimeoutMs = 5000;
// 被災者識別・救助キット投下中の停止時間も含むため、通常の一マス走行より長く取る。
constexpr unsigned long kMoveTimeoutMs = 15000;
constexpr unsigned long kSlopeTimeoutMs = 10000;
constexpr unsigned long kGyroCorrectionTimeoutMs = 5000;
constexpr unsigned long kCameraResponseTimeoutMs = 3000;
constexpr int kMotionTimeoutResult = -1;
// 上部VL53PID
constexpr float kUpperVL53RadiusMm = 28.0f; //センサーが置かれてる円の半径
constexpr float kWallFollowingRightThresholdMm = 131.7f;
constexpr float kWallFollowingLeftThresholdMm = 131.7f;
constexpr float kSideWallConsistencyMm = 35.0f; //斜め前センサーだけ壁なくなって変な値読んでないか判定
constexpr float kWallFollowingKp = 0.5f;
constexpr float kMaxWallFollowingOperation = 40.0f;
constexpr int DR_SENSOR = 2;
constexpr int R_SENSOR = 3;
constexpr int DL_SENSOR = 10;
constexpr int L_SENSOR = 9;
// 左右障害物検知距離閾値&回避用定数とか
constexpr float kObstacleThresholdMm = 120.0f;
constexpr float kTurningAngle = 20.0f;
constexpr int kMovingSpeed = 50;
constexpr int kMovingMs = 1000; //ミリ秒
constexpr float kObstacleSecondTileThresholdMm = 60.0f; //これ以上進んでたら2マス前方に復帰する
constexpr float kWalledTileDistanceMm = 280.0f;
constexpr float kCenteringToleranceMm = 5.0f;

// タイル中心での壁判定値。
constexpr float kMapFrontWallThresholdMm = 136.0f;
constexpr float kMapBackWallThresholdMm = 127.0f;
constexpr float kMapRightWallThresholdMm = 131.7f;
constexpr float kMapLeftWallThresholdMm = 131.7f;

// 弧走行の角速度フィードバック用。実機で必要に応じて調整する。
constexpr float kArcRateKp = 0.35f;
constexpr float kArcRateKi = 0.03f;
constexpr float kMaxArcRateIntegral = 300.0f;
constexpr unsigned long kArcRateResetIntervalMs = 250;

float normalizeAngle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

int clampMotorSpeed(float speed)
{
    return constrain(static_cast<int>(speed), 0, 255);
}

int clampSignedMotorSpeed(float speed)
{
    return constrain(static_cast<int>(roundf(speed)), -255, 255);
}

void blinkRescueKitLed(rescuekits::RescueKitManager& kits, int count)
{
    //for (int i = 0; i < count; i++)
    //{
    for (int j = 0; j < 5; j++)
    {
        digitalWrite(kits.ledPin, HIGH);
        delay(500);
        digitalWrite(kits.ledPin, LOW);
        delay(500);
    }
    //}
}

bool isRightWay(String way)
{
    way.trim();
    way.toLowerCase();
    return way == "right" || way == "r";
}

int availableKitCount(const rescuekits::RescueKitManager& kits)
{
    return max(0, kits.remainingKits);
}

std::array<bool, 4> wallInformationAtTileCenter(const uint16_t distanceData[12])
{
    // 上部VL53は 0:前、3:右、6:後、9:左。
    return {
        distanceData[0] < kMapFrontWallThresholdMm,
        distanceData[3] < kMapRightWallThresholdMm,
        distanceData[6] < kMapBackWallThresholdMm,
        distanceData[9] < kMapLeftWallThresholdMm};
}

bool directionFromPositions(
    const rcj_maze::FloorPosition& from,
    const rcj_maze::FloorPosition& to,
    rcj_maze::Direction& direction)
{
    if (from.floor != to.floor)
    {
        return false;
    }

    const int deltaX = to.x - from.x;
    const int deltaY = to.y - from.y;
    if (deltaX == 1 && deltaY == 0)
    {
        direction = rcj_maze::EAST;
        return true;
    }
    if (deltaX == -1 && deltaY == 0)
    {
        direction = rcj_maze::WEST;
        return true;
    }
    if (deltaX == 0 && deltaY == 1)
    {
        direction = rcj_maze::NORTH;
        return true;
    }
    if (deltaX == 0 && deltaY == -1)
    {
        direction = rcj_maze::SOUTH;
        return true;
    }
    return false;
}

//===========================================================================================================================
bool stopIfStartButtonInterrupted(tb67::ControlMotors& motors)
{
    if (!buttons::consumeStartButtonInterrupt())
    {
        return false;
    }

    motors.stopRobot();
    return true;
}

bool waitWithStartButton(tb67::ControlMotors& motors, unsigned long durationMs)
{
    const unsigned long startTime = millis();
    while (millis() - startTime < durationMs)
    {
        if (stopIfStartButtonInterrupted(motors))
        {
            return true;
        }
    }
    return false;
}

enum class SerialWaitResult
{
    Received,
    Interrupted,
    TimedOut
};

SerialWaitResult waitForSerialWithStartButton(
    HardwareSerial& serial,
    tb67::ControlMotors& motors,
    unsigned long timeoutMs)
{
    const unsigned long startTime = millis();
    while (!serial.available())
    {
        if (stopIfStartButtonInterrupted(motors))
        {
            return SerialWaitResult::Interrupted;
        }
        if (millis() - startTime >= timeoutMs)
        {
            motors.stopRobot();
            return SerialWaitResult::TimedOut;
        }
    }
    return SerialWaitResult::Received;
}
//===========================================================================================================================

} // namespace

int ControlRobot::isWall()
{
    if (distance_data[R_SENSOR] < kWallFollowingRightThresholdMm &&
        distance_data[L_SENSOR] < kWallFollowingLeftThresholdMm)
    {
        if (fabsf(distance_data[DR_SENSOR] - distance_data[R_SENSOR]) < kSideWallConsistencyMm && fabsf(distance_data[DL_SENSOR] - distance_data[L_SENSOR]) < kSideWallConsistencyMm)
        {
            return 3;
        }
    }
    else if (distance_data[R_SENSOR] < kWallFollowingRightThresholdMm)
    {
        if (fabsf(distance_data[DR_SENSOR] - distance_data[R_SENSOR]) < kSideWallConsistencyMm)
        {
            return 1;
        }
    }
    else if (distance_data[L_SENSOR] < kWallFollowingLeftThresholdMm)
    {
        if (fabsf(distance_data[DL_SENSOR] - distance_data[L_SENSOR]) < kSideWallConsistencyMm)
        {
            return 2;
        }
    }
    return 0;
}

float ControlRobot::convertSensorValue(uint16_t distanceMm, int sensorIndex)
{
    const float angle = static_cast<float>(sensorIndex) * PI / 6.0f;
    return (static_cast<float>(distanceMm) + kUpperVL53RadiusMm) * fabsf(sinf(angle));
}

float ControlRobot::wallFollowingOperation()
{
    const int wall = isWall();
    float operation = 0.0f;

    if (wall == 1 || wall == 3)
    {
        // 右斜め前の補正距離が右横より大きいときは、右を向くように補正する。
        const float rightDiagonal = convertSensorValue(distance_data[DR_SENSOR], DR_SENSOR);
        const float rightSide = convertSensorValue(distance_data[R_SENSOR], R_SENSOR);
        operation += rightDiagonal - rightSide;
    }

    if (wall == 2 || wall == 3)
    {
        // 左斜め前の補正距離が左横より大きいときは、左を向くように補正する。
        const float leftDiagonal = convertSensorValue(distance_data[DL_SENSOR], DL_SENSOR);
        const float leftSide = convertSensorValue(distance_data[L_SENSOR], L_SENSOR);
        operation -= leftDiagonal - leftSide;
    }

    return constrain(
        operation * kWallFollowingKp,
        -kMaxWallFollowingOperation,
        kMaxWallFollowingOperation);
}


bool ControlRobot::turnDegrees(float degree)
{
    degree = normalizeAngle(degree);
    if (degree == 0.0f)
    {
        _motors.stopRobot();
        return true;
    }

    float yaw = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    _sensors.readOrientation(yaw, roll, pitch);

    const float start_yaw = yaw;
    const float targetTurn = fabsf(degree);
    const bool turnRight = degree > 0.0f;
    float turned = 0.0f;
    const unsigned long startTime = millis();

    while (turned < targetTurn)
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kTurnTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        if (turnRight)
        {
            _motors.turnRight();
        }
        else
        {
            _motors.turnLeft();
        }

        _sensors.readOrientation(yaw, roll, pitch);
        const float delta = normalizeAngle(yaw - start_yaw);
        turned = turnRight ? delta : -delta;
        if (turned < 0.0f)
        {
            turned += 360.0f;
        }
    }

    _motors.stopRobot();
    return true;
}

void ControlRobot::moveArc(int speed, float angularVelocity)
{
    const int baseSpeed = clampSignedMotorSpeed(speed);
    if (baseSpeed == 0 && angularVelocity == 0.0f)
    {
        _motors.stopRobot();
        _arcRateControlInitialized = false;
        _arcRateIntegral = 0.0f;
        return;
    }

    float yaw = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    _sensors.readOrientation(yaw, roll, pitch);

    const unsigned long now = millis();
    float actualAngularVelocity = 0.0f;
    float deltaTimeSeconds = 0.0f;

    if (_arcRateControlInitialized)
    {
        const unsigned long elapsedMs = now - _arcPreviousTimeMs;
        if (elapsedMs <= kArcRateResetIntervalMs)
        {
            deltaTimeSeconds = static_cast<float>(elapsedMs) / 1000.0f;
            if (deltaTimeSeconds > 0.0f)
            {
                actualAngularVelocity = normalizeAngle(yaw - _arcPreviousYaw) / deltaTimeSeconds;
            }
        }
        else
        {
            // 呼び出し間隔が空いた場合は古い角速度を使わない。
            _arcRateIntegral = 0.0f;
        }
    }
    else
    {
        _arcRateControlInitialized = true;
        _arcRateIntegral = 0.0f;
    }

    const float rateError = angularVelocity - actualAngularVelocity;
    if (deltaTimeSeconds > 0.0f)
    {
        _arcRateIntegral = constrain(
            _arcRateIntegral + rateError * deltaTimeSeconds,
            -kMaxArcRateIntegral,
            kMaxArcRateIntegral);
    }

    // 左右の速度差で角速度を作る。後退時にも旋回方向が変わらない式にする。
    const float turnOperation = kArcRateKp * rateError + kArcRateKi * _arcRateIntegral;
    const int leftSpeed = clampSignedMotorSpeed(static_cast<float>(baseSpeed) - turnOperation);
    const int rightSpeed = clampSignedMotorSpeed(static_cast<float>(baseSpeed) + turnOperation);

    _motors.moveLeftSide(leftSpeed);
    _motors.moveRightSide(rightSpeed);

    _arcPreviousYaw = yaw;
    _arcPreviousTimeMs = now;
}

float ControlRobot::PID(float target_yaw)
{
    float yaw = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    _sensors.readOrientation(yaw, roll, pitch);

    const float error = normalizeAngle(target_yaw - yaw);
    _pidIntegral += error * DELTA_T;
    _pidIntegral = constrain(_pidIntegral, -100.0f, 100.0f);

    const float derivative = (error - _pidPrevError) / DELTA_T;
    _pidPrevError = error;

    return KP * error + KI * _pidIntegral + KD * derivative;
}

bool ControlRobot::deployRescueKits(int q, String way)
{
    blinkRescueKitLed(_rightKits, q); // Unharmedでも光らせる
    bool dropped = false;

    if (q != 0) // Unharmedでない
    {
        q = constrain(q, 1, 2);

        const bool deployRightFirst = isRightWay(way);
        rescuekits::RescueKitManager& primaryKits = deployRightFirst ? _rightKits : _leftKits;
        rescuekits::RescueKitManager& secondaryKits = deployRightFirst ? _leftKits : _rightKits;

        const int primaryCount = min(q, availableKitCount(primaryKits));
        if (primaryCount > 0)
        {
            primaryKits.deployRescueKit(primaryCount, false);
            dropped = true;
        }

        const int remainingCount = q - primaryCount;
        const int secondaryCount = min(remainingCount, availableKitCount(secondaryKits));
        if (secondaryCount > 0)
        {
            secondaryKits.deployRescueKit(secondaryCount, false);
            dropped = true;
        }
    }

    //blinkRescueKitLed(primaryKits, q); 一番上に移動
    return dropped;
}

bool ControlRobot::moveForwardMm(int speed, int range, float start_yaw)
{
    _vl53s.getVL53s(distance_data);
    float startDistance = distance_data[0];
    float nowDistance = startDistance;
    const unsigned long startTime = millis();
    while(startDistance - nowDistance < range)
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kMoveTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        const float operation = PID(start_yaw);
        const int leftSpeed = clampMotorSpeed(speed - operation);
        const int rightSpeed = clampMotorSpeed(speed + operation);

        _motors.moveLeftSide(leftSpeed);
        _motors.moveRightSide(rightSpeed);


        _vl53s.getVL53s(distance_data);
        nowDistance = distance_data[0];
        recordIntermediateTileWallIfReached();
    }
    _motors.stopRobot();
    return true;
}

bool ControlRobot::moveBackwardMm(int speed, int range, float start_yaw)
{
    _vl53s.getVL53s(distance_data);
    float startDistance = distance_data[6];
    float nowDistance = startDistance;
    const unsigned long startTime = millis();
    while(startDistance - nowDistance < range)
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kMoveTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        const float operation = PID(start_yaw);
        const int leftSpeed = clampMotorSpeed(speed - operation);
        const int rightSpeed = clampMotorSpeed(speed + operation);

        _motors.moveRightSide(-leftSpeed);
        _motors.moveLeftSide(-rightSpeed);


        _vl53s.getVL53s(distance_data);
        nowDistance = distance_data[6];
    }
    _motors.stopRobot();
    return true;
}

void ControlRobot::beginIntermediateTileWallRecording(
    float targetDistanceMm,
    float targetYaw,
    const rcj_maze::FloorPosition& startPosition,
    const rcj_maze::FloorPosition& intermediatePosition,
    rcj_maze::MazeMapper& mazeMapper)
{
    _intermediateTileWallRecordingEnabled = false;
    _intermediateTileWallRecorded = false;
    _intermediateTileMapper = nullptr;

    rcj_maze::Direction direction;
    if (!directionFromPositions(startPosition, intermediatePosition, direction))
    {
        return;
    }

    _intermediateTileTargetDistanceMm = targetDistanceMm;
    _intermediateTileTargetYaw = targetYaw;
    _intermediateTileFloor = intermediatePosition.floor;
    _intermediateTileX = intermediatePosition.x;
    _intermediateTileY = intermediatePosition.y;
    _intermediateTileDirection = static_cast<int>(direction);
    _intermediateTileMapper = &mazeMapper;
    _intermediateTileWallRecordingEnabled = true;
}

void ControlRobot::recordIntermediateTileWallIfReached()
{
    if (!_intermediateTileWallRecordingEnabled || _intermediateTileWallRecorded)
    {
        return;
    }

    _vl53s.getVL53s(distance_data);
    const float yawErrorRad = normalizeAngle(_sensors.readYaw() - _intermediateTileTargetYaw)
        * PI / 180.0f;
    const float forwardDistanceMm = distance_data[0] * cosf(yawErrorRad);
    if (forwardDistanceMm > _intermediateTileTargetDistanceMm + kCenteringToleranceMm)
    {
        return;
    }

    const std::array<bool, 4> walls = wallInformationAtTileCenter(distance_data);
    _intermediateTileMapper->updateMap(
        _intermediateTileFloor,
        _intermediateTileX,
        _intermediateTileY,
        static_cast<rcj_maze::Direction>(_intermediateTileDirection),
        walls);
    _followingTileDistanceMm = walls[1] && walls[3]
        ? kWalledTileDistanceMm
        : kTileDistanceMm;
    _intermediateTileWallRecorded = true;
}

void ControlRobot::clearIntermediateTileWallRecording()
{
    _intermediateTileWallRecordingEnabled = false;
    _intermediateTileWallRecorded = false;
    _intermediateTileMapper = nullptr;
}

bool ControlRobot::moveForwardToFrontDistance(int speed, float targetDistanceMm, float targetYaw)
{
    _vl53s.getVL53s(distance_data);
    float nowDistanceMm = distance_data[0];
    const unsigned long startTime = millis();

    while (nowDistanceMm > targetDistanceMm + kCenteringToleranceMm)
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kMoveTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        const float operation = PID(targetYaw);
        const int leftSpeed = clampMotorSpeed(speed - operation);
        const int rightSpeed = clampMotorSpeed(speed + operation);

        _motors.moveLeftSide(leftSpeed);
        _motors.moveRightSide(rightSpeed);

        _vl53s.getVL53s(distance_data);
        nowDistanceMm = distance_data[0];
        recordIntermediateTileWallIfReached();
    }

    _motors.stopRobot();
    return true;
}


void ControlRobot::avoidObstacleControl(String way)
{
    bool wayRight = isRightWay(way);
    float turningAngle;
    if (!wayRight)
    {
        turningAngle = -kTurningAngle;
    }
    else
    {
        turningAngle = kTurningAngle;
    }

    turnDegrees(-turningAngle);
    unsigned long start_time = millis();
    unsigned long now_time = start_time;
    while ((now_time - start_time) <= kMovingMs)
    {
        moveArc(kMovingSpeed, turningAngle);
        recordIntermediateTileWallIfReached();
        now_time = millis();
    }
    moveForwardMm(kMovingSpeed, 100, _sensors.readYaw()); //10cm前進
    start_time = millis();
    now_time = start_time;
    while ((now_time - start_time) <= kMovingMs)
    {
        moveArc(kMovingSpeed, turningAngle);
        recordIntermediateTileWallIfReached();
        now_time = millis();
    }
    turnDegrees(-turningAngle); // ここで道の中央には戻った
    recordIntermediateTileWallIfReached();
}

int ControlRobot::avoidObstacle(float movedDistanceMm)
{
    bool isAvoided = false;
    // 距離センサー1,11
    if (distance_data[1] < kObstacleThresholdMm && distance_data[11] > kObstacleThresholdMm)
    {
        avoidObstacleControl("right");
        isAvoided = true;
    }
    else if (distance_data[1] > kObstacleThresholdMm && distance_data[11] < kObstacleThresholdMm)
    {
        avoidObstacleControl("left");
        isAvoided = true;
    }
    else if (distance_data[1] < kObstacleThresholdMm && distance_data[11] < kObstacleThresholdMm)
    {
        // 0番VL53故障確定　どーにかせい
    }
    // 距離センサーで回避できた場合は、ロードセルによる回避を重ねて実行しない。
    if (!isAvoided)
    {
        // ロードセルの方。ここを使うのは、距離センサー故障または例外的な置かれ方を想定する。
        const int obstacles = _loadCells.readLoadCell();
        switch (obstacles)
        {
        case 0: // なし
            break;
        case 1: // 右
            moveBackwardMm(kMovingSpeed, 300, _sensors.readYaw());
            avoidObstacleControl("right");
            isAvoided = true;
            break;
        case 2: // 左
            moveBackwardMm(kMovingSpeed, 300, _sensors.readYaw());
            avoidObstacleControl("left");
            isAvoided = true;
            break;
        case 3: // 距離センサーが故障してるので頑張りましょう！！！！！！！！！（泣）
            //実装予定
            break;
        default:
            break;
        }
    }

    if (isAvoided) // 中央には戻ってるからあとは前進か後退で適切なタイルの中央にいく
    {
        // ちょうど60 mmは「6 cm以上」として次の次のタイルを選ぶ。
        return movedDistanceMm >= kObstacleSecondTileThresholdMm ? 2 : 1;
    }

    return 0;
}

int ControlRobot::moveForwardOneBlock(
    int speed,
    float start_yaw,
    const rcj_maze::FloorPosition& startPosition,
    const rcj_maze::FloorPosition& goalPosition,
    rcj_maze::MazeMapper& mazeMapper)
{
    _vl53s.getVL53s(distance_data);
    float startDistance = distance_data[0];
    float goalDistance = startDistance - kTileDistanceMm;
    float nowDistance = startDistance;
    bool handledBlueTile = false;
    bool detectedBlueTile = false;
    bool detectedCheckpoint = false;
    const unsigned long motionStartTime = millis();
    _rightCam.print("mode a\n");
    _leftCam.print("mode a\n");
    while(nowDistance > goalDistance)
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return 0;
        }
        if (millis() - motionStartTime >= kMoveTimeoutMs)
        {
            _motors.stopRobot();
            return kMotionTimeoutResult;
        }

        const float operation = PID(start_yaw) + wallFollowingOperation();
        const int leftSpeed = clampMotorSpeed(speed - operation);
        const int rightSpeed = clampMotorSpeed(speed + operation);

        _motors.moveLeftSide(leftSpeed);
        _motors.moveRightSide(rightSpeed);


        _vl53s.getVL53s(distance_data);
        nowDistance = distance_data[0];


        //カラータイル処理
        const sensors::ColorTile colorTile = sensors::detectColorTile(_sensors);
        if (colorTile == sensors::ColorTile::Blue && !handledBlueTile)
        {
            /* 前進終わってから5秒待つように変更(main.cppで待つ処理する)
            _motors.stopRobot();
            if (waitWithStartButton(_motors, 5000))
            {
                return 0;
            }
            */
            detectedBlueTile = true;
            handledBlueTile = true;
        }
        else if (colorTile == sensors::ColorTile::Silver)
        {
            detectedCheckpoint = true;
        }
        else if (colorTile == sensors::ColorTile::Black || colorTile == sensors::ColorTile::Red)
        {
            const int colorResult = colorTile == sensors::ColorTile::Red ? 3 : 4;
            const unsigned long startTime = millis();
            _motors.moveBackward(speed);
            while (nowDistance < startDistance - kReturnToleranceMm
                && millis() - startTime < kReturnTimeoutMs)
            {
                if (stopIfStartButtonInterrupted(_motors))
                {
                    return 0;
                }

                _vl53s.getVL53s(distance_data);
                nowDistance = distance_data[0];
            }
            _motors.stopRobot();
            return colorResult;
        }


        //カメラ処理
        if (_rightCam.available())
        {
            _vl53s.getVL53s(distance_data);
            if (distance_data[3] < rightWallThreshold) //右に壁があるなら
            {
                String line = _rightCam.readStringUntil('\n');
                if (line == "found")
                {
                    _motors.brakeRobot();
                    _rightCam.print("mode b\n");
                    const SerialWaitResult waitResult = waitForSerialWithStartButton(
                        _rightCam, _motors, kCameraResponseTimeoutMs);
                    if (waitResult == SerialWaitResult::Interrupted)
                    {
                        return 0;
                    }
                    if (waitResult == SerialWaitResult::TimedOut)
                    {
                        return kMotionTimeoutResult;
                    }
                    String value = _rightCam.readStringUntil('\n');
                    int value_i = value.toInt();
                    if (value_i == -1){
                    } else // 違ったじゃないなら
                    {
                        // 前進距離の前半なら開始Cell、後半ならゴールCellにいるものとして扱う。
                        const float movedDistanceMm = max(0.0f, startDistance - nowDistance);
                        const rcj_maze::FloorPosition& dropPosition =
                            movedDistanceMm < kTileDistanceMm / 2.0f ? startPosition : goalPosition;
                        rcj_maze::Cell& dropCell = mazeMapper.cell(
                            dropPosition.floor, dropPosition.x, dropPosition.y);
                        if (!dropCell.isDropped && deployRescueKits(value_i, "right"))
                        {
                            dropCell.isDropped = true;
                        }
                    }
                    _rightCam.print("mode c\n");
                }
            }
        }
        if (_leftCam.available())
        {
            _vl53s.getVL53s(distance_data);
            if (distance_data[9] < leftWallThreshold) //左に壁があるなら
            {
                String line = _leftCam.readStringUntil('\n');
                if (line == "found")
                {
                    _motors.brakeRobot();
                    _leftCam.print("mode b\n");
                    const SerialWaitResult waitResult = waitForSerialWithStartButton(
                        _leftCam, _motors, kCameraResponseTimeoutMs);
                    if (waitResult == SerialWaitResult::Interrupted)
                    {
                        return 0;
                    }
                    if (waitResult == SerialWaitResult::TimedOut)
                    {
                        return kMotionTimeoutResult;
                    }
                    String value = _leftCam.readStringUntil('\n');
                    int value_i = value.toInt();
                    if (value_i == -1){
                    } else // 違ったじゃないなら
                    {
                        // 前進距離の前半なら開始Cell、後半ならゴールCellにいるものとして扱う。
                        const float movedDistanceMm = max(0.0f, startDistance - nowDistance);
                        const rcj_maze::FloorPosition& dropPosition =
                            movedDistanceMm < kTileDistanceMm / 2.0f ? startPosition : goalPosition;
                        rcj_maze::Cell& dropCell = mazeMapper.cell(
                            dropPosition.floor, dropPosition.x, dropPosition.y);
                        if (!dropCell.isDropped && deployRescueKits(value_i, "left"))
                        {
                            dropCell.isDropped = true;
                        }
                    }
                    _leftCam.print("mode c\n");
                }
            }
        }

        // ロードセル、距離センサー1,11
        // 回避開始前の壁情報を使い、最初のタイル間隔を決める。
        const std::array<bool, 4> wallsBeforeAvoid = wallInformationAtTileCenter(distance_data);
        const float movedDistanceMm = max(0.0f, startDistance - nowDistance);
        const float firstTileDistanceMm = wallsBeforeAvoid[1] && wallsBeforeAvoid[3]
            ? kWalledTileDistanceMm
            : kTileDistanceMm;
        const float intermediateTargetDistanceMm = startDistance - firstTileDistanceMm;

        // 回避中に1つ前方のタイル中心を通過した瞬間の壁情報を記録する。
        beginIntermediateTileWallRecording(
            intermediateTargetDistanceMm, start_yaw, startPosition, goalPosition, mazeMapper);
        const int obstacleTargetTiles = avoidObstacle(movedDistanceMm);
        if (obstacleTargetTiles == 0)
        {
            clearIntermediateTileWallRecording();
            continue;
        }

        // 回避後には1つ前方の中心を通過済みの場合がある。機体を後退させず、
        // まだ前方にある場合だけその中心まで進む。
        _vl53s.getVL53s(distance_data);
        nowDistance = distance_data[0];
        recordIntermediateTileWallIfReached();

        if (obstacleTargetTiles == 1 &&
            nowDistance > intermediateTargetDistanceMm + kCenteringToleranceMm)
        {
            if (!moveForwardToFrontDistance(kMovingSpeed, intermediateTargetDistanceMm, start_yaw))
            {
                clearIntermediateTileWallRecording();
                return kMotionTimeoutResult;
            }
            clearIntermediateTileWallRecording();
            return kMoveResultObstacleNextTile;
        }

        if (obstacleTargetTiles == 2 &&
            nowDistance > intermediateTargetDistanceMm + kCenteringToleranceMm)
        {
            // 2タイル進む場合も、まだ中間中心より手前なら通過時に壁を記録する。
            if (!moveForwardToFrontDistance(kMovingSpeed, intermediateTargetDistanceMm, start_yaw))
            {
                clearIntermediateTileWallRecording();
                return kMotionTimeoutResult;
            }
            _vl53s.getVL53s(distance_data);
            nowDistance = distance_data[0];
            recordIntermediateTileWallIfReached();
        }

        // 1タイルを指定していても既にその中心を超えた場合は、後退せず次の中心へ進む。
        // 中間タイルの壁情報を回避中に取得できなかった場合は現在値をフォールバックに使う。
        if (!_intermediateTileWallRecorded)
        {
            _followingTileDistanceMm = isWall() == 3
                ? kWalledTileDistanceMm
                : kTileDistanceMm;
        }
        const float finalTargetDistanceMm = intermediateTargetDistanceMm - _followingTileDistanceMm;
        if (!moveForwardToFrontDistance(kMovingSpeed, finalTargetDistanceMm, start_yaw))
        {
            clearIntermediateTileWallRecording();
            return kMotionTimeoutResult;
        }

        clearIntermediateTileWallRecording();
        return kMoveResultObstacleSecondNextTile;
    }
    _motors.stopRobot();
    if (detectedCheckpoint)
    {
        return 1;
    }
    if (detectedBlueTile)
    {
        return 2;
    }
    return 0;
}

bool ControlRobot::climbSlope(int speed, float start_yaw)
{
    float yaw = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    _sensors.readOrientation(yaw, roll, pitch);
    const unsigned long startTime = millis();
    while (pitch < 10) // 登り始めるまで
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kSlopeTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        const float operation = PID(start_yaw);
        const int leftSpeed = clampMotorSpeed(speed - operation);
        const int rightSpeed = clampMotorSpeed(speed + operation);

        _motors.moveLeftSide(leftSpeed);
        _motors.moveRightSide(rightSpeed);

        _sensors.readOrientation(yaw, roll, pitch);
    }
    while (pitch > 3) // 登り終わるまで
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kSlopeTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        const float operation = PID(start_yaw);
        const int leftSpeed = clampMotorSpeed(speed - operation);
        const int rightSpeed = clampMotorSpeed(speed + operation);

        _motors.moveLeftSide(leftSpeed);
        _motors.moveRightSide(rightSpeed);

        _sensors.readOrientation(yaw, roll, pitch);
    }
    // タイルの真ん中まで前進
    _vl53s.getVL53s(distance_data);
    float frontDistance = distance_data[0];
    while (std::fmod(frontDistance, kTileDistanceMm) > kFrontDistanceMm)
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kSlopeTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        const float operation = PID(start_yaw);
        const int leftSpeed = clampMotorSpeed(speed - operation);
        const int rightSpeed = clampMotorSpeed(speed + operation);

        _motors.moveLeftSide(leftSpeed);
        _motors.moveRightSide(rightSpeed);

        _vl53s.getVL53s(distance_data);
        frontDistance = distance_data[0];
    }
    _motors.stopRobot();
    return true;
}

bool ControlRobot::goDownSlope(int speed, float start_yaw)
{
    float yaw = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    _sensors.readOrientation(yaw, roll, pitch);
    const unsigned long startTime = millis();
    while (pitch > -10) // 降り始めるまで
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kSlopeTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        const float operation = PID(start_yaw);
        const int leftSpeed = clampMotorSpeed(speed - operation);
        const int rightSpeed = clampMotorSpeed(speed + operation);

        _motors.moveLeftSide(leftSpeed);
        _motors.moveRightSide(rightSpeed);

        _sensors.readOrientation(yaw, roll, pitch);
    }
    while (pitch < -3) // 降り終わるまで
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kSlopeTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        const float operation = PID(start_yaw);
        const int leftSpeed = clampMotorSpeed(speed - operation);
        const int rightSpeed = clampMotorSpeed(speed + operation);

        _motors.moveLeftSide(leftSpeed);
        _motors.moveRightSide(rightSpeed);

        _sensors.readOrientation(yaw, roll, pitch);
    }
    // タイルの真ん中まで前進
    _vl53s.getVL53s(distance_data);
    float frontDistance = distance_data[0];
    while (std::fmod(frontDistance, kTileDistanceMm) > kFrontDistanceMm)
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kSlopeTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        const float operation = PID(start_yaw);
        const int leftSpeed = clampMotorSpeed(speed - operation);
        const int rightSpeed = clampMotorSpeed(speed + operation);

        _motors.moveLeftSide(leftSpeed);
        _motors.moveRightSide(rightSpeed);

        _vl53s.getVL53s(distance_data);
        frontDistance = distance_data[0];
    }
    _motors.stopRobot();
    return true;
}

bool ControlRobot::correctGyroYaw(int nowDir)
{
    _motors.moveBackward(100);
    if (waitWithStartButton(_motors, 5000))
    {
        return false;
    }
    _motors.stopRobot();
    if (!_sensors.calibrateGyroOffset(nowDir))
    {
        return false;
    }


    float yaw = 0.0f;
    float roll = 0.0f;
    float pitch = 0.0f;
    _sensors.readOrientation(yaw, roll, pitch);

    _vl53s.getVL53s(distance_data);
    float backDistance = distance_data[6];
    const unsigned long startTime = millis();
    while (backDistance < kBackDistanceMm)
    {
        if (stopIfStartButtonInterrupted(_motors))
        {
            return false;
        }
        if (millis() - startTime >= kGyroCorrectionTimeoutMs)
        {
            _motors.stopRobot();
            return false;
        }

        const float operation = PID(yaw);
        const int leftSpeed = clampMotorSpeed(100 - operation);
        const int rightSpeed = clampMotorSpeed(100 + operation);

        _motors.moveLeftSide(leftSpeed);
        _motors.moveRightSide(rightSpeed);

        _vl53s.getVL53s(distance_data);
        backDistance = distance_data[6];
    }
    _motors.stopRobot();
    return true;
}

} // namespace ControlRobot
