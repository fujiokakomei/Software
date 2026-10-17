#include <Arduino.h>

#include "Control.hpp"
#include "Buttons.hpp"
#include "Motor.hpp"
#include "Sensors.hpp"
#include "RescueKits.hpp"
#include "LoadCell.hpp"
#include "upperVL53.hpp"
#include "Mapping.hpp"

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
// 上部VL53は機体中心から半径28 mmの円周上に、30度間隔で配置されている。
constexpr float kUpperVl53RadiusMm = 28.0f;
constexpr float kCorridorWidthMm = 280.0f;
constexpr float kTargetSideWallDistanceMm = kCorridorWidthMm / 2.0f;
constexpr float kWallFollowingKp = 0.35f;
constexpr float kMaxWallFollowingOperation = 45.0f;
// 斜めのVL53が、真横のVL53と同じ横壁を見ていると判断する許容差。
constexpr float kSideWallConsistencyMm = 35.0f;
constexpr unsigned long kTurnTimeoutMs = 5000;
// 被災者識別・救助キット投下中の停止時間も含むため、通常の一マス走行より長く取る。
constexpr unsigned long kMoveTimeoutMs = 15000;
constexpr unsigned long kSlopeTimeoutMs = 10000;
constexpr unsigned long kGyroCorrectionTimeoutMs = 5000;
constexpr unsigned long kCameraResponseTimeoutMs = 3000;
constexpr int kMotionTimeoutResult = -1;

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

bool isValidSideWallReading(uint16_t distance)
{
    // 0は通信開始前の初期値として使われるため、壁の測距値として扱わない。
    return distance > 0;
}

float sideWallDistanceFromCenter(uint16_t rangeMm, uint8_t sensorIndex)
{
    // センサーiは前方から時計回りにi * 30度。測距値を機体中心から壁までの
    // 横方向の距離へ投影する。センサー自体が中心から28 mm離れている分も加算する。
    const float angle = static_cast<float>(sensorIndex) * PI / 6.0f;
    return (static_cast<float>(rangeMm) + kUpperVl53RadiusMm) * fabsf(sinf(angle));
}

float averageSideWallDistance(
    const uint16_t distanceData[12],
    uint8_t directSideSensor,
    const uint8_t* angledSideSensors,
    size_t angledSensorCount)
{
    const float directDistance = sideWallDistanceFromCenter(
        distanceData[directSideSensor], directSideSensor);
    float totalDistance = directDistance;
    size_t sampleCount = 1;

    for (size_t i = 0; i < angledSensorCount; ++i)
    {
        const uint8_t sensorIndex = angledSideSensors[i];
        const uint16_t rangeMm = distanceData[sensorIndex];
        if (!isValidSideWallReading(rangeMm))
        {
            continue;
        }

        const float sideDistance = sideWallDistanceFromCenter(rangeMm, sensorIndex);
        // 通路端の前後壁など、別の壁を見ている斜めセンサーは除外する。
        if (fabsf(sideDistance - directDistance) <= kSideWallConsistencyMm)
        {
            totalDistance += sideDistance;
            ++sampleCount;
        }
    }

    return totalDistance / static_cast<float>(sampleCount);
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

float ControlRobot::wallFollowingOperation() const
{
    constexpr uint8_t kRightAngledSideSensors[] = {1, 2};
    // 4, 5番は右後方のセンサーなので、壁追従には使用しない。
    constexpr uint8_t kLeftAngledSideSensors[] = {7, 8, 10, 11};

    const bool hasRightWall = isValidSideWallReading(distance_data[3])
        && distance_data[3] < rightWallThreshold;
    const bool hasLeftWall = isValidSideWallReading(distance_data[9])
        && distance_data[9] < leftWallThreshold;

    if (!hasRightWall && !hasLeftWall)
    {
        return 0.0f;
    }

    const float rightDistance = hasRightWall
        ? averageSideWallDistance(
            distance_data, 3, kRightAngledSideSensors,
            sizeof(kRightAngledSideSensors) / sizeof(kRightAngledSideSensors[0]))
        : 0.0f;
    const float leftDistance = hasLeftWall
        ? averageSideWallDistance(
            distance_data, 9, kLeftAngledSideSensors,
            sizeof(kLeftAngledSideSensors) / sizeof(kLeftAngledSideSensors[0]))
        : 0.0f;

    float distanceError = 0.0f;
    if (hasRightWall && hasLeftWall)
    {
        // 右へ寄ると右壁との距離は短く、左壁との距離は長くなる。
        distanceError = (leftDistance - rightDistance) / 2.0f;
    }
    else if (hasRightWall)
    {
        distanceError = kTargetSideWallDistanceMm - rightDistance;
    }
    else
    {
        distanceError = leftDistance - kTargetSideWallDistanceMm;
    }

    // 正の補正量は右車輪を速くして左へ向ける、既存のジャイロPIDと同じ符号。
    return constrain(
        kWallFollowingKp * distanceError,
        -kMaxWallFollowingOperation,
        kMaxWallFollowingOperation);
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

void ControlRobot::avoidObstacle()
{
    int obstacles = ControlRobot::_loadCells.readLoadCell();
    switch (obstacles)
    {
    case 0: // なし
        break;
    case 1: // 右
        //実装予定
        break;
    case 2: // 左
        //実装予定
        break;
    case 3: // 距離センサーが故障してるので頑張りましょう！！！！！！！！！（泣）
        //実装予定
        break;
    default:
        break;
    }
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

        //ロードセル
        ControlRobot::avoidObstacle();
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
