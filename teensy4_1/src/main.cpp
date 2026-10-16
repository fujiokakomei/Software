// RCJ rescue maze 2026
#include <Arduino.h>
#include <array>
#include <vector>

#include "Buttons.hpp"
#include "Control.hpp"
#include "LoadCell.hpp"
#include "Mapping.hpp"
#include "Motor.hpp"
#include "RescueKits.hpp"
#include "Sensors.hpp"
#include "upperVL53.hpp"
#include "Walls.hpp"
#include "SearchApp.hpp"



rcj_maze::MazeMapper mazeMapper;
rcj_maze::RobotPose robotPose;
rcj_maze::RobotPose lastCheckpointPose;


int startButtonPin = 37;
int spare1ButtonPin = 36;
int spare2ButtonPin = 33;
buttons::ButtonManager startButton; // スタート、走行中のLoP時に一時停止、リスタート
buttons::ButtonManager spare1Button; // 予備1
buttons::ButtonManager spare2Button; // 予備2


int rightLoadCellPin = 24;
int leftLoadCellPin = 25;
loadcells::LoadCellManager loadCellManager;


int m1_1 = 0;
int m1_2 = 1; //    M4------M1
int m2_1 = 2; //    |       |
int m2_2 = 3; //    |       |
int m3_1 = 4; //    |       |
int m3_2 = 5; //    M3------M2
int m4_1 = 6;
int m4_2 = 7;
tb67::ControlMotors controlMotors;


int rightKitPin = 23;
int leftKitPin = 22;
int ledPin = 29;
int rescueKitQuantity = 4;
rescuekits::RescueKitManager rightKit;
rescuekits::RescueKitManager leftKit;


int ledPinTCS = 40;
sensors::SensorManager sensorManager;


uppervl53s::UpperVL53s upperVL53s(Serial8);


walls::Walls checkWalls(upperVL53s, sensorManager);


controlrobot::ControlRobot controlRobot(
    Serial3, Serial5, // UnitV x2
    controlMotors, 
    sensorManager, 
    rightKit, leftKit, // RescueKit x2
    loadCellManager, 
    upperVL53s
);


// 後から探索アルゴリズム変えやすい(?)ように一応インスタンス化したものを使う
searchapp::searchApp _searchApp;


//===========================================================================================================================


bool isFirstBoot = true; // 探索開始時にfalseにする

enum class RobotState
{
    WaitingStart, // 電源入れたとき
    Searching, // 探索中
    Paused, // LoPにより一時停止中
    Returning, // 探索終了したので帰還中
    Finished // 終わった
};

RobotState robotState;
RobotState resumeStateAfterPause = RobotState::Searching;

searchapp::SearchResult searchResult;

std::array<bool, 4> nowWall;
int frontType;

int moveResult;

int defaultSpeed = 200; // デフォルト前進速度

std::vector<searchapp::SearchMove> returnWay; // 帰り道
rcj_maze::FloorPath returnCoordinate; // 帰り道座標


//===========================================================================================================================

rcj_maze::Direction directionFromDelta(int dx, int dy)
{
    if (dx > 0) return rcj_maze::EAST;
    if (dx < 0) return rcj_maze::WEST;
    if (dy < 0) return rcj_maze::SOUTH;
    return rcj_maze::NORTH;
}

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

void pauseForLoP(RobotState resumeState)
{
    controlMotors.stopRobot();
    resumeStateAfterPause = resumeState;
    robotState = RobotState::Paused;
}

bool pauseIfStartButtonConsumedDuring(RobotState resumeState)
{
    if (!buttons::takeConsumedStartButtonInterrupt())
    {
        return false;
    }

    pauseForLoP(resumeState);
    return true;
}

bool pauseAfterMotionIfInterrupted(bool completed, RobotState resumeState)
{
    if (pauseIfStartButtonConsumedDuring(resumeState))
    {
        return true;
    }

    if (!completed)
    {
        pauseForLoP(resumeState);
        return true;
    }

    return false;
}

bool turnOrPause(float degrees, RobotState resumeState)
{
    return pauseAfterMotionIfInterrupted(controlRobot.turnDegrees(degrees), resumeState);
}

bool turnToDirectionOrPause(rcj_maze::Direction targetDirection, RobotState resumeState)
{
    const int relative = (static_cast<int>(targetDirection) - static_cast<int>(robotPose.dir) + 4) % 4;
    float degrees = 0.0f;

    switch (relative)
    {
    case 1:
        degrees = 90.0f;
        break;
    case 2:
        degrees = 180.0f;
        break;
    case 3:
        degrees = -90.0f;
        break;
    default:
        break;
    }

    if (degrees != 0.0f && turnOrPause(degrees, resumeState))
    {
        return true;
    }

    robotPose.dir = targetDirection;
    return false;
}

// 坂の手前（または上端）から、坂を通過した先の通常セルまで移動する。
// 呼び出し前に robotPose.dir を、通る坂の方向へ合わせておく。
bool traverseSlope(RobotState resumeState, bool updateMapAfterTraversal)
{
    const int startFloor = robotPose.floor;
    bool completed = false;

    if (startFloor == 0)
    {
        completed = controlRobot.climbSlope(defaultSpeed, sensorManager.readYaw());
    }
    else if (startFloor == 1)
    {
        completed = controlRobot.goDownSlope(defaultSpeed, sensorManager.readYaw());
    }
    else
    {
        pauseForLoP(resumeState);
        return false;
    }

    if (pauseAfterMotionIfInterrupted(completed, resumeState))
    {
        return false;
    }

    // 坂セルを経由して反対側フロアの通常セルへ移動する。
    mazeMapper.moveForward(robotPose.x, robotPose.y, robotPose.dir);
    if (updateMapAfterTraversal)
    {
        nowWall = {false, true, false, true};
        mazeMapper.updateMap(robotPose.floor, robotPose.x, robotPose.y, robotPose.dir, nowWall);
    }

    mazeMapper.moveForward(robotPose.x, robotPose.y, robotPose.dir);
    robotPose.floor = startFloor == 0 ? 1 : 0;

    if (updateMapAfterTraversal)
    {
        checkWalls.readWall(nowWall);
        mazeMapper.updateMap(robotPose.floor, robotPose.x, robotPose.y, robotPose.dir, nowWall);
    }

    return true;
}

bool moveForward()
{
    const rcj_maze::FloorPosition startPosition(
        robotPose.floor, robotPose.x, robotPose.y);
    const rcj_maze::FloorPosition goalPosition(
        searchResult.nextFloor, searchResult.nextX, searchResult.nextY);
    moveResult = controlRobot.moveForwardOneBlock(
        defaultSpeed, sensorManager.readYaw(), startPosition, goalPosition, mazeMapper);
    if (moveResult < 0)
    {
        pauseForLoP(robotState);
        return false;
    }
    if (pauseIfStartButtonConsumedDuring(robotState))
    {
        return false;
    }

    switch (moveResult)
    {
    case 0: // 何事もなく成功
        robotPose.floor = searchResult.nextFloor;
        robotPose.x = searchResult.nextX;
        robotPose.y = searchResult.nextY;
        robotPose.dir = searchResult.absoluteDirection;
        break;

    case 1: // チェックポイント＆成功
        robotPose.floor = searchResult.nextFloor;
        robotPose.x = searchResult.nextX;
        robotPose.y = searchResult.nextY;
        robotPose.dir = searchResult.absoluteDirection;
        mazeMapper.markCheckpoint(robotPose.floor, robotPose.x, robotPose.y);

        lastCheckpointPose.x = robotPose.x;
        lastCheckpointPose.y = robotPose.y;
        lastCheckpointPose.floor = robotPose.floor;
        lastCheckpointPose.dir = robotPose.dir; // 実際には使わんけど一応
        break;

    case 2: // 青タイル
        robotPose.floor = searchResult.nextFloor;
        robotPose.x = searchResult.nextX;
        robotPose.y = searchResult.nextY;
        robotPose.dir = searchResult.absoluteDirection;
        mazeMapper.markBlue(robotPose.floor, robotPose.x, robotPose.y);
        controlMotors.brakeRobot(); //完全に停止
        if (waitWithStartButton(controlMotors, 5000)) // 5秒停止
        {
            break;
        }
        break;

    case 3: // 赤タイル
        mazeMapper.markRed(robotPose.floor, searchResult.nextX, searchResult.nextY);
        break;

    case 4: // 黒タイル
        mazeMapper.markBlack(robotPose.floor, searchResult.nextX, searchResult.nextY);
        break;

    default:
        break;
    }

    if (pauseIfStartButtonConsumedDuring(robotState))
    {
        return false;
    }

    return true;
}

bool returnFromLoP()
{
    robotPose.x = lastCheckpointPose.x;
    robotPose.y = lastCheckpointPose.y;
    robotPose.floor = lastCheckpointPose.floor;

    float yaw = sensorManager.readYaw();
    if (315 <= yaw || 45 > yaw)
    {
        if (45 > yaw)
        {
            if (!controlRobot.turnDegrees(-yaw))
            {
                return false;
            }
        }
        else
        {
            if (!controlRobot.turnDegrees(360 - yaw))
            {
                return false;
            }
        }
        robotPose.dir = rcj_maze::NORTH;
    }
    else if (45 <= yaw && 135 > yaw)
    {
        if (!controlRobot.turnDegrees(90 - yaw))
        {
            return false;
        }
        robotPose.dir = rcj_maze::EAST;
    }
    else if (135 <= yaw && 225 > yaw)
    {
        if (!controlRobot.turnDegrees(180 - yaw))
        {
            return false;
        }
        robotPose.dir = rcj_maze::SOUTH;
    }
    else if (225 <= yaw && 315 > yaw)
    {
        if (!controlRobot.turnDegrees(270 - yaw))
        {
            return false;
        }
        robotPose.dir = rcj_maze::WEST;
    }

    return true;
}

//===========================================================================================================================


void setup()
{
    Serial8.begin(115200); // rp2040
    Serial3.begin(115200); // UnitV 1
    Serial5.begin(115200); // UnitV 2
    Serial3.setTimeout(1000); // readStringUntil()の待機上限
    Serial5.setTimeout(1000);


    startButton.setupButton(startButtonPin);
    spare1Button.setupButton(spare1ButtonPin);
    spare2Button.setupButton(spare2ButtonPin);
    attachInterrupt(digitalPinToInterrupt(startButtonPin), buttons::requestStartButtonInterrupt, FALLING);


    loadCellManager.setupLoadCells(rightLoadCellPin, leftLoadCellPin);


    controlMotors.setupMotors(
        m1_1, m1_2, 
        m2_1, m2_2, 
        m3_1, m3_2, 
        m4_1, m4_2
    );


    rightKit.setupRescueKits(rightKitPin, ledPin, rescueKitQuantity);
    leftKit.setupRescueKits(leftKitPin, ledPin, rescueKitQuantity);


    robotState = RobotState::WaitingStart;
}


void loop()
{
    if (buttons::consumeStartButtonInterrupt())
    {
        if (isFirstBoot) // 起動後最初のスタートのみ
        {
            // センサーキャリブレーションなど
            if (!sensorManager.setupSensors(ledPinTCS))
            {
                controlMotors.stopRobot();
                robotState = RobotState::WaitingStart;
                buttons::takeConsumedStartButtonInterrupt();
                return;
            }
            delay(500); // 一応


            // mapper準備
            robotPose.floor = 0;
            robotPose.x = 0;
            robotPose.y = 0;
            robotPose.dir = rcj_maze::NORTH;

            lastCheckpointPose.floor = robotPose.floor;
            lastCheckpointPose.x = robotPose.x;
            lastCheckpointPose.y = robotPose.y;
            lastCheckpointPose.dir = robotPose.dir;

            checkWalls.readWall(nowWall);
            if (!nowWall[0]) // 前が壁でない
            {
                frontType = checkWalls.readFront();
            }

            mazeMapper.updateMap(robotPose.floor, robotPose.x, robotPose.y, robotPose.dir, nowWall); // (0, 0)を作成

            isFirstBoot = false;
            robotState = RobotState::Searching;
            buttons::takeConsumedStartButtonInterrupt();
        }
        else
        {
            if (robotState == RobotState::Searching)
            {
                pauseForLoP(RobotState::Searching);
            }
            else if (robotState == RobotState::Returning)
            {
                pauseForLoP(RobotState::Returning);
            }
            else if (robotState == RobotState::Paused)
            {
                delay(500); // 一応
                if (returnFromLoP())
                {
                    robotState = resumeStateAfterPause;
                }
                else
                {
                    pauseForLoP(resumeStateAfterPause);
                }
            }

            buttons::takeConsumedStartButtonInterrupt();
        }
    }

    switch (robotState)
    {
    case RobotState::Searching:
        checkWalls.readWall(nowWall);
        if (!nowWall[0]) // 前が壁でない
        {
            frontType = checkWalls.readFront(); // 上り坂か下り坂かはrobotPose.floorで判断する
        }

        // 次行くとこ決める、今の壁情報はここでupdateMap()される
        searchResult = _searchApp.decideNextMove(mazeMapper, robotPose, nowWall, frontType);
        
        if (!searchResult.finished)
        {
            if (searchResult.nextIsSlope)
            {
                if (turnToDirectionOrPause(searchResult.absoluteDirection, RobotState::Searching))
                {
                    break;
                }
                traverseSlope(RobotState::Searching, true);
            }
            else if (searchResult.move == searchapp::SearchMove::Forward) // 前進時のみ前方が何か見る
            {
                moveForward();
            }
            else if (searchResult.move == searchapp::SearchMove::Right)
            {
                if (turnOrPause(90, RobotState::Searching))
                {
                    break;
                }
                moveForward();
            }
            else if (searchResult.move == searchapp::SearchMove::Left)
            {
                if (turnOrPause(-90, RobotState::Searching))
                {
                    break;
                }
                moveForward();
            }
            else if (searchResult.move == searchapp::SearchMove::Back)
            {
                if (turnOrPause(180, RobotState::Searching))
                {
                    break;
                }
                moveForward();
            }
        }
        else
        {
            robotState = RobotState::Returning;
        }

        break;
    
    case RobotState::Paused: // 一時停止中
        break;

    case RobotState::Returning: // decideNextMoveでFinishedになったら座標(0, 0)に帰る
    {
        returnCoordinate = _searchApp.decideReturnWay(mazeMapper, robotPose, returnWay);

        if (returnCoordinate.size() < 2)
        {
            // すでに初期位置にいる or 経路なし
            robotState = RobotState::Finished;
            break;
        }

        bool returnInterrupted = false;
        size_t i = 0;
        while (i + 1 < returnCoordinate.size() && !returnInterrupted)
        {
            const rcj_maze::FloorPosition &from = returnCoordinate[i];
            const rcj_maze::FloorPosition &next = returnCoordinate[i + 1];
            const rcj_maze::Cell *currentCell = mazeMapper.findCell(from.floor, from.x, from.y);
            const rcj_maze::Cell *nextCell = mazeMapper.findCell(next.floor, next.x, next.y);

            const bool startsFloorLink = currentCell != nullptr && currentCell->hasFloorLink &&
                currentCell->linkedFloor == next.floor &&
                currentCell->linkedX == next.x &&
                currentCell->linkedY == next.y;
            const bool entersSlope = nextCell != nullptr && nextCell->slope && nextCell->hasFloorLink &&
                directionFromDelta(next.x - from.x, next.y - from.y) == nextCell->floorLinkDirection;

            if (startsFloorLink || entersSlope)
            {
                // 上端から下る場合は現在セル、手前から上る場合は次の坂セルに
                // 保存された向きを使う。坂の途中では向きは変わらない。
                const rcj_maze::Direction slopeDirection = startsFloorLink
                    ? currentCell->floorLinkDirection
                    : nextCell->floorLinkDirection;
                if (turnToDirectionOrPause(slopeDirection, RobotState::Returning))
                {
                    returnInterrupted = true;
                    break;
                }
                if (!traverseSlope(RobotState::Returning, false))
                {
                    returnInterrupted = true;
                    break;
                }

                // 経路上の「坂セル」と「フロアリンク」を一度の物理移動で通過した。
                // 予定座標と合わなければ、誤った坂へ進み続けないよう停止する。
                if (i + 2 >= returnCoordinate.size() ||
                    robotPose.floor != returnCoordinate[i + 2].floor ||
                    robotPose.x != returnCoordinate[i + 2].x ||
                    robotPose.y != returnCoordinate[i + 2].y)
                {
                    controlMotors.stopRobot();
                    pauseForLoP(RobotState::Returning);
                    returnInterrupted = true;
                    break;
                }

                i += 2;
                continue;
            }

            // フロアをまたぐ辺は必ず有効なフロアリンクでなければならない。
            if (from.floor != next.floor)
            {
                controlMotors.stopRobot();
                pauseForLoP(RobotState::Returning);
                returnInterrupted = true;
                break;
            }

            const rcj_maze::Direction targetDirection = directionFromDelta(
                next.x - from.x, next.y - from.y);
            if (turnToDirectionOrPause(targetDirection, RobotState::Returning))
            {
                returnInterrupted = true;
                break;
            }

            // 通常の1マス移動
            searchResult.nextFloor = next.floor;
            searchResult.nextX = next.x;
            searchResult.nextY = next.y;
            searchResult.nextIsSlope = false;
            searchResult.absoluteDirection = targetDirection;
            returnInterrupted = !moveForward();
            ++i;
        }

        if (returnInterrupted)
        {
            break;
        }

        // LED点滅
        controlMotors.brakeRobot();

        for (int i = 0; i < 10; i++)
        {
            digitalWrite(ledPin, HIGH);
            delay(1000);
            digitalWrite(ledPin, LOW);
            delay(1000);
        }

        robotState = RobotState::Finished;
        break;
    }

    case RobotState::Finished: // 終わった
        break;

    default:
        break;
    }
}
