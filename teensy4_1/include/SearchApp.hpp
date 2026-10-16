#pragma once

#include <Arduino.h>
#include <array>
#include <vector>

#include "Mapping.hpp"

namespace searchapp
{
enum class SearchMove
{
    Forward,
    Right,
    Back,
    Left,
    Finished
};

struct SearchResult
{
    SearchMove move = SearchMove::Finished;
    bool finished = true;
    bool nextIsSlope = false;
    rcj_maze::Direction absoluteDirection = rcj_maze::NORTH;
    int nextFloor = 0;
    int nextX = 0;
    int nextY = 0;
};

class searchApp
{
public:
    SearchResult decideNextMove(
        rcj_maze::MazeMapper &mazeMapper,
        const rcj_maze::RobotPose &robotPose,
        const std::array<bool, 4> &walls,
        bool frontIsSlope
    );

    SearchResult decideNextMove(
        rcj_maze::MazeMapper &mazeMapper,
        const rcj_maze::RobotPose &robotPose,
        const std::array<bool, 4> &walls,
        int frontInfo
    );

    rcj_maze::FloorPath decideReturnWay(
        rcj_maze::MazeMapper &mazeMapper,
        const rcj_maze::RobotPose &robotPose,
        std::vector<searchapp::SearchMove> &returnWay
    );

private:
    static SearchMove relativeToMove(int relative);
    static rcj_maze::FloorPosition nextPosition(
        const rcj_maze::RobotPose &robotPose,
        rcj_maze::Direction dir
    );
    static int relativeFromAbsolute(rcj_maze::Direction now, rcj_maze::Direction next);
    static bool canEnter(const rcj_maze::Cell *cell);
    static SearchResult makeResult(
        const rcj_maze::RobotPose &robotPose,
        rcj_maze::Direction dir,
        const rcj_maze::Cell *nextCell,
        bool finished = false
    );
};
} // namespace searchapp
