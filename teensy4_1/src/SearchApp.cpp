#include "SearchApp.hpp"

namespace searchapp
{

namespace
{

constexpr int kFrontRelative = 0;
constexpr int kRightRelative = 1;
constexpr int kBackRelative = 2;
constexpr int kLeftRelative = 3;

const int kDx[4] = {0, 1, 0, -1};
const int kDy[4] = {1, 0, -1, 0};
const int kRightHandOrder[4] = {
    kRightRelative,
    kFrontRelative,
    kLeftRelative,
    kBackRelative
};

int pathCost(const rcj_maze::FloorPath &path, const rcj_maze::MazeMapper &mazeMapper)
{
    int cost = 0;
    for (size_t i = 1; i < path.size(); i++)
    {
        const rcj_maze::FloorPosition &pos = path[i];
        const rcj_maze::Cell *cell = mazeMapper.findCell(pos.floor, pos.x, pos.y);
        cost += (cell != nullptr && (cell->slope || cell->isBlue)) ? 100 : 1;
    }
    return cost;
}

rcj_maze::Direction directionFromDelta(int dx, int dy)
{
    if (dx > 0)
    {
        return rcj_maze::EAST;
    }
    if (dx < 0)
    {
        return rcj_maze::WEST;
    }
    if (dy < 0)
    {
        return rcj_maze::SOUTH;
    }
    return rcj_maze::NORTH;
}

} // namespace

SearchResult searchApp::decideNextMove(
    rcj_maze::MazeMapper &mazeMapper,
    const rcj_maze::RobotPose &robotPose,
    const std::array<bool, 4> &walls,
    bool frontIsSlope
)
{
    SearchResult result;
    result.nextFloor = robotPose.floor;
    result.nextX = robotPose.x;
    result.nextY = robotPose.y;
    result.absoluteDirection = robotPose.dir;

    if (robotPose.floor < 0 || robotPose.floor >= rcj_maze::MazeMapper::kFloorCount)
    {
        return result;
    }

    mazeMapper.updateMap(robotPose.floor, robotPose.x, robotPose.y, robotPose.dir, walls);

    if (frontIsSlope && !walls[kFrontRelative])
    {
        if (robotPose.floor == 0)
        {
            mazeMapper.markRampUp(robotPose.floor, robotPose.x, robotPose.y, robotPose.dir);
        }
        else
        {
            mazeMapper.markRampDown(robotPose.floor, robotPose.x, robotPose.y, robotPose.dir);
        }
    }

    for (int i = 0; i < 4; i++)
    {
        const int relative = kRightHandOrder[i];
        if (walls[relative])
        {
            continue;
        }

        const rcj_maze::Direction absolute = static_cast<rcj_maze::Direction>(
            rcj_maze::MazeMapper::relativeToAbsolute(robotPose.dir, relative)
        );
        const rcj_maze::FloorPosition next = nextPosition(robotPose, absolute);
        const rcj_maze::Cell *nextCell = mazeMapper.findCell(next.floor, next.x, next.y);

        if (!canEnter(nextCell))
        {
            continue;
        }

        if (nextCell == nullptr || !nextCell->visited)
        {
            return makeResult(robotPose, absolute, nextCell);
        }
    }

    std::vector<rcj_maze::FloorPosition> unvisitedCells;
    if (mazeMapper.isMazeClosed(unvisitedCells))
    {
        return result;
    }

    rcj_maze::FloorPath bestPath;
    int bestCost = 0;
    const rcj_maze::FloorPosition start(robotPose.floor, robotPose.x, robotPose.y);

    for (const rcj_maze::FloorPosition &goal : unvisitedCells)
    {
        mazeMapper.cell(goal.floor, goal.x, goal.y);
        const rcj_maze::FloorPath path = mazeMapper.shortestPath(start, goal);
        if (path.size() < 2)
        {
            continue;
        }

        const int cost = pathCost(path, mazeMapper);
        if (bestPath.empty() || cost < bestCost)
        {
            bestPath = path;
            bestCost = cost;
        }
    }

    if (!bestPath.empty())
    {
        const rcj_maze::FloorPosition &next = bestPath[1];
        if (next.floor == robotPose.floor)
        {
            const rcj_maze::Direction absolute = directionFromDelta(
                next.x - robotPose.x,
                next.y - robotPose.y
            );
            const rcj_maze::Cell *nextCell = mazeMapper.findCell(next.floor, next.x, next.y);
            return makeResult(robotPose, absolute, nextCell);
        }

        const rcj_maze::Cell *currentCell = mazeMapper.findCell(
            robotPose.floor,
            robotPose.x,
            robotPose.y
        );
        if (currentCell != nullptr && currentCell->hasFloorLink)
        {
            const rcj_maze::Direction linkDirection = currentCell->floorLinkDirection;
            const int relative = relativeFromAbsolute(robotPose.dir, linkDirection);
            result.move = relativeToMove(relative);
            result.finished = false;
            result.nextIsSlope = true;
            result.absoluteDirection = linkDirection;
            result.nextFloor = next.floor;
            result.nextX = next.x;
            result.nextY = next.y;
            return result;
        }
    }

    for (int i = 0; i < 4; i++)
    {
        const int relative = kRightHandOrder[i];
        if (walls[relative])
        {
            continue;
        }

        const rcj_maze::Direction absolute = static_cast<rcj_maze::Direction>(
            rcj_maze::MazeMapper::relativeToAbsolute(robotPose.dir, relative)
        );
        const rcj_maze::FloorPosition next = nextPosition(robotPose, absolute);
        const rcj_maze::Cell *nextCell = mazeMapper.findCell(next.floor, next.x, next.y);
        if (canEnter(nextCell))
        {
            return makeResult(robotPose, absolute, nextCell);
        }
    }

    return result;
}

SearchResult searchApp::decideNextMove(
    rcj_maze::MazeMapper &mazeMapper,
    const rcj_maze::RobotPose &robotPose,
    const std::array<bool, 4> &walls,
    int frontInfo
)
{
    return decideNextMove(mazeMapper, robotPose, walls, frontInfo == 1);
}

SearchMove searchApp::relativeToMove(int relative)
{
    switch (relative)
    {
    case kFrontRelative:
        return SearchMove::Forward;
    case kRightRelative:
        return SearchMove::Right;
    case kBackRelative:
        return SearchMove::Back;
    case kLeftRelative:
        return SearchMove::Left;
    default:
        return SearchMove::Finished;
    }
}

rcj_maze::FloorPosition searchApp::nextPosition(
    const rcj_maze::RobotPose &robotPose,
    rcj_maze::Direction dir
)
{
    return rcj_maze::FloorPosition(
        robotPose.floor,
        robotPose.x + kDx[dir],
        robotPose.y + kDy[dir]
    );
}

int searchApp::relativeFromAbsolute(rcj_maze::Direction now, rcj_maze::Direction next)
{
    return (static_cast<int>(next) - static_cast<int>(now) + 4) % 4;
}

bool searchApp::canEnter(const rcj_maze::Cell *cell)
{
    return cell == nullptr || (!cell->isBlack && !cell->isRed);
}

SearchResult searchApp::makeResult(
    const rcj_maze::RobotPose &robotPose,
    rcj_maze::Direction dir,
    const rcj_maze::Cell *nextCell,
    bool finished
)
{
    SearchResult result;
    const bool nextIsSlope = nextCell != nullptr && nextCell->slope;
    const rcj_maze::Direction travelDirection =
        nextIsSlope && nextCell->hasFloorLink ? nextCell->floorLinkDirection : dir;
    const int relative = relativeFromAbsolute(robotPose.dir, travelDirection);

    result.move = finished ? SearchMove::Finished : relativeToMove(relative);
    result.finished = finished;
    result.absoluteDirection = travelDirection;
    result.nextIsSlope = nextIsSlope;
    result.nextFloor = robotPose.floor;
    result.nextX = robotPose.x + kDx[dir];
    result.nextY = robotPose.y + kDy[dir];
    return result;
}


rcj_maze::FloorPath searchApp::decideReturnWay(
    rcj_maze::MazeMapper &mazeMapper,
    const rcj_maze::RobotPose &robotPose,
    std::vector<searchapp::SearchMove> &returnWay
)
{
    returnWay.clear();

    rcj_maze::FloorPosition nowPos(robotPose.floor, robotPose.x, robotPose.y);
    rcj_maze::FloorPosition goalPos(0, 0, 0); // 初期位置

    rcj_maze::FloorPath path = mazeMapper.shortestPath(nowPos, goalPos);

    // 経路が見つからない、または自分自身しかない場合は空を返す
    if (path.size() < 2)
    {
        return path;
    }

    // 経路の各ステップをSearchMoveに変換
    rcj_maze::Direction currentDir = robotPose.dir;

    for (size_t i = 0; i + 1 < path.size(); i++)
    {
        const rcj_maze::FloorPosition &from = path[i];
        const rcj_maze::FloorPosition &to = path[i + 1];

        if (from.floor != to.floor)
        {
            const rcj_maze::Cell *linkCell = mazeMapper.findCell(from.floor, from.x, from.y);
            if (linkCell == nullptr || !linkCell->hasFloorLink)
            {
                returnWay.clear();
                return {};
            }

            const rcj_maze::Direction nextDir = linkCell->floorLinkDirection;
            const int relative = relativeFromAbsolute(currentDir, nextDir);
            returnWay.push_back(relativeToMove(relative));
            currentDir = nextDir; // 坂を通過しても向きは変わらない
        }
        else
        {
            int dx = to.x - from.x;
            int dy = to.y - from.y;
            rcj_maze::Direction nextDir = directionFromDelta(dx, dy);
            int relative = relativeFromAbsolute(currentDir, nextDir);
            returnWay.push_back(relativeToMove(relative));
            currentDir = nextDir; // 移動後の向きを更新
        }
    }

    return path;
}

} // namespace searchapp
