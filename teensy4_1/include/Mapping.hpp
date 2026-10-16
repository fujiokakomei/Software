#pragma once

#include <array>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

namespace rcj_maze
{

enum Direction
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
};

using Position = std::pair<int, int>;
using Path = std::vector<Position>;

struct FloorPosition
{
    int floor = 0;
    int x = 0;
    int y = 0;

    FloorPosition() = default;

    FloorPosition(int floorValue, int xValue, int yValue)
        : floor(floorValue), x(xValue), y(yValue)
    {
    }

    bool operator<(const FloorPosition &other) const
    {
        return std::tie(floor, x, y) < std::tie(other.floor, other.x, other.y);
    }

    bool operator==(const FloorPosition &other) const
    {
        return floor == other.floor && x == other.x && y == other.y;
    }

    bool operator!=(const FloorPosition &other) const
    {
        return !(*this == other);
    }
};

using FloorPath = std::vector<FloorPosition>;

struct CheckpointInfo
{
    int floor = 0;
    int x = 0;
    int y = 0;
};

struct Cell
{
    std::array<bool, 4> wall{};
    bool visited = false;
    bool slope = false;
    bool isBlack = false; //侵入禁止
    bool isBlue = false; //5秒停止やけど初回訪問で30point
    bool isCheckpoint = false; //チェックポイント
    bool isRed = false; //Dangerous Zone入口 今んところ入らん予定
    bool isDropped = false; //このCellでレスキューキットを投下済み
    bool hasFloorLink = false;
    int linkedFloor = 0;
    int linkedX = 0;
    int linkedY = 0;
    // このセルからフロアリンク（坂）を通るときに正対すべき向き
    Direction floorLinkDirection = NORTH;
};

struct RobotPose
{
    int floor = 0; // 0が一階、1が2階、2階まで
    int x = 0;
    int y = 0;
    Direction dir = NORTH; //実際の方位関係なく最初に向いてたのが北
};

class MazeMapper
{
public:
    using FloorMap = std::map<Position, Cell>;

    static const int kFloorCount = 2;

    static int relativeToAbsolute(Direction now, int relative);
    static void moveForward(int &x, int &y, Direction dir, int tiles = 1);
    static void moveBackward(int &x, int &y, Direction dir, int tiles = 1);
    static void turnRight(Direction &dir);
    static void turnLeft(Direction &dir);

    Cell &cell(int floor, int x, int y);
    const Cell *findCell(int floor, int x, int y) const;
    const Cell *getNextCell(int floor, int x, int y, Direction dir) const;
    const Cell *getRightCell(int floor, int x, int y, Direction dir) const;
    const Cell *getLeftCell(int floor, int x, int y, Direction dir) const;
    const Cell *getBehindCell(int floor, int x, int y, Direction dir) const;
    FloorMap &floorMap(int floor);
    const FloorMap &floorMap(int floor) const;
    void clear();

    void markVisited(int floor, int x, int y, bool visited = true);
    void markBlack(int floor, int x, int y, bool isBlack = true);
    void markRed(int floor, int x, int y, bool isRed = true);
    void markBlue(int floor, int x, int y, bool isBlue = true);
    void markCheckpoint(int floor, int x, int y, bool isCheckpoint = true);
    void markSlope(int floor, int x, int y, bool slope = true);
    void setWall(int floor, int x, int y, Direction dir, bool exists);
    void updateMap(int floor, int x, int y, Direction dir, const std::array<bool, 4> &sense);
    void markRampUp(int fromFloor, int fromX, int fromY, Direction dir);
    void markRampDown(int fromFloor, int fromX, int fromY, Direction dir);
    void markRampUp(int fromX, int fromY, Direction dir);
    void markRampDown(int fromX, int fromY, Direction dir);

    Path shortestPath(int floor, int sx, int sy, int gx, int gy) const;
    FloorPath shortestPath(const FloorPosition &start, const FloorPosition &goal) const;
    bool isMazeClosed(int floor, std::vector<Position> &unvisitedCells) const;
    bool isMazeClosed(std::vector<FloorPosition> &unvisitedCells) const;

private:
    std::array<FloorMap, kFloorCount> floors_;
    CheckpointInfo lastCheckpoint_;

    static const int dx[4];
    static const int dy[4];
};

} // namespace rcj_maze
