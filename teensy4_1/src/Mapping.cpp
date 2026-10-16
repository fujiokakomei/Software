#include "Mapping.hpp"

#include <algorithm>
#include <functional>
#include <queue>

namespace rcj_maze
{

const int MazeMapper::dx[4] = {0, 1, 0, -1};
const int MazeMapper::dy[4] = {1, 0, -1, 0};

int MazeMapper::relativeToAbsolute(Direction now, int relative)
{
    return (static_cast<int>(now) + relative) % 4;
}

void MazeMapper::moveForward(int &x, int &y, Direction dir, int tiles)
{
    x += dx[dir] * tiles;
    y += dy[dir] * tiles;
}

void MazeMapper::moveBackward(int &x, int &y, Direction dir, int tiles)
{
    x -= dx[dir] * tiles;
    y -= dy[dir] * tiles;
}

void MazeMapper::turnRight(Direction &dir)
{
    dir = static_cast<Direction>((static_cast<int>(dir) + 1) % 4);
}

void MazeMapper::turnLeft(Direction &dir)
{
    dir = static_cast<Direction>((static_cast<int>(dir) + 3) % 4);
}

Cell &MazeMapper::cell(int floor, int x, int y)
{
    return floors_[floor][{x, y}];
}

const Cell *MazeMapper::findCell(int floor, int x, int y) const
{
    const auto &maze = floors_[floor];
    auto it = maze.find({x, y});
    if (it == maze.end()) //存在しない場合
    {
        return nullptr;
    }
    return &it->second;
}

const Cell *MazeMapper::getNextCell(int floor, int x, int y, Direction dir) const
{
    const auto &maze = floors_[floor];
    x += dx[dir];
    y += dy[dir];
    auto it = maze.find({x, y});
    if (it == maze.end())
    {
        return nullptr;
    }
    return &it->second;
}

const Cell *MazeMapper::getRightCell(int floor, int x, int y, Direction dir) const
{
    const auto &maze = floors_[floor];
    dir = static_cast<Direction>((static_cast<int>(dir) + 1) % 4); //右にする
    x += dx[dir];
    y += dy[dir];
    auto it = maze.find({x, y});
    if (it == maze.end())
    {
        return nullptr;
    }
    return &it->second;
}

const Cell *MazeMapper::getLeftCell(int floor, int x, int y, Direction dir) const
{
    const auto &maze = floors_[floor];
    dir = static_cast<Direction>((static_cast<int>(dir) + 3) % 4); //左にする
    x += dx[dir];
    y += dy[dir];
    auto it = maze.find({x, y});
    if (it == maze.end())
    {
        return nullptr;
    }
    return &it->second;
}

const Cell *MazeMapper::getBehindCell(int floor, int x, int y, Direction dir) const
{
    const auto &maze = floors_[floor];
    dir = static_cast<Direction>((static_cast<int>(dir) + 2) % 4); //後ろにする
    x += dx[dir];
    y += dy[dir];
    auto it = maze.find({x, y});
    if (it == maze.end())
    {
        return nullptr;
    }
    return &it->second;
}

MazeMapper::FloorMap &MazeMapper::floorMap(int floor)
{
    return floors_[floor];
}

const MazeMapper::FloorMap &MazeMapper::floorMap(int floor) const
{
    return floors_[floor];
}

void MazeMapper::clear()
{
    for (auto &maze : floors_)
    {
        maze.clear();
    }
}

void MazeMapper::markVisited(int floor, int x, int y, bool visited)
{
    cell(floor, x, y).visited = visited;
}

void MazeMapper::markBlack(int floor, int x, int y, bool isBlack)
{
    cell(floor, x, y).isBlack = isBlack;
}

void MazeMapper::markRed(int floor, int x, int y, bool isRed)
{
    cell(floor, x, y).isRed = isRed;
}

void MazeMapper::markBlue(int floor, int x, int y, bool isBlue)
{
    cell(floor, x, y).isBlue = isBlue;
}

void MazeMapper::markCheckpoint(int floor, int x, int y, bool isCheckpoint)
{
    cell(floor, x, y).isCheckpoint = isCheckpoint;
    lastCheckpoint_.floor = floor;
    lastCheckpoint_.x = x;
    lastCheckpoint_.y = y;
}

void MazeMapper::markSlope(int floor, int x, int y, bool slope)
{
    cell(floor, x, y).slope = slope;
}

void MazeMapper::setWall(int floor, int x, int y, Direction dir, bool exists)
{
    auto &maze = floors_[floor];
    Cell &current = maze[{x, y}];
    current.wall[dir] = exists;

    int nx = x + dx[dir];
    int ny = y + dy[dir];
    maze[{nx, ny}].wall[(dir + 2) % 4] = exists;
}

void MazeMapper::updateMap(int floor, int x, int y, Direction dir, const std::array<bool, 4> &sense)
{
    Cell &current = cell(floor, x, y);
    current.visited = true;

    for (int relative = 0; relative < 4; relative++)
    {
        int absolute = relativeToAbsolute(dir, relative);
        setWall(floor, x, y, static_cast<Direction>(absolute), sense[relative]);
    }
}

void MazeMapper::markRampUp(int fromFloor, int fromX, int fromY, Direction dir) //送られてきたマスの次のマスを坂、その次を上の階
{
    if (fromFloor < 0 || fromFloor + 1 >= kFloorCount)
    {
        return;
    }

    int slopeX = fromX + dx[dir];
    int slopeY = fromY + dy[dir];
    int upperX = slopeX + dx[dir];
    int upperY = slopeY + dy[dir];
    int upperFloor = fromFloor + 1;

    Cell &slopeCell = cell(fromFloor, slopeX, slopeY);
    Cell &upperCell = cell(upperFloor, upperX, upperY);

    slopeCell.slope = true;
    slopeCell.hasFloorLink = true;
    slopeCell.linkedFloor = upperFloor;
    slopeCell.linkedX = upperX;
    slopeCell.linkedY = upperY;
    slopeCell.floorLinkDirection = dir;

    upperCell.hasFloorLink = true;
    upperCell.linkedFloor = fromFloor;
    upperCell.linkedX = slopeX;
    upperCell.linkedY = slopeY;
    upperCell.floorLinkDirection = static_cast<Direction>((static_cast<int>(dir) + 2) % 4);
}

void MazeMapper::markRampDown(int fromFloor, int fromX, int fromY, Direction dir) //送られてきたマスの次のマスを坂、その次を下の階
{
    if (fromFloor <= 0 || fromFloor >= kFloorCount)
    {
        return;
    }

    int slopeX = fromX + dx[dir];
    int slopeY = fromY + dy[dir];
    int lowerX = slopeX + dx[dir];
    int lowerY = slopeY + dy[dir];
    int lowerFloor = fromFloor - 1;

    Cell &slopeCell = cell(fromFloor, slopeX, slopeY);
    Cell &lowerCell = cell(lowerFloor, lowerX, lowerY);

    slopeCell.slope = true;
    slopeCell.hasFloorLink = true;
    slopeCell.linkedFloor = lowerFloor;
    slopeCell.linkedX = lowerX;
    slopeCell.linkedY = lowerY;
    slopeCell.floorLinkDirection = dir;

    lowerCell.hasFloorLink = true;
    lowerCell.linkedFloor = fromFloor;
    lowerCell.linkedX = slopeX;
    lowerCell.linkedY = slopeY;
    lowerCell.floorLinkDirection = static_cast<Direction>((static_cast<int>(dir) + 2) % 4);
}

void MazeMapper::markRampUp(int fromX, int fromY, Direction dir)
{
    markRampUp(0, fromX, fromY, dir);
}

void MazeMapper::markRampDown(int fromX, int fromY, Direction dir)
{
    markRampDown(1, fromX, fromY, dir);
}

Path MazeMapper::shortestPath(int floor, int sx, int sy, int gx, int gy) const
{
    const FloorMap &maze = floors_[floor];
    const Position start = {sx, sy};
    const Position goal = {gx, gy};

    auto startIt = maze.find(start);
    auto goalIt = maze.find(goal);
    if (startIt == maze.end() || goalIt == maze.end())
    {
        return {};
    }

    using State = std::pair<int, Position>;
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;

    std::map<Position, int> minCost;
    std::map<Position, Position> parent;

    pq.push({0, start});
    minCost[start] = 0;

    while (!pq.empty())
    {
        auto currentCost = pq.top().first;
        auto current = pq.top().second;
        pq.pop();

        if (currentCost > minCost[current])
        {
            continue;
        }

        if (current == goal)
        {
            break;
        }

        const Cell &currentCell = maze.at(current);

        for (int dir = 0; dir < 4; dir++)
        {
            if (currentCell.wall[dir])
            {
                continue;
            }

            Position next = {
                current.first + dx[dir],
                current.second + dy[dir]
            };

            auto nextIt = maze.find(next);
            if (nextIt == maze.end())
            {
                continue;
            }

            const Cell &nextCell = nextIt->second;

            // 黒タイルと赤タイルは進入禁止
            if (nextCell.isBlack || nextCell.isRed)
            {
                continue;
            }

            // 目的地以外の未探索セルは通過しない（安全な訪問済みセルのみを通る）
            if (!nextCell.visited && next != goal)
            {
                continue;
            }

            int edgeCost = 1;

            // 青タイルとスロープは極力避けるためにコストを大きくする
            if (nextCell.isBlue || nextCell.slope)
            {
                edgeCost = 100;
            }

            int nextCost = currentCost + edgeCost;

            if (minCost.find(next) == minCost.end() || nextCost < minCost[next])
            {
                minCost[next] = nextCost;
                parent[next] = current;
                pq.push({nextCost, next});
            }
        }
    }

    if (minCost.find(goal) == minCost.end())
    {
        return {};
    }

    Path path;
    Position current = goal;
    while (current != start)
    {
        path.push_back(current);
        current = parent[current];
    }
    path.push_back(start);

    std::reverse(path.begin(), path.end());
    return path;
}

FloorPath MazeMapper::shortestPath(const FloorPosition &start, const FloorPosition &goal) const
{
    if (start.floor < 0 || start.floor >= kFloorCount || goal.floor < 0 || goal.floor >= kFloorCount)
    {
        return {};
    }

    const Position startPos = {start.x, start.y};
    const Position goalPos = {goal.x, goal.y};
    if (floors_[start.floor].find(startPos) == floors_[start.floor].end() ||
        floors_[goal.floor].find(goalPos) == floors_[goal.floor].end())
    {
        return {};
    }

    using State = std::pair<int, FloorPosition>;
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;

    std::map<FloorPosition, int> minCost;
    std::map<FloorPosition, FloorPosition> parent;

    pq.push({0, start});
    minCost[start] = 0;

    while (!pq.empty())
    {
        auto currentCost = pq.top().first;
        auto current = pq.top().second;
        pq.pop();

        if (currentCost > minCost[current])
        {
            continue;
        }

        if (current == goal)
        {
            break;
        }

        const FloorMap &maze = floors_[current.floor];
        auto currentIt = maze.find({current.x, current.y});
        if (currentIt == maze.end())
        {
            continue;
        }

        const Cell &currentCell = currentIt->second;

        for (int dir = 0; dir < 4; dir++)
        {
            if (currentCell.wall[dir])
            {
                continue;
            }

            FloorPosition next = {
                current.floor,
                current.x + dx[dir],
                current.y + dy[dir]
            };

            auto nextIt = maze.find({next.x, next.y});
            if (nextIt == maze.end())
            {
                continue;
            }

            const Cell &nextCell = nextIt->second;
            if (nextCell.isBlack || nextCell.isRed)
            {
                continue;
            }

            if (!nextCell.visited && next != goal)
            {
                continue;
            }

            int edgeCost = (nextCell.isBlue || nextCell.slope) ? 100 : 1;
            int nextCost = currentCost + edgeCost;

            if (minCost.find(next) == minCost.end() || nextCost < minCost[next])
            {
                minCost[next] = nextCost;
                parent[next] = current;
                pq.push({nextCost, next});
            }
        }

        if (currentCell.hasFloorLink)
        {
            FloorPosition next = {
                currentCell.linkedFloor,
                currentCell.linkedX,
                currentCell.linkedY
            };

            if (next.floor >= 0 && next.floor < kFloorCount)
            {
                auto nextIt = floors_[next.floor].find({next.x, next.y});
                if (nextIt != floors_[next.floor].end())
                {
                    const Cell &nextCell = nextIt->second;
                    if (!nextCell.isBlack && !nextCell.isRed && (nextCell.visited || next == goal))
                    {
                        int nextCost = currentCost + 100;
                        if (minCost.find(next) == minCost.end() || nextCost < minCost[next])
                        {
                            minCost[next] = nextCost;
                            parent[next] = current;
                            pq.push({nextCost, next});
                        }
                    }
                }
            }
        }
    }

    if (minCost.find(goal) == minCost.end())
    {
        return {};
    }

    FloorPath path;
    FloorPosition current = goal;
    while (current != start)
    {
        path.push_back(current);
        current = parent[current];
    }
    path.push_back(start);

    std::reverse(path.begin(), path.end());
    return path;
}

bool MazeMapper::isMazeClosed(int floor, std::vector<Position> &unvisitedCells) const
{
    const FloorMap &maze = floors_[floor];
    unvisitedCells.clear();

    for (const auto &entry : maze)
    {
        const Position &pos = entry.first;
        const Cell &current = entry.second;

        if (!current.visited)
        {
            continue;
        }

        for (int dir = 0; dir < 4; dir++)
        {
            if (current.wall[dir])
            {
                continue;
            }

            Position next = {
                pos.first + dx[dir],
                pos.second + dy[dir]
            };

            auto it = maze.find(next);
            if (it == maze.end())
            {
                unvisitedCells.push_back(next);
                continue;
            }

            if (!it->second.visited && !it->second.isBlack && !it->second.isRed)
            {
                unvisitedCells.push_back(next);
            }
        }
    }

    return unvisitedCells.empty();
}

bool MazeMapper::isMazeClosed(std::vector<FloorPosition> &unvisitedCells) const
{
    unvisitedCells.clear();

    for (int floor = 0; floor < kFloorCount; floor++)
    {
        const FloorMap &maze = floors_[floor];

        for (const auto &entry : maze)
        {
            const Position &pos = entry.first;
            const Cell &current = entry.second;

            if (!current.visited)
            {
                continue;
            }

            for (int dir = 0; dir < 4; dir++)
            {
                if (current.wall[dir])
                {
                    continue;
                }

                FloorPosition next = {
                    floor,
                    pos.first + dx[dir],
                    pos.second + dy[dir]
                };

                auto it = maze.find({next.x, next.y});
                if (it == maze.end())
                {
                    unvisitedCells.push_back(next);
                    continue;
                }

                if (!it->second.visited && !it->second.isBlack && !it->second.isRed)
                {
                    unvisitedCells.push_back(next);
                }
            }

            if (current.hasFloorLink)
            {
                FloorPosition next = {
                    current.linkedFloor,
                    current.linkedX,
                    current.linkedY
                };

                if (next.floor < 0 || next.floor >= kFloorCount)
                {
                    continue;
                }

                auto it = floors_[next.floor].find({next.x, next.y});
                if (it == floors_[next.floor].end())
                {
                    unvisitedCells.push_back(next);
                    continue;
                }

                if (!it->second.visited && !it->second.isBlack && !it->second.isRed)
                {
                    unvisitedCells.push_back(next);
                }
            }
        }
    }

    return unvisitedCells.empty();
}

} // namespace rcj_maze
