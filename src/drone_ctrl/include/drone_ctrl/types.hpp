#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include <set>

namespace multi_explore {

constexpr double GRID_RES = 0.1;   // 10cm per cell
constexpr double PI = 3.14159265358979323846;

struct GridPos {
    int x=0, y=0;
    GridPos() = default;
    GridPos(int _x, int _y) : x(_x), y(_y) {}
    bool operator==(const GridPos& o) const { return x == o.x && y == o.y; }
    bool operator<(const GridPos& o) const { return x != o.x ? x < o.x : y < o.y; }
    bool valid(const class GridMap& map) const;
};

struct Point {
    double x, y;
    Point() : x(0), y(0) {}
    Point(double _x, double _y) : x(_x), y(_y) {}
    double dist(const Point& o) const {
        double dx = x - o.x, dy = y - o.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    double heading() const { return std::atan2(y, x); }
};

enum CellState : int8_t {
    CS_UNKNOWN  = 0,
    CS_FREE     = 1,
    CS_OCCUPIED = 2,
    CS_FOV_FREE = 3   // explored by downward camera FOV
};

inline GridPos worldToGrid(double wx, double wy, double xmin, double ymin) {
    return { (int)std::floor((wx - xmin) * (1.0 / GRID_RES)),
             (int)std::floor((wy - ymin) * (1.0 / GRID_RES)) };
}

inline Point gridToWorld(int gx, int gy, double xmin, double ymin) {
    return { (gx + 0.5) * GRID_RES + xmin,
             (gy + 0.5) * GRID_RES + ymin };
}

inline double wrapAngle(double a) {
    while (a >  PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

struct FrontierCluster {
    Point centroid;
    std::vector<GridPos> cells;
    double info_gain;
};

}  // namespace multi_explore
