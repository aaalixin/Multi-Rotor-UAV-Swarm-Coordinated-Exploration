#pragma once
#include "drone_ctrl/types.hpp"
#include "drone_ctrl/grid_map.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace multi_explore {

constexpr int    INFLATE_RADIUS = 3;   // 0.3m (fly_demo default)
constexpr double DRONE_RADIUS   = 0.5;
constexpr double SPLINE_RES     = 0.04;

class PathPlanner {
public:
    const GridMap* map_ = nullptr;

    // FUEL-style: A* + LOS pruning only, no EGO optimization
    std::vector<Point> planFuel(Point start, Point goal, bool allowUnknown = false) const {
        return planImpl(start, goal, allowUnknown, false);
    }

    // fly_demo-style: A* + LOS + EGO + Catmull-Rom (full pipeline)
    std::vector<Point> plan(Point start, Point goal, bool allowUnknown = false) const {
        return planImpl(start, goal, allowUnknown, true);
    }

private:
    std::vector<Point> planImpl(Point start, Point goal, bool allowUnknown, bool useEgo) const {
        if (!map_) return {};

        GridPos gs = worldToGrid(start.x, start.y, map_->xmin(), map_->ymin());
        GridPos gg = worldToGrid(goal.x,  goal.y,  map_->xmin(), map_->ymin());

        gs = nearestFree(gs, allowUnknown);
        gg = nearestFree(gg, allowUnknown);
        if (!gs.valid(*map_) || !gg.valid(*map_)) return {};

        struct Node { GridPos pos; double g,f; GridPos parent; bool closed; };
        std::unordered_map<int64_t, Node> nodes;
        nodes.reserve(50000);
        auto encode = [&](GridPos p) -> int64_t { return (int64_t)p.y * map_->width() + p.x; };
        auto cmp = [](Node* a, Node* b) { return a->f > b->f; };
        std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);

        int64_t sk = encode(gs);
        nodes[sk] = {gs, 0, heuristic(gs,gg), gs, false};
        open.push(&nodes[sk]);

        bool found = false;
        int64_t gk = encode(gg);

        while (!open.empty()) {
            Node* cur = open.top(); open.pop();
            if (cur->closed) continue;
            cur->closed = true;
            if (cur->pos == gg) { found = true; break; }

            for (auto& nb : neighbors8(cur->pos)) {
                if (!nb.valid(*map_)) continue;
                bool walkable = (cur->pos == gs)
                    ? isTraversableCell(nb, allowUnknown)
                    : cellWalkable(nb, allowUnknown);
                if (!walkable) continue;

                double sc = (nb.x != cur->pos.x && nb.y != cur->pos.y) ? 1.414 : 1.0;
                double ng = cur->g + sc;
                int64_t nk = encode(nb);
                auto it = nodes.find(nk);
                if (it == nodes.end()) {
                    nodes[nk] = {nb, ng, ng + heuristic(nb,gg), cur->pos, false};
                    open.push(&nodes[nk]);
                } else if (!it->second.closed && ng < it->second.g) {
                    it->second.g = ng; it->second.f = ng + heuristic(nb,gg);
                    it->second.parent = cur->pos;
                    open.push(&it->second);
                }
            }
        }
        if (!found) return {};

        std::vector<Point> raw;
        int64_t k = gk;
        while (k != sk) {
            auto it = nodes.find(k);
            if (it == nodes.end()) break;
            raw.push_back(gridToWorld(it->second.pos.x, it->second.pos.y, map_->xmin(), map_->ymin()));
            k = encode(it->second.parent);
        }
        raw.push_back(start);
        std::reverse(raw.begin(), raw.end());

        auto pruned = prunePath(raw, allowUnknown);
        auto path = useEgo ? egoOptimize(pruned, allowUnknown) : pruned;
        auto smooth = interpolatePath(path, allowUnknown);

        for (auto& p : smooth) {
            p.x = std::min(std::max(p.x, map_->xmin() + DRONE_RADIUS), map_->xmax() - DRONE_RADIUS);
            p.y = std::min(std::max(p.y, map_->ymin() + DRONE_RADIUS), map_->ymax() - DRONE_RADIUS);
        }
        return smooth;
    }

private:
    static double heuristic(GridPos a, GridPos b) {
        return std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
    }
    static std::vector<GridPos> neighbors8(GridPos p) {
        std::vector<GridPos> nbs;
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                nbs.push_back({p.x + dx, p.y + dy});
            }
        return nbs;
    }
    bool isTraversableCell(GridPos p, bool allowUnknown) const {
        auto s = map_->at(p.x, p.y);
        return s == CS_FREE || s == CS_FOV_FREE || (allowUnknown && s == CS_UNKNOWN);
    }
    bool cellWalkable(GridPos p, bool allowUnknown = false) const {
        if (!p.valid(*map_)) return false;
        Point wp = gridToWorld(p.x, p.y, map_->xmin(), map_->ymin());
        if (wp.x < map_->xmin() + DRONE_RADIUS || wp.x > map_->xmax() - DRONE_RADIUS ||
            wp.y < map_->ymin() + DRONE_RADIUS || wp.y > map_->ymax() - DRONE_RADIUS)
            return false;
        CellState s = map_->at(p.x, p.y);
        if (s == CS_OCCUPIED) return false;
        if (s == CS_UNKNOWN && !allowUnknown) return false;
        for (int dy = -INFLATE_RADIUS; dy <= INFLATE_RADIUS; ++dy)
            for (int dx = -INFLATE_RADIUS; dx <= INFLATE_RADIUS; ++dx)
                if (map_->at(p.x + dx, p.y + dy) == CS_OCCUPIED) return false;
        return true;
    }
    GridPos nearestFree(GridPos p, bool allowUnknown = false) const {
        if (cellWalkable(p, allowUnknown)) return p;
        for (int r = 1; r <= 50; ++r)
            for (int dy = -r; dy <= r; ++dy)
                for (int dx = -r; dx <= r; ++dx) {
                    GridPos nb(p.x + dx, p.y + dy);
                    if (nb.valid(*map_) && cellWalkable(nb, allowUnknown)) return nb;
                }
        return GridPos(-1, -1);
    }

    // ---- LOS pruning ----
    std::vector<Point> prunePath(const std::vector<Point>& raw, bool allowUnknown) const {
        if (raw.size() <= 2) return raw;
        std::vector<Point> pruned; pruned.push_back(raw.front());
        size_t i = 0;
        while (i < raw.size() - 1) {
            size_t farthest = i + 1;
            for (size_t j = raw.size() - 1; j > i; --j) {
                if (lineClear(raw[i], raw[j], allowUnknown)) { farthest = j; break; }
            }
            pruned.push_back(raw[farthest]); i = farthest;
        }
        return pruned;
    }
    bool lineClear(const Point& a, const Point& b, bool allowUnknown) const {
        double dx = b.x - a.x, dy = b.y - a.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        int steps = std::max(4, (int)(dist / (GRID_RES * 0.5)));
        for (int i = 0; i <= steps; ++i) {
            double t = (double)i / steps;
            double wx = a.x + t * dx, wy = a.y + t * dy;
            if (!cellWalkable(worldToGrid(wx, wy, map_->xmin(), map_->ymin()), allowUnknown))
                return false;
        }
        return true;
    }

    // ---- EGO gradient descent ----
    std::vector<Point> egoOptimize(const std::vector<Point>& path, bool allowUnknown) const {
        if (path.size() <= 2) return path;
        const double SAFE_DIST = (INFLATE_RADIUS + 4) * GRID_RES;
        const int SR = INFLATE_RADIUS * 2 + 4;
        const double SMOOTH_W = 0.8, DIST_W = 3.0;
        const double STEP_INIT = 0.04;
        const int MAX_ITER = 200;

        auto nearestObs = [&](double wx, double wy) -> std::pair<double,Point> {
            GridPos gp = worldToGrid(wx, wy, map_->xmin(), map_->ymin());
            double best_d = 1e9; Point dir(0,0);
            for (int dy = -SR; dy <= SR; ++dy)
                for (int dx = -SR; dx <= SR; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    if (map_->at(gp.x+dx, gp.y+dy) == CS_OCCUPIED) {
                        double d = std::sqrt(dx*dx + dy*dy) * GRID_RES;
                        if (d < best_d) { best_d = d; dir = Point(-dx*GRID_RES/d, -dy*GRID_RES/d); }
                    }
                }
            return {best_d, dir};
        };

        std::vector<Point> ctrl = path;
        double step = STEP_INIT;
        for (int iter = 0; iter < MAX_ITER; ++iter) {
            std::vector<Point> grad(ctrl.size());
            double cost = 0;
            for (size_t i = 1; i + 1 < ctrl.size(); ++i) {
                auto [d, dir] = nearestObs(ctrl[i].x, ctrl[i].y);
                if (d < SAFE_DIST && d > 1e-6) {
                    double err = SAFE_DIST - d;
                    cost += DIST_W * err * err * err;
                    grad[i].x += DIST_W * (-3.0*err*err) * dir.x;
                    grad[i].y += DIST_W * (-3.0*err*err) * dir.y;
                }
            }
            for (size_t i = 2; i + 1 < ctrl.size(); ++i) {
                double jx = ctrl[i+1].x - 3*ctrl[i].x + 3*ctrl[i-1].x - ctrl[i-2].x;
                double jy = ctrl[i+1].y - 3*ctrl[i].y + 3*ctrl[i-1].y - ctrl[i-2].y;
                cost += SMOOTH_W*(jx*jx+jy*jy);
                double g = SMOOTH_W*2;
                grad[i-2].x += -g*jx; grad[i-2].y += -g*jy;
                grad[i-1].x += 3*g*jx; grad[i-1].y += 3*g*jy;
                grad[i  ].x += -3*g*jx; grad[i  ].y += -3*g*jy;
                grad[i+1].x += g*jx;    grad[i+1].y += g*jy;
            }
            // Armijo line search
            bool accepted = false;
            double st = step;
            for (int ls = 0; ls < 12; ++ls) {
                std::vector<Point> next = ctrl;
                for (size_t i = 1; i + 1 < ctrl.size(); ++i) {
                    next[i].x -= st * grad[i].x;
                    next[i].y -= st * grad[i].y;
                }
                double nc = 0;
                for (size_t i = 1; i + 1 < ctrl.size(); ++i) {
                    auto [d, dd] = nearestObs(next[i].x, next[i].y);
                    if (d < SAFE_DIST && d > 1e-6) { double e = SAFE_DIST-d; nc += DIST_W*e*e*e; }
                }
                double gnorm2 = 0;
                for (auto& gv : grad) gnorm2 += gv.x*gv.x + gv.y*gv.y;
                if (nc <= cost - 1e-4*st*gnorm2) { ctrl = next; step = std::min(st*1.5, 0.2); accepted = true; break; }
                st *= 0.5;
            }
            if (!accepted || cost < 0.001) break;
        }
        return ctrl;
    }

    // ---- Catmull-Rom interpolation ----
    std::vector<Point> interpolatePath(const std::vector<Point>& ctrl, bool allowUnknown) const {
        if (ctrl.size() < 2) return ctrl;
        std::vector<Point> dense; dense.reserve(1024);
        dense.push_back(ctrl.front());
        for (size_t i = 0; i + 1 < ctrl.size(); ++i) {
            Point p0 = (i>0)?ctrl[i-1]:ctrl[i], p1=ctrl[i], p2=ctrl[i+1];
            Point p3 = (i+2<ctrl.size())?ctrl[i+2]:ctrl[i+1];
            double seg = std::sqrt((p2.x-p1.x)*(p2.x-p1.x)+(p2.y-p1.y)*(p2.y-p1.y));
            int steps = std::max(2, (int)(seg/SPLINE_RES));
            for (int s=1; s<=steps; ++s) {
                double t=(double)s/steps, t2=t*t, t3=t2*t;
                double x=0.5*(2*p1.x+(-p0.x+p2.x)*t+(2*p0.x-5*p1.x+4*p2.x-p3.x)*t2+(-p0.x+3*p1.x-3*p2.x+p3.x)*t3);
                double y=0.5*(2*p1.y+(-p0.y+p2.y)*t+(2*p0.y-5*p1.y+4*p2.y-p3.y)*t2+(-p0.y+3*p1.y-3*p2.y+p3.y)*t3);
                if (cellWalkable(worldToGrid(x,y,map_->xmin(),map_->ymin()),allowUnknown))
                    dense.emplace_back(x,y);
                else {
                    double lx=p1.x+t*(p2.x-p1.x), ly=p1.y+t*(p2.y-p1.y);
                    if (cellWalkable(worldToGrid(lx,ly,map_->xmin(),map_->ymin()),allowUnknown))
                        dense.emplace_back(lx,ly);
                }
            }
        }
        if (dense.size()<2||dense.back().dist(ctrl.back())>0.01) dense.push_back(ctrl.back());
        return dense;
    }
};

}  // namespace multi_explore
