#pragma once
#include "drone_ctrl/types.hpp"
#include <vector>
#include <algorithm>
#include <cstring>
#include <cmath>

namespace multi_explore {

class GridMap {
public:
    GridMap() : w_(0), h_(0), total_(0) {}

    void init(double xmin, double xmax, double ymin, double ymax) {
        xmin_ = xmin; xmax_ = xmax; ymin_ = ymin; ymax_ = ymax;
        w_ = (int)std::ceil((xmax_ - xmin_) / GRID_RES);
        h_ = (int)std::ceil((ymax_ - ymin_) / GRID_RES);
        total_ = w_ * h_;
        cells_.assign(total_, CS_UNKNOWN);
    }

    void clear() { std::fill(cells_.begin(), cells_.end(), CS_UNKNOWN); }

    int  width()      const { return w_; }
    int  height()     const { return h_; }
    int  total()      const { return total_; }
    double xmin()     const { return xmin_; }
    double xmax()     const { return xmax_; }
    double ymin()     const { return ymin_; }
    double ymax()     const { return ymax_; }
    double x_range()  const { return xmax_ - xmin_; }
    double y_range()  const { return ymax_ - ymin_; }

    CellState at(int gx, int gy) const {
        if (gx < 0 || gx >= w_ || gy < 0 || gy >= h_) return CS_UNKNOWN;
        return cells_[gy * w_ + gx];
    }

    CellState atWorld(double wx, double wy) const {
        auto g = worldToGrid(wx, wy, xmin_, ymin_);
        return at(g.x, g.y);
    }

    CellState get(const GridPos& gp) const { return at(gp.x, gp.y); }

    void set(int gx, int gy, CellState s) {
        if (gx < 0 || gx >= w_ || gy < 0 || gy >= h_) return;
        cells_[gy * w_ + gx] = s;
    }

    void set(const GridPos& gp, CellState s) { set(gp.x, gp.y, s); }

    void setWorld(double wx, double wy, CellState s) {
        auto g = worldToGrid(wx, wy, xmin_, ymin_);
        set(g.x, g.y, s);
    }

    void clearObstacles() {
        for (int i = 0; i < total_; ++i)
            if (cells_[i] == CS_OCCUPIED) cells_[i] = CS_UNKNOWN;
    }

    void markObstacle(double wx, double wy) { setWorld(wx, wy, CS_OCCUPIED); }

    void markForwardFov(double dx, double dy, double dyaw,
                        double fx, int cam_w, double fov_max_range) {
        double hfov = 2.0 * std::atan(cam_w / (2.0 * fx));
        double half = hfov / 2.0;
        int steps = (int)(fov_max_range / GRID_RES);
        double arc_len = fov_max_range * hfov;
        int n_rays = std::max(60, (int)(2.0 * arc_len / GRID_RES));
        for (int i = 0; i < n_rays; ++i) {
            double a = dyaw - half + hfov * i / (n_rays - 1);
            double ca = std::cos(a), sa = std::sin(a);
            for (int s = 0; s <= steps; ++s) {
                double dist = s * GRID_RES;
                double wx = dx + ca * dist, wy = dy + sa * dist;
                auto gp = worldToGrid(wx, wy, xmin_, ymin_);
                if (get(gp) == CS_OCCUPIED) break;
                if (get(gp) == CS_UNKNOWN) set(gp, CS_FOV_FREE);
            }
        }
    }

    void markDownwardFov(double dx, double dy, double dyaw,
                         double fx, double fy, double cx, double cy,
                         int cam_w, int cam_h, double height_m) {
        const double H = height_m > 0.3 ? height_m : 0.3;
        double x0 = (0.0 - cx) * H / fx, x1 = (cam_w - cx) * H / fx;
        double y0 = (0.0 - cy) * H / fy, y1 = (cam_h - cy) * H / fy;
        double ch = std::cos(dyaw), sh = std::sin(dyaw);
        const double STEP = GRID_RES;
        for (double ix = std::min(x0, x1); ix <= std::max(x0, x1); ix += STEP)
            for (double iy = std::min(y0, y1); iy <= std::max(y0, y1); iy += STEP) {
                double wx = dx + iy*ch - ix*sh, wy = dy + iy*sh + ix*ch;
                auto gp = worldToGrid(wx, wy, xmin_, ymin_);
                if (get(gp) != CS_OCCUPIED) set(gp, CS_FOV_FREE);
            }
    }

    double exploredPercent() const {
        long explored = 0, non_obs = 0;
        for (int i = 0; i < total_; ++i) {
            if (cells_[i] == CS_FOV_FREE) explored++;
            if (cells_[i] != CS_OCCUPIED) non_obs++;
        }
        return non_obs > 0 ? 100.0 * explored / (double)non_obs : 0.0;
    }

    bool isWalkable(int gx, int gy, int inflate) const {
        for (int dx = -inflate; dx <= inflate; ++dx)
            for (int dy = -inflate; dy <= inflate; ++dy)
                if (at(gx + dx, gy + dy) == CS_OCCUPIED) return false;
        return at(gx, gy) != CS_OCCUPIED;
    }

    bool isKnownFree(int gx, int gy) const {
        auto s = at(gx, gy);
        return s == CS_FREE || s == CS_FOV_FREE;
    }

    bool hasUnknownNeighbor(int gx, int gy) const {
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                if (at(gx + dx, gy + dy) == CS_UNKNOWN) return true;
        return false;
    }

    // ---- Coarse grid for territory assignment ----
    struct CoarseCell {
        int cx, cy;
        double wx, wy;
        int unknown_cnt, fov_cnt;
        bool assigned;
    };

    void buildCoarseGrid(double cell_size, std::vector<CoarseCell>& cells) const {
        int cw = (int)std::ceil(x_range() / cell_size);
        int ch = (int)std::ceil(y_range() / cell_size);
        double cx_sz = x_range() / cw, cy_sz = y_range() / ch;
        cells.clear();
        for (int cy = 0; cy < ch; ++cy)
            for (int cx = 0; cx < cw; ++cx) {
                CoarseCell cc;
                cc.cx=cx; cc.cy=cy;
                cc.wx=xmin_+(cx+0.5)*cx_sz; cc.wy=ymin_+(cy+0.5)*cy_sz;
                cc.unknown_cnt=0; cc.fov_cnt=0; cc.assigned=false;
                int gx0=(int)(cx*cx_sz/GRID_RES), gy0=(int)(cy*cy_sz/GRID_RES);
                int gx1=std::min((int)((cx+1)*cx_sz/GRID_RES),w_);
                int gy1=std::min((int)((cy+1)*cy_sz/GRID_RES),h_);
                for (int gy=gy0;gy<gy1;++gy)
                    for (int gx=gx0;gx<gx1;++gx) {
                        auto s=at(gx,gy);
                        if (s==CS_UNKNOWN) cc.unknown_cnt++;
                        if (s==CS_FOV_FREE) cc.fov_cnt++;
                    }
                if (cc.unknown_cnt>0||cc.fov_cnt>0) cells.push_back(cc);
            }
    }

    void getFrontiersInCell(const CoarseCell& cc, double cell_size,
                            const std::vector<FrontierCluster>& all,
                            std::vector<FrontierCluster>& out) const {
        double cw=x_range()/std::ceil(x_range()/cell_size);
        double ch=y_range()/std::ceil(y_range()/cell_size);
        double x0=xmin_+cc.cx*cw, x1=x0+cw;
        double y0=ymin_+cc.cy*ch, y1=y0+ch;
        out.clear();
        for (auto& fc : all)
            if (fc.centroid.x>=x0&&fc.centroid.x<x1&&fc.centroid.y>=y0&&fc.centroid.y<y1)
                out.push_back(fc);
    }

    bool inBounds(int gx, int gy) const { return gx>=0 && gx<w_ && gy>=0 && gy<h_; }

private:
    double xmin_, xmax_, ymin_, ymax_;
    int w_, h_, total_;
    std::vector<CellState> cells_;
};

inline bool GridPos::valid(const GridMap& map) const { return map.inBounds(x, y); }

}  // namespace multi_explore
