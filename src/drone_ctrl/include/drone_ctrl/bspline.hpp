#pragma once
#include "drone_ctrl/types.hpp"
#include <vector>
#include <cmath>

namespace multi_explore {

// Uniform cubic B-spline in 2D
class UniformBspline2D {
public:
    UniformBspline2D() {}

    void setControlPoints(const std::vector<Point>& cpts) {
        cpts_ = cpts;
    }

    Point eval(double t) const {
        if (cpts_.size() < 4) return cpts_.empty() ? Point(0,0) : cpts_.back();
        t = std::max(0.0, std::min(t, 1.0));
        double total = (double)(cpts_.size() - 3);
        double u = t * total;
        int k = std::min((int)u, (int)cpts_.size() - 4);
        double s = u - k;
        double s2 = s * s, s3 = s2 * s;
        double b0 = (1 - 3*s + 3*s2 - s3) / 6.0;
        double b1 = (4 - 6*s2 + 3*s3) / 6.0;
        double b2 = (1 + 3*s + 3*s2 - 3*s3) / 6.0;
        double b3 = s3 / 6.0;
        Point p;
        p.x = b0 * cpts_[k].x + b1 * cpts_[k+1].x + b2 * cpts_[k+2].x + b3 * cpts_[k+3].x;
        p.y = b0 * cpts_[k].y + b1 * cpts_[k+1].y + b2 * cpts_[k+2].y + b3 * cpts_[k+3].y;
        return p;
    }

    Point evalDerivative(double t) const {
        if (cpts_.size() < 4) return Point(0,0);
        t = std::max(0.0, std::min(t, 1.0));
        double total = (double)(cpts_.size() - 3);
        double u = t * total;
        int k = std::min((int)u, (int)cpts_.size() - 4);
        double s = u - k;
        double s2 = s * s;
        double b0 = (-1 + 2*s - s2) / 2.0;
        double b1 = (-2*s + 3*s2) / 2.0;
        double b2 = (1 + 2*s - 3*s2) / 2.0;
        double b3 = s2 / 2.0;
        Point d;
        d.x = b0 * cpts_[k].x + b1 * cpts_[k+1].x + b2 * cpts_[k+2].x + b3 * cpts_[k+3].x;
        d.y = b0 * cpts_[k].y + b1 * cpts_[k+1].y + b2 * cpts_[k+2].y + b3 * cpts_[k+3].y;
        return d;
    }

    double totalLen() const {
        double len = 0;
        for (size_t i = 0; i + 1 < cpts_.size(); ++i)
            len += cpts_[i].dist(cpts_[i+1]);
        return len;
    }

    bool empty() const { return cpts_.size() < 4; }
    const std::vector<Point>& cpts() const { return cpts_; }

private:
    std::vector<Point> cpts_;
};

// Convert waypoint path to B-spline control points + optimize for smoothness
inline UniformBspline2D pathToBspline(const std::vector<Point>& path) {
    UniformBspline2D bs;
    if (path.size() < 3) {
        if (!path.empty()) {
            std::vector<Point> padded(4, path.back());
            if (path.size() > 1) padded[0] = path[0];
            for (size_t i = 0; i < path.size() && i < 4; ++i) padded[i] = path[i];
            bs.setControlPoints(padded);
        }
        return bs;
    }

    // Add boundary padding for cubic B-spline
    std::vector<Point> cpts = path;
    cpts.insert(cpts.begin(), cpts.front());
    cpts.push_back(cpts.back());
    if (cpts.size() < 4) {
        while (cpts.size() < 4) cpts.push_back(cpts.back());
    }

    // Simple Catmull-Rom → B-spline conversion (uniform)
    // Actually just use A* path directly, pad to min 4 points
    bs.setControlPoints(cpts);
    return bs;
}

}  // namespace multi_explore
