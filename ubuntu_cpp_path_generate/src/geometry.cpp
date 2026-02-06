#include "geometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace path_generate {

Vec3 AABB::half_size() const {
    return {size[0] / 2.0, size[1] / 2.0, size[2] / 2.0};
}

static std::pair<Vec3, Vec3> aabb_min_max(const AABB& box) {
    Vec3 half = box.half_size();
    Vec3 min = {box.center[0] - half[0], box.center[1] - half[1], box.center[2] - half[2]};
    Vec3 max = {box.center[0] + half[0], box.center[1] + half[1], box.center[2] + half[2]};
    return {min, max};
}

double distance_point_aabb(const Vec3& point, const AABB& box) {
    Vec3 half = box.half_size();
    double dx = std::max(std::abs(point[0] - box.center[0]) - half[0], 0.0);
    double dy = std::max(std::abs(point[1] - box.center[1]) - half[1], 0.0);
    double dz = std::max(std::abs(point[2] - box.center[2]) - half[2], 0.0);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool segment_intersects_aabb(const Vec3& p0, const Vec3& p1, const AABB& box) {
    auto min_max = aabb_min_max(box);
    const Vec3& bmin = min_max.first;
    const Vec3& bmax = min_max.second;
    double x0 = p0[0];
    double y0 = p0[1];
    double z0 = p0[2];
    double dx = p1[0] - x0;
    double dy = p1[1] - y0;
    double dz = p1[2] - z0;

    double tmin = 0.0;
    double tmax = 1.0;

    const double eps = 1e-15;
    for (int i = 0; i < 3; ++i) {
        double p = (i == 0) ? x0 : (i == 1 ? y0 : z0);
        double d = (i == 0) ? dx : (i == 1 ? dy : dz);
        double minv = bmin[i];
        double maxv = bmax[i];

        if (std::abs(d) < eps) {
            if (p < minv || p > maxv) {
                return false;
            }
            continue;
        }

        double inv_d = 1.0 / d;
        double t0 = (minv - p) * inv_d;
        double t1 = (maxv - p) * inv_d;
        if (t0 > t1) {
            std::swap(t0, t1);
        }
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmin > tmax) {
            return false;
        }
    }

    return true;
}

double distance_segment_aabb(const Vec3& p0, const Vec3& p1, const AABB& box) {
    if (segment_intersects_aabb(p0, p1, box)) {
        return 0.0;
    }

    auto min_max = aabb_min_max(box);
    const Vec3& bmin = min_max.first;
    const Vec3& bmax = min_max.second;
    double x0 = p0[0];
    double y0 = p0[1];
    double z0 = p0[2];
    double dx = p1[0] - x0;
    double dy = p1[1] - y0;
    double dz = p1[2] - z0;

    std::vector<double> breaks{0.0, 1.0};
    const double eps = 1e-15;
    for (int i = 0; i < 3; ++i) {
        double p = (i == 0) ? x0 : (i == 1 ? y0 : z0);
        double d = (i == 0) ? dx : (i == 1 ? dy : dz);
        if (std::abs(d) < eps) {
            continue;
        }
        double minv = bmin[i];
        double maxv = bmax[i];
        for (double bound : {minv, maxv}) {
            double t = (bound - p) / d;
            if (t > 0.0 && t < 1.0) {
                breaks.push_back(t);
            }
        }
    }
    std::sort(breaks.begin(), breaks.end());
    breaks.erase(std::unique(breaks.begin(), breaks.end()), breaks.end());

    auto point = [&](double t) -> Vec3 {
        return {x0 + dx * t, y0 + dy * t, z0 + dz * t};
    };

    auto sq_distance_to_aabb = [&](const Vec3& pt) -> double {
        Vec3 half = box.half_size();
        double ddx = std::max(std::abs(pt[0] - box.center[0]) - half[0], 0.0);
        double ddy = std::max(std::abs(pt[1] - box.center[1]) - half[1], 0.0);
        double ddz = std::max(std::abs(pt[2] - box.center[2]) - half[2], 0.0);
        return ddx * ddx + ddy * ddy + ddz * ddz;
    };

    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < breaks.size(); ++i) {
        double a = breaks[i];
        double b = breaks[i + 1];
        double tmid = (a + b) / 2.0;
        Vec3 pmid = point(tmid);

        std::vector<std::pair<double, double>> active;
        for (int axis = 0; axis < 3; ++axis) {
            double p_start = (axis == 0) ? x0 : (axis == 1 ? y0 : z0);
            double d_axis = (axis == 0) ? dx : (axis == 1 ? dy : dz);
            double minv = bmin[axis];
            double maxv = bmax[axis];
            double p_mid = pmid[axis];
            if (p_mid < minv) {
                active.emplace_back(d_axis, p_start - minv);
            } else if (p_mid > maxv) {
                active.emplace_back(d_axis, p_start - maxv);
            }
        }

        if (active.empty()) {
            best = std::min(best, sq_distance_to_aabb(point(a)));
            best = std::min(best, sq_distance_to_aabb(point(b)));
            continue;
        }

        double A = 0.0;
        double B = 0.0;
        for (const auto& pair : active) {
            double s = pair.first;
            double c = pair.second;
            A += s * s;
            B += 2.0 * s * c;
        }

        std::vector<double> cand_ts;
        if (A < 1e-18) {
            cand_ts = {a, b};
        } else {
            double t_star = -B / (2.0 * A);
            cand_ts = {a, b, std::max(a, std::min(b, t_star))};
        }

        for (double t : cand_ts) {
            best = std::min(best, sq_distance_to_aabb(point(t)));
        }
    }

    if (!std::isfinite(best)) {
        return std::numeric_limits<double>::infinity();
    }
    return std::sqrt(best);
}

double nearest_distance_to_obstacles(const Vec3& point, const std::vector<AABB>& obstacles) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& obs : obstacles) {
        best = std::min(best, distance_point_aabb(point, obs));
    }
    return best;
}

double nearest_segment_distance_to_obstacles(
    const Vec3& p0,
    const Vec3& p1,
    const std::vector<AABB>& obstacles) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& obs : obstacles) {
        best = std::min(best, distance_segment_aabb(p0, p1, obs));
    }
    return best;
}

}  // namespace path_generate
