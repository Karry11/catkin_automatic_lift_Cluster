#pragma once

#include <vector>

#include "types.h"

namespace path_generate {

struct AABB {
    Vec3 center{};
    Vec3 size{};

    Vec3 half_size() const;
};

double distance_point_aabb(const Vec3& point, const AABB& box);

bool segment_intersects_aabb(const Vec3& p0, const Vec3& p1, const AABB& box);

double distance_segment_aabb(const Vec3& p0, const Vec3& p1, const AABB& box);

double nearest_distance_to_obstacles(const Vec3& point, const std::vector<AABB>& obstacles);

double nearest_segment_distance_to_obstacles(
    const Vec3& p0,
    const Vec3& p1,
    const std::vector<AABB>& obstacles);

}  // namespace path_generate
