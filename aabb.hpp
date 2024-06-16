#pragma once

#include <array>

#include "vec.hpp"

struct AABB {
  Vec3 min, max;
  AABB() : min(0.0f), max(0.0f) {}
  AABB(const Vec3 &min, const Vec3 &max) : min(min), max(max) {}
  Vec3 calc_extent() const { return max - min; }
  Vec3 calc_center() const { return (min * 0.5f) + (max * 0.5f); }
  std::array<Vec3, 8> vertices() const {
    return {
        min,
        max,
        {min.x, min.y, max.z},
        {min.x, max.y, min.z},
        {max.x, min.y, min.z},
        {min.x, max.y, max.z},
        {max.x, max.y, min.z},
        {max.x, min.y, max.z},
    };
  }
};
