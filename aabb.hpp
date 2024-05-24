#pragma once

#include "vec.hpp"

struct AABB {
  Vec3 min, max;
  AABB() : min(0.0f), max(0.0f) {}
  Vec3 calc_extent() const { return max - min; }
  Vec3 calc_center() const { return (min * 0.5f) + (max * 0.5f); }
};
