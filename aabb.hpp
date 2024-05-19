#pragma once

#include "vec.hpp"

struct AABB {
  Vec3 min, max;
  AABB() : min(0.0f), max(0.0f) {}
  Vec3 calc_extent() const { return max - min; }
};
