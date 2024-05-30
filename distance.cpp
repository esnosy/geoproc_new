#include "distance.hpp"
#include "aabb.hpp"
#include "vec.hpp"

static Vec3 closest_in_volume(const Vec3 &p, const AABB &aabb) {
  return p.clamped(aabb.min, aabb.max);
}
float distance_to_volume(const Vec3 &p, const AABB &aabb) {
  return p.dist(closest_in_volume(p, aabb));
}
