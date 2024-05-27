#include "intersect.hpp"

constexpr float RAY_MAX = 1682001.0f;

static bool is_zero(float value) { return std::abs(value) < 1e-6; }

bool does_intersect(const Ray &ray, const AABB &aabb) {
  float running_t_min = 0.0f;
  float running_t_max = RAY_MAX;
  for (int i = 0; i < 3; i++) {
    if (is_zero(ray.direction[i]) && (ray.origin[i] < aabb.min[i] || ray.origin[i] > aabb.max[i])) return false;
    float t_min = (aabb.min[i] - ray.origin[i]) / ray.direction[i];
    float t_max = (aabb.max[i] - ray.origin[i]) / ray.direction[i];
    if (t_min > running_t_max) return false;
    if (t_max < running_t_min) return false;
    running_t_min = std::max(t_min, running_t_min);
    running_t_max = std::min(t_max, running_t_max);
  }
  return true;
}
