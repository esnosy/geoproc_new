#pragma once

#include <optional>

#include "aabb.hpp"
#include "ray.hpp"
#include "segment.hpp"
#include "triangle.hpp"
#include "vec.hpp"

constexpr float RAY_MAX = 1682001.0f;

std::optional<float> intersect(const Ray &ray, const AABB &aabb,
                               float running_t_max = RAY_MAX);
std::optional<float> intersect(const Ray &ray, const Triangle &triangle);

inline bool does_intersect(const Ray &ray, const AABB &aabb) {
  return intersect(ray, aabb).has_value();
}

inline bool does_intersect(const Ray &ray, const Triangle &triangle) {
  return intersect(ray, triangle).has_value();
}

bool does_intersect(const Segment &s, const AABB &aabb);
bool does_intersect(const Triangle &t, const AABB &aabb);
