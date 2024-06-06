#pragma once

#include <optional>

#include "aabb.hpp"
#include "triangle.hpp"
#include "vec.hpp"
#include "ray.hpp"

std::optional<float> intersect(const Ray &ray, const AABB &aabb);
std::optional<float> intersect(const Ray &ray, const Triangle &triangle);

inline bool does_intersect(const Ray &ray, const AABB &aabb) {
  return intersect(ray, aabb).has_value();
}

inline bool does_intersect(const Ray &ray, const Triangle &triangle) {
  return intersect(ray, triangle).has_value();
}
