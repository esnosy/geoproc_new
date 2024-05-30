#pragma once

#include "aabb.hpp"
#include "triangle.hpp"
#include "vec.hpp"

struct Ray {
  Vec3 origin, direction;
};

bool does_intersect(const Ray &ray, const AABB &aabb);
bool does_intersect(const Ray &ray, const Triangle &triangle);
