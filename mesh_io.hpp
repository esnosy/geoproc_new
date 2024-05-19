#pragma once

#include <cassert>
#include <optional>
#include <string_view>
#include <vector>

#include "aabb.hpp"
#include "vec.hpp"

struct Triangle {
  Vec3 a, b, c;
  Triangle(const Vec3 &a, const Vec3 &b, const Vec3 &c) : a(a), b(b), c(c) {}
  float area() const { return (b - a).cross(c - a).mag() * 0.5f; }
  Vec3 &operator[](size_t i) {
    assert(i < 3);
    if (i == 0) return a;
    if (i == 1) return b;
    return c;
  }
  const Vec3 &operator[](size_t i) const {
    assert(i < 3);
    if (i == 0) return a;
    if (i == 1) return b;
    return c;
  }
  AABB calc_aabb() const {
    AABB aabb;
    aabb.min = Vec3::min(a, Vec3::min(b, c));
    aabb.max = Vec3::max(a, Vec3::max(b, c));
    return aabb;
  }
};

struct Mesh {
  std::vector<Triangle> tris;
};

std::optional<Mesh> read_mesh(std::string_view filepath);
