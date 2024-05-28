#pragma once

#include <cassert>
#include <cmath>
#include <iostream>

struct Vec3 {
  float x, y, z;
  explicit Vec3(float s) : x(s), y(s), z(s) {}
  Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
  Vec3 operator+(const Vec3 &v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
  Vec3 operator-(const Vec3 &v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
  Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
  friend Vec3 operator*(float s, const Vec3 &v) { return v * s; }
  Vec3 cross(const Vec3 &v) const { return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
  float mag() const { return std::sqrt(x * x + y * y + z * z); }
  static Vec3 max(const Vec3 &a, const Vec3 &b) {
    return Vec3(std::fmax(a.x, b.x), std::fmax(a.y, b.y), std::fmax(a.z, b.z));
  }
  static Vec3 min(const Vec3 &a, const Vec3 &b) {
    return Vec3(std::fmin(a.x, b.x), std::fmin(a.y, b.y), std::fmin(a.z, b.z));
  }
  Vec3 clamped(const Vec3 &min, const Vec3 &max) const { return Vec3::min(Vec3::max(*this, min), max); }
  float dist(const Vec3 &other) const { return (other - *this).mag(); }
  friend std::ostream &operator<<(std::ostream &os, const Vec3 &v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
  }
  float &operator[](size_t i) {
    assert(i < 3);
    if (i == 0) return x;
    if (i == 1) return y;
    return z;
  }
  float operator[](size_t i) const {
    assert(i < 3);
    if (i == 0) return x;
    if (i == 1) return y;
    return z;
  }
};

struct Vec2 {
  float x, y;
  explicit Vec2(float s) : x(s), y(s) {}
  Vec2(float x, float y) : x(x), y(y) {}
  Vec2 operator+(const Vec2 &v) const { return Vec2(x + v.x, y + v.y); }
  Vec2 operator-(const Vec2 &v) const { return Vec2(x - v.x, y - v.y); }
  Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
  friend Vec2 operator*(float s, const Vec2 &v) { return v * s; }
};
