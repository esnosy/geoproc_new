#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <ostream>

class Fixed_Point_64 {
private:
  uint64_t value = 0;
  constexpr static uint64_t inv_scale = 1000;
  explicit Fixed_Point_64(uint64_t value) : value(value) {}

public:
  explicit Fixed_Point_64(float f) : value(std::round(f * inv_scale)) {}
  explicit Fixed_Point_64(double f) : value(std::round(f * inv_scale)) {}
  Fixed_Point_64 operator+(Fixed_Point_64 other) const { return Fixed_Point_64(value + other.value); }
  Fixed_Point_64 operator-(Fixed_Point_64 other) const { return Fixed_Point_64(value - other.value); }
  Fixed_Point_64 operator*(Fixed_Point_64 other) const { return Fixed_Point_64((value * other.value) / inv_scale); }
  Fixed_Point_64 operator/(Fixed_Point_64 other) const { return Fixed_Point_64((value * inv_scale) / other.value); }
  friend std::ostream &operator<<(std::ostream &os, const Fixed_Point_64 &number) {
    return os << number.value / inv_scale << "." << number.value % inv_scale;
  }
};
