#pragma once

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <ostream>

class Fixed_Point_64 {
private:
  uint64_t value = 0;
  constexpr static uint64_t num_fract_digits = 3;
  constexpr static uint64_t inv_scale = [] {
    uint64_t scale = 1;
    for (uint64_t i = 0; i < num_fract_digits; i++) scale *= 10;
    return scale;
  }();
  explicit Fixed_Point_64(uint64_t value) : value(value) {}

public:
  static Fixed_Point_64 from_float(float f) { return Fixed_Point_64(std::round(f * inv_scale)); }
  static Fixed_Point_64 from_double(double f) { return Fixed_Point_64(std::round(f * inv_scale)); }
  static Fixed_Point_64 from_parts(uint64_t integer, uint64_t fraction) {
    return Fixed_Point_64(integer * inv_scale + fraction);
  }
  Fixed_Point_64 operator+(const Fixed_Point_64 &other) const { return Fixed_Point_64(value + other.value); }
  Fixed_Point_64 operator-(const Fixed_Point_64 &other) const { return Fixed_Point_64(value - other.value); }
  Fixed_Point_64 operator*(const Fixed_Point_64 &other) const { return Fixed_Point_64((value * other.value) / inv_scale); }
  Fixed_Point_64 operator/(const Fixed_Point_64 &other) const { return Fixed_Point_64((value * inv_scale) / other.value); }
  Fixed_Point_64 &operator+=(const Fixed_Point_64 &other) { return *this = *this + other; }
  Fixed_Point_64 &operator-=(const Fixed_Point_64 &other) { return *this = *this - other; }
  Fixed_Point_64 &operator*=(const Fixed_Point_64 &other) { return *this = *this * other; }
  Fixed_Point_64 &operator/=(const Fixed_Point_64 &other) { return *this = *this / other; }
  bool operator==(const Fixed_Point_64 &other) const { return value == other.value; }
  friend std::ostream &operator<<(std::ostream &os, const Fixed_Point_64 &number) {
    // https://stackoverflow.com/a/1714538/8094047
    return os << number.value / inv_scale << "." << std::setfill('0') << std::setw(num_fract_digits)
              << number.value % inv_scale;
  }
};
