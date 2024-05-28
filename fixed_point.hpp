#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <ostream>

class Fixed_Point_64 {
private:
  int64_t value = 0;
  constexpr static int64_t num_fract_digits = 3;
  constexpr static int64_t inv_scale = [] {
    int64_t result = 1;
    for (int64_t i = 0; i < num_fract_digits; i++) result *= 10;
    return result;
  }();
  explicit Fixed_Point_64(int64_t value) : value(value) {}

public:
  // TODO: reduce repeated code
  static Fixed_Point_64 from_float(float f) { return Fixed_Point_64(std::round(f * inv_scale)); }
  static Fixed_Point_64 from_double(double f) { return Fixed_Point_64(std::round(f * inv_scale)); }
  float to_float() const {
    auto div = std::div(std::abs(value), inv_scale);
    float result = float(div.quot) + float(div.rem) / float(inv_scale);
    if (value < 0) result = -result;
    return result;
  };
  double to_double() const {
    auto div = std::div(std::abs(value), inv_scale);
    double result = double(div.quot) + double(div.rem) / double(inv_scale);
    if (value < 0) result = -result;
    return result;
  }
  static Fixed_Point_64 from_unscaled(int64_t value) { return Fixed_Point_64(value); }
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
    if (number.value < 0) os << "-";
    auto div = std::div(std::abs(number.value), inv_scale);
    // https://stackoverflow.com/a/1714538/8094047
    os << div.quot << "." << std::setfill('0') << std::setw(num_fract_digits) << div.rem;
    return os;
  }
};
