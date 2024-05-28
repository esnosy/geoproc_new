#include <iostream>
#include <ostream>

#include "fixed_point.hpp"

int main() {
  auto a = Fixed_Point_64::from_unscaled(50);
  a /= Fixed_Point_64::from_unscaled(2000);
  a *= Fixed_Point_64::from_unscaled(2000);
  std::cout << a << std::endl;

  std::cout << Fixed_Point_64::from_unscaled(-100900) * Fixed_Point_64::from_unscaled(1000) << std::endl;
  return 0;
}
