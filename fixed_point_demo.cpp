#include <iostream>
#include <ostream>

#include "fixed_point.hpp"

int main() {
  auto a = Fixed_Point_64::from_parts(0, 50);
  a /= Fixed_Point_64::from_parts(2, 0);
  a *= Fixed_Point_64::from_parts(2, 0);
  std::cout << a << std::endl;
  return 0;
}
