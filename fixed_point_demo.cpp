#include <ios>
#include <iostream>
#include <ostream>

#include "fixed_point.hpp"

int main() {
  auto a = Fixed_Point_64::from_unscaled(50);
  auto original_a = a;
  a /= Fixed_Point_64::from_unscaled(2000);
  a *= Fixed_Point_64::from_unscaled(2000);
  std::cout << a << std::endl;
  std::cout << std::boolalpha << (a == original_a) << std::endl;

  auto b = Fixed_Point_64::from_unscaled(-100900) *
           Fixed_Point_64::from_unscaled(1000);
  std::cout << b << std::endl;
  std::cout << b.to_float() << std::endl;
  return 0;
}
