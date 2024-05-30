#pragma once

#include <cstdlib>
#include <ios>
#include <iostream>

template <typename T1, typename T2> void assert_equals(T1 a, T2 b) {
  if (a != b) {
    std::cerr << std::boolalpha << "Equality assertion failed: a=" << a << ", b=" << b << std::endl;
    std::exit(1);
  }
}

template <typename T1, typename T2, typename T3> void assert_close(T1 a, T2 b, T3 abs_err) {
  if (std::abs(a - b) > abs_err) {
    std::cerr << std::boolalpha << "Close assertion failed: a=" << a << ", b=" << b << std::endl;
    std::exit(1);
  }
}
