#include <cstdlib>
#include <iostream>
#include <vector>

#include "aabb.hpp"
#include "intersect.hpp"
#include "vec.hpp"

#define runtime_assert(expression)                                                                                          \
  if (!(expression)) {                                                                                                      \
    std::cerr << "Runtime assertion failed: " << #expression << std::endl;                                                  \
    std::exit(1);                                                                                                           \
  }

struct Test_Case {
  AABB aabb;
  Ray ray;
  bool expected_result;
};

int main() {
  std::vector<Test_Case> cases = {
      {{Vec3(0.0f), Vec3(1.0f)}, {Vec3(-1.0f, 0.5f, 0.5f), Vec3(1.0f, 0.0f, 0.0f)}, true},
      {{Vec3(0.0f), Vec3(1.0f)}, {Vec3(-1.0f, 0.5f, 2.0f), Vec3(1.0f, 0.0f, 0.0f)}, false},
      {{Vec3(0.0f), Vec3(1.0f)}, {Vec3(0.5f, 0.5f, 0.5f), Vec3(1.0f, 0.0f, 0.0f)}, true},
  };
  for (const auto &c : cases) {
    runtime_assert(does_intersect(c.ray, c.aabb) == c.expected_result);
  }
  return 0;
}
