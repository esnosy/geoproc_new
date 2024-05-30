#include <vector>

#include "aabb.hpp"
#include "intersect.hpp"
#include "test.hpp"
#include "vec.hpp"

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
    assert_equals(does_intersect(c.ray, c.aabb), c.expected_result);
  }
  // TODO: test more functions
  return 0;
}
