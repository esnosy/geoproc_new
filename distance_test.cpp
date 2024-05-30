#include <cmath>
#include <vector>

#include "aabb.hpp"
#include "distance.hpp"
#include "test.hpp"
#include "vec.hpp"

struct Test_Case {
  Vec3 p;
  AABB aabb;
  float expected_result;
};

int main() {
  std::vector<Test_Case> cases = {
      {Vec3(0.0f), AABB(Vec3(-1.0f), Vec3(1.0f)), 0.0f},
      {Vec3(2.0f), AABB(Vec3(-1.0f), Vec3(1.0f)), std::sqrt(3.0f)},
      {Vec3(0, 0, 1), AABB(Vec3(-1.0f), Vec3(1.0f)), 0.0f},
      {Vec3(0, 0, 2), AABB(Vec3(-1.0f), Vec3(1.0f)), 1.0f},
      {Vec3(-1, 0, 0), AABB(Vec3(-1.0f), Vec3(1.0f)), 0.0f},
  };
  for (const auto &c : cases) {
    float d = distance_to_volume(c.p, c.aabb);
    assert_close(d, c.expected_result, 1e-6f);
  }
  return 0;
}
