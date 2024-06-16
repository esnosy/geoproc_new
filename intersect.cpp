#include <array>
#include <cstdint>
#include <optional>
#include <utility>

#include "intersect.hpp"
#include "math.hpp"
#include "segment.hpp"
#include "vec.hpp"

constexpr float SIGN_TABLE[3][3] = {
    {1, -1, 1},
    {-1, 1, -1},
    {1, -1, 1},
};

std::optional<float> intersect(const Ray &ray, const AABB &aabb,
                               float running_t_max) {
  float running_t_min = 0.0f;
  for (int i = 0; i < 3; i++) {
    if (is_zero(ray.direction[i]) &&
        (ray.origin[i] < aabb.min[i] || ray.origin[i] > aabb.max[i]))
      return std::nullopt;

    float rdi = ray.direction[i];
    float t_min = (aabb.min[i] - ray.origin[i]) / rdi;
    float t_max = (aabb.max[i] - ray.origin[i]) / rdi;

    // "If you multiply or divide each side by a negative quantity, the
    // inequality symbol must be reversed."
    // https://web.archive.org/web/20240603190734/https://mathcentre.ac.uk/resources/uploaded/mc-ty-inequalities-2009-1.pdf
    if (rdi < 0.0f) std::swap(t_min, t_max);

    if (t_min > running_t_max) return std::nullopt;
    if (t_max < running_t_min) return std::nullopt;
    running_t_min = std::max(t_min, running_t_min);
    running_t_max = std::min(t_max, running_t_max);
    assert(running_t_max > running_t_min);
  }
  return running_t_min;
}

bool does_intersect(const Segment &s, const AABB &aabb) {
  return intersect(Ray{s.a, s.b - s.a}, aabb, 1.0f).has_value();
}

static bool is_projected_inside(const Vec3 &p, const Triangle &t) {
  Vec3 ap = p - t.a;
  float u = ap.dot(t.b - t.a);
  float v = ap.dot(t.c - t.a);
  if (u < 0.0f) return false;
  if (u > 1.0f) return false;
  if (v < 0.0f) return false;
  if (v > 1.0f) return false;
  if ((u + v) > 1.0f) return false;
  return true;
}

static bool is_inside(const Vec3 &p, const AABB &aabb) {
  if (p.x < aabb.min.x) return false;
  if (p.y < aabb.min.y) return false;
  if (p.z < aabb.min.z) return false;
  if (p.x > aabb.max.x) return false;
  if (p.y > aabb.max.y) return false;
  if (p.z > aabb.max.z) return false;
  return true;
}

bool does_intersect(const Triangle &t, const AABB &aabb) {
  // Check if any of the triangle's vertices are inside the AABB
  if (is_inside(t.a, aabb)) return true;
  if (is_inside(t.b, aabb)) return true;
  if (is_inside(t.c, aabb)) return true;

  // Check if any of the AABB's vertices, when projected, are inside the
  // triangle
  for (const Vec3 &vertex : aabb.vertices())
    if (is_projected_inside(vertex, t)) return true;

  // Check if any of the triangle's edges intersect the AABB
  if (does_intersect(Segment{t.a, t.b}, aabb)) return true;
  if (does_intersect(Segment{t.b, t.c}, aabb)) return true;
  if (does_intersect(Segment{t.c, t.a}, aabb)) return true;

  return false;
}

static float calculate_cofactor(const std::array<Vec3, 3> &columns,
                                uint8_t cofactor_row, uint8_t cofactor_column) {
  float minor_rows[2][2];
  uint8_t minor_row = 0;
  uint8_t minor_column = 0;
  for (uint8_t row = 0; row < 3; row++) {
    if (row == cofactor_row) continue;
    for (uint8_t column = 0; column < 3; column++) {
      if (column == cofactor_column) continue;
      minor_rows[minor_row][minor_column] = columns[column][row];
      minor_column++;
    }
    minor_column = 0;
    minor_row++;
  }
  return SIGN_TABLE[cofactor_row][cofactor_column] *
         (minor_rows[0][0] * minor_rows[1][1] -
          minor_rows[0][1] * minor_rows[1][0]);
}

static Vec3 calculate_cofactor_column(const std::array<Vec3, 3> &columns,
                                      uint8_t column) {
  return Vec3{
      calculate_cofactor(columns, 0, column),
      calculate_cofactor(columns, 1, column),
      calculate_cofactor(columns, 2, column),
  };
}

// Returns rows of inverse of input columns
static std::optional<std::array<Vec3, 3>>
invert(const std::array<Vec3, 3> &columns) {
  Vec3 cofactor_column_0 = calculate_cofactor_column(columns, 0);
  float det = cofactor_column_0.dot(columns[0]);
  if (is_zero(det)) return std::nullopt;
  std::array<Vec3, 3> cofactor_columns = {
      cofactor_column_0,
      calculate_cofactor_column(columns, 1),
      calculate_cofactor_column(columns, 2),
  };
  const auto &adjoint_rows = cofactor_columns;
  return std::array<Vec3, 3>{
      adjoint_rows[0] / det,
      adjoint_rows[1] / det,
      adjoint_rows[2] / det,
  };
}

static Vec3 transform(const std::array<Vec3, 3> &rows, const Vec3 &v) {
  return Vec3{
      rows[0].dot(v),
      rows[1].dot(v),
      rows[2].dot(v),
  };
}

std::optional<float> intersect(const Ray &ray, const Triangle &triangle) {
  std::array<Vec3, 3> coefficients_matrix_columns = {
      triangle.b - triangle.a,
      triangle.c - triangle.a,
      -ray.direction,
  };
  auto inverse_rows = invert(coefficients_matrix_columns);
  if (!inverse_rows.has_value()) return std::nullopt;
  Vec3 constant_column = ray.origin - triangle.a;
  Vec3 result = transform(*inverse_rows, constant_column);
  float u = result.x;
  float v = result.y;
  float t = result.z;
  if (u > 1.0f) return std::nullopt;
  if (u < 0.0f) return std::nullopt;
  if (v > 1.0f) return std::nullopt;
  if (v < 0.0f) return std::nullopt;
  if (t < 0.0f) return std::nullopt;
  if ((u + v) > 1.0f) return std::nullopt;
  return t;
}
