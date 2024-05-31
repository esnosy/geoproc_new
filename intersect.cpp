#include <array>
#include <cstdint>
#include <optional>

#include "intersect.hpp"
#include "vec.hpp"

constexpr float RAY_MAX = 1682001.0f;

constexpr float SIGN_TABLE[3][3] = {
    {1, -1, 1},
    {-1, 1, -1},
    {1, -1, 1},
};

static bool is_zero(float value) { return std::abs(value) < 1e-9; }

std::optional<float> intersect(const Ray &ray, const AABB &aabb) {
  float running_t_min = 0.0f;
  float running_t_max = RAY_MAX;
  for (int i = 0; i < 3; i++) {
    if (is_zero(ray.direction[i]) &&
        (ray.origin[i] < aabb.min[i] || ray.origin[i] > aabb.max[i]))
      return std::nullopt;
    float t_min = (aabb.min[i] - ray.origin[i]) / ray.direction[i];
    float t_max = (aabb.max[i] - ray.origin[i]) / ray.direction[i];
    if (t_min > running_t_max) return std::nullopt;
    if (t_max < running_t_min) return std::nullopt;
    running_t_min = std::max(t_min, running_t_min);
    running_t_max = std::min(t_max, running_t_max);
  }
  return running_t_min;
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
