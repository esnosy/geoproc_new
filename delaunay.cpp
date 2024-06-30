#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <optional>
#include <queue>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "bvh.hpp"
#include "distance.hpp"
#include "vec.hpp"

struct RGB : std::array<uint8_t, 3> {
  uint8_t &r() { return (*this)[0]; }
  uint8_t &g() { return (*this)[1]; }
  uint8_t &b() { return (*this)[2]; }
  RGB(uint8_t r, uint8_t g, uint8_t b) : std::array<uint8_t, 3>{r, g, b} {}
};

class Image {
public:
  Image(int width, int height) : width(width), height(height) {
    data.resize(width * height, RGB(0, 0, 0));
  }

  RGB &get_pixel(int x, int y) {
    assert(is_in_bounds(x, y));
    return data[y * width + x];
  }

  bool is_in_bounds(int x, int y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
  }

  int get_width() const { return width; }
  int get_height() const { return height; }

  void write_ppm(const std::string &path) const {
    std::ofstream ofs;
    ofs.exceptions(std::ios_base::badbit);
    ofs.open(path, std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    ofs.write((char *)data.data(), width * height * 3);
  }

private:
  int width, height;
  std::vector<RGB> data;
};

static int square(const int &x) { return x * x; }

static void draw_circle(Image &image, int x, int y, int r,
                        const RGB &fill_color) {
  for (int i = x - r; i < x + r; i++)
    for (int j = y - r; j < y + r; j++) {
      if (!image.is_in_bounds(i, j)) continue;
      if ((square(i - x) + square(j - y)) < square(r))
        image.get_pixel(i, j) = fill_color;
    }
}

static void draw_line(Image &image, int sx, int sy, int ex, int ey,
                      const RGB &color) {
  // https://web.archive.org/web/20240611173749/https://www.javatpoint.com/dda-line-drawing-algorithm-in-cpp
  int dx = ex - sx;
  int dy = ey - sy;

  // Determine the number of steps
  int steps = std::max(std::abs(dx), std::abs(dy));

  if (steps == 0) return;

  // Calculate increments
  float x_increment = static_cast<float>(dx) / steps;
  float y_increment = static_cast<float>(dy) / steps;

  // Initialize current coordinates
  float x = static_cast<float>(sx);
  float y = static_cast<float>(sy);

  // Perform the line drawing
  for (int i = 0; i < steps; ++i) {
    int final_x = round(x);
    int final_y = round(y);
    if (image.is_in_bounds(final_x, final_y))
      image.get_pixel(final_x, final_y) = color;
    x += x_increment;
    y += y_increment;
  }
}

static void draw_point(Image &image, const Vec3 &p, RGB color) {
  draw_circle(image, (int)std::floor(p.x), (int)std::floor(p.y), 1, color);
}

static void draw_points(Image &image, const std::vector<Vec3> &points,
                        RGB color) {
  for (const Vec3 &p : points) draw_point(image, p, color);
}

static void draw_line(Image &image, const Vec3 &start, const Vec3 &end,
                      RGB color) {
  draw_line(image, (int)std::floor(start.x), (int)std::floor(start.y),
            (int)std::floor(end.x), (int)std::floor(end.y), color);
}

namespace Closest_Point {
struct Closest_Point_Result {
  uint32_t i;
  float distance;
};

struct Queue_Item {
  const BVH_Node *node;
  float distance;
  bool operator>(const Queue_Item &other) const {
    return distance > other.distance;
  }
};

std::optional<Closest_Point_Result>
closest_point(const Vec3 &p, const std::vector<Vec3> &points,
              const BVH_Tree &bvh) {
  std::optional<Closest_Point_Result> result;
  std::priority_queue<Queue_Item, std::vector<Queue_Item>, std::greater<>>
      queue;
  const BVH_Node *root = bvh.get_root();
  queue.push({root, distance_to_volume(p, root->aabb)});

  while (!queue.empty()) {
    auto [node, d] = queue.top();
    queue.pop();

    if (result.has_value() && d > result->distance) {
      continue;
    }

    if (!node->is_leaf()) {
      float dl = distance_to_volume(p, node->left->aabb);
      float dr = distance_to_volume(p, node->right->aabb);

      queue.push({node->left, dl});
      queue.push({node->right, dr});
      continue;
    }

    for (uint32_t i = node->start; i < node->end; i++) {
      uint32_t real_i = bvh.remap_index(i);
      const Vec3 &other = points[real_i];
      float t = other.dist(p);
      if (!result.has_value() || t < result->distance) {
        result = Closest_Point_Result{real_i, t};
      }
    }
  }

  return result;
}
} // namespace Closest_Point

int main() {
  int seed = 1234;
  int width = 1280;
  int height = 720;
  int num_points = 100000;

  std::mt19937 e(seed);
  std::uniform_real_distribution<float> x_dist(0.0f, (float)width);
  std::uniform_real_distribution<float> y_dist(0.0f, (float)height);

  // Generate random points
  std::vector<Vec3> points_a;
  points_a.reserve(num_points);
  for (int i = 0; i < num_points; i++) {
    Vec3 p(x_dist(e), y_dist(e), 0.0f);
    points_a.push_back(p);
  }

  std::vector<AABB> aabbs;
  aabbs.reserve(num_points);
  for (const Vec3 &p : points_a) aabbs.emplace_back(p, p);

  BVH_Tree bvh(aabbs);

  Image image(width, height);

  std::vector<Vec3> points_b;
  points_b.reserve(num_points);
  for (int i = 0; i < num_points; i++) {
    Vec3 p(x_dist(e), y_dist(e), 0.0f);
    points_b.push_back(p);
  }

  int64_t t = 0;
  for (const Vec3 &p : points_b) {
    auto t0 = std::chrono::high_resolution_clock::now();
    auto cp = Closest_Point::closest_point(p, points_a, bvh);
    auto t1 = std::chrono::high_resolution_clock::now();
    t += std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    if (cp.has_value()) draw_line(image, p, points_a[cp->i], RGB(0, 255, 0));
  }

  std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::nanoseconds(t))
                   .count()
            << std::endl;

  draw_points(image, points_a, RGB(255, 0, 0));
  draw_points(image, points_b, RGB(0, 0, 255));

  image.write_ppm("points.ppm");
  return 0;
}
