#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <stack>
#include <string>
#include <utility>
#include <vector>

#include "aabb.hpp"
#include "bvh.hpp"
#include "intersect.hpp"
#include "mesh_io.hpp"
#include "triangle.hpp"
#include "vec.hpp"
#include "write_ply.hpp"

constexpr float PI = 3.1415927f;

static size_t count_intersections(const Ray &r, const BVH_Tree &tree,
                                  const std::vector<Triangle> &tris) {
  std::stack<const BVH_Node *> stack;
  stack.push(tree.get_root());
  size_t num_hits = 0;
  while (!stack.empty()) {
    const BVH_Node *node = stack.top();
    stack.pop();
    if (!does_intersect(r, node->aabb)) continue;
    if (node->is_leaf()) {
      for (uint32_t i = node->start; i < node->end; i++) {
        const Triangle &t = tris[tree.remap_index(i)];
        if (does_intersect(r, t)) num_hits++;
      }
    } else {
      stack.push(node->left);
      stack.push(node->right);
    }
  }
  return num_hits;
}

struct Closest_Hit_Result {
  float t;
  uint32_t triangle_index;
};

static std::optional<Closest_Hit_Result>
closest_hit(const Ray &r, const BVH_Tree &tree,
            const std::vector<Triangle> &tris) {
  std::optional<Closest_Hit_Result> result;
  std::stack<const BVH_Node *> stack;
  stack.push(tree.get_root());
  while (!stack.empty()) {
    const BVH_Node *node = stack.top();
    stack.pop();
    if (!does_intersect(r, node->aabb)) continue;
    if (!node->is_leaf()) {
      stack.push(node->left);
      stack.push(node->right);
      continue;
    }
    for (uint32_t i = node->start; i < node->end; i++) {
      uint32_t ti = tree.remap_index(i);
      const Triangle &t = tris[ti];
      auto hit = intersect(r, t);
      if (!hit.has_value()) continue;
      if (!result.has_value() || hit.value() < result->t)
        result = {hit.value(), ti};
    }
  }
  return result;
}

static bool is_point_in_volume(const Vec3 &p, const BVH_Tree &tree,
                               const std::vector<Triangle> &tris) {
  Ray r{p, Vec3(0, 0, 1)};
  size_t num_hits = count_intersections(r, tree, tris);
  return num_hits % 2 != 0;
}

static bool is_point_in_volume_2(const Vec3 &p, const BVH_Tree &tree,
                                 const std::vector<Triangle> &tris) {
  Ray r{p, Vec3(0, 0, 1)};
  auto hit = closest_hit(r, tree, tris);
  if (!hit.has_value()) return false;
  const Triangle &t = tris[hit->triangle_index];
  Vec3 n = t.calc_normal();
  return n.dot(r.direction) > 0.0f;
}

static bool is_point_in_volume_3(const Vec3 &p, const BVH_Tree &tree,
                                 const std::vector<Triangle> &tris,
                                 const std::vector<Vec3> &directions) {
  size_t num_inside = 0;
  for (const auto &d : directions) {
    Ray r{p, d};
    auto hit = closest_hit(r, tree, tris);
    if (!hit.has_value()) continue;
    const Triangle &t = tris[hit->triangle_index];
    Vec3 n = t.calc_normal();
    if (n.dot(r.direction) > 0.0f) num_inside++;
  }
  return num_inside >= (directions.size() / 2);
}

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cerr << "Expected arguments: mesh.stl seed n output.ply" << std::endl;
    return 1;
  }
  const char *mesh_filepath = argv[1];
  uint32_t seed = std::stoul(argv[2]);
  long long num_points = std::stoll(argv[3]);
  const char *output_filepath = argv[4];
  std::optional<Mesh> mesh = read_mesh(mesh_filepath);
  if (!mesh.has_value()) {
    std::cerr << "Failed to load mesh" << std::endl;
    return 1;
  }
  std::cout << "Number of triangles: " << mesh->tris.size() << std::endl;

  const std::vector<Triangle> &tris = mesh->tris;

  if (tris.empty()) {
    std::cerr << "Empty mesh" << std::endl;
    return 1;
  }

  std::vector<AABB> aabbs;
  aabbs.reserve(tris.size());
  for (const Triangle &t : tris) aabbs.push_back(t.calc_aabb());

  // Build BVH tree
  BVH_Tree tree(aabbs);

  // Sample random points in AABB
  const AABB &aabb = tree.get_aabb();

  std::cout << "AABB min: " << aabb.min << std::endl;
  std::cout << "AABB max: " << aabb.max << std::endl;

  std::mt19937 prng_engine(seed);

  std::uniform_real_distribution<float> x_dist(aabb.min.x, aabb.max.x);
  std::uniform_real_distribution<float> y_dist(aabb.min.y, aabb.max.y);
  std::uniform_real_distribution<float> z_dist(aabb.min.z, aabb.max.z);

  std::vector<Vec3> points;
  points.reserve(num_points);

  for (long long i = 0; i < num_points; i++) {
    Vec3 p(x_dist(prng_engine), y_dist(prng_engine), z_dist(prng_engine));
    points.push_back(p);
  }

  std::uniform_real_distribution<float> f_dist(0.0f, 1.0f);
  // https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#fragment-SamplingFunctionDefinitions-5
  auto sample_full_sphere = [&] {
    float x = f_dist(prng_engine);
    float y = f_dist(prng_engine);
    float z = 1.0f - 2.0f * x;
    float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
    float phi = 2 * PI * y;
    return Vec3(r * std::cos(phi), r * std::sin(phi), z);
  };

  constexpr size_t num_directions = 2;
  std::vector<Vec3> directions;
  directions.reserve(num_directions);
  for (size_t i = 0; i < num_directions; i++) {
    directions.push_back(sample_full_sphere());
  }

  std::cout << "Filtering points..." << std::endl;
  auto t1 = std::chrono::high_resolution_clock::now();
  bool *is_in_volume = (bool *)malloc(num_points);
#pragma omp parallel for
  for (long long i = 0; i < num_points; i++) {
    is_in_volume[i] = is_point_in_volume_3(points[i], tree, tris, directions);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout
      << "Took "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
      << "ms" << std::endl;

  std::vector<Vec3> filtered_points;
  filtered_points.reserve(num_points);
  for (long long i = 0; i < num_points; i++) {
    if (is_in_volume[i]) filtered_points.push_back(points[i]);
  }
  filtered_points.shrink_to_fit();

  free(is_in_volume);

  std::cout << "Writing ply..." << std::endl;
  write_ply(filtered_points, output_filepath);
  std::cout << "Success" << std::endl;
  return 0;
}
