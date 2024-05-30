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
#include "intersect.hpp"
#include "mesh_io.hpp"
#include "triangle.hpp"
#include "vec.hpp"
#include "write_ply.hpp"

struct BVH_Node {
  AABB aabb;
  uint32_t start, end;
  BVH_Node *right, *left;
  BVH_Node() {
    start = 0;
    end = 0;
    right = nullptr;
    left = nullptr;
  }

  uint32_t num_primitives() const {
    assert(end >= start);
    return end - start;
  }

  bool is_leaf() const { return left == nullptr && right == nullptr; }
};

class BVH_Tree {
private:
  BVH_Node *root = nullptr;
  std::vector<uint32_t> map;

  static AABB join_aabbs(const std::vector<AABB> &aabs) {
    AABB aabb = aabs[0];
    for (size_t i = 1; i < aabs.size(); i++) {
      aabb.min = Vec3::min(aabb.min, aabs[i].min);
      aabb.max = Vec3::max(aabb.max, aabs[i].max);
    }
    return aabb;
  }

  static AABB join_aabbs(const std::vector<AABB> &aabs, const std::vector<uint32_t> &map, uint32_t start, uint32_t end) {
    AABB aabb = aabs[map[start]];
    for (uint32_t i = start + 1; i < end; i++) {
      aabb.min = Vec3::min(aabb.min, aabs[map[i]].min);
      aabb.max = Vec3::max(aabb.max, aabs[map[i]].max);
    }
    return aabb;
  }

public:
  // Memory is free in destructor, avoid double free by disabling copy and move
  BVH_Tree(const BVH_Tree &) = delete;
  BVH_Tree(BVH_Tree &&) = delete;
  BVH_Tree &operator=(const BVH_Tree &) = delete;
  BVH_Tree &operator=(BVH_Tree &&) = delete;

  explicit BVH_Tree(const std::vector<AABB> &aabbs) {
    root = new BVH_Node;
    root->start = 0;
    root->end = aabbs.size();
    root->aabb = join_aabbs(aabbs);

    // Initialize indices map
    map.reserve(aabbs.size());
    for (uint32_t i = 0; i < aabbs.size(); i++) map.push_back(i);

    std::stack<BVH_Node *> stack;
    stack.push(root);
    while (!stack.empty()) {
      BVH_Node *node = stack.top();
      stack.pop();
      if (node->num_primitives() == 1) continue;
      Vec3 extent = node->aabb.calc_extent();
      uint8_t split_axis = 0;
      if (extent.y > extent.x) split_axis = 1;
      if (extent.z > extent[split_axis]) split_axis = 2;
      float split_pos = node->aabb.min[split_axis] * 0.5f + node->aabb.max[split_axis] * 0.5f;
      uint32_t i = node->start;
      uint32_t j = node->end;
      while (i < j) {
        if (aabbs[map[j - 1]].calc_center()[split_axis] > split_pos) j--;
        else std::swap(map[j - 1], map[i++]);
      }
      if (i == node->start || i == node->end) continue; // Partitioning failed

      BVH_Node *left = new BVH_Node;
      left->start = node->start;
      left->end = i;
      left->aabb = join_aabbs(aabbs, map, left->start, left->end);
      node->left = left;
      stack.push(left);

      BVH_Node *right = new BVH_Node;
      right->start = i;
      right->end = node->end;
      right->aabb = join_aabbs(aabbs, map, right->start, right->end);
      node->right = right;
      stack.push(right);
    }
  }
  // Remap an index stored in a BVH node to an valid index in the primitives array
  uint32_t remap_index(uint32_t i) const {
    assert(i < map.size());
    return map[i];
  }
  const BVH_Node *get_root() const { return root; }
  const AABB &get_aabb() const { return root->aabb; }

  ~BVH_Tree() {
    std::stack<BVH_Node *> stack;
    stack.push(root);
    while (!stack.empty()) {
      BVH_Node *node = stack.top();
      stack.pop();
      if (node->left) stack.push(node->left);
      if (node->right) stack.push(node->right);
      delete node;
    }
  }
};

static size_t count_intersections(const Ray &r, const BVH_Tree &tree, const std::vector<Triangle> &tris) {
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

static bool is_point_in_volume(const Vec3 &p, const BVH_Tree &tree, const std::vector<Triangle> &tris) {
  Ray r{p, Vec3(0, 0, 1)};
  size_t num_hits = count_intersections(r, tree, tris);
  return num_hits % 2 != 0;
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

  std::cout << "Filtering points..." << std::endl;
  auto t1 = std::chrono::high_resolution_clock::now();
  bool *is_in_volume = (bool *)malloc(num_points);
#pragma omp parallel for
  for (long long i = 0; i < num_points; i++) {
    is_in_volume[i] = is_point_in_volume(points[i], tree, tris);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "Took " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms" << std::endl;

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
}
