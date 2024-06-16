#pragma once

#include <optional>
#include <vector>

#include "aabb.hpp"

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
  BVH_Node *root = nullptr;
  BVH_Node *nodes_buffer = nullptr;
  BVH_Node *current_free_node = nullptr;
  std::vector<uint32_t> map;

  static AABB join_aabbs(const std::vector<AABB> &aabs) {
    AABB aabb = aabs[0];
    for (size_t i = 1; i < aabs.size(); i++) {
      aabb.min = Vec3::min(aabb.min, aabs[i].min);
      aabb.max = Vec3::max(aabb.max, aabs[i].max);
    }
    return aabb;
  }

  static AABB join_aabbs(const std::vector<AABB> &aabs,
                         const std::vector<uint32_t> &map, uint32_t start,
                         uint32_t end) {
    AABB aabb = aabs[map[start]];
    for (uint32_t i = start + 1; i < end; i++) {
      aabb.min = Vec3::min(aabb.min, aabs[map[i]].min);
      aabb.max = Vec3::max(aabb.max, aabs[map[i]].max);
    }
    return aabb;
  }

  BVH_Node *new_node() {
    BVH_Node *result = current_free_node++;
    *result = BVH_Node();
    return result;
  }

public:
  // Memory is freed in destructor, avoid double free and freeing nullptr by
  // disabling copy and move
  BVH_Tree(const BVH_Tree &) = delete;
  BVH_Tree(BVH_Tree &&) = delete;
  BVH_Tree &operator=(const BVH_Tree &) = delete;
  BVH_Tree &operator=(BVH_Tree &&) = delete;

  explicit BVH_Tree(const std::vector<AABB> &aabbs);
  // Remap an index stored in a BVH node to an valid index in the primitives
  // array
  uint32_t remap_index(uint32_t i) const {
    assert(i < map.size());
    return map[i];
  }
  const BVH_Node *get_root() const { return root; }
  const AABB &get_aabb() const { return root->aabb; }

  ~BVH_Tree() { free(nodes_buffer); }
};

struct Closest_Point_Result {
  uint32_t i;
  float t;
};

std::optional<Closest_Point_Result>
closest_point(const Vec3 &p, const std::vector<Vec3> &points,
              const BVH_Tree &bvh);
