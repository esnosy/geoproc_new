#include <stack>

#include "bvh.hpp"
#include "distance.hpp"

BVH_Tree::BVH_Tree(const std::vector<AABB> &aabbs) {
  assert(!aabbs.empty());
  // Number of nodes of full binary tree with n leaves is 2n - 1
  nodes_buffer = (BVH_Node *)malloc(sizeof(BVH_Node) * (aabbs.size() * 2 - 1));
  current_free_node = nodes_buffer;

  root = new_node();
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
    float split_pos =
        node->aabb.min[split_axis] * 0.5f + node->aabb.max[split_axis] * 0.5f;
    uint32_t i = node->start;
    uint32_t j = node->end;
    while (i < j) {
      if (aabbs[map[j - 1]].calc_center()[split_axis] > split_pos) j--;
      else std::swap(map[j - 1], map[i++]);
    }
    if (i == node->start || i == node->end) continue; // Partitioning failed

    BVH_Node *left = new_node();
    left->start = node->start;
    left->end = i;
    left->aabb = join_aabbs(aabbs, map, left->start, left->end);
    node->left = left;
    stack.push(left);

    BVH_Node *right = new_node();
    right->start = i;
    right->end = node->end;
    right->aabb = join_aabbs(aabbs, map, right->start, right->end);
    node->right = right;
    stack.push(right);
  }
}

std::optional<Closest_Point_Result>
closest_point(const Vec3 &p, const std::vector<Vec3> &points,
              const BVH_Tree &bvh) {
  std::optional<Closest_Point_Result> result;
  std::stack<const BVH_Node *> stack;
  stack.push(bvh.get_root());
  while (!stack.empty()) {
    const BVH_Node *node = stack.top();
    stack.pop();
    if (!node->is_leaf()) {
      // Pick closest AABB
      float ld = distance_to_volume(p, node->left->aabb);
      float rd = distance_to_volume(p, node->right->aabb);
      stack.push((ld < rd) ? node->left : node->right);
      continue;
    }
    for (uint32_t i = node->start; i < node->end; i++) {
      uint32_t pi = bvh.remap_index(i);
      const Vec3 &other_p = points[pi];
      float t = p.dist(other_p);
      if (!result.has_value() || t < result->t) result = {pi, t};
    }
  }
  if (!result.has_value()) return std::nullopt;

  // Second pass to ensure closest point
  stack.push(bvh.get_root());
  while (!stack.empty()) {
    const BVH_Node *node = stack.top();
    stack.pop();
    if (!node->is_leaf()) {
      float ld = distance_to_volume(p, node->left->aabb);
      float rd = distance_to_volume(p, node->right->aabb);
      if (ld < result->t) distance_to_volume(p, node->left->aabb);
      if (rd < result->t) distance_to_volume(p, node->right->aabb);
      continue;
    }
    for (uint32_t i = node->start; i < node->end; i++) {
      uint32_t pi = bvh.remap_index(i);
      const Vec3 &other_p = points[pi];
      float t = p.dist(other_p);
      if (t < result->t) result = {pi, t};
    }
  }

  return result;
}
