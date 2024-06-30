#include "bvh.hpp"
#include "intersect.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
#include <stack>
#include <string>
#include <string_view>
#include <vector>

#include "mesh_io.hpp"
#include "vec.hpp"
#include "write_ply.hpp"

static Mesh read_mesh_non_optional(std::string_view filepath) {
  std::optional<Mesh> mesh = read_mesh(filepath);
  if (!mesh.has_value()) {
    std::cerr << "Failed to load mesh from " << filepath << std::endl;
    std::exit(1);
  }
  return *mesh;
}

struct Indexed_Mesh {
public:
  struct Triangle {
    std::array<uint32_t, 3> vertices;
    std::array<uint32_t, 3> edges;
  };
  struct Edge {
    uint32_t a, b;
  };
  std::vector<Triangle> tris;
  std::vector<Edge> edges;
  std::vector<Vec3> vertices;

  explicit Indexed_Mesh(const Mesh &mesh) {
    for (const ::Triangle &t : mesh.tris)
      for (int i = 0; i < 3; i++) vertices.push_back(t[i]);

    for (uint32_t i = 0; i < mesh.tris.size(); i += 3) {
      edges.push_back({i, i + 1});
      edges.push_back({i + 1, i + 2});
      edges.push_back({i + 2, i});
      tris.push_back({{i, i + 1, i + 2}, {i, i + 1, i + 2}});
    }
  }
};

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Expected arguments: a.stl b.stl output.stl" << std::endl;
    return 1;
  }
  const char *a_filepath = argv[1];
  const char *b_filepath = argv[2];
  const char *output_filepath = argv[3];

  Mesh a = read_mesh_non_optional(a_filepath);
  Mesh b = read_mesh_non_optional(b_filepath);

  std::vector<AABB> aabs;
  aabs.reserve(b.tris.size());
  for (const Triangle &t : b.tris) aabs.push_back(t.calc_aabb());
  BVH_Tree bvh(aabs);

  for (const auto &a_t : a.tris) {
    std::stack<const BVH_Node *> stack;
    stack.push(bvh.get_root());
    while (!stack.empty()) {
      const BVH_Node *node = stack.top();
      stack.pop();
      if (!does_intersect(a_t, node->aabb)) continue;
      if (!node->is_leaf()) {
        stack.push(node->left);
        stack.push(node->right);
        continue;
      }
      // TODO: do triangle/triangle intersection
    }
  }
}
