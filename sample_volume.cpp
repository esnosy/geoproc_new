#include <cstddef>
#include <cstdint>
#include <iostream>
#include <random>
#include <stack>
#include <string>
#include <vector>

#include "mesh_io.hpp"
#include "vec.hpp"
#include "write_ply.hpp"

struct BVH_Node {
  AABB aabb;
  uint32_t start, end;
  BVH_Node *right, *left;
};

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cerr << "Expected arguments: mesh.stl seed n output.ply" << std::endl;
    return 1;
  }
  const char *mesh_filepath = argv[1];
  uint32_t seed = std::stoul(argv[2]);
  size_t num_points = std::stoul(argv[3]);
  const char *output_filepath = argv[4];
  std::optional<Mesh> mesh = read_mesh(mesh_filepath);
  if (!mesh.has_value()) {
    std::cerr << "Failed to load mesh" << std::endl;
    return 1;
  }
  std::cout << "Number of triangles: " << mesh->tris.size() << std::endl;

  const std::vector<Triangle> &tris = mesh->tris;

  AABB aabb = tris[0].calc_aabb();

  for (size_t i = 1; i < tris.size(); i++) {
    for (size_t j = 0; j < 3; j++) {
      aabb.min = Vec3::min(aabb.min, tris[i][j]);
      aabb.max = Vec3::max(aabb.max, tris[i][j]);
    }
  }

  std::cout << "AABB min: " << aabb.min << std::endl;
  std::cout << "AABB max: " << aabb.max << std::endl;

  std::mt19937 prng_engine(seed);

  std::uniform_real_distribution<float> x_dist(aabb.min.x, aabb.max.x);
  std::uniform_real_distribution<float> y_dist(aabb.min.y, aabb.max.y);
  std::uniform_real_distribution<float> z_dist(aabb.min.z, aabb.max.z);

  std::vector<Vec3> points;
  points.reserve(num_points);

  for (size_t i = 0; i < num_points; i++) {
    Vec3 p(x_dist(prng_engine), y_dist(prng_engine), z_dist(prng_engine));
    points.push_back(p);
  }

  // BVH_Node *root = new BVH_Node;
  // TODO: build BVH and filter points

  write_ply(points, output_filepath);
}
