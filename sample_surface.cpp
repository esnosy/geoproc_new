#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "mesh_io.hpp"
#include "vec.hpp"
#include "write_ply.hpp"

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

  std::vector<float> triangle_areas;
  triangle_areas.reserve(mesh->tris.size());
  for (const Triangle &t : mesh->tris) {
    triangle_areas.push_back(t.area());
  }

  std::mt19937 prng_engine(seed);
  std::discrete_distribution<uint32_t> i_dist(triangle_areas.begin(), triangle_areas.end());
  auto get_rand_index = [&] { return i_dist(prng_engine); };

  std::uniform_real_distribution<float> u_dist(0.0f, 1.0f);
  std::uniform_real_distribution<float> v_dist(0.0f, 1.0f);

  auto get_rand_u = [&] { return u_dist(prng_engine); };
  auto get_rand_v = [&] { return v_dist(prng_engine); };

  // https://www.pbr-book.org/3ed-2018/Monte_Carlo_Integration/2D_Sampling_with_Multidimensional_Transformations#UniformSampleTriangle
  auto get_rand_bary = [&] {
    float u = get_rand_u();
    float v = get_rand_v();
    u = std::sqrt(u);
    return Vec2(1.0f - u, v * u);
  };

  std::vector<Vec3> points;
  points.reserve(num_points);

  for (size_t i = 0; i < num_points; i++) {
    const Triangle &t = mesh->tris[get_rand_index()];
    auto bary = get_rand_bary();
    Vec3 ab = t.b - t.a;
    Vec3 ac = t.c - t.a;
    Vec3 p = bary.x * ab + bary.y * ac + t.a;
    points.push_back(p);
  }
  write_ply(points, output_filepath);
  return 0;
}
