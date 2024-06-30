#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <random>
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

int main(int argc, char **argv) {
  if (argc != 4) {
    std::cerr << "Expected arguments: source.stl target.stl output.stl" << std::endl;
    return 1;
  }
  const char *source_filepath = argv[1];
  const char *target_filepath = argv[2];
  const char *output_filepath = argv[3];

  Mesh source = read_mesh_non_optional(source_filepath);
  Mesh target = read_mesh_non_optional(target_filepath);
}
