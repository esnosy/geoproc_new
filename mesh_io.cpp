#include <fstream>
#include <iostream>

#include "mesh_io.hpp"

static bool ends_with(std::string_view str, std::string_view suffix) {
  if (suffix.size() == 0) return true;
  if (str.size() < suffix.size()) {
    return false;
  }
  for (size_t i = 0; i < suffix.size(); i++) {
    if (str[str.size() - suffix.size() + i] != suffix[i]) return false;
  }
  return true;
}

static Mesh read_stl_binary(std::ifstream &ifs, uint32_t num_tris) {
  Mesh mesh;
  mesh.tris.reserve(num_tris);
  for (uint32_t i = 0; i < num_tris; i++) {
    // Skip normal
    ifs.seekg(sizeof(float[3]), std::ios_base::cur);
    Triangle t(Vec3(0.0f), Vec3(0.0f), Vec3(0.0f));
    static_assert(sizeof(Triangle) == sizeof(float[3][3]));
    // TODO: handle endianness
    ifs.read((char *)&t, sizeof(float[3][3]));
    // Skip "attribute byte count"
    ifs.seekg(sizeof(uint16_t), std::ios_base::cur);

    mesh.tris.push_back(t);
  }
  return mesh;
}

static std::optional<Mesh> read_stl(std::string_view filepath) {
  Mesh mesh;
  std::ifstream ifs;
  ifs.exceptions(std::ios_base::badbit);
  ifs.open(std::string(filepath), std::ios_base::binary);
  // Assume binary .stl file
  ifs.seekg(80, std::ios_base::beg); // Skip binary header

  uint32_t num_tris = 0;
  ifs.read((char *)&num_tris, sizeof(uint32_t));

  size_t expected_tris_size = num_tris * 50;

  auto tris_begin = ifs.tellg();
  ifs.seekg(0, std::ios_base::end);
  size_t actual_tris_size = (ifs.tellg() - tris_begin);

  if (actual_tris_size == expected_tris_size) {
    ifs.seekg(tris_begin);
    return read_stl_binary(ifs, num_tris);
  } else {
    ifs.seekg(0, std::ios_base::beg);
    std::cerr << "Unsupported file format: ASCII STL" << std::endl;
    return std::nullopt;
  }
}

std::optional<Mesh> read_mesh(std::string_view filepath) {
  if (ends_with(filepath, ".stl")) {
    return read_stl(filepath);
  }
  return std::nullopt;
}
