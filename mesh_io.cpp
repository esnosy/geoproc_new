#include <cstddef>
#include <fstream>
#include <ios>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

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

enum class PLY_Type {
  Int8,
  UInt8,
  Int16,
  UInt16,
  Int32,
  UInt32,
  Float32,
  Float64,
};

static std::optional<PLY_Type> str_to_ply_type(const std::string &s) {
  if (s == "char" || s == "int8") return PLY_Type::Int8;
  if (s == "uchar" || s == "uint8") return PLY_Type::UInt8;
  if (s == "short" || s == "int16") return PLY_Type::Int16;
  if (s == "ushort" || s == "uint16") return PLY_Type::UInt16;
  if (s == "int" || s == "int32") return PLY_Type::Int32;
  if (s == "uint" || s == "uint32") return PLY_Type::UInt32;
  if (s == "float" || s == "float32") return PLY_Type::Float32;
  if (s == "double" || s == "float64") return PLY_Type::Float64;
  return std::nullopt;
}

struct PLY_Property_Definition {
  std::string name;
  PLY_Type type;
  PLY_Type list_size_type;
  bool is_list = false;
};

struct PLY_Element_Definition {
  std::string name;
  size_t count = 0;
  std::vector<PLY_Property_Definition> property_defs;
};

static std::optional<Mesh> read_ply(std::string_view filepath) {
  Mesh mesh;
  std::vector<PLY_Element_Definition> element_defs;
  std::ifstream ifs;
  ifs.exceptions(std::ios_base::badbit);
  ifs.open(std::string(filepath), std::ios_base::binary);
  std::string signature;
  ifs >> signature;
  if (signature != "ply") std::cerr << "WARNING: Expected signature ply found " << signature << std::endl;
  std::string file_format, token;
  while (ifs.good()) {
    ifs >> token;
    if (token == "format") {
      ifs >> file_format;
      ifs >> token; // Skip file format version
    } else if (token == "comment") ifs.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    else if (token == "element") {
      PLY_Element_Definition element;
      ifs >> element.name >> element.count;
      element_defs.push_back(element);
    } else if (token == "property") {
      if (element_defs.empty()) {
        std::cerr << "ERROR: Found ply property before first element" << std::endl;
        return std::nullopt;
      }
      ifs >> token;
      PLY_Property_Definition pdef;
      if (token == "list") {
        pdef.is_list = true;
        ifs >> token;
        auto list_size_type = str_to_ply_type(token);
        if (!list_size_type.has_value()) {
          std::cerr << "ERROR: Unknown ply list size type " << token << std::endl;
          return std::nullopt;
        }
        pdef.list_size_type = list_size_type.value();
        ifs >> token; // Read type string of list values
      }
      auto type = str_to_ply_type(token);
      if (!type.has_value()) {
        std::cerr << "ERROR: Unknown ply type " << token << std::endl;
        return std::nullopt;
      }
      pdef.type = type.value();
      ifs >> pdef.name;
      std::cout << pdef.name << std::endl;
      element_defs.back().property_defs.push_back(pdef);
    } else if (token == "end_header") break;
  }

  std::cout << file_format << std::endl;

  if (file_format == "binary_little_endian") {
    std::cerr << "Unsupported ply format: binary_little_endian" << std::endl;
    return std::nullopt;
  }
  if (file_format == "binary_big_endian") {
    std::cerr << "Unsupported ply format: binary_big_endian" << std::endl;
    return std::nullopt;
  }
  if (file_format == "ascii") {
    for (const auto &ed : element_defs) {
      for (size_t i = 0; i < ed.count; i++) {
      }
    }
  }
  std::cerr << "ERROR: Unknown ply format " << file_format << std::endl;
  return std::nullopt;
}

std::optional<Mesh> read_mesh(std::string_view filepath) {
  if (ends_with(filepath, ".stl")) {
    return read_stl(filepath);
  } else if (ends_with(filepath, ".ply")) {
    return read_ply(filepath);
  }
  return std::nullopt;
}
