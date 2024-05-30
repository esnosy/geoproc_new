#pragma once

#include <cassert>
#include <optional>
#include <string_view>
#include <vector>

#include "triangle.hpp"

struct Mesh {
  std::vector<Triangle> tris;
};

std::optional<Mesh> read_mesh(std::string_view filepath);
