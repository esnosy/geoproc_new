#pragma once

#include <string>
#include <vector>

#include "vec.hpp"

void write_ply(const std::vector<Vec3> &points, const std::string &output_path);
