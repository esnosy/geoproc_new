#pragma once

#include <string>
#include <vector>

#include "vec.hpp"

void write_ply_header(std::ofstream &ofs, size_t num_points);
void write_ply(const std::vector<Vec3> &points, const std::string &output_path);
