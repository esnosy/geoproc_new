#include <fstream>

#include "endianness.hpp"
#include "write_ply.hpp"

void write_ply_header(std::ofstream &ofs, size_t num_points) {
  ofs << "ply\n";
  ofs << "format ";
  if (endian::native == endian::little) {
    ofs << "binary_little_endian";
  } else {
    ofs << "binary_big_endian";
  }
  ofs << " 1.0\n";
  ofs << "element vertex " << num_points << "\n";
  ofs << "property float x\n";
  ofs << "property float y\n";
  ofs << "property float z\n";
  ofs << "end_header\n";
}

void write_ply(const std::vector<Vec3> &points,
               const std::string &output_path) {
  std::ofstream ofs;
  ofs.exceptions(std::ios_base::badbit);
  ofs.open(output_path, std::ios_base::binary);
  write_ply_header(ofs, points.size());
  static_assert(sizeof(Vec3) == sizeof(float[3]));
  ofs.write((char *)points.data(), sizeof(Vec3) * points.size());
}
