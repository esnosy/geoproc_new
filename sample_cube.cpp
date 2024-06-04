#include <cstdint>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "vec.hpp"
#include "write_ply.hpp"

int main() {
  std::string output_path = "points.ply";
  constexpr uint64_t seed = 1234;
  std::mt19937_64 e(seed);
  std::uniform_real_distribution<float> d(0.0f, 1.0f);
  constexpr uint64_t buffer_size = 1000'000;
  constexpr uint64_t num_buffers = 1300;
  std::vector<Vec3> buffer;
  buffer.reserve(buffer_size);
  std::ofstream ofs;
  ofs.exceptions(std::ios_base::badbit);
  ofs.open(output_path, std::ios_base::binary);
  write_ply_header(ofs, buffer_size * num_buffers);
  for (uint64_t i = 0; i < num_buffers; i++) {
    for (uint64_t j = 0; j < buffer_size; j++) {
      Vec3 p(d(e), d(e), d(e));
      buffer.push_back(p);
    }
    ofs.write((char *)buffer.data(), buffer.size() * sizeof(Vec3));
    buffer.clear();
  }
  return 0;
}
