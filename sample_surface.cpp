#include <cmath>
#include <cstdint>
#include <fstream>
#include <ios>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <vector>

struct Vec3 {
  float x, y, z;
  explicit Vec3(float s) : x(s), y(s), z(s) {}
  Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
  Vec3 operator+(const Vec3 &v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
  Vec3 operator-(const Vec3 &v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
  Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
  friend Vec3 operator*(float s, const Vec3 &v) { return v * s; }
  Vec3 cross(const Vec3 &v) const { return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
  float mag() const { return std::sqrt(x * x + y * y + z * z); }
};

struct Vec2 {
  float x, y;
  explicit Vec2(float s) : x(s), y(s) {}
  Vec2(float x, float y) : x(x), y(y) {}
  Vec2 operator+(const Vec2 &v) const { return Vec2(x + v.x, y + v.y); }
  Vec2 operator-(const Vec2 &v) const { return Vec2(x - v.x, y - v.y); }
  Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
  friend Vec2 operator*(float s, const Vec2 &v) { return v * s; }
};

struct Triangle {
  Vec3 a, b, c;
  Triangle(const Vec3 &a, const Vec3 &b, const Vec3 &c) : a(a), b(b), c(c) {}
  float area() const { return (b - a).cross(c - a).mag() * 0.5f; }
};

struct Mesh {
  std::vector<Triangle> tris;
};

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

static std::optional<Mesh> read_mesh(std::string_view filepath) {
  if (ends_with(filepath, ".stl")) {
    return read_stl(filepath);
  }
  return std::nullopt;
}

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
  std::discrete_distribution<uint32_t> prng_dist(triangle_areas.begin(), triangle_areas.end());
  auto get_rand_index = [&] { return prng_dist(prng_engine); };

  std::uniform_real_distribution<float> u_dist(0.0f, 1.0f);
  std::uniform_real_distribution<float> v_dist(0.0f, 1.0f);

  auto get_rand_u = [&] { return u_dist(prng_engine); };
  auto get_rand_v = [&] { return v_dist(prng_engine); };

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

  std::ofstream ofs;
  ofs.exceptions(std::ios_base::badbit);
  ofs.open(output_filepath, std::ios_base::binary);
  ofs << "ply\n";
  // TODO: handle endianess
  ofs << "format binary_little_endian 1.0\n";
  ofs << "element vertex " << num_points << "\n";
  ofs << "property float x\n";
  ofs << "property float y\n";
  ofs << "property float z\n";
  ofs << "end_header\n";
  static_assert(sizeof(Vec3) == sizeof(float[3]));
  ofs.write((char *)points.data(), sizeof(Vec3) * points.size());

  return 0;
}
