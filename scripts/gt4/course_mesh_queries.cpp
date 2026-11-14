#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <vector>
struct V {
  float x, y, z;
  int16_t i;
  uint8_t a, b;
};
struct T {
  uint16_t a, b, c;
  uint8_t m, f;
  uint32_t q;
};
struct G {
  float p[10];
  uint32_t i;
};
struct Q {
  uint32_t index, tri, material;
  float x, y, z;
};
int main(int argc, char **argv) {
  std::ifstream f(argv[1], std::ios::binary), g(argv[2], std::ios::binary);
  std::ofstream o(argv[3], std::ios::binary);
  uint32_t nv, nt, n, count = 0;
  f.read((char *)&nv, 4);
  f.read((char *)&nt, 4);
  g.read((char *)&n, 4);
  std::vector<V> vs(nv);
  std::vector<T> ts(nt);
  std::vector<G> gs(n);
  f.read((char *)vs.data(), nv * 16);
  f.read((char *)ts.data(), nt * 12);
  g.read((char *)gs.data(), n * 44);
  if (!f || !g)
    return 2;
  o.write((char *)&count, 4);
  for (auto gate : gs) {
    double x = gate.p[3], z = gate.p[5];
    if (!std::isfinite(x) || !std::isfinite(z))
      continue;
    std::vector<double> heights;
    for (unsigned i = 0; i < nt; i++) {
      auto tri = ts[i];
      auto A = vs[tri.a], B = vs[tri.b], C = vs[tri.c];
      if (x < std::min({A.x, B.x, C.x}) || x > std::max({A.x, B.x, C.x}) ||
          z < std::min({A.z, B.z, C.z}) || z > std::max({A.z, B.z, C.z}))
        continue;
      double den = (B.z - C.z) * (A.x - C.x) + (C.x - B.x) * (A.z - C.z);
      if (std::abs(den) < 1e-12)
        continue;
      double u = ((B.z - C.z) * (x - C.x) + (C.x - B.x) * (z - C.z)) / den,
             v = ((C.z - A.z) * (x - C.x) + (A.x - C.x) * (z - C.z)) / den,
             w = 1 - u - v;
      if (std::min({u, v, w}) < -.000001)
        continue;
      double y = u * A.y + v * B.y + w * C.y;
      if (!std::isfinite(y))
        continue;
      bool dup = false;
      for (auto h : heights)
        if (std::abs(h - y) < .0001)
          dup = true;
      if (dup)
        continue;
      heights.push_back(y);
      Q q{gate.i, i, uint32_t(tri.m & 31), float(x), float(y), float(z)};
      o.write((char *)&q, 24);
      count++;
    }
  }
  o.seekp(0);
  o.write((char *)&count, 4);
}
