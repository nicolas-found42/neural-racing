#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
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
struct S {
  float v, x, y, z, l, r;
  uint32_t i, m;
};
int main(int argc, char **argv) {
  std::ifstream f(argv[1], std::ios::binary), g(argv[2], std::ios::binary);
  uint32_t nv, nt, n;
  f.read((char *)&nv, 4);
  f.read((char *)&nt, 4);
  g.read((char *)&n, 4);
  std::vector<V> vs(nv);
  std::vector<T> ts(nt);
  std::vector<S> ss(n);
  f.read((char *)vs.data(), nv * 16);
  f.read((char *)ts.data(), nt * 12);
  g.read((char *)ss.data(), n * 32);
  if (!f || !g)
    return 2;
  std::ofstream diag;
  if (argc > 4)
    diag.open(argv[4]);
  diag << std::setprecision(17);
  uint32_t mask = std::stoul(argv[3]);
  unsigned missing = 0, total = 0, footmissing = 0;
  double maxerr = 0;
  for (unsigned i = 1; i < n; i++) {
    S a = ss[i - 1], b = ss[i];
    double len = std::hypot(b.x - a.x, b.y - a.y);
    int steps = std::max(1, int(std::ceil(len / 1.)));
    for (int k = 0; k <= steps; k++) {
      double t = double(k) / steps, x = a.x + t * (b.x - a.x),
             z = -(a.y + t * (b.y - a.y)), y = a.z + t * (b.z - a.z),
             dx = len ? (b.x - a.x) / len : 1,
             dz = len ? -(b.y - a.y) / len : 0;
      for (int side = -1; side <= 1; side++) {
        double px = x - side * dz * .95, pz = z + side * dx * .95, best = 1e100;
        for (auto tri : ts) {
          if (!(mask & (1u << (tri.m & 31))))
            continue;
          V A = vs[tri.a], B = vs[tri.b], C = vs[tri.c];
          if (px < std::min({A.x, B.x, C.x}) ||
              px > std::max({A.x, B.x, C.x}) ||
              pz < std::min({A.z, B.z, C.z}) || pz > std::max({A.z, B.z, C.z}))
            continue;
          double den = (B.z - C.z) * (A.x - C.x) + (C.x - B.x) * (A.z - C.z);
          if (std::abs(den) < 1e-12)
            continue;
          double u = ((B.z - C.z) * (px - C.x) + (C.x - B.x) * (pz - C.z)) /
                     den,
                 v = ((C.z - A.z) * (px - C.x) + (A.x - C.x) * (pz - C.z)) /
                     den,
                 w = 1 - u - v;
          if (u < -.00001 || v < -.00001 || w < -.00001)
            continue;
          best = std::min(best, std::abs(u * A.y + v * B.y + w * C.y - y));
        }
        if (best > 2 && diag)
          diag << i << "," << side << "," << px << "," << y << "," << pz << ","
               << (a.v + t * (b.v - a.v)) << "," << best << "\n";
        if (side == 0) {
          total++;
          if (best > 2)
            missing++;
          if (best < 1e90)
            maxerr = std::max(maxerr, best);
        } else if (best > 2)
          footmissing++;
      }
    }
  }
  std::cout << total << " " << missing << " " << footmissing << " " << maxerr
            << "\n";
}
