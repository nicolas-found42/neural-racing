// Actual RNW triangle cross-sections. No invented road widths.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
struct V {
  float x, y, z;
  int16_t index;
  uint8_t shade, grip;
};
struct T {
  uint16_t a, b, c;
  uint8_t type, flags;
  uint32_t raw;
};
struct C {
  float v[10];
  uint32_t index;
};
struct I {
  double lo, hi, yl, yh;
  uint32_t material;
  double y(double x) const {
    return hi == lo ? yl : yl + (yh - yl) * (x - lo) / (hi - lo);
  }
};
int main(int argc, char **argv) {
  if (argc != 4 && argc != 5 && argc != 6 && argc != 7)
    return 2;
  uint32_t mask = argc >= 5 ? uint32_t(std::stoul(argv[4])) : 0xffffffff;
  double clearance = argc >= 6 ? std::stod(argv[5]) : 1.;
  std::ifstream f(argv[1], std::ios::binary), g(argv[2], std::ios::binary);
  uint32_t nv, nt, nc;
  f.read((char *)&nv, 4);
  f.read((char *)&nt, 4);
  g.read((char *)&nc, 4);
  if (nv > 1000000 || nt > 1000000 || nc > 100000)
    return 3;
  std::vector<V> vs(nv);
  std::vector<T> ts(nt);
  std::vector<C> cs(nc);
  f.read((char *)vs.data(), nv * 16);
  f.read((char *)ts.data(), nt * 12);
  g.read((char *)cs.data(), nc * 44);
  if (!f || !g)
    return 4;
  std::ofstream o(argv[3], std::ios::binary);
  o.write((char *)&nc, 4);
  unsigned missing = 0;
  double sum = 0;
  double previous_height = 0;
  bool has_previous = false;
  for (auto c : cs) {
    if (argc >= 5 && has_previous)
      c.v[4] = previous_height;
    double dx = c.v[6] - c.v[0], dz = c.v[8] - c.v[2], len = std::hypot(dx, dz);
    std::vector<I> is;
    if (len > 0) {
      dx /= len;
      dz /= len;
      for (auto t : ts) {
        if (t.a >= nv || t.b >= nv || t.c >= nv)
          return 5;
        V vv[3] = {vs[t.a], vs[t.b], vs[t.c]};
        double u[3], v[3];
        for (int k = 0; k < 3; k++) {
          double x = vv[k].x - c.v[3], z = vv[k].z - c.v[5];
          u[k] = x * dx + z * dz;
          v[k] = -x * dz + z * dx;
        }
        if (*std::min_element(v, v + 3) > 0 || *std::max_element(v, v + 3) < 0)
          continue;
        std::vector<std::pair<double, double>> p;
        for (int k = 0; k < 3; k++) {
          int j = (k + 1) % 3;
          if (v[k] == 0)
            p.push_back({u[k], vv[k].y});
          if ((v[k] < 0 && v[j] > 0) || (v[k] > 0 && v[j] < 0)) {
            double a = v[k] / (v[k] - v[j]);
            p.push_back(
                {u[k] + a * (u[j] - u[k]), vv[k].y + a * (vv[j].y - vv[k].y)});
          }
        }
        if (p.size() < 2)
          continue;
        std::sort(p.begin(), p.end());
        auto a = p.front(), b = p.back();
        if (b.first - a.first < 1e-8)
          continue;
        is.push_back(
            {a.first, b.first, a.second, b.second, uint32_t(t.type & 31)});
      }
    }
    std::vector<int> parent(is.size());
    for (size_t i = 0; i < is.size(); i++)
      parent[i] = i;
    auto find = [&](int i) {
      while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
      }
      return i;
    };
    for (size_t i = 0; i < is.size(); i++)
      for (size_t j = 0; j < i; j++) {
        auto a = is[i], b = is[j];
        if (!(mask & (1u << a.material)) || !(mask & (1u << b.material)) ||
            (argc == 4 && a.material != b.material))
          continue;
        double lo = std::max(a.lo, b.lo), hi = std::min(a.hi, b.hi);
        if (lo > hi + 1e-4)
          continue;
        double t = (lo + hi) * .5;
        if (std::abs(a.y(t) - b.y(t)) < .002)
          parent[find(i)] = find(j);
      }
    std::vector<I> components;
    std::vector<std::vector<int>> members;
    for (size_t i = 0; i < is.size(); i++)
      if ((mask & (1u << is[i].material)) && find(i) == int(i)) {
        I cur = is[i];
        std::vector<int> mm;
        for (size_t j = 0; j < is.size(); j++)
          if (find(j) == int(i)) {
            mm.push_back(j);
            if (is[j].lo < cur.lo) {
              cur.lo = is[j].lo;
              cur.yl = is[j].yl;
            }
            if (is[j].hi > cur.hi) {
              cur.hi = is[j].hi;
              cur.yh = is[j].yh;
            }
          }
        components.push_back(cur);
        members.push_back(mm);
      }
    int best = -1;
    double error = 1e100;
    bool best_inside = false;
    for (size_t i = 0; i < components.size(); i++) {
      auto q = components[i];
      if (argc == 4 && !(q.lo <= 1e-5 && q.hi >= -1e-5))
        continue;
      double e = 1e100;
      for (int j : members[i]) {
        auto p = is[j];
        double t = std::clamp(0.0, p.lo, p.hi);
        e = std::min(e, std::hypot(t, p.y(t) - c.v[4]));
      }
      bool inside = q.lo <= 0 && q.hi >= 0;
      if ((inside && !best_inside) || (inside == best_inside && e < error)) {
        error = e;
        best = i;
        best_inside = inside;
      }
    }
    float left = std::numeric_limits<float>::quiet_NaN(), right = left;
    uint32_t mat = 0xffffffff;
    double center = 0, height = c.v[4];
    if (best >= 0) {
      auto cur = components[best];
      center =
          argc >= 5
              ? (argc == 7
                     ? (cur.lo + cur.hi) * .5
                     : std::clamp(
                           0.0,
                           cur.lo + std::min(clearance, (cur.hi - cur.lo) * .5),
                           cur.hi -
                               std::min(clearance, (cur.hi - cur.lo) * .5)))
              : 0;
      left = float(center - cur.lo);
      right = float(cur.hi - center);
      height = cur.y(center);
      double e = 1e100;
      for (int member : members[best]) {
        auto q = is[member];
        if (!(q.lo <= center + 1e-5 && q.hi >= center - 1e-5))
          continue;
        double qe = std::abs(q.y(center) - c.v[4]);
        if (qe < e) {
          e = qe;
          mat = q.material;
          height = q.y(center);
        }
      }
      sum += left + right;
    } else
      missing++;
    previous_height = height;
    has_previous = best >= 0;
    float record[6] = {c.v[9],
                       float(c.v[3] + center * dx),
                       float(-(c.v[5] + center * dz)),
                       float(height),
                       left,
                       right};
    o.write((char *)record, 24);
    o.write((char *)&c.index, 4);
    o.write((char *)&mat, 4);
  }
  std::cout << nc << " " << missing << " "
            << (nc > missing ? sum / (nc - missing) : 0) << "\n";
}
