// Physical source-ground cross-sections, not painted racing-strip boundaries.
// Boundary spans/closure are original 0x3e6a58; outward direction is original
// callback 0x45d550, independently executed on Play! EE. Surface grouping is
// original native class: IDs 2/3/4/6 are class 1, all others class 0.
// Navigation margin never changes the measured source edge positions.
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
  uint32_t material, flags, triangle;
  double y(double x) const {
    return hi == lo ? yl : yl + (yh - yl) * (x - lo) / (hi - lo);
  }
};
static_assert(sizeof(V) == 16 && sizeof(T) == 12 && sizeof(C) == 44);
int main(int argc, char **argv) {
  const uint16_t endian_probe = 1;
  if (*reinterpret_cast<const uint8_t *>(&endian_probe) != 1)
    return 12;
  if (argc != 6)
    return 2;
  constexpr double mirror_half_width = 1.914 / 2;
  constexpr double navigation_margin = .10;
  constexpr double clearance = mirror_half_width + navigation_margin;
  int preferred = std::stoi(argv[5]);
  auto native_class = [](unsigned m) {
    return m == 2 || m == 3 || m == 4 || m == 6;
  };
  std::ifstream bf(argv[4], std::ios::binary);
  uint32_t nb, ni;
  bf.read((char *)&nb, 4);
  bf.read((char *)&ni, 4);
  if (!bf || nb > 1000000 || ni > 100000000)
    return 8;
  std::vector<V> bounds(nb);
  std::vector<uint16_t> indices(ni);
  bf.read((char *)bounds.data(), nb * 16);
  bf.read((char *)indices.data(), ni * 2);
  if (!bf)
    return 8;
  std::ofstream edges(std::string(argv[3]) + ".edges", std::ios::binary);
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
  edges.write((char *)&nc, 4);
  unsigned missing = 0;
  double sum = 0;

  for (auto c : cs) {

    double dx = c.v[6] - c.v[0], dz = c.v[8] - c.v[2], len = std::hypot(dx, dz);
    std::vector<I> is;
    if (len > 0) {
      dx /= len;
      dz /= len;
      for (size_t ti = 0; ti < ts.size(); ti++) {
        auto t = ts[ti];
        if (t.a >= nv || t.b >= nv || t.c >= nv)
          return 5;
        V vv[3] = {vs[t.a], vs[t.b], vs[t.c]};
        const double ny =
            (double(vv[1].z) - vv[0].z) * (double(vv[2].x) - vv[0].x) -
            (double(vv[1].x) - vv[0].x) * (double(vv[2].z) - vv[0].z);
        if (ny <= 1e-10)
          continue; // Original front-facing ground query, not ceilings.

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
        is.push_back({a.first, b.first, a.second, b.second,
                      uint32_t(t.type & 31), t.flags, uint32_t(ti)});
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
    std::vector<int> deck(is.size());
    for (size_t i = 0; i < deck.size(); i++)
      deck[i] = i;
    auto deck_find = [&](int i) {
      while (deck[i] != i) {
        deck[i] = deck[deck[i]];
        i = deck[i];
      }
      return i;
    };
    for (size_t i = 0; i < is.size(); i++)
      for (size_t j = 0; j < i; j++) {
        auto a = is[i], b = is[j];
        double lo = std::max(a.lo, b.lo), hi = std::min(a.hi, b.hi);
        if (lo > hi + 1e-4)
          continue;
        double t = (lo + hi) * .5;
        if (std::abs(a.y(t) - b.y(t)) >= .002)
          continue;
        deck[deck_find(i)] = deck_find(j);
        if (native_class(a.material) == native_class(b.material))
          parent[find(i)] = find(j);
      }
    std::vector<I> components;
    std::vector<std::vector<int>> members;
    for (size_t i = 0; i < is.size(); i++)
      if (find(i) == int(i)) {
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
    auto clip_component = [&](size_t component) {
      auto cur = components[component];
      std::vector<uint16_t> candidates;
      int selected_deck = deck_find(members[component].front());
      for (size_t member = 0; member < is.size(); member++) {
        if (deck_find(member) != selected_deck)
          continue;
        auto q = is[member];

        auto word = ts[q.triangle].raw;
        uint32_t start = word & 0xffffff, n = word >> 24;
        if (start + n > indices.size())
          throw std::runtime_error("boundary span");
        candidates.insert(candidates.end(), indices.begin() + start,
                          indices.begin() + start + n);
      }
      for (auto bi : candidates) {
        if (bi >= bounds.size())
          throw std::runtime_error("boundary index");
        auto a = bounds[bi];
        if ((unsigned(a.shade) | (unsigned(a.grip) << 8)) > 1)
          throw std::runtime_error("unsupported boundary flag");
        int next = int(bi) + 1;
        if (a.index < 0)
          next += a.index;
        if (next < 0 || next >= int(bounds.size()))
          throw std::runtime_error("boundary closure");
        auto b = bounds[next];
        double ex = b.x - a.x, ez = b.z - a.z;
        double cross0 = ex * (c.v[5] - a.z) - ez * (c.v[3] - a.x);
        double slope = ex * dz - ez * dx;
        if (cross0 >= 0 || std::abs(slope) < 1e-12)
          continue;
        double u = -cross0 / slope;
        double px = c.v[3] + u * dx, pz = c.v[5] + u * dz;
        double ee = ex * ex + ez * ez;
        if (ee < 1e-12)
          continue;
        double along = ((px - a.x) * ex + (pz - a.z) * ez) / ee;
        if (along < 0 || along > 1)
          continue;
        if (u > 0)
          cur.hi = std::min(cur.hi, u);
        else
          cur.lo = std::max(cur.lo, u);
      }
      return cur;
    };
    for (size_t i = 0; i < components.size(); i++)
      components[i] = clip_component(i);
    int best = -1;
    double error = 1e100;
    bool best_inside = false, best_fits = false;
    for (size_t i = 0; i < components.size(); i++) {
      auto q = components[i];
      if (q.hi <= q.lo || int(native_class(q.material)) != preferred)
        continue;
      double e = 1e100;
      for (int j : members[i]) {
        auto p = is[j];
        double t = std::clamp(0.0, p.lo, p.hi);
        e = std::min(e, std::hypot(t, p.y(t) - c.v[4]));
      }
      bool inside = q.lo <= 0 && q.hi >= 0;
      bool fits = q.hi - q.lo >= (2 * clearance);
      if ((fits && !best_fits) ||
          (fits == best_fits && ((inside && !best_inside) ||
                                 (inside == best_inside && e < error)))) {
        error = e;
        best = i;
        best_inside = inside;
        best_fits = fits;
      }
    }
    float left = std::numeric_limits<float>::quiet_NaN(), right = left;
    uint32_t mat = 0xffffffff;
    double center = 0, height = c.v[4];
    if (best >= 0) {
      auto cur = components[best];

      // Move the authored reference only enough for the measured car footprint.
      // Do not use apron midpoints: they introduce unrelated lateral jumps.
      center = 0;
      center =
          std::clamp(0.0, cur.lo + std::min(clearance, (cur.hi - cur.lo) * .5),
                     cur.hi - std::min(clearance, (cur.hi - cur.lo) * .5));
      left = float(cur.hi - center);
      right = float(center - cur.lo);
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

    float record[6] = {c.v[9],
                       float(c.v[3] + center * dx),
                       float(-(c.v[5] + center * dz)),
                       float(height),
                       left,
                       right};
    o.write((char *)record, 24);
    o.write((char *)&c.index, 4);
    o.write((char *)&mat, 4);
    float ep[4] = {float(c.v[3] + (center + left) * dx),
                   float(-(c.v[5] + (center + left) * dz)),
                   float(c.v[3] + (center - right) * dx),
                   float(-(c.v[5] + (center - right) * dz))};
    edges.write((char *)ep, 16);
  }
  std::cout << nc << " " << missing << " "
            << (nc > missing ? sum / (nc - missing) : 0) << "\n";
}
