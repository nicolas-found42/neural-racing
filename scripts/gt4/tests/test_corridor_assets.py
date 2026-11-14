"""Regressions for source scale and aligned explicit edges in installed payloads."""

import itertools
import math
import struct
import unittest
from pathlib import Path

ASSETS = (
    Path(__file__).resolve().parents[3]
    / "crates/neural_racing_core/src/circuit_data/gt4"
)


def rows(path, fmt, stride):
    data = path.read_bytes()
    count = struct.unpack_from("<I", data)[0]
    if len(data) != 4 + count * stride:
        raise ValueError(path)
    return [struct.unpack_from(fmt, data, 4 + i * stride) for i in range(count)]


class SourceGeometry(unittest.TestCase):
    def test_suzuka_start_uses_native_physical_boundary(self):
        q = rows(ASSETS / "suzuka.stations", "<6fII", 32)[0]
        # Original measured blocker Z=-505.96973 and ground edge Z=-521.94336.
        # The former generated ribbon incorrectly merged35.77675m here.
        self.assertAlmostEqual(q[4] + q[5], 15.97407, delta=0.005)
        edge = rows(ASSETS / "suzuka.edges", "<4f", 16)[0]
        self.assertGreater(edge[1], q[2])  # core LEFT at this +X-facing station
        self.assertLess(edge[3], q[2])

    def test_nurburgring_start_has_no_independent_projection_jump(self):
        q = rows(ASSETS / "nurburgring.stations", "<6fII", 32)[:10]
        self.assertLess(
            max(math.dist(a[1:3], b[1:3]) for a, b in itertools.pairwise(q)), 3.0
        )

    def test_all_95_station_edge_pairs_preserve_source_frame(self):
        files = sorted(ASSETS.glob("*.edges"))
        self.assertEqual(len(files), 95)
        for path in files:
            stations = rows(path.with_suffix(".stations"), "<6fII", 32)
            edges = rows(path, "<4f", 16)
            self.assertEqual(len(stations), len(edges), path.stem)
            for i, (q, e) in enumerate(zip(stations, edges)):
                with self.subTest(course=path.stem, row=i):
                    scale = max(1.0, *(abs(x) for x in (*q[1:3], *e)))
                    # Quantization of original f32 world-coordinate endpoints.
                    tolerance = 4 * math.sqrt(2) * 2 ** (math.frexp(scale)[1] - 24)
                    self.assertLessEqual(
                        abs(math.dist(q[1:3], e[:2]) - q[4]), tolerance
                    )
                    self.assertLessEqual(
                        abs(math.dist(q[1:3], e[2:]) - q[5]), tolerance
                    )


if __name__ == "__main__":
    unittest.main()
