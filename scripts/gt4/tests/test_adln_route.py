"""Original EE outputs, including controlled straight/positive/negative arcs."""

import json
import math
import sys
import unittest
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import itertools

from adln_route import sample


class OriginalSampler(unittest.TestCase):
    def test_original_289_outputs(self):
        fixture = json.loads((SCRIPTS / "fixtures/adln_geometry.json").read_text())
        self.assertEqual(len(fixture["cases"]), 289)
        for case in fixture["cases"]:
            with self.subTest(
                course=case["course"], route=case["type"], v=case["native_v"]
            ):
                route = fixture["routes"][f"{case['course']}:{case['type']}"]
                point = sample(route["rows"], route["length"], case["native_v"])
                self.assertEqual(point[-1], case["native_segment"])
                self.assertLessEqual(math.dist(point[:3], case["native_xyz"]), 0.0005)

    def test_authored_nurburgring_start_has_no_projection_spike(self):
        fixture = json.loads((SCRIPTS / "fixtures/adln_geometry.json").read_text())
        route = fixture["routes"]["nurburgring:6"]
        points = [
            sample(route["rows"], route["length"], float(v)) for v in range(0, 20, 2)
        ]
        self.assertLess(
            max(math.dist(a[:2], b[:2]) for a, b in itertools.pairwise(points)), 2.5
        )


if __name__ == "__main__":
    unittest.main()
