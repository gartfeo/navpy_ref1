"""Regression tests for coverage strip placement (trackGenerator.js) and the
true polygon inset (geometry.js).

Guards two lane-placement bugs:

1. Wide-swath under-coverage: the survey used to inset the polygon by halfSwath
   AND start the first lane at +spacing/2 (a double cross-track margin), so for
   wide swaths coverage collapsed to a single centered strip and the area edges
   were never imaged — while the planner still reported the plan as valid.
2. Non-uniform lane grid: the follow-up fix placed the first lane halfSwath
   inside the border (an overlap-dependent dead margin at both borders) and
   pinned the last lane at pMax - halfSwath, leaving one narrower remainder gap
   between the last two routes. Lanes must instead tile the cross-track width
   into equal bands of pitch g <= spacing with a g/2 border standoff, which
   keeps coverage complete because spacing <= 2*halfSwath.
"""
import json
import os
import re
import subprocess
import tempfile
import unittest

_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..",
    "src", "gcs", "frontend", "src", "utils",
))

_JS_FILES = ["projection.js", "plannerConfig.js", "geometry.js",
             "clipping.js", "trackGenerator.js"]


def _strip_es_modules(src):
    out = []
    for line in src.split("\n"):
        s = line.strip()
        if s.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", s):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_JS = ""
for _f in _JS_FILES:
    _JS += _strip_es_modules(open(os.path.join(_UTILS_DIR, _f), encoding="utf-8").read()) + "\n"


def _run_node(script):
    # Write the (import/export-stripped) JS to a temp file and run `node <file>`
    # rather than `node -e <code>`: the concatenated payload can exceed Windows'
    # command-line length limit (WinError 206) when passed as an argument.
    fd, path = tempfile.mkstemp(suffix=".js")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(_JS + "\n" + script)
        result = subprocess.run(
            ["node", path], capture_output=True, text=True, timeout=10,
        )
    finally:
        os.remove(path)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TrackCoverageTest(unittest.TestCase):
    """generateStrips must fully cover the polygon perpendicular to the lanes."""

    def _coverage(self, half_extent_m, half_swath, spacing, angle=0):
        return _run_node(f"""
          const H = {half_extent_m};
          const poly = [[-H,-H],[H,-H],[H,H],[-H,H]];
          const strips = generateStrips(poly, {spacing}, {angle});
          const perpX = -Math.sin({angle}), perpY = Math.cos({angle});
          const offs = strips.map(st => {{
            const mx = (st[0][0] + st[1][0]) / 2, my = (st[0][1] + st[1][1]) / 2;
            return mx * perpX + my * perpY;
          }}).sort((a, b) => a - b);
          let maxGap = 0;
          for (let i = 1; i < offs.length; i++)
            maxGap = Math.max(maxGap, (offs[i] - {half_swath}) - (offs[i - 1] + {half_swath}));
          console.log(JSON.stringify({{
            strips: strips.length,
            offs,
            covMin: offs.length ? Math.min(...offs) - {half_swath} : null,
            covMax: offs.length ? Math.max(...offs) + {half_swath} : null,
            maxGap,
          }}));
        """)

    def test_wide_swath_covers_area_no_gap(self):
        # 2.44 km^2 square (H=781), wide halfSwath 500. Used to collapse to 1 strip
        # covering [-781, 219], leaving ~36% of the area unimaged.
        r = self._coverage(781, 500, 900)
        self.assertGreaterEqual(r["strips"], 2)
        self.assertLessEqual(r["covMin"], -781 + 1)      # near edge imaged
        self.assertGreaterEqual(r["covMax"], 781 - 1)    # far edge imaged
        self.assertLessEqual(r["maxGap"], 1.0)           # no cross-track gap

    def test_far_edge_covered_when_extent_not_multiple_of_spacing(self):
        r = self._coverage(1500, 500, 900)
        self.assertLessEqual(r["covMin"], -1500 + 1)
        self.assertGreaterEqual(r["covMax"], 1500 - 1)
        self.assertLessEqual(r["maxGap"], 1.0)

    def test_narrow_area_single_centered_lane_covers(self):
        # Area narrower than one swath — one centered lane still spans it.
        r = self._coverage(300, 500, 900)
        self.assertEqual(r["strips"], 1)
        self.assertLessEqual(r["covMin"], -300 + 1)
        self.assertGreaterEqual(r["covMax"], 300 - 1)

    def test_rotated_area_covers(self):
        r = self._coverage(781, 500, 900, angle=0.6)   # ~34 deg
        self.assertGreaterEqual(r["strips"], 2)
        self.assertLessEqual(r["covMin"], -781 + 2)
        self.assertGreaterEqual(r["covMax"], 781 - 2)
        self.assertLessEqual(r["maxGap"], 1.0)

    def test_route_gaps_all_equal_and_within_spacing(self):
        # W=1000, spacing=150: the old placement produced gaps
        # [150,150,150,150,150,84] — the last route pinned at pMax-halfSwath left
        # one narrow remainder pass. The tiling must give identical gaps <= spacing.
        r = self._coverage(500, 83, 150)
        offs = r["offs"]
        self.assertEqual(r["strips"], 7)
        gaps = [offs[i + 1] - offs[i] for i in range(len(offs) - 1)]
        self.assertLessEqual(max(gaps), 150 + 1e-6)
        self.assertAlmostEqual(max(gaps), min(gaps), delta=1e-6)
        # coverage still complete: g/2 <= halfSwath
        self.assertLessEqual(r["covMin"], -500)
        self.assertGreaterEqual(r["covMax"], 500)

    def test_border_standoff_is_half_gap_not_swath(self):
        # High overlap used to push the outer routes halfSwath (=2 route gaps)
        # inside the borders. The standoff must be exactly half the route gap,
        # symmetric on both borders, regardless of the camera swath.
        r = self._coverage(500, 83, 42)
        offs = r["offs"]
        gap = offs[1] - offs[0]
        self.assertAlmostEqual(offs[0] - (-500), gap / 2, delta=1e-6)
        self.assertAlmostEqual(500 - offs[-1], gap / 2, delta=1e-6)


class InsetPolygonTest(unittest.TestCase):
    """insetPolygon must be a true edge-normal offset, not a centroid scale."""

    def _inset(self, poly, m):
        return _run_node(f"console.log(JSON.stringify(insetPolygon({json.dumps(poly)}, {m})));")

    def test_true_offset_rectangle(self):
        # 3000x800 inset by 200 -> 2600 x 400 (centroid-scaling gave 2613 x 697).
        o = self._inset([[0, 0], [3000, 0], [3000, 800], [0, 800]], 200)
        xs = [p[0] for p in o]
        ys = [p[1] for p in o]
        self.assertAlmostEqual(max(xs) - min(xs), 2600, delta=1)
        self.assertAlmostEqual(max(ys) - min(ys), 400, delta=1)

    def test_square_inset_shrinks_uniformly(self):
        o = self._inset([[0, 0], [1000, 0], [1000, 1000], [0, 1000]], 100)
        xs = [p[0] for p in o]
        ys = [p[1] for p in o]
        self.assertAlmostEqual(max(xs) - min(xs), 800, delta=1)
        self.assertAlmostEqual(max(ys) - min(ys), 800, delta=1)

    def test_over_inset_returns_original(self):
        orig = [[0, 0], [100, 0], [100, 100], [0, 100]]
        self.assertEqual(self._inset(orig, 200), orig)


if __name__ == "__main__":
    unittest.main()
