"""Tests for trailAccum.js trail accumulation logic via Node.js subprocess."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "utils",
))


def _strip_es_modules(src):
    """Remove ES module import/export syntax so Node.js can eval the code."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_JS_FILE = os.path.join(_UTILS_DIR, "trailAccum.js")
_TRAIL_JS = _strip_es_modules(
    open(_JS_FILE, encoding="utf-8").read()
)


def _run_js(script):
    code = _TRAIL_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestDistDeg2(unittest.TestCase):
    def test_basic(self):
        out = _run_js("""
            const d = distDeg2({ lon: 0, lat: 0 }, { lon: 3, lat: 4 });
            console.log(d);
        """)
        self.assertAlmostEqual(float(out), 25.0, places=5)

    def test_same_point(self):
        out = _run_js("""
            const d = distDeg2({ lon: 35, lat: 32 }, { lon: 35, lat: 32 });
            console.log(d);
        """)
        self.assertAlmostEqual(float(out), 0.0, places=10)


class TestAccumulate(unittest.TestCase):
    def test_first_point_added(self):
        out = _run_js("""
            const trail = [];
            accumulate(trail, { lon: 35, lat: 32, alt: 100 });
            console.log(trail.length);
        """)
        self.assertEqual(int(out), 1)

    def test_stationary_no_duplicates(self):
        out = _run_js("""
            const trail = [];
            const pt = { lon: 35, lat: 32, alt: 100 };
            accumulate(trail, pt);
            accumulate(trail, { lon: 35, lat: 32, alt: 100 });
            accumulate(trail, { lon: 35, lat: 32, alt: 100 });
            console.log(trail.length);
        """)
        self.assertEqual(int(out), 1)

    def test_moving_accumulates(self):
        out = _run_js("""
            const trail = [];
            accumulate(trail, { lon: 35.0, lat: 32.0, alt: 100 });
            accumulate(trail, { lon: 35.001, lat: 32.0, alt: 100 });
            accumulate(trail, { lon: 35.002, lat: 32.0, alt: 100 });
            console.log(trail.length);
        """)
        self.assertEqual(int(out), 3)

    def test_long_flight_keeps_start_and_stays_bounded(self):
        """A flight far longer than the cap keeps its first point and stays under the cap."""
        out = _run_js("""
            const trail = [];
            const N = MAX_TRAIL_LENGTH * 5;
            for (let i = 0; i < N; i++) {
                accumulate(trail, { lon: 35.0 + i * 0.001, lat: 32.0, alt: i });
            }
            console.log(JSON.stringify({
                len: trail.length, max: MAX_TRAIL_LENGTH,
                first: trail[0].alt, last: trail[trail.length - 1].alt, n: N,
            }));
        """)
        r = json.loads(out)
        self.assertLessEqual(r["len"], r["max"])
        self.assertEqual(r["first"], 0)
        self.assertEqual(r["last"], r["n"] - 1)

    def test_thin_keeps_recent_full_resolution(self):
        out = _run_js("""
            const trail = [];
            for (let i = 0; i < 1000; i++) trail.push({ lon: i, lat: 0, alt: i });
            thin(trail);
            const tail = trail.slice(-RECENT_FULL_RES).map(p => p.alt);
            console.log(JSON.stringify({
                len: trail.length, first: trail[0].alt, recent: RECENT_FULL_RES,
                tailContiguous: tail.every((a, k) => a === 1000 - RECENT_FULL_RES + k),
                ordered: trail.every((p, k) => k === 0 || p.alt > trail[k - 1].alt),
            }));
        """)
        r = json.loads(out)
        self.assertLessEqual(r["len"], (1000 - r["recent"]) // 2 + r["recent"])
        self.assertEqual(r["first"], 0)
        self.assertTrue(r["tailContiguous"])
        self.assertTrue(r["ordered"])

    def test_long_flight_keeps_every_loop(self):
        """Repeated thinning must not collapse an earlier loop that returns to its start."""
        out = _run_js("""
            const trail = [];
            const LOOP = 8224, N = 18000;
            for (let i = 0; i < N; i++) {
                const a = 2 * Math.PI * i / LOOP;
                accumulate(trail, { lon: 35 + 0.04 * Math.cos(a), lat: 32 + 0.04 * Math.sin(a), alt: i });
            }
            const firstLoop = trail.filter(p => p.alt < LOOP).length;
            let gap = 0;
            for (let k = 1; k < trail.length; k++) gap = Math.max(gap, Math.sqrt(distDeg2(trail[k - 1], trail[k])));
            console.log(JSON.stringify({ len: trail.length, max: MAX_TRAIL_LENGTH, firstLoop, gap }));
        """)
        r = json.loads(out)
        self.assertLessEqual(r["len"], r["max"])
        self.assertGreater(r["firstLoop"], 100)
        # Loop circumference ~0.25 deg; no kept gap may cut across it.
        self.assertLess(r["gap"], 0.01)

    def test_small_loop_survives_long_straight_leg(self):
        """A small early loop must survive a very long straight leg after it."""
        out = _run_js("""
            const trail = [];
            for (let i = 0; i < 150; i++) {
                const a = 2 * Math.PI * i / 150;
                accumulate(trail, { lon: 35 + 0.001 * Math.cos(a), lat: 32 + 0.001 * Math.sin(a), alt: i });
            }
            for (let i = 150; i < 36000; i++) {
                accumulate(trail, { lon: 35.001 + (i - 150) * 0.001, lat: 32, alt: i });
            }
            console.log(JSON.stringify({
                len: trail.length, max: MAX_TRAIL_LENGTH,
                loop: trail.filter(p => p.alt < 150).length,
            }));
        """)
        r = json.loads(out)
        self.assertLessEqual(r["len"], r["max"])
        self.assertGreater(r["loop"], 20)

    def test_thin_keeps_out_and_back_tip(self):
        out = _run_js("""
            const trail = [];
            let i = 0;
            for (let k = 0; k <= 400; k++) accumulate(trail, { lon: 35 + k * 3e-5, lat: 32, alt: 100, i: i++ });
            for (let k = 399; k >= 0; k--) accumulate(trail, { lon: 35 + k * 3e-5, lat: 32, alt: 100, i: i++ });
            for (let k = 1; k <= 1200; k++) accumulate(trail, { lon: 35 - k * 3e-5, lat: 32, alt: 100, i: i++ });
            console.log(JSON.stringify({ len: trail.length, maxLon: Math.max(...trail.map(p => p.lon)) }));
        """)
        r = json.loads(out)
        self.assertLess(r["len"], 2001)
        self.assertAlmostEqual(r["maxLon"], 35.012, places=6)

    def test_thin_keeps_climb_on_straight_track(self):
        out = _run_js("""
            const trail = [];
            for (let k = 0; k < 2001; k++) {
                const alt = k <= 200 ? 100 + k : k <= 400 ? 500 - k : 100;  // early 300 m hump
                accumulate(trail, { lon: 35 + k * 3e-5, lat: 32, alt });
            }
            console.log(JSON.stringify({ len: trail.length, maxAlt: Math.max(...trail.map(p => p.alt)) }));
        """)
        r = json.loads(out)
        self.assertLess(r["len"], 2001)
        self.assertAlmostEqual(r["maxAlt"], 300, places=6)

    def test_simplify_keeps_corner_drops_collinear(self):
        out = _run_js("""
            const pts = [];
            for (let i = 0; i <= 10; i++) pts.push({ lon: i, lat: 0 });
            for (let i = 1; i <= 10; i++) pts.push({ lon: 10, lat: i });
            const s = simplify(pts, 3);
            console.log(JSON.stringify(s.map(p => [p.lon, p.lat])));
        """)
        self.assertEqual(json.loads(out), [[0, 0], [10, 0], [10, 10]])

    def test_thin_short_trail_unchanged(self):
        out = _run_js("""
            const trail = [];
            for (let i = 0; i < RECENT_FULL_RES; i++) trail.push({ lon: i, lat: 0, alt: i });
            thin(trail);
            console.log(trail.length);
        """)
        self.assertEqual(int(out), 200)

    def test_sub_threshold_rejected(self):
        """Movement smaller than ~2m should not add a point."""
        out = _run_js("""
            const trail = [];
            accumulate(trail, { lon: 35.0, lat: 32.0, alt: 100 });
            accumulate(trail, { lon: 35.000005, lat: 32.0, alt: 100 });
            console.log(trail.length);
        """)
        self.assertEqual(int(out), 1)


if __name__ == "__main__":
    unittest.main()
