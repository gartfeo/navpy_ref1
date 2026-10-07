"""Tests for pitchOptimizer.js — pitch+altitude optimizer via Node subprocess."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re
import math


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))


def _strip_es_modules(src):
    """Remove ES module import/export syntax so Node.js can eval the code."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}", stripped):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_JS_CODE = _strip_es_modules(
    open(os.path.join(_UTILS_DIR, "pitchOptimizer.js"), encoding="utf-8").read()
)


def _run_js(script):
    """Run a JS snippet via Node.js and return parsed JSON output."""
    full = _JS_CODE + "\n" + script
    result = run_node(full, timeout=30)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestComputeFootprintCorners(unittest.TestCase):
    """computeFootprintCorners projects camera FOV to ground plane."""

    def test_returns_4_corners(self):
        result = _run_js("""
            const pts = computeFootprintCorners(-35, 0.42, 0.24, 3000, 200);
            console.log(JSON.stringify(pts));
        """)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 4)
        for pt in result:
            self.assertEqual(len(pt), 2)

    def test_returns_null_for_upward_camera(self):
        result = _run_js("""
            const pts = computeFootprintCorners(30, 0.2, 0.1, 3000, 200);
            console.log(JSON.stringify(pts));
        """)
        self.assertIsNone(result)

    def test_steeper_pitch_closer_footprint(self):
        """Steeper downward pitch produces a footprint closer to nadir."""
        result = _run_js("""
            const fp1 = computeFootprintCorners(-20, 0.42, 0.24, 3000, 200);
            const fp2 = computeFootprintCorners(-60, 0.42, 0.24, 3000, 200);
            const cx1 = fp1.reduce((s, p) => s + p[0], 0) / 4;
            const cx2 = fp2.reduce((s, p) => s + p[0], 0) / 4;
            console.log(JSON.stringify({ cx1, cx2 }));
        """)
        # -60 deg looks more steeply down -> closer forward distance
        self.assertGreater(result["cx1"], result["cx2"])

    def test_higher_altitude_larger_footprint(self):
        """Higher altitude produces larger ground footprint area."""
        result = _run_js("""
            const fp1 = computeFootprintCorners(-35, 0.42, 0.24, 3000, 100);
            const fp2 = computeFootprintCorners(-35, 0.42, 0.24, 3000, 300);
            const a1 = shoelaceArea(fp1);
            const a2 = shoelaceArea(fp2);
            console.log(JSON.stringify({ a1, a2 }));
        """)
        self.assertGreater(result["a2"], result["a1"])


class TestShoelaceArea(unittest.TestCase):
    """shoelaceArea computes polygon area correctly."""

    def test_unit_square(self):
        result = _run_js("""
            const area = shoelaceArea([[0,0], [1,0], [1,1], [0,1]]);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 1.0, places=10)

    def test_triangle(self):
        result = _run_js("""
            const area = shoelaceArea([[0,0], [4,0], [0,3]]);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 6.0, places=10)


class TestClipConvexPolygons(unittest.TestCase):
    """clipConvexPolygons implements Sutherland-Hodgman correctly."""

    def test_overlapping_squares(self):
        """Two overlapping unit squares produce correct intersection area."""
        result = _run_js("""
            const a = [[0,0], [2,0], [2,2], [0,2]];
            const b = [[1,1], [3,1], [3,3], [1,3]];
            const inter = clipConvexPolygons(a, b);
            const area = shoelaceArea(inter);
            console.log(JSON.stringify({ area, n: inter.length }));
        """)
        self.assertAlmostEqual(result["area"], 1.0, places=6)

    def test_no_overlap(self):
        """Disjoint polygons produce empty intersection."""
        result = _run_js("""
            const a = [[0,0], [1,0], [1,1], [0,1]];
            const b = [[5,5], [6,5], [6,6], [5,6]];
            const inter = clipConvexPolygons(a, b);
            console.log(JSON.stringify(inter.length));
        """)
        self.assertEqual(result, 0)

    def test_contained_polygon(self):
        """A polygon fully inside another clips to the inner polygon."""
        result = _run_js("""
            const outer = [[0,0], [10,0], [10,10], [0,10]];
            const inner = [[2,2], [4,2], [4,4], [2,4]];
            const inter = clipConvexPolygons(inner, outer);
            const area = shoelaceArea(inter);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 4.0, places=6)


class TestUnionArea(unittest.TestCase):
    """unionArea via inclusion-exclusion."""

    def test_single_polygon(self):
        result = _run_js("""
            const area = unionArea([[[0,0], [2,0], [2,2], [0,2]]]);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 4.0, places=6)

    def test_two_overlapping(self):
        """Union of two overlapping squares: 4 + 4 - 1 = 7."""
        result = _run_js("""
            const a = [[0,0], [2,0], [2,2], [0,2]];
            const b = [[1,1], [3,1], [3,3], [1,3]];
            const area = unionArea([a, b]);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 7.0, places=5)

    def test_disjoint_polygons(self):
        """Union of disjoint polygons = sum of areas."""
        result = _run_js("""
            const a = [[0,0], [1,0], [1,1], [0,1]];
            const b = [[5,5], [6,5], [6,6], [5,6]];
            const area = unionArea([a, b]);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 2.0, places=6)

    def test_null_polygons_ignored(self):
        result = _run_js("""
            const a = [[0,0], [2,0], [2,2], [0,2]];
            const area = unionArea([a, null, null]);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 4.0, places=6)


class TestDetectWeight(unittest.TestCase):
    """detectWeight computes detection probability from slant range."""

    def test_weight_at_max_range(self):
        """At mdd slant range, weight ≈ 0.25 (barely detectable)."""
        result = _run_js("""
            // footprint centroid at forward distance that gives slant = mdd
            const mdd = 1000;
            const alt = 200;
            // cx such that sqrt(alt^2 + cx^2) = mdd -> cx = sqrt(mdd^2 - alt^2)
            const cx = Math.sqrt(mdd*mdd - alt*alt);
            const fp = [[cx-10,-10],[cx+10,-10],[cx+10,10],[cx-10,10]];
            const w = detectWeight(fp, mdd, alt);
            console.log(JSON.stringify(w));
        """)
        self.assertAlmostEqual(result, 0.25, places=2)

    def test_weight_at_close_range(self):
        """At mdd/4 slant range, weight = 1.0 (fully reliable)."""
        result = _run_js("""
            const mdd = 1000;
            const alt = 50;
            // cx such that sqrt(alt^2 + cx^2) = mdd/4 = 250
            const cx = Math.sqrt(250*250 - 50*50);
            const fp = [[cx-10,-10],[cx+10,-10],[cx+10,10],[cx-10,10]];
            const w = detectWeight(fp, mdd, alt);
            console.log(JSON.stringify(w));
        """)
        self.assertAlmostEqual(result, 1.0, places=2)

    def test_weight_capped_at_one(self):
        """Very close range should cap at 1.0."""
        result = _run_js("""
            const fp = [[5,-5],[15,-5],[15,5],[5,5]];
            const w = detectWeight(fp, 1000, 10);
            console.log(JSON.stringify(w));
        """)
        self.assertAlmostEqual(result, 1.0, places=6)

    def test_null_footprint_returns_zero(self):
        """Null or invalid footprint returns 0."""
        result = _run_js("""
            console.log(JSON.stringify([
                detectWeight(null, 1000, 200),
                detectWeight([], 1000, 200),
                detectWeight([[0,0],[1,1]], 1000, 200),
            ]));
        """)
        for w in result:
            self.assertEqual(w, 0)


class TestWeightedUnionArea(unittest.TestCase):
    """weightedUnionArea weights coverage by detection quality."""

    def test_single_polygon(self):
        """Single polygon: area × weight."""
        result = _run_js("""
            const fp = [[0,0],[100,0],[100,50],[0,50]];
            const area = weightedUnionArea([fp], [0.5]);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 5000 * 0.5, places=2)

    def test_full_weight_equals_area(self):
        """Weight = 1.0 gives plain area."""
        result = _run_js("""
            const fp = [[0,0],[100,0],[100,50],[0,50]];
            const area = weightedUnionArea([fp], [1.0]);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 5000.0, places=2)

    def test_two_overlapping_weighted(self):
        """Two overlapping footprints with different weights."""
        result = _run_js("""
            // a: [0,0]-[2,0]-[2,2]-[0,2], area=4
            // b: [1,0]-[3,0]-[3,2]-[1,2], area=4
            // intersection: [1,0]-[2,0]-[2,2]-[1,2], area=2
            const a = [[0,0],[2,0],[2,2],[0,2]];
            const b = [[1,0],[3,0],[3,2],[1,2]];
            const wa = 0.8, wb = 0.4;
            // Expected: 4*0.8 + 4*0.4 - 2*max(0.8,0.4) = 3.2 + 1.6 - 1.6 = 3.2
            const area = weightedUnionArea([a, b], [wa, wb]);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 3.2, places=4)

    def test_null_polygons_ignored(self):
        """Null polygons are skipped."""
        result = _run_js("""
            const fp = [[0,0],[10,0],[10,10],[0,10]];
            const area = weightedUnionArea([fp, null, null], [0.5, 0.3, 0.7]);
            console.log(JSON.stringify(area));
        """)
        self.assertAlmostEqual(result, 100 * 0.5, places=2)


class TestAngularGap(unittest.TestCase):
    """angularGap checks coverage across the approach pitch envelope."""

    def test_no_gap_full_coverage(self):
        """Two cameras with overlapping FOVs covering the required range."""
        result = _run_js("""
            // fovV in radians: 30 deg = 0.5236 rad -> halfV = 15 deg
            const devs = [
                { pitchDeg: 5, fovV: 30 * Math.PI / 180 },
                { pitchDeg: -25, fovV: 30 * Math.PI / 180 },
            ];
            const gap = angularGap(devs, -40, 20);
            console.log(JSON.stringify(gap));
        """)
        self.assertAlmostEqual(result, 0.0, places=6)

    def test_gap_exists(self):
        """Two cameras with a gap between them."""
        result = _run_js("""
            // cam1: pitch=10, halfV=5 -> covers [5, 15]
            // cam2: pitch=-30, halfV=5 -> covers [-35, -25]
            // gap between -25 and 5 = 30 deg
            const devs = [
                { pitchDeg: 10, fovV: 10 * Math.PI / 180 },
                { pitchDeg: -30, fovV: 10 * Math.PI / 180 },
            ];
            const gap = angularGap(devs, -35, 15);
            console.log(JSON.stringify(gap));
        """)
        self.assertAlmostEqual(result, 6.0, places=4)

    def test_single_camera_partial(self):
        """Single camera covering only part of the required range."""
        result = _run_js("""
            // pitch=-20, halfV=15 -> covers [-35, -5]
            // required: [-60, 20]
            // uncovered: [-60, -35] + [-5, 20] = 25 + 25 = 50 raw
            // weighted gap accounts for position within envelope
            const devs = [{ pitchDeg: -20, fovV: 30 * Math.PI / 180 }];
            const gap = angularGap(devs, -60, 20);
            console.log(JSON.stringify(gap));
        """)
        self.assertAlmostEqual(result, 9.375, places=3)

    def test_single_camera_gap_position_sensitive(self):
        """angularGap returns different values for steep vs shallow positions."""
        result = _run_js("""
            // 10-deg FOV camera in 40-deg envelope [-50, -10]
            // Placed at steep end: pitch=-45, covers [-50, -40]
            const steep = [{ pitchDeg: -45, fovV: 10 * Math.PI / 180 }];
            // Placed at shallow end: pitch=-15, covers [-20, -10]
            const shallow = [{ pitchDeg: -15, fovV: 10 * Math.PI / 180 }];
            const gSteep = angularGap(steep, -50, -10);
            const gShallow = angularGap(shallow, -50, -10);
            console.log(JSON.stringify({ steep: gSteep, shallow: gShallow }));
        """)
        # Shallow placement should have lower excess gap (covers high-weight end)
        self.assertGreater(result["steep"], result["shallow"],
                           "Steep placement should have larger excess gap than shallow")
        # Steep excess must be positive; shallow is optimal so excess = 0
        self.assertGreater(result["steep"], 0)
        self.assertEqual(result["shallow"], 0)

    def test_single_camera_prefers_shallow_pitch(self):
        """Optimizer picks a shallow pitch for novoxy10-like single camera."""
        result = _run_js("""
            // novoxy10-like: 52.4 hFOV, 30.4 vFOV, mdd=3000
            const devices = [{
                fovH: 52.4 * Math.PI / 180,
                fovV: 30.4 * Math.PI / 180,
                mdd: 3000,
                currentPitch: -35,
            }];
            const r = optimizePitches(devices, { min: 50, max: 300 },
                                      { min: -60, max: 20 });
            console.log(JSON.stringify(r));
        """)
        # With weighted gap, optimizer should pick a pitch shallower than -25
        # (old behavior was ~-40 which is too steep for navigation)
        self.assertGreater(result["pitches"][0], -25,
                           f"Expected pitch > -25, got {result['pitches'][0]}")


class TestOptimizePitches(unittest.TestCase):
    """optimizePitches joint optimizer."""

    def test_single_device_reasonable_result(self):
        """Single device optimizes to a valid pitch and altitude."""
        result = _run_js("""
            const devices = [{
                fovH: 0.42, fovV: 0.47, mdd: 700, currentPitch: -35,
            }];
            const r = optimizePitches(devices, { min: 50, max: 300 },
                                      { min: -60, max: 20 });
            console.log(JSON.stringify(r));
        """)
        self.assertIn("pitches", result)
        self.assertIn("altitude", result)
        self.assertEqual(len(result["pitches"]), 1)
        self.assertGreaterEqual(result["pitches"][0], -90)
        self.assertLessEqual(result["pitches"][0], 90)
        self.assertGreaterEqual(result["altitude"], 50)
        self.assertLessEqual(result["altitude"], 300)

    def test_two_devices_cover_envelope(self):
        """Two devices should be spread to cover the pitch envelope."""
        result = _run_js("""
            const devices = [
                { fovH: 0.82, fovV: 0.47, mdd: 700, currentPitch: 5 },
                { fovH: 0.42, fovV: 0.24, mdd: 700, currentPitch: -35 },
            ];
            const r = optimizePitches(devices, { min: 50, max: 300 },
                                      { min: -40, max: 15 });
            console.log(JSON.stringify(r));
        """)
        self.assertEqual(len(result["pitches"]), 2)
        # The two pitches should be different
        self.assertNotAlmostEqual(result["pitches"][0], result["pitches"][1], places=1)

    def test_optimizer_prefers_no_gap(self):
        """Optimizer should strongly prefer configurations with no angular gap."""
        result = _run_js("""
            const devices = [
                { fovH: 0.5, fovV: 0.4, mdd: 1000, currentPitch: 5 },
                { fovH: 0.5, fovV: 0.4, mdd: 1000, currentPitch: -30 },
            ];
            // Narrow envelope that two ~23-deg-FOV cameras can cover
            const r = optimizePitches(devices, { min: 50, max: 400 },
                                      { min: -30, max: 10 });
            // Compute angular gap of the result
            const halfV0 = 0.4 * 180 / Math.PI / 2;
            const halfV1 = 0.4 * 180 / Math.PI / 2;
            const intervals = r.pitches.map(p => [p - halfV0, p + halfV0]).sort((a,b) => a[0]-b[0]);
            // Merge
            const merged = [intervals[0].slice()];
            for (let i = 1; i < intervals.length; i++) {
                if (intervals[i][0] <= merged[merged.length-1][1])
                    merged[merged.length-1][1] = Math.max(merged[merged.length-1][1], intervals[i][1]);
                else merged.push(intervals[i].slice());
            }
            let gap = 0, cursor = -30;
            for (const [lo, hi] of merged) {
                if (lo > cursor) gap += Math.min(lo, 10) - cursor;
                cursor = Math.max(cursor, hi);
                if (cursor >= 10) break;
            }
            if (cursor < 10) gap += 10 - cursor;
            console.log(JSON.stringify({ gap: Math.max(0, gap), pitches: r.pitches }));
        """)
        # The optimizer should find a near-gap-free configuration (< 2° tolerance
        # due to grid search quantization)
        self.assertLess(result["gap"], 2.0,
                        msg=f"Expected gap < 2°, got {result['gap']} with pitches {result['pitches']}")

    def test_optimizer_prefers_moderate_altitude(self):
        """Detection weighting should prevent optimizer from picking max altitude."""
        result = _run_js("""
            const devices = [{
                fovH: 0.42, fovV: 0.47, mdd: 700, currentPitch: -35,
            }];
            const r = optimizePitches(devices, { min: 50, max: 500 },
                                      { min: -60, max: 20 });
            console.log(JSON.stringify(r));
        """)
        # With detection weighting, the optimizer should NOT pick the maximum
        # altitude because POIs become barely detectable at long slant ranges
        self.assertLess(result["altitude"], 500,
                        msg=f"Expected altitude < max (500), got {result['altitude']}")

    def test_dual_camera_no_avoidable_gap(self):
        """Two cameras with enough total FOV should produce zero avoidable gap."""
        result = _run_js("""
            // novoxy_dual-like: two cameras with ~30 + ~27 deg vFOV in 80-deg envelope
            const devices = [
                { fovH: 0.53, fovV: 0.53, mdd: 661, currentPitch: 5 },
                { fovH: 0.45, fovV: 0.47, mdd: 753, currentPitch: -28 },
            ];
            const r = optimizePitches(devices, { min: 200, max: 500 },
                                      { min: -60, max: 20 });
            const devInfos = devices.map((d, i) => ({
                pitchDeg: r.pitches[i], fovV: d.fovV,
            }));
            const gap = angularGap(devInfos, -60, 20);
            console.log(JSON.stringify({ gap, pitches: r.pitches, altitude: r.altitude }));
        """)
        self.assertAlmostEqual(result["gap"], 0.0, places=0,
                               msg=f"Expected zero avoidable gap, got {result['gap']} "
                                   f"with pitches {result['pitches']}")

    def test_returns_score(self):
        """Result includes a numeric score."""
        result = _run_js("""
            const r = optimizePitches(
                [{ fovH: 0.42, fovV: 0.24, mdd: 700, currentPitch: -35 }],
                { min: 50, max: 200 },
                { min: -50, max: 10 },
            );
            console.log(JSON.stringify(typeof r.score));
        """)
        self.assertEqual(result, "number")


if __name__ == "__main__":
    unittest.main()
