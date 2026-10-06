"""Tests for cameraFootprint.js — multi-camera footprint computation."""
import math
import unittest
import subprocess
import json
import os
import re
import tempfile
from unittest.mock import Mock

from navpy.modules.vision.vision_profiles import (
    DETECTOR_CLASS_DIMENSIONS,
    get_class_detect_size,
)

# Per-class characteristic sizes are OWNED by the backend. Pin the smallest
# Class 4 and Class 0 diagonals from the single source so the JS expectations
# below track navpy.modules.vision.vision_profiles, not stale hardcoded numbers.
CLASS_4_SIZE = get_class_detect_size(4)
CLASS_0_SIZE = get_class_detect_size(0)


# Path to JS source directories
_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))
_MAP_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "map", "utils",
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


# Load JS files in dependency order
_JS_FILES = [
    (_UTILS_DIR, "projection.js"),
    (_UTILS_DIR, "plannerConfig.js"),
    (_UTILS_DIR, "scanGeometry.js"),
    (_UTILS_DIR, "geo.js"),
    (_MAP_UTILS_DIR, "gimbalTelemetry.js"),
    (_MAP_UTILS_DIR, "cameraFootprint.js"),
]

_JS_CODE = ""
for directory, fname in _JS_FILES:
    fpath = os.path.join(directory, fname)
    _JS_CODE += _strip_es_modules(open(fpath, encoding="utf-8").read()) + "\n"


# Mirror the backend's top-level `detector_class_dimensions` block and load it into the
# planner config so the JS per-class sizes match the Python single source.
_BACKEND_DOCK_CLASSES = {
    str(cid): {
        "width_m": w,
        "height_m": h,
        "size_m": get_class_detect_size(cid),
    }
    for cid, (w, h) in DETECTOR_CLASS_DIMENSIONS.items()
}
_CONFIGURE_DOCK_CLASSES_JS = (
    f"configureDetectorClassDimensions({json.dumps(_BACKEND_DOCK_CLASSES)});\n"
)


def _run_js(script):
    """Run a JS snippet via Node.js and return parsed JSON output.

    The bundle is written to a temp file and run as ``node <file>`` rather than
    ``node -e <code>`` — the inlined bundle exceeds the Windows command-line
    length limit (WinError 206) once the footprint helpers grow.
    """
    full = _JS_CODE + "\n" + _CONFIGURE_DOCK_CLASSES_JS + script
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".js", delete=False, encoding="utf-8",
    ) as fh:
        fh.write(full)
        path = fh.name
    try:
        result = subprocess.run(
            ["node", path],
            capture_output=True, text=True, timeout=10,
        )
    finally:
        os.unlink(path)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


def _seg_intersect(a, b, c, d):
    """True if segment a-b properly crosses segment c-d (planar, lon=x lat=y)."""
    def ccw(p, q, r):
        return (r[1] - p[1]) * (q[0] - p[0]) - (q[1] - p[1]) * (r[0] - p[0])
    d1, d2 = ccw(c, d, a), ccw(c, d, b)
    d3, d4 = ccw(a, b, c), ccw(a, b, d)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _polygon_is_simple(pts):
    """True if the closed lat/lon polygon has no crossing (non-adjacent) edges.

    Consecutive (and wrap-around) duplicate vertices are dropped first: the live
    footprint is padded with coincident vertices to a constant length (so the
    interpolating animation never snaps on a clip vertex-count change), and those
    zero-length edges are invisible. The outline that must be simple is the
    deduplicated, visible one — keeping the coincident pads would make the proper
    self-intersection test misfire on the shared endpoint at a degenerate corner.
    """
    raw = [(p["lon"], p["lat"]) for p in pts]
    coords = []
    for c in raw:
        if not coords or coords[-1] != c:
            coords.append(c)
    if len(coords) > 1 and coords[0] == coords[-1]:
        coords.pop()
    n = len(coords)
    for i in range(n):
        a, b = coords[i], coords[(i + 1) % n]
        for j in range(i + 1, n):
            # skip edges that share a vertex with edge i
            if (i + 1) % n == j or (j + 1) % n == i:
                continue
            c, d = coords[j], coords[(j + 1) % n]
            if _seg_intersect(a, b, c, d):
                return False
    return True


class TestOverlapFootprintConsistency(unittest.TestCase):
    """The settings 'Overlap' metric (overlap = meanFovV - spread) must agree with
    the live 2D footprint at LEVEL flight: overlap=0 => the two cameras' footprints
    abut (no overlap, no gap). This locks the consistency the operator expected and
    proves the metric is correct — so any footprint overlap seen in flight is from
    vehicle attitude or a stale running profile, NOT a metric/geometry bug.

    Edges are measured against the geometric FOV (maxDetectDist large), i.e. the
    coverage contract; the detection-range cap deliberately extends the envelope
    farther and is orthogonal to camera-to-camera coverage abutment.
    """

    FY_HI, FY_LO, IMG_H, FOVH, ALT = 1984.98, 2262.15, 1080, 0.8, 150.0  # novoxy_dual z1

    def _fwd_range(self, fovv_rad, pitch):
        fp = _run_js(f"""
            const cam = {{ fovH: {self.FOVH}, fovV: {fovv_rad}, pitchDeg: {pitch},
                          maxDetectDist: 1e9 }};
            console.log(JSON.stringify(
                computeCameraFootprint(40.0, 44.0, {self.ALT}, 0, 0, 0, cam)));
        """)
        self.assertIsNotNone(fp)
        north = [(p["lat"] - 40.0) * 111320 for p in fp]
        return min(north), max(north)

    def _gap_for_overlap(self, overlap_deg, center=-20.0):
        """Signed forward gap between the upper camera's NEAR edge and the lower
        camera's FAR edge at the given settings overlap. >0 gap, ~0 abut, <0 overlap."""
        fovv_hi = 2 * math.atan(self.IMG_H / (2 * self.FY_HI))
        fovv_lo = 2 * math.atan(self.IMG_H / (2 * self.FY_LO))
        mean_fov_deg = math.degrees((fovv_hi + fovv_lo) / 2)
        spread = mean_fov_deg - overlap_deg            # overlap = meanFovV - spread
        p_hi = center + spread / 2                     # shallower (upper) camera
        p_lo = center - spread / 2                     # steeper (lower) camera
        near_hi, _ = self._fwd_range(fovv_hi, p_hi)
        _, far_lo = self._fwd_range(fovv_lo, p_lo)
        return near_hi - far_lo

    def test_overlap_zero_footprints_abut(self):
        self.assertAlmostEqual(self._gap_for_overlap(0.0), 0.0, delta=2.0)

    def test_positive_overlap_footprints_overlap(self):
        self.assertLess(self._gap_for_overlap(8.0), -5.0)

    def test_negative_overlap_leaves_gap(self):
        self.assertGreater(self._gap_for_overlap(-8.0), 5.0)


class TestFootprintHorizonClip(unittest.TestCase):
    """Footprint must clip the FOV to the horizon — no bow-tie when the upper
    edge grazes/exceeds the horizon (a shallow, nose-down, rolled camera)."""

    def test_rolled_above_horizon_fov_is_simple_not_bowtie(self):
        # Shallow camera (-10), vehicle nose-down (-3) + rolled (7): the upper FOV
        # edge points above the horizon. The OLD 4-corner projection self-intersects.
        result = _run_js("""
            const cam = { fovH: 0.8, fovV: 0.53, pitchDeg: -10 };
            const fp = computeCameraFootprint(40.31, 44.45, 95, 0, -3, 7, cam);
            console.log(JSON.stringify(fp));
        """)
        self.assertIsNotNone(result)
        self.assertGreaterEqual(len(result), 3)
        self.assertTrue(
            _polygon_is_simple(result),
            f"footprint self-intersects (bow-tie): {result}",
        )

    def test_steep_camera_unchanged_simple_quad(self):
        # All corners below the horizon -> clip is a no-op -> 4-point simple trapezoid.
        result = _run_js("""
            const cam = { fovH: 0.42, fovV: 0.24, pitchDeg: -45 };
            const fp = computeCameraFootprint(32.0, 34.8, 200, 90, 0, 0, cam);
            console.log(JSON.stringify(fp));
        """)
        self.assertEqual(len(result), 4)
        self.assertTrue(_polygon_is_simple(result))

    def test_above_horizon_partial_fov_is_simple(self):
        # Level camera centred above horizon (+5) but lower FOV sees ground.
        result = _run_js("""
            const cam = { fovH: 0.82, fovV: 0.47, pitchDeg: 5 };
            const fp = computeCameraFootprint(32.0, 34.8, 200, 0, 0, 0, cam);
            console.log(JSON.stringify(fp));
        """)
        self.assertIsNotNone(result)
        self.assertTrue(_polygon_is_simple(result))

    def test_simple_through_roll_no_bowtie(self):
        # THE BOW-TIE FIX: the horizon clip (Sutherland-Hodgman) yields a SIMPLE
        # polygon at EVERY roll — a convex frustum clipped by the horizon half-plane
        # stays convex. A down camera swept through a hard bank that takes its upper
        # corners across the horizon never self-intersects. (The abandoned fixed
        # 4-corner clamp bow-tied here for near-horizontal slivers; the clip does
        # not.) The constant on-screen vertex count that removes the turn jitter is
        # applied separately by padFootprintLoop — see TestPadFootprintLoop.
        result = _run_js("""
            const cam = { fovH: 0.9, fovV: 0.5, pitchDeg: -16 };
            const out = [];
            for (let roll = -55; roll <= 55; roll += 5) {
                out.push(computeCameraFootprint(32.0, 34.8, 150, 0, -3, roll, cam));
            }
            console.log(JSON.stringify(out));
        """)
        drew = 0
        for idx, fp in enumerate(result):
            if fp is None:
                continue
            drew += 1
            self.assertGreaterEqual(len(fp), 3)
            self.assertTrue(
                _polygon_is_simple(fp),
                f"footprint self-intersects (bow-tie) at sweep index {idx}: {fp}",
            )
        self.assertGreater(drew, 10)   # most of the swept bank drew a footprint

    def test_near_horizontal_rolled_sliver_is_simple(self):
        # Regression lock for the operator's reported intersection: a near-horizontal
        # forward camera (pitch -2) with the vehicle pitched up and rolled grazes the
        # ground as a thin far sliver whose 4 projected corners are nearly collinear
        # and whose "near"/"far" ordering inverts under roll. The fixed 4-corner clamp
        # self-intersected (bow-tie) on exactly this; the clip stays simple.
        result = _run_js("""
            const cam = { fovH: 0.6, fovV: 0.3, pitchDeg: -2, maxDetectDist: 1e9 };
            const fp = computeCameraFootprint(32.0, 34.8, 150, 0, 3, 10, cam);
            console.log(JSON.stringify(fp));
        """)
        self.assertIsNotNone(result)
        self.assertTrue(
            _polygon_is_simple(result),
            f"near-horizontal sliver self-intersects (bow-tie): {result}",
        )


class TestComputeCameraFootprint(unittest.TestCase):
    """computeCameraFootprint with explicit camConfig."""

    def test_explicit_camconfig_produces_4_points(self):
        """Passing an explicit camConfig should produce a valid 4-point polygon."""
        result = _run_js("""
            const cam = { fovH: 0.42, fovV: 0.24, pitchDeg: -35 };
            const fp = computeCameraFootprint(32.0, 34.8, 200, 90, 0, 0, cam);
            console.log(JSON.stringify(fp));
        """)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 4)
        for pt in result:
            self.assertIn("lat", pt)
            self.assertIn("lon", pt)

    def test_no_camconfig_uses_default(self):
        """Without camConfig, uses global getCameraConfig()."""
        result = _run_js("""
            const fp = computeCameraFootprint(32.0, 34.8, 200, 90, 0, 0);
            console.log(JSON.stringify(fp));
        """)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 4)

    def test_different_pitch_produces_different_footprint(self):
        """Two configs with different pitch should produce different footprints."""
        result = _run_js("""
            const cam1 = { fovH: 0.42, fovV: 0.24, pitchDeg: -10 };
            const cam2 = { fovH: 0.42, fovV: 0.24, pitchDeg: -55 };
            const fp1 = computeCameraFootprint(32.0, 34.8, 200, 0, 0, 0, cam1);
            const fp2 = computeCameraFootprint(32.0, 34.8, 200, 0, 0, 0, cam2);
            console.log(JSON.stringify({ fp1, fp2 }));
        """)
        fp1 = result["fp1"]
        fp2 = result["fp2"]
        self.assertIsNotNone(fp1)
        self.assertIsNotNone(fp2)
        # Centroids should differ significantly (different pitch angles)
        c1_lat = sum(p["lat"] for p in fp1) / 4
        c2_lat = sum(p["lat"] for p in fp2) / 4
        self.assertGreater(abs(c1_lat - c2_lat), 0.0001)

    def test_forward_camera_projects_ahead(self):
        """A shallow-angle camera heading north projects mostly north of aircraft."""
        result = _run_js("""
            const cam = { fovH: 0.42, fovV: 0.24, pitchDeg: -10 };
            const fp = computeCameraFootprint(32.0, 34.8, 200, 0, 0, 0, cam);
            console.log(JSON.stringify(fp));
        """)
        self.assertIsNotNone(result)
        # Centroid should be north of the aircraft (lat > 32.0) for shallow-angle cam heading north
        centroid_lat = sum(p["lat"] for p in result) / 4
        self.assertGreater(centroid_lat, 32.0)

    def test_above_horizon_center_but_fov_hits_ground(self):
        """Camera center above horizon (pitchDeg=5) but FOV bottom sees ground."""
        result = _run_js("""
            const cam = { fovH: 0.82, fovV: 0.47, pitchDeg: 5 };
            const fp = computeCameraFootprint(32.0, 34.8, 200, 0, 0, 0, cam);
            console.log(JSON.stringify(fp));
        """)
        # Should NOT be null — part of FOV hits ground even though center ray is above horizon
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 4)

    def test_camera_pointing_sky_returns_null(self):
        """Camera pointing well above horizon with narrow FOV returns null."""
        result = _run_js("""
            const cam = { fovH: 0.2, fovV: 0.1, pitchDeg: 20 };
            const fp = computeCameraFootprint(32.0, 34.8, 200, 0, 0, 0, cam);
            console.log(JSON.stringify(fp));
        """)
        self.assertIsNone(result)

    def test_low_altitude_returns_null(self):
        """Alt below 10m returns null."""
        result = _run_js("""
            const cam = { fovH: 0.42, fovV: 0.24, pitchDeg: -35 };
            const fp = computeCameraFootprint(32.0, 34.8, 5, 90, 0, 0, cam);
            console.log(JSON.stringify(fp));
        """)
        self.assertIsNone(result)


class TestDeviceConfigs(unittest.TestCase):
    """getDeviceConfigs / setDeviceConfigs / clearDeviceConfigs."""

    def test_fallback_returns_empty_until_configured(self):
        """Without backend or manual device configs, getDeviceConfigs returns no coverage devices."""
        result = _run_js("""
            clearDeviceConfigs();
            const configs = getDeviceConfigs();
            console.log(JSON.stringify(configs));
        """)
        self.assertEqual(result, [])

    def test_set_returns_all_devices(self):
        """setDeviceConfigs stores and getDeviceConfigs retrieves all devices."""
        result = _run_js("""
            setDeviceConfigs([
                { name: "cam1", fovH: 0.42, fovV: 0.24, pitchDeg: 5, primary: true },
                { name: "cam2", fovH: 0.3, fovV: 0.2, pitchDeg: -35, primary: false },
            ]);
            const configs = getDeviceConfigs();
            console.log(JSON.stringify(configs));
        """)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["name"], "cam1")
        self.assertTrue(result[0]["primary"])
        self.assertEqual(result[1]["name"], "cam2")
        self.assertFalse(result[1]["primary"])

    def test_clear_reverts_to_empty(self):
        """clearDeviceConfigs resets coverage devices until backend config is available."""
        result = _run_js("""
            setDeviceConfigs([
                { name: "cam1", fovH: 0.42, fovV: 0.24, pitchDeg: 5, primary: true },
                { name: "cam2", fovH: 0.3, fovV: 0.2, pitchDeg: -35, primary: false },
            ]);
            clearDeviceConfigs();
            const configs = getDeviceConfigs();
            console.log(JSON.stringify(configs));
        """)
        self.assertEqual(result, [])

    def test_multi_device_footprints_differ(self):
        """Footprints computed with different device configs should differ."""
        result = _run_js("""
            const devices = [
                { name: "fwd", fovH: 0.42, fovV: 0.24, pitchDeg: -10, primary: true },
                { name: "down", fovH: 0.3, fovV: 0.2, pitchDeg: -55, primary: false },
            ];
            setDeviceConfigs(devices);
            const configs = getDeviceConfigs();
            const fps = configs.map(c =>
                computeCameraFootprint(32.0, 34.8, 200, 0, 0, 0, c)
            );
            console.log(JSON.stringify(fps));
        """)
        self.assertEqual(len(result), 2)
        self.assertIsNotNone(result[0])
        self.assertIsNotNone(result[1])
        # Centroids should differ
        c0_lat = sum(p["lat"] for p in result[0]) / 4
        c1_lat = sum(p["lat"] for p in result[1]) / 4
        self.assertGreater(abs(c0_lat - c1_lat), 0.0001)


class TestConfigurePlannerDeviceConfigs(unittest.TestCase):
    """configurePlanner resolves device configs from profile catalog."""

    def test_profile_resolves_all_devices(self):
        """configurePlanner with profileCatalog populates device configs."""
        result = _run_js("""
            const settings = {
                camera: {
                    vision_profile: "dual_cam",
                    vision_device: "cam_a",
                    vision_zoom: "1",
                },
            };
            const catalog = {
                dual_cam: {
                    devices: [
                        {
                            name: "cam_a", image_width: 1920, image_height: 1080,
                            pitch_deg: 5,
                            gimbal_device_id: 1,
                            publishes_gimbal_telemetry: true,
                            setup_att: [90, 0, 90],
                            setup_seq: "XYZ",
                            gimbal_seq: "XYZ",
                            zooms: { "1": { fx: 2000, fy: 2000, detect_range_m: 740, confirm_range_m: 180 } },
                        },
                        {
                            name: "cam_b", image_width: 1920, image_height: 1080,
                            pitch_deg: -35,
                            gimbal_device_id: 2,
                            publishes_gimbal_telemetry: true,
                            setup_att: [90, 0, 90],
                            setup_seq: "XYZ",
                            gimbal_seq: "XYZ",
                            zooms: { "1": { fx: 1500, fy: 1500, detect_range_m: 555, confirm_range_m: 135 } },
                        },
                    ],
                },
            };
            configurePlanner(settings, catalog);
            const configs = getDeviceConfigs();
            console.log(JSON.stringify(configs));
        """)
        self.assertEqual(len(result), 2)
        self.assertEqual(result[0]["name"], "cam_a")
        self.assertTrue(result[0]["primary"])
        self.assertEqual(result[0]["gimbal_device_id"], 1)
        self.assertEqual(result[0]["footprintSource"], "live-gimbal")
        self.assertEqual(result[0]["imageWidth"], 1920)
        self.assertEqual(result[0]["imageHeight"], 1080)
        self.assertEqual(result[0]["setup_att"], [90, 0, 90])
        self.assertEqual(result[0]["setup_seq"], "XYZ")
        self.assertNotIn("gimbal_seq", result[0])
        self.assertEqual(result[1]["name"], "cam_b")
        self.assertFalse(result[1]["primary"])
        self.assertEqual(result[1]["gimbal_device_id"], 2)
        # FOV should be in radians, matching getCameraConfig() range (~0.2–1.0 rad)
        import math
        expected_fovH_a = 2 * math.atan(1920 / (2 * 2000))
        self.assertAlmostEqual(result[0]["fovH"], expected_fovH_a, places=6)
        self.assertAlmostEqual(result[0]["maxDetectDist"], 740, places=6)
        self.assertAlmostEqual(result[1]["maxDetectDist"], 555, places=6)
        for cfg in result:
            self.assertGreater(cfg["fovH"], 0.1, "fovH too small — possible DEG2RAD double-conversion")
            self.assertGreater(cfg["fovV"], 0.1, "fovV too small — possible DEG2RAD double-conversion")

    def test_profile_uses_backend_detect_range_from_catalog(self):
        """configurePlanner should trust backend detect_range_m for live coverage."""
        result = _run_js("""
            const settings = {
                camera: {
                    vision_profile: "source_of_truth",
                    vision_device: "cam",
                    vision_zoom: "1",
                },
            };
            const catalog = {
                source_of_truth: {
                    devices: [{
                        name: "cam", image_width: 1920, image_height: 1080,
                        pitch_deg: -20,
                        zooms: { "1": { fx: 2000, fy: 2000, detect_range_m: 1234, confirm_range_m: 180 } },
                    }],
                },
            };
            configurePlanner(settings, catalog);
            console.log(JSON.stringify(getDeviceConfigs()));
        """)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["maxDetectDist"], 1234)

    def test_no_profile_clears_device_configs(self):
        """configurePlanner without profile clears device configs."""
        result = _run_js("""
            // First set some device configs
            setDeviceConfigs([
                { name: "cam1", fovH: 0.4, fovV: 0.2, pitchDeg: 0, primary: true },
            ]);
            // Now configure without profile
            configurePlanner({ camera: {} });
            const configs = getDeviceConfigs();
            console.log(JSON.stringify(configs));
        """)
        self.assertEqual(result, [])


class TestMaxDetectDistance(unittest.TestCase):
    """Footprint projection cap uses per-device maxDetectDist."""

    def test_maxDetectDist_extends_footprint(self):
        """With maxDetectDist=3000, footprint extends beyond the old 8*alt cap."""
        result = _run_js("""
            // +5 deg forward camera, 150m alt, heading north, level flight
            const cam = { fovH: 0.82, fovV: 0.47, pitchDeg: 5, maxDetectDist: 3000 };
            const fp = computeCameraFootprint(32.0, 34.8, 150, 0, 0, 0, cam);
            // Compute max distance from aircraft to any footprint corner
            const EARTH_R = 6371000;
            let maxDist = 0;
            for (const pt of fp) {
                const dlat = (pt.lat - 32.0) * Math.PI / 180 * EARTH_R;
                const dlon = (pt.lon - 34.8) * Math.PI / 180 * EARTH_R * Math.cos(32.0 * Math.PI / 180);
                const d = Math.sqrt(dlat*dlat + dlon*dlon);
                if (d > maxDist) maxDist = d;
            }
            console.log(JSON.stringify({ maxDist }));
        """)
        # Old cap was 8*150=1200m — new footprint should exceed that
        self.assertGreater(result["maxDist"], 1200,
                           "Footprint should extend beyond old 8*alt cap with maxDetectDist=3000")
        # But should not exceed maxDetectDist
        self.assertLessEqual(result["maxDist"], 3100,
                             "Footprint should not exceed maxDetectDist significantly")

    def test_no_maxDetectDist_caps_at_factor(self):
        """Without maxDetectDist, footprint caps at MAX_PROJ_FACTOR * alt."""
        result = _run_js("""
            // Same camera but no maxDetectDist
            const cam = { fovH: 0.82, fovV: 0.47, pitchDeg: 5 };
            const fp = computeCameraFootprint(32.0, 34.8, 150, 0, 0, 0, cam);
            const EARTH_R = 6371000;
            let maxDist = 0;
            for (const pt of fp) {
                const dlat = (pt.lat - 32.0) * Math.PI / 180 * EARTH_R;
                const dlon = (pt.lon - 34.8) * Math.PI / 180 * EARTH_R * Math.cos(32.0 * Math.PI / 180);
                const d = Math.sqrt(dlat*dlat + dlon*dlon);
                if (d > maxDist) maxDist = d;
            }
            console.log(JSON.stringify({ maxDist }));
        """)
        # Should cap at 8*150=1200m
        self.assertLessEqual(result["maxDist"], 1250,
                             "Without maxDetectDist, footprint should cap at MAX_PROJ_FACTOR * alt")

    def test_maxDetectDist_is_slant_range_cap(self):
        """Detection envelope treats maxDetectDist as detector slant range, not ground range."""
        result = _run_js("""
            const cam = { fovH: 0.82, fovV: 0.47, pitchDeg: -45, maxDetectDist: 300 };
            const fp = computeCameraDetectionEnvelope(32.0, 34.8, 150, 0, 0, 0, cam);
            const EARTH_R = 6371000;
            let maxDist = 0;
            for (const pt of fp) {
                const dlat = (pt.lat - 32.0) * Math.PI / 180 * EARTH_R;
                const dlon = (pt.lon - 34.8) * Math.PI / 180 * EARTH_R * Math.cos(32.0 * Math.PI / 180);
                const d = Math.sqrt(dlat*dlat + dlon*dlon);
                if (d > maxDist) maxDist = d;
            }
            console.log(JSON.stringify({ maxDist }));
        """)
        self.assertLess(
            result["maxDist"],
            261,
            "Slant-range envelope should project shorter than a 300 m horizontal cap",
        )

    def test_configurePlanner_computes_maxDetectDist_with_imgsz(self):
        """configurePlanner uses backend detect ranges for live maxDetectDist."""
        result = _run_js("""
            const settings = {
                camera: {
                    vision_profile: "test_profile",
                    vision_device: "fwd_cam",
                    vision_zoom: "1",
                },
            };
            const catalog = {
                test_profile: {
                    reference_height_m: 2.0,
                    imgsz: 640,
                    devices: [
                        {
                            name: "fwd_cam", image_width: 1920, image_height: 1080,
                            pitch_deg: 5,
                            zooms: { "1": { fx: 2000, fy: 2000, detect_range_m: 740, confirm_range_m: 180 } },
                        },
                        {
                            name: "down_cam", image_width: 1920, image_height: 1080,
                            pitch_deg: -35,
                            zooms: { "1": { fx: 1500, fy: 1500, detect_range_m: 555, confirm_range_m: 135 } },
                        },
                    ],
                },
            };
            configurePlanner(settings, catalog);
            const configs = getDeviceConfigs();
            console.log(JSON.stringify(configs));
        """)
        self.assertEqual(len(result), 2)
        self.assertAlmostEqual(result[0]["maxDetectDist"], 740, places=1)
        self.assertAlmostEqual(result[1]["maxDetectDist"], 555, places=1)

    def test_zoom_changes_maxDetectDist(self):
        """Higher zoom (larger fy) increases maxDetectDist proportionally."""
        result = _run_js("""
            const settings = {
                camera: {
                    vision_profile: "zoom_test",
                    vision_device: "cam",
                    vision_zoom: "2",
                },
            };
            const catalog = {
                zoom_test: {
                    reference_height_m: 2.0,
                    imgsz: 640,
                    devices: [
                        {
                            name: "cam", image_width: 1920, image_height: 1080,
                            pitch_deg: 5,
                            zooms: {
                                "1": { fx: 2000, fy: 2000, detect_range_m: 740, confirm_range_m: 180 },
                                "2": { fx: 4000, fy: 4000, detect_range_m: 1480, confirm_range_m: 360 },
                            },
                        },
                    ],
                },
            };
            configurePlanner(settings, catalog);
            const configs = getDeviceConfigs();
            console.log(JSON.stringify(configs));
        """)
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(result[0]["maxDetectDist"], 1480, places=1)

    def test_fallback_returns_no_detect_range_without_backend_configs(self):
        """Without backend device configs, the planner should not invent detect ranges."""
        result = _run_js("""
            clearDeviceConfigs();
            const configs = getDeviceConfigs();
            console.log(JSON.stringify(configs));
        """)
        self.assertEqual(result, [])


class TestComputeMaxDetectDist(unittest.TestCase):
    """Exported computeMaxDetectDist function."""

    def test_known_values(self):
        """computeMaxDetectDist returns fy * MIN_CLASS_SIZE / MIN_CONFIRM_PIXELS."""
        result = _run_js("""
            const d = computeMaxDetectDist(2000, 1920, 1080, 2.0, 640);
            console.log(JSON.stringify(d));
        """)
        # The CONFIRM-range base; ProfileSelector ×detectRangeScale converts it to
        # the selected POI's detection range before the diagram renders it.
        expected = 2000 * CLASS_4_SIZE / 20
        self.assertAlmostEqual(result, expected, places=2)

    def test_ignores_legacy_params(self):
        """Legacy params (reference_height_m, imgsz, image dims) don't affect result."""
        result = _run_js("""
            const d = computeMaxDetectDist(1500, 1920, 1080, null, null);
            console.log(JSON.stringify(d));
        """)
        # mdd = fy * Person diagonal / 20 — legacy params have no effect
        expected = 1500 * CLASS_4_SIZE / 20
        self.assertAlmostEqual(result, expected, places=2)

    def test_matches_confirm_range_catalog_values(self):
        """computeMaxDetectDist is the CONFIRM-range base (×detectRangeScale in
        ProfileSelector yields the diagram's detection range)."""
        result = _run_js(f"""
            const fy = 3000;
            const direct = computeMaxDetectDist(fy, 1920, 1080, 1.8, 640);
            const expectedConfirm = fy * {CLASS_4_SIZE} / 20;
            console.log(JSON.stringify({{ direct, expectedConfirm }}));
        """)
        self.assertAlmostEqual(result["direct"], result["expectedConfirm"], places=6)

    def test_higher_fy_increases_range(self):
        """Higher fy (more zoom) produces larger detection distance."""
        result = _run_js("""
            const d1 = computeMaxDetectDist(2000, 1920, 1080, 2.0, 640);
            const d2 = computeMaxDetectDist(4000, 1920, 1080, 2.0, 640);
            console.log(JSON.stringify({ d1, d2 }));
        """)
        self.assertAlmostEqual(result["d2"], result["d1"] * 2, places=2)


class TestCameraBodyTransform(unittest.TestCase):
    """Verify frontend camera-to-body transform matches backend geo_ref_calc."""

    def test_transform_matches_backend(self):
        """For a known pitch, camera->body transform matches the backend matrix.

        Backend (geo_ref_calc.py) combined camera->body:
            bx =  sin(p)*cy + cos(p)*cz
            by =  cx
            bz =  cos(p)*cy - sin(p)*cz
        """
        result = _run_js("""
            const pitchDeg = -35;
            const cam = { fovH: 0.42, fovV: 0.24, pitchDeg };
            const mount = _mountConstants(cam);

            // Test corner: bottom-right (cx=tanH, cy=tanV, cz=1)
            const tanH = Math.tan(cam.fovH / 2);
            const tanV = Math.tan(cam.fovV / 2);
            const cx = tanH, cy = tanV, cz = 1;

            // Frontend transform
            const bx = mount.sinP * cy + mount.cosP * cz;
            const by = cx;
            const bz = mount.cosP * cy - mount.sinP * cz;

            // Backend reference
            const p = pitchDeg * Math.PI / 180;
            const ref_bx = Math.sin(p) * cy + Math.cos(p) * cz;
            const ref_by = cx;
            const ref_bz = Math.cos(p) * cy - Math.sin(p) * cz;

            console.log(JSON.stringify({
                bx, by, bz, ref_bx, ref_by, ref_bz,
                sinP: mount.sinP, cosP: mount.cosP,
            }));
        """)
        self.assertAlmostEqual(result["bx"], result["ref_bx"], places=10)
        self.assertAlmostEqual(result["by"], result["ref_by"], places=10)
        self.assertAlmostEqual(result["bz"], result["ref_bz"], places=10)

    def test_near_edge_closer_than_far_edge(self):
        """Bottom-of-image (near ground) projects closer than top-of-image (far)."""
        result = _run_js("""
            const cam = { fovH: 0.42, fovV: 0.24, pitchDeg: -35 };
            const fp = computeCameraFootprint(32.0, 34.8, 200, 0, 0, 0, cam);
            // fp[0]=top-left, fp[1]=top-right (far), fp[2]=bottom-right, fp[3]=bottom-left (near)
            const EARTH_R = 6371000;
            function dist(pt) {
                const dlat = (pt.lat - 32.0) * Math.PI / 180 * EARTH_R;
                const dlon = (pt.lon - 34.8) * Math.PI / 180 * EARTH_R * Math.cos(32.0 * Math.PI / 180);
                return Math.sqrt(dlat*dlat + dlon*dlon);
            }
            // Average distance of top (far) vs bottom (near) edges
            const farDist = (dist(fp[0]) + dist(fp[1])) / 2;
            const nearDist = (dist(fp[2]) + dist(fp[3])) / 2;
            console.log(JSON.stringify({ farDist, nearDist }));
        """)
        # Top-of-image (far edge) should project farther than bottom-of-image (near edge)
        self.assertGreater(result["farDist"], result["nearDist"],
                           "Far edge (top of image) should project farther than near edge (bottom)")


class TestLiveGimbalFootprint(unittest.TestCase):
    """Live gimbal quaternion projection and fallback behavior."""

    def assert_footprints_close(self, actual, expected, places=8):
        self.assertIsNotNone(actual)
        self.assertIsNotNone(expected)
        self.assertEqual(len(actual), len(expected))
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a["lat"], e["lat"], places=places)
            self.assertAlmostEqual(a["lon"], e["lon"], places=places)

    @staticmethod
    def _centroid_lat(footprint):
        return sum(p["lat"] for p in footprint) / len(footprint)

    def test_live_fixed_gimbal_matches_static_fallback(self):
        """A q built from the profile pitch and setup metadata matches legacy projection."""
        result = _run_js("""
            function pitchQuat(deg) {
                const h = deg * Math.PI / 360;
                return [Math.cos(h), 0, Math.sin(h), 0];
            }
            const cam = {
                fovH: 0.42, fovV: 0.24, pitchDeg: -35,
                imageHeight: 1080,
                gimbal_device_id: 1, setup_att: [90, 0, 90], setup_seq: "XYZ",
            };
            const telemetry = {
                device_id: 1, q: pitchQuat(-35), flags: 32,
                failure_flags: 0, stale: false,
                fov_h_rad: 0.42, fov_v_rad: 0.24, optics_stale: false,
                zoom_level: 1,
            };
            const staticFp = computeStaticCameraFootprint(32.0, 34.8, 200, 0, 0, 0, cam);
            const live = selectGimbalTelemetry({ gimbals: { "1": telemetry } }, cam);
            const liveFp = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 0, 0, cam, live
            );
            console.log(JSON.stringify({ staticFp, liveFp }));
        """)
        self.assert_footprints_close(result["liveFp"], result["staticFp"])

    def test_live_gimbal_compensates_vehicle_pitch(self):
        """Changing live body-frame q can hold the footprint while aircraft pitch changes."""
        result = _run_js("""
            function pitchQuat(deg) {
                const h = deg * Math.PI / 360;
                return [Math.cos(h), 0, Math.sin(h), 0];
            }
            const cam = {
                fovH: 0.42, fovV: 0.24, pitchDeg: -35,
                imageHeight: 1080,
                gimbal_device_id: 1, setup_att: [90, 0, 90], setup_seq: "XYZ",
            };
            const liveLevel = {
                device_id: 1, q: pitchQuat(-35), flags: 32,
                failure_flags: 0, stale: false,
                fov_h_rad: 0.42, fov_v_rad: 0.24, optics_stale: false,
                zoom_level: 1,
            };
            const liveCompensated = {
                device_id: 1, q: pitchQuat(-55), flags: 32,
                failure_flags: 0, stale: false,
                fov_h_rad: 0.42, fov_v_rad: 0.24, optics_stale: false,
                zoom_level: 1,
            };
            const levelTelemetry = selectGimbalTelemetry({ gimbals: { "1": liveLevel } }, cam);
            const compensatedTelemetry = selectGimbalTelemetry(
                { gimbals: { "1": liveCompensated } }, cam
            );
            const level = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 0, 0, cam, levelTelemetry
            );
            const compensated = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 20, 0, cam, compensatedTelemetry
            );
            const bodyFixed = computeStaticCameraFootprint(32.0, 34.8, 200, 0, 20, 0, cam);
            console.log(JSON.stringify({ level, compensated, bodyFixed }));
        """)
        self.assert_footprints_close(result["compensated"], result["level"])
        diff = abs(self._centroid_lat(result["bodyFixed"]) - self._centroid_lat(result["level"]))
        self.assertGreater(diff, 0.0001)

    def test_unusable_live_gimbal_fails_closed(self):
        """Stale, failed, earth-frame, bad q, or bad setup metadata produces no live footprint."""
        result = _run_js("""
            function pitchQuat(deg) {
                const h = deg * Math.PI / 360;
                return [Math.cos(h), 0, Math.sin(h), 0];
            }
            const cam = {
                fovH: 0.42, fovV: 0.24, pitchDeg: -35,
                imageHeight: 1080,
                gimbal_device_id: 1, setup_att: [90, 0, 90], setup_seq: "XYZ",
            };
            const valid = {
                device_id: 1, q: pitchQuat(-55), flags: 32,
                failure_flags: 0, stale: false,
                fov_h_rad: 0.42, fov_v_rad: 0.24, optics_stale: false,
                zoom_level: 1,
            };
            const cases = [
                { dev: cam, t: { ...valid, stale: true } },
                { dev: cam, t: { ...valid, stale: undefined } },
                { dev: cam, t: { ...valid, stale: null } },
                { dev: cam, t: { ...valid, flags: 64 } },
                { dev: cam, t: { ...valid, flags: 96 } },
                { dev: cam, t: { ...valid, failure_flags: 1 } },
                { dev: cam, t: { ...valid, failure_flags: undefined } },
                { dev: cam, t: { ...valid, failure_flags: null } },
                { dev: cam, t: { ...valid, failure_flags: NaN } },
                { dev: cam, t: { ...valid, q: [0, 0, 0, 0] } },
                { dev: { ...cam, setup_att: undefined }, t: valid },
                { dev: { ...cam, setup_seq: "X" }, t: valid },
                { dev: { ...cam, setup_seq: "XY" }, t: valid },
                { dev: { ...cam, setup_seq: "ABQ" }, t: valid },
            ];
            const selections = cases.map(({ dev, t }) =>
                selectGimbalTelemetry({ gimbals: { "1": t } }, dev)
            );
            const liveFootprints = cases.map(({ dev }, i) =>
                computeLiveGimbalCameraFootprint(32.0, 34.8, 200, 0, 20, 0, dev, selections[i])
            );
            console.log(JSON.stringify({ liveFootprints, selections }));
        """)
        self.assertEqual(result["liveFootprints"], [None] * 14)
        self.assertEqual(result["selections"][:10], [None] * 10)
        for selection in result["selections"][10:]:
            self.assertIsNotNone(selection)

    def test_live_only_footprint_has_no_static_fallback(self):
        """Live-only projection returns null unless usable MAVLink gimbal telemetry is present."""
        result = _run_js("""
            function pitchQuat(deg) {
                const h = deg * Math.PI / 360;
                return [Math.cos(h), 0, Math.sin(h), 0];
            }
            const cam = {
                fovH: 0.42, fovV: 0.24, pitchDeg: -35,
                imageHeight: 1080,
                gimbal_device_id: 1, setup_att: [90, 0, 90], setup_seq: "XYZ",
            };
            const valid = {
                device_id: 1, q: pitchQuat(-35), flags: 32,
                failure_flags: 0, stale: false,
                fov_h_rad: 0.42, fov_v_rad: 0.24, optics_stale: false,
                zoom_level: 1,
            };
            const invalid = { ...valid, stale: true };
            const staticFp = computeStaticCameraFootprint(32.0, 34.8, 200, 0, 0, 0, cam);
            const live = selectGimbalTelemetry({ gimbals: { "1": valid } }, cam);
            const rejectedTelemetry = selectGimbalTelemetry({ gimbals: { "1": invalid } }, cam);
            const liveFp = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 0, 0, cam, live
            );
            const missing = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 0, 0, cam, null
            );
            const rejected = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 0, 0, cam, rejectedTelemetry
            );
            console.log(JSON.stringify({ staticFp, liveFp, missing, rejected }));
        """)
        self.assert_footprints_close(result["liveFp"], result["staticFp"])
        self.assertIsNone(result["missing"])
        self.assertIsNone(result["rejected"])

    def test_scaled_quaternion_is_normalized(self):
        """Scaled unit quaternions project the same as normalized quaternions."""
        result = _run_js("""
            function pitchQuat(deg) {
                const h = deg * Math.PI / 360;
                return [Math.cos(h), 0, Math.sin(h), 0];
            }
            const cam = {
                fovH: 0.42, fovV: 0.24, pitchDeg: -35,
                imageHeight: 1080,
                gimbal_device_id: 1, setup_att: [90, 0, 90], setup_seq: "XYZ",
            };
            const q = pitchQuat(-35);
            const unitTelemetry = {
                device_id: 1, q, flags: 32, failure_flags: 0, stale: false,
                fov_h_rad: 0.42, fov_v_rad: 0.24, optics_stale: false,
                zoom_level: 1,
            };
            const scaledTelemetry = {
                device_id: 1, q: q.map((v) => v * 3), flags: 32,
                failure_flags: 0, stale: false,
                fov_h_rad: 0.42, fov_v_rad: 0.24, optics_stale: false,
                zoom_level: 1,
            };
            const unitLive = selectGimbalTelemetry({ gimbals: { "1": unitTelemetry } }, cam);
            const scaledLive = selectGimbalTelemetry({ gimbals: { "1": scaledTelemetry } }, cam);
            const unitFp = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 0, 0, cam, unitLive
            );
            const scaledFp = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 0, 0, cam, scaledLive
            );
            console.log(JSON.stringify({ unitFp, scaledFp }));
        """)
        self.assert_footprints_close(result["scaledFp"], result["unitFp"])

    def test_multi_device_gimbal_mapping_uses_device_id(self):
        """Vehicle gimbals map to devices by gimbal_device_id, not device array index."""
        result = _run_js("""
            function pitchQuat(deg) {
                const h = deg * Math.PI / 360;
                return [Math.cos(h), 0, Math.sin(h), 0];
            }
            const base = { fovH: 0.42, fovV: 0.24, imageHeight: 1080, setup_att: [90, 0, 90], setup_seq: "XYZ" };
            const devA = { ...base, name: "a", pitchDeg: -35, gimbal_device_id: 2 };
            const devB = { ...base, name: "b", pitchDeg: -35, gimbal_device_id: 4 };
            const vehicle = {
                gimbals: {
                    "2": {
                        device_id: 2, q: pitchQuat(-20), flags: 32, failure_flags: 0, stale: false,
                        fov_h_rad: 0.42, fov_v_rad: 0.24, optics_stale: false, zoom_level: 1,
                    },
                    "4": {
                        device_id: 4, q: pitchQuat(-60), flags: 32, failure_flags: 0, stale: false,
                        fov_h_rad: 0.42, fov_v_rad: 0.24, optics_stale: false, zoom_level: 1,
                    },
                },
            };
            const tA = selectGimbalTelemetry(vehicle, devA);
            const tB = selectGimbalTelemetry(vehicle, devB);
            const fpA = computeLiveGimbalCameraFootprint(32.0, 34.8, 200, 0, 0, 0, devA, tA);
            const fpB = computeLiveGimbalCameraFootprint(32.0, 34.8, 200, 0, 0, 0, devB, tB);
            console.log(JSON.stringify({
                ids: [tA.device_id, tB.device_id],
                signatures: [gimbalTelemetrySignature(tA), gimbalTelemetrySignature(tB)],
                fpA,
                fpB,
            }));
        """)
        self.assertEqual(result["ids"], [2, 4])
        self.assertNotEqual(result["signatures"][0], result["signatures"][1])
        diff = abs(self._centroid_lat(result["fpA"]) - self._centroid_lat(result["fpB"]))
        self.assertGreater(diff, 0.0001)

    def test_live_zoom_fov_narrows_footprint(self):
        """Live MAVLink FOV, not configured profile FOV, controls footprint size."""
        result = _run_js("""
            function pitchQuat(deg) {
                const h = deg * Math.PI / 360;
                return [Math.cos(h), 0, Math.sin(h), 0];
            }
            function area(points) {
                const lat0 = 32.0 * Math.PI / 180;
                const earth = 6371000;
                const xy = points.map((p) => ({
                    x: (p.lon - 34.8) * Math.PI / 180 * earth * Math.cos(lat0),
                    y: (p.lat - 32.0) * Math.PI / 180 * earth,
                }));
                let a = 0;
                for (let i = 0; i < xy.length; i += 1) {
                    const j = (i + 1) % xy.length;
                    a += xy[i].x * xy[j].y - xy[j].x * xy[i].y;
                }
                return Math.abs(a) / 2;
            }
            const cam = {
                fovH: 0.82, fovV: 0.47, pitchDeg: -35,
                imageHeight: 1080,
                gimbal_device_id: 1,
                footprintSource: "live-gimbal",
                setup_att: [90, 0, 90],
                setup_seq: "XYZ",
            };
            const base = {
                device_id: 1, q: pitchQuat(-35), flags: 32,
                failure_flags: 0, stale: false, optics_stale: false,
            };
            const wide = selectGimbalTelemetry({ gimbals: { "1": {
                ...base, fov_h_rad: 0.82, fov_v_rad: 0.47, zoom_level: 1,
            }}}, cam);
            const narrow = selectGimbalTelemetry({ gimbals: { "1": {
                ...base, fov_h_rad: 0.22, fov_v_rad: 0.12, zoom_level: 5,
            }}}, cam);
            const wideCam = resolveLiveGimbalCameraConfig(cam, wide);
            const narrowCam = resolveLiveGimbalCameraConfig(cam, narrow);
            const wideFp = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 0, 0, wideCam, wide
            );
            const narrowFp = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 0, 0, narrowCam, narrow
            );
            console.log(JSON.stringify({
                wideArea: area(wideFp),
                narrowArea: area(narrowFp),
                ranges: [wideCam.maxDetectDist, narrowCam.maxDetectDist],
                zooms: [wide.zoomLevel, narrow.zoomLevel],
            }));
        """)
        self.assertEqual(result["zooms"], [1, 5])
        self.assertGreater(result["ranges"][1], result["ranges"][0])
        self.assertLess(result["narrowArea"], result["wideArea"] * 0.4)

    def test_live_detect_range_uses_smallest_plan_class(self):
        """Live detection envelope uses the smallest selected vehicle-plan class."""
        result = _run_js(f"""
            const fovV = 0.47;
            const imageHeight = 1080;
            const fy = imageHeight / (2 * Math.tan(fovV / 2));
            const mediumOnly = computeLiveDetectRangeFromFov(fovV, imageHeight, ["medium"]);
            const mixedPlan = computeLiveDetectRangeFromFov(fovV, imageHeight, ["medium", "small"]);
            const fallback = computeLiveDetectRangeFromFov(fovV, imageHeight, []);
            console.log(JSON.stringify({{
                mediumOnly,
                mixedPlan,
                fallback,
                expectedMediumPreset: fy * {CLASS_0_SIZE} / 8,
                expectedSmallPreset: fy * {CLASS_4_SIZE} / 8,
            }}));
        """)
        self.assertAlmostEqual(result["mediumOnly"], result["expectedMediumPreset"], places=6)
        self.assertAlmostEqual(result["mixedPlan"], result["expectedSmallPreset"], places=6)
        self.assertAlmostEqual(result["fallback"], result["expectedSmallPreset"], places=6)

    def test_live_zoom_only_change_updates_signature(self):
        """Coverage sampling notices zoom/FOV changes even when q is unchanged."""
        result = _run_js("""
            const q = [1, 0, 0, 0];
            const a = {
                device_id: 1, q, flags: 32, failure_flags: 0,
                fovH: 0.82, fovV: 0.47, zoomLevel: 1,
            };
            const b = {
                ...a,
                fovH: 0.22,
                fovV: 0.12,
                zoomLevel: 5,
            };
            console.log(JSON.stringify({
                sameQ: a.q.join(",") === b.q.join(","),
                sigA: gimbalTelemetrySignature(a),
                sigB: gimbalTelemetrySignature(b),
            }));
        """)
        self.assertTrue(result["sameQ"])
        self.assertNotEqual(result["sigA"], result["sigB"])

    def test_missing_live_optics_fails_closed(self):
        """Live footprint config is unavailable without fresh MAVLink optics."""
        result = _run_js("""
            function pitchQuat(deg) {
                const h = deg * Math.PI / 360;
                return [Math.cos(h), 0, Math.sin(h), 0];
            }
            const cam = {
                fovH: 0.42, fovV: 0.24, pitchDeg: -35,
                imageHeight: 1080,
                gimbal_device_id: 1,
                footprintSource: "live-gimbal",
                setup_att: [90, 0, 90],
                setup_seq: "XYZ",
            };
            const noFov = {
                device_id: 1, q: pitchQuat(-35), flags: 32,
                failure_flags: 0, stale: false,
            };
            const staleFov = {
                ...noFov,
                fov_h_rad: 0.42,
                fov_v_rad: 0.24,
                optics_stale: true,
            };
            const badHeight = {
                ...cam,
                imageHeight: undefined,
            };
            const noFovLive = selectGimbalTelemetry({ gimbals: { "1": noFov } }, cam);
            const staleLive = selectGimbalTelemetry({ gimbals: { "1": staleFov } }, cam);
            const validLive = selectGimbalTelemetry({ gimbals: { "1": {
                ...noFov,
                fov_h_rad: 0.42,
                fov_v_rad: 0.24,
                optics_stale: false,
            }}}, cam);
            console.log(JSON.stringify({
                noFovLive,
                staleLive,
                badConfig: resolveLiveGimbalCameraConfig(badHeight, validLive),
            }));
        """)
        self.assertIsNone(result["noFovLive"])
        self.assertIsNone(result["staleLive"])
        self.assertIsNone(result["badConfig"])


class TestLiveGimbalEndToEndContract(unittest.TestCase):
    """Pin the publisher -> backend snapshot -> JS footprint quaternion contract."""

    def test_published_snapshot_quaternion_projects_live_footprint(self):
        from gcs.backend.companion_identity import COMPANION_COMPONENT_ID
        from gcs.backend.vehicle_manager import VehicleEntry
        from navpy.modules.common.models.attitude import Attitude
        from navpy.modules.vision.camera_mount import CameraMountOptics
        from navpy.modules.vision import vision_profiles
        from navpy.modules.vision.gimbal_telemetry_publisher import GimbalTelemetryPublisher
        from navpy.modules.vision.peripheral.gimbal_abc import GimbalData

        class FakeMount:
            def get_gimbal_data(self):
                return GimbalData(att=Attitude(-35, 0, 0))

            def get_live_optics(self):
                return CameraMountOptics(
                    zoom_command="2.0",
                    zoom_level="2.0",
                    sample_id="sample-1",
                    fov_h_rad=0.42,
                    fov_v_rad=0.24,
                )

        class FakeSender:
            def __init__(self):
                self.messages = []

            def send_mavlink_message(self, msg, *, source_component=None):
                # The companion shares its aircraft's system id (1 here) and
                # stamps component 191 by default, overriding it with the
                # camera component for optics. Simulate both so VehicleEntry's
                # source filter accepts this vehicle's own gimbal telemetry.
                component = (
                    COMPANION_COMPONENT_ID if source_component is None
                    else source_component
                )
                msg.get_srcComponent = lambda: component
                msg.get_srcSystem = lambda: 1
                self.messages.append(msg)

        class FakeVehicle:
            def on_message(self, *_args):
                pass

        sender = FakeSender()
        spec = vision_profiles.CameraMountSpec(
            mount=FakeMount(),
            device={},
            gimbal_device_id=1,
            profile_device_index=0,
        )
        GimbalTelemetryPublisher(sender, [spec], Mock()).publish_once()

        entry = VehicleEntry(1, FakeVehicle(), "uav")
        for msg in sender.messages:
            if msg.get_type() == "GIMBAL_DEVICE_ATTITUDE_STATUS":
                entry._on_gimbal_device_attitude_status(msg)
            elif msg.get_type() == "CAMERA_FOV_STATUS":
                entry._on_camera_fov_status(msg)
            elif msg.get_type() == "CAMERA_SETTINGS":
                entry._on_camera_settings(msg)
        gimbals = entry._gimbal_snapshot()

        result = _run_js(f"""
            const cam = {{
                fovH: 0.42, fovV: 0.24, pitchDeg: -35,
                imageHeight: 1080,
                gimbal_device_id: 1,
                footprintSource: "live-gimbal",
                setup_att: [90, 0, 90],
                setup_seq: "XYZ",
            }};
            const vehicle = {{ gimbals: {json.dumps(gimbals)} }};
            const live = selectGimbalTelemetry(vehicle, cam);
            const liveCam = resolveLiveGimbalCameraConfig(cam, live);
            const liveFp = computeLiveGimbalCameraFootprint(
                32.0, 34.8, 200, 0, 0, 0, liveCam, live
            );
            const staticFp = computeStaticCameraFootprint(32.0, 34.8, 200, 0, 0, 0, cam);
            console.log(JSON.stringify({{
                live,
                liveCam,
                liveFp,
                staticFp,
            }}));
        """)

        self.assertIsNotNone(result["live"])
        self.assertIsNotNone(result["liveCam"])
        self.assertEqual(result["live"]["zoomLevel"], 2.0)
        self.assertIsNotNone(result["liveFp"])
        self.assertEqual(len(result["liveFp"]), len(result["staticFp"]))
        for live_pt, static_pt in zip(result["liveFp"], result["staticFp"]):
            self.assertAlmostEqual(live_pt["lat"], static_pt["lat"], places=8)
            self.assertAlmostEqual(live_pt["lon"], static_pt["lon"], places=8)


class TestCameraHitsGround(unittest.TestCase):
    """The shared ground-intersection gate — used by BOTH the settings diagram
    and the live map footprint so a camera that does not look at the ground
    leaves no footprint in either view."""

    def _hits(self, pitch, fov_v, alt, mdd):
        return _run_js(f"""
            console.log(JSON.stringify(
                cameraHitsGround({pitch}, {fov_v}, {alt}, {mdd})));
        """)

    def test_down_camera_reaches_ground(self):
        # -30deg, ~28.6deg FOV, 150m, 1000m range: steepest ray hits ~215m.
        self.assertTrue(self._hits(-30, 0.5, 150, 1000))

    def test_forward_camera_does_not_reach_ground_in_range(self):
        # +2deg, ~19.5deg FOV, 150m, 736m range: steepest ray hits ~1113m > range.
        self.assertFalse(self._hits(2, 0.34, 150, 736))

    def test_forward_camera_reaches_with_huge_range(self):
        # Same shallow camera but an unbounded range DOES reach the ground.
        self.assertTrue(self._hits(2, 0.34, 150, 1e9))

    def test_steepest_ray_above_horizon_is_false(self):
        # Whole FOV above the horizon (looking up) — never hits the ground.
        self.assertFalse(self._hits(10, 0.1, 150, 1e9))

    def test_nonpositive_altitude_or_range_is_false(self):
        self.assertFalse(self._hits(-30, 0.5, 0, 1000))
        self.assertFalse(self._hits(-30, 0.5, 150, 0))


class TestDiagramRangeComposition(unittest.TestCase):
    """Regression guard: the DetectionRangeDiagram's effective range is
    computeMaxDetectDist (CONFIRM base) × detectRangeScale, and that product MUST
    equal the DETECTION range of the selected POI (fy·poiSize/MIN_DETECT_PIXELS)
    — the same range the live map footprint uses. computeMaxDetectDist must stay on
    MIN_CONFIRM_PIXELS or this product double-counts the 20/8 factor (2.5× too far)."""

    def test_composed_diagram_range_equals_detection_range(self):
        medium = get_class_detect_size(0)
        person = get_class_detect_size(4)   # the planner's MIN_CLASS_SIZE
        result = _run_js(f"""
            configureDetectorClassDimensions({json.dumps(_BACKEND_DOCK_CLASSES)});
            const fy = 2000;
            const base = computeMaxDetectDist(fy, 1920, 1080, 2.0, 640);  // confirm base
            const scale = detectRangeScale({medium}, getMinClassSize());     // selected = Medium
            console.log(JSON.stringify({{ composed: base * scale }}));
        """)
        # Must equal the detection range of the SELECTED POI (Medium), ÷8.
        self.assertAlmostEqual(result["composed"], 2000 * medium / 8, places=2)
        # And NOT the 2.5× double-counted value the reverted ÷8 change produced.
        self.assertNotAlmostEqual(result["composed"], 2000 * medium / 8 * 20 / 8, places=1)


class TestPadFootprintLoop(unittest.TestCase):
    """padFootprintLoop gives the live footprint a CONSTANT vertex count (with
    invisible duplicate vertices) so the interpolating animation never snaps on a
    horizon-clip vertex-count change (the turn jitter) — while keeping the polygon
    SIMPLE and the real corners SHARP (no rounding)."""

    def _pad(self, loop, n):
        return _run_js(f"""
            console.log(JSON.stringify(padFootprintLoop({json.dumps(loop)}, {n})));
        """)

    @staticmethod
    def _pt(lon, lat):
        return {"lon": lon, "lat": lat}

    def test_pads_short_loops_to_constant_count(self):
        tri = [self._pt(0, 0), self._pt(1, 0), self._pt(0, 1)]
        quad = tri + [self._pt(1, 1)]
        five = quad + [self._pt(0.5, -0.5)]
        for loop in (tri, quad, five):
            self.assertEqual(len(self._pad(loop, 6)), 6)

    def test_pad_repeats_last_vertex(self):
        quad = [self._pt(0, 0), self._pt(1, 0), self._pt(1, 1), self._pt(0, 1)]
        out = self._pad(quad, 6)
        self.assertEqual(out[:4], quad)            # real corners untouched (sharp)
        self.assertEqual(out[4], quad[-1])         # padding duplicates the last vertex
        self.assertEqual(out[5], quad[-1])

    def test_already_long_enough_unchanged(self):
        loop = [self._pt(i, 0) for i in range(6)]
        self.assertEqual(self._pad(loop, 6), loop)

    def test_padding_preserves_unique_corners(self):
        # The visible shape is the set of UNIQUE vertices — padding must not add,
        # move, or round any real corner (the operator rejected rounded corners).
        quad = [self._pt(0, 0), self._pt(1, 0), self._pt(1, 1), self._pt(0, 1)]
        out = self._pad(quad, 6)
        unique = [dict(t) for t in {tuple(sorted(p.items())) for p in out}]
        self.assertEqual(len(unique), 4)

    def test_padding_keeps_polygon_simple(self):
        quad = [self._pt(0, 0), self._pt(1, 0), self._pt(1, 1), self._pt(0, 1)]
        self.assertTrue(_polygon_is_simple(self._pad(quad, 6)))

    def test_swept_roll_padded_count_is_constant_and_simple(self):
        # End-to-end no-jitter contract: a down camera swept through a bank emits a
        # VARYING clip count (3..5), but padded to 6 every drawn footprint has the
        # SAME length (no animation snap) and stays simple (no bow-tie).
        result = _run_js("""
            const cam = { fovH: 0.9, fovV: 0.5, pitchDeg: -16 };
            const out = [];
            for (let roll = -55; roll <= 55; roll += 5) {
                const fp = computeCameraFootprint(32.0, 34.8, 150, 0, -3, roll, cam);
                out.push(fp ? padFootprintLoop(fp, 6) : null);
            }
            console.log(JSON.stringify(out));
        """)
        drew = [fp for fp in result if fp is not None]
        self.assertGreater(len(drew), 10)
        self.assertTrue(all(len(fp) == 6 for fp in drew),
                        "padded footprint length is not constant (would snap/jitter)")
        for idx, fp in enumerate(drew):
            self.assertTrue(_polygon_is_simple(fp),
                            f"padded footprint self-intersects at index {idx}: {fp}")


if __name__ == "__main__":
    unittest.main()
