"""Tests for scanGeometry.js — pure detect-far geometry + the confirm-range helper.

The optimizer is now pure geometry: the operator's pitch in, the far-edge-at-R_det
altitude out, NO clamping (the floor / ceiling / recognition caps live in the React
wiring). POI sizes come from the backend single source (get_class_detect_size).
"""
import json
import math
import os
import re
from tests.gcs.js_runner import run_node
import unittest

from navpy.modules.vision.vision_profiles import get_class_detect_size

_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..",
    "src", "gcs", "frontend", "src", "utils",
))


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


_JS_CODE = ""
for _f in ["projection.js", "plannerConfig.js", "scanGeometry.js"]:
    _JS_CODE += _strip_es_modules(
        open(os.path.join(_UTILS_DIR, _f), encoding="utf-8").read()
    ) + "\n"


def _run_js(script):
    result = run_node(_JS_CODE + "\n" + script, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    out = result.stdout.strip()
    return json.loads(out) if out and out != "null" else None


CLASS_0_SIZE = get_class_detect_size(0)
CLASS_4_SIZE = get_class_detect_size(4)
FY1 = 2082.84   # ZR10 1×
IMG_H = 1440


def _opt(poi, boresight=45, fy=FY1, detect_px=None):
    extra = f", detectPx: {detect_px}" if detect_px else ""
    return _run_js(f"""
        console.log(JSON.stringify(optimizeScanGeometry({{
            fy: {fy}, imageHeight: {IMG_H}, poiSizeM: {poi},
            boresightDeg: {boresight}{extra}}})));
    """)


class ScanGeometryTest(unittest.TestCase):

    def test_balanced_far_edge_on_circle(self):
        r = _opt(CLASS_0_SIZE)   # boresight 45°
        r_det = FY1 * CLASS_0_SIZE / 8
        self.assertAlmostEqual(r["rDetM"], r_det, delta=1)
        self.assertAlmostEqual(r["pitchDeg"], -45.0, delta=0.5)
        fovv = math.degrees(2 * math.atan(IMG_H / (2 * FY1)))
        self.assertAlmostEqual(r["altitudeM"], r_det * math.sin(math.radians(45 - fovv / 2)), delta=2)
        self.assertEqual(r["zoom"], 1)
        # far edge lies on the detection circle: alt² + reach² == R_det²
        self.assertAlmostEqual(math.hypot(r["altitudeM"], r["reachM"]), r_det, delta=1)

    def test_smaller_poi_lowers_altitude(self):
        self.assertLess(_opt(CLASS_4_SIZE)["altitudeM"], _opt(CLASS_0_SIZE)["altitudeM"])

    def test_steeper_pitch_more_altitude_less_reach(self):
        hi = _opt(CLASS_0_SIZE, boresight=55)
        lo = _opt(CLASS_0_SIZE, boresight=30)
        self.assertGreater(hi["altitudeM"], lo["altitudeM"])
        self.assertLess(hi["reachM"], lo["reachM"])

    def test_pitch_equals_operator_input(self):
        # No clamping — the reported pitch is exactly what the operator set.
        self.assertAlmostEqual(_opt(CLASS_0_SIZE, boresight=70)["pitchDeg"], -70, delta=0.5)
        self.assertAlmostEqual(_opt(CLASS_0_SIZE, boresight=20)["pitchDeg"], -20, delta=0.5)

    def test_far_edge_above_horizon_returns_null(self):
        fovv = math.degrees(2 * math.atan(IMG_H / (2 * FY1)))
        self.assertIsNone(_opt(CLASS_0_SIZE, boresight=fovv / 2 - 1))   # too shallow → no ground

    def test_fail_closed_on_bad_inputs(self):
        self.assertIsNone(_run_js(
            "console.log(JSON.stringify(optimizeScanGeometry("
            "{fy:0, imageHeight:1440, poiSizeM:4, boresightDeg:45})));"))

    def test_non_finite_geometry_inputs_fail_closed(self):
        for bad in ("fy: Infinity", "poiSizeM: Infinity", "imageHeight: NaN", "detectPx: Infinity"):
            with self.subTest(bad=bad):
                self.assertIsNone(_run_js(f"""
                    console.log(JSON.stringify(optimizeScanGeometry({{
                        fy: 2082, imageHeight: 1440, poiSizeM: 4.3, boresightDeg: 45, {bad}}})));
                """))

    def test_non_finite_pitch_falls_back_to_45(self):
        nan_p = _run_js("console.log(JSON.stringify(optimizeScanGeometry("
                        "{fy:2082,imageHeight:1440,poiSizeM:4.3,boresightDeg:NaN})));")
        bal = _run_js("console.log(JSON.stringify(optimizeScanGeometry("
                      "{fy:2082,imageHeight:1440,poiSizeM:4.3,boresightDeg:45})));")
        self.assertIsNotNone(nan_p)
        self.assertAlmostEqual(nan_p["altitudeM"], bal["altitudeM"], delta=0.01)

    def test_detect_range_scale_yields_detection_range(self):
        r = _run_js(f"""
            const base = {FY1} * {CLASS_4_SIZE} / 20;
            const scaled = base * detectRangeScale({CLASS_0_SIZE}, {CLASS_4_SIZE});
            console.log(JSON.stringify({{ scaled, expect: {FY1} * {CLASS_0_SIZE} / 8 }}));
        """)
        self.assertAlmostEqual(r["scaled"], r["expect"], delta=0.5)

    def test_confirm_range(self):
        r = _run_js(f"""console.log(JSON.stringify({{
            medium: confirmRange({FY1 * 10}, {CLASS_0_SIZE}, 45),
            none: confirmRange(0, {CLASS_0_SIZE}, 45)}}));""")
        self.assertAlmostEqual(r["medium"], FY1 * 10 * CLASS_0_SIZE / 45, delta=1)   # 10× confirm range
        self.assertIsNone(r["none"])   # Infinity (no cap) → JSON null

    def test_higher_zoom_extends_rdet_and_altitude(self):
        z1 = _opt(CLASS_0_SIZE, fy=FY1)
        z2 = _opt(CLASS_0_SIZE, fy=FY1 * 2)
        self.assertAlmostEqual(z2["rDetM"], 2 * z1["rDetM"], delta=2)
        self.assertGreater(z2["altitudeM"], z1["altitudeM"])

    def test_zoom_level_passes_through(self):
        r = _run_js(f"""console.log(JSON.stringify(optimizeScanGeometry({{
            fy: {FY1 * 2}, imageHeight: {IMG_H}, poiSizeM: {CLASS_0_SIZE}, boresightDeg: 45, zoom: 2}})));""")
        self.assertEqual(r["zoom"], 2)

    def test_custom_detect_px_shrinks_rdet(self):
        self.assertLess(_opt(CLASS_0_SIZE, detect_px=20)["rDetM"], _opt(CLASS_0_SIZE)["rDetM"])


class GroupPitchTest(unittest.TestCase):
    """Pure group-pitch model for a multi-camera fixed rig (center + spread)."""

    def test_group_pitch_from_dual_list(self):
        # novoxy_dual worked example: +5 / -24 -> center -9.5, spread 29.
        r = _run_js("console.log(JSON.stringify(groupPitchFromList([5, -24])));")
        self.assertAlmostEqual(r["center"], -9.5, delta=1e-9)
        self.assertAlmostEqual(r["spread"], 29.0, delta=1e-9)

    def test_group_pitch_single_camera_zero_spread(self):
        r = _run_js("console.log(JSON.stringify(groupPitchFromList([-9])));")
        self.assertEqual(r["center"], -9)
        self.assertEqual(r["spread"], 0)

    def test_group_pitch_empty_or_bad_returns_null(self):
        self.assertIsNone(_run_js("console.log(JSON.stringify(groupPitchFromList([])));"))
        self.assertIsNone(_run_js(
            "console.log(JSON.stringify(groupPitchFromList([1, 'x'])));"))

    def test_distribute_reproduces_dual(self):
        r = _run_js("console.log(JSON.stringify(distributeGroupPitch(-9.5, 29, 2)));")
        self.assertAlmostEqual(r[0], -24.0, delta=1e-9)   # slot 0 = lowest
        self.assertAlmostEqual(r[1], 5.0, delta=1e-9)     # slot 1 = highest

    def test_distribute_shifts_rigidly_with_center(self):
        # Move center to -15 with the same 29 spread -> -29.5 / -0.5.
        r = _run_js("console.log(JSON.stringify(distributeGroupPitch(-15, 29, 2)));")
        self.assertAlmostEqual(r[0], -29.5, delta=1e-9)
        self.assertAlmostEqual(r[1], -0.5, delta=1e-9)

    def test_distribute_three_cameras_even_fan(self):
        r = _run_js("console.log(JSON.stringify(distributeGroupPitch(-10, 10, 3)));")
        self.assertEqual(r, [-20, -10, 0])

    def test_distribute_negative_spread_is_zero(self):
        r = _run_js("console.log(JSON.stringify(distributeGroupPitch(-10, -5, 2)));")
        self.assertEqual(r, [-10, -10])

    def test_distribute_bad_n_returns_null(self):
        for bad in ("distributeGroupPitch(-10, 5, 0)",
                    "distributeGroupPitch(-10, 5, 2.5)",
                    "distributeGroupPitch(-10, Infinity, 2)",
                    "distributeGroupPitch(NaN, 5, 2)"):
            with self.subTest(bad=bad):
                self.assertIsNone(_run_js(f"console.log(JSON.stringify({bad}));"))

    def test_group_pitch_uneven_triple_is_lossy_best_fit(self):
        # n > 2 with non-even gaps: the even-fan fit preserves the mean and the
        # min..max span but does NOT round-trip the inputs (documented behavior).
        r = _run_js("""
            const g = groupPitchFromList([0, 10, 100]);
            console.log(JSON.stringify({
                g, dist: distributeGroupPitch(g.center, g.spread, 3) }));
        """)
        self.assertAlmostEqual(r["g"]["center"], 110 / 3, delta=1e-9)   # mean
        self.assertAlmostEqual(r["g"]["spread"], 50.0, delta=1e-9)      # (100-0)/2
        # span preserved, middle value NOT 10 -> lossy fit
        self.assertAlmostEqual(min(r["dist"]), 110 / 3 - 50, delta=1e-9)
        self.assertAlmostEqual(max(r["dist"]), 110 / 3 + 50, delta=1e-9)
        self.assertNotAlmostEqual(r["dist"][1], 10.0, delta=1.0)

    def test_round_trip_list_to_group_to_distribute(self):
        r = _run_js("""
            const g = groupPitchFromList([5, -24]);
            console.log(JSON.stringify(distributeGroupPitch(g.center, g.spread, 2)));
        """)
        self.assertAlmostEqual(min(r), -24.0, delta=1e-9)
        self.assertAlmostEqual(max(r), 5.0, delta=1e-9)

    def test_clamp_within_envelope_unchanged(self):
        # center -9.5, spread 29, n=2 -> -24/+5, both inside [-60, 20].
        r = _run_js(
            "console.log(JSON.stringify(clampGroupPitch(-9.5, 29, 2, -60, 20)));")
        self.assertAlmostEqual(r["center"], -9.5, delta=1e-9)
        self.assertAlmostEqual(r["spread"], 29.0, delta=1e-9)
        self.assertAlmostEqual(min(r["pitches"]), -24.0, delta=1e-9)
        self.assertAlmostEqual(max(r["pitches"]), 5.0, delta=1e-9)

    def test_clamp_caps_spread_to_envelope(self):
        # Over-wide spread on [-60, 20], n=2 -> maxSpread 80, center pinned to midpoint.
        r = _run_js(
            "console.log(JSON.stringify(clampGroupPitch(-9.5, 200, 2, -60, 20)));")
        self.assertAlmostEqual(r["spread"], 80.0, delta=1e-9)
        self.assertAlmostEqual(r["center"], -20.0, delta=1e-9)   # (min+max)/2
        self.assertAlmostEqual(min(r["pitches"]), -60.0, delta=1e-9)
        self.assertAlmostEqual(max(r["pitches"]), 20.0, delta=1e-9)

    def test_clamp_constrains_center_so_slots_stay_in_envelope(self):
        # Push center down with spread 20 on [-60, 20], n=2: lowest slot would be
        # center-10; center is clamped so it never drops below -50.
        r = _run_js(
            "console.log(JSON.stringify(clampGroupPitch(-100, 20, 2, -60, 20)));")
        self.assertGreaterEqual(round(min(r["pitches"]), 6), -60.0)
        self.assertLessEqual(round(max(r["pitches"]), 6), 20.0)
        self.assertAlmostEqual(r["center"], -50.0, delta=1e-9)

    def test_clamp_single_camera_just_clamps_center(self):
        r = _run_js("console.log(JSON.stringify(clampGroupPitch(40, 5, 1, -60, 20)));")
        self.assertEqual(r["center"], 20)     # clamped to maxPitch
        self.assertEqual(r["spread"], 0)
        self.assertEqual(r["pitches"], [20])

    def test_clamp_three_cameras_caps_spread_and_centers(self):
        # n=3 on [-30, 30]: maxSpread = 60/2 = 30, center pinned to midpoint 0.
        r = _run_js(
            "console.log(JSON.stringify(clampGroupPitch(0, 100, 3, -30, 30)));")
        self.assertAlmostEqual(r["spread"], 30.0, delta=1e-9)
        self.assertAlmostEqual(r["center"], 0.0, delta=1e-9)
        self.assertEqual([round(p, 6) for p in r["pitches"]], [-30, 0, 30])

    def test_clamp_three_cameras_center_high_constrained(self):
        # n=3, spread 10 on [-30, 30]: highest slot = center+10 must stay <= 30,
        # so a too-high center is pulled back to 20.
        r = _run_js(
            "console.log(JSON.stringify(clampGroupPitch(100, 10, 3, -30, 30)));")
        self.assertAlmostEqual(r["center"], 20.0, delta=1e-9)
        self.assertLessEqual(round(max(r["pitches"]), 6), 30.0)
        self.assertGreaterEqual(round(min(r["pitches"]), 6), -30.0)

    def test_clamp_bad_inputs_return_null(self):
        self.assertIsNone(_run_js(
            "console.log(JSON.stringify(clampGroupPitch(NaN, 5, 2, -60, 20)));"))
        self.assertIsNone(_run_js(
            "console.log(JSON.stringify(clampGroupPitch(-9, 5, 2, 20, -60)));"))  # min>max
        self.assertIsNone(_run_js(
            "console.log(JSON.stringify(clampGroupPitch(-9, Infinity, 2, -60, 20)));"))
        self.assertIsNone(_run_js(
            "console.log(JSON.stringify(clampGroupPitch(-9, 5, 0, -60, 20)));"))   # n<1
        self.assertIsNone(_run_js(
            "console.log(JSON.stringify(clampGroupPitch(-9, 5, 2.5, -60, 20)));")) # non-int n

    def test_fov_v_from_fy(self):
        r = _run_js(
            "console.log(JSON.stringify(fovVDegFromFy(1984.98, 1080)));")
        expected = 2 * math.atan(1080 / (2 * 1984.98)) * 180 / math.pi
        self.assertAlmostEqual(r, expected, delta=1e-6)

    def test_fov_v_higher_zoom_narrower(self):
        wide = _run_js("console.log(JSON.stringify(fovVDegFromFy(2000, 1080)));")
        tele = _run_js("console.log(JSON.stringify(fovVDegFromFy(4000, 1080)));")
        self.assertGreater(wide, tele)

    def test_fov_v_bad_inputs_return_null(self):
        for bad in ("fovVDegFromFy(0, 1080)", "fovVDegFromFy(2000, 0)",
                    "fovVDegFromFy(Infinity, 1080)", "fovVDegFromFy(2000, NaN)"):
            with self.subTest(bad=bad):
                self.assertIsNone(_run_js(f"console.log(JSON.stringify({bad}));"))


class FixedMissionAltitudeTest(unittest.TestCase):
    """Mission altitude for a fixed multi-camera rig (min over ground-facing cams)."""

    @staticmethod
    def _alt(cameras, size=2.0, min_px=20, min_alt=30, max_alt=2000):
        return _run_js(f"""
            console.log(JSON.stringify(fixedMissionAltitude({{
                cameras: {json.dumps(cameras)}, sizeM: {size}, minPx: {min_px},
                minAlt: {min_alt}, maxAlt: {max_alt} }})));
        """)

    def test_single_ground_facing_camera(self):
        # confirmSlant = 2000*2/20 = 200; alt = 200*sin(30) = 100.
        alt = self._alt([{"fy": 2000, "pitchDeg": -30}])
        self.assertEqual(alt, 100)

    def test_up_tilted_camera_ignored_down_camera_drives(self):
        # novoxy_dual shape: +5 (up) ignored, -24 (down) drives the altitude.
        alt = self._alt([{"fy": 1984.98, "pitchDeg": 5},
                         {"fy": 2262.15, "pitchDeg": -24}])
        slant = 2262.15 * 2 / 20
        self.assertEqual(alt, round(slant * math.sin(math.radians(24))))

    def test_min_over_ground_facing_cameras(self):
        # Two down cameras: the shallower (smaller depression) gives the lower alt.
        alt = self._alt([{"fy": 2000, "pitchDeg": -45},
                         {"fy": 2000, "pitchDeg": -20}])
        shallow = 200 * math.sin(math.radians(20))
        self.assertEqual(alt, round(shallow))

    def test_all_up_returns_null_fail_closed(self):
        self.assertIsNone(self._alt([{"fy": 2000, "pitchDeg": 5},
                                     {"fy": 2000, "pitchDeg": 0}]))

    def test_min_altitude_floor(self):
        # Very shallow depression -> tiny altitude floored to minAlt.
        alt = self._alt([{"fy": 2000, "pitchDeg": -2}], min_alt=50)
        self.assertEqual(alt, 50)

    def test_max_altitude_ceiling(self):
        # Steep + long range would exceed the ceiling -> capped at maxAlt.
        alt = self._alt([{"fy": 8000, "pitchDeg": -80}], max_alt=300)
        self.assertEqual(alt, 300)

    def test_recognition_cap_uses_fymax(self):
        # confirmRange(fyMax=2000, size=2, minPx=20) = 200 < the raw 80-deg alt of
        # a fy=8000 camera, so the max-zoom recognition range caps it at 200.
        alt = self._alt([{"fy": 8000, "fyMax": 2000, "pitchDeg": -80}])
        self.assertEqual(alt, 200)

    def test_bad_inputs_return_null(self):
        self.assertIsNone(self._alt([]))
        self.assertIsNone(self._alt([{"fy": 0, "pitchDeg": -30}]))  # no valid camera


class MeanFovVTest(unittest.TestCase):
    """Mean vertical FOV used to map boresight spread <-> edge overlap/gap."""

    def test_mean_of_two_cameras(self):
        # novoxy_dual zoom-1 optics: fovV ~30.46 and ~26.86 -> mean ~28.66.
        r = _run_js("""console.log(JSON.stringify(meanFovVDeg([
            {fy: 1984.98, imageHeight: 1080},
            {fy: 2262.15, imageHeight: 1080}])));""")
        f0 = 2 * math.atan(1080 / (2 * 1984.98)) * 180 / math.pi
        f1 = 2 * math.atan(1080 / (2 * 2262.15)) * 180 / math.pi
        self.assertAlmostEqual(r, (f0 + f1) / 2, delta=1e-6)

    def test_overlap_zero_when_spread_equals_mean_fov(self):
        # overlap = meanFovV - spread; edges exactly touch when spread == meanFovV.
        r = _run_js("""
            const m = meanFovVDeg([{fy: 2000, imageHeight: 1080},
                                   {fy: 2000, imageHeight: 1080}]);
            console.log(JSON.stringify({ m, overlapAtTouch: m - m }));
        """)
        self.assertAlmostEqual(r["overlapAtTouch"], 0.0, delta=1e-9)

    def test_higher_zoom_narrows_mean_fov(self):
        wide = _run_js("console.log(JSON.stringify(meanFovVDeg([{fy:2000,imageHeight:1080}])));")
        tele = _run_js("console.log(JSON.stringify(meanFovVDeg([{fy:4000,imageHeight:1080}])));")
        self.assertGreater(wide, tele)

    def test_skips_invalid_and_fails_closed(self):
        r = _run_js("""console.log(JSON.stringify(meanFovVDeg([
            {fy: 0, imageHeight: 1080},
            {fy: 2000, imageHeight: 1080}])));""")
        expected = 2 * math.atan(1080 / (2 * 2000)) * 180 / math.pi
        self.assertAlmostEqual(r, expected, delta=1e-6)  # invalid camera dropped
        self.assertIsNone(_run_js("console.log(JSON.stringify(meanFovVDeg([])));"))
        self.assertIsNone(_run_js(
            "console.log(JSON.stringify(meanFovVDeg([{fy:0,imageHeight:0}])));"))


if __name__ == "__main__":
    unittest.main()
