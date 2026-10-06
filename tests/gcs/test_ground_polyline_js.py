"""Tests for groundPolyline.js — cross-browser-safe ground-following polylines.

Cesium's native clampToGround is not reliable on every browser/GPU, so this module keeps
native clampToGround where supported and falls back to a terrain-sampled,
lifted absolute-height polyline where it isn't.
"""
import json
import os
import re
from tests.gcs.js_runner import run_node
import unittest

_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "map", "utils",
))
_SRC_UTILS_DIR = os.path.normpath(os.path.join(
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
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_GEO_JS = _strip_es_modules(
    open(os.path.join(_SRC_UTILS_DIR, "geo.js"), encoding="utf-8").read()
)
_GROUND_POLYLINE_JS = _strip_es_modules(
    open(os.path.join(_UTILS_DIR, "groundPolyline.js"), encoding="utf-8").read()
)
_MODULE_SRC = _GEO_JS + "\n" + _GROUND_POLYLINE_JS

# Minimal hand-written Cesium stubs — no real WGS84 math, just enough of the
# API surface (and a coherent fromDegrees <-> fromCartesian round-trip via
# stashed properties) to exercise this module's logic in Node.
_CESIUM_STUB = """\
function _Cartesian3(x, y, z) { this.x = x || 0; this.y = y || 0; this.z = z || 0; }
_Cartesian3.fromDegrees = function(lon, lat, height) {
  const c = new _Cartesian3(lon, lat, height || 0);
  c.__lon = lon; c.__lat = lat; c.__height = height || 0;
  return c;
};
const _Cartographic = {
  fromCartesian: function(c) {
    return { longitude: (c.__lon ?? c.x) * Math.PI / 180, latitude: (c.__lat ?? c.y) * Math.PI / 180 };
  },
  fromDegrees: function(lon, lat) {
    return { longitude: lon * Math.PI / 180, latitude: lat * Math.PI / 180, __lon: lon, __lat: lat };
  },
};
function _CallbackProperty(callback, isConstant) { this._callback = callback; this.isConstant = isConstant; }
_CallbackProperty.prototype.getValue = function() { return this._callback(); };
const Cesium = {
  Cartesian3: _Cartesian3,
  Cartographic: _Cartographic,
  CallbackProperty: _CallbackProperty,
  Math: { toDegrees: (rad) => rad * 180 / Math.PI },
  Entity: { supportsPolylinesOnTerrain: (scene) => !!scene.__supported },
  sampleTerrainMostDetailed: () => Promise.resolve([]),
};
"""


def _run_js(script):
    code = _CESIUM_STUB + "\n" + _MODULE_SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _run_json(script):
    return json.loads(_run_js(script))


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

class TestToLonLat(unittest.TestCase):
    def test_plain_lon_lat_passthrough(self):
        out = _run_json("console.log(JSON.stringify(toLonLat({ lon: 34.5, lat: 32.1 }, Cesium)));")
        self.assertEqual(out, {"lon": 34.5, "lat": 32.1})

    def test_cartesian3_round_trip(self):
        out = _run_json("""
            const c = Cesium.Cartesian3.fromDegrees(34.5, 32.1, 150);
            console.log(JSON.stringify(toLonLat(c, Cesium)));
        """)
        self.assertAlmostEqual(out["lon"], 34.5, places=6)
        self.assertAlmostEqual(out["lat"], 32.1, places=6)


class TestDensifyLonLatPath(unittest.TestCase):
    def test_short_segment_untouched(self):
        # ~11m apart (well under the 75m default) — no extra points inserted.
        out = _run_json("""
            const pts = [{ lon: 34.0, lat: 32.0 }, { lon: 34.0001, lat: 32.0 }];
            console.log(JSON.stringify(densifyLonLatPath(pts)));
        """)
        self.assertEqual(len(out), 2)

    def test_long_segment_gets_intermediate_points(self):
        # ~1.1km apart — must be subdivided under the 75m default.
        out = _run_json("""
            const pts = [{ lon: 34.0, lat: 32.0 }, { lon: 34.01, lat: 32.0 }];
            console.log(JSON.stringify(densifyLonLatPath(pts)));
        """)
        self.assertGreater(len(out), 10)
        # Endpoints preserved exactly.
        self.assertAlmostEqual(out[0]["lon"], 34.0)
        self.assertAlmostEqual(out[-1]["lon"], 34.01)

    def test_loop_closes_path(self):
        out = _run_json("""
            const pts = [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }, { lon: 34.001, lat: 32.001 }];
            console.log(JSON.stringify(densifyLonLatPath(pts, { loop: true })));
        """)
        self.assertAlmostEqual(out[0]["lon"], out[-1]["lon"])
        self.assertAlmostEqual(out[0]["lat"], out[-1]["lat"])

    def test_no_loop_leaves_path_open(self):
        out = _run_json("""
            const pts = [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }, { lon: 34.001, lat: 32.001 }];
            console.log(JSON.stringify(densifyLonLatPath(pts, { loop: false })));
        """)
        self.assertNotAlmostEqual(out[-1]["lat"], out[0]["lat"], places=6)

    def test_fewer_than_two_points_passthrough(self):
        out = _run_json("console.log(JSON.stringify(densifyLonLatPath([{ lon: 1, lat: 2 }])));")
        self.assertEqual(out, [{"lon": 1, "lat": 2}])


class TestResolveFallbackHeight(unittest.TestCase):
    def test_prefers_live_height(self):
        out = _run_js("console.log(resolveFallbackHeight({ liveHeight: 100, cachedHeight: 200 }));")
        self.assertEqual(float(out), 102.0)

    def test_live_height_zero_is_used_not_treated_as_missing(self):
        """0m elevation (sea level) must not be treated as falsy/missing."""
        out = _run_js("console.log(resolveFallbackHeight({ liveHeight: 0, cachedHeight: 200 }));")
        self.assertEqual(float(out), 2.0)

    def test_falls_back_to_cached_when_live_missing(self):
        out = _run_js("console.log(resolveFallbackHeight({ liveHeight: undefined, cachedHeight: 50 }));")
        self.assertEqual(float(out), 52.0)

    def test_falls_back_to_zero_when_both_missing(self):
        out = _run_js("console.log(resolveFallbackHeight({ liveHeight: undefined, cachedHeight: undefined }));")
        self.assertEqual(float(out), 2.0)

    def test_ignores_nan(self):
        out = _run_js("console.log(resolveFallbackHeight({ liveHeight: NaN, cachedHeight: 50 }));")
        self.assertEqual(float(out), 52.0)

    def test_custom_lift(self):
        out = _run_js("console.log(resolveFallbackHeight({ liveHeight: 10, cachedHeight: null, liftM: 5 }));")
        self.assertEqual(float(out), 15.0)


class TestRoundCacheKey(unittest.TestCase):
    def test_same_point_same_key(self):
        out = _run_json("console.log(JSON.stringify([roundCacheKey(34.5, 32.1), roundCacheKey(34.5, 32.1)]));")
        self.assertEqual(out[0], out[1])

    def test_different_points_different_keys(self):
        out = _run_json("console.log(JSON.stringify([roundCacheKey(34.5, 32.1), roundCacheKey(34.6, 32.1)]));")
        self.assertNotEqual(out[0], out[1])


# ---------------------------------------------------------------------------
# Wiring: addGroundPolyline / removeGroundPolyline
# ---------------------------------------------------------------------------

def _make_viewer_script(supported, live_height="undefined"):
    return f"""
        const added = [];
        const viewer = {{
          terrainProvider: {{ id: 'fake-provider' }},
          scene: {{
            __supported: {str(supported).lower()},
            globe: {{ getHeight: () => {live_height} }},
            requestRender: () => {{}},
          }},
          entities: {{
            add: (opts) => {{ const e = {{ __opts: opts }}; added.push(e); return e; }},
            remove: () => {{}},
          }},
        }};
    """


class TestAddGroundPolylineSupported(unittest.TestCase):
    def test_uses_native_clamp_to_ground(self):
        out = _run_json(_make_viewer_script(supported=True) + """
            const entity = addGroundPolyline(viewer, Cesium, {
              getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }],
              width: 2, material: 'red',
            });
            console.log(JSON.stringify({
              clampToGround: entity.__opts.polyline.clampToGround,
              positionCount: entity.__opts.polyline.positions.getValue().length,
            }));
        """)
        self.assertTrue(out["clampToGround"])
        self.assertEqual(out["positionCount"], 2)

    def test_loop_closes_positions_on_native_path(self):
        out = _run_json(_make_viewer_script(supported=True) + """
            const entity = addGroundPolyline(viewer, Cesium, {
              getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }, { lon: 34.001, lat: 32.001 }],
              width: 2, material: 'red', loop: true,
            });
            const positions = entity.__opts.polyline.positions.getValue();
            console.log(JSON.stringify({
              count: positions.length,
              firstEqualsLast: positions[0].__lon === positions[positions.length - 1].__lon,
            }));
        """)
        self.assertEqual(out["count"], 4)
        self.assertTrue(out["firstEqualsLast"])

    def test_empty_positions_returns_empty(self):
        out = _run_json(_make_viewer_script(supported=True) + """
            const entity = addGroundPolyline(viewer, Cesium, {
              getPositions: () => [],
              width: 2, material: 'red',
            });
            console.log(JSON.stringify(entity.__opts.polyline.positions.getValue()));
        """)
        self.assertEqual(out, [])


class TestAddGroundPolylineFallback(unittest.TestCase):
    def test_disables_clamp_to_ground_when_unsupported(self):
        out = _run_json(_make_viewer_script(supported=False) + """
            const entity = addGroundPolyline(viewer, Cesium, {
              getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }],
              width: 2, material: 'red',
            });
            console.log(JSON.stringify({ clampToGround: entity.__opts.polyline.clampToGround }));
        """)
        self.assertFalse(out["clampToGround"])

    def test_renders_lifted_height_when_terrain_not_loaded(self):
        """globe.getHeight() returning undefined (tile not loaded) must not
        make the fallback line disappear — it renders at the lift height
        above sea level rather than not rendering at all."""
        out = _run_json(_make_viewer_script(supported=False, live_height="undefined") + """
            const entity = addGroundPolyline(viewer, Cesium, {
              getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }],
              width: 2, material: 'red',
            });
            const positions = entity.__opts.polyline.positions.getValue();
            console.log(JSON.stringify(positions.map(p => p.__height)));
        """)
        self.assertTrue(all(h == 2.0 for h in out))

    def test_uses_live_terrain_height_when_available(self):
        out = _run_json(_make_viewer_script(supported=False, live_height="321") + """
            const entity = addGroundPolyline(viewer, Cesium, {
              getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }],
              width: 2, material: 'red',
            });
            const positions = entity.__opts.polyline.positions.getValue();
            console.log(JSON.stringify(positions.map(p => p.__height)));
        """)
        self.assertTrue(all(h == 323.0 for h in out))

    def test_densifies_long_segments(self):
        out = _run_json(_make_viewer_script(supported=False) + """
            const entity = addGroundPolyline(viewer, Cesium, {
              getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.01, lat: 32.0 }],
              width: 2, material: 'red',
            });
            const positions = entity.__opts.polyline.positions.getValue();
            console.log(positions.length);
        """)
        self.assertGreater(int(out), 10)

    def test_no_terrain_provider_yet_does_not_sample(self):
        """viewer.terrainProvider is undefined for ~1s after viewer creation
        (before Ion terrain replaces the initial provider). Must not call
        sampleTerrainMostDetailed(undefined, ...) — real Cesium throws on a
        missing provider argument."""
        out = _run_json(_make_viewer_script(supported=False) + """
            viewer.terrainProvider = undefined;
            let sampleCalls = 0;
            Cesium.sampleTerrainMostDetailed = () => { sampleCalls++; return Promise.resolve([]); };
            const entity = addGroundPolyline(viewer, Cesium, {
              getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }],
              width: 2, material: 'red',
            });
            entity.__opts.polyline.positions.getValue();
            entity.__opts.polyline.positions.getValue();
            console.log(JSON.stringify({ sampleCalls }));
        """)
        self.assertEqual(out["sampleCalls"], 0)


class TestFallbackAsyncSampling(unittest.TestCase):
    """Real-timer tests for the debounce/provider-generation fix (found in
    Codex post-step review): a continuously re-evaluated CallbackProperty
    must not starve sampleTerrainMostDetailed forever, and a sample kicked
    off before a terrain-provider swap must not write stale heights after."""

    def test_repeated_evaluation_of_unchanged_positions_samples_once(self):
        out = _run_json(_make_viewer_script(supported=False) + """
            (async () => {
              let sampleCalls = 0;
              Cesium.sampleTerrainMostDetailed = () => { sampleCalls++; return Promise.resolve([]); };
              const entity = addGroundPolyline(viewer, Cesium, {
                getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }],
                width: 2, material: 'red',
              });
              // Simulate ~10 rendered frames at the same (unchanged) positions,
              // as would happen while the scene clock ticks with nothing moving.
              for (let i = 0; i < 10; i++) {
                entity.__opts.polyline.positions.getValue();
                await new Promise((r) => setTimeout(r, 20));
              }
              await new Promise((r) => setTimeout(r, 200));
              console.log(JSON.stringify({ sampleCalls }));
            })();
        """)
        self.assertEqual(out["sampleCalls"], 1)

    def test_changed_positions_reschedule_a_new_sample(self):
        out = _run_json(_make_viewer_script(supported=False) + """
            (async () => {
              let sampleCalls = 0;
              Cesium.sampleTerrainMostDetailed = () => { sampleCalls++; return Promise.resolve([]); };
              const entity = addGroundPolyline(viewer, Cesium, {
                getPositions: () => currentPositions,
                width: 2, material: 'red',
              });
              let currentPositions = [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }];
              entity.__opts.polyline.positions.getValue();
              await new Promise((r) => setTimeout(r, 400));
              currentPositions = [{ lon: 35.0, lat: 33.0 }, { lon: 35.001, lat: 33.0 }];
              entity.__opts.polyline.positions.getValue();
              await new Promise((r) => setTimeout(r, 400));
              console.log(JSON.stringify({ sampleCalls }));
            })();
        """)
        self.assertEqual(out["sampleCalls"], 2)

    def test_stale_sample_dropped_after_provider_swap(self):
        out = _run_json(_make_viewer_script(supported=False) + """
            (async () => {
              // Sample resolves slowly (simulating a real in-flight network
              // request) so we can swap the provider while it's pending.
              Cesium.sampleTerrainMostDetailed = () => new Promise((resolve) => {
                setTimeout(() => resolve([{ height: 999 }, { height: 999 }]), 200);
              });
              const entity = addGroundPolyline(viewer, Cesium, {
                getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }],
                width: 2, material: 'red',
              });
              entity.__opts.polyline.positions.getValue();       // t=0: schedules debounce (fires t=300), provider=A
              await new Promise((r) => setTimeout(r, 350));      // t=350: debounce fired at t=300, sample in flight (resolves t=500)
              viewer.terrainProvider = { id: 'provider-B' };     // swap provider before the stale sample resolves
              entity.__opts.polyline.positions.getValue();       // detects the swap, clears cache, bumps generation
              await new Promise((r) => setTimeout(r, 250));      // t=600: stale sample (t=500) has resolved and must be dropped
              const positions = entity.__opts.polyline.positions.getValue();
              console.log(JSON.stringify(positions.map((p) => p.__height)));
            })();
        """)
        # Stale 999m heights from provider A must not have been written —
        # each point still renders at the lift-only height (no live/cached height).
        self.assertTrue(all(h == 2.0 for h in out))

    def test_provider_swap_reschedules_sample_for_unchanged_positions(self):
        """A provider swap must force a fresh sample even when positions
        themselves haven't changed — the cleared cache needs repopulating
        against the new provider, not left permanently empty because the
        position signature looked "already scheduled"."""
        out = _run_json(_make_viewer_script(supported=False) + """
            (async () => {
              let sampleCalls = 0;
              Cesium.sampleTerrainMostDetailed = () => { sampleCalls++; return Promise.resolve([]); };
              const entity = addGroundPolyline(viewer, Cesium, {
                getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }],
                width: 2, material: 'red',
              });
              entity.__opts.polyline.positions.getValue();   // schedules 1st sample against provider A
              await new Promise((r) => setTimeout(r, 400));  // let it fire
              viewer.terrainProvider = { id: 'provider-B' };
              entity.__opts.polyline.positions.getValue();   // same positions, but provider changed
              await new Promise((r) => setTimeout(r, 400));  // let a 2nd sample fire against provider B
              console.log(JSON.stringify({ sampleCalls }));
            })();
        """)
        self.assertEqual(out["sampleCalls"], 2)


class TestRemoveGroundPolyline(unittest.TestCase):
    def test_calls_entities_remove(self):
        out = _run_json(_make_viewer_script(supported=True) + """
            let removed = null;
            viewer.entities.remove = (e) => { removed = e; };
            const entity = addGroundPolyline(viewer, Cesium, {
              getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }],
              width: 2, material: 'red',
            });
            removeGroundPolyline(viewer, entity);
            console.log(JSON.stringify({ removedIsEntity: removed === entity }));
        """)
        self.assertTrue(out["removedIsEntity"])

    def test_disposes_fallback_pending_timer_without_throwing(self):
        out = _run_js(_make_viewer_script(supported=False) + """
            const entity = addGroundPolyline(viewer, Cesium, {
              getPositions: () => [{ lon: 34.0, lat: 32.0 }, { lon: 34.001, lat: 32.0 }],
              width: 2, material: 'red',
            });
            entity.__opts.polyline.positions.getValue();
            removeGroundPolyline(viewer, entity);
            console.log('ok');
        """)
        self.assertEqual(out, "ok")

    def test_noop_on_null_entity(self):
        out = _run_js(_make_viewer_script(supported=True) + """
            removeGroundPolyline(viewer, null);
            console.log('ok');
        """)
        self.assertEqual(out, "ok")


if __name__ == "__main__":
    unittest.main()
