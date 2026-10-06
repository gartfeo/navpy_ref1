"""Tests for ProfileSelector pure JS helpers."""
import json
import os
import re
from tests.gcs.js_runner import run_node
import unittest

from navpy.modules.vision.vision_profiles import (
    DETECTOR_CLASS_DIMENSIONS,
    compute_confirm_slant_range,
    get_class_detect_size,
)


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))
_SETTINGS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "settings",
))


def _strip_es_modules(src):
    """Remove ES module syntax so Node.js can eval the helper code."""
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


_JS_FILES = [
    (_UTILS_DIR, "plannerConfig.js"),
    (_SETTINGS_DIR, "profileSelectorUtils.js"),
]

_JS_CODE = ""
for directory, fname in _JS_FILES:
    fpath = os.path.join(directory, fname)
    _JS_CODE += _strip_es_modules(open(fpath, encoding="utf-8").read()) + "\n"


# The backend OWNS per-class characteristic sizes. Mirror its top-level
# `detector_class_dimensions` block (GET /api/vision-profiles) and feed it into the planner
# config so the JS pure helpers use the same single source as Python.
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
    """Run a JS snippet via Node.js and return parsed JSON output."""
    result = run_node(_JS_CODE + "\n" + _CONFIGURE_DOCK_CLASSES_JS + script, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestProfileSelectorUtils(unittest.TestCase):
    def test_build_catalog_device_infos_uses_selected_zoom_range_key(self):
        result = _run_js("""
            const catalog = {
                profiles: {
                    sim: {
                        devices: [{
                            name: "cam",
                            image_width: 1920,
                            image_height: 1080,
                            pitch_deg: -28.7,
                            gimbal_device_id: 3,
                            publishes_gimbal_telemetry: true,
                            setup_att: [90, 0, 90],
                            setup_seq: "XYZ",
                            zooms: {
                                "1": { fx: 2000, fy: 2000, detect_range_m: 740, confirm_range_m: 180 },
                                "2": { fx: 4000, fy: 4000, detect_range_m: 1480, confirm_range_m: 360 },
                            },
                        }],
                    },
                },
            };
            const live = buildCatalogDeviceInfos(catalog, "sim", "cam", "2", "detect_range_m");
            const diagram = buildCatalogDeviceInfos(catalog, "sim", "cam", "2", "confirm_range_m");
            console.log(JSON.stringify({ live, diagram }));
        """)
        self.assertEqual(result["live"][0]["maxDetectDist"], 1480)
        self.assertEqual(result["diagram"][0]["maxDetectDist"], 360)
        self.assertEqual(result["live"][0]["gimbal_device_id"], 3)
        self.assertEqual(result["live"][0]["footprintSource"], "live-gimbal")
        self.assertEqual(result["live"][0]["imageHeight"], 1080)
        self.assertEqual(result["live"][0]["setup_att"], [90, 0, 90])

    def test_build_diagram_devices_uses_active_zoom_confirm_range_for_overrides(self):
        result = _run_js("""
            const catalog = {
                profiles: {
                    sim: {
                        reference_height_m: 2.0,
                        imgsz: 640,
                        devices: [{
                            name: "cam",
                            image_width: 1920,
                            image_height: 1080,
                            pitch_deg: -28.7,
                            zooms: {
                                "1": { fx: 2000, fy: 2000, detect_range_m: 740, confirm_range_m: 180 },
                                "2": { fx: 4000, fy: 4000, detect_range_m: 1480, confirm_range_m: 360 },
                            },
                        }],
                    },
                },
            };
            const localOverrides = {
                profile: {},
                devices: {},
                zooms: {
                    "cam/2": { camera_fy: 4500 },
                },
            };
            const devices = buildDiagramDevices(catalog, "sim", "cam", "2", null, localOverrides);
            console.log(JSON.stringify(devices));
        """)
        self.assertEqual(len(result), 1)
        self.assertAlmostEqual(
            result[0]["maxDetectDist"],
            compute_confirm_slant_range(4500),
            places=6,
        )


class TestDetectorClassDimensionsConfiguredFlag(unittest.TestCase):
    """isDetectorClassDimensionsConfigured() gates operator-authoritative computations
    (ProfileSelector optimize) off the before-fetch fallback."""

    @staticmethod
    def _run_raw(script):
        # No configureDetectorClassDimensions prepend, so we can assert the false->true
        # transition the real fallback-guard depends on.
        result = run_node(_JS_CODE + "\n" + script, timeout=10)
        if result.returncode != 0:
            raise RuntimeError(f"Node.js error:\n{result.stderr}")
        return json.loads(result.stdout.strip())

    def test_false_until_configured_then_true(self):
        result = self._run_raw("""
            const before = isDetectorClassDimensionsConfigured();
            configureDetectorClassDimensions({ "0": { size_m: 4.30 }, "4": { size_m: 1.87 } });
            const after = isDetectorClassDimensionsConfigured();
            console.log(JSON.stringify({ before, after }));
        """)
        self.assertFalse(result["before"])
        self.assertTrue(result["after"])

    def test_stays_false_for_absent_or_empty(self):
        result = self._run_raw("""
            const before = isDetectorClassDimensionsConfigured();
            configureDetectorClassDimensions(null);
            configureDetectorClassDimensions({});
            const after = isDetectorClassDimensionsConfigured();
            console.log(JSON.stringify({ before, after }));
        """)
        self.assertFalse(result["before"])
        self.assertFalse(result["after"])

    def test_stays_configured_after_later_invalid_catalog(self):
        # Once backend sizes are loaded, a later invalid/empty catalog must NOT
        # revert to the JS fallback: last-known-good backend sizes are retained
        # and the flag stays true (matching the map planner). The flag guards
        # against the FALLBACK, not against staleness; detector_class_dimensions is not
        # operator-editable so a valid->invalid transition is not reachable in
        # the UI anyway.
        result = self._run_raw("""
            configureDetectorClassDimensions({ "0": { size_m: 4.30 }, "4": { size_m: 1.87 } });
            const afterValid = isDetectorClassDimensionsConfigured();
            const sizeAfterValid = getClassDetectSize(0);
            configureDetectorClassDimensions({});
            configureDetectorClassDimensions(null);
            const afterInvalid = isDetectorClassDimensionsConfigured();
            const sizeAfterInvalid = getClassDetectSize(0);
            console.log(JSON.stringify({ afterValid, afterInvalid, sizeAfterValid, sizeAfterInvalid }));
        """)
        self.assertTrue(result["afterValid"])
        self.assertTrue(result["afterInvalid"])
        self.assertEqual(result["sizeAfterValid"], 4.30)
        self.assertEqual(result["sizeAfterInvalid"], 4.30)

    def test_js_fallback_mirrors_backend_per_class(self):
        # The JS before-fetch FALLBACK_CLASS_DETECT_SIZES is a second copy of the
        # per-class diagonals; pin it to the backend single source so it cannot
        # silently drift. getClassDetectSize() returns the fallback until
        # configureDetectorClassDimensions runs, so _run_raw (no configure) reads it.
        ids = sorted(DETECTOR_CLASS_DIMENSIONS.keys())
        script = (
            "const out = {};\n"
            + "".join(f"out[{cid}] = getClassDetectSize({cid});\n" for cid in ids)
            + "console.log(JSON.stringify(out));\n"
        )
        result = self._run_raw(script)
        for cid in ids:
            self.assertAlmostEqual(
                result[str(cid)], get_class_detect_size(cid), places=3,
                msg=f"JS fallback for class {cid} drifted from the backend single source",
            )


if __name__ == "__main__":
    unittest.main()
