"""Tests for useSettings planner readiness state."""
import json
import os
import re
from tests.gcs.js_runner import run_node
import unittest


_HOOK_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks",
    "useSettings.js",
))

_HELPER_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks",
    "settingsPlannerState.js",
))


def _strip_es_modules(src):
    """Remove ES module syntax so Node.js can eval the code."""
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


_HELPER_CODE = _strip_es_modules(open(_HELPER_FILE, encoding="utf-8").read())


def _run_js(script):
    """Run a JS snippet via Node.js and return parsed JSON output."""
    result = run_node(_HELPER_CODE + "\n" + script, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestResolvePlannerState(unittest.TestCase):
    def test_ready_when_settings_and_catalog_exist(self):
        result = _run_js("""
            const state = resolvePlannerState(
                { camera: { vision_profile: "zr10" } },
                {
                    profiles: { zr10: { devices: [] } },
                    detector_class_dimensions: {
                        "0": { width_m: 3.5, height_m: 2.5, size_m: 4.30 },
                    },
                },
            );
            console.log(JSON.stringify(state));
        """)
        self.assertTrue(result["plannerReady"])
        self.assertIsNone(result["plannerLoadError"])
        self.assertIn("zr10", result["plannerProfiles"])
        self.assertEqual(result["plannerDetectorClassDimensions"]["0"]["size_m"], 4.30)

    def test_not_ready_when_detector_class_dimensions_missing(self):
        # detector_class_dimensions is the single source for per-class sizes; without it
        # the planner must NOT silently fall back to JS defaults.
        result = _run_js("""
            const state = resolvePlannerState(
                { camera: { vision_profile: "zr10" } },
                { profiles: { zr10: { devices: [] } } },
            );
            console.log(JSON.stringify(state));
        """)
        self.assertFalse(result["plannerReady"])
        self.assertIsNotNone(result["plannerLoadError"])
        self.assertIsNone(result["plannerDetectorClassDimensions"])

    def test_not_ready_when_dock_class_dimensions_missing(self):
        # The dock class (0) is absent — the dock preset would fall back to the
        # JS default size. The planner must treat this as not-ready.
        result = _run_js("""
            const state = resolvePlannerState(
                { camera: { vision_profile: "zr10" } },
                {
                    profiles: { zr10: { devices: [] } },
                    detector_class_dimensions: { "1": { width_m: 3.0, height_m: 2.5, size_m: 3.91 } },
                },
            );
            console.log(JSON.stringify(state));
        """)
        self.assertFalse(result["plannerReady"])
        self.assertIsNone(result["plannerDetectorClassDimensions"])

    def test_error_when_settings_missing(self):
        result = _run_js("""
            const state = resolvePlannerState(null, { profiles: { zr10: {} } });
            console.log(JSON.stringify(state));
        """)
        self.assertFalse(result["plannerReady"])
        self.assertEqual(result["plannerLoadError"], "Failed to load settings.")
        self.assertIsNone(result["plannerProfiles"])

    def test_error_when_catalog_missing(self):
        result = _run_js("""
            const state = resolvePlannerState({ camera: {} }, null);
            console.log(JSON.stringify(state));
        """)
        self.assertFalse(result["plannerReady"])
        self.assertEqual(result["plannerLoadError"], "Failed to load vision profiles.")
        self.assertIsNone(result["plannerProfiles"])


class TestUseSettingsSource(unittest.TestCase):
    def setUp(self):
        with open(_HOOK_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_imports_planner_state_helper(self):
        self.assertIn("resolvePlannerState", self.source)
        self.assertIn("./settingsPlannerState", self.source)

    def test_declares_planner_ready_state(self):
        self.assertIn("const [plannerReady, setPlannerReady] = useState(false);", self.source)

    def test_declares_planner_load_error_state(self):
        self.assertIn("const [plannerLoadError, setPlannerLoadError] = useState(null);", self.source)

    def test_exports_planner_states(self):
        return_idx = self.source.rfind("return {")
        self.assertNotEqual(return_idx, -1, "No return { found in hook")
        return_block = self.source[return_idx:]
        self.assertIn("plannerReady", return_block)
        self.assertIn("plannerLoadError", return_block)


if __name__ == "__main__":
    unittest.main()
