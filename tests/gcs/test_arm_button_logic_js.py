"""Tests for getArmAction utility in armAction.js."""
import json
import unittest
from tests.gcs.js_runner import run_node
import os


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "armAction.js",
))

# Strip ES module syntax so Node.js can eval the code.
_raw = open(_UTIL_PATH, encoding="utf-8").read()
_JS_SRC = (
    _raw
    .replace("export function ", "function ")
    .replace("export const ", "const ")
)


def _run_js(script):
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _get_arm_action(params_json):
    """Run getArmAction and return parsed JSON result."""
    return json.loads(
        _run_js(f"console.log(JSON.stringify(getArmAction({params_json})));")
    )


class TestArmAction(unittest.TestCase):
    """getArmAction return values for arm and disarm scenarios."""

    # --- ARM cases (armed=false) ---

    def test_arm_severity_ok_no_long_press(self):
        result = _get_arm_action('{"armed": false, "prearmSeverity": "ok", "altRel": 0}')
        self.assertEqual(result, {"requiresLongPress": False, "hint": None, "force": False})

    def test_arm_severity_warn_requires_long_press(self):
        result = _get_arm_action('{"armed": false, "prearmSeverity": "warn", "altRel": 0}')
        self.assertEqual(result, {"requiresLongPress": True, "hint": "armAction.holdToArm", "force": True})

    def test_arm_severity_fail_requires_long_press_force(self):
        result = _get_arm_action('{"armed": false, "prearmSeverity": "fail", "altRel": 0}')
        self.assertEqual(result, {"requiresLongPress": True, "hint": "armAction.holdToForceArm", "force": True})

    # --- DISARM cases (armed=true) ---

    def test_disarm_ground_no_long_press(self):
        result = _get_arm_action('{"armed": true, "prearmSeverity": "ok", "altRel": 0}')
        self.assertEqual(result, {"requiresLongPress": False, "hint": None, "force": False})

    def test_disarm_just_below_threshold_no_long_press(self):
        result = _get_arm_action('{"armed": true, "prearmSeverity": "ok", "altRel": 2.9}')
        self.assertEqual(result, {"requiresLongPress": False, "hint": None, "force": False})

    def test_disarm_exactly_at_threshold_requires_force_disarm(self):
        result = _get_arm_action('{"armed": true, "prearmSeverity": "ok", "altRel": 3}')
        self.assertEqual(result, {"requiresLongPress": True, "hint": "armAction.holdToForceDisarm", "force": True})

    def test_disarm_airborne_requires_force_disarm(self):
        result = _get_arm_action('{"armed": true, "prearmSeverity": "ok", "altRel": 50}')
        self.assertEqual(result, {"requiresLongPress": True, "hint": "armAction.holdToForceDisarm", "force": True})

    def test_disarm_alt_rel_null_safe_default(self):
        result = _get_arm_action('{"armed": true, "prearmSeverity": "ok", "altRel": null}')
        self.assertEqual(result, {"requiresLongPress": False, "hint": None, "force": False})

    def test_disarm_alt_rel_undefined_safe_default(self):
        result = _get_arm_action('{"armed": true, "prearmSeverity": "ok"}')
        self.assertEqual(result, {"requiresLongPress": False, "hint": None, "force": False})



class TestConstants(unittest.TestCase):
    """Exported constants have the expected values."""

    def test_long_press_ms_is_1000(self):
        out = _run_js("console.log(LONG_PRESS_MS);")
        self.assertEqual(int(out), 1000)

    def test_airborne_alt_threshold_is_3(self):
        out = _run_js("console.log(AIRBORNE_ALT_THRESHOLD);")
        self.assertEqual(float(out), 3)


class TestIsHoldKey(unittest.TestCase):
    """Only Enter/Space arm a keyboard press-and-hold (used by useLongPress)."""

    def _is_hold_key(self, key_json):
        return _run_js(f"console.log(isHoldKey({key_json}));").strip()

    def test_enter_arms_hold(self):
        self.assertEqual(self._is_hold_key('"Enter"'), "true")

    def test_space_arms_hold(self):
        self.assertEqual(self._is_hold_key('" "'), "true")

    def test_other_keys_do_not_arm_hold(self):
        for key in ('"a"', '"Tab"', '"Spacebar"', '"ArrowDown"', '"Escape"', '""'):
            with self.subTest(key=key):
                self.assertEqual(self._is_hold_key(key), "false")


if __name__ == "__main__":
    unittest.main()
