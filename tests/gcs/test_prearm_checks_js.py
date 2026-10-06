"""Tests for computePrearmWarnings utility in prearmChecks.js."""
import unittest
from tests.gcs.js_runner import run_node
import os


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "prearmChecks.js",
))

# Strip ES module syntax so Node.js can eval the code.
_raw = open(_UTIL_PATH, encoding="utf-8").read()
_JS_SRC = _raw.replace("export function ", "function ")


def _run_js(script):
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _compute(vehicle_json):
    """Run computePrearmWarnings and return JSON result."""
    return _run_js(
        f"console.log(JSON.stringify(computePrearmWarnings({vehicle_json})));"
    )


class TestNullVehicle(unittest.TestCase):
    """Null / undefined vehicle returns ok with no warnings."""

    def test_null_vehicle(self):
        out = _compute("null")
        self.assertEqual(out, '{"warnings":[],"advisories":[],"severity":"ok"}')

    def test_undefined_vehicle(self):
        out = _compute("undefined")
        self.assertEqual(out, '{"warnings":[],"advisories":[],"severity":"ok"}')


class TestPrearmOk(unittest.TestCase):
    """prearm_ok states."""

    def test_prearm_null(self):
        import json
        result = json.loads(_compute('{"prearm_ok": null, "gps_fix": 3}'))
        self.assertIn("prearm.waitingForStatus", result["warnings"])
        self.assertEqual(result["severity"], "warn")

    def test_prearm_undefined(self):
        import json
        # Omit prearm_ok entirely
        result = json.loads(_compute('{"gps_fix": 3}'))
        self.assertIn("prearm.waitingForStatus", result["warnings"])

    def test_prearm_false(self):
        import json
        result = json.loads(_compute('{"prearm_ok": false, "gps_fix": 3}'))
        self.assertIn("prearm.prearmFailed", result["warnings"])
        self.assertEqual(result["severity"], "fail")

    def test_prearm_true_no_issues(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 3}'))
        self.assertEqual(result["warnings"], [])
        self.assertEqual(result["severity"], "ok")


class TestPrearmCheckState(unittest.TestCase):
    """prearm_check_state takes precedence and splits advisories from warnings."""

    def test_checks_disabled_is_advisory_not_warning(self):
        import json
        result = json.loads(_compute(
            '{"prearm_ok": null, "prearm_check_state": "checks_disabled", "gps_fix": 3}'
        ))
        # Not a blocker: no warning, no "waiting for status", just an advisory.
        self.assertEqual(result["warnings"], [])
        self.assertIn("prearm.checksDisabled", result["advisories"])
        self.assertNotIn("prearm.waitingForStatus", result["warnings"])
        self.assertEqual(result["severity"], "ok")

    def test_no_sys_status_is_waiting_warning(self):
        import json
        result = json.loads(_compute(
            '{"prearm_check_state": "no_sys_status", "gps_fix": 3}'
        ))
        self.assertIn("prearm.waitingForStatus", result["warnings"])
        self.assertEqual(result["advisories"], [])

    def test_not_reported_is_warning(self):
        import json
        result = json.loads(_compute(
            '{"prearm_check_state": "not_reported", "gps_fix": 3}'
        ))
        self.assertIn("prearm.statusNotReported", result["warnings"])

    def test_ok_state_no_issues(self):
        import json
        result = json.loads(_compute(
            '{"prearm_check_state": "ok", "gps_fix": 3}'
        ))
        self.assertEqual(result["warnings"], [])
        self.assertEqual(result["advisories"], [])
        self.assertEqual(result["severity"], "ok")

    def test_failed_state_blocks(self):
        import json
        result = json.loads(_compute(
            '{"prearm_check_state": "failed", "gps_fix": 3}'
        ))
        self.assertIn("prearm.prearmFailed", result["warnings"])
        self.assertEqual(result["severity"], "fail")

    def test_state_takes_precedence_over_prearm_ok(self):
        import json
        # prearm_ok null would legacy-map to waiting, but the explicit state wins.
        result = json.loads(_compute(
            '{"prearm_ok": null, "prearm_check_state": "ok", "gps_fix": 3}'
        ))
        self.assertEqual(result["warnings"], [])
        self.assertEqual(result["advisories"], [])

    def test_unknown_state_fails_closed(self):
        import json
        result = json.loads(_compute(
            '{"prearm_check_state": "some_future_state", "gps_fix": 3}'
        ))
        self.assertIn("prearm.statusUnknown", result["warnings"])
        self.assertEqual(result["advisories"], [])

    def test_checks_disabled_advisory_does_not_raise_severity(self):
        import json
        # Advisory present but a real warning (GPS) drives severity to warn.
        result = json.loads(_compute(
            '{"prearm_check_state": "checks_disabled", "gps_fix": 1}'
        ))
        self.assertIn("prearm.checksDisabled", result["advisories"])
        self.assertIn("prearm.noGps3d", result["warnings"])
        self.assertEqual(result["severity"], "warn")


class TestVisibleAdvisories(unittest.TestCase):
    """visibleAdvisories filters dismissed advisories (the card's dismiss logic)."""

    def _visible(self, advisories_json, dismissed_json):
        import json
        return json.loads(_run_js(
            f"console.log(JSON.stringify(visibleAdvisories({advisories_json}, {dismissed_json})));"
        ))

    def test_none_dismissed_shows_all(self):
        self.assertEqual(
            self._visible('["prearm.checksDisabled"]', '[]'),
            ["prearm.checksDisabled"],
        )

    def test_dismissed_key_filtered(self):
        self.assertEqual(
            self._visible('["prearm.checksDisabled"]', '["prearm.checksDisabled"]'),
            [],
        )

    def test_object_advisory_filtered_by_key(self):
        self.assertEqual(
            self._visible('[{"key": "prearm.checksDisabled", "x": 1}]',
                          '["prearm.checksDisabled"]'),
            [],
        )

    def test_empty_and_null_inputs(self):
        self.assertEqual(self._visible('[]', '[]'), [])
        self.assertEqual(self._visible('null', '[]'), [])


class TestThrottleCheck(unittest.TestCase):
    """Throttle position warnings."""

    def test_throttle_idle(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "rc3": 1000, "gps_fix": 3}'))
        self.assertNotIn("prearm.throttleNotZero", result["warnings"])

    def test_throttle_at_threshold(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "rc3": 1050, "gps_fix": 3}'))
        self.assertNotIn("prearm.throttleNotZero", result["warnings"])

    def test_throttle_above_threshold(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "rc3": 1100, "gps_fix": 3}'))
        self.assertIn("prearm.throttleNotZero", result["warnings"])
        self.assertEqual(result["severity"], "warn")

    def test_throttle_null(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 3}'))
        self.assertNotIn("prearm.throttleNotZero", result["warnings"])


class TestGpsCheck(unittest.TestCase):
    """GPS fix warnings."""

    def test_gps_3d_fix(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 3}'))
        self.assertNotIn("prearm.noGps3d", result["warnings"])
        self.assertNotIn("prearm.gpsUnknown", result["warnings"])

    def test_gps_2d_fix(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 2}'))
        self.assertIn("prearm.noGps3d", result["warnings"])

    def test_gps_no_fix(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 0}'))
        self.assertIn("prearm.noGps3d", result["warnings"])

    def test_gps_null(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": null}'))
        self.assertIn("prearm.gpsUnknown", result["warnings"])

    def test_gps_undefined(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true}'))
        self.assertIn("prearm.gpsUnknown", result["warnings"])


class TestBatteryCheck(unittest.TestCase):
    """Battery level warnings."""

    def test_battery_ok(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 3, "battery": 50}'))
        for w in result["warnings"]:
            if isinstance(w, dict):
                self.assertNotEqual(w.get("key"), "prearm.batteryLow")

    def test_battery_at_threshold(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 3, "battery": 15}'))
        for w in result["warnings"]:
            if isinstance(w, dict):
                self.assertNotEqual(w.get("key"), "prearm.batteryLow")

    def test_battery_low(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 3, "battery": 12}'))
        battery_warnings = [w for w in result["warnings"] if isinstance(w, dict) and w.get("key") == "prearm.batteryLow"]
        self.assertEqual(len(battery_warnings), 1)
        self.assertEqual(battery_warnings[0]["pct"], 12)

    def test_battery_null(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 3}'))
        for w in result["warnings"]:
            if isinstance(w, dict):
                self.assertNotEqual(w.get("key"), "prearm.batteryLow")


class TestPrearmStatusTexts(unittest.TestCase):
    """PreArm status text extraction."""

    def test_extracts_prearm_messages_when_prearm_false(self):
        import json
        vehicle = '{"prearm_ok": false, "gps_fix": 3, "status_texts": [{"text": "PreArm: Hardware safety switch"}, {"text": "Normal message"}]}'
        result = json.loads(_compute(vehicle))
        self.assertIn("Hardware safety switch", result["warnings"])
        self.assertNotIn("Normal message", result["warnings"])
        # Generic message replaced by specific details
        self.assertNotIn("prearm.prearmFailed", result["warnings"])

    def test_ignores_prearm_messages_when_prearm_true(self):
        """Cached PreArm messages should NOT produce warnings when prearm_ok is true."""
        import json
        vehicle = '{"prearm_ok": true, "gps_fix": 3, "status_texts": [{"text": "PreArm: Hardware safety switch"}]}'
        result = json.loads(_compute(vehicle))
        self.assertNotIn("Hardware safety switch", result["warnings"])
        self.assertEqual(result["severity"], "ok")

    def test_no_duplicate_prearm(self):
        import json
        vehicle = '{"prearm_ok": false, "gps_fix": 3, "status_texts": [{"text": "PreArm: Throttle not zero"}, {"text": "PreArm: Throttle not zero"}]}'
        result = json.loads(_compute(vehicle))
        count = result["warnings"].count("Throttle not zero")
        self.assertEqual(count, 1)

    def test_generic_fallback_when_no_prearm_texts(self):
        """When prearm_ok is false but no PreArm: messages, show generic."""
        import json
        result = json.loads(_compute('{"prearm_ok": false, "gps_fix": 3, "status_texts": []}'))
        self.assertIn("prearm.prearmFailed", result["warnings"])

    def test_generic_fallback_no_status_texts(self):
        """When prearm_ok is false and status_texts absent, show generic."""
        import json
        result = json.loads(_compute('{"prearm_ok": false, "gps_fix": 3}'))
        self.assertIn("prearm.prearmFailed", result["warnings"])

    def test_specific_replaces_generic(self):
        """When specific PreArm: messages exist, generic message should not appear."""
        import json
        vehicle = '{"prearm_ok": false, "gps_fix": 3, "status_texts": [{"text": "PreArm: Throttle too high"}, {"text": "PreArm: GPS not healthy"}]}'
        result = json.loads(_compute(vehicle))
        self.assertIn("Throttle too high", result["warnings"])
        self.assertIn("GPS not healthy", result["warnings"])
        self.assertNotIn("prearm.prearmFailed", result["warnings"])

    def test_empty_status_texts(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 3, "status_texts": []}'))
        self.assertEqual(result["warnings"], [])

    def test_rtl_prearm_clears_after_mode_change(self):
        """Simulates RTL→FBWA: prearm_ok becomes true, stale PreArm texts ignored."""
        import json
        vehicle = '{"prearm_ok": true, "gps_fix": 3, "status_texts": [{"text": "PreArm: Mode not armable"}]}'
        result = json.loads(_compute(vehicle))
        self.assertNotIn("Mode not armable", result["warnings"])
        self.assertEqual(result["severity"], "ok")

    def test_mode_not_armable_filtered_when_prearm_false(self):
        """'Mode not armable' as only PreArm failure → no warnings, not 'fail'."""
        import json
        vehicle = '{"prearm_ok": false, "gps_fix": 3, "status_texts": [{"text": "PreArm: Mode not armable"}]}'
        result = json.loads(_compute(vehicle))
        self.assertNotIn("Mode not armable", result["warnings"])
        self.assertNotIn("prearm.prearmFailed", result["warnings"])
        self.assertEqual(result["severity"], "ok")

    def test_mode_not_armable_plus_real_failure(self):
        """'Mode not armable' + real failure → severity 'fail', real failure shown."""
        import json
        vehicle = '{"prearm_ok": false, "gps_fix": 3, "status_texts": [{"text": "PreArm: Mode not armable"}, {"text": "PreArm: GPS not healthy"}]}'
        result = json.loads(_compute(vehicle))
        self.assertNotIn("Mode not armable", result["warnings"])
        self.assertIn("GPS not healthy", result["warnings"])
        self.assertEqual(result["severity"], "fail")

    def test_prearm_false_no_status_texts_generic_fallback(self):
        """prearm_ok=false + no STATUSTEXT → generic 'Pre-arm check failed'."""
        import json
        result = json.loads(_compute('{"prearm_ok": false, "gps_fix": 3}'))
        self.assertIn("prearm.prearmFailed", result["warnings"])
        self.assertEqual(result["severity"], "fail")

    def test_manual_mode_not_armable_filtered(self):
        """'MANUAL mode not armable' variant is also filtered."""
        import json
        vehicle = '{"prearm_ok": false, "gps_fix": 3, "status_texts": [{"text": "PreArm: MANUAL mode not armable"}, {"text": "PreArm: GPS not healthy"}]}'
        result = json.loads(_compute(vehicle))
        self.assertNotIn("MANUAL mode not armable", result["warnings"])
        self.assertIn("GPS not healthy", result["warnings"])


class TestSeverityLevels(unittest.TestCase):
    """Severity level computation."""

    def test_ok_no_warnings(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 3}'))
        self.assertEqual(result["severity"], "ok")

    def test_warn_with_throttle(self):
        import json
        result = json.loads(_compute('{"prearm_ok": true, "gps_fix": 3, "rc3": 1200}'))
        self.assertEqual(result["severity"], "warn")

    def test_fail_prearm_false(self):
        import json
        result = json.loads(_compute('{"prearm_ok": false, "gps_fix": 3}'))
        self.assertEqual(result["severity"], "fail")

    def test_fail_overrides_warn(self):
        """prearm_ok=false always produces 'fail' even with other warnings."""
        import json
        result = json.loads(_compute('{"prearm_ok": false, "gps_fix": 2, "rc3": 1200}'))
        self.assertEqual(result["severity"], "fail")


if __name__ == "__main__":
    unittest.main()
