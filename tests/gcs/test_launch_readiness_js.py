"""Tests for computeLaunchReadiness utility in prearmChecks.js."""
import json
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


def _readiness(vehicle_json):
    """Run computeLaunchReadiness and return parsed result."""
    out = _run_js(
        f"console.log(JSON.stringify(computeLaunchReadiness({vehicle_json})));"
    )
    return json.loads(out)


class TestFullyHealthy(unittest.TestCase):
    def test_all_ok(self):
        r = _readiness('{"prearm_ok": true, "gps_fix": 3, "companion_ok": true, "mission_uploaded": true}')
        self.assertTrue(r["ready"])
        self.assertEqual(r["issues"], [])


class TestPrearmBlocks(unittest.TestCase):
    def test_prearm_failed(self):
        r = _readiness('{"prearm_ok": false, "gps_fix": 3, "companion_ok": true, "mission_uploaded": true}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.prearmFailed", r["issues"])

    def test_waiting_for_status(self):
        r = _readiness('{"gps_fix": 3, "companion_ok": true, "mission_uploaded": true}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.waitingForStatus", r["issues"])


class TestCompanionCheck(unittest.TestCase):
    def test_no_companion(self):
        r = _readiness('{"prearm_ok": true, "gps_fix": 3, "companion_ok": false, "mission_uploaded": true}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.companionNotConnected", r["issues"])

    def test_companion_ok(self):
        r = _readiness('{"prearm_ok": true, "gps_fix": 3, "companion_ok": true, "mission_uploaded": true}')
        self.assertNotIn("prearm.companionNotConnected", r["issues"])


class TestGpsCheck(unittest.TestCase):
    def test_no_gps_fix(self):
        r = _readiness('{"prearm_ok": true, "gps_fix": 2, "companion_ok": true, "mission_uploaded": true}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.noGps3d", r["issues"])

    def test_gps_unknown(self):
        r = _readiness('{"prearm_ok": true, "companion_ok": true, "mission_uploaded": true}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.gpsUnknown", r["issues"])


class TestThrottleCheck(unittest.TestCase):
    def test_throttle_high(self):
        r = _readiness('{"prearm_ok": true, "gps_fix": 3, "rc3": 1200, "companion_ok": true, "mission_uploaded": true}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.throttleNotZero", r["issues"])


class TestBatteryCheck(unittest.TestCase):
    def test_battery_low(self):
        r = _readiness('{"prearm_ok": true, "gps_fix": 3, "battery": 10, "companion_ok": true, "mission_uploaded": true}')
        self.assertFalse(r["ready"])
        self.assertTrue(any(isinstance(w, dict) and w.get("key") == "prearm.batteryLow" for w in r["issues"]))


class TestMissionCheck(unittest.TestCase):
    def test_mission_not_uploaded(self):
        r = _readiness('{"prearm_ok": true, "gps_fix": 3, "companion_ok": true, "mission_uploaded": false}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.missionNotUploaded", r["issues"])

    def test_mission_uploaded_missing(self):
        r = _readiness('{"prearm_ok": true, "gps_fix": 3, "companion_ok": true}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.missionNotUploaded", r["issues"])

    def test_mission_total_counts_as_uploaded(self):
        """Vehicle with mission items loaded should not show 'Mission not uploaded'."""
        r = _readiness('{"prearm_ok": true, "gps_fix": 3, "companion_ok": true, "mission_total": 10}')
        self.assertTrue(r["ready"])
        self.assertNotIn("prearm.missionNotUploaded", r["issues"])


class TestModeNotArmableFiltered(unittest.TestCase):
    def test_mode_not_armable_does_not_block_readiness(self):
        """'Mode not armable' as only PreArm failure → ready=true, no issues."""
        r = _readiness('{"prearm_ok": false, "gps_fix": 3, "companion_ok": true, "mission_uploaded": true, "status_texts": [{"text": "PreArm: Mode not armable"}]}')
        self.assertTrue(r["ready"])
        self.assertNotIn("Mode not armable", r["issues"])
        self.assertNotIn("prearm.prearmFailed", r["issues"])

    def test_real_prearm_still_blocks(self):
        """Other PreArm messages still block readiness when 'not armable' is filtered."""
        r = _readiness('{"prearm_ok": false, "gps_fix": 3, "companion_ok": true, "mission_uploaded": true, "status_texts": [{"text": "PreArm: Mode not armable"}, {"text": "PreArm: Hardware safety switch"}]}')
        self.assertFalse(r["ready"])
        self.assertIn("Hardware safety switch", r["issues"])
        self.assertNotIn("Mode not armable", r["issues"])


class TestChecksDisabled(unittest.TestCase):
    def test_checks_disabled_does_not_block_launch(self):
        """Arming checks disabled is armable: ready=true, advisory not in issues."""
        r = _readiness('{"prearm_check_state": "checks_disabled", "gps_fix": 3, "companion_ok": true, "mission_uploaded": true}')
        self.assertTrue(r["ready"])
        self.assertEqual(r["issues"], [])
        self.assertIn("prearm.checksDisabled", r["advisories"])

    def test_checks_disabled_still_blocked_by_other_issue(self):
        """A real blocker (no GPS) still gates even when checks are disabled."""
        r = _readiness('{"prearm_check_state": "checks_disabled", "gps_fix": 1, "companion_ok": true, "mission_uploaded": true}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.noGps3d", r["issues"])
        self.assertIn("prearm.checksDisabled", r["advisories"])

    def test_unknown_state_blocks_launch(self):
        """Unknown state fails closed in the UI, matching the backend gate."""
        r = _readiness('{"prearm_check_state": "some_future_state", "gps_fix": 3, "companion_ok": true, "mission_uploaded": true}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.statusUnknown", r["issues"])


def _readiness_gated(vehicle_json, gates_json):
    out = _run_js(
        f"console.log(JSON.stringify(computeLaunchReadiness({vehicle_json}, {gates_json})));"
    )
    return json.loads(out)


class TestTunableGates(unittest.TestCase):
    """Frontend gate parity with the backend ReadinessGates."""

    HEALTHY = '{"prearm_ok": true, "gps_fix": 3, "companion_ok": true, "mission_uploaded": true}'

    def test_disable_battery_allows_low(self):
        v = '{"prearm_ok": true, "gps_fix": 3, "battery": 5, "companion_ok": true, "mission_uploaded": true}'
        # Default: low battery blocks
        self.assertFalse(_readiness(v)["ready"])
        # Disabled: ignored
        r = _readiness_gated(v, '{"checkBattery": false}')
        self.assertTrue(r["ready"])

    def test_tunable_battery_threshold(self):
        v = '{"prearm_ok": true, "gps_fix": 3, "battery": 25, "companion_ok": true, "mission_uploaded": true}'
        self.assertTrue(_readiness(v)["ready"])  # 25 >= 15 default
        r = _readiness_gated(v, '{"checkBattery": true, "minBatteryPct": 30}')
        self.assertFalse(r["ready"])

    def test_tunable_throttle_threshold(self):
        v = '{"prearm_ok": true, "gps_fix": 3, "rc3": 1000, "companion_ok": true, "mission_uploaded": true}'
        self.assertTrue(_readiness(v)["ready"])  # 1000 <= 1050 default
        r = _readiness_gated(v, '{"checkThrottle": true, "maxThrottleRc3": 900}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.throttleNotZero", r["issues"])

    def test_disable_gps_allows_no_fix(self):
        v = '{"prearm_ok": true, "gps_fix": 0, "companion_ok": true, "mission_uploaded": true}'
        self.assertFalse(_readiness(v)["ready"])
        r = _readiness_gated(v, '{"checkGps": false}')
        self.assertTrue(r["ready"])

    def test_disable_prearm_allows_failed(self):
        v = '{"prearm_ok": false, "gps_fix": 3, "companion_ok": true, "mission_uploaded": true}'
        self.assertFalse(_readiness(v)["ready"])
        r = _readiness_gated(v, '{"checkPrearm": false}')
        self.assertTrue(r["ready"])

    def test_block_on_unknown_battery(self):
        v = '{"prearm_ok": true, "gps_fix": 3, "companion_ok": true, "mission_uploaded": true}'
        # Default: missing battery passes
        self.assertTrue(_readiness(v)["ready"])
        r = _readiness_gated(v, '{"checkBattery": true, "blockOnUnknownBattery": true}')
        self.assertFalse(r["ready"])
        self.assertIn("prearm.batteryUnknown", r["issues"])

    def test_gps_accuracy_off_by_default(self):
        v = '{"prearm_ok": true, "gps_fix": 3, "gps_hacc": 9.0, "companion_ok": true, "mission_uploaded": true}'
        self.assertTrue(_readiness(v)["ready"])  # accuracy not checked by default

    def test_gps_accuracy_blocks_when_enabled(self):
        v = '{"prearm_ok": true, "gps_fix": 3, "gps_hacc": 9.0, "companion_ok": true, "mission_uploaded": true}'
        r = _readiness_gated(v, '{"checkGpsAcc": true, "maxGpsHaccM": 1.0}')
        self.assertFalse(r["ready"])
        self.assertTrue(any(isinstance(w, dict) and w.get("key") == "prearm.gpsAccLow" for w in r["issues"]))

    def test_gps_accuracy_passes_good(self):
        v = '{"prearm_ok": true, "gps_fix": 3, "gps_hacc": 0.3, "companion_ok": true, "mission_uploaded": true}'
        r = _readiness_gated(v, '{"checkGpsAcc": true, "maxGpsHaccM": 1.0}')
        self.assertTrue(r["ready"])

    def test_gps_accuracy_unknown_does_not_block(self):
        v = '{"prearm_ok": true, "gps_fix": 3, "companion_ok": true, "mission_uploaded": true}'
        r = _readiness_gated(v, '{"checkGpsAcc": true, "maxGpsHaccM": 1.0}')
        self.assertTrue(r["ready"])


class TestLaunchGatesFromSettings(unittest.TestCase):
    def test_defaults_when_missing(self):
        out = _run_js("console.log(JSON.stringify(launchGatesFromSettings(null)));")
        g = json.loads(out)
        self.assertTrue(g["checkGps"])
        self.assertTrue(g["checkBattery"])
        self.assertEqual(g["maxThrottleRc3"], 1050)
        self.assertEqual(g["minBatteryPct"], 15)
        self.assertFalse(g["blockOnUnknownBattery"])

    def test_maps_settings_launch(self):
        settings = ('{"launch": {"check_gps_enabled": false, "max_throttle_rc3": 1100, '
                    '"min_battery_pct": 20, "block_on_unknown_battery": true, '
                    '"check_gps_acc_enabled": true, "max_gps_hacc_m": 0.5}}')
        out = _run_js(f"console.log(JSON.stringify(launchGatesFromSettings({settings})));")
        g = json.loads(out)
        self.assertFalse(g["checkGps"])
        self.assertEqual(g["maxThrottleRc3"], 1100)
        self.assertEqual(g["minBatteryPct"], 20)
        self.assertTrue(g["blockOnUnknownBattery"])
        self.assertTrue(g["checkGpsAcc"])
        self.assertEqual(g["maxGpsHaccM"], 0.5)
        # Unspecified stays enabled
        self.assertTrue(g["checkBattery"])

    def test_gps_acc_defaults_off(self):
        out = _run_js("console.log(JSON.stringify(launchGatesFromSettings(null)));")
        g = json.loads(out)
        self.assertFalse(g["checkGpsAcc"])
        self.assertEqual(g["maxGpsHaccM"], 1.0)


class TestNullVehicle(unittest.TestCase):
    def test_null_returns_ready(self):
        r = _readiness("null")
        self.assertTrue(r["ready"])
        self.assertEqual(r["issues"], [])
        self.assertEqual(r["advisories"], [])


class TestMultipleIssues(unittest.TestCase):
    def test_several_issues(self):
        r = _readiness('{"prearm_ok": false, "gps_fix": 0, "companion_ok": false}')
        self.assertFalse(r["ready"])
        self.assertGreater(len(r["issues"]), 2)

    def test_aggregate_empty_list(self):
        """No vehicles means allLaunchReady should be false (tested at JS level)."""
        out = _run_js("""
            var vehicles = [];
            var allReady = vehicles.length > 0 && vehicles.every(function(v) { return computeLaunchReadiness(v).ready; });
            console.log(JSON.stringify(allReady));
        """)
        self.assertEqual(out, "false")


if __name__ == "__main__":
    unittest.main()
