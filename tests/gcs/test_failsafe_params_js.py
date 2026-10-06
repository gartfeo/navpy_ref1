"""Pure-helper tests for src/gcs/frontend/src/utils/failsafeParams.js.

Drives the JS via Node so the same code that ships to the browser is
exercised. These lock the safety-critical contracts of the curated Failsafe
tab: firmware detection, the firmware-specific battery-action enum (the
Copter 1=Land/2=RTL vs Plane 1=RTL/2=Land swap), present-param filtering, and
the pre-flight "is failsafe configured?" checks.
"""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "failsafeParams.js",
))

_raw = open(_UTIL_PATH, encoding="utf-8").read()
_JS_SRC = (
    _raw
    .replace("export function ", "function ")
    .replace("export const ", "const ")
)


def _run_js(script: str) -> str:
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _call_json(expr: str):
    out = _run_js(f"console.log(JSON.stringify({expr}));")
    return json.loads(out)


# ---------------------------------------------------------------------
class TestDetectFirmware(unittest.TestCase):
    def test_copter_markers(self):
        self.assertEqual(_call_json("detectFirmware(['FS_THR_ENABLE', 'BATT_LOW_VOLT'])"), "copter")
        self.assertEqual(_call_json("detectFirmware(['FS_GCS_ENABLE'])"), "copter")

    def test_plane_markers(self):
        self.assertEqual(_call_json("detectFirmware(['THR_FAILSAFE', 'BATT_LOW_VOLT'])"), "plane")
        self.assertEqual(_call_json("detectFirmware(['FS_LONG_ACTN', 'FS_GCS_ENABL'])"), "plane")

    def test_plane_enabl_not_confused_with_copter_enable(self):
        # FS_GCS_ENABL is a prefix of FS_GCS_ENABLE; exact-match must keep them apart.
        self.assertEqual(_call_json("detectFirmware(['FS_GCS_ENABL'])"), "plane")
        self.assertEqual(_call_json("detectFirmware(['FS_GCS_ENABLE'])"), "copter")

    def test_unknown_when_no_markers(self):
        self.assertEqual(_call_json("detectFirmware(['BATT_LOW_VOLT', 'BATT_FS_LOW_ACT'])"), "unknown")

    def test_unknown_when_conflicting(self):
        self.assertEqual(_call_json("detectFirmware(['THR_FAILSAFE', 'FS_THR_ENABLE'])"), "unknown")


class TestBatteryActionSwap(unittest.TestCase):
    """The dangerous divergence: same param name, swapped labels by firmware."""

    def test_copter_low_action_labels(self):
        opts = _call_json("optionsForParam({type:'select', firmwareEnum:true}, 'copter')")
        by_val = {o["value"]: o["label"] for o in opts}
        self.assertEqual(by_val[1], "Land")
        self.assertEqual(by_val[2], "RTL")
        self.assertEqual(by_val[0], "Warn only")

    def test_plane_low_action_labels_are_swapped(self):
        opts = _call_json("optionsForParam({type:'select', firmwareEnum:true}, 'plane')")
        by_val = {o["value"]: o["label"] for o in opts}
        self.assertEqual(by_val[1], "RTL")
        self.assertEqual(by_val[2], "Land")
        self.assertEqual(by_val[4], "QLand")

    def test_unknown_firmware_yields_no_options(self):
        # No safe label table for a firmware-specific enum when firmware is
        # unknown — the UI falls back to a raw number rather than guess.
        self.assertEqual(_call_json("optionsForParam({type:'select', firmwareEnum:true}, 'unknown')"), [])

    def test_action_label_lookup_and_out_of_range(self):
        self.assertEqual(_call_json("actionLabel({type:'select', firmwareEnum:true}, 2, 'copter')"), "RTL")
        self.assertEqual(_call_json("actionLabel({type:'select', firmwareEnum:true}, 2, 'plane')"), "Land")
        self.assertIsNone(_call_json("actionLabel({type:'select', firmwareEnum:true}, 99, 'copter')"))

    def test_option_keys_point_at_firmware_table(self):
        opts_c = _call_json("optionsForParam({type:'select', firmwareEnum:true}, 'copter')")
        opts_p = _call_json("optionsForParam({type:'select', firmwareEnum:true}, 'plane')")
        self.assertTrue(opts_c[0]["key"].endswith("BATT_ACTION_COPTER.0"))
        self.assertTrue(opts_p[0]["key"].endswith("BATT_ACTION_PLANE.0"))


class TestFixedEnums(unittest.TestCase):
    def test_fixed_enum_independent_of_firmware(self):
        a = _call_json("optionsForParam({type:'select', enum:'FS_LONG_ACTN'}, 'plane')")
        b = _call_json("optionsForParam({type:'select', enum:'FS_LONG_ACTN'}, 'copter')")
        self.assertEqual(a, b)
        self.assertEqual({o["value"]: o["label"] for o in a}[1], "Return to Launch (RTL)")

    def test_non_select_has_no_options(self):
        self.assertEqual(_call_json("optionsForParam({type:'number'}, 'copter')"), [])


class TestVisibleParams(unittest.TestCase):
    def test_only_present_params(self):
        names = _call_json(
            "visibleParamsForGroup("
            "FAILSAFE_GROUPS.find(g=>g.id==='battery'),"
            "['BATT_FS_LOW_ACT','BATT_LOW_VOLT'], 'copter').map(d=>d.name)"
        )
        self.assertEqual(set(names), {"BATT_FS_LOW_ACT", "BATT_LOW_VOLT"})

    def test_firmware_tag_hides_other_firmware(self):
        # Even if both throttle-enable names are present (shouldn't happen),
        # a copter view shows only the copter-tagged one.
        names = _call_json(
            "visibleParamsForGroup("
            "FAILSAFE_GROUPS.find(g=>g.id==='rcThrottle'),"
            "['FS_THR_ENABLE','THR_FAILSAFE'], 'copter').map(d=>d.name)"
        )
        self.assertIn("FS_THR_ENABLE", names)
        self.assertNotIn("THR_FAILSAFE", names)

    def test_unknown_firmware_shows_present_regardless_of_tag(self):
        names = _call_json(
            "visibleParamsForGroup("
            "FAILSAFE_GROUPS.find(g=>g.id==='gcs'),"
            "['FS_GCS_ENABL'], 'unknown').map(d=>d.name)"
        )
        self.assertEqual(names, ["FS_GCS_ENABL"])


class TestFailsafeChecks(unittest.TestCase):
    def _checks(self, values: dict, firmware: str):
        result = _call_json(f"buildFailsafeChecks({json.dumps(values)}, '{firmware}')")
        return {c["id"]: c for c in result}

    def test_all_configured_copter(self):
        checks = self._checks(
            {"FS_THR_ENABLE": 1, "BATT_FS_LOW_ACT": 2, "BATT_LOW_VOLT": 14.0, "FS_GCS_ENABLE": 1},
            "copter",
        )
        self.assertTrue(all(c["ok"] for c in checks.values()))
        summary = _call_json(
            "summarizeChecks(buildFailsafeChecks("
            "{FS_THR_ENABLE:1, BATT_FS_LOW_ACT:2, BATT_LOW_VOLT:14, FS_GCS_ENABLE:1}, 'copter'))"
        )
        self.assertTrue(summary["configured"])
        self.assertEqual(summary["warnCount"], 0)

    def test_disabled_throttle_warns(self):
        checks = self._checks({"FS_THR_ENABLE": 0}, "copter")
        self.assertFalse(checks["throttle"]["ok"])
        self.assertEqual(checks["throttle"]["level"], "warn")

    def test_battery_action_warn_only_flags(self):
        checks = self._checks({"BATT_FS_LOW_ACT": 0}, "copter")
        self.assertFalse(checks["batteryAction"]["ok"])

    def test_battery_threshold_ok_if_either_set(self):
        ok_volt = self._checks({"BATT_LOW_VOLT": 14.0, "BATT_LOW_MAH": 0}, "copter")
        self.assertTrue(ok_volt["batteryThreshold"]["ok"])
        ok_mah = self._checks({"BATT_LOW_VOLT": 0, "BATT_LOW_MAH": 1500}, "copter")
        self.assertTrue(ok_mah["batteryThreshold"]["ok"])
        warn = self._checks({"BATT_LOW_VOLT": 0, "BATT_LOW_MAH": 0}, "copter")
        self.assertFalse(warn["batteryThreshold"]["ok"])

    def test_absent_params_are_not_checked(self):
        checks = self._checks({"BATT_FS_LOW_ACT": 2}, "copter")
        self.assertIn("batteryAction", checks)
        self.assertNotIn("throttle", checks)
        self.assertNotIn("gcs", checks)

    def test_plane_uses_plane_param_names(self):
        checks = self._checks({"THR_FAILSAFE": 1, "FS_GCS_ENABL": 2}, "plane")
        self.assertTrue(checks["throttle"]["ok"])
        self.assertTrue(checks["gcs"]["ok"])

    def test_empty_is_not_configured(self):
        summary = _call_json("summarizeChecks([])")
        self.assertFalse(summary["configured"])
        self.assertEqual(summary["total"], 0)


class TestParamNameCoverage(unittest.TestCase):
    def test_corrected_param_names_present(self):
        names = _call_json("allFailsafeParamNames()")
        # The corrected Plane throttle-enable name (NOT the nonexistent
        # THR_FS_ENABLE from the original brief).
        self.assertIn("THR_FAILSAFE", names)
        self.assertNotIn("THR_FS_ENABLE", names)
        # FS_SHORT_TIMEOUT does not exist in ArduPilot — must not be exposed.
        self.assertNotIn("FS_SHORT_TIMEOUT", names)
        # Firmware-specific spellings both present.
        self.assertIn("FS_GCS_ENABLE", names)
        self.assertIn("FS_GCS_ENABL", names)


if __name__ == "__main__":
    unittest.main()
