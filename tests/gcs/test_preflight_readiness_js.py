"""Tests for the preflightReadiness.js utility (runs under Node.js).

preflightReadiness.js imports from prearmChecks.js, so both sources are
concatenated with their ES-module syntax stripped before evaluation — matching
the project's Node-subprocess test style for frontend utilities.
"""
import json
import os
import re
from tests.gcs.js_runner import run_node
import unittest

_DIR = os.path.dirname(__file__)
_UTILS = os.path.normpath(os.path.join(
    _DIR, "..", "..", "src", "gcs", "frontend", "src", "utils",
))


def _load(name):
    with open(os.path.join(_UTILS, name), encoding="utf-8") as f:
        return f.read()


# Strip `export ` keywords and the import line so Node can eval the bundle.
_PREARM = _load("prearmChecks.js").replace("export ", "")
_PREFLIGHT = _load("preflightReadiness.js").replace("export ", "")
_PREFLIGHT = re.sub(r"^import .*\n", "", _PREFLIGHT, flags=re.M)
_JS_SRC = _PREARM + "\n" + _PREFLIGHT


def _run_js(script):
    result = run_node(_JS_SRC + "\n" + script, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _call(fn, *args):
    arg_str = ", ".join(json.dumps(a) for a in args)
    return json.loads(_run_js(f"console.log(JSON.stringify({fn}({arg_str})));"))


def _by_key(readiness):
    """Index a computeVehicleReadiness() result's checks by row key."""
    return {c["key"]: c for c in readiness["checks"]}


_GOOD = {
    "prearm_check_state": "ok",
    "gps_fix": 3, "gps_sats": 14, "gps_hacc": 0.5,
    "battery": 80, "voltage": 16.2,
    "companion_ok": True, "companion_status": "ok",
    "mission_uploaded": True,
    "ekf": {"flags": 831, "velocity_variance": 0.1, "pos_horiz_variance": 0.1,
            "pos_vert_variance": 0.1, "compass_variance": 0.1,
            "terrain_alt_variance": 0.0},
    "sensors": {"gyro": True, "accel": True, "mag": True,
                "abs_pressure": True, "rc": True},
}

# Full launch-gate defaults with the GPS-accuracy sub-check turned ON. Passed
# explicitly to computeVehicleReadiness for the backstop test that proves a
# warn-severity GPS row plus a launch-gate block still forces overall NO-GO.
# All fields are spelled out because a PARTIAL gates object would leave other
# flags undefined (falsy), silently disabling their checks.
_GATES_GPSACC = {
    "checkPrearm": True, "checkGps": True,
    "checkGpsAcc": True, "maxGpsHaccM": 1.0,
    "checkThrottle": True, "maxThrottleRc3": 1050,
    "checkBattery": True, "minBatteryPct": 15, "blockOnUnknownBattery": False,
}


class TestEkfCheck(unittest.TestCase):
    def test_null_is_na(self):
        self.assertEqual(_call("computeEkfCheck", None)["status"], "na")

    def test_low_variance_is_go(self):
        ekf = {"flags": 831, "velocity_variance": 0.2, "pos_horiz_variance": 0.1,
               "pos_vert_variance": 0.1, "compass_variance": 0.1,
               "terrain_alt_variance": 0.0}
        self.assertEqual(_call("computeEkfCheck", ekf)["status"], "go")

    def test_mid_variance_is_warn(self):
        ekf = {"flags": 831, "velocity_variance": 0.6, "pos_horiz_variance": 0.1,
               "pos_vert_variance": 0.1, "compass_variance": 0.1,
               "terrain_alt_variance": 0.0}
        self.assertEqual(_call("computeEkfCheck", ekf)["status"], "warn")

    def test_high_variance_is_nogo(self):
        ekf = {"flags": 831, "velocity_variance": 0.9, "pos_horiz_variance": 0.1,
               "pos_vert_variance": 0.1, "compass_variance": 0.1,
               "terrain_alt_variance": 0.0}
        self.assertEqual(_call("computeEkfCheck", ekf)["status"], "nogo")

    def test_uninitialized_flag_is_nogo(self):
        ekf = {"flags": 1024, "velocity_variance": 0.0, "pos_horiz_variance": 0.0,
               "pos_vert_variance": 0.0, "compass_variance": 0.0,
               "terrain_alt_variance": 0.0}
        out = _call("computeEkfCheck", ekf)
        self.assertEqual(out["status"], "nogo")
        self.assertEqual(out["detail"]["reason"], "uninitialized")

    def test_terrain_variance_excluded(self):
        # High terrain variance must NOT force a NO-GO when nav variances are low.
        ekf = {"flags": 831, "velocity_variance": 0.1, "pos_horiz_variance": 0.1,
               "pos_vert_variance": 0.1, "compass_variance": 0.1,
               "terrain_alt_variance": 0.95}
        self.assertEqual(_call("computeEkfCheck", ekf)["status"], "go")


class TestSensorCheck(unittest.TestCase):
    def test_null_is_na(self):
        self.assertEqual(_call("computeSensorCheck", None)["status"], "na")

    def test_all_healthy_is_go(self):
        self.assertEqual(
            _call("computeSensorCheck", _GOOD["sensors"])["status"], "go")

    def test_one_failed_is_nogo(self):
        s = dict(_GOOD["sensors"], gyro=False)
        out = _call("computeSensorCheck", s)
        self.assertEqual(out["status"], "nogo")
        self.assertIn("gyro", out["detail"]["failed"])

    def test_all_null_is_na(self):
        s = {"gyro": None, "accel": None, "mag": None, "abs_pressure": None}
        self.assertEqual(_call("computeSensorCheck", s)["status"], "na")


class TestRcCheck(unittest.TestCase):
    def test_present_healthy_is_go(self):
        self.assertEqual(_call("computeRcCheck", {"rc": True})["status"], "go")

    def test_present_unhealthy_is_nogo(self):
        self.assertEqual(_call("computeRcCheck", {"rc": False})["status"], "nogo")

    def test_missing_is_na(self):
        self.assertEqual(_call("computeRcCheck", {"rc": None})["status"], "na")
        self.assertEqual(_call("computeRcCheck", None)["status"], "na")


class TestGpsCheck(unittest.TestCase):
    def test_3d_fix_is_go(self):
        self.assertEqual(_call("computeGpsCheck", _GOOD)["status"], "go")

    def test_2d_fix_is_nogo(self):
        v = dict(_GOOD, gps_fix=2)
        self.assertEqual(_call("computeGpsCheck", v)["status"], "nogo")

    def test_null_fix_is_nogo(self):
        v = dict(_GOOD, gps_fix=None)
        self.assertEqual(_call("computeGpsCheck", v)["status"], "nogo")


class TestBatteryCheck(unittest.TestCase):
    def test_ok_is_go(self):
        self.assertEqual(_call("computeBatteryCheck", _GOOD)["status"], "go")

    def test_low_is_nogo(self):
        v = dict(_GOOD, battery=10)
        self.assertEqual(_call("computeBatteryCheck", v)["status"], "nogo")

    def test_unknown_is_warn_by_default(self):
        v = dict(_GOOD, battery=None)
        self.assertEqual(_call("computeBatteryCheck", v)["status"], "warn")


class TestPrearmCheck(unittest.TestCase):
    def test_ok_is_go(self):
        self.assertEqual(_call("computePrearmCheck", _GOOD)["status"], "go")

    def test_failed_is_nogo(self):
        v = dict(_GOOD, prearm_check_state="failed")
        self.assertEqual(_call("computePrearmCheck", v)["status"], "nogo")

    def test_checks_disabled_is_warn(self):
        v = dict(_GOOD, prearm_check_state="checks_disabled")
        self.assertEqual(_call("computePrearmCheck", v)["status"], "warn")

    def test_waiting_is_warn(self):
        v = dict(_GOOD, prearm_check_state="no_sys_status")
        self.assertEqual(_call("computePrearmCheck", v)["status"], "warn")


class TestCompanionCheck(unittest.TestCase):
    def test_companion_ok_false_is_nogo(self):
        self.assertEqual(
            _call("computeCompanionCheck", {"companion_ok": False})["status"], "nogo")

    def test_status_down_is_nogo(self):
        # Defensive: explicit 'down' blocks even if companion_ok looks stale.
        out = _call("computeCompanionCheck", {"companion_status": "down", "companion_ok": True})
        self.assertEqual(out["status"], "nogo")

    def test_checking_is_warn(self):
        out = _call("computeCompanionCheck", {"companion_status": "checking", "companion_ok": True})
        self.assertEqual(out["status"], "warn")

    def test_ok_is_go(self):
        out = _call("computeCompanionCheck", {"companion_status": "ok", "companion_ok": True})
        self.assertEqual(out["status"], "go")

    def test_companion_ok_true_without_status_is_go(self):
        # An affirmative companion_ok (not down) reads GO even if status is absent.
        self.assertEqual(_call("computeCompanionCheck", {"companion_ok": True})["status"], "go")

    def test_missing_is_na(self):
        # No status and no companion_ok at all => nothing to judge (non-blocking).
        self.assertEqual(_call("computeCompanionCheck", {})["status"], "na")
        self.assertEqual(_call("computeCompanionCheck", None)["status"], "na")


class TestMissionCheck(unittest.TestCase):
    def test_uploaded_is_go(self):
        self.assertEqual(
            _call("computeMissionCheck", {"mission_uploaded": True})["status"], "go")

    def test_items_on_board_is_go(self):
        self.assertEqual(
            _call("computeMissionCheck", {"mission_total": 10})["status"], "go")

    def test_not_uploaded_is_nogo(self):
        out = _call("computeMissionCheck", {"mission_uploaded": False, "mission_total": 0})
        self.assertEqual(out["status"], "nogo")

    def test_missing_is_nogo(self):
        # Full reconcile: no verified upload and no items => cannot launch.
        self.assertEqual(_call("computeMissionCheck", {})["status"], "nogo")


class TestThrottleCheck(unittest.TestCase):
    def test_high_is_nogo(self):
        self.assertEqual(_call("computeThrottleCheck", {"rc3": 1200})["status"], "nogo")

    def test_low_is_go(self):
        self.assertEqual(_call("computeThrottleCheck", {"rc3": 1000})["status"], "go")

    def test_missing_rc3_is_na(self):
        self.assertEqual(_call("computeThrottleCheck", {})["status"], "na")

    def test_check_disabled_is_na(self):
        gates = dict(_GATES_GPSACC, checkThrottle=False)
        self.assertEqual(_call("computeThrottleCheck", {"rc3": 1200}, gates)["status"], "na")


class TestOverall(unittest.TestCase):
    def test_good_vehicle_is_go(self):
        out = _call("computePreflightReadiness", _GOOD)
        self.assertEqual(out["overall"], "go")
        self.assertTrue(out["go"])
        self.assertEqual(len(out["checks"]), 6)

    def test_nogo_row_blocks_overall(self):
        v = dict(_GOOD, gps_fix=0)
        out = _call("computePreflightReadiness", v)
        self.assertEqual(out["overall"], "nogo")
        self.assertFalse(out["go"])

    def test_warn_row_does_not_block(self):
        v = dict(_GOOD, prearm_check_state="checks_disabled")
        out = _call("computePreflightReadiness", v)
        self.assertEqual(out["overall"], "warn")
        self.assertTrue(out["go"])

    def test_null_vehicle(self):
        out = _call("computePreflightReadiness", None)
        self.assertEqual(out["overall"], "na")
        self.assertTrue(out["go"])
        self.assertEqual(out["checks"], [])


class TestAirspeedCheck(unittest.TestCase):
    def test_no_pitot_is_na(self):
        self.assertEqual(_call("computeAirspeedCheck", {"airspeed_present": False})["status"], "na")
        self.assertEqual(_call("computeAirspeedCheck", {})["status"], "na")

    def test_unhealthy_is_nogo(self):
        v = {"airspeed_present": True, "airspeed_ok": False, "air_speed": 0.2}
        self.assertEqual(_call("computeAirspeedCheck", v)["status"], "nogo")

    def test_ground_reading_is_warn(self):
        v = {"airspeed_present": True, "airspeed_ok": True, "air_speed": 6.0,
             "armed": False, "ground_speed": 0.0}
        out = _call("computeAirspeedCheck", v)
        self.assertEqual(out["status"], "warn")
        self.assertEqual(out["detail"]["reason"], "groundReading")

    def test_low_ground_reading_is_go(self):
        v = {"airspeed_present": True, "airspeed_ok": True, "air_speed": 0.3,
             "armed": False, "ground_speed": 0.0}
        self.assertEqual(_call("computeAirspeedCheck", v)["status"], "go")

    def test_armed_high_reading_is_go(self):
        # Flying → not stationary, so a high reading is expected, not a nudge.
        v = {"airspeed_present": True, "airspeed_ok": True, "air_speed": 20.0,
             "armed": True, "ground_speed": 18.0}
        self.assertEqual(_call("computeAirspeedCheck", v)["status"], "go")

    def test_moving_on_ground_high_reading_is_go(self):
        v = {"airspeed_present": True, "airspeed_ok": True, "air_speed": 6.0,
             "armed": False, "ground_speed": 5.0}
        self.assertEqual(_call("computeAirspeedCheck", v)["status"], "go")


_GOOD_LINKED = dict(_GOOD, link_ok=True)


class TestVehicleReadiness(unittest.TestCase):
    def test_link_down_is_na_with_no_checks(self):
        v = dict(_GOOD_LINKED, link_ok=False)
        out = _call("computeVehicleReadiness", v)
        self.assertEqual(out["overall"], "na")
        self.assertEqual(out["checks"], [])

    def test_healthy_linked_vehicle_is_go(self):
        out = _call("computeVehicleReadiness", _GOOD_LINKED)
        self.assertEqual(out["overall"], "go")

    def test_airspeed_nogo_overrides_otherwise_healthy(self):
        v = dict(_GOOD_LINKED, airspeed_present=True, airspeed_ok=False, air_speed=0.0)
        out = _call("computeVehicleReadiness", v)
        self.assertEqual(out["overall"], "nogo")

    def test_airspeed_warn_raises_go_to_warn_but_not_over_nogo(self):
        v = dict(_GOOD_LINKED, airspeed_present=True, airspeed_ok=True,
                 air_speed=6.0, armed=False, ground_speed=0.0)
        out = _call("computeVehicleReadiness", v)
        self.assertEqual(out["overall"], "warn")

    def test_companion_down_makes_overall_nogo(self):
        # BOTH a visible companion row AND the overall must reflect the block.
        v = dict(_GOOD_LINKED, companion_status="down", companion_ok=False)
        out = _call("computeVehicleReadiness", v)
        self.assertEqual(out["overall"], "nogo")
        self.assertEqual(_by_key(out)["companion"]["status"], "nogo")

    def test_companion_checking_is_warn_not_nogo(self):
        v = dict(_GOOD_LINKED, companion_status="checking", companion_ok=True)
        out = _call("computeVehicleReadiness", v)
        self.assertEqual(out["overall"], "warn")
        self.assertEqual(_by_key(out)["companion"]["status"], "warn")

    def test_mission_missing_makes_overall_nogo(self):
        v = dict(_GOOD_LINKED)
        v.pop("mission_uploaded", None)
        v.pop("mission_total", None)
        out = _call("computeVehicleReadiness", v)
        self.assertEqual(out["overall"], "nogo")
        self.assertEqual(_by_key(out)["mission"]["status"], "nogo")

    def test_throttle_high_makes_overall_nogo(self):
        v = dict(_GOOD_LINKED, rc3=1200)
        out = _call("computeVehicleReadiness", v)
        self.assertEqual(out["overall"], "nogo")
        self.assertEqual(_by_key(out)["throttle"]["status"], "nogo")


class TestLaunchGateBackstop(unittest.TestCase):
    """The authoritative fold: even when a vehicle's worst ROW is only 'warn',
    a launch-gate block (computeLaunchReadiness().ready === false) must force the
    overall verdict to NO-GO — so the board can never read GO/CAUTION while the
    launch button is disabled. These cases have NO 'nogo' row on their own; they
    prove the fold rather than a row override."""

    def _launch_ready(self, vehicle, *gates):
        return _call("computeLaunchReadiness", vehicle, *gates)["ready"]

    def test_waiting_for_prearm_status_folds_to_nogo(self):
        v = dict(_GOOD_LINKED, prearm_check_state="no_sys_status")
        self.assertFalse(self._launch_ready(v))          # launch blocked
        out = _call("computeVehicleReadiness", v)
        self.assertEqual(out["overall"], "nogo")         # backstop forces NO-GO
        # ...but the reason row is still only 'warn' and present in the checks,
        # so the header NO-GO always has a visible current-row reason.
        self.assertEqual(_by_key(out)["prearm"]["status"], "warn")

    def test_unknown_prearm_state_folds_to_nogo(self):
        v = dict(_GOOD_LINKED, prearm_check_state="some_future_state")
        self.assertFalse(self._launch_ready(v))
        self.assertEqual(_call("computeVehicleReadiness", v)["overall"], "nogo")

    def test_gps_accuracy_block_folds_to_nogo(self):
        v = dict(_GOOD_LINKED, gps_hacc=9.0)
        self.assertFalse(self._launch_ready(v, _GATES_GPSACC))
        out = _call("computeVehicleReadiness", v, _GATES_GPSACC)
        self.assertEqual(out["overall"], "nogo")
        # GPS row is only 'warn' for a high HACC, but the gate block wins.
        self.assertEqual(_by_key(out)["gps"]["status"], "warn")


class TestFleetStatus(unittest.TestCase):
    def test_empty_list_is_na(self):
        self.assertEqual(_call("computeFleetStatus", []), "na")

    def test_all_link_down_is_na(self):
        vehicles = [dict(_GOOD_LINKED, link_ok=False), dict(_GOOD_LINKED, link_ok=False)]
        self.assertEqual(_call("computeFleetStatus", vehicles), "na")

    def test_all_go_is_go(self):
        vehicles = [_GOOD_LINKED, _GOOD_LINKED]
        self.assertEqual(_call("computeFleetStatus", vehicles), "go")

    def test_one_warn_makes_fleet_warn(self):
        warn_v = dict(_GOOD_LINKED, prearm_check_state="checks_disabled")
        vehicles = [_GOOD_LINKED, warn_v]
        self.assertEqual(_call("computeFleetStatus", vehicles), "warn")

    def test_one_nogo_makes_fleet_nogo_even_with_a_warn(self):
        warn_v = dict(_GOOD_LINKED, prearm_check_state="checks_disabled")
        nogo_v = dict(_GOOD_LINKED, gps_fix=0)
        vehicles = [_GOOD_LINKED, warn_v, nogo_v]
        self.assertEqual(_call("computeFleetStatus", vehicles), "nogo")

    def test_na_vehicle_does_not_mask_a_real_issue(self):
        down_v = dict(_GOOD_LINKED, link_ok=False)
        nogo_v = dict(_GOOD_LINKED, gps_fix=0)
        vehicles = [down_v, nogo_v]
        self.assertEqual(_call("computeFleetStatus", vehicles), "nogo")

    def test_companion_down_makes_fleet_nogo(self):
        # The reported bug: a downed companion must roll up to a NO-GO fleet
        # verdict, matching the launch gate — not a green fleet.
        down_cc = dict(_GOOD_LINKED, companion_status="down", companion_ok=False)
        vehicles = [_GOOD_LINKED, down_cc]
        self.assertEqual(_call("computeFleetStatus", vehicles), "nogo")


if __name__ == "__main__":
    unittest.main()
