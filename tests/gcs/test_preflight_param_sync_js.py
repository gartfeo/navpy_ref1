"""Tests for computeFleetParamSync in preflightReadiness.js (CONF-01, D-01..D-06).

Locks the fleet-level "params not synced" preflight check:
    - it reuses computeFleetHarmonize (skipUavSpecific:true) rather than
      re-implementing per-param comparison, so this row and the Sync UAVs /
      Harmonize UI can never disagree (D-02),
    - a divergence on a shared, non-skip-listed param (e.g. AAS_NAV_AUTO_CM)
      -> 'warn', with detail naming the param and its distinctValues,
    - a divergence confined to an identity/cal/bus skip-listed param -> 'go'
      (not 'warn') — those are SUPPOSED to differ per board,
    - fewer than two vehicles with a loaded snapshot -> 'na', never 'warn'
      (D-03/D-06: nothing to compare yet is not itself a warning).

preflightReadiness.js imports from prearmChecks.js and paramHarmonize.js
(which in turn imports fullParams.js and paramSkipList.js), so all five
sources are concatenated with their ES-module syntax stripped before
evaluation — matching the project's Node-subprocess test style for frontend
utilities (see test_preflight_readiness_js.py / test_param_harmonize_js.py).
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


def _strip(src):
    # Strip `export ` keywords and any import lines so Node can eval the
    # concatenated bundle as one plain script (same technique as the other
    # tests/gcs/*_js.py runners).
    out = src.replace("export function ", "function ").replace("export const ", "const ").replace("export ", "")
    return re.sub(r"^import .*\n", "", out, flags=re.M)


_JS_SRC = "\n".join([
    _strip(_load("fullParams.js")),
    _strip(_load("paramSkipList.js")),
    _strip(_load("paramHarmonize.js")),
    _strip(_load("prearmChecks.js")),
    _strip(_load("preflightReadiness.js")),
])


def _run_js(script):
    result = run_node(_JS_SRC + "\n" + script, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _snap(params):
    """params: list of (name, value, ap_type) or (name, value, ap_type, read_only)."""
    by, order = {}, []
    for p in params:
        name, value, ap_type = p[0], p[1], p[2]
        read_only = p[3] if len(p) > 3 else False
        by[name] = {"name": name, "value": value, "ap_type": ap_type,
                    "default": 0, "default_known": False, "flags": 0,
                    "read_only": read_only}
        order.append(name)
    return {"paramsByName": by, "nameOrder": order}


def _param_sync(snaps, sys_ids):
    expr = "computeFleetParamSync({snapshotsByVehicle:%s, sysIds:%s})" % (
        json.dumps(snaps), json.dumps(sys_ids),
    )
    return json.loads(_run_js(f"console.log(JSON.stringify({expr}));"))


I8, I32, F = 1, 3, 4


class TestAgreement(unittest.TestCase):
    def test_three_uavs_agree_is_go(self):
        snaps = {
            "1": _snap([("AAS_NAV_AUTO_CM", 1, I8), ("NAVL1_PERIOD", 20, F)]),
            "2": _snap([("AAS_NAV_AUTO_CM", 1, I8), ("NAVL1_PERIOD", 20, F)]),
            "3": _snap([("AAS_NAV_AUTO_CM", 1, I8), ("NAVL1_PERIOD", 20, F)]),
        }
        r = _param_sync(snaps, [1, 2, 3])
        self.assertEqual(r["key"], "paramSync")
        self.assertEqual(r["status"], "go")
        self.assertEqual(r["detail"]["divergent"], 0)
        self.assertEqual(r["detail"]["rows"], [])


class TestDivergenceWarns(unittest.TestCase):
    def test_confirm_mode_divergence_warns_with_detail(self):
        snaps = {
            "1": _snap([("AAS_NAV_AUTO_CM", 1, I8)]),
            "2": _snap([("AAS_NAV_AUTO_CM", 0, I8)]),
        }
        r = _param_sync(snaps, [1, 2])
        self.assertEqual(r["status"], "warn")
        self.assertEqual(r["detail"]["divergent"], 1)
        row = r["detail"]["rows"][0]
        self.assertEqual(row["name"], "AAS_NAV_AUTO_CM")
        # distinctValues names which sysIds hold which value.
        values_by_sid = {}
        for group in row["distinctValues"]:
            for sid in group["sysIds"]:
                values_by_sid[sid] = group["value"]
        self.assertEqual(values_by_sid, {1: 1, 2: 0})

    def test_any_generic_shared_param_divergence_warns(self):
        # D-02: generic over ALL non-UAV-specific params, not confirm-mode only.
        snaps = {
            "1": _snap([("NAVL1_PERIOD", 20, F)]),
            "2": _snap([("NAVL1_PERIOD", 25, F)]),
        }
        r = _param_sync(snaps, [1, 2])
        self.assertEqual(r["status"], "warn")
        self.assertEqual([row["name"] for row in r["detail"]["rows"]], ["NAVL1_PERIOD"])

    def test_three_uavs_two_of_three_still_warns(self):
        snaps = {
            "1": _snap([("AAS_NAV_AUTO_CM", 1, I8)]),
            "2": _snap([("AAS_NAV_AUTO_CM", 1, I8)]),
            "3": _snap([("AAS_NAV_AUTO_CM", 0, I8)]),
        }
        r = _param_sync(snaps, [1, 2, 3])
        self.assertEqual(r["status"], "warn")
        self.assertEqual(r["detail"]["divergent"], 1)


class TestSkipListExclusion(unittest.TestCase):
    def test_identity_cal_only_divergence_is_go_not_warn(self):
        # Per-board compass calibration is SUPPOSED to differ across UAVs —
        # never a "not synced" warning.
        snaps = {
            "1": _snap([("COMPASS_OFS_X", 10, I32)]),
            "2": _snap([("COMPASS_OFS_X", 25, I32)]),
        }
        r = _param_sync(snaps, [1, 2])
        self.assertEqual(r["status"], "go")
        self.assertEqual(r["detail"]["divergent"], 0)
        self.assertEqual(r["detail"]["rows"], [])

    def test_mixed_skip_and_actionable_divergence_only_reports_actionable(self):
        snaps = {
            "1": _snap([("COMPASS_OFS_X", 10, I32), ("AAS_NAV_AUTO_CM", 1, I8)]),
            "2": _snap([("COMPASS_OFS_X", 25, I32), ("AAS_NAV_AUTO_CM", 0, I8)]),
        }
        r = _param_sync(snaps, [1, 2])
        self.assertEqual(r["status"], "warn")
        self.assertEqual([row["name"] for row in r["detail"]["rows"]], ["AAS_NAV_AUTO_CM"])

    def test_readonly_divergence_is_go_not_warn(self):
        # Firmware-maintained counters differ per board and can't be written
        # via the Harmonize UI this row links to — not actionable noise here.
        snaps = {
            "1": _snap([("STAT_BOOTCNT", 10, I32, True)]),
            "2": _snap([("STAT_BOOTCNT", 25, I32, True)]),
        }
        r = _param_sync(snaps, [1, 2])
        self.assertEqual(r["status"], "go")
        self.assertEqual(r["detail"]["rows"], [])


class TestNotEnoughData(unittest.TestCase):
    def test_single_vehicle_is_na(self):
        snaps = {"1": _snap([("AAS_NAV_AUTO_CM", 1, I8)])}
        r = _param_sync(snaps, [1])
        self.assertEqual(r["status"], "na")
        self.assertIsNone(r["detail"])

    def test_zero_vehicles_is_na(self):
        r = _param_sync({}, [])
        self.assertEqual(r["status"], "na")
        self.assertIsNone(r["detail"])

    def test_one_loaded_one_unloaded_is_na(self):
        # Second sysId is connected but its snapshot hasn't downloaded yet
        # (D-06 auto-download in flight) -> nothing to compare yet, not a warn.
        snaps = {"1": _snap([("AAS_NAV_AUTO_CM", 1, I8)])}
        r = _param_sync(snaps, [1, 2])
        self.assertEqual(r["status"], "na")


class TestReusesFleetHarmonize(unittest.TestCase):
    def test_output_matches_fleet_harmonize_divergent_rows(self):
        # No re-implemented per-param comparison: the divergent row names must
        # be exactly the non-uav-specific 'divergent' rows computeFleetHarmonize
        # itself produces for the same input.
        snaps = {
            "1": _snap([("AAS_NAV_AUTO_CM", 1, I8), ("COMPASS_OFS_X", 1, I32)]),
            "2": _snap([("AAS_NAV_AUTO_CM", 0, I8), ("COMPASS_OFS_X", 2, I32)]),
        }
        script = (
            "const h=computeFleetHarmonize({snapshotsByVehicle:%s, sysIds:%s, skipUavSpecific:true});"
            "const expected=h.rows.filter(r=>r.status==='divergent'&&!r.isUavSpecific).map(r=>r.name);"
            "const s=computeFleetParamSync({snapshotsByVehicle:%s, sysIds:%s});"
            "console.log(JSON.stringify({expected, got: s.detail.rows.map(r=>r.name)}));"
            % (json.dumps(snaps), json.dumps([1, 2]), json.dumps(snaps), json.dumps([1, 2]))
        )
        result = json.loads(_run_js(script))
        self.assertEqual(result["got"], result["expected"])
        self.assertEqual(result["expected"], ["AAS_NAV_AUTO_CM"])


if __name__ == "__main__":
    unittest.main()
