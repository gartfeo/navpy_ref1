"""Tests for src/gcs/frontend/src/utils/paramGroups.js.

Locks the functional priority-group classifier that backs the compare sidebar:
    - the NavPy AAS_* navigation group is first,
    - per-board identity/cal is a HARD override (never mis-sorts into tuning),
    - wiring/ports is a low-priority ordinary group,
    - the two low groups reuse the skip-list (single source of truth),
    - summarizeCompareGroups returns ordered non-empty groups with differ counts,
      file-only bucketed last regardless of function.
"""
import json
import os
import subprocess
import tempfile
import unittest


def _read(*parts):
    return open(os.path.normpath(os.path.join(os.path.dirname(__file__), *parts)),
                encoding="utf-8").read()


def _strip(src):
    out = (src.replace("export function ", "function ")
              .replace("export const ", "const "))
    return "\n".join(
        line for line in out.splitlines() if not line.lstrip().startswith("import ")
    )


_BASE = ("..", "..", "src", "gcs", "frontend", "src", "utils")
_JS_SRC = "\n".join([
    _strip(_read(*_BASE, "paramSkipList.js")),
    _strip(_read(*_BASE, "paramGroups.js")),
])


def _run_js(script):
    fd, path = tempfile.mkstemp(suffix=".js")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(_JS_SRC + "\n" + script)
        result = subprocess.run(["node", path], capture_output=True, text=True, timeout=10)
    finally:
        os.unlink(path)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _classify(name):
    return _run_js(f"console.log(classifyParam({json.dumps(name)}));")


def _summary(rows):
    return json.loads(_run_js(
        f"console.log(JSON.stringify(summarizeCompareGroups({json.dumps(rows)})));"))


def _bucket(row):
    return _run_js(f"console.log(groupBucketForRow({json.dumps(row)}));")


class TestClassify(unittest.TestCase):
    def test_navigation_first(self):
        # AAS_* core + MTUNE_* in-flight tuning support both bucket to navigation.
        for n in ["AAS_TARG_ALT", "AAS_DEL_PITCH", "AAS_NAV_MIN_ALT",
                  "MTUNE_BTMSK", "MTUNE_MIN_PTCH_A", "MTUNE_TRG_CH"]:
            self.assertEqual(_classify(n), "navigation")

    def test_control_tuning(self):
        for n in ["PTCH2SRV_RMAX_DN", "TECS_PITCH_MAX", "NAVL1_PERIOD",
                  "RLL_RATE_P", "AUTOTUNE_LEVEL", "ARSPD_FBW_MIN"]:
            self.assertEqual(_classify(n), "control-tuning")

    def test_operator_reclassifications_to_tuning(self):
        # Corrections from field use (ArduPlane 4.4+ names that had fallen to
        # 'other', plus attitude limits treated as tune targets on this airframe).
        for n in ["AIRSPEED_MIN", "AIRSPEED_MAX", "AIRSPEED_CRUISE",
                  "LAND_PITCH_DEG", "PTCH_LIM_MAX_DEG", "PTCH_LIM_MIN_DEG",
                  "ROLL_LIMIT_DEG", "LIM_ROLL_CD", "LIM_PITCH_MAX",
                  "RC_OVERRIDE_TIME", "SERVO_SBUS_RATE"]:
            self.assertEqual(_classify(n), "control-tuning")

    def test_navigation_mission(self):
        for n in ["RTL_ALTITUDE", "MIS_TOTAL", "FENCE_ACTION", "TKOFF_LVL_ALT", "WP_RADIUS"]:
            self.assertEqual(_classify(n), "navigation-mission")

    def test_failsafe(self):
        for n in ["FS_LONG_ACTN", "BATT_LOW_MAH", "ARMING_CHECK", "THR_FAILSAFE"]:
            self.assertEqual(_classify(n), "failsafe-limits")

    def test_modes_rc(self):
        for n in ["RC10_TRIM", "FLTMODE1", "SERVO3_FUNCTION", "RCMAP_ROLL"]:
            self.assertEqual(_classify(n), "flight-modes-rc")

    def test_airframe(self):
        for n in ["BRD_OPTIONS", "INS_FAST_SAMPLE", "EK3_ENABLE", "LOG_BITMASK",
                  "BATT_VOLT_PIN", "BATT_CURR_PIN"]:
            self.assertEqual(_classify(n), "airframe-system")

    def test_battery_pins_are_airframe_but_cal_stays_per_board(self):
        # The ADC pins are wiring/config (airframe); the multipliers are per-board cal.
        self.assertEqual(_classify("BATT_VOLT_PIN"), "airframe-system")
        self.assertEqual(_classify("BATT_CURR_PIN"), "airframe-system")
        self.assertEqual(_classify("BATT_VOLT_MULT"), "per-board-cal-identity")
        self.assertEqual(_classify("BATT_AMP_PERVLT"), "per-board-cal-identity")

    def test_sensors_wiring(self):
        for n in ["SERIAL2_PROTOCOL", "GPS_TYPE", "ARSPD_USE", "COMPASS_ORIENT", "AHRS_EKF_TYPE"]:
            self.assertEqual(_classify(n), "sensors-wiring-ports")

    def test_per_board_is_hard_override(self):
        # Identity/cal wins even against families that would otherwise match a
        # higher-priority functional group.
        for n in ["COMPASS_OFS_X", "AHRS_TRIM_X", "ARSPD_OFFSET", "ARSPD_DEVID",
                  "BATT_VOLT_MULT", "COMPASS_DEV_ID", "BARO1_GND_PRESS", "SYSID_THISMAV"]:
            self.assertEqual(_classify(n), "per-board-cal-identity")

    def test_unknown_is_other(self):
        self.assertEqual(_classify("ZZZ_MADE_UP"), "other")

    def test_arspd_three_way_split(self):
        # The same family fans out by function — a good override/precedence probe.
        self.assertEqual(_classify("ARSPD_OFFSET"), "per-board-cal-identity")  # cal
        self.assertEqual(_classify("ARSPD_USE"), "sensors-wiring-ports")       # wiring
        self.assertEqual(_classify("ARSPD_FBW_MIN"), "control-tuning")         # envelope


class TestSummarize(unittest.TestCase):
    def _row(self, name, status="differ"):
        return {"name": name, "status": status}

    def test_ordered_nonempty_with_counts(self):
        rows = [
            self._row("AAS_TARG_ALT"), self._row("TECS_PITCH_MAX"),
            self._row("PTCH2SRV_P"), self._row("COMPASS_OFS_X"),
            self._row("SERIAL2_PROTOCOL", "same"),   # same -> not counted
            self._row("SOME_FILE_ONLY", "file-only"),
        ]
        s = _summary(rows)
        self.assertEqual(s["counts"]["navigation"], 1)
        self.assertEqual(s["counts"]["control-tuning"], 2)
        self.assertEqual(s["counts"]["per-board-cal-identity"], 1)
        self.assertEqual(s["counts"]["file-only"], 1)
        self.assertEqual(s["totalDiffer"], 4)  # AAS + 2 tuning + compass
        # Priority order, file-only last, empty groups dropped.
        self.assertEqual(
            s["order"],
            ["navigation", "control-tuning", "per-board-cal-identity", "file-only"],
        )

    def test_same_rows_never_counted(self):
        rows = [self._row("TECS_PITCH_MAX", "same"), self._row("AAS_TARG_ALT", "same")]
        s = _summary(rows)
        self.assertEqual(s["order"], [])
        self.assertEqual(s["totalDiffer"], 0)


class TestReadOnlyBucket(unittest.TestCase):
    """Read-only (view-only firmware) rows get their own bucket, which wins over
    the functional/status classification so a firmware param never lands in a
    writable group or the "All" total (operator: "let also user to see the
    readonly" — a dedicated sidebar group, Option B)."""

    def test_readonly_row_buckets_to_read_only(self):
        # readOnly wins even for a name that would otherwise classify elsewhere.
        self.assertEqual(
            _bucket({"name": "STAT_BOOTCNT", "status": "read-only", "readOnly": True}),
            "read-only")
        self.assertEqual(
            _bucket({"name": "MIS_TOTAL", "status": "read-only", "readOnly": True}),
            "read-only")

    def test_non_readonly_row_buckets_normally(self):
        self.assertEqual(_bucket({"name": "AAS_TARG_ALT", "status": "differ"}), "navigation")
        self.assertEqual(_bucket({"name": "SOME_FILE", "status": "file-only"}), "file-only")

    def test_summary_read_only_bucket_last_and_not_in_total(self):
        rows = [
            {"name": "AAS_TARG_ALT", "status": "differ"},
            {"name": "STAT_BOOTCNT", "status": "read-only", "readOnly": True},
            {"name": "STAT_RUNTIME", "status": "read-only", "readOnly": True},
            {"name": "SOME_FILE", "status": "file-only"},
        ]
        s = _summary(rows)
        self.assertEqual(s["counts"]["read-only"], 2)
        self.assertEqual(s["totalDiffer"], 1)  # read-only rows never count as differ
        # Both status buckets sink to the bottom: file-only, then read-only last.
        self.assertEqual(s["order"], ["navigation", "file-only", "read-only"])


if __name__ == "__main__":
    unittest.main()
