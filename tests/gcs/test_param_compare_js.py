"""Tests for src/gcs/frontend/src/utils/paramCompare.js.

Locks the compare model that backs the "Compare parameters" feature:
    - "differs" uses the SAME coerce/equality semantics as the write path, so a
      row shown as differing is exactly what apply would submit (int 1.9 vs 1 is
      "same"; 2.9 vs 1 writes the stored value 2),
    - per-UAV fileStored (ap_type may differ across vehicles) and partial presence,
    - unloaded snapshot is distinct from "absent on vehicle",
    - with the skip toggle on (default) only "actionable" rows (plain diffs present
      on the vehicle) are selected; identity/cal AND bus/port rows are deselected
      (and hidden by the tab) yet still collectable when explicitly selected,
    - duplicate file rows (last-wins) and the vehicle-only count are reported.
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
    _strip(_read(*_BASE, "fullParams.js")),
    _strip(_read(*_BASE, "paramSkipList.js")),
    _strip(_read(*_BASE, "paramCompare.js")),
])


def _run_js(script):
    # Run from a temp file rather than `node -e` — the concatenated source plus
    # inline snapshots can exceed the Windows command-line length limit (WinError
    # 206). Mirrors the harmonize test runner.
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


def _snap(params, name_order=None):
    """params: list of (name, value, ap_type) or (name, value, ap_type, read_only)."""
    by = {}
    order = []
    for p in params:
        name, value, ap_type = p[0], p[1], p[2]
        read_only = p[3] if len(p) > 3 else False
        by[name] = {"name": name, "value": value, "ap_type": ap_type,
                    "default": 0, "default_known": False, "flags": 0,
                    "read_only": read_only}
        order.append(name)
    return {"paramsByName": by, "nameOrder": order if name_order is None else name_order}


def _compare(file_rows, snaps, sys_ids):
    expr = (
        "computeParamCompare({fileRows:%s, snapshotsByVehicle:%s, sysIds:%s})"
        % (json.dumps(file_rows), json.dumps(snaps), json.dumps(sys_ids))
    )
    return json.loads(_run_js(f"console.log(JSON.stringify({expr}));"))


def _collect(file_rows, snaps, sys_ids, selected):
    script = (
        "const m=computeParamCompare({fileRows:%s, snapshotsByVehicle:%s, sysIds:%s});"
        "console.log(JSON.stringify(collectCompareChanges(m.rows, %s)));"
        % (json.dumps(file_rows), json.dumps(snaps), json.dumps(sys_ids),
           json.dumps(selected))
    )
    return json.loads(_run_js(script))


def _row(model, name):
    return next(r for r in model["rows"] if r["name"] == name)


# ap_type codes.
I8, I32, F = 1, 3, 4


class TestDiffMatchesWritePath(unittest.TestCase):
    def test_int_coerce_equal_is_same(self):
        snaps = {"1": _snap([("X", 1, I32)])}
        m = _compare([{"name": "X", "value": 1.9}], snaps, [1])
        row = _row(m, "X")
        self.assertEqual(row["status"], "same")
        self.assertFalse(row["perVehicle"]["1"]["differs"])
        self.assertEqual(_collect([{"name": "X", "value": 1.9}], snaps, [1], ["X"]), {})

    def test_int_coerce_differ_collects_truncated(self):
        snaps = {"1": _snap([("X", 1, I32)])}
        rows = [{"name": "X", "value": 2.9}]
        m = _compare(rows, snaps, [1])
        self.assertEqual(_row(m, "X")["status"], "differ")
        self.assertEqual(_collect(rows, snaps, [1], ["X"]), {"1": [{"name": "X", "value": 2}]})

    def test_int_clamp_to_storage_range(self):
        snaps = {"1": _snap([("X", 0, I8)])}
        rows = [{"name": "X", "value": 200}]
        self.assertEqual(_collect(rows, snaps, [1], ["X"]), {"1": [{"name": "X", "value": 127}]})

    def test_float_within_tolerance_is_same(self):
        snaps = {"1": _snap([("X", 1.0, F)])}
        m = _compare([{"name": "X", "value": 1.0000001}], snaps, [1])
        self.assertEqual(_row(m, "X")["status"], "same")

    def test_float_outside_tolerance_differs(self):
        snaps = {"1": _snap([("X", 1.0, F)])}
        m = _compare([{"name": "X", "value": 1.5}], snaps, [1])
        self.assertEqual(_row(m, "X")["status"], "differ")


class TestMultiVehicle(unittest.TestCase):
    def test_per_uav_filestored_by_type(self):
        # Same name, int on UAV1 (differs), float on UAV2 (same after coercion).
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 1.9, F)])}
        rows = [{"name": "X", "value": 1.9}]
        m = _compare(rows, snaps, [1, 2])
        row = _row(m, "X")
        self.assertEqual(row["writableSysIds"], [1])
        self.assertTrue(row["perVehicle"]["1"]["differs"])
        self.assertFalse(row["perVehicle"]["2"]["differs"])
        self.assertEqual(_collect(rows, snaps, [1, 2], ["X"]), {"1": [{"name": "X", "value": 1}]})

    def test_partial_presence_writes_only_present(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("Y", 0, I32)])}
        rows = [{"name": "X", "value": 9}]
        m = _compare(rows, snaps, [1, 2])
        row = _row(m, "X")
        self.assertEqual(row["status"], "differ")
        self.assertFalse(row["perVehicle"]["2"]["present"])
        self.assertEqual(_collect(rows, snaps, [1, 2], ["X"]), {"1": [{"name": "X", "value": 9}]})

    def test_default_write_cell_counts(self):
        snaps = {"1": _snap([("X", 1, I32), ("Y", 1, I32)]),
                 "2": _snap([("X", 1, I32)])}
        rows = [{"name": "X", "value": 9}, {"name": "Y", "value": 9}]
        m = _compare(rows, snaps, [1, 2])
        self.assertEqual(m["counts"]["defaultSelectedRows"], 2)
        self.assertEqual(m["counts"]["defaultWriteCells"], 3)  # X on 1&2, Y on 1


class TestUnloadedVsAbsent(unittest.TestCase):
    def test_all_unloaded_is_pending(self):
        m = _compare([{"name": "X", "value": 1}], {}, [1, 2])
        row = _row(m, "X")
        self.assertEqual(row["status"], "pending")
        self.assertFalse(row["perVehicle"]["1"]["loaded"])

    def test_absent_on_all_loaded_is_file_only(self):
        snaps = {"1": _snap([("OTHER", 0, I32)]), "2": _snap([("OTHER", 0, I32)])}
        m = _compare([{"name": "X", "value": 1}], snaps, [1, 2])
        self.assertEqual(_row(m, "X")["status"], "file-only")
        self.assertEqual(m["counts"]["fileOnly"], 1)

    def test_absent_on_loaded_but_other_unloaded_is_pending(self):
        # UAV1 loaded without X, UAV2 not loaded -> can't call file-only yet.
        snaps = {"1": _snap([("OTHER", 0, I32)])}
        m = _compare([{"name": "X", "value": 1}], snaps, [1, 2])
        self.assertEqual(_row(m, "X")["status"], "pending")


class TestSkipListIntegration(unittest.TestCase):
    def test_identity_default_unselected_but_collectable(self):
        snaps = {"1": _snap([("COMPASS_OFS_X", 10, I32)])}
        rows = [{"name": "COMPASS_OFS_X", "value": 25}]
        m = _compare(rows, snaps, [1])
        row = _row(m, "COMPASS_OFS_X")
        self.assertEqual(row["safetyTag"], "identity-cal-skip")
        self.assertTrue(row["isUavSpecific"])
        self.assertEqual(row["status"], "differ")
        self.assertFalse(row["defaultSelected"])
        self.assertEqual(m["counts"]["skippedDiffer"], 1)
        # Explicit override still collects it.
        self.assertEqual(_collect(rows, snaps, [1], ["COMPASS_OFS_X"]),
                         {"1": [{"name": "COMPASS_OFS_X", "value": 25}]})

    def test_identity_never_auto_selected(self):
        # Per-board calibration must NEVER be auto-selected — copying it corrupts
        # the target. There is no toggle that can override this (Codex gate 15:
        # the old "show all" escape hatch is gone — the priority-group sidebar
        # replaced it, and identity/cal stays excluded regardless). Explicit tick
        # still works (see test_identity_default_unselected_but_collectable).
        snaps = {"1": _snap([("COMPASS_OFS_X", 10, I32)])}
        m = _compare([{"name": "COMPASS_OFS_X", "value": 25}], snaps, [1])
        self.assertFalse(_row(m, "COMPASS_OFS_X")["defaultSelected"])

    def test_bus_port_always_default_selected(self):
        # Bus/port is safe shared config, so it's unconditionally a default-
        # selection candidate — unlike identity/cal, and NOT gated by any toggle
        # (Codex gate 15: this used to depend on a skipUavSpecific flag shared
        # with Sync UAVs' own toggle, which let that unrelated mode's state leak
        # into compare's selection).
        rows = [{"name": "SERIAL2_PROTOCOL", "value": 2}]
        snaps = {"1": _snap([("SERIAL2_PROTOCOL", 1, I32)])}
        m = _compare(rows, snaps, [1])
        row = _row(m, "SERIAL2_PROTOCOL")
        self.assertEqual(row["safetyTag"], "bus-port-keep")
        self.assertFalse(row["isUavSpecific"])
        self.assertFalse(row["actionable"])  # tagged rows are never "actionable"
        self.assertTrue(row["defaultSelected"])
        self.assertEqual(m["counts"]["busPortDiffer"], 1)
        self.assertEqual(_collect(rows, snaps, [1], ["SERIAL2_PROTOCOL"]),
                         {"1": [{"name": "SERIAL2_PROTOCOL", "value": 2}]})

    def test_neutral_param_has_no_tag(self):
        snaps = {"1": _snap([("ATC_RAT_RLL_P", 0.1, F)])}
        m = _compare([{"name": "ATC_RAT_RLL_P", "value": 0.2}], snaps, [1])
        row = _row(m, "ATC_RAT_RLL_P")
        self.assertIsNone(row["safetyTag"])
        # A plain differing param present on the vehicle is the actionable case:
        # shown + selected by default.
        self.assertTrue(row["actionable"])
        self.assertTrue(row["defaultSelected"])


class TestRobustness(unittest.TestCase):
    def test_non_finite_values_dropped_and_counted(self):
        snaps = {"1": _snap([("X", 5, I32), ("Y", 5, I32)])}
        rows = [
            {"name": "X", "value": float("nan")},
            {"name": "Y", "value": 9},
            {"name": "Z", "value": float("inf")},
        ]
        m = _compare(rows, snaps, [1])
        self.assertEqual([r["name"] for r in m["rows"]], ["Y"])
        self.assertEqual(m["counts"]["invalid"], 2)

    def test_nonnumeric_string_value_dropped(self):
        snaps = {"1": _snap([("X", 5, I32)])}
        m = _compare([{"name": "X", "value": "abc"}], snaps, [1])
        self.assertEqual(m["rows"], [])
        self.assertEqual(m["counts"]["invalid"], 1)

    def test_zero_value_differs_and_collects(self):
        snaps = {"1": _snap([("X", 5, I32)])}
        rows = [{"name": "X", "value": 0}]
        m = _compare(rows, snaps, [1])
        self.assertEqual(_row(m, "X")["status"], "differ")
        self.assertEqual(_collect(rows, snaps, [1], ["X"]), {"1": [{"name": "X", "value": 0}]})

    def test_empty_sysids_is_pending(self):
        m = _compare([{"name": "X", "value": 1}], {}, [])
        self.assertEqual(_row(m, "X")["status"], "pending")
        self.assertEqual(m["counts"]["vehicleOnly"], 0)

    def test_sysids_as_strings(self):
        snaps = {"1": _snap([("X", 5, I32)])}
        rows = [{"name": "X", "value": 9}]
        m = _compare(rows, snaps, ["1"])
        self.assertEqual(_row(m, "X")["status"], "differ")
        self.assertEqual(_collect(rows, snaps, ["1"], ["X"]), {"1": [{"name": "X", "value": 9}]})

    def test_collect_after_json_roundtrip(self):
        # Mirrors the React path: the model is stored in state (serialisable)
        # before collectCompareChanges runs over it.
        snaps = {"1": _snap([("X", 5, I32)])}
        script = (
            "const m=computeParamCompare({fileRows:%s, snapshotsByVehicle:%s, sysIds:%s});"
            "const rt=JSON.parse(JSON.stringify(m.rows));"
            "console.log(JSON.stringify(collectCompareChanges(rt, ['X'])));"
            % (json.dumps([{"name": "X", "value": 9}]), json.dumps(snaps), json.dumps([1]))
        )
        self.assertEqual(json.loads(_run_js(script)), {"1": [{"name": "X", "value": 9}]})

    def test_partial_loaded_absence_no_diff_is_same(self):
        # UAV1 loaded with matching value, UAV2 not loaded -> 'same' (reflects
        # loaded data; UAV2 cell is pending, not absent).
        snaps = {"1": _snap([("X", 9, I32)])}
        m = _compare([{"name": "X", "value": 9}], snaps, [1, 2])
        self.assertEqual(_row(m, "X")["status"], "same")
        self.assertFalse(_row(m, "X")["perVehicle"]["2"]["loaded"])


class TestSelectionHelpers(unittest.TestCase):
    def _model_then(self, fn_call, file_rows, snaps, sys_ids):
        script = (
            "const m=computeParamCompare({fileRows:%s, snapshotsByVehicle:%s, "
            "sysIds:%s});console.log(JSON.stringify(%s));"
            % (json.dumps(file_rows), json.dumps(snaps), json.dumps(sys_ids), fn_call)
        )
        return json.loads(_run_js(script))

    def test_default_selection_is_differing_non_skipped(self):
        snaps = {"1": _snap([("X", 1, I32), ("COMPASS_OFS_X", 1, I32), ("Y", 5, I32)])}
        rows = [
            {"name": "X", "value": 9},               # differ, neutral -> selected
            {"name": "COMPASS_OFS_X", "value": 9},    # differ, identity -> skipped
            {"name": "Y", "value": 5},                # same -> not selected
        ]
        self.assertEqual(
            self._model_then("defaultCompareSelection(m.rows)", rows, snaps, [1]),
            ["X"],
        )

    def test_default_selection_adds_bus_port_but_never_identity(self):
        # Bus/port always joins the default selection; per-board identity/cal
        # never does, unconditionally (Codex gate 15 — no toggle affects this).
        snaps = {"1": _snap([("X", 1, I32), ("COMPASS_OFS_X", 1, I32),
                             ("SERIAL2_PROTOCOL", 1, I32)])}
        rows = [{"name": "X", "value": 9},
                {"name": "COMPASS_OFS_X", "value": 9},
                {"name": "SERIAL2_PROTOCOL", "value": 2}]
        got = self._model_then("defaultCompareSelection(m.rows)", rows, snaps, [1])
        self.assertEqual(sorted(got), ["SERIAL2_PROTOCOL", "X"])  # no COMPASS_OFS_X

    def test_reconcile_drops_no_longer_writable_keeps_differing(self):
        # X still differs (kept), Y is now same (dropped), Z absent from file model.
        snaps = {"1": _snap([("X", 1, I32), ("Y", 5, I32)])}
        rows = [{"name": "X", "value": 9}, {"name": "Y", "value": 5}]
        got = self._model_then(
            "reconcileCompareSelection(['X','Y','Z'], m.rows)", rows, snaps, [1])
        self.assertEqual(got, ["X"])

    def test_reconcile_preserves_manually_selected_identity_that_still_differs(self):
        snaps = {"1": _snap([("COMPASS_OFS_X", 1, I32)])}
        rows = [{"name": "COMPASS_OFS_X", "value": 9}]  # identity, still differs
        got = self._model_then(
            "reconcileCompareSelection(['COMPASS_OFS_X'], m.rows)", rows, snaps, [1])
        self.assertEqual(got, ["COMPASS_OFS_X"])


class TestReadOnlyViewOnly(unittest.TestCase):
    """Read-only (firmware-maintained) params — STAT_* counters, *_DEVID,
    MIS_TOTAL, *_GND_PRESS — can never be written. They're kept as VIEW-ONLY rows
    (readOnly:true) so the operator can still inspect them in the dedicated
    "Read-only" sidebar bucket, but never as a writable/selectable differ
    (operator report: "we should filter also readonly params ... let also user to
    see the readonly")."""

    def test_readonly_param_is_view_only_row(self):
        snaps = {"1": _snap([("STAT_BOOTCNT", 100, I32, True), ("X", 1, I32)])}
        rows = [{"name": "STAT_BOOTCNT", "value": 5}, {"name": "X", "value": 9}]
        m = _compare(rows, snaps, [1])
        ro = _row(m, "STAT_BOOTCNT")
        self.assertTrue(ro["readOnly"])
        self.assertEqual(ro["status"], "read-only")
        self.assertFalse(ro["anyDiffers"])
        self.assertFalse(ro["defaultSelected"])
        self.assertEqual(ro["writableSysIds"], [])
        self.assertFalse(ro["perVehicle"]["1"]["differs"])  # no diff highlight
        self.assertEqual(m["counts"]["readOnly"], 1)
        self.assertEqual(m["counts"]["differ"], 1)  # only X counts as a differ

    def test_readonly_not_default_selected_or_collectable(self):
        # Naturally differs per board; must never be written even if named.
        snaps = {"1": _snap([("STAT_RUNTIME", 100, I32, True)]),
                 "2": _snap([("STAT_RUNTIME", 200, I32, True)])}
        rows = [{"name": "STAT_RUNTIME", "value": 0}]
        m = _compare(rows, snaps, [1, 2])
        self.assertTrue(_row(m, "STAT_RUNTIME")["readOnly"])
        self.assertEqual(m["counts"]["differ"], 0)
        self.assertEqual(_collect(rows, snaps, [1, 2], ["STAT_RUNTIME"]), {})

    def test_single_readonly_cell_marks_row(self):
        # The read_only flag is name-derived and uniform, but one flagged cell is
        # enough to mark the row view-only (defensive against a partial snapshot).
        snaps = {"1": _snap([("STAT_FLTTIME", 10, I32, True)]),
                 "2": _snap([("STAT_FLTTIME", 20, I32, False)])}
        m = _compare([{"name": "STAT_FLTTIME", "value": 0}], snaps, [1, 2])
        self.assertTrue(_row(m, "STAT_FLTTIME")["readOnly"])
        self.assertEqual(m["counts"]["readOnly"], 1)

    def test_non_readonly_param_not_flagged(self):
        snaps = {"1": _snap([("X", 1, I32, False)])}
        m = _compare([{"name": "X", "value": 9}], snaps, [1])
        self.assertFalse(_row(m, "X").get("readOnly", False))
        self.assertEqual(m["counts"]["readOnly"], 0)

    def test_readonly_file_only_not_flagged(self):
        # read_only comes from the vehicle record; a param absent on every vehicle
        # (file-only) has no flag to read, so it stays a plain file-only row.
        snaps = {"1": _snap([("OTHER", 0, I32)])}
        m = _compare([{"name": "STAT_BOOTCNT", "value": 5}], snaps, [1])
        self.assertEqual(_row(m, "STAT_BOOTCNT")["status"], "file-only")
        self.assertFalse(_row(m, "STAT_BOOTCNT").get("readOnly", False))
        self.assertEqual(m["counts"]["readOnly"], 0)


class TestDuplicatesAndVehicleOnly(unittest.TestCase):
    def test_duplicate_last_wins_first_order(self):
        snaps = {"1": _snap([("X", 0, I32)])}
        rows = [{"name": "X", "value": 1}, {"name": "X", "value": 5}]
        m = _compare(rows, snaps, [1])
        self.assertEqual(m["order"], ["X"])
        self.assertEqual(m["duplicateNames"], ["X"])
        self.assertEqual(m["counts"]["duplicates"], 1)
        self.assertEqual(_row(m, "X")["fileValue"], 5)

    def test_vehicle_only_count(self):
        snaps = {"1": _snap([("A", 0, I32), ("B", 0, I32), ("C", 0, I32)]),
                 "2": _snap([("A", 0, I32), ("B", 0, I32)])}
        m = _compare([{"name": "A", "value": 1}], snaps, [1, 2])
        self.assertEqual(m["counts"]["vehicleOnly"], 2)  # B, C (counted once)

    def test_order_preserves_file_sequence(self):
        snaps = {"1": _snap([("A", 0, I32), ("B", 0, I32)])}
        rows = [{"name": "B", "value": 1}, {"name": "A", "value": 1}]
        m = _compare(rows, snaps, [1])
        self.assertEqual(m["order"], ["B", "A"])


if __name__ == "__main__":
    unittest.main()
