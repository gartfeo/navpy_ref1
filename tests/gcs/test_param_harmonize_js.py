"""Tests for src/gcs/frontend/src/utils/paramHarmonize.js.

Locks the fleet-harmonize model that backs the "Sync UAVs" feature:
    - a row appears only when >= 2 PRESENT UAVs disagree (partial presence and
      single-UAV params are not harmonizable),
    - majority = the unique top value (2-of-3 wins; a 1-1 tie or all-distinct has
      no majority and is not auto-selected),
    - identity/calibration rows are shown but excluded from the default selection,
    - the winner is coerced/compared PER TARGET so the collected changes equal what
      the scoped writer (resolveSubmissions) would submit,
    - type-mismatch rows are flagged and not default-selected.
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
    _strip(_read(*_BASE, "paramHarmonize.js")),
])


def _run_js(script):
    # Run from a temp file rather than `node -e` — the concatenated source plus
    # inline snapshots can exceed the Windows command-line length limit.
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


def _harmonize(snaps, sys_ids, skip=True):
    expr = ("computeFleetHarmonize({snapshotsByVehicle:%s, sysIds:%s, skipUavSpecific:%s})"
            % (json.dumps(snaps), json.dumps(sys_ids), "true" if skip else "false"))
    return json.loads(_run_js(f"console.log(JSON.stringify({expr}));"))


def _collect(snaps, sys_ids, selected, winners=None, skip=True):
    script = (
        "const m=computeFleetHarmonize({snapshotsByVehicle:%s, sysIds:%s, skipUavSpecific:%s});"
        "console.log(JSON.stringify(collectHarmonizeChanges(m.rows, %s, %s)));"
        % (json.dumps(snaps), json.dumps(sys_ids), "true" if skip else "false",
           json.dumps(selected), json.dumps(winners or {}))
    )
    return json.loads(_run_js(script))


def _row(model, name):
    return next((r for r in model["rows"] if r["name"] == name), None)


def _base(snaps, sys_ids, name, winners=None, skip=True):
    """effectiveBaseSysId for one row — the UAV cell badged as the base source."""
    script = (
        "const m=computeFleetHarmonize({snapshotsByVehicle:%s, sysIds:%s, skipUavSpecific:%s});"
        "const r=m.rows.find(x=>x.name===%s);"
        "console.log(JSON.stringify(effectiveBaseSysId(r, %s)));"
        % (json.dumps(snaps), json.dumps(sys_ids), "true" if skip else "false",
           json.dumps(name), json.dumps(winners or {}))
    )
    return json.loads(_run_js(script))


I8, I32, F = 1, 3, 4


class TestDivergenceAndMajority(unittest.TestCase):
    def test_two_of_three_majority(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, I32)])}
        m = _harmonize(snaps, [1, 2, 3])
        row = _row(m, "X")
        self.assertEqual(row["modalValue"], 5)
        self.assertEqual(row["defaultWinner"], 5)
        self.assertTrue(row["defaultSelected"])
        # collect pushes the winner (5) only to the outlier (u3).
        self.assertEqual(_collect(snaps, [1, 2, 3], ["X"]), {"3": [{"name": "X", "value": 5}]})

    def test_two_uav_tie_has_no_majority(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 9, I32)])}
        m = _harmonize(snaps, [1, 2])
        row = _row(m, "X")
        self.assertIsNone(row["modalValue"])
        self.assertIsNone(row["defaultWinner"])
        self.assertFalse(row["defaultSelected"])
        self.assertEqual(m["counts"]["noMajority"], 1)
        # No winner -> nothing collected even if selected.
        self.assertEqual(_collect(snaps, [1, 2], ["X"]), {})

    def test_all_distinct_has_no_majority(self):
        snaps = {"1": _snap([("X", 1, I32)]), "2": _snap([("X", 2, I32)]),
                 "3": _snap([("X", 3, I32)])}
        row = _row(_harmonize(snaps, [1, 2, 3]), "X")
        self.assertIsNone(row["defaultWinner"])
        self.assertFalse(row["defaultSelected"])

    def test_agreeing_param_is_not_a_row(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)])}
        self.assertEqual(_harmonize(snaps, [1, 2])["rows"], [])

    def test_manual_winner_override_selects_and_collects(self):
        # 1-1 tie: operator picks 9 as the winner -> push to u1.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 9, I32)])}
        self.assertEqual(
            _collect(snaps, [1, 2], ["X"], winners={"X": 9}),
            {"1": [{"name": "X", "value": 9}]},
        )


class TestPluralityAndLoading(unittest.TestCase):
    def test_four_uav_two_one_one_has_modal_winner(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 8, I32)]), "4": _snap([("X", 9, I32)])}
        row = _row(_harmonize(snaps, [1, 2, 3, 4]), "X")
        self.assertEqual(row["modalValue"], 5)      # plurality (2 vs 1 vs 1)
        self.assertTrue(row["defaultSelected"])
        self.assertEqual(_collect(snaps, [1, 2, 3, 4], ["X"]),
                         {"3": [{"name": "X", "value": 5}], "4": [{"name": "X", "value": 5}]})

    def test_four_uav_two_two_tie_has_no_modal(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, I32)]), "4": _snap([("X", 9, I32)])}
        row = _row(_harmonize(snaps, [1, 2, 3, 4]), "X")
        self.assertIsNone(row["modalValue"])
        self.assertFalse(row["defaultSelected"])

    def test_unloaded_uav_blocks_default_selection(self):
        # u3 connected but its snapshot hasn't loaded -> allLoaded false -> the
        # fleet isn't fully known, so nothing auto-selects.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 9, I32)])}
        m = _harmonize(snaps, [1, 2, 3])
        self.assertFalse(m["allLoaded"])
        row = _row(m, "X")
        self.assertFalse(row["defaultSelected"])


class TestPresence(unittest.TestCase):
    def test_single_present_uav_is_not_harmonizable(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("OTHER", 0, I32)])}
        self.assertIsNone(_row(_harmonize(snaps, [1, 2]), "X"))

    def test_partial_presence_divergent_flagged(self):
        # X present on u1,u2 (differ), absent on u3 -> divergent + partial; u3 not written.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 9, I32)]),
                 "3": _snap([("OTHER", 0, I32)])}
        m = _harmonize(snaps, [1, 2, 3])
        row = _row(m, "X")
        self.assertTrue(row["partial"])
        self.assertFalse(row["defaultSelected"])  # partial -> not auto-selected
        self.assertEqual(sorted(row["presentSysIds"]), [1, 2])
        self.assertFalse(row["perVehicle"]["3"]["present"])
        # winner 5 -> only u2 is an outlier; u3 (absent) never written.
        self.assertEqual(_collect(snaps, [1, 2, 3], ["X"], winners={"X": 5}),
                         {"2": [{"name": "X", "value": 5}]})

    def test_partial_with_present_majority_still_not_default_selected(self):
        # X present+agreeing on u1,u2, differing on u3, absent on u4 -> modal 5
        # exists but partial -> not default-selected (fleet can't be fully synced).
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, I32)]), "4": _snap([("OTHER", 0, I32)])}
        row = _row(_harmonize(snaps, [1, 2, 3, 4]), "X")
        self.assertEqual(row["modalValue"], 5)
        self.assertTrue(row["partial"])
        self.assertFalse(row["defaultSelected"])


class TestReadOnlyViewOnly(unittest.TestCase):
    """Read-only (firmware-maintained) params — STAT_* counters, *_DEVID,
    MIS_TOTAL, *_GND_PRESS — can never be written. A divergent read-only param is
    kept as a VIEW-ONLY row (readOnly:true, defaultWinner:null) so the operator can
    still inspect it in the dedicated "Read-only" sidebar bucket, but it's inert in
    every write/selection path (operator report: "let also user to see the
    readonly"). Surfaced as a `readOnly` count."""

    def test_divergent_readonly_is_view_only_row(self):
        snaps = {"1": _snap([("STAT_RUNTIME", 100, I32, True)]),
                 "2": _snap([("STAT_RUNTIME", 200, I32, True)]),
                 "3": _snap([("STAT_RUNTIME", 300, I32, True)])}
        m = _harmonize(snaps, [1, 2, 3])
        ro = _row(m, "STAT_RUNTIME")
        self.assertIsNotNone(ro)
        self.assertTrue(ro["readOnly"])
        self.assertEqual(ro["status"], "read-only")
        self.assertIsNone(ro["defaultWinner"])
        self.assertFalse(ro["defaultSelected"])
        self.assertEqual(m["counts"]["readOnly"], 1)
        self.assertEqual(m["counts"]["divergent"], 0)  # not a harmonizable row
        self.assertEqual(m["counts"]["noMajority"], 0)
        # Inert: even force-selected with a winner, it collects nothing.
        self.assertEqual(_collect(snaps, [1, 2, 3], ["STAT_RUNTIME"],
                                  winners={"STAT_RUNTIME": 100}), {})

    def test_agreeing_readonly_not_shown_or_counted(self):
        # A read-only param that agrees across the fleet is dropped by the
        # agreement test first (not divergent), so it isn't a row and isn't counted
        # — the count reflects only would-be sync rows.
        snaps = {"1": _snap([("STAT_BOOTCNT", 7, I32, True)]),
                 "2": _snap([("STAT_BOOTCNT", 7, I32, True)])}
        m = _harmonize(snaps, [1, 2])
        self.assertIsNone(_row(m, "STAT_BOOTCNT"))
        self.assertEqual(m["counts"]["readOnly"], 0)

    def test_single_readonly_cell_marks_row(self):
        snaps = {"1": _snap([("STAT_FLTTIME", 10, I32, True)]),
                 "2": _snap([("STAT_FLTTIME", 20, I32, False)])}
        ro = _row(_harmonize(snaps, [1, 2]), "STAT_FLTTIME")
        self.assertIsNotNone(ro)
        self.assertTrue(ro["readOnly"])
        self.assertEqual(_harmonize(snaps, [1, 2])["counts"]["readOnly"], 1)

    def test_non_readonly_divergent_not_flagged(self):
        snaps = {"1": _snap([("X", 5, I32, False)]), "2": _snap([("X", 9, I32, False)])}
        m = _harmonize(snaps, [1, 2])
        self.assertFalse(_row(m, "X").get("readOnly", False))
        self.assertEqual(m["counts"]["readOnly"], 0)


class TestSkipList(unittest.TestCase):
    def test_identity_row_shown_but_not_default_selected(self):
        snaps = {"1": _snap([("COMPASS_OFS_X", 5, I32)]),
                 "2": _snap([("COMPASS_OFS_X", 5, I32)]),
                 "3": _snap([("COMPASS_OFS_X", 9, I32)])}
        m = _harmonize(snaps, [1, 2, 3], skip=True)
        row = _row(m, "COMPASS_OFS_X")
        self.assertEqual(row["safetyTag"], "identity-cal-skip")
        self.assertTrue(row["isUavSpecific"])
        self.assertEqual(row["modalValue"], 5)   # majority still computed
        self.assertFalse(row["defaultSelected"])      # but not selected by default
        self.assertEqual(m["counts"]["skippedDivergent"], 1)

    def test_identity_default_selected_when_skip_off(self):
        snaps = {"1": _snap([("COMPASS_OFS_X", 5, I32)]),
                 "2": _snap([("COMPASS_OFS_X", 5, I32)]),
                 "3": _snap([("COMPASS_OFS_X", 9, I32)])}
        row = _row(_harmonize(snaps, [1, 2, 3], skip=False), "COMPASS_OFS_X")
        self.assertTrue(row["defaultSelected"])


class TestCoercionAndParity(unittest.TestCase):
    def test_type_mismatch_flagged_not_default_selected(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, F)])}
        row = _row(_harmonize(snaps, [1, 2, 3]), "X")
        self.assertTrue(row["typeMismatch"])
        self.assertIsNone(row["defaultWinner"])
        self.assertFalse(row["defaultSelected"])

    def test_winner_coerced_per_target_no_op_skipped(self):
        # int outlier where winner coerces to the current value -> not written.
        # u1=2, u2=2 (majority 2), u3=2.9 stored as 2 already? u3 int value 2.9 isn't
        # a valid stored int; use u3=3 so it's a real outlier, and u4 already == winner.
        snaps = {"1": _snap([("X", 2, I32)]), "2": _snap([("X", 2, I32)]),
                 "3": _snap([("X", 3, I32)]), "4": _snap([("X", 2, I32)])}
        # winner = 2 (majority of 3). Only u3 differs.
        self.assertEqual(_collect(snaps, [1, 2, 3, 4], ["X"]),
                         {"3": [{"name": "X", "value": 2}]})

    def test_collect_matches_resolve_submissions(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, I32)])}
        script = (
            "const snaps={\"1\":normaliseSnapshot({params:%s}),\"2\":normaliseSnapshot({params:%s}),\"3\":normaliseSnapshot({params:%s})};"
            "const m=computeFleetHarmonize({snapshotsByVehicle:snaps, sysIds:[1,2,3]});"
            "const changes=collectHarmonizeChanges(m.rows,['X'],{});"
            "const resolved=resolveSubmissions([1,2,3],{changesByVehicle:changes, snapshotsByVehicle:snaps});"
            "console.log(JSON.stringify({changes, resolved}));"
            % (json.dumps(list(snaps["1"]["paramsByName"].values())),
               json.dumps(list(snaps["2"]["paramsByName"].values())),
               json.dumps(list(snaps["3"]["paramsByName"].values())))
        )
        out = json.loads(_run_js(script))
        # The scoped writer re-validates against the live snapshot and must agree.
        # resolveSubmissions lists every id (empty [] for non-outliers); collect
        # omits empties. Same changes either way.
        resolved_nonempty = {k: v for k, v in out["resolved"].items() if v}
        self.assertEqual(out["changes"], resolved_nonempty)
        self.assertEqual(out["changes"], {"3": [{"name": "X", "value": 5}]})

    def test_type_mismatch_manual_winner_coerces_per_target(self):
        # u1,u2 int32=5, u3 float=9.5 (type mismatch, no default winner). Operator
        # picks 9.5 -> int targets store 9, the float holder is not written.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9.5, F)])}
        self.assertEqual(
            _collect(snaps, [1, 2, 3], ["X"], winners={"X": 9.5}),
            {"1": [{"name": "X", "value": 9}], "2": [{"name": "X", "value": 9}]},
        )

    def test_harmonize_cell_states(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, I32)])}
        states = json.loads(_run_js(
            "const m=computeFleetHarmonize({snapshotsByVehicle:%s, sysIds:[1,2,3]});"
            "const r=m.rows.find(x=>x.name==='X');"
            "console.log(JSON.stringify(harmonizeCellStates(r, r.defaultWinner)));"
            % json.dumps(snaps)
        ))
        self.assertTrue(states["1"]["isWinner"])
        self.assertTrue(states["2"]["isWinner"])
        self.assertFalse(states["1"]["isOutlier"])
        self.assertTrue(states["3"]["isOutlier"])
        self.assertTrue(states["3"]["wouldWrite"])


class TestSelectionHelpers(unittest.TestCase):
    def test_default_selection_excludes_no_majority_and_identity(self):
        snaps = {
            "1": _snap([("A", 1, I32), ("B", 1, I32), ("COMPASS_OFS_X", 1, I32)]),
            "2": _snap([("A", 1, I32), ("B", 2, I32), ("COMPASS_OFS_X", 1, I32)]),
            "3": _snap([("A", 9, I32), ("B", 3, I32), ("COMPASS_OFS_X", 9, I32)]),
        }
        # A: 2-of-3 majority (1) -> selected. B: all distinct -> no majority.
        # COMPASS_OFS_X: majority 1 but identity -> not selected.
        got = json.loads(_run_js(
            "const m=computeFleetHarmonize({snapshotsByVehicle:%s, sysIds:[1,2,3]});"
            "console.log(JSON.stringify(defaultHarmonizeSelection(m.rows)));" % json.dumps(snaps)
        ))
        self.assertEqual(got, ["A"])

    def test_reconcile_keeps_still_divergent_with_winner(self):
        # A is a 2-of-3 majority (has a default winner) -> kept; GONE not in model.
        snaps = {"1": _snap([("A", 1, I32)]), "2": _snap([("A", 1, I32)]),
                 "3": _snap([("A", 9, I32)])}
        got = json.loads(_run_js(
            "const m=computeFleetHarmonize({snapshotsByVehicle:%s, sysIds:[1,2,3]});"
            "console.log(JSON.stringify(reconcileHarmonizeSelection(['A','GONE'], m.rows, {})));"
            % json.dumps(snaps)
        ))
        self.assertEqual(got, ["A"])


class TestDroppedHarmonizeWinnerNames(unittest.TestCase):
    """Codex gate-15 follow-up (PR #90, deferred non-blocking): the plain-recompute
    reconcile path (post-write/refresh in ParametersTab.jsx) can drop a row from the
    selection too, not just the two operator-driven deselect paths (toggle a row
    off, "select all" off) already fixed. droppedHarmonizeWinnerNames reports which
    names to forget from the manual-pick winner map so that path can't leave a stale
    pick behind either."""

    def test_names_dropped_from_selection_are_reported(self):
        got = json.loads(_run_js(
            "console.log(JSON.stringify("
            "droppedHarmonizeWinnerNames(['A','B','C'], ['B'])));"
        ))
        self.assertEqual(sorted(got), ["A", "C"])

    def test_nothing_dropped_when_selection_unchanged(self):
        got = json.loads(_run_js(
            "console.log(JSON.stringify("
            "droppedHarmonizeWinnerNames(['A','B'], ['A','B'])));"
        ))
        self.assertEqual(got, [])

    def test_accepts_a_set_for_prior_selection(self):
        got = json.loads(_run_js(
            "console.log(JSON.stringify("
            "droppedHarmonizeWinnerNames(new Set(['A','B']), ['A'])));"
        ))
        self.assertEqual(got, ["B"])

    def test_end_to_end_row_dropped_by_reconcile_reports_as_pruned(self):
        # u1 and u2 agree on X (5/5) -> not divergent -> X's row is absent from
        # the model entirely, so reconcile drops it from the selection regardless
        # of the (now-orphaned) manual pick in the winners map. This is the
        # "no longer divergent" case reconcileHarmonizeSelection handles;
        # droppedHarmonizeWinnerNames must flag X so the caller can prune it.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)])}
        got = json.loads(_run_js(
            "const m=computeFleetHarmonize({snapshotsByVehicle:%s, sysIds:[1,2]});"
            "const reconciled = reconcileHarmonizeSelection(['X'], m.rows, {X:9});"
            "console.log(JSON.stringify(droppedHarmonizeWinnerNames(['X'], reconciled)));"
            % json.dumps(snaps)
        ))
        self.assertEqual(got, ["X"])


class TestStaleWinner(unittest.TestCase):
    def test_stale_winner_falls_back_to_modal(self):
        # 2-of-3 majority (5). A stale pick 99 (not on any UAV) self-heals to 5.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, I32)])}
        self.assertEqual(_collect(snaps, [1, 2, 3], ["X"], winners={"X": 99}),
                         {"3": [{"name": "X", "value": 5}]})

    def test_stale_winner_no_default_collects_nothing(self):
        # 2-UAV tie (no default) + stale pick 99 -> no effective winner -> nothing.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 9, I32)])}
        self.assertEqual(_collect(snaps, [1, 2], ["X"], winners={"X": 99}), {})

    def test_reconcile_drops_row_with_null_effective_winner(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 9, I32)])}
        got = json.loads(_run_js(
            "const m=computeFleetHarmonize({snapshotsByVehicle:%s, sysIds:[1,2]});"
            "console.log(JSON.stringify(reconcileHarmonizeSelection(['X'], m.rows, {X:99})));"
            % json.dumps(snaps)
        ))
        self.assertEqual(got, [])

    def test_reconcile_keeps_row_with_valid_manual_winner(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 9, I32)])}
        got = json.loads(_run_js(
            "const m=computeFleetHarmonize({snapshotsByVehicle:%s, sysIds:[1,2]});"
            "console.log(JSON.stringify(reconcileHarmonizeSelection(['X'], m.rows, {X:9})));"
            % json.dumps(snaps)
        ))
        self.assertEqual(got, ["X"])

    def test_string_sysids(self):
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 9, I32)])}
        self.assertEqual(_collect(snaps, ["1", "2"], ["X"], winners={"X": 5}),
                         {"2": [{"name": "X", "value": 5}]})


class TestEffectiveBaseSysId(unittest.TestCase):
    """The 'base' cell badged in the grid: the source of the winning value. Honors
    the operator's clicked UAV; falls back to a stable first-holder for the default
    winner; self-heals a stale pick; null when the row has no winner."""

    def test_default_winner_base_is_first_holder(self):
        # u1=5, u2=5, u3=9 -> modal winner 5; base is the first present holder (u1).
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, I32)])}
        self.assertEqual(_base(snaps, [1, 2, 3], "X"), 1)

    def test_manual_pick_badges_the_clicked_uav(self):
        # u1=5, u2=5, u3=9. Operator clicks u2 (value 5). Even though u1 also holds
        # 5, the base badge follows the CLICKED cell, not the first holder.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, I32)])}
        self.assertEqual(_base(snaps, [1, 2, 3], "X", winners={"X": {"value": 5, "sysId": 2}}), 2)

    def test_manual_pick_of_minority_value(self):
        # Operator overrides the modal winner: clicks u3 (value 9). Base = u3.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, I32)])}
        self.assertEqual(_base(snaps, [1, 2, 3], "X", winners={"X": {"value": 9, "sysId": 3}}), 3)

    def test_stale_pick_self_heals_to_first_holder(self):
        # A pick whose value no UAV holds any more -> winner falls back to the modal
        # (5) and the base falls back to the first holder (u1), never the stale sysId.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 5, I32)]),
                 "3": _snap([("X", 9, I32)])}
        self.assertEqual(_base(snaps, [1, 2, 3], "X", winners={"X": {"value": 999, "sysId": 3}}), 1)

    def test_no_winner_has_no_base(self):
        # 1-1 tie, no pick -> no winner -> no base cell.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 9, I32)])}
        self.assertIsNone(_base(snaps, [1, 2], "X"))

    def test_legacy_bare_value_pick_still_resolves_base(self):
        # A pick stored as a bare value (no sysId) still yields a base: the first
        # present holder of that value. Keeps the model tolerant.
        snaps = {"1": _snap([("X", 5, I32)]), "2": _snap([("X", 9, I32)])}
        self.assertEqual(_base(snaps, [1, 2], "X", winners={"X": 9}), 2)


if __name__ == "__main__":
    unittest.main()
