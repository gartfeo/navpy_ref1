"""Pure-helper tests for src/gcs/frontend/src/utils/fullParams.js.

Drives the JS via Node so the same code that ships to the browser is
exercised. Locks the data-shape contract that the read-only Parameters
tab (Step 5), the editor (Step 6), and the .param load/save (Step 7)
all depend on.
"""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "fullParams.js",
))
_USE_FULL_PARAMS_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks",
    "useFullParams.js",
))
_USE_WS_HANDLERS_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks",
    "useWsHandlers.js",
))
_APP_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src",
    "App.jsx",
))
_PARAMETERS_TAB_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "settings", "ParametersTab.jsx",
))
_VIRTUAL_PARAM_GRID_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "settings", "VirtualParamGrid.jsx",
))
_CIRCULAR_PROGRESS_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "settings", "CircularProgress.jsx",
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


# Helpers to build typical inputs.
def _record(name, value, ap_type=4, default=None, default_known=False, flags=0):
    return {
        "name": name, "value": value, "ap_type": ap_type,
        "default": default, "default_known": default_known, "flags": flags,
    }


def _snapshot(records, num=None, total=None, fetched=None, stale=False):
    return {
        "params": records,
        "num_params": num if num is not None else len(records),
        "total_params": total if total is not None else len(records),
        "fetched_at_unix_s": fetched if fetched is not None else 1.0,
        "stale": stale,
    }


# ---------------------------------------------------------------------
class TestNormaliseSnapshot(unittest.TestCase):
    def test_preserves_order(self):
        snap = _snapshot([_record("ZZ", 1), _record("AA", 2), _record("MM", 3)])
        result = _call_json(f"normaliseSnapshot({json.dumps(snap)})")
        self.assertEqual(result["nameOrder"], ["ZZ", "AA", "MM"])
        self.assertEqual(result["paramsByName"]["AA"]["value"], 2)
        self.assertEqual(result["numParams"], 3)
        self.assertEqual(result["totalParams"], 3)
        self.assertFalse(result["stale"])

    def test_empty(self):
        result = _call_json(f"normaliseSnapshot({json.dumps(_snapshot([]))})")
        self.assertEqual(result["nameOrder"], [])
        self.assertEqual(result["paramsByName"], {})

    def test_drops_records_without_name(self):
        snap = _snapshot([_record("OK", 1), {"value": 2}])
        result = _call_json(f"normaliseSnapshot({json.dumps(snap)})")
        self.assertEqual(result["nameOrder"], ["OK"])


class TestFullParamLoadProgress(unittest.TestCase):
    def test_non_downloading_status_returns_null(self):
        result = _call_json("fullParamLoadProgress({kind: 'idle'})")
        self.assertIsNone(result)

    def test_starting_download_has_no_bytes_yet(self):
        status = {"kind": "downloading", "counts": {"uavTotal": 1}}
        result = _call_json(f"fullParamLoadProgress({json.dumps(status)})")
        self.assertEqual(result["done"], 0)
        self.assertEqual(result["total"], 1)
        self.assertEqual(result["bytesRead"], 0)
        self.assertEqual(result["percent"], 0)
        self.assertFalse(result["hasBytes"])

    def test_byte_progress_averages_vehicle_downloads(self):
        status = {
            "kind": "downloading",
            "counts": {
                "uavDone": 2,
                "uavOk": 1,
                "uavTotal": 4,
                "byVehicle": {
                    "2": {"bytesRead": 1024, "sizeEstimate": 2048},
                    "1": {"bytesRead": 512, "sizeEstimate": 1024},
                },
            },
        }
        result = _call_json(f"fullParamLoadProgress({json.dumps(status)})")
        self.assertEqual(result["done"], 2)
        self.assertEqual(result["ok"], 1)
        self.assertEqual(result["total"], 4)
        self.assertEqual(result["bytesRead"], 1536)
        self.assertEqual(result["sizeEstimate"], 3072)
        self.assertEqual(result["percent"], 25)
        self.assertEqual(result["vehicleProgress"][0]["percent"], 50)
        self.assertEqual([p["sysId"] for p in result["vehicleProgress"]], [1, 2])

    def test_per_vehicle_has_bytes_flag(self):
        # A UAV with no bytes and no known size is still in MAVFTP setup — the
        # header must show an indeterminate spinner (hasBytes=False), not a ring
        # frozen at 0%. A UAV with bytes/size flowing shows the determinate ring.
        status = {
            "kind": "downloading",
            "counts": {
                "uavTotal": 2,
                "byVehicle": {
                    "1": {"bytesRead": 0, "sizeEstimate": 0, "totalBytes": 0},
                    "2": {"bytesRead": 10, "sizeEstimate": 100},
                },
            },
        }
        result = _call_json(f"fullParamLoadProgress({json.dumps(status)})")
        by_sys = {p["sysId"]: p for p in result["vehicleProgress"]}
        self.assertFalse(by_sys[1]["hasBytes"])
        self.assertTrue(by_sys[2]["hasBytes"])

    def test_finished_vehicle_uses_exact_total_in_multi_uav_progress(self):
        status = {
            "kind": "downloading",
            "counts": {
                "uavDone": 1,
                "uavOk": 1,
                "uavTotal": 3,
                "byVehicle": {
                    "1": {
                        "bytesRead": 16775,
                        "sizeEstimate": 17796,
                        "totalBytes": 16775,
                        "done": True,
                    },
                    "2": {"bytesRead": 8898, "sizeEstimate": 17796},
                    "3": {"bytesRead": 0, "sizeEstimate": 17796},
                },
            },
        }
        result = _call_json(f"fullParamLoadProgress({json.dumps(status)})")
        self.assertEqual([p["percent"] for p in result["vehicleProgress"]], [100, 50, 0])
        self.assertEqual(result["percent"], 50)

    def test_counts_are_clamped_to_total(self):
        status = {
            "kind": "downloading",
            "counts": {"uavDone": 9, "uavOk": 8, "uavTotal": 3},
        }
        result = _call_json(f"fullParamLoadProgress({json.dumps(status)})")
        self.assertEqual(result["done"], 3)
        self.assertEqual(result["ok"], 3)

    def test_done_uses_exact_total_bytes(self):
        status = {
            "kind": "downloading",
            "counts": {
                "uavDone": 1,
                "uavOk": 1,
                "uavTotal": 1,
                "byVehicle": {
                    "3": {
                        "bytesRead": 16775,
                        "sizeEstimate": 17796,
                        "totalBytes": 16775,
                        "done": True,
                    },
                },
            },
        }
        result = _call_json(f"fullParamLoadProgress({json.dumps(status)})")
        self.assertTrue(result["knownTotal"])
        self.assertEqual(result["totalBytes"], 16775)
        self.assertEqual(result["percent"], 100)

    def test_partial_refresh_retains_per_vehicle_success_and_error(self):
        status = {
            "kind": "partial",
            "op": "refresh",
            "counts": {
                "uavDone": 2,
                "uavOk": 1,
                "uavTotal": 2,
                "byVehicle": {
                    "161": {"done": True, "bytesRead": 120, "totalBytes": 120},
                    "162": {"done": True, "error": "HTTP 502: MAVFTP failed"},
                },
            },
        }
        result = _call_json(f"fullParamLoadProgress({json.dumps(status)})")
        by_sys = {p["sysId"]: p for p in result["vehicleProgress"]}
        self.assertEqual(result["ok"], 1)
        self.assertEqual(by_sys[161]["percent"], 100)
        self.assertEqual(by_sys[162]["error"], "HTTP 502: MAVFTP failed")


class TestFullParamFleetFetch(unittest.TestCase):
    def test_refresh_generation_rejects_disconnect_and_replacement_request(self):
        result = _call_json("""
          (() => {
            const generations = {161: 4};
            const live = new Set([161]);
            return {
              current: ownsFullParamRefresh(generations, live, 161, 4),
              older: ownsFullParamRefresh(generations, live, 161, 3),
              disconnected: ownsFullParamRefresh(generations, new Set(), 161, 4),
            };
          })()
        """)
        self.assertEqual(result, {"current": True, "older": False, "disconnected": False})

    def test_hook_and_grid_use_durable_incremental_refresh_state(self):
        hook = open(_USE_FULL_PARAMS_PATH, encoding="utf-8").read()
        tab = open(_PARAMETERS_TAB_PATH, encoding="utf-8").read()
        refresh = hook[hook.index("const refreshVehicles"):hook.index("const handleDownloadProgress")]
        self.assertIn("fetchFullParamFleet(active", hook)
        self.assertIn("refreshProgressByVehicle", hook)
        self.assertIn("setSnapshotsByVehicle((prev)", hook)
        self.assertEqual(refresh.count("clearVehicleDrafts("), 1)
        self.assertIn("if (!ownsRefresh(sid)) return", refresh)
        self.assertIn("releasePending(active.filter(ownsRefresh))", refresh)
        self.assertIn("fullParams?.refreshProgressByVehicle", tab)
        self.assertIn("fetchedRef.current.delete(sid)", tab)
        self.assertIn("fullParams?.pendingByVehicle", tab)
        self.assertIn("kind === 'refresh'", tab)

    def test_success_is_published_before_slow_peer_settles(self):
        out = _run_js("""
            (async () => {
              let releaseSlow;
              const slow = new Promise((resolve) => { releaseSlow = resolve; });
              const events = [];
              const fleet = fetchFullParamFleet(
                [161, 162],
                (sid) => sid === 161 ? Promise.resolve({params: [{name: 'A'}]}) : slow,
                (sid, data) => events.push(['success', sid, data.params[0].name]),
                (sid, result) => events.push(['settled', sid, result.ok]),
              );
              await new Promise((resolve) => setImmediate(resolve));
              events.push(['checkpoint']);
              releaseSlow({params: [{name: 'B'}]});
              const results = await fleet;
              console.log(JSON.stringify({events, results}));
            })();
        """)
        result = json.loads(out)
        checkpoint = result["events"].index(["checkpoint"])
        self.assertLess(result["events"].index(["success", 161, "A"]), checkpoint)
        self.assertFalse(any(e[:2] == ["success", 162] for e in result["events"][:checkpoint]))
        self.assertTrue(all(r["ok"] for r in result["results"]))

    def test_partial_fleet_returns_success_and_failure_details(self):
        result = json.loads(_run_js("""
            (async () => {
              const successes = [];
              const settled = [];
              const results = await fetchFullParamFleet(
                [161, 162],
                async (sid) => {
                  if (sid === 162) throw new Error('HTTP 502: MAVFTP transport failed');
                  return {params: [{name: 'OK'}]};
                },
                (sid) => successes.push(sid),
                (sid, outcome) => settled.push([sid, outcome.ok, outcome.error]),
              );
              console.log(JSON.stringify({successes, settled, results}));
            })();
        """))
        self.assertEqual(result["successes"], [161])
        self.assertEqual(result["settled"][1], [162, False, "HTTP 502: MAVFTP transport failed"])
        self.assertEqual(result["results"][1]["error"], "HTTP 502: MAVFTP transport failed")


class TestCoerceValueForRecord(unittest.TestCase):
    def test_int_string_to_int(self):
        rec = _record("X", 0, ap_type=3)  # INT32
        result = _call_json(f"coerceValueForRecord({json.dumps(rec)}, '42')")
        self.assertEqual(result, 42)

    def test_float_string_to_float(self):
        rec = _record("X", 0, ap_type=4)
        result = _call_json(f"coerceValueForRecord({json.dumps(rec)}, '1.5')")
        self.assertEqual(result, 1.5)

    def test_int_truncates_fractional_toward_zero(self):
        # Honest-permissive: stage the value the autopilot would store
        # (AP_Param::set_float truncates toward zero), not a rejection.
        rec = _record("X", 0, ap_type=3)
        self.assertEqual(_call_json(f"coerceValueForRecord({json.dumps(rec)}, '2.1')"), 2)
        self.assertEqual(_call_json(f"coerceValueForRecord({json.dumps(rec)}, '2.9')"), 2)
        self.assertEqual(_call_json(f"coerceValueForRecord({json.dumps(rec)}, '-3.9')"), -3)

    def test_int_clamps_to_storage_range(self):
        rec8 = _record("X", 0, ap_type=1)  # INT8: -128..127
        self.assertEqual(_call_json(f"coerceValueForRecord({json.dumps(rec8)}, '200')"), 127)
        self.assertEqual(_call_json(f"coerceValueForRecord({json.dumps(rec8)}, '-200')"), -128)
        rec16 = _record("X", 0, ap_type=2)  # INT16: -32768..32767
        self.assertEqual(_call_json(f"coerceValueForRecord({json.dumps(rec16)}, '70000')"), 32767)

    def test_empty_string_rejected(self):
        rec = _record("X", 0, ap_type=4)
        result = _call_json(f"coerceValueForRecord({json.dumps(rec)}, '')")
        self.assertIsNone(result)

    def test_non_numeric_rejected(self):
        rec = _record("X", 0, ap_type=4)
        result = _call_json(f"coerceValueForRecord({json.dumps(rec)}, 'abc')")
        self.assertIsNone(result)

    def test_int_non_finite_rejected(self):
        rec = _record("X", 0, ap_type=1)
        self.assertIsNone(_call_json(f"coerceValueForRecord({json.dumps(rec)}, 'abc')"))
        self.assertIsNone(_call_json(f"coerceValueForRecord({json.dumps(rec)}, 'Infinity')"))


class TestDescribeParamType(unittest.TestCase):
    def test_int_types_report_label_and_storage_range(self):
        self.assertEqual(
            _call_json("describeParamType(1)"),
            {"label": "int8", "isInt": True, "min": -128, "max": 127},
        )
        self.assertEqual(
            _call_json("describeParamType(2)"),
            {"label": "int16", "isInt": True, "min": -32768, "max": 32767},
        )
        self.assertEqual(_call_json("describeParamType(3)")["label"], "int32")

    def test_float_has_no_range(self):
        d = _call_json("describeParamType(4)")
        self.assertEqual(d["label"], "float")
        self.assertFalse(d["isInt"])
        self.assertIsNone(d["min"])


class TestSelectChangedNames(unittest.TestCase):
    def _changed(self, draft, snap):
        return _call_json(
            f"selectChangedNames({json.dumps(draft)}, normaliseSnapshot({json.dumps(snap)}))"
        )

    def test_empty_draft_no_changes(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        self.assertEqual(self._changed({}, snap), [])

    def test_unchanged_draft_filtered(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        self.assertEqual(self._changed({"X": 1}, snap), [])

    def test_changed_draft_returned(self):
        snap = _snapshot([_record("X", 1, ap_type=3), _record("Y", 2.5, ap_type=4)])
        result = self._changed({"X": "5", "Y": "2.5"}, snap)
        self.assertEqual(result, [{"name": "X", "value": 5}])

    def test_unknown_name_dropped(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        self.assertEqual(self._changed({"NOPE": 9}, snap), [])

    def test_fractional_int_that_truncates_to_current_is_no_change(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        # "1.5" truncates to 1 == stored 1 → no change.
        self.assertEqual(self._changed({"X": "1.5"}, snap), [])

    def test_fractional_int_submits_the_stored_integer(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        # "5.7" truncates to 5 != stored 1 → submitted as the coerced integer.
        self.assertEqual(self._changed({"X": "5.7"}, snap), [{"name": "X", "value": 5}])

    def test_non_numeric_draft_dropped(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        self.assertEqual(self._changed({"X": "abc"}, snap), [])

    def test_clamped_int_submits_clamped_value(self):
        snap = _snapshot([_record("X", 5, ap_type=1)])  # INT8, stored 5
        self.assertEqual(self._changed({"X": "200"}, snap), [{"name": "X", "value": 127}])

    def test_clamped_int_at_bound_is_no_change(self):
        snap = _snapshot([_record("X", 127, ap_type=1)])  # already at INT8 max
        self.assertEqual(self._changed({"X": "200"}, snap), [])


class TestResolveSubmissions(unittest.TestCase):
    """Step 3 — scoped write path. Both the draft path and the explicit
    changesByVehicle path run through the same snapshot gate (selectChangedNames),
    so a scoped compare-apply can't push unrelated drafts and never submits
    unknown/no-op/invalid changes."""

    def _resolve(self, ids, *, changes=None, drafts=None, snaps=None):
        snap_expr = "{"
        for sid, snap in (snaps or {}).items():
            snap_expr += f"{json.dumps(sid)}: normaliseSnapshot({json.dumps(snap)}),"
        snap_expr += "}"
        opts = "{"
        if changes is not None:
            opts += f"changesByVehicle: {json.dumps(changes)},"
        if drafts is not None:
            opts += f"draftsByVehicle: {json.dumps(drafts)},"
        opts += f"snapshotsByVehicle: {snap_expr},}}"
        return _call_json(f"resolveSubmissions({json.dumps(ids)}, {opts})")

    def test_draft_path_matches_select_changed_names(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        self.assertEqual(
            self._resolve([1], drafts={"1": {"X": "5"}}, snaps={"1": snap}),
            {"1": [{"name": "X", "value": 5}]},
        )

    def test_empty_changes_object_submits_nothing_and_ignores_drafts(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        self.assertEqual(
            self._resolve([1], changes={}, drafts={"1": {"X": "5"}}, snaps={"1": snap}),
            {"1": []},
        )

    def test_explicit_string_sid_keys_resolve_for_numeric_ids(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        self.assertEqual(
            self._resolve([1], changes={"1": [{"name": "X", "value": 5}]}, snaps={"1": snap}),
            {"1": [{"name": "X", "value": 5}]},
        )

    def test_explicit_changes_exclude_unrelated_drafts(self):
        snap = _snapshot([_record("X", 1, ap_type=3), _record("Y", 0, ap_type=3)])
        out = self._resolve(
            [1], changes={"1": [{"name": "Y", "value": 9}]},
            drafts={"1": {"X": "5"}}, snaps={"1": snap},
        )
        self.assertEqual(out, {"1": [{"name": "Y", "value": 9}]})

    def test_explicit_unknown_name_dropped(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        self.assertEqual(
            self._resolve([1], changes={"1": [{"name": "NOPE", "value": 9}]}, snaps={"1": snap}),
            {"1": []},
        )

    def test_explicit_no_op_dropped(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        self.assertEqual(
            self._resolve([1], changes={"1": [{"name": "X", "value": 1}]}, snaps={"1": snap}),
            {"1": []},
        )

    def test_explicit_value_re_coerced(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        self.assertEqual(
            self._resolve([1], changes={"1": [{"name": "X", "value": 2.9}]}, snaps={"1": snap}),
            {"1": [{"name": "X", "value": 2}]},
        )

    def test_explicit_zero_preserved(self):
        snap = _snapshot([_record("X", 5, ap_type=3)])
        self.assertEqual(
            self._resolve([1], changes={"1": [{"name": "X", "value": 0}]}, snaps={"1": snap}),
            {"1": [{"name": "X", "value": 0}]},
        )

    def test_missing_explicit_sid_returns_empty(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        out = self._resolve(
            [1, 2], changes={"1": [{"name": "X", "value": 9}]},
            snaps={"1": snap, "2": snap},
        )
        self.assertEqual(out, {"1": [{"name": "X", "value": 9}], "2": []})

    def test_explicit_non_finite_dropped(self):
        snap = _snapshot([_record("X", 1.0, ap_type=4), _record("Y", 1.0, ap_type=4)])
        out = self._resolve(
            [1],
            changes={"1": [{"name": "X", "value": float("inf")},
                           {"name": "Y", "value": "abc"}]},
            snaps={"1": snap},
        )
        self.assertEqual(out, {"1": []})

    def test_null_changes_falls_back_to_drafts(self):
        snap = _snapshot([_record("X", 1, ap_type=3)])
        out = _call_json(
            "resolveSubmissions([1], {changesByVehicle: null, "
            "draftsByVehicle: {\"1\": {X: \"5\"}}, "
            f"snapshotsByVehicle: {{\"1\": normaliseSnapshot({json.dumps(snap)})}}}})"
        )
        self.assertEqual(out, {"1": [{"name": "X", "value": 5}]})


class TestScopedWriteWiring(unittest.TestCase):
    def test_hook_uses_resolve_submissions_with_changes_option(self):
        source = open(_USE_FULL_PARAMS_PATH, encoding="utf-8").read()
        self.assertIn("resolveSubmissions(active", source)
        self.assertIn("changesByVehicle", source)
        # The old inline draft-diff loop is gone and the hook no longer calls
        # selectChangedNames directly (it goes through resolveSubmissions).
        self.assertNotIn("submittedByVehicle[sid] = selectChangedNames", source)
        self.assertNotIn("selectChangedNames", source)


class TestDraftValueMatchesSubmitted(unittest.TestCase):
    def _matches(self, rec, draft, submitted):
        return _call_json(
            f"draftValueMatchesSubmitted("
            f"{json.dumps(rec)}, {json.dumps(draft)}, {json.dumps(submitted)})"
        )

    def test_int_string_matches_typed_submitted_value(self):
        rec = _record("X", 0, ap_type=3)
        self.assertTrue(self._matches(rec, "42", 42))

    def test_integer_decimal_string_matches_integer_value(self):
        rec = _record("X", 0, ap_type=3)
        self.assertTrue(self._matches(rec, "2.0", 2))

    def test_float_tolerance_matches_submitted_value(self):
        rec = _record("X", 0, ap_type=4)
        self.assertTrue(self._matches(rec, "1.00000001", 1.0))

    def test_fractional_int_matches_when_it_truncates_to_submitted(self):
        # "2.9" stores as 2 == submitted 2 → match (honest-permissive cleanup).
        rec = _record("X", 0, ap_type=3)
        self.assertTrue(self._matches(rec, "2.9", 2))

    def test_fractional_int_differs_when_it_truncates_elsewhere(self):
        rec = _record("X", 0, ap_type=3)
        self.assertFalse(self._matches(rec, "3.1", 2))

    def test_clamped_int_matches_submitted_bound(self):
        # int8 "200" stores as 127 == submitted 127 → match.
        rec = _record("X", 0, ap_type=1)
        self.assertTrue(self._matches(rec, "200", 127))

    def test_invalid_draft_does_not_match(self):
        rec = _record("X", 0, ap_type=4)
        self.assertFalse(self._matches(rec, "abc", 0))
        self.assertFalse(self._matches(rec, "", 0))


class TestSelectNonDefaultNames(unittest.TestCase):
    def _non_default(self, snapshots, sids, names, drafts=None):
        norm_expr = "{"
        for sid, snap in snapshots.items():
            norm_expr += f"{json.dumps(sid)}: normaliseSnapshot({json.dumps(snap)}),"
        norm_expr += "}"
        return _call_json(
            f"selectNonDefaultNames({norm_expr}, {json.dumps(sids)}, "
            f"{json.dumps(names)}, {json.dumps(drafts or {})})"
        )

    def test_selects_names_with_any_known_non_default_value(self):
        s1 = _snapshot([
            _record("A", 1, ap_type=3, default=0, default_known=True),
            _record("B", 2, ap_type=3, default=2, default_known=True),
            _record("C", 9, ap_type=3, default=0, default_known=False),
        ])
        s2 = _snapshot([
            _record("A", 0, ap_type=3, default=0, default_known=True),
            _record("B", 2, ap_type=3, default=2, default_known=True),
        ])
        self.assertEqual(
            self._non_default({"1": s1, "2": s2}, ["1", "2"], ["A", "B", "C"]),
            ["A"],
        )

    def test_draft_value_is_used_for_non_default_filter(self):
        snap = _snapshot([
            _record("A", 1, ap_type=3, default=0, default_known=True),
            _record("B", 1, ap_type=3, default=0, default_known=True),
        ])
        drafts = {"1": {"A": "0", "B": "3"}}
        self.assertEqual(
            self._non_default({"1": snap}, ["1"], ["A", "B"], drafts),
            ["B"],
        )

    def test_fractional_draft_that_coerces_to_default_is_not_non_default(self):
        # Stored 5 differs from default 2, but the draft "2.9" stores as 2 ==
        # default → no longer counts as non-default.
        snap = _snapshot([_record("A", 5, ap_type=1, default=2, default_known=True)])
        self.assertEqual(self._non_default({"1": snap}, ["1"], ["A"]), ["A"])
        self.assertEqual(
            self._non_default({"1": snap}, ["1"], ["A"], {"1": {"A": "2.9"}}),
            [],
        )


class TestAggregateBatchResults(unittest.TestCase):
    def test_full_success(self):
        submitted = {1: [{"name": "A", "value": 1}], 2: [{"name": "B", "value": 2}]}
        results = {
            "1": {"A": {"ok": True}},
            "2": {"B": {"ok": True}},
        }
        agg = _call_json(
            f"aggregateBatchResults({json.dumps(results)}, {json.dumps(submitted)})"
        )
        self.assertEqual(agg["uavOk"], 2)
        self.assertEqual(agg["uavFail"], 0)
        self.assertEqual(agg["cellOk"], 2)
        self.assertEqual(agg["cellFail"], 0)

    def test_network_failure_with_submitted_changes(self):
        submitted = {1: [{"name": "A", "value": 1}, {"name": "B", "value": 2}]}
        results = {"1": None}
        agg = _call_json(
            f"aggregateBatchResults({json.dumps(results)}, {json.dumps(submitted)})"
        )
        self.assertEqual(agg["uavFail"], 1)
        self.assertEqual(agg["cellFail"], 2)
        self.assertEqual(len(agg["errors"]), 2)

    def test_partial_per_cell_fail(self):
        submitted = {1: [{"name": "A", "value": 1}, {"name": "B", "value": 2}]}
        results = {"1": {"A": {"ok": True}, "B": {"ok": False, "error": "echo timeout"}}}
        agg = _call_json(
            f"aggregateBatchResults({json.dumps(results)}, {json.dumps(submitted)})"
        )
        self.assertEqual(agg["uavOk"], 0)
        self.assertEqual(agg["uavFail"], 1)
        self.assertEqual(agg["cellOk"], 1)
        self.assertEqual(agg["cellFail"], 1)
        self.assertEqual(agg["errors"][0]["error"], "echo timeout")

    def test_no_submitted_skipped_from_counts(self):
        submitted = {1: [], 2: [{"name": "A", "value": 1}]}
        results = {"2": {"A": {"ok": True}}}
        agg = _call_json(
            f"aggregateBatchResults({json.dumps(results)}, {json.dumps(submitted)})"
        )
        # Vehicle 1 had nothing to submit; counts only reflect vehicle 2.
        self.assertEqual(agg["uavOk"], 1)
        self.assertEqual(agg["uavFail"], 0)


class TestConsensusByName(unittest.TestCase):
    def _consensus(self, snapshots, sids, name):
        norm_expr = "{"
        for sid, snap in snapshots.items():
            norm_expr += f"{json.dumps(sid)}: normaliseSnapshot({json.dumps(snap)}),"
        norm_expr += "}"
        return _call_json(
            f"consensusByName({norm_expr}, {json.dumps(sids)}, {json.dumps(name)})"
        )

    def test_all_loaded_same_value_ready(self):
        snap = _snapshot([_record("X", 5, ap_type=3)])
        result = self._consensus({"1": snap, "2": snap}, ["1", "2"], "X")
        self.assertEqual(result["kind"], "ready")
        self.assertEqual(result["value"], 5)

    def test_one_unloaded_loading(self):
        snap = _snapshot([_record("X", 5, ap_type=3)])
        # No snapshot for sid 2.
        result = _call_json(
            f"consensusByName({{ '1': normaliseSnapshot({json.dumps(snap)}) }}, "
            f"['1', '2'], 'X')"
        )
        self.assertEqual(result["kind"], "loading")

    def test_all_loaded_no_match_unknown(self):
        snap = _snapshot([_record("X", 5, ap_type=3)])
        result = self._consensus({"1": snap, "2": snap}, ["1", "2"], "Y")
        self.assertEqual(result["kind"], "unknown")

    def test_loaded_disagree_mixed(self):
        s1 = _snapshot([_record("X", 5, ap_type=3)])
        s2 = _snapshot([_record("X", 9, ap_type=3)])
        result = self._consensus({"1": s1, "2": s2}, ["1", "2"], "X")
        self.assertEqual(result["kind"], "mixed")

    def test_partial_missing_is_mixed(self):
        # One vehicle has the param, the other doesn't — surface as mixed
        # so the operator sees that vehicles disagree.
        s1 = _snapshot([_record("X", 5, ap_type=3)])
        s2 = _snapshot([_record("Y", 1, ap_type=3)])  # X missing here
        result = self._consensus({"1": s1, "2": s2}, ["1", "2"], "X")
        self.assertEqual(result["kind"], "mixed")


class TestPruneToSet(unittest.TestCase):
    def test_drops_disconnected(self):
        result = _call_json(
            "pruneToSet({1: 'a', 2: 'b', 3: 'c'}, [1, 3])"
        )
        self.assertEqual(result, {"1": "a", "3": "c"})

    def test_returns_same_when_no_change(self):
        # We can't easily test reference identity through JSON, but result
        # must match the input shape unchanged.
        result = _call_json(
            "pruneToSet({1: 'a', 2: 'b'}, [1, 2])"
        )
        self.assertEqual(result, {"1": "a", "2": "b"})


class TestArmedTokenLive(unittest.TestCase):
    def test_live_token(self):
        result = _call_json(
            "isArmedTokenLive({nonce: 'abc', expires_at_unix_s: 100}, 50)"
        )
        self.assertTrue(result)

    def test_expired_token(self):
        result = _call_json(
            "isArmedTokenLive({nonce: 'abc', expires_at_unix_s: 50}, 100)"
        )
        self.assertFalse(result)

    def test_missing_nonce(self):
        result = _call_json(
            "isArmedTokenLive({expires_at_unix_s: 100}, 50)"
        )
        self.assertFalse(result)


class TestFullParamProgressWiring(unittest.TestCase):
    def test_hook_exposes_download_progress_handler(self):
        source = open(_USE_FULL_PARAMS_PATH, encoding="utf-8").read()
        self.assertIn("handleDownloadProgress", source)
        self.assertIn("byVehicle", source)
        self.assertIn("bytes_read", source)

    def test_ws_handler_routes_full_param_progress(self):
        source = open(_USE_WS_HANDLERS_PATH, encoding="utf-8").read()
        self.assertIn("full_param_progress", source)
        self.assertIn("handleDownloadProgress", source)

    def test_app_passes_full_params_to_ws_handlers(self):
        source = open(_APP_PATH, encoding="utf-8").read()
        self.assertIn("fullParams,", source)

    def test_parameters_tab_wires_column_header_progress(self):
        source = open(_PARAMETERS_TAB_PATH, encoding="utf-8").read()
        # Per-vehicle progress + edited flags are passed down to the grid, which
        # renders a ring/check in each column header (no separate band/chip).
        self.assertIn("progressByVehicle", source)
        self.assertIn("changedByVehicle", source)
        self.assertIn("fullParamLoadProgress", source)
        self.assertIn('aria-busy', source)
        # Compare/harmonize modes make the grid read-only; normal mode gates on refresh.
        self.assertIn("editable={!paramsRefreshing && !compareMode && !harmonizeMode}", source)
        self.assertNotIn("ParamLoadStatus", source)
        self.assertNotIn("loadProgressPercent", source)
        self.assertNotIn("Downloading full parameters", source)
        # The "Parameters updated" result chip was removed.
        self.assertNotIn("refresh_status_ok", source)

    def test_grid_renders_per_vehicle_progress_ring(self):
        source = open(_VIRTUAL_PARAM_GRID_PATH, encoding="utf-8").read()
        self.assertIn("CircularProgress", source)
        self.assertIn("VehicleHeaderStatus", source)
        self.assertIn("progressByVehicle", source)
        self.assertIn("changedByVehicle", source)
        # Percent shown as a number; warning colour reused for unsaved changes.
        self.assertIn("colors.success", source)
        self.assertIn("colors.warning", source)
        # Setup phase (no bytes yet) shows an indeterminate spinner instead of a
        # ring frozen at 0%; labelled with the "contacting" string.
        self.assertIn("indeterminate", source)
        self.assertIn("contacting", source)
        # Read-only params are gated as non-editable in the cell.
        self.assertIn("readOnly", source)
        self.assertIn("read_only", source)

    def test_circular_progress_is_pure_svg_ring(self):
        source = open(_CIRCULAR_PROGRESS_PATH, encoding="utf-8").read()
        self.assertIn("stroke-dashoffset".replace("-", ""), source.replace("-", ""))
        self.assertIn("percent", source)
        self.assertIn("colors.", source)
        # Indeterminate spinner uses CSS keyframes (so it honours
        # prefers-reduced-motion), not SVG SMIL.
        self.assertIn("indeterminate", source)
        self.assertIn("@keyframes", source)
        self.assertIn("prefers-reduced-motion", source)
        self.assertNotIn("animateTransform", source)

    def test_parameters_tab_uses_settings_theme_colors(self):
        sources = [
            open(_PARAMETERS_TAB_PATH, encoding="utf-8").read(),
            open(_VIRTUAL_PARAM_GRID_PATH, encoding="utf-8").read(),
            open(_CIRCULAR_PROGRESS_PATH, encoding="utf-8").read(),
        ]
        forbidden = ["#1a1a1a", "#0d0d0d", "#222", "#333", "#444", "#555"]
        for source in sources:
            self.assertIn("colors.", source)
            for literal in forbidden:
                self.assertNotIn(literal, source)

    def test_update_ack_cleanup_uses_semantic_draft_matching(self):
        source = open(_USE_FULL_PARAMS_PATH, encoding="utf-8").read()
        self.assertIn("clearVehicleDrafts", source)
        self.assertIn("draftValueMatchesSubmitted", source)
        self.assertIn("draftsRef.current = next", source)
        self.assertNotIn("String(value) === String(rec.value)", source)
        self.assertNotIn("draft[change.name] === change.value", source)


class TestParamHeaderStatus(unittest.TestCase):
    """Grid column-header state — must never go blank after a UAV finishes."""

    def _kind(self, progress, loaded):
        prog = "null" if progress is None else json.dumps(progress)
        return _call_json(f"paramHeaderStatus({prog}, {json.dumps(loaded)})")

    def test_active_while_downloading(self):
        self.assertEqual(self._kind({"percent": 47, "done": False, "error": None}, False), "active")

    def test_error(self):
        self.assertEqual(self._kind({"done": False, "error": "boom"}, False), "error")
        # An error on a re-refresh still shows the failure even if an old snapshot exists.
        self.assertEqual(self._kind({"done": False, "error": "boom"}, True), "error")

    def test_done_via_progress_before_snapshot_stored(self):
        # The bug: a UAV that finished early must show 'done' (check), not blank,
        # while waiting for the slower UAVs' snapshots to land.
        self.assertEqual(self._kind({"done": True, "error": None}, False), "done")

    def test_done_via_loaded_snapshot(self):
        self.assertEqual(self._kind(None, True), "done")

    def test_idle_when_nothing_yet(self):
        self.assertEqual(self._kind(None, False), "idle")


class TestTypedEditorSource(unittest.TestCase):
    """Step 2 — the editor stages the coerced stored value and shows a
    readable type + storage range instead of a raw ap_type number."""

    def test_cell_commits_coerced_stored_value(self):
        source = open(_VIRTUAL_PARAM_GRID_PATH, encoding="utf-8").read()
        self.assertIn("coerceValueForRecord", source)
        # Commit stages the coerced value, falling back to raw text only when
        # the entry is not a finite number (no silent revert).
        self.assertIn("coerced !== null ? coerced : text", source)

    def test_tooltip_shows_readable_type_not_raw_ap_type(self):
        source = open(_VIRTUAL_PARAM_GRID_PATH, encoding="utf-8").read()
        self.assertIn("describeParamType", source)
        self.assertIn("type: ", source)
        # The raw "ap_type: <n>" tooltip label is gone.
        self.assertNotIn("ap_type: ${rec.ap_type}", source)

    def test_import_report_surfaces_coerced_count(self):
        source = open(_PARAMETERS_TAB_PATH, encoding="utf-8").read()
        self.assertIn("totalCoerced", source)
        self.assertIn("a.coerced", source)
        self.assertIn("settings.parameters.coerced", source)

    def test_cell_button_discards_edit_not_reset_to_default(self):
        # The per-cell ↺ button cancels a pending edit (reverts to the vehicle's
        # current value, which setVehicleDraft prunes), shown only when a draft
        # exists. It no longer stages the default ("reset to default").
        source = open(_VIRTUAL_PARAM_GRID_PATH, encoding="utf-8").read()
        self.assertIn("onDiscardDraft", source)
        self.assertIn("onChange(sysId, name, rec.value)", source)
        self.assertIn("showDiscardBtn = present && hasDraft", source)
        self.assertNotIn("onResetDefault", source)
        self.assertNotIn("reset to default", source)


class TestWriteProgressStateMachine(unittest.TestCase):
    """Per-UAV write progress: seed at start, WS-driven updates, straggler
    guard, clear on completion, prune on disconnect, header precedence."""

    def test_use_full_params_write_progress_wiring(self):
        src = open(_USE_FULL_PARAMS_PATH, encoding="utf-8").read()
        self.assertIn("handleWriteProgress", src)
        # written clamped to [0, total].
        self.assertIn("Math.min(total", src)
        # Straggler guard: ignore frames after the write slot is released.
        self.assertIn("pendingRef.current[sid]", src)
        # Seeded and cleared via the write-progress state; pruned on disconnect.
        self.assertIn("setWriteProgressByVehicle", src)
        self.assertIn("writeProgressByVehicle", src)

    def test_grid_write_ring_precedence(self):
        src = open(_VIRTUAL_PARAM_GRID_PATH, encoding="utf-8").read()
        # A write in progress shows a green ring + percent, gated on an
        # unfinished writeProgress (precedence over download/loaded state).
        self.assertIn("writeProgress", src)
        self.assertIn("!writeProgress.done", src)
        self.assertIn("settings.parameters.writing", src)


class TestFilterChangesByNames(unittest.TestCase):
    """Scoping a write diff to a name set (used by the Failsafe tab's
    per-group Save so it never flushes unrelated drafts)."""

    _CHANGES = '[{"name":"FS_THR_ENABLE","value":1},{"name":"BATT_LOW_VOLT","value":14},{"name":"AAS_DEL_PITCH","value":-30}]'

    def test_keeps_only_listed_names(self):
        result = _call_json(
            f"filterChangesByNames({self._CHANGES}, ['FS_THR_ENABLE','BATT_LOW_VOLT'])"
        )
        self.assertEqual([c["name"] for c in result], ["FS_THR_ENABLE", "BATT_LOW_VOLT"])

    def test_accepts_a_set(self):
        result = _call_json(
            f"filterChangesByNames({self._CHANGES}, new Set(['AAS_DEL_PITCH']))"
        )
        self.assertEqual([c["name"] for c in result], ["AAS_DEL_PITCH"])

    def test_nullish_names_is_passthrough(self):
        result = _call_json(f"filterChangesByNames({self._CHANGES}, null)")
        self.assertEqual(len(result), 3)

    def test_empty_inputs(self):
        self.assertEqual(_call_json("filterChangesByNames(null, ['X'])"), [])
        self.assertEqual(_call_json(f"filterChangesByNames({self._CHANGES}, [])"), [])


if __name__ == "__main__":
    unittest.main()
