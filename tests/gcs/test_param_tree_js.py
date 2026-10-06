"""Pure-helper tests for src/gcs/frontend/src/components/settings/paramTree.js.

Drives the JS via Node so canonical name order, prefix grouping, filter
logic, and virtual-window math are locked into a contract before the
ParametersTab consumes them.
"""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "settings",
    "paramTree.js",
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


def _snap(name_order):
    return {"nameOrder": list(name_order), "paramsByName": {}, "records": []}


class TestCanonicalNameOrder(unittest.TestCase):
    def test_first_vehicle_wins(self):
        snaps = {1: _snap(["A", "B", "C"]), 2: _snap(["X", "B", "Y"])}
        result = _call_json(
            f"canonicalNameOrder({json.dumps(snaps)}, [1, 2])"
        )
        self.assertEqual(result, ["A", "B", "C", "X", "Y"])

    def test_missing_snapshot_skipped(self):
        snaps = {1: _snap(["A"])}
        result = _call_json(f"canonicalNameOrder({json.dumps(snaps)}, [1, 2])")
        self.assertEqual(result, ["A"])

    def test_empty_returns_empty(self):
        result = _call_json("canonicalNameOrder({}, [])")
        self.assertEqual(result, [])

    def test_no_duplicates(self):
        snaps = {1: _snap(["A", "B"]), 2: _snap(["B", "A"])}
        result = _call_json(f"canonicalNameOrder({json.dumps(snaps)}, [1, 2])")
        self.assertEqual(result, ["A", "B"])


class TestBuildPrefixSidebar(unittest.TestCase):
    def test_groups_by_underscore(self):
        result = _call_json(
            "buildPrefixSidebar(['ARMING_REQUIRE','ARMING_CHECK','RC1_OPTION','RC2_OPTION','FORMAT'])"
        )
        # Sorted alphabetically by prefix.
        self.assertEqual(result, [
            {"prefix": "ARMING", "count": 2, "children": []},
            {"prefix": "RC1", "count": 1, "children": []},
            {"prefix": "RC2", "count": 1, "children": []},
        ])

    def test_no_underscore_names_do_not_create_sidebar_group(self):
        result = _call_json(
            "buildPrefixSidebar(['FORMAT_VERSION','VERSION'])"
        )
        # 'FORMAT' counts once (FORMAT_VERSION); VERSION stays in All/search.
        self.assertEqual(result, [
            {"prefix": "FORMAT", "count": 1, "children": []},
        ])

    def test_only_no_underscore_names_return_no_sidebar_groups(self):
        result = _call_json("buildPrefixSidebar(['FORMAT','VERSION'])")
        self.assertEqual(result, [])

    def test_adds_second_level_subgroups(self):
        result = _call_json(
            "buildPrefixSidebar(['INS_POS1_X','INS_POS1_Y','INS_GYR_ID','INS_FAST_SAMPLE','ARMING_CHECK'])"
        )
        self.assertEqual(result, [
            {"prefix": "ARMING", "count": 1, "children": []},
            {
                "prefix": "INS",
                "count": 4,
                "children": [
                    {"prefix": "INS_FAST", "count": 1},
                    {"prefix": "INS_GYR", "count": 1},
                    {"prefix": "INS_POS1", "count": 2},
                ],
            },
        ])


class TestFilterCanonicalNames(unittest.TestCase):
    def _filter(self, names, **kw):
        return _call_json(
            f"filterCanonicalNames({json.dumps(names)}, {json.dumps(kw)})"
        )

    def test_no_filter_returns_all(self):
        names = ["AA", "BB", "CC"]
        self.assertEqual(self._filter(names), ["AA", "BB", "CC"])

    def test_search_substring_case_insensitive(self):
        names = ["ARMING_CHECK", "RC1_OPTION", "ARMING_REQUIRE"]
        result = self._filter(names, searchText="arm")
        self.assertEqual(result, ["ARMING_CHECK", "ARMING_REQUIRE"])

    def test_selected_prefix(self):
        names = ["ARMING_CHECK", "RC1_OPTION", "ARMING_REQUIRE"]
        result = self._filter(names, selectedPrefix="ARMING")
        self.assertEqual(result, ["ARMING_CHECK", "ARMING_REQUIRE"])

    def test_selected_subgroup(self):
        names = ["INS_POS1_X", "INS_POS1_Y", "INS_GYR_ID", "INS_FAST_SAMPLE"]
        result = self._filter(names, selectedPrefix="INS_POS1")
        self.assertEqual(result, ["INS_POS1_X", "INS_POS1_Y"])

    def test_prefix_and_search_combined(self):
        names = ["ARMING_CHECK", "ARMING_REQUIRE", "RC1_OPTION"]
        result = self._filter(names,
                              selectedPrefix="ARMING", searchText="check")
        self.assertEqual(result, ["ARMING_CHECK"])

    def test_root_prefix_filters_no_underscore(self):
        names = ["FORMAT", "ARMING_X"]
        result = self._filter(names, selectedPrefix="(root)")
        self.assertEqual(result, ["FORMAT"])

    def test_all_returns_unfiltered(self):
        names = ["FORMAT", "ARMING_X"]
        result = self._filter(names, selectedPrefix="(all)")
        self.assertEqual(result, ["FORMAT", "ARMING_X"])


class TestComputeVisibleRange(unittest.TestCase):
    def _range(self, scrollTop, viewportHeight, rowHeight, rowCount, overscan=6):
        return _call_json(
            f"computeVisibleRange({scrollTop}, {viewportHeight}, "
            f"{rowHeight}, {rowCount}, {overscan})"
        )

    def test_basic(self):
        # Viewport 0..200 with 28px rows -> rows 0..7 visible. With overscan
        # 6, range is [max(0,0-6)=0, min(rowCount, 8+6)=14].
        self.assertEqual(self._range(0, 200, 28, 1500), {"start": 0, "end": 14})

    def test_scrolled(self):
        # Scrolled to top=560 (row 20). Viewport 200 → rows 20..27. Overscan
        # 6 → start=14, end=33.
        result = self._range(560, 200, 28, 1500, 6)
        self.assertEqual(result["start"], 14)
        self.assertEqual(result["end"], 34)

    def test_zero_rows(self):
        self.assertEqual(self._range(0, 200, 28, 0), {"start": 0, "end": 0})

    def test_clamps_to_row_count(self):
        result = self._range(2_000_000, 200, 28, 100)
        self.assertLessEqual(result["end"], 100)
        self.assertGreaterEqual(result["start"], 0)


class TestRecordsAllSame(unittest.TestCase):
    def _all_same(self, recs):
        return _call_json(f"recordsAllSame({json.dumps(recs)})")

    def test_empty_true(self):
        self.assertTrue(self._all_same([]))

    def test_single_true(self):
        self.assertTrue(self._all_same([{"value": 1, "ap_type": 4}]))

    def test_two_same_int(self):
        rec = {"value": 5, "ap_type": 3}
        self.assertTrue(self._all_same([rec, dict(rec)]))

    def test_two_diff_int(self):
        self.assertFalse(self._all_same([
            {"value": 5, "ap_type": 3}, {"value": 6, "ap_type": 3},
        ]))

    def test_float_within_tolerance(self):
        self.assertTrue(self._all_same([
            {"value": 1.0, "ap_type": 4},
            {"value": 1.0 + 1e-7, "ap_type": 4},
        ]))

    def test_float_outside_tolerance(self):
        self.assertFalse(self._all_same([
            {"value": 1.0, "ap_type": 4},
            {"value": 1.5, "ap_type": 4},
        ]))

    def test_partial_missing_false(self):
        self.assertFalse(self._all_same([
            {"value": 1, "ap_type": 3}, None,
        ]))


class TestComputeVisibleRangeClamps(unittest.TestCase):
    def test_overscrolled_returns_valid_range(self):
        # scrollTop way beyond rowCount * rowHeight — should clamp.
        result = _call_json(
            "computeVisibleRange(1000000, 200, 28, 5, 6)"
        )
        self.assertGreaterEqual(result["start"], 0)
        self.assertGreaterEqual(result["end"], result["start"])
        self.assertLessEqual(result["end"], 5)


class TestFormatValueForDisplay(unittest.TestCase):
    def _fmt(self, record):
        return _call_json(f"({{ s: formatValueForDisplay({json.dumps(record)}) }})")["s"]

    def test_int_formatting(self):
        self.assertEqual(self._fmt({"value": 42, "ap_type": 3}), "42")
        self.assertEqual(self._fmt({"value": 42.0, "ap_type": 3}), "42")

    def test_float_formatting(self):
        # 3.14 to 6 sig figs is 3.14, trim trailing zeros.
        self.assertEqual(self._fmt({"value": 3.14, "ap_type": 4}), "3.14")

    def test_null_record(self):
        self.assertEqual(_call_json(
            "({s: formatValueForDisplay(null)})"
        )["s"], "")


def _rec(value, ap_type=4):
    return {"value": value, "ap_type": ap_type}


class TestDifferingParamNames(unittest.TestCase):
    """Names whose value differs across UAVs (or is missing on some)."""

    def test_fewer_than_two_uavs_is_empty(self):
        snaps = {1: {"paramsByName": {"A": _rec(1)}}}
        r = _call_json(f"[...differingParamNames({json.dumps(snaps)}, [1], ['A'])]")
        self.assertEqual(r, [])

    def test_differing_and_missing_flagged(self):
        snaps = {
            1: {"paramsByName": {"A": _rec(1.0), "B": _rec(2.0), "C": _rec(3.0)}},
            2: {"paramsByName": {"A": _rec(1.0), "B": _rec(9.0)}},  # B differs, C missing
        }
        r = _call_json(
            f"[...differingParamNames({json.dumps(snaps)}, [1, 2], ['A', 'B', 'C'])]"
        )
        self.assertEqual(sorted(r), ["B", "C"])

    def test_all_same_is_empty(self):
        snaps = {
            1: {"paramsByName": {"A": _rec(5, 2)}},
            2: {"paramsByName": {"A": _rec(5, 2)}},
        }
        r = _call_json(f"[...differingParamNames({json.dumps(snaps)}, [1, 2], ['A'])]")
        self.assertEqual(r, [])


if __name__ == "__main__":
    unittest.main()
