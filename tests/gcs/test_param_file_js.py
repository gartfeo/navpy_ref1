"""Pure-helper tests for src/gcs/frontend/src/utils/paramFile.js.

Locks the contract for `.param` file save/load so Step 7 round-trips
through `parseParamFile` and `snapshotToParamFile` correctly, and
`validateRowsForVehicle` correctly partitions rows into
accepted/skipped/rejected per vehicle.
"""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_FILE_UTIL = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "paramFile.js",
))
_FULL_UTIL = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "fullParams.js",
))


def _strip_imports(src: str, *, also_drop_full_import: bool) -> str:
    out = (
        src.replace("export function ", "function ")
           .replace("export const ", "const ")
    )
    if also_drop_full_import:
        # paramFile imports from './fullParams' — drop the import line; we
        # concat the source directly.
        out = "\n".join(
            line for line in out.splitlines()
            if not line.lstrip().startswith("import ")
        )
    return out


_FULL_SRC = _strip_imports(open(_FULL_UTIL, encoding="utf-8").read(),
                           also_drop_full_import=False)
_FILE_SRC = _strip_imports(open(_FILE_UTIL, encoding="utf-8").read(),
                           also_drop_full_import=True)
_JS_SRC = _FULL_SRC + "\n" + _FILE_SRC


def _run_js(script: str) -> str:
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _call_json(expr: str):
    return json.loads(_run_js(f"console.log(JSON.stringify({expr}));"))


class TestParseParamFile(unittest.TestCase):
    def test_csv_format(self):
        text = "ARMING_REQUIRE,1\nRC1_OPTION,55\n"
        result = _call_json(f"parseParamFile({json.dumps(text)})")
        self.assertEqual(result["rows"], [
            {"name": "ARMING_REQUIRE", "value": 1},
            {"name": "RC1_OPTION", "value": 55},
        ])
        self.assertEqual(result["errors"], [])

    def test_whitespace_format(self):
        text = "ARMING_REQUIRE 1\nRC1_OPTION  55\n"
        result = _call_json(f"parseParamFile({json.dumps(text)})")
        self.assertEqual([r["name"] for r in result["rows"]],
                         ["ARMING_REQUIRE", "RC1_OPTION"])

    def test_comments_and_blanks_ignored(self):
        text = "# header\n\nARMING_REQUIRE,1\n# trailing\n"
        result = _call_json(f"parseParamFile({json.dumps(text)})")
        self.assertEqual(len(result["rows"]), 1)
        self.assertEqual(result["errors"], [])

    def test_invalid_name(self):
        text = "bad-name,1\n"
        result = _call_json(f"parseParamFile({json.dumps(text)})")
        self.assertEqual(result["rows"], [])
        self.assertEqual(len(result["errors"]), 1)

    def test_non_numeric_value(self):
        text = "ARM,foo\n"
        result = _call_json(f"parseParamFile({json.dumps(text)})")
        self.assertEqual(len(result["errors"]), 1)


class TestValidateRowsForVehicle(unittest.TestCase):
    def _validate(self, rows, snap_records):
        snap = {
            "params": snap_records,
            "num_params": len(snap_records),
            "total_params": len(snap_records),
            "fetched_at_unix_s": 1.0,
            "stale": False,
        }
        norm_expr = f"normaliseSnapshot({json.dumps(snap)})"
        return _call_json(
            f"validateRowsForVehicle({json.dumps(rows)}, {norm_expr})"
        )

    def test_accepted_path(self):
        rows = [{"name": "X", "value": 5}]
        recs = [{"name": "X", "value": 0, "ap_type": 3,
                 "default": 0, "default_known": False, "flags": 0}]
        result = self._validate(rows, recs)
        self.assertEqual(result["accepted"], [{"name": "X", "value": 5}])
        self.assertEqual(result["skipped"], [])
        self.assertEqual(result["rejected"], [])

    def test_skipped_unknown(self):
        rows = [{"name": "NOT_A_PARAM", "value": 1}]
        recs = [{"name": "X", "value": 0, "ap_type": 3,
                 "default": 0, "default_known": False, "flags": 0}]
        result = self._validate(rows, recs)
        self.assertEqual(len(result["skipped"]), 1)

    def test_fractional_for_int_accepted_coerced(self):
        # Honest-permissive: import stages the value the autopilot would store
        # (truncate toward zero) and flags the adjustment for the preview.
        rows = [{"name": "X", "value": 1.5}]
        recs = [{"name": "X", "value": 0, "ap_type": 3,
                 "default": 0, "default_known": False, "flags": 0}]
        result = self._validate(rows, recs)
        self.assertEqual(result["rejected"], [])
        self.assertEqual(len(result["accepted"]), 1)
        self.assertEqual(result["accepted"][0]["name"], "X")
        self.assertEqual(result["accepted"][0]["value"], 1)
        self.assertTrue(result["accepted"][0]["coerced"])
        self.assertEqual(result["accepted"][0]["rawValue"], 1.5)

    def test_out_of_range_int_accepted_clamped(self):
        rows = [{"name": "X", "value": 200}]
        recs = [{"name": "X", "value": 0, "ap_type": 1,  # INT8 -128..127
                 "default": 0, "default_known": False, "flags": 0}]
        result = self._validate(rows, recs)
        self.assertEqual(result["rejected"], [])
        self.assertEqual(result["accepted"][0]["value"], 127)
        self.assertTrue(result["accepted"][0]["coerced"])

    def test_exact_int_accepted_without_coerced_flag(self):
        rows = [{"name": "X", "value": 5}]
        recs = [{"name": "X", "value": 0, "ap_type": 3,
                 "default": 0, "default_known": False, "flags": 0}]
        result = self._validate(rows, recs)
        self.assertEqual(result["accepted"], [{"name": "X", "value": 5}])

    def test_non_numeric_value_rejected(self):
        rows = [{"name": "X", "value": "abc"}]
        recs = [{"name": "X", "value": 0, "ap_type": 3,
                 "default": 0, "default_known": False, "flags": 0}]
        result = self._validate(rows, recs)
        self.assertEqual(len(result["rejected"]), 1)
        self.assertIn("invalid", result["rejected"][0]["reason"])

    def test_no_snapshot_all_skipped(self):
        result = _call_json(
            f"validateRowsForVehicle({json.dumps([{'name':'X','value':1}])}, null)"
        )
        self.assertEqual(len(result["skipped"]), 1)


class TestSnapshotToParamFile(unittest.TestCase):
    def test_round_trip(self):
        snap = {
            "params": [
                {"name": "A", "value": 5, "ap_type": 3,
                 "default": 0, "default_known": False, "flags": 0},
                {"name": "B", "value": 1.5, "ap_type": 4,
                 "default": 0.0, "default_known": True, "flags": 1},
            ],
            "num_params": 2, "total_params": 2,
            "fetched_at_unix_s": 1.0, "stale": False,
        }
        text = _run_js(
            f"console.log(JSON.stringify(snapshotToParamFile(normaliseSnapshot({json.dumps(snap)}))));"
        )
        text = json.loads(text)
        # Round-trip via parseParamFile.
        parsed = _call_json(f"parseParamFile({json.dumps(text)})")
        self.assertEqual(len(parsed["rows"]), 2)
        names = [r["name"] for r in parsed["rows"]]
        self.assertEqual(names, ["A", "B"])
        # Integer value preserved exactly.
        a = parsed["rows"][0]
        self.assertEqual(a["value"], 5)
        # Float value within tolerance.
        b = parsed["rows"][1]
        self.assertAlmostEqual(b["value"], 1.5, places=5)


if __name__ == "__main__":
    unittest.main()
