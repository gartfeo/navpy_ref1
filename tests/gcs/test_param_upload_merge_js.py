"""Tests for the pure helpers in paramUpload.js.

These guard the AAS-param upload flow contract:
    - selectChangedParams() picks only fields whose current value differs
      from the downloaded baseline.
    - mergeUploadResults() commits to the baseline ONLY the fields the
      autopilot acked, using the submitted snapshot rather than live
      edited state (so an in-flight re-edit cannot be silently baselined).
"""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "paramUpload.js",
))

# Strip ES module syntax so Node.js can eval the code.
_raw = open(_UTIL_PATH, encoding="utf-8").read()
_JS_SRC = (
    _raw
    .replace("export function ", "function ")
    .replace("export const ", "const ")
)


def _run_js(script):
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _select(vp_json, dl_json):
    return json.loads(_run_js(
        f"console.log(JSON.stringify(selectChangedParams({vp_json}, {dl_json})));"
    ))


def _merge(baseline_json, submitted_json, results_json):
    return json.loads(_run_js(
        "console.log(JSON.stringify(mergeUploadResults("
        f"{baseline_json}, {submitted_json}, {results_json})));"
    ))


class TestSelectChangedParams(unittest.TestCase):
    def test_picks_only_changed_fields(self):
        result = _select(
            '{"del_pitch": 10, "use_trn": true, "log_rate": 2}',
            '{"del_pitch": 5,  "use_trn": true, "log_rate": 2}',
        )
        self.assertEqual(result, {"del_pitch": 10})

    def test_loose_equality_ignores_numeric_string_round_trip(self):
        # Inputs from <input type="number"> can come back as strings.
        result = _select('{"del_pitch": "10"}', '{"del_pitch": 10}')
        self.assertEqual(result, {})

    def test_field_missing_from_baseline_counts_as_changed(self):
        result = _select('{"new_field": 7}', "{}")
        self.assertEqual(result, {"new_field": 7})

    def test_empty_or_null_inputs_return_empty(self):
        self.assertEqual(_select("null", "null"), {})
        self.assertEqual(_select("{}", '{"del_pitch": 5}'), {})


class TestMergeUploadResults(unittest.TestCase):
    def test_only_acked_fields_merge_into_baseline(self):
        baseline = '{"del_pitch": 5, "use_trn": true}'
        submitted = '{"del_pitch": 10, "use_trn": false}'
        results = '{"del_pitch": true, "use_trn": false}'
        merged = _merge(baseline, submitted, results)
        self.assertEqual(merged, {"del_pitch": 10, "use_trn": True})  # use_trn stayed pending

    def test_failed_fields_remain_pending_so_ui_shows_diff(self):
        baseline = '{"del_pitch": 5}'
        submitted = '{"del_pitch": 10}'
        results = '{"del_pitch": false}'
        self.assertEqual(_merge(baseline, submitted, results), {"del_pitch": 5})

    def test_uses_submitted_snapshot_not_live_edits(self):
        # Race scenario: user PUT del_pitch=10, then re-typed 99 before ack.
        # When the ack for 10 arrives, baseline must move to 10 (the value
        # actually written), not 99 (which is still unsent).
        baseline = '{"del_pitch": 5}'
        submitted_snapshot = '{"del_pitch": 10}'
        results = '{"del_pitch": true}'
        merged = _merge(baseline, submitted_snapshot, results)
        self.assertEqual(merged, {"del_pitch": 10})

    def test_does_not_mutate_baseline(self):
        out = _run_js(
            "const b = {del_pitch: 5};"
            "mergeUploadResults(b, {del_pitch: 10}, {del_pitch: true});"
            "console.log(JSON.stringify(b));"
        )
        self.assertEqual(json.loads(out), {"del_pitch": 5})

    def test_unknown_result_keys_ignored(self):
        # Defensive: if the backend returned a key we never submitted, we
        # don't fabricate a value for it.
        baseline = "{}"
        submitted = '{"del_pitch": 10}'
        results = '{"del_pitch": true, "ghost": true}'
        self.assertEqual(_merge(baseline, submitted, results), {"del_pitch": 10})

    def test_null_inputs_safe(self):
        self.assertEqual(_merge("null", "null", "null"), {})
        self.assertEqual(_merge("{}", "{}", "{}"), {})


if __name__ == "__main__":
    unittest.main()
