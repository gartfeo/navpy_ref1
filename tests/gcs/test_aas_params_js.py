"""Tests for the pure helpers in aasParams.js.

Locks in the data-shape contract that the per-vehicle AAS hook (Step 3)
and every consumer (planning sidebar, settings modal, task confirm card)
depend on:
    - AAS_DEFAULTS matches a known-expected literal.
    - getConsensus classifies session / loading / ready / mixed / unknown
      including the partial-missing case (some loaded vehicles have a
      value, others have undefined for the same key).
    - pickDisplayValue falls back through session draft -> defaults -> null.
    - selectChangedFields respects loose equality and an optional key list.
    - mergeAckedFieldsIntoBaseline never advances baseline on a failed ack.
    - aggregateFanoutResults distinguishes per-field failures from a whole-
      vehicle network failure when results payload is null/missing.
"""
import json
import os
from tests.gcs.js_runner import run_node
import unittest


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "aasParams.js",
))

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


def _consensus(cbv_json, sids_json, key):
    return json.loads(_run_js(
        f"console.log(JSON.stringify(getConsensus({cbv_json}, {sids_json}, {json.dumps(key)})));"
    ))


def _pick(consensus_json, session_json, key):
    expr = f"pickDisplayValue({consensus_json}, {session_json}, {json.dumps(key)})"
    return json.loads(_run_js(
        f"console.log(JSON.stringify({{v: {expr}}}));"
    ))["v"]


def _changed(draft_json, baseline_json, keylist_json="undefined"):
    return json.loads(_run_js(
        f"console.log(JSON.stringify(selectChangedFields({draft_json}, {baseline_json}, {keylist_json})));"
    ))


def _merge(baseline_json, submitted_json, results_json):
    return json.loads(_run_js(
        "console.log(JSON.stringify(mergeAckedFieldsIntoBaseline("
        f"{baseline_json}, {submitted_json}, {results_json})));"
    ))


def _aggregate(per_vehicle_json, submitted_json):
    return json.loads(_run_js(
        "console.log(JSON.stringify(aggregateFanoutResults("
        f"{per_vehicle_json}, {submitted_json})));"
    ))


def _defaults():
    return json.loads(_run_js("console.log(JSON.stringify(AAS_DEFAULTS));"))


# Frozen expected defaults. Locked deliberately to a literal so the test
# is not coupled to any backend Pydantic model and remains stable across
# refactors of the settings store.
_EXPECTED_DEFAULTS = {
    "del_pitch": 0,
    "del_thr": -1,
    "del_dir": False,
    "del_p_kp": 1.5,
    "del_pld": -1,
    "del_plrd": -1,
    "del_ctrl": 2,
    "use_trn": True,
    "targ_wps": 16,
    "targ_alt": 150,
    "nav_last_wp": 3,
    "nav_min_alt": 150,
    "nav_cwt": 30,
    "nav_cgt": 15,
    "nav_auto_cm": True,
    "nav_cm_fl": False,
    "nav_oneshot": False,
    "log_defer": False,
    "log_rate": 2,
}


class TestAasDefaults(unittest.TestCase):
    def test_defaults_match_expected(self):
        self.assertEqual(_defaults(), _EXPECTED_DEFAULTS)


class TestConsensus(unittest.TestCase):
    def test_session_when_no_sysids(self):
        self.assertEqual(_consensus("{}", "[]", "nav_auto_cm"), {"state": "session"})

    def test_loading_when_any_sysid_pending(self):
        self.assertEqual(
            _consensus('{"1": {"nav_auto_cm": true}}', "[1, 2]", "nav_auto_cm"),
            {"state": "loading"},
        )

    def test_ready_when_all_loaded_agree(self):
        result = _consensus(
            '{"1": {"nav_auto_cm": false}, "2": {"nav_auto_cm": false}}',
            "[1, 2]",
            "nav_auto_cm",
        )
        self.assertEqual(result, {"state": "ready", "value": False})

    def test_mixed_when_loaded_disagree(self):
        result = _consensus(
            '{"1": {"nav_auto_cm": true}, "2": {"nav_auto_cm": false}}',
            "[1, 2]",
            "nav_auto_cm",
        )
        self.assertEqual(result["state"], "mixed")
        self.assertEqual(result["valuesBySysId"], {"1": True, "2": False})

    def test_mixed_when_partial_missing(self):
        # One loaded vehicle has the key, the other doesn't. Per Codex
        # finding 2.3 this is mixed, NOT ready or unknown.
        result = _consensus(
            '{"1": {"nav_auto_cm": true}, "2": {}}',
            "[1, 2]",
            "nav_auto_cm",
        )
        self.assertEqual(result["state"], "mixed")
        self.assertEqual(result["valuesBySysId"], {"1": True, "2": None})

    def test_unknown_when_all_undefined(self):
        result = _consensus(
            '{"1": {}, "2": {}}',
            "[1, 2]",
            "nav_auto_cm",
        )
        self.assertEqual(result, {"state": "unknown"})

    def test_loading_takes_precedence_over_disagreement(self):
        result = _consensus(
            '{"1": {"nav_auto_cm": true}}',
            "[1, 2]",
            "nav_auto_cm",
        )
        self.assertEqual(result, {"state": "loading"})


class TestPickDisplayValue(unittest.TestCase):
    def test_session_falls_back_to_default(self):
        self.assertEqual(
            _pick('{"state": "session"}', "{}", "nav_auto_cm"),
            True,
        )

    def test_session_uses_session_draft_when_present(self):
        self.assertEqual(
            _pick('{"state": "session"}', '{"nav_auto_cm": false}', "nav_auto_cm"),
            False,
        )

    def test_ready_returns_consensus_value(self):
        self.assertEqual(
            _pick('{"state": "ready", "value": 42}', "{}", "nav_cwt"),
            42,
        )

    def test_mixed_returns_null(self):
        self.assertIsNone(
            _pick('{"state": "mixed", "valuesBySysId": {}}', "{}", "nav_cwt"),
        )

    def test_loading_returns_null(self):
        self.assertIsNone(_pick('{"state": "loading"}', "{}", "nav_cwt"))

    def test_unknown_returns_null(self):
        self.assertIsNone(_pick('{"state": "unknown"}', "{}", "nav_cwt"))

    def test_session_unknown_key_returns_null(self):
        self.assertIsNone(_pick('{"state": "session"}', "{}", "not_a_real_key"))


class TestSelectChangedFields(unittest.TestCase):
    def test_only_changed_fields(self):
        result = _changed(
            '{"del_pitch": 10, "use_trn": true, "log_rate": 2}',
            '{"del_pitch": 5,  "use_trn": true, "log_rate": 2}',
        )
        self.assertEqual(result, {"del_pitch": 10})

    def test_numeric_string_loose_equality(self):
        # Per Codex finding 2.11: numeric strings should compare equal to
        # their numeric counterpart. Edited "30" should NOT count as changed
        # against baseline 30.
        self.assertEqual(
            _changed('{"nav_cwt": "30"}', '{"nav_cwt": 30}'),
            {},
        )

    def test_key_list_filters_comparison(self):
        # Per Codex finding 2.11. Caller wants only nav_auto_cm uploaded
        # even though del_pitch also changed.
        result = _changed(
            '{"del_pitch": 10, "nav_auto_cm": false}',
            '{"del_pitch": 5,  "nav_auto_cm": true}',
            '["nav_auto_cm"]',
        )
        self.assertEqual(result, {"nav_auto_cm": False})

    def test_empty_key_list_returns_empty(self):
        # An explicit empty key list means "no keys are valid for upload."
        # Must NOT fall through to "compare every draft key" -- otherwise
        # a caller computing "no valid keys" could silently upload every
        # pending draft field.
        result = _changed(
            '{"del_pitch": 10, "nav_auto_cm": false}',
            '{"del_pitch": 5,  "nav_auto_cm": true}',
            "[]",
        )
        self.assertEqual(result, {})

    def test_empty_when_draft_null(self):
        self.assertEqual(_changed("null", '{"del_pitch": 5}'), {})

    def test_treats_baseline_null_as_no_baseline(self):
        result = _changed('{"del_pitch": 10}', "null")
        self.assertEqual(result, {"del_pitch": 10})


class TestMergeAcked(unittest.TestCase):
    def test_failed_ack_does_not_advance_baseline(self):
        # Per Codex finding 2.4. Baseline must remain unchanged for a key
        # whose ack was false.
        result = _merge(
            '{"del_pitch": 5}',
            '{"del_pitch": 10}',
            '{"del_pitch": false}',
        )
        self.assertEqual(result, {"del_pitch": 5})

    def test_successful_ack_advances_to_submitted_value(self):
        result = _merge(
            '{"del_pitch": 5}',
            '{"del_pitch": 10}',
            '{"del_pitch": true}',
        )
        self.assertEqual(result, {"del_pitch": 10})

    def test_uses_submitted_snapshot_not_live_state(self):
        # If the submitted payload omitted a key, an ack for that key must
        # not be merged in (this protects against in-flight re-edits).
        result = _merge(
            '{"del_pitch": 5}',
            '{}',
            '{"del_pitch": true}',
        )
        self.assertEqual(result, {"del_pitch": 5})

    def test_null_inputs_safe(self):
        self.assertEqual(_merge("null", "null", "null"), {})


class TestAggregateFanout(unittest.TestCase):
    def test_partial_failure_per_field(self):
        # Vehicle 1 acked del_pitch; vehicle 2 failed del_pitch.
        result = _aggregate(
            '{"1": {"del_pitch": true}, "2": {"del_pitch": false}}',
            '{"1": {"del_pitch": 10}, "2": {"del_pitch": 10}}',
        )
        self.assertEqual(result["uavOk"], 1)
        self.assertEqual(result["uavFail"], 1)
        self.assertEqual(result["fieldOk"], 1)
        self.assertEqual(result["fieldFail"], 1)
        self.assertEqual(result["perKeyOk"], {"del_pitch": 1})
        self.assertEqual(result["perKeyFail"], {"del_pitch": 1})

    def test_network_failure_counts_every_submitted_key(self):
        # Per Codex finding 2.5. Vehicle 2's PUT itself returned no result,
        # so every submitted key for that vehicle is counted as failed.
        result = _aggregate(
            '{"1": {"del_pitch": true, "use_trn": true}}',
            '{"1": {"del_pitch": 10, "use_trn": false}, "2": {"del_pitch": 10, "use_trn": false}}',
        )
        self.assertEqual(result["uavOk"], 1)
        self.assertEqual(result["uavFail"], 1)
        self.assertEqual(result["fieldOk"], 2)
        self.assertEqual(result["fieldFail"], 2)
        self.assertEqual(result["perKeyFail"], {"del_pitch": 1, "use_trn": 1})

    def test_skips_vehicles_with_no_submission(self):
        # A vehicle in submittedByVehicle with empty payload contributes nothing.
        result = _aggregate(
            '{"1": {"del_pitch": true}}',
            '{"1": {"del_pitch": 10}, "2": {}}',
        )
        self.assertEqual(result["uavOk"], 1)
        self.assertEqual(result["uavFail"], 0)

    def test_explicit_null_results_treated_as_network_failure(self):
        result = _aggregate(
            '{"1": null}',
            '{"1": {"del_pitch": 10}}',
        )
        self.assertEqual(result["uavFail"], 1)
        self.assertEqual(result["fieldFail"], 1)


if __name__ == "__main__":
    unittest.main()
