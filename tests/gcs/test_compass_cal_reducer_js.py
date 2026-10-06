"""Tests for the pure compass-cal reducer (utils/compassCal.js) via Node."""
import json
import os
import subprocess
import unittest


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "compassCal.js",
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
    result = subprocess.run(
        ["node", "-e", code],
        capture_output=True, text=True, timeout=5,
    )
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _reduce(prev_json, event_json):
    return json.loads(_run_js(
        f"console.log(JSON.stringify(reduceCompassCal({prev_json}, {event_json})));"
    ))


# MAG_CAL_STATUS values
RUNNING_STEP_ONE = 2
SUCCESS = 4
FAILED = 5


class TestReduceProgress(unittest.TestCase):
    def test_progress_creates_running_state(self):
        r = _reduce("null", json.dumps({
            "sys_id": 1, "compass_id": 0, "pct": 30,
            "cal_status": RUNNING_STEP_ONE, "report": False,
        }))
        self.assertEqual(r["status"], "running")
        self.assertEqual(r["compasses"]["0"]["pct"], 30)
        self.assertFalse(r["rebootRequired"])

    def test_progress_pct_zero_preserved(self):
        r = _reduce("null", json.dumps({
            "compass_id": 0, "pct": 0, "cal_status": RUNNING_STEP_ONE, "report": False,
        }))
        self.assertEqual(r["compasses"]["0"]["pct"], 0)


class TestReduceReport(unittest.TestCase):
    def test_single_compass_success(self):
        r = _reduce("null", json.dumps({
            "compass_id": 0, "cal_status": SUCCESS, "report": True,
            "fitness": 3.2, "autosaved": 1,
        }))
        self.assertEqual(r["status"], "success")
        self.assertTrue(r["rebootRequired"])
        self.assertEqual(r["compasses"]["0"]["pct"], 100)
        self.assertEqual(r["compasses"]["0"]["fitness"], 3.2)

    def test_single_compass_failure(self):
        r = _reduce("null", json.dumps({
            "compass_id": 0, "cal_status": FAILED, "report": True, "fitness": 99,
        }))
        self.assertEqual(r["status"], "failed")
        self.assertFalse(r["rebootRequired"])

    def test_two_compasses_one_pending_stays_running(self):
        # compass 0 reported success; compass 1 still running -> overall running
        after0 = _reduce("null", json.dumps({
            "compass_id": 0, "cal_status": SUCCESS, "report": True, "fitness": 2,
        }))
        after1 = _reduce(json.dumps(after0), json.dumps({
            "compass_id": 1, "pct": 50, "cal_status": RUNNING_STEP_ONE, "report": False,
        }))
        self.assertEqual(after1["status"], "running")
        self.assertFalse(after1["rebootRequired"])

    def test_two_compasses_both_success(self):
        after0 = _reduce("null", json.dumps({
            "compass_id": 0, "cal_status": SUCCESS, "report": True, "fitness": 2,
        }))
        after1 = _reduce(json.dumps(after0), json.dumps({
            "compass_id": 1, "cal_status": SUCCESS, "report": True, "fitness": 2,
        }))
        self.assertEqual(after1["status"], "success")
        self.assertTrue(after1["rebootRequired"])


class TestPurity(unittest.TestCase):
    def test_reduce_does_not_mutate_prev(self):
        out = _run_js(
            "const prev = reduceCompassCal(null, {compass_id:0, pct:10, cal_status:2});"
            "const before = JSON.stringify(prev);"
            "reduceCompassCal(prev, {compass_id:0, pct:90, cal_status:3});"
            "console.log(before === JSON.stringify(prev));"
        )
        self.assertEqual(out, "true")


class TestStatusKey(unittest.TestCase):
    def test_status_key_mapping(self):
        out = _run_js(
            "console.log(JSON.stringify([0,1,2,3,4,5,6,7,99].map(statusKey)));"
        )
        self.assertEqual(json.loads(out), [
            "notStarted", "waiting", "step1", "step2",
            "success", "failed", "badOrientation", "badRadius", "unknown",
        ])


class TestDeriveStatus(unittest.TestCase):
    def test_empty_started_is_running(self):
        out = _run_js("console.log(deriveStatus({}, true));")
        self.assertEqual(out, "running")

    def test_empty_not_started_is_idle(self):
        out = _run_js("console.log(deriveStatus({}, false));")
        self.assertEqual(out, "idle")


if __name__ == "__main__":
    unittest.main()
