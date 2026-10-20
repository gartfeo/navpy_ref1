"""Tests for coverageHistory.js (camera marks keyed by sys_id) via Node.js."""
import json
import os
import unittest

from tests.gcs.js_runner import run_node
from tests.gcs.test_uav_trails_js import _strip_es_modules

_JS_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "utils", "coverageHistory.js",
))
_JS = _strip_es_modules(open(_JS_FILE, encoding="utf-8").read())


def _run_js(script):
    result = run_node(_JS + "\n" + script, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestCoverageHistory(unittest.TestCase):
    def test_cap_evicts_the_fleets_oldest_mark(self):
        r = _run_js("""
            const h = newCoverageHistory();
            const removed = [];
            recordMark(h, 3, { entity: 'u3a' });
            recordMark(h, 1, { entity: 'u1a' });
            recordMark(h, 3, { entity: 'u3b' });
            recordMark(h, 1, { entity: 'u1b' });
            evictOverCap(h, 2, (e) => removed.push(e));
            console.log(JSON.stringify({ removed, n: markCount(h),
              u1: h.bySid[1].marks.map(m => m.entity), u3: h.bySid[3].marks.map(m => m.entity) }));
        """)
        self.assertEqual(r["removed"], ["u3a", "u1a"])
        self.assertEqual(r["n"], 2)
        self.assertEqual(r["u1"], ["u1b"])
        self.assertEqual(r["u3"], ["u3b"])

    def test_drop_sid_removes_only_that_uav(self):
        r = _run_js("""
            const h = newCoverageHistory();
            const removed = [];
            recordMark(h, 1, { entity: 'a' });
            recordMark(h, 2, { entity: 'b' });
            recordMark(h, 1, { entity: 'c' });
            dropSid(h, 1, (e) => removed.push(e));
            evictOverCap(h, 0, (e) => removed.push(e));
            console.log(JSON.stringify({ removed, has1: 1 in h.bySid, n: markCount(h) }));
        """)
        self.assertEqual(r["removed"], ["a", "c", "b"])
        self.assertFalse(r["has1"])
        self.assertEqual(r["n"], 0)

    def test_clear_resets_marks_and_sampling_anchor(self):
        r = _run_js("""
            const h = newCoverageHistory();
            const removed = [];
            coverageFor(h, 1).lastPos = { lat: 1, lon: 2 };
            recordMark(h, 1, { entity: 'a' });
            clearCoverage(h, (e) => removed.push(e));
            console.log(JSON.stringify({ removed, n: markCount(h), anchor: coverageFor(h, 1).lastPos }));
        """)
        self.assertEqual(r["removed"], ["a"])
        self.assertEqual(r["n"], 0)
        self.assertIsNone(r["anchor"])


if __name__ == "__main__":
    unittest.main()
