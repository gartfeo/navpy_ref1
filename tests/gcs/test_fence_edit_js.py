"""Tests for fenceEdit.js — operator edits of the inclusion-fence polygon.

The fence is auto-derived until the first edit gesture seeds a custom ring;
these helpers implement seed-then-edit. Runs the real production source via
Node.js subprocess.
"""
import json
import os
import unittest

from tests.gcs.js_runner import run_node

_SRC = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src",
    "utils", "fenceEdit.js",
))
_JS = open(_SRC, encoding="utf-8").read()
_JS = _JS.replace("export function ", "function ").replace("export const ", "const ")


def _run(script):
    result = run_node(_JS + "\n" + script, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error: {result.stderr}")
    return json.loads(result.stdout.strip())


AUTO = [
    {"lat": 0, "lon": 0}, {"lat": 0, "lon": 10},
    {"lat": 10, "lon": 10}, {"lat": 10, "lon": 0},
]
PT = {"lat": 5, "lon": -3}


class TestFenceVertexDrag(unittest.TestCase):

    def test_first_drag_seeds_from_auto(self):
        """custom=null + drag -> a new ring based on the auto fence."""
        out = _run(
            f"console.log(JSON.stringify(fenceVertexDrag(null, {json.dumps(AUTO)}, 0, {json.dumps(PT)})));"
        )
        self.assertEqual(len(out), 4)
        self.assertEqual(out[0], PT)
        self.assertEqual(out[1], AUTO[1])

    def test_drag_on_existing_custom(self):
        custom = [{"lat": 1, "lon": 1}, {"lat": 1, "lon": 2}, {"lat": 2, "lon": 2}]
        out = _run(
            f"console.log(JSON.stringify(fenceVertexDrag({json.dumps(custom)}, {json.dumps(AUTO)}, 2, {json.dumps(PT)})));"
        )
        self.assertEqual(len(out), 3)
        self.assertEqual(out[2], PT)
        self.assertEqual(out[0], custom[0])  # seeded from custom, not auto

    def test_out_of_range_returns_custom_unchanged(self):
        out = _run(
            f"console.log(JSON.stringify(fenceVertexDrag(null, {json.dumps(AUTO)}, 99, {json.dumps(PT)})));"
        )
        self.assertIsNone(out)

    def test_no_auto_no_custom_returns_null(self):
        out = _run(
            f"console.log(JSON.stringify(fenceVertexDrag(null, null, 0, {json.dumps(PT)})));"
        )
        self.assertIsNone(out)

    def test_auto_input_not_mutated(self):
        out = _run(
            f"const auto = {json.dumps(AUTO)};"
            f"fenceVertexDrag(null, auto, 0, {json.dumps(PT)});"
            f"console.log(JSON.stringify(auto[0]));"
        )
        self.assertEqual(out, AUTO[0])


class TestFenceMidpointInsert(unittest.TestCase):

    def test_insert_after_index(self):
        out = _run(
            f"console.log(JSON.stringify(fenceMidpointInsert(null, {json.dumps(AUTO)}, 1, {json.dumps(PT)})));"
        )
        self.assertEqual(len(out), 5)
        self.assertEqual(out[2], PT)

    def test_insert_after_last_edge(self):
        out = _run(
            f"console.log(JSON.stringify(fenceMidpointInsert(null, {json.dumps(AUTO)}, 3, {json.dumps(PT)})));"
        )
        self.assertEqual(len(out), 5)
        self.assertEqual(out[4], PT)


class TestFenceVertexDelete(unittest.TestCase):

    def test_delete_vertex(self):
        out = _run(
            f"console.log(JSON.stringify(fenceVertexDelete(null, {json.dumps(AUTO)}, 1)));"
        )
        self.assertEqual(len(out), 3)
        self.assertNotIn(AUTO[1], out)

    def test_min_ring_guard(self):
        """A triangle cannot lose a vertex — returns the input unchanged."""
        tri = AUTO[:3]
        out = _run(
            f"console.log(JSON.stringify(fenceVertexDelete({json.dumps(tri)}, null, 0)));"
        )
        self.assertEqual(out, tri)


if __name__ == "__main__":
    unittest.main()
