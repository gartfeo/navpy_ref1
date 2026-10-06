"""Tests for exclusionDraw.js — the keep-out drawing click reducer.

The keep-out ring must CLOSE at 3 points (same logic as the search zone) and
later clicks insert into the nearest edge of the active ring. Runs the real
production source (geo.js + exclusionDraw.js) via Node.js subprocess.
"""
import json
import os
import unittest

from tests.gcs.js_runner import run_node

_SRC_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src",
))


def _load(rel):
    src = open(os.path.join(_SRC_DIR, rel), encoding="utf-8").read()
    # Strip ESM syntax so the concatenated sources share one Node scope.
    lines = [l for l in src.splitlines() if not l.startswith("import ")]
    src = "\n".join(lines)
    src = src.replace("export function ", "function ")
    src = src.replace("export const ", "const ")
    return src


_ALL_JS = _load("utils/geo.js") + "\n" + _load("utils/exclusionDraw.js")


def _run(script):
    result = run_node(_ALL_JS + "\n" + script, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error: {result.stderr}")
    return json.loads(result.stdout.strip())


def _click(state, latlon):
    return _run(
        f"console.log(JSON.stringify(nextExclusionClickState("
        f"{json.dumps(state)}, {json.dumps(latlon)})));"
    )


def _clicks(points):
    """Apply a sequence of clicks starting from the empty state."""
    return _run(
        f"let s = {{draft: [], rings: [], activeRing: null}};"
        f"for (const p of {json.dumps(points)}) s = nextExclusionClickState(s, p);"
        f"console.log(JSON.stringify(s));"
    )


P1 = {"lat": 0, "lon": 0}
P2 = {"lat": 0, "lon": 10}
P3 = {"lat": 10, "lon": 10}


class TestExclusionClickReducer(unittest.TestCase):

    def test_first_two_clicks_stay_in_draft(self):
        s = _clicks([P1, P2])
        self.assertEqual(len(s["draft"]), 2)
        self.assertEqual(s["rings"], [])
        self.assertIsNone(s["activeRing"])

    def test_third_click_closes_the_ring(self):
        """Same logic as the zone: 3 points -> the polygon is closed NOW."""
        s = _clicks([P1, P2, P3])
        self.assertEqual(s["draft"], [])
        self.assertEqual(len(s["rings"]), 1)
        self.assertEqual(len(s["rings"][0]), 3)
        self.assertEqual(s["activeRing"], 0)

    def test_fourth_click_inserts_at_nearest_edge(self):
        """Clicks on a closed ring insert into the closest edge (zone logic)."""
        # Click near the bottom edge (P1->P2) of the triangle.
        s = _clicks([P1, P2, P3, {"lat": -1, "lon": 5}])
        self.assertEqual(len(s["rings"][0]), 4)
        # Inserted between P1 and P2 (index 1).
        self.assertEqual(s["rings"][0][1], {"lat": -1, "lon": 5})

    def test_coincident_click_ignored_in_draft(self):
        """Double-click's second hit on the same spot adds nothing."""
        s = _clicks([P1, P1])
        self.assertEqual(len(s["draft"]), 1)

    def test_coincident_click_ignored_on_ring(self):
        s = _clicks([P1, P2, P3, P3])
        self.assertEqual(len(s["rings"][0]), 3)

    def test_null_latlon_is_noop(self):
        s = _click({"draft": [P1], "rings": [], "activeRing": None}, None)
        self.assertEqual(len(s["draft"]), 1)

    def test_second_ring_after_finish(self):
        """After ending the active ring, three more clicks close a new one."""
        s = _run(
            f"let s = {{draft: [], rings: [], activeRing: null}};"
            f"for (const p of {json.dumps([P1, P2, P3])}) s = nextExclusionClickState(s, p);"
            f"s = {{...s, activeRing: null}};"  # double-click ends editing
            f"const more = [{{lat: 20, lon: 20}}, {{lat: 20, lon: 30}}, {{lat: 30, lon: 30}}];"
            f"for (const p of more) s = nextExclusionClickState(s, p);"
            f"console.log(JSON.stringify(s));"
        )
        self.assertEqual(len(s["rings"]), 2)
        self.assertEqual(s["activeRing"], 1)

    def test_input_state_not_mutated(self):
        """The reducer is pure — the input ring must not change."""
        out = _run(
            f"const rings = [[{json.dumps(P1)}, {json.dumps(P2)}, {json.dumps(P3)}]];"
            f"const s = {{draft: [], rings, activeRing: 0}};"
            f"nextExclusionClickState(s, {{lat: -1, lon: 5}});"
            f"console.log(JSON.stringify(rings[0].length));"
        )
        self.assertEqual(out, 3)

    def test_active_ring_insert_reports_inserted(self):
        """An active-ring insert reports {ring, index} so a finishing
        double-click can revoke its own first click."""
        s = _clicks([P1, P2, P3, {"lat": -1, "lon": 5}])
        self.assertEqual(s["inserted"], {"ring": 0, "index": 1})

    def test_closing_click_not_reported_as_inserted(self):
        """The ring-closing 3rd vertex is the intended corner — not revocable."""
        s = _clicks([P1, P2, P3])
        self.assertIsNone(s["inserted"])

    def test_coincident_click_is_reference_noop(self):
        """A coincident (double-click second-hit) click must return the SAME
        draft/rings references — the hook preserves the pending revocation
        record only when it can detect the click as a no-op by identity.
        (A real double-click is click#1 insert → click#2 coincident → finish;
        losing the record on click#2 broke revocation in live testing.)"""
        out = _run(
            f"const rings = [[{json.dumps(P1)}, {json.dumps(P2)}, {json.dumps(P3)}]];"
            f"const draft = [];"
            f"const s = {{draft, rings, activeRing: 0}};"
            f"const n = nextExclusionClickState(s, {json.dumps(P3)});"  # coincident with ring vertex
            f"console.log(JSON.stringify({{sameRings: n.rings === rings, sameDraft: n.draft === draft, inserted: n.inserted}}));"
        )
        self.assertTrue(out["sameRings"])
        self.assertTrue(out["sameDraft"])
        self.assertIsNone(out["inserted"])


class TestRevokeInsertedVertex(unittest.TestCase):
    """Undo the stray vertex a finishing double-click's first click inserted."""

    def _revoke(self, rings, inserted):
        return _run(
            f"console.log(JSON.stringify(revokeInsertedVertex("
            f"{json.dumps(rings)}, {json.dumps(inserted)})));"
        )

    RING4 = [P1, {"lat": -1, "lon": 5}, P2, P3]  # vertex 1 was just inserted

    def test_removes_matching_vertex(self):
        out = self._revoke([self.RING4], {"ring": 0, "index": 1, "latlon": {"lat": -1, "lon": 5}})
        self.assertEqual(out, [[P1, P2, P3]])

    def test_position_mismatch_is_noop(self):
        """A concurrent edit moved things — never delete the wrong vertex."""
        out = self._revoke([self.RING4], {"ring": 0, "index": 1, "latlon": {"lat": 9, "lon": 9}})
        self.assertEqual(out, [self.RING4])

    def test_never_shrinks_below_three(self):
        tri = [P1, P2, P3]
        out = self._revoke([tri], {"ring": 0, "index": 1, "latlon": P2})
        self.assertEqual(out, [tri])

    def test_missing_ring_is_noop(self):
        out = self._revoke([self.RING4], {"ring": 5, "index": 1, "latlon": {"lat": -1, "lon": 5}})
        self.assertEqual(out, [self.RING4])

    def test_null_inserted_is_noop(self):
        out = self._revoke([self.RING4], None)
        self.assertEqual(out, [self.RING4])


class TestActiveRingAfterRemove(unittest.TestCase):

    def _r(self, active, removed):
        return _run(f"console.log(JSON.stringify(activeRingAfterRemove({json.dumps(active)}, {removed})));")

    def test_removing_active_ends_editing(self):
        self.assertIsNone(self._r(1, 1))

    def test_removing_earlier_shifts_index(self):
        self.assertEqual(self._r(2, 0), 1)

    def test_removing_later_keeps_index(self):
        self.assertEqual(self._r(0, 2), 0)

    def test_no_active_stays_none(self):
        self.assertIsNone(self._r(None, 0))


if __name__ == "__main__":
    unittest.main()
