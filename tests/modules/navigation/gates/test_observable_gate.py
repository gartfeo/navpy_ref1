"""Step 2: ObservableGate contract tests.

Covers GateResult invariants, DwellGate timing (with an injected FakeClock),
AllGate composite behavior (first-blocking, all-children-evaluated), and
construction-time rejection of invalid configurations.
"""
from __future__ import annotations

import unittest
from typing import List, Optional

from navpy.modules.navigation.gates.observable_gate import (
    AllGate,
    DwellGate,
    GateResult,
    GateStatus,
    ObservableGate,
)


class FakeClock:
    """Deterministic monotonic clock for dwell tests."""

    def __init__(self, start: float = 0.0):
        self._t = start

    def now(self) -> float:
        return self._t

    def advance(self, dt: float) -> None:
        self._t += dt


class StubGate(ObservableGate[object]):
    """Gate that returns a scripted sequence of GateResults.

    After the sequence is exhausted the last result is repeated.
    reset_calls tracks reset() invocations for composite-reset tests.
    """

    def __init__(self, results: List[GateResult]):
        if not results:
            raise ValueError("StubGate requires at least one scripted result")
        self._results = results
        self._idx = 0
        self.reset_calls = 0

    def evaluate(self, ctx: object) -> GateResult:
        result = self._results[self._idx]
        if self._idx < len(self._results) - 1:
            self._idx += 1
        return result

    def reset(self) -> None:
        self._idx = 0
        self.reset_calls += 1


def _pass() -> GateResult:
    return GateResult(status=GateStatus.PASS)


def _block(reason: str = "NOT_READY") -> GateResult:
    return GateResult(status=GateStatus.BLOCK, reason=reason)


def _invalid(reason: str = "NO_SIGNAL") -> GateResult:
    return GateResult(status=GateStatus.INVALID, reason=reason)


class GateResultInvariantTests(unittest.TestCase):
    def test_pass_allows_no_reason(self):
        r = GateResult(status=GateStatus.PASS)
        self.assertTrue(r.passed)
        self.assertIsNone(r.reason)

    def test_pass_rejects_reason(self):
        with self.assertRaises(ValueError):
            GateResult(status=GateStatus.PASS, reason="anything")

    def test_block_requires_reason(self):
        with self.assertRaises(ValueError):
            GateResult(status=GateStatus.BLOCK)

    def test_invalid_requires_reason(self):
        with self.assertRaises(ValueError):
            GateResult(status=GateStatus.INVALID)

    def test_block_with_reason_ok(self):
        r = GateResult(status=GateStatus.BLOCK, reason="X")
        self.assertFalse(r.passed)
        self.assertEqual(r.reason, "X")

    def test_details_allowed_for_any_status(self):
        for status, reason in (
            (GateStatus.PASS, None),
            (GateStatus.BLOCK, "X"),
            (GateStatus.INVALID, "Y"),
        ):
            r = GateResult(status=status, reason=reason, details={"k": 1})
            self.assertEqual(r.details["k"], 1)

    def test_passed_property_tracks_status(self):
        self.assertTrue(GateResult(status=GateStatus.PASS).passed)
        self.assertFalse(GateResult(status=GateStatus.BLOCK, reason="X").passed)
        self.assertFalse(GateResult(status=GateStatus.INVALID, reason="X").passed)


class DwellGateConstructionTests(unittest.TestCase):
    def test_negative_dwell_rejected(self):
        with self.assertRaises(ValueError):
            DwellGate(StubGate([_pass()]), dwell_s=-0.1)

    def test_zero_dwell_allowed(self):
        # Construction must succeed; behavior verified separately.
        DwellGate(StubGate([_pass()]), dwell_s=0.0)


class DwellGateTimingTests(unittest.TestCase):
    def test_zero_dwell_passes_immediately(self):
        clock = FakeClock()
        gate = DwellGate(StubGate([_pass()]), dwell_s=0.0, clock=clock.now)
        result = gate.evaluate(object())
        self.assertEqual(result.status, GateStatus.PASS)
        self.assertEqual(result.details["elapsed_s"], 0.0)
        self.assertEqual(result.details["required_s"], 0.0)

    def test_blocks_until_dwell_elapsed(self):
        clock = FakeClock()
        gate = DwellGate(StubGate([_pass()]), dwell_s=0.4, clock=clock.now)

        # t = 0.0: first PASS seen, dwell timer starts, elapsed=0 < 0.4 -> BLOCK
        r0 = gate.evaluate(object())
        self.assertEqual(r0.status, GateStatus.BLOCK)
        self.assertEqual(r0.reason, "DWELL_NOT_MET")
        self.assertEqual(r0.details["elapsed_s"], 0.0)
        self.assertEqual(r0.details["required_s"], 0.4)

        clock.advance(0.2)
        r1 = gate.evaluate(object())
        self.assertEqual(r1.status, GateStatus.BLOCK)
        self.assertAlmostEqual(r1.details["elapsed_s"], 0.2)

        clock.advance(0.2)  # total 0.4
        r2 = gate.evaluate(object())
        self.assertEqual(r2.status, GateStatus.PASS)
        self.assertAlmostEqual(r2.details["elapsed_s"], 0.4)
        self.assertIsNone(r2.reason)

    def test_block_resets_dwell(self):
        clock = FakeClock()
        inner = StubGate([_pass(), _pass(), _block("X"), _pass(), _pass()])
        gate = DwellGate(inner, dwell_s=0.2, clock=clock.now)

        gate.evaluate(object())  # PASS, starts dwell at t=0
        clock.advance(0.1)
        gate.evaluate(object())  # PASS, elapsed=0.1 < 0.2 -> BLOCK (dwell)

        clock.advance(0.05)
        r_block = gate.evaluate(object())  # inner BLOCK, clears dwell
        self.assertEqual(r_block.status, GateStatus.BLOCK)
        self.assertEqual(r_block.reason, "X")

        # Inner back to PASS; dwell restarts from the new start time.
        clock.advance(0.0)  # t = 0.15
        r_restart = gate.evaluate(object())
        self.assertEqual(r_restart.status, GateStatus.BLOCK)  # dwell just started
        self.assertEqual(r_restart.reason, "DWELL_NOT_MET")
        self.assertAlmostEqual(r_restart.details["elapsed_s"], 0.0)

        clock.advance(0.2)
        r_pass = gate.evaluate(object())
        self.assertEqual(r_pass.status, GateStatus.PASS)

    def test_invalid_propagates_and_resets_dwell(self):
        clock = FakeClock()
        inner = StubGate([_pass(), _invalid("NO_SIG"), _pass(), _pass()])
        gate = DwellGate(inner, dwell_s=0.1, clock=clock.now)

        gate.evaluate(object())  # PASS, dwell starts at t=0
        clock.advance(0.05)

        r_invalid = gate.evaluate(object())
        self.assertEqual(r_invalid.status, GateStatus.INVALID)
        self.assertEqual(r_invalid.reason, "NO_SIG")

        # After INVALID, dwell should have been cleared: next PASS restarts.
        clock.advance(0.0)  # still t=0.05
        r_restart = gate.evaluate(object())
        self.assertEqual(r_restart.status, GateStatus.BLOCK)
        self.assertAlmostEqual(r_restart.details["elapsed_s"], 0.0)

        clock.advance(0.1)
        r_pass = gate.evaluate(object())
        self.assertEqual(r_pass.status, GateStatus.PASS)

    def test_reset_clears_dwell_and_inner(self):
        clock = FakeClock()
        inner = StubGate([_pass(), _pass(), _pass()])
        gate = DwellGate(inner, dwell_s=0.2, clock=clock.now)

        gate.evaluate(object())  # dwell starts
        clock.advance(0.1)
        gate.evaluate(object())  # still blocking

        gate.reset()
        self.assertEqual(inner.reset_calls, 1)

        # After reset, dwell must restart from the next PASS.
        # clock is still at 0.1; inner script produces another PASS.
        r = gate.evaluate(object())
        self.assertEqual(r.status, GateStatus.BLOCK)
        self.assertAlmostEqual(r.details["elapsed_s"], 0.0)

        clock.advance(0.2)
        r2 = gate.evaluate(object())
        self.assertEqual(r2.status, GateStatus.PASS)


class AllGateConstructionTests(unittest.TestCase):
    def test_empty_gate_list_rejected(self):
        with self.assertRaises(ValueError):
            AllGate([])

    def test_duplicate_names_rejected(self):
        g1 = StubGate([_pass()])
        g2 = StubGate([_pass()])
        with self.assertRaises(ValueError):
            AllGate([("same", g1), ("same", g2)])


class AllGateBehaviorTests(unittest.TestCase):
    def test_all_pass(self):
        g1 = StubGate([_pass()])
        g2 = StubGate([_pass()])
        composite = AllGate([("a", g1), ("b", g2)])
        r = composite.evaluate(object())
        self.assertEqual(r.status, GateStatus.PASS)
        self.assertIsNone(r.details["first_blocking"])
        names = [n for n, _ in r.details["gate_results"]]
        self.assertEqual(names, ["a", "b"])

    def test_first_block_reported(self):
        g1 = StubGate([_pass()])
        g2 = StubGate([_block("B2")])
        g3 = StubGate([_block("B3")])
        composite = AllGate([("a", g1), ("b", g2), ("c", g3)])
        r = composite.evaluate(object())
        self.assertEqual(r.status, GateStatus.BLOCK)
        self.assertEqual(r.reason, "b:B2")
        self.assertEqual(r.details["first_blocking"], "b")

    def test_invalid_reported(self):
        g1 = StubGate([_pass()])
        g2 = StubGate([_invalid("NO_SIG")])
        composite = AllGate([("a", g1), ("b", g2)])
        r = composite.evaluate(object())
        self.assertEqual(r.status, GateStatus.INVALID)
        self.assertEqual(r.reason, "b:NO_SIG")
        self.assertEqual(r.details["first_blocking"], "b")

    def test_block_before_invalid_wins(self):
        g1 = StubGate([_block("EARLY")])
        g2 = StubGate([_invalid("LATE")])
        composite = AllGate([("a", g1), ("b", g2)])
        r = composite.evaluate(object())
        self.assertEqual(r.status, GateStatus.BLOCK)
        self.assertEqual(r.details["first_blocking"], "a")
        # Both children were still evaluated.
        status_by_name = dict(r.details["gate_results"])
        self.assertEqual(status_by_name["a"].status, GateStatus.BLOCK)
        self.assertEqual(status_by_name["b"].status, GateStatus.INVALID)

    def test_all_children_evaluated_each_call(self):
        """Composite must evaluate every child per call so nested
        DwellGates tick even after an earlier failure."""
        clock = FakeClock()
        blocker = StubGate([_block("B")])
        inner_pass = StubGate([_pass(), _pass(), _pass(), _pass()])
        dwell = DwellGate(inner_pass, dwell_s=0.2, clock=clock.now)
        composite = AllGate([("blocker", blocker), ("dwell", dwell)])

        # Tick 1: blocker fails, but dwell must still be ticked.
        composite.evaluate(object())
        # Tick 2 after clock advance past dwell window: dwell should be PASS
        # standalone, though composite still blocks because of `blocker`.
        clock.advance(0.3)
        r = composite.evaluate(object())
        self.assertEqual(r.status, GateStatus.BLOCK)
        status_by_name = dict(r.details["gate_results"])
        self.assertEqual(status_by_name["blocker"].status, GateStatus.BLOCK)
        self.assertEqual(status_by_name["dwell"].status, GateStatus.PASS)

    def test_reset_resets_all_children(self):
        g1 = StubGate([_pass()])
        g2 = StubGate([_pass()])
        composite = AllGate([("a", g1), ("b", g2)])
        composite.reset()
        self.assertEqual(g1.reset_calls, 1)
        self.assertEqual(g2.reset_calls, 1)


class NestedCompositeTests(unittest.TestCase):
    def test_nested_all_gate_with_dwells(self):
        """An AllGate of two DwellGates should pass only when both dwells
        have been met continuously."""
        clock = FakeClock()
        dwell_a = DwellGate(StubGate([_pass()]), dwell_s=0.1, clock=clock.now)
        dwell_b = DwellGate(StubGate([_pass()]), dwell_s=0.3, clock=clock.now)
        composite = AllGate([("a", dwell_a), ("b", dwell_b)])

        # t=0: both dwells block
        r0 = composite.evaluate(object())
        self.assertEqual(r0.status, GateStatus.BLOCK)
        self.assertEqual(r0.details["first_blocking"], "a")

        clock.advance(0.1)
        r1 = composite.evaluate(object())
        self.assertEqual(r1.status, GateStatus.BLOCK)
        # a has now passed; b still blocking
        self.assertEqual(r1.details["first_blocking"], "b")

        clock.advance(0.2)  # total 0.3
        r2 = composite.evaluate(object())
        self.assertEqual(r2.status, GateStatus.PASS)


if __name__ == "__main__":
    unittest.main()
