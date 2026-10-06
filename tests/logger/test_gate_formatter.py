"""Step 3: gate result formatting tests."""
from __future__ import annotations

import unittest

from navpy.logger.gate_formatter import (
    format_gate_result,
    format_gate_results_list,
    sanitize,
)
from navpy.modules.navigation.gates.observable_gate import GateResult, GateStatus


class SanitizeTests(unittest.TestCase):
    def test_sanitizes_separators(self):
        self.assertEqual(sanitize("a;b=c,d"), "a_b_c_d")

    def test_sanitizes_crlf(self):
        self.assertEqual(sanitize("a\r\nb"), "a__b")

    def test_empty_returns_empty(self):
        self.assertEqual(sanitize(""), "")

    def test_clean_string_unchanged(self):
        self.assertEqual(sanitize("NOT_READY"), "NOT_READY")


class FormatGateResultTests(unittest.TestCase):
    def test_pass(self):
        r = GateResult(status=GateStatus.PASS)
        self.assertEqual(format_gate_result(r), "PASS")

    def test_block(self):
        r = GateResult(status=GateStatus.BLOCK, reason="NOT_READY")
        self.assertEqual(format_gate_result(r), "BLOCK:NOT_READY")

    def test_invalid(self):
        r = GateResult(status=GateStatus.INVALID, reason="NO_SIGNAL")
        self.assertEqual(format_gate_result(r), "INVALID:NO_SIGNAL")

    def test_reason_with_separators_sanitized(self):
        r = GateResult(
            status=GateStatus.BLOCK, reason="a;b=c,d\r\nnext"
        )
        self.assertEqual(format_gate_result(r), "BLOCK:a_b_c_d__next")


class FormatGateResultsListTests(unittest.TestCase):
    def test_mixed_statuses(self):
        results = [
            ("gate_a", GateResult(status=GateStatus.PASS)),
            ("gate_b", GateResult(status=GateStatus.BLOCK, reason="X")),
            ("gate_c", GateResult(status=GateStatus.INVALID, reason="Y")),
        ]
        out = format_gate_results_list(results)
        self.assertEqual(out, "gate_a=PASS;gate_b=BLOCK:X;gate_c=INVALID:Y")

    def test_empty_list_returns_empty(self):
        self.assertEqual(format_gate_results_list([]), "")

    def test_order_preserved(self):
        results = [
            ("z", GateResult(status=GateStatus.PASS)),
            ("a", GateResult(status=GateStatus.PASS)),
        ]
        self.assertEqual(format_gate_results_list(results), "z=PASS;a=PASS")

    def test_names_sanitized(self):
        results = [
            ("na;me=1", GateResult(status=GateStatus.PASS)),
            ("b,\r\nad", GateResult(status=GateStatus.BLOCK, reason="R")),
        ]
        self.assertEqual(
            format_gate_results_list(results),
            "na_me_1=PASS;b___ad=BLOCK:R",
        )


if __name__ == "__main__":
    unittest.main()
