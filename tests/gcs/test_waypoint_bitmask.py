"""Tests for waypoint bitmask logic used in SettingsModal MiniWaypointPath."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re

# Read the bitmask utility and strip ES module syntax
_BITMASK_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "bitmask.js",
))
_BITMASK_SRC = re.sub(r"^export\s+", "", open(_BITMASK_PATH, encoding="utf-8").read(), flags=re.MULTILINE)

# Build a Node.js snippet with the bitmask functions + test helpers
_JS_FUNCS = "\n".join([
    _BITMASK_SRC,
    # Toggle logic (XOR bit)
    "function toggleBit(mask, idx) { return mask ^ (1 << idx); }",
    # Cleanup logic: clear bits below navLastWp
    "function cleanupMask(mask, navLastWp) {"
    "  if (navLastWp > 0 && mask > 0) {"
    "    var clearBits = (1 << navLastWp) - 1;"
    "    return mask & ~clearBits;"
    "  }"
    "  return mask;"
    "}",
])


def _run_node(expr):
    """Evaluate a JS expression and return the result."""
    script = _JS_FUNCS + f"\nconsole.log(JSON.stringify({expr}));"
    result = run_node(script, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error: {result.stderr}")
    return json.loads(result.stdout.strip())


class TestDecodeBitmask(unittest.TestCase):
    def test_zero(self):
        self.assertEqual(_run_node("decodeBitmask(0)"), "")

    def test_single_bit(self):
        self.assertEqual(_run_node("decodeBitmask(4)"), "3")  # bit 2 → WP 3

    def test_multiple_bits(self):
        # bits 0,2,3 = 1+4+8 = 13 → WPs 1,3,4
        self.assertEqual(_run_node("decodeBitmask(13)"), "1,3,4")

    def test_string_input(self):
        self.assertEqual(_run_node("decodeBitmask('5')"), "1,3")  # bits 0,2 → WPs 1,3

    def test_nan_input(self):
        self.assertEqual(_run_node("decodeBitmask('abc')"), "")


class TestEncodeBitmask(unittest.TestCase):
    def test_empty(self):
        self.assertEqual(_run_node("encodeBitmask('')"), 0)

    def test_single(self):
        self.assertEqual(_run_node("encodeBitmask('3')"), 4)  # WP 3 → bit 2 → 1<<2

    def test_multiple(self):
        self.assertEqual(_run_node("encodeBitmask('1,3,4')"), 13)  # WPs 1,3,4 → bits 0,2,3 → 1+4+8

    def test_roundtrip(self):
        self.assertEqual(_run_node("encodeBitmask(decodeBitmask(42))"), 42)

    def test_trailing_comma(self):
        """Trailing comma should not break encoding — just ignores the empty part."""
        self.assertEqual(_run_node("encodeBitmask('3,')"), 4)  # Same as '3'

    def test_middle_comma_spacing(self):
        self.assertEqual(_run_node("encodeBitmask('3, ,5')"), 20)  # WPs 3,5 → bits 2,4

    def test_out_of_range_ignored(self):
        self.assertEqual(_run_node("encodeBitmask('25')"), 0)  # >24 ignored

    def test_negative_ignored(self):
        self.assertEqual(_run_node("encodeBitmask('-1,2')"), 2)  # -1 ignored, WP 2 → bit 1


class TestToggleBit(unittest.TestCase):
    def test_toggle_on(self):
        self.assertEqual(_run_node("toggleBit(0, 3)"), 8)

    def test_toggle_off(self):
        self.assertEqual(_run_node("toggleBit(8, 3)"), 0)

    def test_toggle_preserves_others(self):
        # 13 = bits 0,2,3; toggle bit 1 → 15 = bits 0,1,2,3
        self.assertEqual(_run_node("toggleBit(13, 1)"), 15)

    def test_toggle_twice_is_identity(self):
        self.assertEqual(_run_node("toggleBit(toggleBit(42, 5), 5)"), 42)


class TestCleanupMask(unittest.TestCase):
    def test_no_cleanup_when_last_wp_zero(self):
        self.assertEqual(_run_node("cleanupMask(15, 0)"), 15)

    def test_clears_bits_below(self):
        # mask=15 (bits 0,1,2,3), navLastWp=2 → clear bits 0,1 → 12 (bits 2,3)
        self.assertEqual(_run_node("cleanupMask(15, 2)"), 12)

    def test_no_change_when_all_above(self):
        # mask=12 (bits 2,3), navLastWp=2 → nothing to clear
        self.assertEqual(_run_node("cleanupMask(12, 2)"), 12)

    def test_clears_all_when_last_wp_high(self):
        # mask=7 (bits 0,1,2), navLastWp=5 → clear bits 0..4 → 0
        self.assertEqual(_run_node("cleanupMask(7, 5)"), 0)

    def test_zero_mask_unchanged(self):
        self.assertEqual(_run_node("cleanupMask(0, 3)"), 0)


if __name__ == "__main__":
    unittest.main()
