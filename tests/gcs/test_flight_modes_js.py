"""Tests for flightModes.js pure utility functions via Node.js subprocess."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))

_JS_PATH = os.path.join(_UTILS_DIR, "flightModes.js")


def _strip_es_modules(src):
    """Remove ES module import/export syntax so Node.js can eval the code."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_JS_CODE = _strip_es_modules(open(_JS_PATH, encoding="utf-8").read())


def _run_js(snippet):
    """Evaluate JS snippet with flightModes loaded, return parsed JSON."""
    full = _JS_CODE + "\n" + snippet
    result = run_node(full, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestPrimaryModes(unittest.TestCase):
    def test_primary_modes_count(self):
        r = _run_js('console.log(JSON.stringify(PRIMARY_MODES.length))')
        self.assertEqual(r, 5)

    def test_primary_modes_contains_manual(self):
        r = _run_js('console.log(JSON.stringify(PRIMARY_MODES.includes("MANUAL")))')
        self.assertTrue(r)

    def test_primary_modes_contains_auto(self):
        r = _run_js('console.log(JSON.stringify(PRIMARY_MODES.includes("AUTO")))')
        self.assertTrue(r)

    def test_primary_modes_order(self):
        r = _run_js('console.log(JSON.stringify(PRIMARY_MODES))')
        self.assertEqual(r, ["MANUAL", "FBWA", "LOITER", "RTL", "AUTO"])


class TestSecondaryModes(unittest.TestCase):
    def test_secondary_modes_count(self):
        r = _run_js('console.log(JSON.stringify(SECONDARY_MODES.length))')
        self.assertEqual(r, 5)

    def test_secondary_modes_contains_guided(self):
        r = _run_js('console.log(JSON.stringify(SECONDARY_MODES.includes("GUIDED")))')
        self.assertTrue(r)

    def test_secondary_modes_order(self):
        r = _run_js('console.log(JSON.stringify(SECONDARY_MODES))')
        self.assertEqual(r, ["GUIDED", "STABILIZE", "CIRCLE", "CRUISE", "FBWB"])


class TestAllModes(unittest.TestCase):
    def test_all_modes_is_combined(self):
        r = _run_js('console.log(JSON.stringify(ALL_MODES.length))')
        self.assertEqual(r, 10)

    def test_no_duplicates(self):
        r = _run_js('console.log(JSON.stringify(new Set(ALL_MODES).size))')
        self.assertEqual(r, 10)

    def test_primary_before_secondary(self):
        r = _run_js('console.log(JSON.stringify(ALL_MODES))')
        self.assertEqual(r[:5], ["MANUAL", "FBWA", "LOITER", "RTL", "AUTO"])
        self.assertEqual(r[5:], ["GUIDED", "STABILIZE", "CIRCLE", "CRUISE", "FBWB"])


class TestIsPrimaryMode(unittest.TestCase):
    def test_manual_is_primary(self):
        r = _run_js('console.log(JSON.stringify(isPrimaryMode("MANUAL")))')
        self.assertTrue(r)

    def test_guided_not_primary(self):
        r = _run_js('console.log(JSON.stringify(isPrimaryMode("GUIDED")))')
        self.assertFalse(r)

    def test_unknown_not_primary(self):
        r = _run_js('console.log(JSON.stringify(isPrimaryMode("QHOVER")))')
        self.assertFalse(r)


class TestIsSecondaryMode(unittest.TestCase):
    def test_guided_is_secondary(self):
        r = _run_js('console.log(JSON.stringify(isSecondaryMode("GUIDED")))')
        self.assertTrue(r)

    def test_manual_not_secondary(self):
        r = _run_js('console.log(JSON.stringify(isSecondaryMode("MANUAL")))')
        self.assertFalse(r)

    def test_unknown_not_secondary(self):
        r = _run_js('console.log(JSON.stringify(isSecondaryMode("QHOVER")))')
        self.assertFalse(r)


class TestRadioModes(unittest.TestCase):
    def test_radio_modes_count(self):
        r = _run_js('console.log(JSON.stringify(RADIO_MODES.length))')
        self.assertEqual(r, 6)

    def test_radio_modes_contents(self):
        r = _run_js('console.log(JSON.stringify(RADIO_MODES))')
        self.assertEqual(r, ["MANUAL", "FBWA", "FBWB", "STABILIZE", "GUIDED", "CRUISE"])

    def test_radio_modes_subset_of_all(self):
        r = _run_js('console.log(JSON.stringify(RADIO_MODES.every(m => ALL_MODES.includes(m))))')
        self.assertTrue(r)

    def test_safe_modes_not_in_radio(self):
        """AUTO, RTL, LOITER, CIRCLE should NOT be in RADIO_MODES."""
        for mode in ["AUTO", "RTL", "LOITER", "CIRCLE"]:
            r = _run_js(f'console.log(JSON.stringify(RADIO_MODES.includes("{mode}")))')
            self.assertFalse(r, f"{mode} should not be in RADIO_MODES")

    def test_unsafe_modes_in_radio(self):
        """MANUAL, FBWA, FBWB, STABILIZE, GUIDED, CRUISE should be in RADIO_MODES."""
        for mode in ["MANUAL", "FBWA", "FBWB", "STABILIZE", "GUIDED", "CRUISE"]:
            r = _run_js(f'console.log(JSON.stringify(RADIO_MODES.includes("{mode}")))')
            self.assertTrue(r, f"{mode} should be in RADIO_MODES")


class TestQuickModes(unittest.TestCase):
    def test_quick_modes_count(self):
        r = _run_js('console.log(JSON.stringify(QUICK_MODES.length))')
        self.assertEqual(r, 3)

    def test_quick_modes_order(self):
        r = _run_js('console.log(JSON.stringify(QUICK_MODES))')
        self.assertEqual(r, ["AUTO", "FBWA", "RTL"])

    def test_quick_modes_subset_of_all(self):
        r = _run_js('console.log(JSON.stringify(QUICK_MODES.every(m => ALL_MODES.includes(m))))')
        self.assertTrue(r)


class TestOtherModes(unittest.TestCase):
    def test_other_modes_count(self):
        r = _run_js('console.log(JSON.stringify(OTHER_MODES.length))')
        self.assertEqual(r, 7)

    def test_other_modes_order(self):
        r = _run_js('console.log(JSON.stringify(OTHER_MODES))')
        self.assertEqual(r, ["MANUAL", "LOITER", "GUIDED", "STABILIZE", "CIRCLE", "CRUISE", "FBWB"])

    def test_other_modes_subset_of_all(self):
        r = _run_js('console.log(JSON.stringify(OTHER_MODES.every(m => ALL_MODES.includes(m))))')
        self.assertTrue(r)

    def test_no_overlap_with_quick(self):
        r = _run_js('console.log(JSON.stringify(QUICK_MODES.some(m => OTHER_MODES.includes(m))))')
        self.assertFalse(r)


if __name__ == "__main__":
    unittest.main()
