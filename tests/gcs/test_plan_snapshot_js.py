"""Tests for planSnapshot.js — dirty detection between plan state and snapshot."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))


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


_JS_CODE = _strip_es_modules(
    open(os.path.join(_UTILS_DIR, "planSnapshot.js"), encoding="utf-8").read()
)


def _run_js(script):
    """Run a JS snippet via Node.js and return parsed JSON output."""
    full = _JS_CODE + "\n" + script
    result = run_node(full, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestIsPlanDirty(unittest.TestCase):
    """isPlanDirty correctly detects state changes vs snapshot."""

    def test_same_state_not_dirty(self):
        """Identical tracks, polygon, and search_pattern → not dirty."""
        result = _run_js("""
            const state = {
                searchPattern: "distributed",
                plan: { zones: [{ track: [{ lat: 1, lon: 2, alt: 100 }] }] },
                polygon: [{ lat: 1, lon: 2 }, { lat: 3, lon: 4 }, { lat: 5, lon: 6 }],
            };
            const snap = JSON.parse(JSON.stringify(state));
            console.log(JSON.stringify(isPlanDirty(state, snap)));
        """)
        self.assertFalse(result)

    def test_different_tracks_dirty(self):
        """Modified track coordinates → dirty."""
        result = _run_js("""
            const current = {
                searchPattern: "distributed",
                plan: { zones: [{ track: [{ lat: 1, lon: 2 }] }] },
                polygon: [{ lat: 1, lon: 2 }, { lat: 3, lon: 4 }, { lat: 5, lon: 6 }],
            };
            const snap = {
                searchPattern: "distributed",
                plan: { zones: [{ track: [{ lat: 1, lon: 99 }] }] },
                polygon: [{ lat: 1, lon: 2 }, { lat: 3, lon: 4 }, { lat: 5, lon: 6 }],
            };
            console.log(JSON.stringify(isPlanDirty(current, snap)));
        """)
        self.assertTrue(result)

    def test_different_polygon_dirty(self):
        """Modified polygon vertex → dirty."""
        result = _run_js("""
            const current = {
                searchPattern: "distributed",
                plan: { zones: [] },
                polygon: [{ lat: 1, lon: 2 }, { lat: 3, lon: 4 }, { lat: 5, lon: 6 }],
            };
            const snap = {
                searchPattern: "distributed",
                plan: { zones: [] },
                polygon: [{ lat: 1, lon: 2 }, { lat: 3, lon: 4 }, { lat: 5, lon: 99 }],
            };
            console.log(JSON.stringify(isPlanDirty(current, snap)));
        """)
        self.assertTrue(result)

    def test_different_search_pattern_dirty(self):
        """Different search_pattern string → dirty."""
        result = _run_js("""
            const current = {
                searchPattern: "corridor",
                plan: { zones: [] },
                polygon: [],
            };
            const snap = {
                searchPattern: "distributed",
                plan: { zones: [] },
                polygon: [],
            };
            console.log(JSON.stringify(isPlanDirty(current, snap)));
        """)
        self.assertTrue(result)

    def test_no_snapshot_not_dirty(self):
        """Null snapshot → not dirty (never synced)."""
        result = _run_js("""
            const current = {
                searchPattern: "distributed",
                plan: { zones: [{ track: [{ lat: 1, lon: 2 }] }] },
                polygon: [{ lat: 1, lon: 2 }],
            };
            console.log(JSON.stringify(isPlanDirty(current, null)));
        """)
        self.assertFalse(result)

    def test_alt_zero_preserved(self):
        """Track with alt: 0 is preserved in comparison (not falsy-skipped)."""
        result = _run_js("""
            const current = {
                searchPattern: "distributed",
                plan: { zones: [{ track: [{ lat: 1, lon: 2, alt: 0 }] }] },
                polygon: [],
            };
            const snap = {
                searchPattern: "distributed",
                plan: { zones: [{ track: [{ lat: 1, lon: 2, alt: 0 }] }] },
                polygon: [],
            };
            console.log(JSON.stringify(isPlanDirty(current, snap)));
        """)
        self.assertFalse(result)

    def test_alt_zero_vs_nonzero_dirty(self):
        """Track alt: 0 vs alt: 100 → dirty."""
        result = _run_js("""
            const current = {
                searchPattern: "distributed",
                plan: { zones: [{ track: [{ lat: 1, lon: 2, alt: 0 }] }] },
                polygon: [],
            };
            const snap = {
                searchPattern: "distributed",
                plan: { zones: [{ track: [{ lat: 1, lon: 2, alt: 100 }] }] },
                polygon: [],
            };
            console.log(JSON.stringify(isPlanDirty(current, snap)));
        """)
        self.assertTrue(result)

    def test_extra_zone_dirty(self):
        """More zones in current than snapshot → dirty."""
        result = _run_js("""
            const current = {
                searchPattern: "distributed",
                plan: { zones: [
                    { track: [{ lat: 1, lon: 2 }] },
                    { track: [{ lat: 3, lon: 4 }] },
                ] },
                polygon: [],
            };
            const snap = {
                searchPattern: "distributed",
                plan: { zones: [{ track: [{ lat: 1, lon: 2 }] }] },
                polygon: [],
            };
            console.log(JSON.stringify(isPlanDirty(current, snap)));
        """)
        self.assertTrue(result)


if __name__ == "__main__":
    unittest.main()
