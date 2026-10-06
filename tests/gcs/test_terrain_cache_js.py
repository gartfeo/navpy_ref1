"""Tests for terrainCache.js — needsTerrainSample logic."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "utils",
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


_JS_FILE = os.path.join(_UTILS_DIR, "terrainCache.js")
_TERRAIN_CACHE_JS = _strip_es_modules(
    open(_JS_FILE, encoding="utf-8").read()
)


def _run_js(script):
    code = _TERRAIN_CACHE_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestNeedsTerrainSample(unittest.TestCase):
    """needsTerrainSample(cached, lat, lon, now)"""

    def test_no_cache_entry(self):
        out = _run_js("""
            console.log(JSON.stringify(needsTerrainSample(undefined, 32.0, 34.0, 1000)));
        """)
        self.assertTrue(json.loads(out))

    def test_null_terrain_height(self):
        out = _run_js("""
            const cached = { lat: 32.0, lon: 34.0, terrainHeight: null, ts: 1000 };
            console.log(JSON.stringify(needsTerrainSample(cached, 32.0, 34.0, 1000)));
        """)
        self.assertTrue(json.loads(out))

    def test_fresh_nearby_returns_false(self):
        out = _run_js("""
            const cached = { lat: 32.0, lon: 34.0, terrainHeight: 100, ts: 5000 };
            console.log(JSON.stringify(needsTerrainSample(cached, 32.0001, 34.0001, 5100)));
        """)
        self.assertFalse(json.loads(out))

    def test_stale_cache_triggers_resample(self):
        out = _run_js("""
            const cached = { lat: 32.0, lon: 34.0, terrainHeight: 100, ts: 0 };
            // 11 seconds later (> 10_000 ms threshold)
            console.log(JSON.stringify(needsTerrainSample(cached, 32.0, 34.0, 11000)));
        """)
        self.assertTrue(json.loads(out))

    def test_large_lat_move_triggers_resample(self):
        out = _run_js("""
            const cached = { lat: 32.0, lon: 34.0, terrainHeight: 100, ts: 5000 };
            // Moved > 0.0005 degrees in latitude (~55 m)
            console.log(JSON.stringify(needsTerrainSample(cached, 32.001, 34.0, 5100)));
        """)
        self.assertTrue(json.loads(out))

    def test_large_lon_move_triggers_resample(self):
        out = _run_js("""
            const cached = { lat: 32.0, lon: 34.0, terrainHeight: 100, ts: 5000 };
            // Moved > 0.0005 degrees in longitude
            console.log(JSON.stringify(needsTerrainSample(cached, 32.0, 34.001, 5100)));
        """)
        self.assertTrue(json.loads(out))

    def test_just_under_threshold_no_resample(self):
        """Movement just under RESAMPLE_DEG should NOT trigger."""
        out = _run_js("""
            const cached = { lat: 32.0, lon: 34.0, terrainHeight: 50, ts: 5000 };
            console.log(JSON.stringify(needsTerrainSample(cached, 32.0004, 34.0004, 5100)));
        """)
        self.assertFalse(json.loads(out))

    def test_age_exactly_at_threshold_no_resample(self):
        """Age exactly at RESAMPLE_AGE_MS should NOT trigger (not strictly greater)."""
        out = _run_js("""
            const cached = { lat: 32.0, lon: 34.0, terrainHeight: 50, ts: 0 };
            console.log(JSON.stringify(needsTerrainSample(cached, 32.0, 34.0, 10000)));
        """)
        self.assertFalse(json.loads(out))


if __name__ == "__main__":
    unittest.main()
