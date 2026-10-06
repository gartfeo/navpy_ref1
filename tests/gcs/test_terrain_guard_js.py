"""Tests for terrainGuard.js — shouldApplyTerrainSample logic."""
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


_JS_FILE = os.path.join(_UTILS_DIR, "terrainGuard.js")
_TERRAIN_GUARD_JS = _strip_es_modules(
    open(_JS_FILE, encoding="utf-8").read()
)


def _run_js(script):
    code = _TERRAIN_GUARD_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestShouldApplyTerrainSample(unittest.TestCase):
    """shouldApplyTerrainSample(sampledHeights)"""

    def test_all_zeros_returns_false(self):
        """All-zero heights (terrain not loaded) should NOT overwrite."""
        out = _run_js("""
            console.log(JSON.stringify(shouldApplyTerrainSample([0, 0, 0])));
        """)
        self.assertFalse(json.loads(out))

    def test_mixed_heights_returns_true(self):
        """Mix of non-zero heights should overwrite."""
        out = _run_js("""
            console.log(JSON.stringify(shouldApplyTerrainSample([0, 150, 200])));
        """)
        self.assertTrue(json.loads(out))

    def test_all_nonzero_returns_true(self):
        """All non-zero heights should overwrite."""
        out = _run_js("""
            console.log(JSON.stringify(shouldApplyTerrainSample([100, 150, 200])));
        """)
        self.assertTrue(json.loads(out))

    def test_empty_array_returns_false(self):
        """Empty array should NOT overwrite."""
        out = _run_js("""
            console.log(JSON.stringify(shouldApplyTerrainSample([])));
        """)
        self.assertFalse(json.loads(out))

    def test_null_heights_treated_as_zero(self):
        """Null heights should be treated as zero — all-null should NOT overwrite."""
        out = _run_js("""
            console.log(JSON.stringify(shouldApplyTerrainSample([null, null])));
        """)
        self.assertFalse(json.loads(out))

    def test_undefined_heights_treated_as_zero(self):
        """Undefined heights should be treated as zero."""
        out = _run_js("""
            console.log(JSON.stringify(shouldApplyTerrainSample([undefined, undefined])));
        """)
        self.assertFalse(json.loads(out))

    def test_not_array_returns_false(self):
        """Non-array input should return false."""
        out = _run_js("""
            console.log(JSON.stringify(shouldApplyTerrainSample(null)));
        """)
        self.assertFalse(json.loads(out))

    def test_single_nonzero_returns_true(self):
        """Single non-zero height should overwrite."""
        out = _run_js("""
            console.log(JSON.stringify(shouldApplyTerrainSample([42])));
        """)
        self.assertTrue(json.loads(out))


if __name__ == "__main__":
    unittest.main()
