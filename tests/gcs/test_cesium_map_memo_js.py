"""Tests for CesiumMap cesiumMapPropsEqual comparator."""
import unittest
from tests.gcs.js_runner import run_node
import os
import re


_MAP_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "CesiumMap.jsx",
))


def _extract_comparator(src):
    """Extract the cesiumMapPropsEqual function from the CesiumMap source."""
    lines = src.split("\n")
    out = []
    capturing = False
    brace_depth = 0
    for line in lines:
        if "function cesiumMapPropsEqual" in line:
            capturing = True
        if capturing:
            out.append(line)
            brace_depth += line.count("{") - line.count("}")
            if brace_depth == 0 and len(out) > 1:
                break
    return "\n".join(out)


_COMPARATOR_JS = _extract_comparator(
    open(_MAP_FILE, encoding="utf-8").read()
)


def _run_js(script):
    code = _COMPARATOR_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestCesiumMapPropsEqual(unittest.TestCase):
    def test_identical_props(self):
        out = _run_js("""
            const a = { x: 1, y: 'hello', z: null };
            console.log(cesiumMapPropsEqual(a, a));
        """)
        self.assertEqual(out, "true")

    def test_equal_primitives(self):
        out = _run_js("""
            const a = { x: 1, y: 'hello' };
            const b = { x: 1, y: 'hello' };
            console.log(cesiumMapPropsEqual(a, b));
        """)
        self.assertEqual(out, "true")

    def test_different_value(self):
        out = _run_js("""
            const a = { x: 1, y: 'hello' };
            const b = { x: 2, y: 'hello' };
            console.log(cesiumMapPropsEqual(a, b));
        """)
        self.assertEqual(out, "false")

    def test_different_reference(self):
        """Different object references should return false."""
        out = _run_js("""
            const arr1 = [1, 2, 3];
            const arr2 = [1, 2, 3];
            const a = { data: arr1 };
            const b = { data: arr2 };
            console.log(cesiumMapPropsEqual(a, b));
        """)
        self.assertEqual(out, "false")

    def test_same_reference(self):
        """Same object reference should return true."""
        out = _run_js("""
            const arr = [1, 2, 3];
            const a = { data: arr };
            const b = { data: arr };
            console.log(cesiumMapPropsEqual(a, b));
        """)
        self.assertEqual(out, "true")

    def test_handles_null_undefined(self):
        out = _run_js("""
            const a = { x: null, y: undefined };
            const b = { x: null, y: undefined };
            console.log(cesiumMapPropsEqual(a, b));
        """)
        self.assertEqual(out, "true")

    def test_empty_props(self):
        out = _run_js("""
            console.log(cesiumMapPropsEqual({}, {}));
        """)
        self.assertEqual(out, "true")


if __name__ == "__main__":
    unittest.main()
