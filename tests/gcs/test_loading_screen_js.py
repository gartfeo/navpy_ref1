"""Tests for fadeState.js — computeFadeState pure logic."""
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
    """Remove ES module import/export syntax so Node.js can eval."""
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


_JS_FILE = os.path.join(_UTILS_DIR, "fadeState.js")
_FADE_JS = _strip_es_modules(
    open(_JS_FILE, encoding="utf-8").read()
)


def _run_js(script):
    code = _FADE_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestComputeFadeState(unittest.TestCase):
    """computeFadeState(visible, prevMounted)"""

    def test_visible_true_mounts(self):
        """visible=true should always mount and not fade."""
        out = _run_js("""
            console.log(JSON.stringify(computeFadeState(true, false)));
        """)
        result = json.loads(out)
        self.assertTrue(result["mounted"])
        self.assertFalse(result["fading"])

    def test_visible_true_already_mounted(self):
        """visible=true while already mounted stays mounted, no fade."""
        out = _run_js("""
            console.log(JSON.stringify(computeFadeState(true, true)));
        """)
        result = json.loads(out)
        self.assertTrue(result["mounted"])
        self.assertFalse(result["fading"])

    def test_visible_false_triggers_fade(self):
        """visible=false while mounted should start fading."""
        out = _run_js("""
            console.log(JSON.stringify(computeFadeState(false, true)));
        """)
        result = json.loads(out)
        self.assertTrue(result["mounted"])
        self.assertTrue(result["fading"])

    def test_visible_false_unmounted_stays_unmounted(self):
        """visible=false and already unmounted stays unmounted."""
        out = _run_js("""
            console.log(JSON.stringify(computeFadeState(false, false)));
        """)
        result = json.loads(out)
        self.assertFalse(result["mounted"])
        self.assertFalse(result["fading"])


if __name__ == "__main__":
    unittest.main()
