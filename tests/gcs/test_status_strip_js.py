"""Tests for StatusStrip displayDuration function."""
import unittest
from tests.gcs.js_runner import run_node
import os
import re


_COMPONENT_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "hud",
    "StatusStrip.jsx",
))


def _strip_es_modules(src):
    """Remove ES module import/export and JSX so Node.js can eval the code."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        if re.match(r"^export\s+default\s", stripped):
            break  # stop before the React component
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_JS_SRC = _strip_es_modules(
    open(_COMPONENT_PATH, encoding="utf-8").read()
)


def _run_js(script):
    code = _JS_SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestDisplayDuration(unittest.TestCase):
    """displayDuration returns correct ms per severity bracket."""

    def test_emerg(self):
        self.assertEqual(_run_js("console.log(displayDuration(0));"), "15000")

    def test_alert(self):
        self.assertEqual(_run_js("console.log(displayDuration(1));"), "15000")

    def test_crit(self):
        self.assertEqual(_run_js("console.log(displayDuration(2));"), "15000")

    def test_err(self):
        self.assertEqual(_run_js("console.log(displayDuration(3));"), "10000")

    def test_warn(self):
        self.assertEqual(_run_js("console.log(displayDuration(4));"), "7000")

    def test_notice(self):
        self.assertEqual(_run_js("console.log(displayDuration(5));"), "5000")

    def test_info(self):
        self.assertEqual(_run_js("console.log(displayDuration(6));"), "5000")

    def test_debug(self):
        self.assertEqual(_run_js("console.log(displayDuration(7));"), "5000")


if __name__ == "__main__":
    unittest.main()
