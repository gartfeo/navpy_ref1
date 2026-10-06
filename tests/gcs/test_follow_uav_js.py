"""Tests for resolveFollowTarget in useFollowUav.js."""
import unittest
from tests.gcs.js_runner import run_node
import os
import re


_HOOKS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "hooks",
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
        if re.match(r"^export\s+default\s+function\s", stripped):
            line = re.sub(r"^export\s+default\s+(function)\s", r"\1 ", line)
            out.append(line)
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_JS_FILE = os.path.join(_HOOKS_DIR, "useFollowUav.js")
_SRC = _strip_es_modules(
    open(_JS_FILE, encoding="utf-8").read()
)


def _run_js(script):
    code = _SRC + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestResolveFollowTarget(unittest.TestCase):
    def test_null_follow_returns_null(self):
        out = _run_js(
            "console.log(resolveFollowTarget(null, [{sys_id:1},{sys_id:2}]));"
        )
        self.assertEqual(out, "null")

    def test_undefined_follow_returns_null(self):
        out = _run_js(
            "console.log(resolveFollowTarget(undefined, [{sys_id:1}]));"
        )
        self.assertEqual(out, "null")

    def test_vehicle_exists_returns_sys_id(self):
        out = _run_js(
            "console.log(resolveFollowTarget(1, [{sys_id:1},{sys_id:2}]));"
        )
        self.assertEqual(out, "1")

    def test_vehicle_disconnected_returns_null(self):
        out = _run_js(
            "console.log(resolveFollowTarget(3, [{sys_id:1},{sys_id:2}]));"
        )
        self.assertEqual(out, "null")

    def test_empty_vehicle_list_returns_null(self):
        out = _run_js(
            "console.log(resolveFollowTarget(1, []));"
        )
        self.assertEqual(out, "null")

    def test_null_vehicle_list_returns_null(self):
        out = _run_js(
            "console.log(resolveFollowTarget(1, null));"
        )
        self.assertEqual(out, "null")


if __name__ == "__main__":
    unittest.main()
