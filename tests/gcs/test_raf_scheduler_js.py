"""Tests for rafScheduler.js — shared requestAnimationFrame scheduler."""
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


_RAF_FILE = os.path.join(_UTILS_DIR, "rafScheduler.js")
_RAF_JS = _strip_es_modules(
    open(_RAF_FILE, encoding="utf-8").read()
)

# Stub requestAnimationFrame/cancelAnimationFrame for Node.js
_STUBS = """\
let _rafCounter = 0;
let _rafCallbacks = {};
function requestAnimationFrame(fn) {
    const id = ++_rafCounter;
    _rafCallbacks[id] = fn;
    return id;
}
function cancelAnimationFrame(id) {
    delete _rafCallbacks[id];
}
function _flushRaf() {
    const cbs = Object.values(_rafCallbacks);
    _rafCallbacks = {};
    for (const fn of cbs) fn();
}
"""


def _run_js(script):
    code = _STUBS + "\n" + _RAF_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestScheduleRaf(unittest.TestCase):
    def test_registers_callback(self):
        out = _run_js("""
            let called = 0;
            scheduleRaf('a', () => { called++; });
            _flushRaf();
            console.log(called);
        """)
        self.assertEqual(out, "1")

    def test_multiple_callbacks(self):
        out = _run_js("""
            let a = 0, b = 0;
            scheduleRaf('a', () => { a++; });
            scheduleRaf('b', () => { b++; });
            _flushRaf();
            console.log(a, b);
        """)
        self.assertEqual(out, "1 1")

    def test_replace_callback(self):
        """Scheduling with same ID replaces the previous callback."""
        out = _run_js("""
            let first = 0, second = 0;
            scheduleRaf('x', () => { first++; });
            scheduleRaf('x', () => { second++; });
            _flushRaf();
            console.log(first, second);
        """)
        self.assertEqual(out, "0 1")


class TestCancelRaf(unittest.TestCase):
    def test_cancel_removes_callback(self):
        out = _run_js("""
            let called = 0;
            scheduleRaf('a', () => { called++; });
            cancelRaf('a');
            // No pending RAF to flush since cancelAnimationFrame was called
            console.log(called);
        """)
        self.assertEqual(out, "0")

    def test_cancel_one_keeps_others(self):
        out = _run_js("""
            let a = 0, b = 0;
            scheduleRaf('a', () => { a++; });
            scheduleRaf('b', () => { b++; });
            cancelRaf('a');
            _flushRaf();
            console.log(a, b);
        """)
        self.assertEqual(out, "0 1")


class TestLoopStops(unittest.TestCase):
    def test_no_raf_when_empty(self):
        """After cancelling all, rafId should be null."""
        out = _run_js("""
            scheduleRaf('a', () => {});
            cancelRaf('a');
            console.log(rafId);
        """)
        self.assertEqual(out, "null")


if __name__ == "__main__":
    unittest.main()
