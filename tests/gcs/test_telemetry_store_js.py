"""Tests for telemetryStore.js — plain JS pub/sub store for vehicle telemetry."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_STORE_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "stores",
    "telemetryStore.js",
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
        if re.match(r"^export\s+default\s", stripped):
            # export default telemetryStore; -> skip
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_STORE_JS = _strip_es_modules(
    open(_STORE_FILE, encoding="utf-8").read()
)

# Stub WebSocket and window/location
_STUBS = """\
class WebSocket { constructor() { this.readyState = 0; } send() {} close() {} }
WebSocket.OPEN = 1;
const window = { location: { protocol: 'http:', host: 'localhost:8000' } };
const TextDecoder = class { decode(buf) { return Buffer.from(buf).toString(); } };
"""


def _run_js(script):
    code = _STUBS + "\n" + _STORE_JS + "\n" + script
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


class TestSubscribe(unittest.TestCase):
    def test_subscribe_and_notify(self):
        out = _run_js("""
            let count = 0;
            subscribe(() => { count++; });
            _notify();
            _notify();
            console.log(count);
        """)
        self.assertEqual(out, "2")

    def test_unsubscribe(self):
        out = _run_js("""
            let count = 0;
            const unsub = subscribe(() => { count++; });
            _notify();
            unsub();
            _notify();
            console.log(count);
        """)
        self.assertEqual(out, "1")


class TestGetSnapshot(unittest.TestCase):
    def test_increments_on_notify(self):
        out = _run_js("""
            const v1 = getSnapshot();
            _notify();
            const v2 = getSnapshot();
            _notify();
            const v3 = getSnapshot();
            console.log(v2 - v1, v3 - v2);
        """)
        self.assertEqual(out, "1 1")


class TestVehicleList(unittest.TestCase):
    def test_empty_by_default(self):
        out = _run_js("""
            console.log(JSON.stringify(getVehicleList()));
        """)
        self.assertEqual(json.loads(out), [])

    def test_cached_between_calls(self):
        out = _run_js("""
            const a = getVehicleList();
            const b = getVehicleList();
            console.log(a === b);
        """)
        self.assertEqual(out, "true")

    def test_refreshed_after_notify(self):
        out = _run_js("""
            const a = getVehicleList();
            _vehicles = { 1: { sys_id: 1 } };
            _notify();
            const b = getVehicleList();
            console.log(a === b, b.length);
        """)
        self.assertEqual(out, "false 1")


class TestSysIdKey(unittest.TestCase):
    def test_empty(self):
        out = _run_js("""
            console.log(JSON.stringify(getSysIdKey()));
        """)
        self.assertEqual(json.loads(out), "")

    def test_sorted(self):
        out = _run_js("""
            _vehicles = { 3: {}, 1: {}, 2: {} };
            _notify();
            console.log(getSysIdKey());
        """)
        self.assertEqual(out, "1,2,3")

    def test_cached(self):
        out = _run_js("""
            _vehicles = { 1: {} };
            _notify();
            const a = getSysIdKey();
            const b = getSysIdKey();
            console.log(a === b);
        """)
        self.assertEqual(out, "true")


class TestRemoveVehicle(unittest.TestCase):
    def test_removes_and_notifies(self):
        out = _run_js("""
            _vehicles = { 1: { sys_id: 1 }, 2: { sys_id: 2 } };
            let notified = false;
            subscribe(() => { notified = true; });
            removeVehicle(1);
            console.log(Object.keys(getVehicles()).length, notified);
        """)
        self.assertEqual(out, "1 true")


class TestMessageHandlers(unittest.TestCase):
    def test_handlers_object_is_shared(self):
        out = _run_js("""
            const h = getMessageHandlers();
            h.test = () => 42;
            console.log(typeof getMessageHandlers().test);
        """)
        self.assertEqual(out, "function")


if __name__ == "__main__":
    unittest.main()
