"""Tests for connection device defaults used by the GCS connection panel."""
import json
import os
import re
import unittest

from tests.gcs.js_runner import run_node


_UTIL_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "connectionDevice.js",
))

_PANEL_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "ConnectionPanel.jsx",
))


def _strip_es_modules(src):
    out = []
    for line in src.split("\n"):
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_UTIL_CODE = _strip_es_modules(open(_UTIL_FILE, encoding="utf-8").read())


def _run_js(script):
    result = run_node(_UTIL_CODE + "\n" + script, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestConnectionDeviceDefaults(unittest.TestCase):
    def test_factory_default_is_gcs_monitor_port(self):
        result = _run_js("""
            console.log(JSON.stringify({ device: DEFAULT_GCS_DEVICE }));
        """)
        self.assertEqual(result["device"], "udp:0.0.0.0:15550")

    def test_empty_panel_state_adopts_backend_default(self):
        result = _run_js("""
            console.log(JSON.stringify({
                device: resolvePanelDevice("", "udp:0.0.0.0:15552", false),
            }));
        """)
        self.assertEqual(result["device"], "udp:0.0.0.0:15552")

    def test_legacy_mission_planner_device_is_replaced(self):
        result = _run_js("""
            console.log(JSON.stringify({
                device: resolvePanelDevice(
                    "udp:0.0.0.0:14550",
                    "udp:0.0.0.0:15552",
                    false
                ),
            }));
        """)
        self.assertEqual(result["device"], "udp:0.0.0.0:15552")

    def test_user_edited_device_is_preserved(self):
        result = _run_js("""
            console.log(JSON.stringify({
                device: resolvePanelDevice(
                    "udp:127.0.0.1:16660",
                    "udp:0.0.0.0:15552",
                    true
                ),
            }));
        """)
        self.assertEqual(result["device"], "udp:127.0.0.1:16660")


class TestConnectionPanelSource(unittest.TestCase):
    def setUp(self):
        with open(_PANEL_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_panel_does_not_seed_mission_planner_port(self):
        self.assertNotIn("udp:0.0.0.0:14550", self.source)

    def test_sidebar_auto_scan_waits_for_device_update(self):
        self.assertIn("[autoScan, device]", self.source)


if __name__ == "__main__":
    unittest.main()
