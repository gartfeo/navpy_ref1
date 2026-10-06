"""Tests for pendingVehicles.js.

Pending vehicles bridge the frontend gap between backend discovery/connect
returning and the first telemetry snapshot arriving over WebSocket.
"""
import json
import os
import re
import unittest

from tests.gcs.js_runner import run_node


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))

_JS_PATH = os.path.join(_UTILS_DIR, "pendingVehicles.js")

_MONITORING_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "sidebar", "MonitoringSidebar.jsx",
))

_APP_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "App.jsx",
))

_CONNECTION_PANEL_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "ConnectionPanel.jsx",
))


def _strip_es_modules(src):
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


_JS_CODE = _strip_es_modules(open(_JS_PATH, encoding="utf-8").read())


def _run_js(snippet):
    result = run_node(_JS_CODE + "\n" + snippet, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestPendingVehicles(unittest.TestCase):
    def test_discovery_result_becomes_connecting_placeholder(self):
        result = _run_js("""
        const pending = pendingVehicleFromDiscovery({ sys_id: 2, name: 's1-u2' });
        console.log(JSON.stringify(pending));
        """)
        self.assertEqual(result["sys_id"], 2)
        self.assertEqual(result["name"], "s1-u2")
        self.assertTrue(result["pending"])
        self.assertFalse(result["link_ok"])
        self.assertFalse(result["armed"])

    def test_upsert_replaces_by_sys_id_and_sorts(self):
        result = _run_js("""
        const pending = upsertPendingVehicles(
          [{ sys_id: 3, name: 'old' }],
          [{ sys_id: 2, name: 's1-u2' }, { sys_id: 3, name: 's1-u3' }],
        );
        console.log(JSON.stringify(pending.map((v) => [v.sys_id, v.name])));
        """)
        self.assertEqual(result, [[2, "s1-u2"], [3, "s1-u3"]])

    def test_prune_removes_live_and_removed_ids(self):
        result = _run_js("""
        const pending = [
          { sys_id: 1, pending: true },
          { sys_id: 2, pending: true },
          { sys_id: 3, pending: true },
        ];
        const pruned = prunePendingVehicles(pending, [{ sys_id: 2 }], [3]);
        console.log(JSON.stringify(pruned.map((v) => v.sys_id)));
        """)
        self.assertEqual(result, [1])

    def test_merge_appends_only_non_live_pending(self):
        result = _run_js("""
        const live = [{ sys_id: 1, name: 'live-1' }];
        const pending = [{ sys_id: 1, name: 'pending-1' }, { sys_id: 2, name: 'pending-2' }];
        const merged = mergePendingVehicles(live, pending);
        console.log(JSON.stringify(merged.map((v) => [v.sys_id, v.name])));
        """)
        self.assertEqual(result, [[1, "live-1"], [2, "pending-2"]])


class TestMonitoringSidebarPendingLaunchGate(unittest.TestCase):
    def setUp(self):
        with open(_MONITORING_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_pending_cards_block_fleet_start(self):
        self.assertIn(
            "const hasPendingStatusVehicles = statusVehicleList.some((v) => v.pending);",
            self.source,
        )
        self.assertIn("&& !hasPendingStatusVehicles", self.source)
        self.assertIn("disabled={!canContainerLaunch || !launcher.launcherConnected || !launchRosterComplete}", self.source)
        self.assertIn("disabled={busy || !launchRosterComplete}", self.source)


class TestAvailableVehiclesFeedStatusCards(unittest.TestCase):
    def setUp(self):
        with open(_APP_FILE, encoding="utf-8") as f:
            self.app_source = f.read()
        with open(_CONNECTION_PANEL_FILE, encoding="utf-8") as f:
            self.panel_source = f.read()

    def test_connection_panel_publishes_available_rows(self):
        self.assertIn("onAvailableVehiclesChange", self.panel_source)
        self.assertIn("onAvailableVehiclesChange?.(availableFiltered)", self.panel_source)
        self.assertIn("() => onAvailableVehiclesChange?.([])", self.panel_source)
        self.assertIn("seenIds = liveSeenIds;", self.panel_source)
        self.assertNotIn("if (liveSeenIds.length > 0) seenIds = liveSeenIds;", self.panel_source)

    def test_app_merges_available_rows_into_status_pending_vehicles(self):
        self.assertIn("const [availableVehicles, setAvailableVehicles]", self.app_source)
        self.assertIn("const statusPendingVehicles = React.useMemo", self.app_source)
        self.assertIn("() => upsertPendingVehicles(pendingVehicles, availableVehicles)", self.app_source)
        self.assertIn("pendingVehicles={statusPendingVehicles}", self.app_source)

    def test_app_does_not_subscribe_to_seen_ids_for_available_rows(self):
        self.assertNotIn("useTelemetryStore(s => s.getSeenIds())", self.app_source)


if __name__ == "__main__":
    unittest.main()
