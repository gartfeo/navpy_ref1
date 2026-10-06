"""Tests for disconnect zone cleanup: Node.js logic + source-level wiring checks."""
import os
import subprocess
import unittest

_HOOK_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks",
    "useVehicleConnection.js",
))

_PLAN_DOWNLOAD_RUN_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "planDownloadRun.js",
))

_UPLOAD_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks",
    "useMissionUpload.js",
))

_SIDEBAR_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "sidebar",
    "MonitoringSidebar.jsx",
))

_NODE_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "frontend", "test_disconnect_zone_cleanup_logic.js",
))


class TestDisconnectZoneCleanupLogic(unittest.TestCase):
    """Run the Node.js test for the zone-filtering logic."""

    def test_node_logic(self):
        result = subprocess.run(
            ["node", _NODE_TEST],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, f"Node test failed:\n{result.stderr}")


class TestDisconnectHandlerRobustness(unittest.TestCase):
    """Verify useVehicleConnection has the fallback for zones without sys_id."""

    def setUp(self):
        with open(_HOOK_FILE, encoding="utf-8") as f:
            self.source = f.read()
        with open(_PLAN_DOWNLOAD_RUN_FILE, encoding="utf-8") as f:
            self.plan_download_source = f.read()

    def test_fallback_count_based_trim(self):
        """handleDisconnect should fall back to count-based removal."""
        self.assertIn("shouldFallbackDisconnectZoneRemoval(", self.source)
        self.assertIn("remaining.length === oldZones.length", self.plan_download_source)
        self.assertIn("!planBelongsToRun(plan, run)", self.plan_download_source)

    def test_removes_by_sys_id(self):
        self.assertIn("removedSet.has(z.sys_id)", self.source)

    def test_clears_plan_when_empty(self):
        self.assertIn("setPlan(null)", self.source)


class TestUploadStampsSysId(unittest.TestCase):
    """Verify useMissionUpload stamps sys_id on zones during upload."""

    def setUp(self):
        with open(_UPLOAD_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_sys_id_assigned_in_zones_with_alt(self):
        """zonesWithAlt should include sys_id from vehicleList."""
        self.assertIn("sys_id: vehicleList[i]?.sys_id", self.source)


class TestConnectMarksBusyImmediately(unittest.TestCase):
    """Verify handleConnect marks sysId as downloading before the connect call."""

    def setUp(self):
        with open(_HOOK_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_add_downloading_before_connect(self):
        """addDownloading should appear before api.connectVehicle in handleConnect."""
        # Find handleConnect body
        start = self.source.index("const handleConnect")
        body = self.source[start:start + 600]
        add_idx = body.index("addDownloading([sysId], connectOwner)")
        connect_idx = body.index("api.connectVehicle")
        self.assertLess(add_idx, connect_idx,
                        "addDownloading must be called before api.connectVehicle")


class TestDownloadButtonShowsProgress(unittest.TestCase):
    """Verify MonitoringSidebar Download Plan button shows busy state."""

    def setUp(self):
        with open(_SIDEBAR_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_accepts_downloading_sys_ids_prop(self):
        self.assertIn("downloadingSysIds", self.source)

    def test_button_shows_downloading_text(self):
        self.assertIn("monitor.downloadingPlan", self.source)

    def test_button_shows_connecting_text(self):
        self.assertIn("monitor.connectingPlan", self.source)

    def test_button_disabled_when_busy(self):
        self.assertIn("disabled={isBusyConnecting}", self.source)


if __name__ == "__main__":
    unittest.main()
