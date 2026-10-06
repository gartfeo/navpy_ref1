"""Tests for per-waypoint upload progress: Node.js logic + source-level wiring."""
import os
import subprocess
import unittest

_NODE_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "frontend", "test_upload_progress_logic.js",
))

_APP_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "App.jsx",
))

_BOTTOM_BAR_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components", "BottomBar.jsx",
))

_MISSIONS_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "backend", "routes", "missions.py",
))

_WP_BUILDER_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "backend", "planner", "waypoint_builder.py",
))

_VEHICLE_MISSION_FACET_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "navpy", "modules", "vehicle",
    "vehicle_public_mission.py",
))

_WS_HANDLERS_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks", "useWsHandlers.js",
))

_MISSION_UPLOAD_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "navpy", "modules", "vehicle", "mission_upload.py",
))


class TestUploadProgressLogic(unittest.TestCase):
    """Run the Node.js test for progress state + label logic."""

    def test_node_logic(self):
        result = subprocess.run(
            ["node", _NODE_TEST],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, f"Node test failed:\n{result.stderr}")


class TestAppWiresUploadProgressHandler(unittest.TestCase):
    """Verify upload_progress WS handler is registered and wired to App."""

    def setUp(self):
        with open(_APP_FILE, encoding="utf-8") as f:
            self.app_source = f.read()
        with open(_WS_HANDLERS_FILE, encoding="utf-8") as f:
            self.ws_source = f.read()

    def test_handler_registered(self):
        self.assertIn("upload_progress", self.ws_source)

    def test_sets_upload_progress_state(self):
        self.assertIn("setUploadProgress", self.app_source)

    def test_passes_upload_progress_to_bottom_bar(self):
        self.assertIn("uploadProgress={uploadProgress}", self.app_source)


class TestBottomBarShowsPerUavProgress(unittest.TestCase):
    """Verify BottomBar renders per-UAV upload progress."""

    def setUp(self):
        with open(_BOTTOM_BAR_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_accepts_upload_progress_prop(self):
        self.assertIn("uploadProgress", self.source)

    def test_renders_wp_count(self):
        """UploadProgressItem should show wp_sent/wp_total."""
        self.assertIn("wp_sent", self.source)
        self.assertIn("wp_total", self.source)

    def test_stage_labels(self):
        for stage in ("clearing", "uploading", "verifying"):
            self.assertIn(stage, self.source, f"Missing stage label: {stage}")


class TestBackendBroadcastsPerWaypoint(unittest.TestCase):
    """Verify missions.py broadcasts per-waypoint progress."""

    def setUp(self):
        with open(_MISSIONS_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_wp_sent_in_broadcast(self):
        self.assertIn('"wp_sent"', self.source)

    def test_wp_total_in_broadcast(self):
        self.assertIn('"wp_total"', self.source)

    def test_on_wp_progress_callback(self):
        self.assertIn("on_wp_progress", self.source)


class TestWaypointBuilderThreadsCallback(unittest.TestCase):
    """Verify waypoint_builder passes on_wp_progress to vehicle."""

    def setUp(self):
        with open(_WP_BUILDER_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_accepts_on_wp_progress(self):
        self.assertIn("on_wp_progress", self.source)

    def test_passes_to_upload_mission(self):
        self.assertIn("on_wp_sent=on_wp_progress", self.source)


class TestVehicleMavOnWpSent(unittest.TestCase):
    """Verify upload_mission accepts on_wp_sent callback."""

    def setUp(self):
        with open(_VEHICLE_MISSION_FACET_FILE, encoding="utf-8") as f:
            self.veh_source = f.read()
        with open(_MISSION_UPLOAD_FILE, encoding="utf-8") as f:
            self.msn_source = f.read()

    def test_on_wp_sent_parameter(self):
        self.assertIn("on_wp_sent", self.veh_source)

    def test_callback_called_with_seq_and_total(self):
        """The focused uploader reports each newly completed sequence."""
        self.assertIn("on_wp_sent(next_sequence, total)", self.msn_source)


if __name__ == "__main__":
    unittest.main()
