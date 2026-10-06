"""Source wiring for the single Jetson tracking-overlay video panel.

Behaviour the panel's own Vitest suites cover (state machine, session/track
release, corner placement): ``src/gcs/frontend/src/hooks/useWhepStream.test.jsx``,
``src/gcs/frontend/src/components/VideoPanel.test.jsx`` and
``src/gcs/frontend/src/vendor/mediamtx/reader.test.js``.

What is asserted here instead is the part that only exists in App.jsx and in the
repository layout:

- the panel is mounted in the monitor view, and NOT during manual control, so it
  cannot sit over the joysticks, the flight-mode column or the FPV HUD;
- every operator control the video could have displaced is still wired:
  confirm / deny / cancel, E-STOP in both hosts, and manual control;
- the feed is configured by exactly one public build-time variable — no
  credentials in the tree, no settings field, no backend route;
- the vendored third-party reader carries its provenance and licence.
"""
import os
import re
import unittest

_REPO = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
_FRONTEND = os.path.join(_REPO, "src", "gcs", "frontend", "src")
_BACKEND = os.path.join(_REPO, "src", "gcs", "backend")
_ENV_VAR = "VITE_VIDEO_WHEP_URL"


def _read(*parts):
    with open(os.path.join(_FRONTEND, *parts), encoding="utf-8") as handle:
        return handle.read()


def _walk_sources(root, suffixes):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in {"node_modules", "__pycache__"}]
        for name in filenames:
            if name.endswith(suffixes):
                path = os.path.join(dirpath, name)
                with open(path, encoding="utf-8") as handle:
                    yield path, handle.read()


class TestAppMountsPanelInMonitorOnly(unittest.TestCase):
    def setUp(self):
        self.src = _read("App.jsx")

    def test_imports_and_mounts_the_panel(self):
        self.assertIn("import VideoPanel from './components/VideoPanel'", self.src)
        self.assertIn(
            "{phase === PHASES.MONITOR && !mc.manualControlEnabled && <VideoPanel />}",
            self.src,
        )
        # One feed, mounted once.
        self.assertEqual(self.src.count("<VideoPanel"), 1)

    def test_mounted_inside_the_map_pane_before_the_manual_control_block(self):
        map_start = self.src.index("<CesiumMap")
        manual_start = self.src.index("{mc.manualControlEnabled && (")
        panel = self.src.index("<VideoPanel />")
        self.assertLess(map_start, panel)
        self.assertLess(panel, manual_start)

    def test_operator_controls_are_untouched(self):
        # Confirmation actions (map cards and sidebar) still reach the handlers.
        for handler in ("handleTaskApprove", "handleTaskDeny", "handleTaskCancel"):
            self.assertIn(f"ws.{handler}", self.src)
        # E-STOP still goes to both hosts (see test_estop_placement_js.py).
        self.assertEqual(self.src.count("onEstop={commands.handleEstop}"), 2)
        # Manual control entry and its overlays.
        self.assertIn("onToggleManualControl={mc.handleToggleManualControl}", self.src)
        self.assertIn("<ManualControlOverlay", self.src)
        self.assertIn("<FlightModeColumn", self.src)


class TestPanelLayering(unittest.TestCase):
    def test_video_sits_below_the_control_layers(self):
        src = _read("components", "VideoPanel.jsx")
        match = re.search(r"const Z_INDEX = (\d+);", src)
        self.assertIsNotNone(match, "VideoPanel must name its z-order explicitly")
        # Map overlay buttons are 10, confirmation cards 20.
        self.assertLess(int(match.group(1)), 10)

    def test_no_latency_claim_in_the_operator_surface(self):
        for name in (("components", "VideoPanel.jsx"), ("hooks", "useWhepStream.js")):
            self.assertNotIn("latency", _read(*name).lower())


class TestSingleBuildTimeConfiguration(unittest.TestCase):
    def test_only_the_config_helper_reads_the_variable(self):
        readers = [
            os.path.relpath(path, _REPO)
            for path, text in _walk_sources(_FRONTEND, (".js", ".jsx"))
            if _ENV_VAR in text
        ]
        self.assertEqual(
            readers, [os.path.join("src", "gcs", "frontend", "src", "utils", "videoConfig.js")]
        )

    def test_absent_by_default_and_no_credentials_or_baked_host(self):
        src = _read("utils", "videoConfig.js")
        self.assertIn("return null", src)
        # No endpoint, host or credential baked into the bundle.
        for token in ("192.168.", "http://", "https://", "user", "pass", "token"):
            self.assertNotIn(token, src.split("*/", 1)[1])

    def test_no_settings_field_and_no_backend_involvement(self):
        for path, text in _walk_sources(os.path.join(_FRONTEND, "components", "settings"), (".jsx", ".js")):
            self.assertNotIn("whep", text.lower(), path)
        for path, text in _walk_sources(_BACKEND, (".py",)):
            self.assertNotIn("whep", text.lower(), path)


class TestVendoredReaderProvenance(unittest.TestCase):
    def setUp(self):
        self.notice = _read("vendor", "mediamtx", "NOTICE")
        self.reader = _read("vendor", "mediamtx", "reader.js")

    def test_notice_names_source_revision_and_licence(self):
        self.assertIn("https://github.com/bluenviron/mediamtx", self.notice)
        self.assertIn("v1.20.0", self.notice)
        self.assertIn("1b943637a4b5778bb929a7af7687b048fecaa03f", self.notice)
        self.assertIn("MIT License", self.notice)
        self.assertIn("Copyright (c) 2019 aler9", self.notice)
        self.assertIn("Permission is hereby granted, free of charge", self.notice)
        self.assertIn('THE SOFTWARE IS PROVIDED "AS IS"', self.notice)

    def test_session_release_is_one_helper_used_by_every_path(self):
        self.assertIn("#releaseSession()", self.reader)
        # close(), the error path and the late-POST path — no fourth DELETE site.
        self.assertEqual(self.reader.count("this.#releaseSession();"), 3)
        self.assertEqual(self.reader.count('method: "DELETE"'), 1)

    def test_location_is_used_as_returned_not_rebuilt(self):
        self.assertIn('res.headers.get("location")', self.reader)
        # The DELETE takes the stored session URL, never a re-derived path.
        self.assertIn("fetch(sessionUrl, {", self.reader)


class TestLocaleStrings(unittest.TestCase):
    def test_panel_strings_exist_in_both_locales(self):
        for locale in ("en", "hy"):
            src = _read("locales", f"{locale}.json")
            self.assertIn('"videoPanel"', src)
            for key in ("title", "connecting", "stalled", "error", "retry", "expand", "collapse"):
                self.assertIn(f'"{key}"', src)


if __name__ == "__main__":
    unittest.main()
