"""Source wiring for the monitor view's vehicle window (docs/design/gcs_video_streams.md).

Behaviour the window's own Vitest suites cover (state machine, session/track
release, corner placement): ``src/gcs/frontend/src/hooks/useWhepStream.test.jsx``,
``src/gcs/frontend/src/hooks/useVehicleWindow.test.jsx``,
``src/gcs/frontend/src/components/VehicleWindow.test.jsx`` and
``src/gcs/frontend/src/vendor/mediamtx/reader.test.js``.

What is asserted here instead is the part that only exists in App.jsx and in the
repository layout:

- the window is mounted in the monitor view, and NOT during manual control, so it
  cannot sit over the joysticks, the flight-mode column or the FPV HUD;
- every operator control the video could have displaced is still wired:
  confirm / deny / cancel, E-STOP in both hosts, and manual control;
- streams come only from each vehicle's telemetry ``video_streams`` (the old
  build-time ``VITE_VIDEO_WHEP_URL`` and ``videoConfig.js`` are gone, no shim),
  and there is no settings field for a URL;
- the vendored third-party reader carries its provenance and licence;
- the map never chases a UAV (v11): no follow selector, no chase camera, and
  the window never touches the map camera, RC or browser storage.
"""
import json
import os
import re
import unittest

_REPO = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", ".."))
_FRONTEND = os.path.join(_REPO, "src", "gcs", "frontend", "src")
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


class TestAppMountsWindowInMonitorOnly(unittest.TestCase):
    def setUp(self):
        self.src = _read("App.jsx")

    def test_imports_and_mounts_the_window(self):
        self.assertIn("import VehicleWindow, { MAP_SWAPPED_Z_INDEX } from './components/VehicleWindow'", self.src)
        # Monitor only, never in manual control (vehicleWindowVisible).
        self.assertIn("monitor, rcEnabled: mc.manualControlEnabled, fleetSize: videoFleet.length", self.src)
        self.assertIn("active: monitor && !mc.manualControlEnabled", self.src)
        self.assertIn("{showWindow && (", self.src)
        # One window, mounted once.
        self.assertEqual(self.src.count("<VehicleWindow"), 1)

    def test_mounted_inside_the_map_pane_before_the_manual_control_block(self):
        map_start = self.src.index("<CesiumMap")
        manual_start = self.src.index("{mc.manualControlEnabled && (")
        window = self.src.index("<VehicleWindow")
        self.assertLess(map_start, window)
        self.assertLess(window, manual_start)

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


class TestWindowLayering(unittest.TestCase):
    def test_video_sits_below_the_control_layers(self):
        src = _read("components", "VehicleWindow.jsx")
        match = re.search(r"const Z_INDEX = (\d+);", src)
        self.assertIsNotNone(match, "VehicleWindow must name its z-order explicitly")
        # Map overlay buttons are 10, confirmation cards 20.
        self.assertLess(int(match.group(1)), 10)

    def test_no_latency_claim_in_the_operator_surface(self):
        for name in (("components", "VehicleWindow.jsx"), ("components", "SyntheticView.jsx"), ("hooks", "useWhepStream.js")):
            self.assertNotIn("latency", _read(*name).lower())


class TestStreamsComeFromTelemetry(unittest.TestCase):
    def test_build_time_url_and_config_helper_are_gone(self):
        self.assertFalse(os.path.exists(os.path.join(_FRONTEND, "utils", "videoConfig.js")))
        readers = [
            os.path.relpath(path, _REPO)
            for path, text in _walk_sources(_FRONTEND, (".js", ".jsx"))
            if _ENV_VAR in text or "videoConfig" in text
        ]
        self.assertEqual(readers, [])

    def test_no_settings_field(self):
        for path, text in _walk_sources(os.path.join(_FRONTEND, "components", "settings"), (".jsx", ".js")):
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

    def test_fetch_policy_patch_is_in_place(self):
        self.assertIn('{ redirect: "error", credentials: "omit" }', self.reader)
        self.assertIn("#checkedSessionUrl(", self.reader)
        self.assertIn("6. Fetch policy", self.notice)

    def test_location_is_used_as_returned_not_rebuilt(self):
        self.assertIn('res.headers.get("location")', self.reader)
        # The DELETE takes the stored session URL, never a re-derived path.
        self.assertIn("fetch(sessionUrl, {", self.reader)


class TestNoMapChase(unittest.TestCase):
    def test_follow_selector_and_state_are_gone(self):
        self.assertNotIn("followSysId", _read("App.jsx"))
        overlay = _read("components", "map", "MonitorMapOverlay.jsx")
        self.assertNotIn("<select", overlay)
        self.assertNotIn("monitorOverlay.follow", overlay)
        for locale in ("en", "hy"):
            src = _read("locales", f"{locale}.json")
            self.assertNotIn('"follow"', src)
            self.assertNotIn('"minimize"', src)

    def test_map_camera_only_moves_for_rc_fpv(self):
        follow = _read("components", "map", "hooks", "useFollowUav.js")
        self.assertNotIn("lookAtTransform", follow)
        self.assertIn("fpvSysId", follow)

    def test_window_never_touches_map_camera_rc_or_storage(self):
        for parts in (("components", "VehicleWindow.jsx"), ("hooks", "useVehicleWindow.js")):
            src = _read(*parts)
            for name in ("mapCameraRef", "flyToRef", "resetViewRef", "viewerRef", "manualControl",
                         "localStorage", "sessionStorage"):
                self.assertNotIn(name, src, f"{parts[-1]} must not reference {name}")

    def test_sidebar_fly_to_swaps_back_before_flying(self):
        # While swapped the map's fly-to waits for the swap back; without unswap() a
        # sidebar coordinate click would wait until some other action restored the map.
        app = _read("App.jsx")
        body = re.search(r"const flyToLocation = useCallback\(\(lat, lon\) => \{(.*?)\}, \[", app, re.S)
        self.assertIsNotNone(body)
        self.assertRegex(body.group(1), r"unswap\(\);\s*flyToRef\.current\?\.\(lat, lon\)")
        self.assertIn("onFlyToLocation={flyToLocation}", app)

    def test_swap_ends_when_the_pane_has_no_room_for_the_small_map(self):
        app = _read("App.jsx")
        self.assertIn("const swapped = showWindow && win.swapped && windowLayout.canSwap;", app)
        self.assertRegex(app, r"useLayoutEffect\(\(\) => \{\s*if \(win\.swapped && !windowLayout\.canSwap\) unswap\(\);")


class TestLocaleStrings(unittest.TestCase):
    def test_window_strings_exist_in_both_locales(self):
        for locale in ("en", "hy"):
            src = _read("locales", f"{locale}.json")
            self.assertIn('"vehicleWindow"', src)
            self.assertNotIn('"videoPanel"', src)
            for key in ("region", "reconnect", "dismiss", "videoLost", "linkLost", "maximize", "restore",
                        "pickerPrompt", "syntheticTag", "waitingForPose", "syntheticUnavailable", "chip", "uavList",
                        "noRoom", "opensCamera", "opensSynthetic", "showVideo", "hideVideo", "retry",
                        "groupFor", "noCameraFor"):
                self.assertIn(f'"{key}"', src)

    def test_v12_removed_strings_are_gone(self):
        # Close, the overflow menu and the header state words left with v12;
        # "No camera" and the fleet-wide source name with v12.1.
        for locale in ("en", "hy"):
            window = json.loads(_read("locales", f"{locale}.json"))["vehicleWindow"]
            for key in ("close", "moreUavs", "moreUavsLabel", "syntheticTitle", "noCamera"):
                self.assertNotIn(key, window)
            self.assertNotIn("group", window["source"])
            self.assertIn("video", json.loads(_read("locales", f"{locale}.json"))["monitorOverlay"])


class TestVideoIconOpensTheWindow(unittest.TestCase):
    def test_app_wires_the_video_icon_to_the_window(self):
        app = _read("App.jsx")
        self.assertIn("fleetSize: videoFleet.length, open: win.open", app)
        self.assertIn("videoOpen={win.open}", app)
        self.assertIn("onToggleVideo={win.toggleOpen}", app)

    def test_window_has_no_close_button(self):
        self.assertNotIn("vehicleWindow.close", _read("components", "VehicleWindow.jsx"))


if __name__ == "__main__":
    unittest.main()
