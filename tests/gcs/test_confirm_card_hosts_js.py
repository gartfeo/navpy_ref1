"""Source wiring for the task-confirm hosts (todo 69).

Behaviour is covered by the vitest suites (ConfirmCardList.test.jsx,
TopBar.test.jsx, rcConfirmGeometry.test.jsx); App.jsx itself cannot render in
jsdom, so these assert how App mounts the one list component:
- the right panel is a ConfirmCardList whose scrolling part holds the planning
  and the monitor sidebar, so rounds show in both phases;
- View camera only in monitor (the vehicle window exists only there);
- the RC list sits 8 px from the FPV tag, the right stick and the UAV
  selector, under the HUD warning strip.
"""
import os
import unittest

_FRONTEND = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src",
))


def _read(*parts):
    with open(os.path.join(_FRONTEND, *parts), encoding="utf-8") as f:
        return f.read()


class TestRightPanelHost(unittest.TestCase):
    def setUp(self):
        self.src = _read("App.jsx")
        start = self.src.index("{!mc.manualControlEnabled && <ConfirmCardList")
        self.panel = self.src[start:self.src.index("</ConfirmCardList>}", start)]

    def test_panel_list_holds_both_sidebars(self):
        self.assertIn("<PlanningSidebar", self.panel)
        self.assertIn("<MonitoringSidebar", self.panel)

    def test_view_camera_in_monitor_only(self):
        self.assertIn("videoActionFor={phase === PHASES.MONITOR ? videoActionFor : null}", self.panel)

    def test_cards_render_only_through_the_list(self):
        self.assertNotIn("<TaskConfirmCard", self.src)
        self.assertEqual(self.src.count("<ConfirmCardList"), 2)


class TestRcHost(unittest.TestCase):
    def setUp(self):
        self.src = _read("App.jsx")
        start = self.src.index("{rcConfirmShown && (\n            <ConfirmCardList")
        self.rc = self.src[start:self.src.index("/>", start)]

    def test_shown_in_rc_while_any_round_exists(self):
        self.assertIn(
            "const rcConfirmShown = mc.manualControlEnabled && Object.keys(taskConfirm.pendingConfirms || {}).length > 0;",
            self.src,
        )

    def test_held_banner_spans_between_the_mode_buttons_and_the_list(self):
        for line in (
            "const RC_HELD_LEFT_PX = MODE_COLUMN_INSET_PX + MODE_BUTTON_WIDTH_PX + RC_CONFIRM_GAP_PX;",
            "const RC_HELD_RIGHT_PX = RC_CONFIRM_RIGHT_PX + sidebarWidth + RC_CONFIRM_GAP_PX;",
        ):
            self.assertIn(line, self.src)
        start = self.src.index("<FpvViewOverlay")
        overlay = self.src[start:self.src.index("/>", start)]
        self.assertIn("heldLeft={rcConfirmShown ? RC_HELD_LEFT_PX : undefined}", overlay)
        self.assertIn("heldRight={rcConfirmShown ? RC_HELD_RIGHT_PX : undefined}", overlay)

    def test_placement_derives_from_what_it_must_not_cover(self):
        for line in (
            "const RC_CONFIRM_GAP_PX = 8;",
            "const RC_CONFIRM_TOP_PX = RC_FPV_TAG_TOP_PX + FPV_TAG_HEIGHT_PX + RC_CONFIRM_GAP_PX;",
            "const RC_CONFIRM_RIGHT_PX = STICK_INSET_PX + STICK_SIZE_PX + RC_CONFIRM_GAP_PX;",
            "const RC_CONFIRM_BOTTOM_PX = VEHICLE_SELECTOR_SPAN_PX + RC_CONFIRM_GAP_PX;",
        ):
            self.assertIn(line, self.src)

    def test_list_uses_the_placement_and_cap(self):
        self.assertIn("<ConfirmCardList", self.rc)
        self.assertIn("top: RC_CONFIRM_TOP_PX,", self.rc)
        self.assertIn("right: RC_CONFIRM_RIGHT_PX,", self.rc)
        self.assertIn("width: sidebarWidth,", self.rc)
        self.assertIn("maxHeight: `calc(100% - ${RC_CONFIRM_TOP_PX + RC_CONFIRM_BOTTOM_PX}px)`,", self.rc)
        self.assertIn("zIndex: RC_CONFIRM_Z_INDEX,", self.rc)
        self.assertNotIn("videoActionFor", self.rc)

    def test_under_the_warning_strip_over_the_fpv_tag(self):
        self.assertIn("const RC_CONFIRM_Z_INDEX = 9;", self.src)
        hud = _read("components", "TelemetryHud.jsx")
        strip = hud[hud.index("STATUSTEXT warnings"):]
        self.assertIn("zIndex: 10,", strip[:strip.index("<StatusStrip")])
        self.assertIn("zIndex: 8", _read("components", "map", "FpvViewOverlay.jsx"))


if __name__ == "__main__":
    unittest.main()
