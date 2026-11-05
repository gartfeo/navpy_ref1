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

    def test_footer_slot_has_no_box_and_a_round_outranks_the_start_action(self):
        # With a box of its own the slot would be a pinned action's whole
        # containing block, and the action would scroll away with the cards.
        self.assertIn("style={{ display: panelCollapsed ? 'none' : 'contents' }}", self.panel)
        self.assertIn("startPinned={!roundsExist}", self.panel)
        # Any round, decided too: pending only would pin START MISSION back
        # under the finger that just tapped Approve.
        self.assertIn(
            "const roundsExist = Object.keys(taskConfirm.pendingConfirms || {}).length > 0;",
            self.src,
        )

    def test_collapse_keeps_the_one_list_and_the_sidebars_mounted(self):
        # No key and no conditional mount: a pending card's state, its auto-fire
        # and the sidebars' form data survive a toggle.
        opening = self.panel[:self.panel.index(">\n          <div style={panelCollapsed")]
        self.assertNotIn("key=", opening)
        for line in (
            "style={panelHostStyle({ mode: sidePanel.mode, roundsExist, box: collapsedBox })}",
            "heightPx={panelCollapsed ? collapsedBox.height : mapPane.height}",
            "childrenHidden={panelCollapsed}",
            "<div style={panelCollapsed ? { display: 'none' } : undefined}>",
            "footerEl={panelFooterEl}",
            "hidden={panelCollapsed}",
        ):
            self.assertIn(line, self.panel)

    def test_collapsed_state_wiring(self):
        # These come with no tap: they must not reach the list's box.
        box = self.src[self.src.index("collapsedListBox({"):]
        for measured in ("windowChromePx", "mapCreditsPx"):
            self.assertNotIn(measured, box[:box.index("});")])
        for line in (
            "const panelCollapsed = sidePanel.mode === 'collapsed';",
            # A maximized window is not at its normal place: no room to make under it.
            "windowBox: showWindow && !swapped ? windowLayout.window : null,",
            "const collapsedListShown = panelCollapsed && roundsExist && !mc.manualControlEnabled;",
            # The list can reach the toast's place, and starts below the top controls.
            "top={collapsedListShown ? PANEL_MARGIN_PX : undefined}",
            # The toggle's badge counts what still asks, not decided rows.
            "const pendingCount = Object.values(taskConfirm.pendingConfirms || {}).filter(isPending).length;",
            "onTogglePanel={panelToggleShown ? sidePanel.toggle : undefined}",
            "pendingCount={pendingCount}",
        ):
            self.assertIn(line, self.src)

    def test_view_camera_closes_a_drawer(self):
        # The window lies under a drawer: the card would say "Viewing" with no picture.
        for line in (
            "const closeDrawer = sidePanel.mode === 'overlay' ? sidePanel.toggle : null;",
            "onView: () => { viewCamera(sysId); closeDrawer?.(); },",
        ):
            self.assertIn(line, self.src)

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
        # On a phone the mode buttons need the inset with or without a list, and
        # the list may narrow or move (rcListPlacement).
        self.assertIn("heldLeft={rcConfirmShown || rcList.raised ? RC_HELD_LEFT_PX : undefined}", overlay)
        self.assertIn("heldRight={rcConfirmShown ? rcHeldRight : undefined}", overlay)
        self.assertIn("const rcHeldRight = rcList.heldRight ?? RC_HELD_RIGHT_PX;", self.src)
        # The raised list keeps room for the banner whether it shows or not: a banner
        # coming, going or wrapping comes with no tap, and must not move a card.
        self.assertIn("tagLeft={rcList.raised ? RC_HELD_LEFT_PX : undefined}", overlay)
        self.assertIn("heldBottom: RC_HELD_TOP_PX + FPV_HELD_ROOM_PX,", self.src)

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
        # Last in the style, so on a narrow pane the placement overrides App's numbers.
        self.assertIn("...rcList.style,\n              }}", self.rc)
        self.assertIn("heightPx={rcList.heightPx}", self.rc)

    def test_the_speed_tape_lies_under_the_list_and_the_mode_column(self):
        # docs/design/gcs_rc_physical_sticks.md: the tape sits below or beside
        # the column, under the list (9), the held banner (8) and the column's
        # open picker (10), and takes no taps.
        hud = _read("components", "TelemetryHud.jsx")
        self.assertIn("const TAPE_Z_INDEX = 7;", hud)
        tape = hud[hud.index("Speed tape"):]
        self.assertIn("zIndex: TAPE_Z_INDEX,", tape[:tape.index("<SpeedTape")])
        # RC control passes its tape boxes through rcHudLayout (todo 275).
        self.assertIn("layout={rcHudLayout({", self.src)
        self.assertIn("speedBox: speedTapeBox({", self.src)
        # Over a list only while one is raised above the sticks on a phone.
        self.assertIn("listRaised={rcConfirmShown && rcList.raised}", self.src)

    def test_under_the_warning_strip_over_the_fpv_tag(self):
        self.assertIn("const RC_CONFIRM_Z_INDEX = 9;", self.src)
        hud = _read("components", "TelemetryHud.jsx")
        strip = hud[hud.index("STATUSTEXT warnings"):]
        self.assertIn("zIndex: 10,", strip[:strip.index("<StatusStrip")])
        self.assertIn("zIndex: 8", _read("components", "map", "FpvViewOverlay.jsx"))


class TestConfirmCueWiring(unittest.TestCase):
    """Todos 67/68: both hosts show the cards by time left, with the chip and
    the sound, from the one useTaskConfirmation instance."""

    def setUp(self):
        self.app = _read("App.jsx")
        self.list = _read("components", "ConfirmCardList.jsx")

    def test_both_hosts_get_the_cue(self):
        self.assertEqual(self.app.count("<ConfirmCardList"), 2)
        self.assertEqual(self.app.count("cue={taskConfirm.cue}"), 2)

    def test_the_hook_reads_each_uavs_raw_wait(self):
        self.assertIn("aasParams.getConfirmedValue(sysId, 'nav_cwt', undefined)", self.app)
        self.assertIn("useTaskConfirmation({ waitSecFor: confirmWaitSecFor })", self.app)

    def test_the_list_orders_by_the_selector(self):
        self.assertIn("orderedConfirmEntries(pendingConfirms, waitSecFor, fired)", self.list)
        self.assertNotIn("Object.entries(pendingConfirms", self.list)

    def test_the_list_reads_raw_params_for_labels_and_order(self):
        self.assertIn("aasParams.getConfirmedValue(sid, key, undefined)", self.list)
        self.assertIn("<ConfirmSummary", self.list)


if __name__ == "__main__":
    unittest.main()
