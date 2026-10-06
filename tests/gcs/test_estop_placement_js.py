"""Source wiring for E-STOP placement.

Final layout:
- The E-STOP split-button dropdown is ONE shared component (EstopDropdown):
  the button opens a scope menu — Stop ALL, or a single UAV (colored UAV
  badge, then a red "Stop" label) — and every choice is gated by the danger
  ConfirmModal before onEstop fires (undefined = ALL, [sysId] = one).
- Primary host: MonitoringSidebar UAV Status header (monitor view).
- Fallback host: TopBar, rendered ONLY when the sidebar instance is not
  mounted — planning phase or manual control — while vehicles are connected
  (Codex review of PR #222: E-STOP used to vanish entirely in those states).
- VehicleStatusCard carries no stop control (and no ConfirmModal).
"""
import os
import unittest

_FRONTEND = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src",
))


def _read(*parts):
    with open(os.path.join(_FRONTEND, *parts), encoding="utf-8") as f:
        return f.read()


class TestEstopDropdownComponent(unittest.TestCase):
    def setUp(self):
        self.src = _read("components", "EstopDropdown.jsx")

    def test_button_opens_scope_menu(self):
        # The E-STOP button opens the scope dropdown (it does not stop directly).
        self.assertIn("setShowEstopMenu((v) => !v)", self.src)
        self.assertIn("topBar.estop", self.src)

    def test_scope_menu_lists_all_and_per_uav(self):
        self.assertIn("topBar.stopAll", self.src)
        # "Stop ALL" row triggers the all-stop confirm.
        self.assertIn("openEstop(null)", self.src)
        # Per-UAV rows come from the vehicle list, read as a red "Stop"
        # action, and carry the colored UAV badge (badge first, then label).
        self.assertIn("vehicleList || []).map", self.src)
        self.assertIn("openEstop({ sysId: v.sys_id", self.src)
        self.assertIn("topBar.stop", self.src)
        self.assertIn("<UavBadge", self.src)
        self.assertIn("import UavBadge", self.src)

    def test_confirm_scoped_and_calls_onestop_with_scope(self):
        self.assertIn("ConfirmModal", self.src)
        # ALL -> undefined; single -> [sysId].
        self.assertIn("onEstop(estopTarget ? [estopTarget.sysId] : undefined)", self.src)
        self.assertIn("topBar.emergencyStop", self.src)
        self.assertIn("topBar.disarmOne", self.src)
        self.assertIn("topBar.disarmAll", self.src)

    def test_rows_respond_as_buttons(self):
        # Inline styles can't express :hover; the scoped class provides the fill.
        self.assertIn(".estopMenuItem:hover", self.src)


class TestSidebarHostsPrimaryDropdown(unittest.TestCase):
    def setUp(self):
        self.src = _read("components", "sidebar", "MonitoringSidebar.jsx")

    def test_renders_shared_component(self):
        self.assertIn("import EstopDropdown", self.src)
        self.assertIn("<EstopDropdown vehicleList={vehicleList} onEstop={onEstop} />", self.src)

    def test_no_inline_menu_leftovers(self):
        # The dropdown internals live in the shared component only.
        self.assertNotIn("setShowEstopMenu", self.src)
        self.assertNotIn("openEstop", self.src)
        self.assertNotIn("topBar.stopAll", self.src)
        self.assertNotIn("topBar.disarm", self.src)


class TestTopBarFallbackHost(unittest.TestCase):
    """Codex findings on PR #222: the sidebar is unmounted in the planning
    phase (App gates it on PHASES.MONITOR) and hidden under manual control —
    both left connected/armed fleets with NO E-STOP anywhere. The TopBar hosts
    the same shared dropdown exactly in those states."""

    def setUp(self):
        self.src = _read("components", "TopBar.jsx")

    def test_renders_shared_component_as_fallback(self):
        self.assertIn("import EstopDropdown", self.src)
        self.assertIn('size="topbar"', self.src)
        self.assertIn("showEstopFallback &&", self.src)

    def test_fallback_condition_covers_both_gaps(self):
        # Vehicles connected AND (not monitor phase OR manual control).
        self.assertIn("uavCount > 0", self.src)
        self.assertIn("phase !== PHASES.MONITOR || manualControlEnabled", self.src)
        self.assertIn("import { PHASES }", self.src)

    def test_no_inline_menu_of_its_own(self):
        self.assertNotIn("setShowEstopMenu", self.src)
        self.assertNotIn("openEstop(", self.src)
        self.assertNotIn("topBar.stopAll", self.src)
        self.assertNotIn("ConfirmModal", self.src)


class TestCardHasNoStopControl(unittest.TestCase):
    def test_vehicle_card_has_no_estop_or_confirm(self):
        src = _read("components", "sidebar", "VehicleStatusCard.jsx")
        self.assertNotIn("onEstop", src)
        self.assertNotIn("Estop", src)
        self.assertNotIn("topBar.", src)
        # No ConfirmModal on the card at all: per-UAV stop confirms live in the
        # shared dropdown, the reboot confirm in PreflightReadiness.
        self.assertNotIn("ConfirmModal", src)


class TestEstopCommandForwardsScope(unittest.TestCase):
    def test_handle_estop_forwards_sysids(self):
        src = _read("hooks", "useVehicleCommands.js")
        self.assertIn("api.sendCommand('estop', sysIds)", src)
        self.assertNotIn("handlePersonalEstop", src)


class TestAppWiring(unittest.TestCase):
    def test_estop_wired_to_both_hosts(self):
        src = _read("App.jsx")
        self.assertEqual(
            src.count("onEstop={commands.handleEstop}"), 2,
            "E-STOP handler goes to both hosts: TopBar (fallback) + MonitoringSidebar",
        )
        block = src[src.index("<TopBar"):]
        block = block[:block.index("/>")]
        self.assertIn("onEstop={commands.handleEstop}", block)
        self.assertIn("manualControlEnabled={mc.manualControlEnabled}", block)


class TestLocaleStrings(unittest.TestCase):
    def test_scope_strings_present(self):
        for loc in ("en", "hy"):
            src = _read("locales", f"{loc}.json")
            self.assertIn('"estop"', src)
            self.assertIn('"stop"', src)
            self.assertIn('"stopAll"', src)
            self.assertIn('"disarmOne"', src)
            self.assertIn('"disarmAll"', src)
            self.assertIn('"emergencyStop"', src)


if __name__ == "__main__":
    unittest.main()
