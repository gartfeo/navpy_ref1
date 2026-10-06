"""Source wiring for the per-UAV reboot action.

The reboot button lives in the Preflight readiness overlay's per-vehicle
Actions row (moved there from the vehicle status card), gated by the caution
ConfirmModal. The backend armed-refusal (test_reboot_command.py) is unchanged;
the UI additionally disables the button for armed or link-down vehicles.
"""
import os
import unittest

_FRONTEND = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src",
))


def _read(*parts):
    with open(os.path.join(_FRONTEND, *parts), encoding="utf-8") as f:
        return f.read()


class TestPreflightReadinessHasReboot(unittest.TestCase):
    def setUp(self):
        self.src = _read("components", "PreflightReadiness.jsx")

    def test_actions_row_renders_reboot_button(self):
        self.assertIn("preflight.actions", self.src)
        self.assertIn("reboot.button", self.src)

    def test_reboot_gated_by_caution_confirm(self):
        self.assertIn("import ConfirmModal", self.src)
        self.assertIn("reboot.confirmTitle", self.src)
        self.assertIn("confirm.confirmReboot", self.src)
        self.assertIn('tone="caution"', self.src)
        # Confirm fires the handler with the chosen vehicle's sys_id.
        self.assertIn("onReboot(rebootPoi.sysId)", self.src)

    def test_button_disabled_when_armed_or_link_down(self):
        self.assertIn("linkDown || v.armed", self.src)
        self.assertIn("reboot.armedBlocked", self.src)


class TestCardAndSidebarHaveNoReboot(unittest.TestCase):
    def test_vehicle_card_has_no_reboot(self):
        src = _read("components", "sidebar", "VehicleStatusCard.jsx")
        self.assertNotIn("onReboot", src)
        self.assertNotIn("reboot.", src)

    def test_monitoring_sidebar_does_not_forward_reboot(self):
        src = _read("components", "sidebar", "MonitoringSidebar.jsx")
        self.assertNotIn("onReboot", src)


class TestAppWiresRebootToReadiness(unittest.TestCase):
    def test_single_reboot_wiring_site(self):
        src = _read("App.jsx")
        self.assertEqual(
            src.count("onReboot={commands.handleReboot}"), 1,
            "Reboot is wired exactly once -- to PreflightReadiness",
        )
        # The one wiring site sits inside the PreflightReadiness element.
        block = src[src.index("<PreflightReadiness"):]
        block = block[:block.index("/>")]
        self.assertIn("onReboot={commands.handleReboot}", block)


class TestLocaleStrings(unittest.TestCase):
    def test_actions_row_label_present(self):
        for loc in ("en", "hy"):
            src = _read("locales", f"{loc}.json")
            self.assertIn('"actions"', src)
            self.assertIn('"confirmReboot"', src)


if __name__ == "__main__":
    unittest.main()
