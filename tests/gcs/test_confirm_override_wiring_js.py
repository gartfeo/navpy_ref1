"""Source-level wiring checks for CONF-03: the recognition-gate blocked-state
badge (D-15/D-16/D-17) and the "Ask me anyway" one-shot override
(D-18/D-19/D-20).

The pure gate-bypass/status-parsing logic is covered by
tests/modules/nav/test_confirm_override.py (drone) and
tests/gcs/test_confirm_blocked_statustext.py (GCS parser). These assert the
React layer is wired to it: the card renders the blocked reason, the override
button and forced-popup approve both require press-and-hold, and every
render site forwards the new props through.
"""
import os
import unittest

_FRONTEND = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src",
))


def _read(*parts):
    with open(os.path.join(_FRONTEND, *parts), encoding="utf-8") as f:
        return f.read()


class TestVehicleStatusCardBlockedBadge(unittest.TestCase):
    def setUp(self):
        self.src = _read("components", "sidebar", "VehicleStatusCard.jsx")

    def test_renders_confirm_blocked_row(self):
        self.assertIn("v.confirm_blocked", self.src)
        self.assertIn("<ConfirmBlockedRow", self.src)

    def test_named_reason_labels(self):
        self.assertIn("vehicle.confirmBlockedPixels", self.src)
        self.assertIn("vehicle.confirmBlockedZoom", self.src)

    def test_ask_me_anyway_uses_long_press(self):
        self.assertIn("import useLongPress from '../../hooks/useLongPress'", self.src)
        self.assertIn("LONG_PRESS_MS", self.src)
        self.assertIn("vehicle.askMeAnyway", self.src)

    def test_override_button_disabled_without_handler(self):
        # canForce gates rendering the button at all -- no dead/disabled
        # button left on screen when the wiring is missing.
        self.assertIn("typeof onForceConfirm === 'function'", self.src)

    def test_accepts_on_force_confirm_prop(self):
        self.assertIn("onForceConfirm", self.src)


class TestTaskConfirmCardForcedMark(unittest.TestCase):
    def setUp(self):
        self.src = _read("components", "TaskConfirmCard.jsx")

    def test_accepts_forced_prop(self):
        self.assertIn("forced = false", self.src)

    def test_gate_overridden_mark_shown_only_when_forced(self):
        self.assertIn("pending && forced", self.src)
        self.assertIn("task.gateOverridden", self.src)

    def test_forced_approve_uses_long_press(self):
        self.assertIn("import useLongPress from '../hooks/useLongPress'", self.src)
        self.assertIn("approveHoldHandlers", self.src)
        self.assertIn("task.holdToApprove", self.src)

    def test_normal_approve_stays_instant(self):
        # The non-forced branch must still exist with the original one-click
        # approve wired to onApprove.
        self.assertIn("onClick={() => onApprove(sysId)}", self.src)


class TestUseTaskConfirmationForceOverride(unittest.TestCase):
    def setUp(self):
        self.src = _read("hooks", "useTaskConfirmation.js")

    def test_exposes_forced_state_and_action(self):
        self.assertIn("forcedConfirms", self.src)
        self.assertIn("forceConfirm", self.src)

    def test_posts_to_force_confirm_route(self):
        self.assertIn("/api/control/task_confirm_override", self.src)

    def test_marks_forced_only_after_successful_post(self):
        self.assertIn("markForced(sysId, taskId)", self.src)
        # markForced must be called after the fetch's ok-check, not before --
        # verify by position: the fetch call appears before markForced.
        fetch_pos = self.src.index("/api/control/task_confirm_override")
        mark_pos = self.src.index("markForced(sysId, taskId)")
        self.assertLess(fetch_pos, mark_pos)


class TestMonitoringSidebarForwardsForceConfirm(unittest.TestCase):
    def setUp(self):
        self.src = _read("components", "sidebar", "MonitoringSidebar.jsx")

    def test_top_level_accepts_props(self):
        self.assertIn("forcedConfirms,", self.src)
        self.assertIn("onForceConfirm,", self.src)

    def test_task_confirm_card_gets_forced_flag(self):
        self.assertIn("forced={forcedConfirms?.[sid] === entry.taskId}", self.src)

    def test_active_monitoring_forwards_to_vehicle_status_card(self):
        # 1x MonitoringSidebar -> ActiveMonitoring, + 2x ActiveMonitoring ->
        # VehicleStatusCard (both render sites) = 3 occurrences.
        self.assertEqual(
            self.src.count("onForceConfirm={onForceConfirm}"), 3,
            "MonitoringSidebar -> ActiveMonitoring and both VehicleStatusCard "
            "render sites must all forward onForceConfirm",
        )


class TestAppWiresForceConfirm(unittest.TestCase):
    def setUp(self):
        self.src = _read("App.jsx")

    def test_manual_overlay_card_gets_forced_flag(self):
        self.assertIn("forced={taskConfirm.forcedConfirms[sid] === entry.taskId}", self.src)

    def test_sidebar_gets_forced_confirms_and_handler(self):
        self.assertIn("forcedConfirms={taskConfirm.forcedConfirms}", self.src)
        self.assertIn("onForceConfirm={taskConfirm.forceConfirm}", self.src)


if __name__ == "__main__":
    unittest.main()
