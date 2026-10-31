"""Source-level wiring checks for confirmation view/cancel + abort (step 5).

The pure lifecycle logic is covered by test_task_confirmation_state(.js); these
assert the React layer is wired to it: the card is status-driven, the hook is
guarded and reducer-backed, and the WS layer converges card + assignment state.
"""
import os
import unittest

_FRONTEND = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src",
))


def _read(*parts):
    with open(os.path.join(_FRONTEND, *parts), encoding="utf-8") as f:
        return f.read()


class TestTaskConfirmCard(unittest.TestCase):
    def setUp(self):
        self.src = _read("components", "TaskConfirmCard.jsx")

    def test_uses_reducer_selectors(self):
        end = self.src.index("} from '../hooks/taskConfirmationState'")
        imported = self.src[self.src.rindex("import {", 0, end):end]
        for name in ("isPending", "canCancel", "confirmRoundKey"):
            self.assertIn(name, imported)

    def test_auto_timeout_guarded_to_pending(self):
        # Decided cards must never count down or auto-fire.
        self.assertIn("if (!pending) return undefined;", self.src)

    def test_cancel_control_wired(self):
        self.assertIn("onCancel(sysId)", self.src)
        self.assertIn("canCancel(entry)", self.src)
        self.assertIn("task.cancelTask", self.src)


class TestUseTaskConfirmation(unittest.TestCase):
    def setUp(self):
        self.src = _read("hooks", "useTaskConfirmation.js")

    def test_backed_by_reducer(self):
        self.assertIn("from './taskConfirmationState'", self.src)
        self.assertIn("expireDecided", self.src)

    def test_new_lifecycle_handlers(self):
        for name in ("handleConfirmReset", "handleDisarmAfterGuided", "cancel"):
            self.assertIn(name, self.src)

    def test_abort_not_in_action_table(self):
        # A per-UAV abort is the destructive E-STOP command, not a recoverable
        # confirm action -- it must not appear as an action-table entry.
        self.assertNotIn("abort:", self.src)

    def test_action_table_and_failed_post_guard(self):
        self.assertIn("ACTION_CONFIRMED", self.src)
        # Optimistic decided state only after a successful POST.
        self.assertIn("if (!res.ok)", self.src)

    def test_status_guards_in_respond(self):
        self.assertIn("entry.status !== 'pending'", self.src)
        self.assertIn("entry.status !== 'approved'", self.src)


class TestUseWsHandlers(unittest.TestCase):
    def setUp(self):
        self.src = _read("hooks", "useWsHandlers.js")

    def test_reset_ws_event(self):
        self.assertIn("'task_confirm_reset'", self.src)

    def test_no_vehicle_abort_handler(self):
        # The recoverable per-UAV abort was replaced by the destructive E-STOP;
        # there is no vehicle_abort WS event anymore.
        self.assertNotIn("vehicle_abort", self.src)

    def test_disarm_after_guided_clears_card(self):
        self.assertIn("taskConfirm.handleDisarmAfterGuided", self.src)

    def test_cancel_wrapper(self):
        self.assertIn("handleTaskCancel", self.src)

    def test_assignment_convergence_task_keyed(self):
        self.assertIn("taskAssign.handleConfirmedStatusByTask", self.src)
        self.assertIn("taskAssign.handleResolvedCleanupByTask", self.src)


class TestUseTaskAssignment(unittest.TestCase):
    def setUp(self):
        self.src = _read("hooks", "useTaskAssignment.js")

    def test_task_keyed_helpers_exported(self):
        self.assertIn("handleConfirmedStatusByTask", self.src)
        self.assertIn("handleResolvedCleanupByTask", self.src)

    def test_cleanup_is_uav_keyed(self):
        # A confirm carries the UAV's own POI id, never the owner's task id,
        # so removal goes by UAV: only the helper slot whose asked round this
        # answers, so another POI's round cannot wipe the task.
        # Behaviour: tests/gcs/test_task_assignment_state.js.
        state = _read("utils", "taskAssignmentState.js")
        self.assertIn("const slot = confirmingSlot(state, data);", state)
        self.assertIn("answersRound(slot.confirm, data)", state)
        self.assertNotIn("entry.taskId === data.task_id", state)


class TestMissionStatusUsesPendingOnly(unittest.TestCase):
    def test_vehicle_status_card_uses_has_pending_confirm(self):
        src = _read("components", "sidebar", "VehicleStatusCard.jsx")
        self.assertIn("hasPendingConfirm(pendingConfirms", src)
        self.assertNotIn("!!pendingConfirms?.[v.sys_id]", src)


class TestRenderSitesWireCancel(unittest.TestCase):
    def test_app_passes_cancel_to_both_sites(self):
        src = _read("App.jsx")
        self.assertEqual(
            src.count("onCancel={ws.handleTaskCancel}"), 2,
            "Both confirm lists (right panel + RC overlay) must wire onCancel",
        )

    def test_list_forwards_cancel(self):
        # The cards moved from MonitoringSidebar into ConfirmCardList (todo 69).
        src = _read("components", "ConfirmCardList.jsx")
        self.assertIn("onCancel={onCancel}", src)
        self.assertNotIn("TaskConfirmCard", _read("components", "sidebar", "MonitoringSidebar.jsx"))


if __name__ == "__main__":
    unittest.main()
