"""Tests for task deny cleanup: source-level wiring checks.

The cleanup itself (vehicle_reset in the taskAssignmentState reducer) is
tested on the real module by test_task_assignment_state.js.
"""
import os
import unittest

_HOOK_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks",
    "useTaskAssignment.js",
))

_STATE_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "taskAssignmentState.js",
))

_APP_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "App.jsx",
))


class TestHandleDeniedCleanupHook(unittest.TestCase):
    """Verify useTaskAssignment exposes handleDeniedCleanup."""

    def setUp(self):
        with open(_HOOK_FILE, encoding="utf-8") as f:
            self.source = f.read()
        with open(_STATE_FILE, encoding="utf-8") as f:
            self.state_source = f.read()

    def test_function_defined(self):
        self.assertIn("handleDeniedCleanup", self.source)

    def test_filters_by_receiver_id(self):
        self.assertIn("entry.receiverId === sysId", self.state_source)

    def test_exposed_as_a_handler(self):
        self.assertIn("handleDeniedCleanup: (sysId) =>", self.source)


_WS_HANDLERS_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks",
    "useWsHandlers.js",
))


class TestAppDenyWiring(unittest.TestCase):
    """Verify deny wiring: useWsHandlers wraps deny with cleanup, App.jsx uses it."""

    def setUp(self):
        with open(_APP_FILE, encoding="utf-8") as f:
            self.app_source = f.read()
        with open(_WS_HANDLERS_FILE, encoding="utf-8") as f:
            self.ws_source = f.read()

    def test_handle_task_deny_defined_in_ws_handlers(self):
        self.assertIn("handleTaskDeny", self.ws_source)

    def test_handle_task_deny_calls_deny(self):
        self.assertIn("taskConfirm.deny(sysId)", self.ws_source)

    def test_deny_cleanup_converges_via_ws_response(self):
        """Assignment cleanup is no longer done in the deny wrapper (that would
        diverge from the card on a failed POST). It converges for all clients,
        task-keyed, via the task_confirm_response WS handler."""
        self.assertIn("taskAssign.handleResolvedCleanupByTask", self.ws_source)
        self.assertNotIn("taskAssign.handleDeniedCleanup(sysId)", self.ws_source)

    def test_no_raw_deny_in_on_deny_props(self):
        """onDeny props must not use taskConfirm.deny directly."""
        self.assertNotIn("onDeny={taskConfirm.deny}", self.app_source)

    def test_on_deny_uses_ws_handler(self):
        """Both onDeny props must reference ws.handleTaskDeny."""
        count = self.app_source.count("onDeny={ws.handleTaskDeny}")
        self.assertEqual(count, 2,
                         f"Expected 2 onDeny={{ws.handleTaskDeny}} but found {count}")


if __name__ == "__main__":
    unittest.main()
