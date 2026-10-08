"""Tests for the 4-state assignment status model (taskAssignmentState.js).

Behaviour is tested on the real reducer by test_task_assignment_state.js;
these pin the model's shape and its wiring into the hook and the markers.
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

_MONITORING_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "sidebar", "MonitoringSidebar.jsx",
))

_VEHICLE_CARD_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "sidebar", "VehicleStatusCard.jsx",
))

_ASSIGNMENT_MARKERS_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "map", "hooks", "useAssignmentMarkers.js",
))

_ASSIGNMENT_LIST_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "components",
    "sidebar", "AssignmentList.jsx",
))


def _function_body(source, name):
    start = source.index(f"export function {name}(")
    end = source.find("\nexport function ", start + 1)
    return source[start:end if end != -1 else len(source)]


class TestTaskAssignmentStatusModel(unittest.TestCase):
    def setUp(self):
        with open(_HOOK_FILE, encoding="utf-8") as f:
            self.source = f.read()
        with open(_STATE_FILE, encoding="utf-8") as f:
            self.state = f.read()

    def test_statuses_rank_waiting_assigned_confirming_confirmed(self):
        self.assertIn(
            "const RANK = Object.freeze({ waiting: 0, assigned: 1, confirming: 2, confirmed: 3 });",
            self.state,
        )

    def test_assign_request_sets_waiting_not_assigned(self):
        """Step 3 leaves the helper WAITING; it is not assigned yet."""
        body = _function_body(self.state, "assignRequest")
        self.assertIn("ASSIGNMENT_STATUS.WAITING", body)
        self.assertNotIn("ASSIGNMENT_STATUS.ASSIGNED", body)

    def test_assign_response_does_not_assign(self):
        """The helper's "doing" is not the truth; only the owner's APPLIED is."""
        self.assertNotIn("ASSIGNMENT_STATUS.ASSIGNED", _function_body(self.state, "assignResponse"))

    def test_owner_applied_sets_assigned(self):
        self.assertIn("ASSIGNMENT_STATUS.ASSIGNED", _function_body(self.state, "assignAck"))

    def test_old_accepted_status_absent(self):
        """The legacy status value 'accepted' must not appear anywhere."""
        self.assertNotIn("'accepted'", self.source)
        self.assertNotIn("'accepted'", self.state)

    def test_confirming_and_confirmed_set_by_the_reducer(self):
        self.assertIn("ASSIGNMENT_STATUS.CONFIRMING", _function_body(self.state, "taskConfirming"))
        self.assertIn("ASSIGNMENT_STATUS.CONFIRMED", _function_body(self.state, "taskConfirmed"))

    def test_hook_exposes_the_confirm_status_handlers(self):
        self.assertIn("handleConfirmingStatus: onTask('task_confirming')", self.source)
        self.assertIn("handleConfirmedStatusByTask: onTask('task_confirmed')", self.source)

    def test_hook_routes_the_owner_applied(self):
        self.assertIn("handleAssignAck: on('task_assign_ack')", self.source)


class TestMonitoringSidebarUsesNewArchitecture(unittest.TestCase):
    def setUp(self):
        with open(_MONITORING_FILE, encoding="utf-8") as f:
            self.source = f.read()
        with open(_VEHICLE_CARD_FILE, encoding="utf-8") as f:
            self.card_source = f.read()

    def test_assignment_list_not_imported(self):
        """AssignmentList component must not be imported — it was removed."""
        self.assertNotIn("AssignmentList", self.source)

    def test_imports_compute_mission_status(self):
        """VehicleStatusCard must import computeMissionStatus."""
        self.assertIn("computeMissionStatus", self.card_source)

    def test_imports_mission_status_row(self):
        """VehicleStatusCard must import MissionStatusRow."""
        self.assertIn("MissionStatusRow", self.card_source)

    def test_no_assignment_row_component(self):
        """Old AssignmentRow component must be removed."""
        self.assertNotIn("function AssignmentRow", self.source)

    def test_no_assignment_status_colors(self):
        """Old ASSIGNMENT_STATUS_COLORS constant must be removed."""
        self.assertNotIn("ASSIGNMENT_STATUS_COLORS", self.source)

    def test_no_wp_progress_component(self):
        """Old WPProgress component must be removed (replaced by MissionStatusRow ProgressBar)."""
        self.assertNotIn("function WPProgress", self.source)


class TestAssignmentMarkersUsesAssignedStatus(unittest.TestCase):
    def setUp(self):
        with open(_ASSIGNMENT_MARKERS_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_checkmark_from_assigned_on_not_accepted(self):
        """The checkmark shows once the owner applied it (assigned and later),
        not on the helper's own accept and not while WAITING."""
        self.assertIn("isAssignedOrLater(entry) ? ' \\u2713'", self.source)
        self.assertNotIn("=== 'accepted'", self.source)

    def test_markers_keyed_by_entry_key(self):
        """Keys are `${owner}:${task}` / `helper:${id}` strings, not task ids."""
        self.assertIn("seen.add(key);", self.source)
        self.assertNotIn("Number(", self.source)


class TestAssignmentListFileRemoved(unittest.TestCase):
    def test_assignment_list_jsx_does_not_exist(self):
        """AssignmentList.jsx must not exist — it was replaced by AssignmentRow."""
        self.assertFalse(
            os.path.exists(_ASSIGNMENT_LIST_FILE),
            f"AssignmentList.jsx should not exist but was found at {_ASSIGNMENT_LIST_FILE}",
        )


if __name__ == "__main__":
    unittest.main()
