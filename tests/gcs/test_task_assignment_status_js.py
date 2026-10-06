"""Tests for the 4-state assignment status model in useTaskAssignment.js."""
import os
import unittest

_HOOK_FILE = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "hooks",
    "useTaskAssignment.js",
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


class TestUseTaskAssignmentStatusModel(unittest.TestCase):
    def setUp(self):
        with open(_HOOK_FILE, encoding="utf-8") as f:
            self.source = f.read()

    def test_handle_assign_request_sets_assigning(self):
        """handleAssignRequest must set status to 'assigning', not 'assigned'."""
        self.assertIn("status: 'assigning'", self.source)

    def test_handle_assign_request_does_not_set_assigned_directly(self):
        """handleAssignRequest must not set status to 'assigned' — that belongs
        to handleAssignResponse."""
        # Verify 'assigning' appears before 'assigned' (i.e. the request handler
        # uses 'assigning') by checking the relative position in the file.
        assigning_pos = self.source.index("status: 'assigning'")
        assigned_pos = self.source.index("status: 'assigned'")
        self.assertLess(assigning_pos, assigned_pos,
                        "'assigning' should appear before 'assigned' in the file")

    def test_handle_assign_response_sets_assigned(self):
        """handleAssignResponse must set status to 'assigned' on acceptance."""
        self.assertIn("status: 'assigned'", self.source)

    def test_old_accepted_status_absent(self):
        """The legacy status value 'accepted' must not appear anywhere."""
        self.assertNotIn("'accepted'", self.source)

    def test_handle_confirming_status_function_exists(self):
        """handleConfirmingStatus must be defined in the hook."""
        self.assertIn("handleConfirmingStatus", self.source)

    def test_handle_confirming_status_sets_confirming(self):
        """handleConfirmingStatus must set status to 'confirming'."""
        self.assertIn("status: 'confirming'", self.source)

    def test_handle_confirmed_status_function_exists(self):
        """The task-keyed confirmed-status setter must be defined in the hook."""
        self.assertIn("handleConfirmedStatusByTask", self.source)

    def test_handle_confirmed_status_sets_confirmed(self):
        """The confirmed-status setter must set status to 'confirmed'."""
        self.assertIn("status: 'confirmed'", self.source)

    def test_handle_confirming_status_exported_in_return(self):
        """handleConfirmingStatus must appear in the return object."""
        # Find the return block and verify both handlers are listed there.
        return_idx = self.source.rfind("return {")
        self.assertNotEqual(return_idx, -1, "No return { found in hook")
        return_block = self.source[return_idx:]
        self.assertIn("handleConfirmingStatus", return_block)

    def test_handle_confirmed_status_exported_in_return(self):
        """The task-keyed confirmed-status setter must appear in the return."""
        return_idx = self.source.rfind("return {")
        self.assertNotEqual(return_idx, -1, "No return { found in hook")
        return_block = self.source[return_idx:]
        self.assertIn("handleConfirmedStatusByTask", return_block)


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

    def test_checkmark_check_uses_assigned_not_accepted(self):
        """The checkmark label suffix must test for 'assigned', not 'accepted'."""
        self.assertIn("=== 'assigned'", self.source)
        self.assertNotIn("=== 'accepted'", self.source)


class TestAssignmentListFileRemoved(unittest.TestCase):
    def test_assignment_list_jsx_does_not_exist(self):
        """AssignmentList.jsx must not exist — it was replaced by AssignmentRow."""
        self.assertFalse(
            os.path.exists(_ASSIGNMENT_LIST_FILE),
            f"AssignmentList.jsx should not exist but was found at {_ASSIGNMENT_LIST_FILE}",
        )


if __name__ == "__main__":
    unittest.main()
