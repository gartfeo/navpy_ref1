"""Tests for the monitor map plan-entry visibility/label logic.

Runs test_plan_entry_logic.js via a Node.js subprocess to verify computePlanEntry
(constants/monitorPhases.js) returns the correct {show, isEdit} across the plan /
mission / arm / connecting state matrix — the ubiquitous "Edit Plan" /
"Start Planning" entry that moved from the sidebar header onto the monitor map.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_plan_entry_logic.js",
))


class TestPlanEntry(unittest.TestCase):

    def test_plan_entry_logic_js(self):
        """Run Node.js plan-entry tests and verify PASS."""
        result = subprocess.run(
            ["node", _JS_TEST],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(
            result.returncode, 0,
            f"Node.js test failed (exit {result.returncode}):\n{result.stderr}",
        )
        self.assertIn("PASS", result.stdout, "Expected PASS in stdout")


if __name__ == "__main__":
    unittest.main()
