"""Run the Node test of the real taskAssignmentState reducer.

It replaces the earlier tests that exercised copies of the hook's updaters
(test_task_assignment_logic.js, test_denied_cleanup_logic.js); their cases
are ported onto the real module.
"""
import os
import subprocess
import unittest

_NODE_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_task_assignment_state.js",
))


class TestTaskAssignmentStateNode(unittest.TestCase):
    """Run the Node.js reducer test against the real ESM module."""

    def test_node_reducer(self):
        result = subprocess.run(
            ["node", _NODE_TEST],
            capture_output=True, text=True, timeout=20,
        )
        self.assertEqual(
            result.returncode, 0,
            f"Node test failed:\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}",
        )
        self.assertIn("PASS", result.stdout)


if __name__ == "__main__":
    unittest.main()
