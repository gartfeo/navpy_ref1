"""Runs the mission download reconciliation regression in Node.js."""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_mission_reconciliation_logic.js",
))


class TestMissionReconciliation(unittest.TestCase):

    def test_mission_reconciliation_logic_js(self):
        result = subprocess.run(
            ["node", _JS_TEST],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(
            result.returncode, 0,
            f"Node.js test failed (exit {result.returncode}):\n{result.stderr}",
        )
        self.assertIn("regression passed", result.stdout)


if __name__ == "__main__":
    unittest.main()
