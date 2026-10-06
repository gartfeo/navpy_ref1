"""Tests for fleet mission-download plan lineage tagging.

Runs test_plan_download_run_logic.js via Node.js subprocess to verify a
download run recognizes plans descended from its own corridor trim and does not
cancel itself after the first UAV.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_plan_download_run_logic.js",
))

_FRONTEND_SRC = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "src", "gcs", "frontend", "src",
))


class TestPlanDownloadRun(unittest.TestCase):

    def test_plan_download_run_logic_js(self):
        """Run Node.js plan lineage tests and verify pass."""
        result = subprocess.run(
            ["node", _JS_TEST],
            capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(
            result.returncode, 0,
            f"Node.js test failed (exit {result.returncode}):\n{result.stderr}",
        )
        self.assertIn("passed", result.stdout, "Expected 'passed' in stdout")

    def test_operator_upload_cancels_startup_before_generation(self):
        """Upload must explicitly invalidate late startup continuations."""
        with open(os.path.join(_FRONTEND_SRC, "hooks", "useVehicleConnection.js"), encoding="utf-8") as f:
            connection_source = f.read()
        with open(os.path.join(_FRONTEND_SRC, "hooks", "useMissionUpload.js"), encoding="utf-8") as f:
            upload_source = f.read()
        with open(os.path.join(_FRONTEND_SRC, "App.jsx"), encoding="utf-8") as f:
            app_source = f.read()

        self.assertIn(
            "cancelStartupMissionReconciliation: disableStartupMissionReconciliation",
            connection_source,
        )
        self.assertIn(
            "onOperatorAction: cancelStartupMissionReconciliation",
            app_source,
        )
        upload_body = upload_source[upload_source.index("const handleUpload"):]
        self.assertLess(
            upload_body.index("onOperatorAction?.()"),
            upload_body.index("await localGenerate("),
            "startup continuation must be cancelled before upload generation mutates the plan",
        )


if __name__ == "__main__":
    unittest.main()
