"""Tests for fetchWithRetry utility.

Runs test_fetch_retry_logic.js via Node.js subprocess to verify retry
behaviour: success on first try, retry on 5xx, no retry on 4xx,
retry on network error, and exhaust retries.
"""
import os
import subprocess
import unittest


_JS_TEST = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "test_fetch_retry_logic.js",
))


class TestFetchRetry(unittest.TestCase):

    def test_fetch_retry_logic_js(self):
        """Run Node.js fetch retry tests and verify PASS output."""
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
