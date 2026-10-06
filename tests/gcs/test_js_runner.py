"""Tests for the shared Node.js runner (``tests/gcs/js_runner.py``).

The runner exists to dodge the Windows ~32767-char command-line limit that
``node -e <code>`` hits for large programs (WinError 206). These tests pin the
behaviour the JS-utility tests rely on: large programs run, CommonJS/sloppy-mode
globals match ``node -e``, non-ASCII round-trips, and timeout/return code pass through.
"""
import subprocess
import unittest

from tests.gcs.js_runner import run_node


class TestRunNode(unittest.TestCase):
    def test_large_program_runs(self):
        """A program well over the ~32 KB cmdline limit executes (the WinError 206 case)."""
        padding = "// " + "x" * 40000 + "\n"  # > 32767 chars of source
        result = run_node(padding + "console.log(JSON.stringify(40000));")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "40000")

    def test_commonjs_sloppy_mode_globals(self):
        """Top-level ``this === globalThis`` and implicit globals — same as ``node -e``."""
        result = run_node(
            "function f(){ return this === globalThis; }\n"
            "implicitGlobal = 7;\n"
            "console.log(JSON.stringify([this === globalThis, f(), implicitGlobal]));"
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "[true,true,7]")

    def test_node_e_compat_surface(self):
        """Pin the stdin runtime surface so a future switch to temp files/.mjs is caught."""
        result = run_node(
            "console.log(JSON.stringify({"
            "req: typeof require, dir: __dirname, file: __filename, "
            "argvLen: process.argv.length}));"
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        import json
        env = json.loads(result.stdout.strip())
        self.assertEqual(env["req"], "function")
        self.assertEqual(env["dir"], ".")
        self.assertEqual(env["file"], "[stdin]")
        self.assertEqual(env["argvLen"], 1)

    def test_non_ascii_round_trip(self):
        """Non-ASCII source and output survive (utf-8 in, utf-8 out)."""
        result = run_node("console.log(JSON.stringify('✓ ok'));")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), '"✓ ok"')

    def test_nonzero_return_code_passthrough(self):
        """A throwing program reports a nonzero return code and stderr, not an exception."""
        result = run_node("throw new Error('boom');")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("boom", result.stderr)

    def test_timeout_passthrough(self):
        """The ``timeout`` argument reaches subprocess and fires on a hung program."""
        with self.assertRaises(subprocess.TimeoutExpired):
            run_node("setTimeout(() => {}, 60000);", timeout=1)


if __name__ == "__main__":
    unittest.main()
