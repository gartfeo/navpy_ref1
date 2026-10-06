"""Bridge the Vitest React component suite into the GCS pytest run.

The GCS frontend has component tests (Vitest + @testing-library/react + jsdom)
under ``src/gcs/frontend/src/**/*.test.jsx`` that cover the safety-critical
container-launch guard end-to-end (press-and-hold to arm, confirm modal, single
click never launches). This wrapper runs them via ``npm test`` so
``python -m pytest tests/gcs`` exercises them alongside the existing
Node-subprocess JS tests.

Note: the repo ``pyproject.toml`` sets ``addopts = "--ignore=tests/gcs"``, so the
root ``python -m pytest tests/`` (the Python-package CI) skips this directory and
never shells out to Node; an explicit ``pytest tests/gcs`` target overrides the
ignore and collects this file.

Behaviour:
  * SKIP (not fail) only when the frontend deps are not installed
    (``node_modules/vitest`` missing) — a checkout without ``npm install`` still
    runs the rest of the suite.
  * FAIL on any nonzero exit, a timeout, or npm missing while deps exist, with
    stdout/stderr/cwd/exit-code in the message.
"""
import os
import subprocess
import sys
import unittest

_FRONTEND_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend",
))
# Presence marker for the installed test runner. Checked specifically (the vitest
# package, not merely node_modules/) so the suite skips precisely when the runner
# is absent — a bare or partial install that lacks vitest skips cleanly instead
# of failing with a confusing "npm test" script error. Once the marker exists,
# every other failure mode (nonzero exit, timeout, npm off PATH) is a hard fail.
_VITEST_MARKER = os.path.join(_FRONTEND_DIR, "node_modules", "vitest", "package.json")

# npm is ``npm.cmd`` on Windows (a shell script, not an .exe), so it must run
# through the shell; on POSIX a plain argv list is correct.
_IS_WINDOWS = sys.platform.startswith("win")
_TIMEOUT_S = 300


class TestFrontendComponentTests(unittest.TestCase):
    """Run the Vitest component suite and require it to pass."""

    def test_vitest_component_suite_passes(self):
        if not os.path.isfile(_VITEST_MARKER):
            self.skipTest(
                "frontend deps not installed (missing "
                "node_modules/vitest); run `npm install` in src/gcs/frontend"
            )

        cmd = "npm test" if _IS_WINDOWS else ["npm", "test"]
        try:
            result = subprocess.run(
                cmd,
                cwd=_FRONTEND_DIR,
                shell=_IS_WINDOWS,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                timeout=_TIMEOUT_S,
            )
        except FileNotFoundError as exc:  # npm off PATH though deps exist (POSIX)
            self.fail(f"npm not found running the Vitest suite (cwd={_FRONTEND_DIR}): {exc}")
        except subprocess.TimeoutExpired as exc:
            self.fail(
                f"Vitest suite timed out after {_TIMEOUT_S}s (cwd={_FRONTEND_DIR})\n"
                f"--- stdout ---\n{exc.stdout or ''}\n"
                f"--- stderr ---\n{exc.stderr or ''}"
            )

        self.assertEqual(
            result.returncode, 0,
            "Vitest component suite failed "
            f"(exit {result.returncode}, cwd={_FRONTEND_DIR})\n"
            f"--- stdout ---\n{result.stdout}\n"
            f"--- stderr ---\n{result.stderr}",
        )


if __name__ == "__main__":
    unittest.main()
