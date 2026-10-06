"""Tests for branchColor() in src/gcs/frontend/src/utils/appBranch.js.

The topbar tag color is derived deterministically from the branch name so
every branch / chat instance gets a stable, distinct color. Drives the JS via
Node to lock the hash contract (determinism, hue range, output shape).
"""
import json
import os
import re
from tests.gcs.js_runner import run_node
import unittest


_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils", "appBranch.js",
))

_raw = open(_UTIL_PATH, encoding="utf-8").read()
# Strip module exports so the helper can be evaluated standalone. The top-level
# `__APP_BRANCH__` reference is guarded by `typeof`, which is safe (returns
# 'undefined') when the global is absent in Node.
_JS_SRC = (
    _raw
    .replace("export function ", "function ")
    .replace("export const ", "const ")
)


def _run_js(script: str) -> str:
    result = run_node(_JS_SRC + "\n" + script, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return result.stdout.strip()


def _color(branch: str):
    return json.loads(_run_js(
        f"console.log(JSON.stringify(branchColor({json.dumps(branch)})));"
    ))


class TestBranchColor(unittest.TestCase):
    def test_deterministic(self):
        self.assertEqual(
            _color("feat/param-load-status"),
            _color("feat/param-load-status"),
        )

    def test_hue_in_range(self):
        for name in ["main", "dev", "feat/x", "fix/y", "test/integration-all", ""]:
            c = _color(name)
            self.assertGreaterEqual(c["hue"], 0)
            self.assertLess(c["hue"], 360)

    def test_distinct_branches_spread_out(self):
        names = [
            "main", "dev", "feat/param-load-status",
            "fix/mavftp-reliable-wakeup", "test/integration-all",
            "feat/uav-confirm-cancel-abort",
        ]
        hues = {_color(n)["hue"] for n in names}
        # The hash should spread distinct names across the hue wheel — allow a
        # single rare collision but not a pile-up onto one color.
        self.assertGreaterEqual(len(hues), len(names) - 1)

    def test_outputs_hsl_strings(self):
        c = _color("dev")
        for key in ("bg", "border", "fg", "dot"):
            self.assertTrue(re.match(r"^hsl\(\d+,", c[key]), f"{key}={c[key]}")

    def test_empty_branch_is_valid(self):
        c = _color("")
        self.assertIn("hue", c)
        self.assertTrue(re.match(r"^hsl\(\d+,", c["bg"]))


if __name__ == "__main__":
    unittest.main()
