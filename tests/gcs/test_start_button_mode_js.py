"""Tests for resolveStartButtonMode in startButtonMode.js.

Guards the launch-safety property: a container START (armReady=true) must never
start on a single click when ready — it is gated behind a hold-to-arm — while the
bungee START MISSION (armReady=false) keeps its single-click behaviour and the
not-ready path stays a hold-to-force.
"""
import json
import os
import unittest

from tests.gcs.js_runner import run_node

_UTIL_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
    "startButtonMode.js",
))

# Strip ES module syntax so Node.js can eval the code.
_JS_SRC = (
    open(_UTIL_PATH, encoding="utf-8").read()
    .replace("export function ", "function ")
    .replace("export const ", "const ")
)


def _resolve(params_json):
    code = _JS_SRC + f"\nconsole.log(JSON.stringify(resolveStartButtonMode({params_json})));"
    result = run_node(code, timeout=5)
    if result.returncode != 0:
        raise RuntimeError(f"node failed:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestResolveStartButtonMode(unittest.TestCase):
    def test_container_ready_holds_to_arm_never_single_click(self):
        # The whole point: container START, ready, must NOT start on a click.
        result = _resolve('{"armReady": true, "ready": true, "disabled": false}')
        self.assertEqual(result, {"clickToStart": False, "holdEnabled": True, "holdMode": "arm"})

    def test_bungee_ready_still_single_click(self):
        # START MISSION keeps its single-click start (no behaviour change).
        result = _resolve('{"armReady": false, "ready": true, "disabled": false}')
        self.assertEqual(result, {"clickToStart": True, "holdEnabled": False, "holdMode": None})

    def test_container_not_ready_holds_to_force(self):
        result = _resolve('{"armReady": true, "ready": false, "disabled": false}')
        self.assertEqual(result, {"clickToStart": False, "holdEnabled": True, "holdMode": "force"})

    def test_bungee_not_ready_holds_to_force(self):
        result = _resolve('{"armReady": false, "ready": false, "disabled": false}')
        self.assertEqual(result, {"clickToStart": False, "holdEnabled": True, "holdMode": "force"})

    def test_disabled_is_inert_even_when_ready_and_armed(self):
        result = _resolve('{"armReady": true, "ready": true, "disabled": true}')
        self.assertEqual(result, {"clickToStart": False, "holdEnabled": False, "holdMode": None})

    def test_disabled_not_ready_is_inert(self):
        result = _resolve('{"armReady": true, "ready": false, "disabled": true}')
        self.assertEqual(result, {"clickToStart": False, "holdEnabled": False, "holdMode": None})

    def test_defaults_armReady_and_disabled_optional(self):
        # ready alone (armReady/disabled default false) behaves like bungee ready.
        result = _resolve('{"ready": true}')
        self.assertEqual(result, {"clickToStart": True, "holdEnabled": False, "holdMode": None})

    def test_no_single_click_ever_launches_a_container(self):
        # Property sweep: for a container button (armReady=true), clickToStart is
        # false in every state — no single click can ever launch.
        for ready in (True, False):
            for disabled in (True, False):
                result = _resolve(
                    json.dumps({"armReady": True, "ready": ready, "disabled": disabled})
                )
                self.assertFalse(
                    result["clickToStart"],
                    msg=f"container clickToStart must be False (ready={ready}, disabled={disabled})",
                )


if __name__ == "__main__":
    unittest.main()
