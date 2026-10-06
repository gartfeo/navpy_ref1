"""Tests for preflightRemediation.js (remediationFor / topIssue). Runs under Node.

The module has no imports, so its source is evaluated standalone with the ES
`export` keywords stripped — matching the project's Node-subprocess test style.
"""
import json
import os
from tests.gcs.js_runner import run_node
import unittest

_PATH = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "src", "gcs", "frontend", "src", "utils",
    "preflightRemediation.js",
))
_SRC = open(_PATH, encoding="utf-8").read().replace("export ", "")


def _run(script):
    r = run_node(_SRC + "\n" + script, timeout=5)
    if r.returncode != 0:
        raise RuntimeError(f"node failed:\n{r.stderr}")
    return r.stdout.strip()


def _rem(check):
    return json.loads(_run(
        f"console.log(JSON.stringify(remediationFor({json.dumps(check)})));"))


def _top(checks):
    return json.loads(_run(
        f"console.log(JSON.stringify(topIssue({json.dumps(checks)})));"))


class TestRemediationFor(unittest.TestCase):
    def test_green_and_na_return_null(self):
        self.assertIsNone(_rem({"key": "gps", "status": "go"}))
        self.assertIsNone(_rem({"key": "ekf", "status": "na"}))

    def test_gps_navigation_only(self):
        r = _rem({"key": "gps", "status": "nogo"})
        self.assertEqual(r["hintKey"], "preflight.fix.gps")
        self.assertIsNone(r["action"])

    def test_ekf_is_cal(self):
        self.assertEqual(_rem({"key": "ekf", "status": "warn"})["action"]["type"], "cal")

    def test_rc_links_radio(self):
        a = _rem({"key": "rc", "status": "nogo"})["action"]
        self.assertEqual(a["type"], "link")
        self.assertEqual(a["tool"], "radio")
        self.assertIn("ardupilot.org", a["url"])

    def test_airspeed_is_cal(self):
        self.assertEqual(_rem({"key": "airspeed", "status": "warn"})["action"]["type"], "cal")

    def test_battery_navigation_only(self):
        self.assertIsNone(_rem({"key": "battery", "status": "nogo"})["action"])

    def test_sensors_mag_links_compass(self):
        a = _rem({"key": "sensors", "status": "nogo", "detail": {"failed": ["mag"]}})["action"]
        self.assertEqual(a["tool"], "compass")

    def test_sensors_gyro_is_cal(self):
        a = _rem({"key": "sensors", "status": "nogo", "detail": {"failed": ["gyro"]}})["action"]
        self.assertEqual(a["type"], "cal")

    def test_prearm_compass_message_links_compass(self):
        r = _rem({"key": "prearm", "status": "nogo",
                  "detail": {"warnings": ["Compass not healthy"], "advisories": []}})
        self.assertEqual(r["action"]["tool"], "compass")

    def test_prearm_gps_message_navigation(self):
        r = _rem({"key": "prearm", "status": "nogo",
                  "detail": {"warnings": ["GPS not healthy"], "advisories": []}})
        self.assertEqual(r["hintKey"], "preflight.fix.gps")
        self.assertIsNone(r["action"])

    def test_prearm_advisory_checks_disabled(self):
        r = _rem({"key": "prearm", "status": "warn",
                  "detail": {"warnings": [], "advisories": ["prearm.checksDisabled"]}})
        self.assertEqual(r["hintKey"], "preflight.fix.checksDisabled")

    def test_companion_down_navigation(self):
        r = _rem({"key": "companion", "status": "nogo", "detail": {"status": "down"}})
        self.assertEqual(r["hintKey"], "preflight.fix.companion")
        self.assertIsNone(r["action"])

    def test_companion_checking_milder_hint(self):
        # Transient 'checking' gets its own softer hint, not the "start it" text.
        r = _rem({"key": "companion", "status": "warn", "detail": {"status": "checking"}})
        self.assertEqual(r["hintKey"], "preflight.fix.companionChecking")
        self.assertIsNone(r["action"])

    def test_mission_navigation_only(self):
        r = _rem({"key": "mission", "status": "nogo", "detail": {"total": None}})
        self.assertEqual(r["hintKey"], "preflight.fix.mission")
        self.assertIsNone(r["action"])

    def test_throttle_navigation_only(self):
        r = _rem({"key": "throttle", "status": "nogo", "detail": {"rc3": 1200}})
        self.assertEqual(r["hintKey"], "preflight.fix.throttle")
        self.assertIsNone(r["action"])


class TestTopIssue(unittest.TestCase):
    def test_none_when_all_clear(self):
        self.assertIsNone(_top([{"key": "gps", "status": "go"}, {"key": "ekf", "status": "na"}]))

    def test_nogo_beats_warn(self):
        r = _top([
            {"key": "ekf", "status": "warn"},
            {"key": "gps", "status": "nogo"},
        ])
        self.assertEqual(r["check"]["key"], "gps")

    def test_priority_breaks_ties(self):
        # prearm outranks battery at the same (nogo) severity.
        r = _top([
            {"key": "battery", "status": "nogo"},
            {"key": "prearm", "status": "nogo", "detail": {"warnings": [], "advisories": []}},
        ])
        self.assertEqual(r["check"]["key"], "prearm")

    def test_companion_outranks_prearm(self):
        # Companion (the launch blocker the operator most needs to see) leads.
        r = _top([
            {"key": "prearm", "status": "nogo", "detail": {"warnings": [], "advisories": []}},
            {"key": "companion", "status": "nogo", "detail": {"status": "down"}},
        ])
        self.assertEqual(r["check"]["key"], "companion")

    def test_new_launch_blocked_rows_are_actionable(self):
        # Guarantee: a NO-GO on any new launch-gate row yields a non-null next
        # step, so the action strip is never empty for these blockers.
        for key, detail in (
            ("companion", {"status": "down"}),
            ("mission", {"total": None}),
            ("throttle", {"rc3": 1200}),
        ):
            with self.subTest(key=key):
                top = _top([{"key": key, "status": "nogo", "detail": detail}])
                self.assertIsNotNone(top)
                self.assertIsNotNone(top["hintKey"])

    def test_unknown_key_sorts_last_not_first(self):
        # A future un-prioritized row must not hijack the next-step ahead of a
        # known launch blocker at equal severity.
        r = _top([
            {"key": "mystery_future_row", "status": "nogo"},
            {"key": "companion", "status": "nogo", "detail": {"status": "down"}},
        ])
        self.assertEqual(r["check"]["key"], "companion")


if __name__ == "__main__":
    unittest.main()
