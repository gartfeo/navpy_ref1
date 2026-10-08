"""Tests for missionStatus.js — UAV mission lifecycle status computation."""
import unittest
from tests.gcs.js_runner import run_node
import json
import os
import re


_UTILS_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__),
    "..", "..", "src", "gcs", "frontend", "src", "utils",
))

_JS_PATH = os.path.join(_UTILS_DIR, "missionStatus.js")


def _strip_es_modules(src):
    """Remove ES module import/export syntax so Node.js can eval the code."""
    lines = src.split("\n")
    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("import "):
            continue
        if re.match(r"^export\s*\{.*\}\s*from\s", stripped):
            continue
        line = re.sub(r"^export\s+(function|const|let|var)\s", r"\1 ", line)
        out.append(line)
    return "\n".join(out)


# Inline the colors object that missionStatus.js imports
_COLORS_PREAMBLE = """
const colors = {
  textDim: '#8899aa',
  success: '#4caf50',
  accent: '#00d2ff',
  warning: '#ff9800',
  error: '#f44336',
};
"""

_JS_CODE = _COLORS_PREAMBLE + _strip_es_modules(
    open(_JS_PATH, encoding="utf-8").read()
)


def _run_js(snippet):
    """Evaluate JS snippet with missionStatus loaded, return parsed JSON."""
    full = _JS_CODE + "\n" + snippet
    result = run_node(full, timeout=10)
    if result.returncode != 0:
        raise RuntimeError(f"Node.js error:\n{result.stderr}")
    return json.loads(result.stdout.strip())


class TestConfirmedStatus(unittest.TestCase):
    """Priority 1: assignment with status 'confirmed'."""

    def test_confirmed_assignment(self):
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 5 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'confirmed' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 3)));
        """)
        self.assertEqual(r["status"], "confirmed")
        self.assertEqual(r["labelKey"], "missionStatus.confirmed")
        self.assertEqual(r["color"], "#f44336")

    def test_confirmed_even_when_disarmed(self):
        r = _run_js("""
        const v = { mode: 'MANUAL', armed: false, mission_progress: 0 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'confirmed' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 0)));
        """)
        self.assertEqual(r["status"], "confirmed")


class TestConfirmingSyncedWithPopup(unittest.TestCase):
    """CONFIRMING is driven by hasPendingConfirm (popup visible), not assignment status."""

    def test_pending_confirm_shows_confirming(self):
        """When confirm popup is active, UAV card shows CONFIRMING."""
        r = _run_js("""
        const v = { mode: 'GUIDED', armed: true, mission_progress: 5 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'assigned' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 3, true)));
        """)
        self.assertEqual(r["status"], "confirming")

    def test_no_pending_confirm_shows_approaching(self):
        """Without confirm popup, GUIDED + assigned = approaching."""
        r = _run_js("""
        const v = { mode: 'GUIDED', armed: true, mission_progress: 5 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'assigned' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 3, false)));
        """)
        self.assertEqual(r["status"], "approaching")

    def test_assignment_confirming_status_without_popup_shows_assigned(self):
        """Assignment status 'confirming' without popup shows vehicle state (assigned)."""
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 5 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'confirming' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 3, false)));
        """)
        self.assertEqual(r["status"], "assigned")


class TestApproachingStatus(unittest.TestCase):
    """Priority 3: assignment status 'assigned' + GUIDED mode."""

    def test_assigned_in_guided_mode(self):
        r = _run_js("""
        const v = { mode: 'GUIDED', armed: true, mission_progress: 5 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'assigned' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 3)));
        """)
        self.assertEqual(r["status"], "approaching")
        self.assertEqual(r["labelKey"], "missionStatus.approaching")
        self.assertEqual(r["color"], "#ff9800")

    def test_assigned_in_auto_mode_not_approaching(self):
        """If mode is AUTO (not GUIDED), should fall through to 'assigned'."""
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 5 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'assigned' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 3)));
        """)
        self.assertEqual(r["status"], "assigned")


class TestWaitingStatus(unittest.TestCase):
    """Step 3 seen, owner's APPLIED not yet: the helper does not fly the task
    (docs/design/swarm-task-assignment-ack.md, decision 3)."""

    def test_step_3_shows_waiting(self):
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 5 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'waiting' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 3)));
        """)
        self.assertEqual(r["status"], "waiting")
        self.assertEqual(r["labelKey"], "missionStatus.waiting")
        self.assertEqual(r["color"], "#8899aa")

    def test_waiting_is_never_approaching(self):
        """The waiting check comes before the GUIDED check."""
        r = _run_js("""
        const v = { mode: 'GUIDED', armed: true, mission_progress: 5 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'waiting' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 3)));
        """)
        self.assertEqual(r["status"], "waiting")

    def test_pending_confirm_still_shows_confirming(self):
        r = _run_js("""
        const v = { mode: 'GUIDED', armed: true, mission_progress: 5 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'waiting' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 3, true)));
        """)
        self.assertEqual(r["status"], "confirming")


class TestAssignedStatus(unittest.TestCase):
    """Priority 4: assignment with status 'assigned' (non-GUIDED)."""

    def test_assigned_status(self):
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 5 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'assigned' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 3)));
        """)
        self.assertEqual(r["status"], "assigned")
        self.assertEqual(r["labelKey"], "missionStatus.assigned")
        self.assertEqual(r["color"], "#00d2ff")

    def test_assigned_in_manual_mode(self):
        r = _run_js("""
        const v = { mode: 'MANUAL', armed: true, mission_progress: 0 };
        const a = { taskId: 1, taskType: 'dock', lat: 40, lon: 44, status: 'assigned' };
        console.log(JSON.stringify(computeMissionStatus(v, a, 0)));
        """)
        self.assertEqual(r["status"], "assigned")


class TestIdleStatus(unittest.TestCase):
    """Priority 5: not armed, no assignment."""

    def test_disarmed_no_assignment(self):
        r = _run_js("""
        const v = { mode: 'MANUAL', armed: false, mission_progress: 0 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 0)));
        """)
        self.assertEqual(r["status"], "idle")
        self.assertEqual(r["labelKey"], "missionStatus.idle")
        self.assertEqual(r["color"], "#8899aa")

    def test_disarmed_auto_mode(self):
        """Even in AUTO, disarmed means idle."""
        r = _run_js("""
        const v = { mode: 'AUTO', armed: false, mission_progress: 5 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 3)));
        """)
        self.assertEqual(r["status"], "idle")


class TestEnRouteStatus(unittest.TestCase):
    """Priority 6: AUTO mode, progress > 0, but before search waypoints."""

    def test_en_route_before_search_wps(self):
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 2 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 5)));
        """)
        self.assertEqual(r["status"], "en_route")
        self.assertEqual(r["labelKey"], "missionStatus.enRoute")
        self.assertEqual(r["color"], "#8899aa")

    def test_en_route_at_wp_just_before_offset(self):
        """mission_progress=4, wpOffset=5 => 4-5 = -1 < 0 => en_route."""
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 4 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 5)));
        """)
        self.assertEqual(r["status"], "en_route")

    def test_at_offset_boundary_is_searching(self):
        """mission_progress=5, wpOffset=5 => 5-5 = 0, NOT < 0 => searching."""
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 5 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 5)));
        """)
        self.assertEqual(r["status"], "searching")

    def test_past_offset_is_searching(self):
        """mission_progress=8, wpOffset=5 => 8-5 = 3, NOT < 0 => searching."""
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 8 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 5)));
        """)
        self.assertEqual(r["status"], "searching")


class TestSearchingStatus(unittest.TestCase):
    """Priority 7: AUTO mode (armed, past pre-search waypoints)."""

    def test_searching_auto_armed(self):
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 10 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 3)));
        """)
        self.assertEqual(r["status"], "searching")
        self.assertEqual(r["labelKey"], "missionStatus.searching")
        self.assertEqual(r["color"], "#4caf50")

    def test_searching_auto_zero_progress(self):
        """mission_progress=0, wpOffset=5 => progress not > 0, skip en_route check => searching."""
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 0 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 5)));
        """)
        self.assertEqual(r["status"], "searching")

    def test_searching_zero_offset(self):
        """wpOffset=0, mission_progress=1 => 1-0=1 >= 0 => searching."""
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 1 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 0)));
        """)
        self.assertEqual(r["status"], "searching")


class TestDefaultIdleStatus(unittest.TestCase):
    """Priority 8: fallback to idle."""

    def test_guided_armed_no_assignment(self):
        r = _run_js("""
        const v = { mode: 'GUIDED', armed: true, mission_progress: 0 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 0)));
        """)
        self.assertEqual(r["status"], "idle")

    def test_manual_armed_no_assignment(self):
        r = _run_js("""
        const v = { mode: 'MANUAL', armed: true, mission_progress: 0 };
        console.log(JSON.stringify(computeMissionStatus(v, null, 0)));
        """)
        self.assertEqual(r["status"], "idle")


class TestSwarmBadge(unittest.TestCase):
    """BUSY / UNKNOWN badge from the snapshot's swarm heartbeat state."""

    def _badge(self, swarm):
        return _run_js(f"""
        console.log(JSON.stringify(computeSwarmBadge({json.dumps(swarm)})));
        """)

    def test_no_badge_before_the_first_beat_or_while_free(self):
        self.assertIsNone(self._badge(None))
        self.assertIsNone(self._badge(
            {"state": "FREE", "boot": 7, "seq": 3, "stale": False},
        ))

    def test_busy(self):
        r = self._badge({"state": "BUSY", "boot": 7, "seq": 3, "stale": False})
        self.assertEqual(r["labelKey"], "vehicle.swarmBusy")
        self.assertEqual(r["titleKey"], "vehicle.swarmBusyTitle")

    def test_stale_is_unknown_whatever_it_said(self):
        for state in ("FREE", "BUSY"):
            r = self._badge({"state": state, "boot": 7, "seq": 3, "stale": True})
            self.assertEqual(r["labelKey"], "vehicle.swarmUnknown", state)
            self.assertEqual(r["titleKey"], "vehicle.swarmUnknownTitle", state)


class TestReturnShape(unittest.TestCase):
    """All returned objects have the correct keys."""

    def test_all_keys_present(self):
        r = _run_js("""
        const v = { mode: 'AUTO', armed: true, mission_progress: 10 };
        const result = computeMissionStatus(v, null, 3);
        console.log(JSON.stringify({
          hasStatus: 'status' in result,
          hasLabel: 'labelKey' in result,
          hasColor: 'color' in result,
          keyCount: Object.keys(result).length,
        }));
        """)
        self.assertTrue(r["hasStatus"])
        self.assertTrue(r["hasLabel"])
        self.assertTrue(r["hasColor"])
        self.assertEqual(r["keyCount"], 3)


if __name__ == "__main__":
    unittest.main()
