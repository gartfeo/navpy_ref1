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
        # Plus assignment, outcome, last and stale (gcs_mission_phase.md).
        self.assertEqual(r["keyCount"], 7)


# -- NavPy phase (docs/design/gcs_mission_phase.md) ---------------------------

def _status(vehicle, assignment="null", pending="false", wp=0):
    return _run_js(f"""
    const v = {vehicle};
    const a = {assignment};
    console.log(JSON.stringify(computeMissionStatus(v, a, {wp}, {pending})));
    """)


_SLOT = ("{{ taskId: 3, taskType: 'dock', lat: 40.01, lon: 44.01, "
         "status: '{status}', doingUid: {{ boot_id: {boot}, msg_seq: 4 }}, "
         "assignedUid: {{ boot_id: {boot}, msg_seq: 4 }} }}")


def _slot(status="assigned", boot=7):
    return _SLOT.format(status=status, boot=boot)


def _v(phase, *, mode="GUIDED", armed="true", progress=5, boot=7,
       outcome="null", reason="null", stale="false", approach_seq="null"):
    phase_js = "null" if phase is None else f"'{phase}'"
    return (f"{{ mode: '{mode}', armed: {armed}, mission_progress: {progress}, "
            f"lat: 40, lon: 44, swarm: {{ state: 'FREE', phase: {phase_js}, "
            f"outcome: {outcome}, fail_reason: {reason}, boot: {boot}, "
            f"approach_seq: {approach_seq}, seq: 1, stale: {stale} }} }}")


class TestPhaseBadges(unittest.TestCase):
    """With swarm.phase the badge comes from NavPy, not the guess."""

    def test_owner_reviews_its_own_dock_with_no_assignment(self):  # todo 60
        self.assertEqual(_status(_v("CONFIRM"))["status"], "confirming")

    def test_owner_approaching_its_own_dock_is_confirmed(self):  # todo 60
        self.assertEqual(_status(_v("APPROACH"))["status"], "confirmed")

    def test_homing(self):
        r = _status(_v("HOMING", mode="GUIDED"))
        self.assertEqual(r["status"], "homing")
        self.assertEqual(r["labelKey"], "missionStatus.homing")

    def test_helper_to_task_then_confirmed(self):  # todo 61
        to_task = _status(_v("TO_TASK"), _slot("assigned"))
        self.assertEqual(to_task["status"], "approaching")
        self.assertEqual(to_task["assignment"]["taskId"], 3)
        confirmed = _status(_v("APPROACH"), _slot("assigned"))
        self.assertEqual(confirmed["status"], "confirmed")
        self.assertIsNone(confirmed["assignment"])

    def test_search_reads_searching_or_en_route_by_progress(self):
        self.assertEqual(_status(_v("SEARCH", mode="AUTO", progress=5), wp=2)["status"],
                         "searching")
        self.assertEqual(_status(_v("SEARCH", mode="AUTO", progress=1), wp=3)["status"],
                         "en_route")

    def test_recovering(self):
        self.assertEqual(_status(_v("RECOVERING"))["status"], "recovering")

    def test_docked_after_disarm(self):  # todo 63
        r = _status(_v("HOLD", armed="false", outcome="'docked'"))
        self.assertEqual(r["status"], "docked")
        self.assertEqual(r["labelKey"], "missionStatus.docked")
        self.assertIsNone(r["outcome"]["reasonKey"])

    def test_dock_failed_with_reason(self):  # todo 76
        r = _status(_v("HOLD", armed="false", outcome="'failed'",
                       reason="'missed_dock'"))
        self.assertEqual(r["status"], "dock_failed")
        self.assertEqual(r["outcome"]["reasonKey"], "missionStatus.reasons.missed_dock")

    def test_dock_failed_unknown_reason(self):
        r = _status(_v("RECOVERING", outcome="'failed'", reason="'unknown'"))
        self.assertEqual(r["status"], "dock_failed")
        self.assertEqual(r["outcome"]["reasonKey"], "missionStatus.reasons.unknown")

    def test_searching_with_a_last_outcome(self):
        r = _status(_v("SEARCH", mode="AUTO", outcome="'docked'"))
        self.assertEqual(r["status"], "searching")
        self.assertEqual(r["last"]["kind"], "docked")
        self.assertIsNone(r["outcome"])

    def test_stale_end_state_is_kept_and_marked(self):
        r = _status(_v("HOLD", armed="false", mode="MANUAL", outcome="'docked'",
                       stale="true"))
        self.assertEqual(r["status"], "docked")
        self.assertTrue(r["stale"])

    def test_stale_phase_with_no_outcome_falls_back_to_the_guess(self):
        r = _status(_v("APPROACH", mode="AUTO", stale="true"), wp=0)
        self.assertEqual(r["status"], "searching")
        self.assertFalse(r["stale"])

    def test_hold_without_outcome_guesses_from_mode_only(self):
        r = _status(_v("HOLD", mode="GUIDED"), _slot("confirmed"))
        self.assertEqual(r["status"], "idle")
        self.assertIsNone(r["assignment"])


class TestPhaseTaskLabel(unittest.TestCase):
    """Fail-closed task label: ASSIGNED / CONFIRMING from the current boot."""

    def test_confirmed_slot_gives_no_label_or_badge_in_any_phase(self):
        for phase in ("HOLD", "SEARCH", "TO_TASK", "HOMING", "CONFIRM",
                      "APPROACH", "RECOVERING"):
            for boot in (7, 6):
                r = _status(_v(phase), _slot("confirmed", boot))
                self.assertIsNone(r["assignment"], (phase, boot))
                if phase != "APPROACH":
                    self.assertNotEqual(r["status"], "confirmed", (phase, boot))

    def test_older_boot_slot_gives_no_label_after_busy_beats(self):
        for status, phase in (("assigned", "TO_TASK"), ("confirming", "CONFIRM"),
                              ("assigned", "CONFIRM")):
            r = _status(_v(phase, boot=8), _slot(status, boot=7))
            self.assertIsNone(r["assignment"], (status, phase))

    def test_current_boot_confirming_slot_labels_confirm(self):
        r = _status(_v("CONFIRM"), _slot("confirming"))
        self.assertEqual(r["assignment"]["taskId"], 3)

    def test_waiting_slot_gives_no_label_and_no_waiting_badge(self):
        r = _status(_v("SEARCH", mode="AUTO"), _slot("waiting"))
        self.assertEqual(r["status"], "searching")
        self.assertIsNone(r["assignment"])

    def test_current_task_slot_picks_only_current_boot_live_slots(self):
        r = _run_js("""
        const swarm = { phase: 'TO_TASK', boot: 7 };
        const assignments = {
          a: { receiverId: 2, status: 'confirmed', assignedUid: { boot_id: 7 }, taskId: 1 },
          b: { receiverId: 2, status: 'assigned', assignedUid: { boot_id: 6 }, taskId: 2 },
          c: { receiverId: 2, status: 'assigned', assignedUid: { boot_id: 7 }, taskId: 3 },
          d: { receiverId: 3, status: 'confirming', assignedUid: { boot_id: 7 }, taskId: 4 },
        };
        console.log(JSON.stringify(currentTaskSlot(assignments, 2, swarm).taskId));
        """)
        self.assertEqual(r, 3)


class TestNoPhaseFallback(unittest.TestCase):
    """No phase (old companion or trace): today's guess, assignment kept."""

    def test_null_phase_uses_the_guess_with_the_assignment(self):
        r = _status(_v(None), _slot("confirmed"))
        self.assertEqual(r["status"], "confirmed")
        self.assertEqual(r["assignment"]["taskId"], 3)

    def test_old_swarm_snapshot_without_phase_field(self):
        r = _run_js("""
        const v = { mode: 'GUIDED', armed: true, swarm: { state: 'BUSY', boot: 1, seq: 2, stale: false } };
        console.log(JSON.stringify(computeMissionStatus(v, null, 0)));
        """)
        self.assertEqual(r["status"], "idle")


class TestPhaseReviewFixes(unittest.TestCase):
    """Codex final-diff round 1 (gcs_mission_phase.md, Review)."""

    def test_search_reads_searching_in_any_mode(self):
        for mode in ("GUIDED", "LOITER", "AUTO"):
            self.assertEqual(_status(_v("SEARCH", mode=mode))["status"], "searching", mode)

    def test_search_before_the_search_waypoints_is_en_route(self):
        self.assertEqual(_status(_v("SEARCH", mode="AUTO", progress=2), wp=3)["status"],
                         "en_route")
        self.assertEqual(_status(_v("SEARCH", mode="AUTO", progress=3), wp=3)["status"],
                         "searching")

    def test_hold_in_armed_auto_is_en_route_never_searching(self):
        for progress in (0, 5):
            self.assertEqual(_status(_v("HOLD", mode="AUTO", progress=progress))["status"],
                             "en_route", progress)
        self.assertEqual(_status(_v("HOLD", mode="AUTO", armed="false"))["status"], "idle")

    def test_stale_phase_never_restores_an_assignment(self):
        for mode, armed in (("GUIDED", "true"), ("MANUAL", "false")):
            r = _status(_v("HOLD", mode=mode, armed=armed, stale="true"),
                        _slot("assigned"))
            self.assertIn(r["status"], ("idle", "en_route", "searching"), mode)
            self.assertIsNone(r["assignment"], mode)

    def test_slot_answered_before_the_last_approach_gives_no_label(self):
        # Same boot, no FREE beat heard: the old ASSIGNED slot (doing seq 4)
        # predates the approach seen at seq 10.
        for phase in ("TO_TASK", "CONFIRM"):
            r = _status(_v(phase, approach_seq=10), _slot("assigned"))
            self.assertIsNone(r["assignment"], phase)
        r = _run_js("""
        const swarm = { phase: 'TO_TASK', boot: 7, approach_seq: 10 };
        const assignments = {
          a: { receiverId: 2, status: 'assigned', assignedUid: { boot_id: 7, msg_seq: 4 }, taskId: 3 },
        };
        console.log(JSON.stringify(currentTaskSlot(assignments, 2, swarm)));
        """)
        self.assertIsNone(r)

    def test_slot_answered_after_the_last_approach_keeps_its_label(self):
        r = _status(_v("TO_TASK", approach_seq=3), _slot("assigned"))
        self.assertEqual(r["assignment"]["taskId"], 3)


_TAS_PATH = os.path.join(_UTILS_DIR, "taskAssignmentState.js")


class TestApproachFenceWithTheRealReducer(unittest.TestCase):
    """Codex final-diff round 2: a repeated handshake during NAV refreshes the
    slot's "doing" but not when it was first assigned."""

    def test_a_repeated_doing_after_the_approach_does_not_relabel_a_later_task(self):
        reducer = _strip_es_modules(open(_TAS_PATH, encoding="utf-8").read())
        out = run_node(_JS_CODE + "\n" + reducer + """
        const OWNER = 1, HELPER = 2, TASK = 3;
        let s = INITIAL_ASSIGNMENT_STATE;
        const uid = (boot, seq) => ({ boot_id: boot, msg_seq: seq });
        const offer = (seq) => ({ sender_id: OWNER, receiver_id: HELPER, task_id: TASK,
          task_type: 'dock', lat: 40.01, lon: 44.01, uid: uid(9, seq) });
        const doing = (seq) => ({ sender_id: HELPER, receiver_id: OWNER, task_id: TASK,
          is_accepted: true, uid: uid(7, seq) });
        const applied = (ownerSeq, seq) => ({ owner_id: OWNER, helper_id: HELPER,
          task_id: TASK, uid: uid(9, ownerSeq), ref: uid(7, seq) });
        const step = (type, data) => { s = reduceTaskAssignment(s, { type, data }); };
        step('task_assign_request', offer(1));
        step('task_assign_response', doing(4));
        step('task_assign_ack', applied(2, 4));
        // The approach is heard at seq 10; the owner's repeat of the offer
        // gets a new "doing" (seq 11) and APPLIED during NAV.
        step('task_assign_request', offer(3));
        step('task_assign_response', doing(11));
        step('task_assign_ack', applied(4, 11));
        const slot = vehicleAssignment(s.assignments, HELPER);
        const later = { mode: 'GUIDED', armed: true, mission_progress: 5, lat: 40, lon: 44,
          swarm: { state: 'BUSY', phase: 'TO_TASK', outcome: 'docked', boot: 7,
                   approach_seq: 10, seq: 14, stale: false } };
        const r = computeMissionStatus(later, currentTaskSlot(s.assignments, HELPER, later.swarm), 0, false);
        const direct = computeMissionStatus(later, slot, 0, false);
        console.log(JSON.stringify({
          doing: slot && slot.doingUid, assigned: slot && slot.assignedUid,
          status: r.status, label: r.assignment, direct: direct.assignment,
        }));
        """, timeout=10)
        self.assertEqual(out.returncode, 0, out.stderr)
        r = json.loads(out.stdout.strip())
        # The probe reaches the case: the repeat refreshed "doing" only.
        self.assertEqual(r["doing"], {"boot_id": 7, "msg_seq": 11})
        self.assertEqual(r["assigned"], {"boot_id": 7, "msg_seq": 4})
        self.assertEqual(r["status"], "approaching")
        self.assertIsNone(r["label"])
        self.assertIsNone(r["direct"])


if __name__ == "__main__":
    unittest.main()
