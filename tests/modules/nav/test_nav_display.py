"""NavPy's display phase and last outcome (docs/design/gcs_mission_phase.md)."""

from __future__ import annotations

import ast
from pathlib import Path
from unittest.mock import Mock

import pytest

from navpy.core.approach.logs.navigation_snap_types import ClosestSnap
from navpy.core.approach.nav_exit_cleanup import NavCleanupResult
from navpy.core.approach.vision_nav import nav_fail_reason as fail_reason
from navpy.core.approach.vision_nav.episode_latch import (
    FinalApproachEpisodeTerminal,
    FinalApproachTerminal,
)
from navpy.core.flow.nav_display import (
    NavDisplay,
    NavDisplayBuilder,
    NavOutcome,
    NavOutcomeLatch,
    NavPhase,
    decode_state_bits,
    nav_phase,
)
from navpy.core.flow.nav_state import NavigationTaskState, NavState
from navpy.core.flow.nav_transition import NavTransition
from navpy.core.kernel import runtime_time as rt
from navpy.core.link.swarm_heartbeat_msg import SwarmNodeState
from navpy.core.ports.dock_contact import DockContactEvent, DockContactIdentity
from navpy.core.swarm.task_actor_slots import SelectedTaskSlot

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "src"
IDENTITY = DockContactIdentity(vehicle=2, task_id=11, dock_uid=0, episode=3)


def _heartbeat_state(slot):
    """The byte as TaskActor wires it into the heartbeat."""
    return int(slot.node_state()) | slot.display_bits


# Phase table, first match wins


@pytest.mark.parametrize("committed,peer,returning,phase", [
    (NavState.CONFIRM, True, True, NavPhase.CONFIRM),
    (NavState.NAV, True, True, NavPhase.APPROACH),
    # A homing UAV diverted to a peer task flies the task.
    (NavState.DETECT, True, True, NavPhase.TO_TASK),
    (NavState.DETECT, False, True, NavPhase.HOMING),
    (NavState.DETECT, False, False, NavPhase.SEARCH),
    (NavState.RESET, False, False, NavPhase.RECOVERING),
    (NavState.RECOVERY, False, True, NavPhase.RECOVERING),
    # Before the detect waypoint, mode not active, one-shot end.
    (NavState.ONHOLD, True, True, NavPhase.HOLD),
    (NavState.DOCKED, False, False, NavPhase.HOLD),
])
def test_phase_table(committed, peer, returning, phase):
    assert nav_phase(committed, peer, returning) is phase


def test_hub_orbit_reads_homing_then_confirm_then_approach():
    task = NavigationTaskState()
    builder = NavDisplayBuilder(NavOutcomeLatch(), task, lambda: True)

    phases = [
        builder.build(state).phase
        for state in (NavState.DETECT, NavState.CONFIRM, NavState.NAV)
    ]

    assert phases == [NavPhase.HOMING, NavPhase.CONFIRM, NavPhase.APPROACH]


def test_builder_reads_peer_navigation_when_it_builds():
    task = NavigationTaskState()
    builder = NavDisplayBuilder(NavOutcomeLatch(), task, lambda: False)
    assert builder.build(NavState.DETECT).phase is NavPhase.SEARCH

    task.peer_navigation = True

    assert builder.build(NavState.DETECT).phase is NavPhase.TO_TASK


# NAV exit writes the outcome latch


def _exit(latch, *, contact=None, terminal=None):
    cleanup = Mock()
    cleanup.run.return_value = NavCleanupResult(
        snap=ClosestSnap(dist=0.7),
        errors=(),
        own_dock_contact=contact,
        dock_terminal=terminal,
    )
    transition = NavTransition(
        Mock(), cleanup, Mock(), Mock(return_value=()),
        NavigationTaskState(final_approach_navigation_active=True),
        lambda: False, logger=Mock(), outcome=latch,
    )
    return transition.exit(NavState.RESET)


def _contact():
    return DockContactEvent(IDENTITY, 5.0, 0.1, 0.1, 0.0)


def _failed(reason):
    return FinalApproachEpisodeTerminal(
        IDENTITY, FinalApproachTerminal.FAILED, reason,
    )


def test_a_contact_latches_docked():
    latch = NavOutcomeLatch()

    _exit(latch, contact=_contact())

    assert latch.last == NavOutcome(docked=True)


def test_a_failed_dock_latches_its_reason():
    latch = NavOutcomeLatch()

    _exit(latch, terminal=_failed(fail_reason.MISSED_DOCK))

    assert latch.last == NavOutcome(docked=False, reason="missed_dock")


def test_an_unnamed_failure_latches_nav_fail():
    latch = NavOutcomeLatch()

    _exit(latch, terminal=_failed(None))

    assert latch.last == NavOutcome(docked=False, reason="nav_fail")


def test_an_exit_without_an_outcome_keeps_the_last_one():
    latch = NavOutcomeLatch()
    _exit(latch, contact=_contact())

    _exit(latch)  # the real build: no armed episode, no terminal

    assert latch.last == NavOutcome(docked=True)


def test_only_the_next_outcome_replaces_it():
    latch = NavOutcomeLatch()
    _exit(latch, contact=_contact())

    _exit(latch, terminal=_failed(fail_reason.NO_DETECTION))

    assert latch.last == NavOutcome(docked=False, reason="no_detection")


def test_a_failed_exit_latches_nothing():
    latch = NavOutcomeLatch()
    cleanup = Mock()
    cleanup.run.return_value = NavCleanupResult(
        snap=None, errors=(RuntimeError("cleanup"),),
        own_dock_contact=_contact(), dock_terminal=None,
    )
    transition = NavTransition(
        Mock(), cleanup, Mock(), Mock(return_value=()),
        NavigationTaskState(final_approach_navigation_active=True),
        lambda: False, outcome=latch,
    )

    with pytest.raises(Exception):
        transition.exit(NavState.RESET)

    assert latch.last is None


# The outcome survives every state between two beats


@pytest.mark.parametrize("states", [
    # Cyclic: DOCKED's reset, the post-dock climb, back to search.
    (NavState.DOCKED, NavState.RECOVERY, NavState.DETECT),
    # Cyclic, fast: the next approach starts before the next beat.
    (NavState.DOCKED, NavState.DETECT, NavState.CONFIRM, NavState.NAV),
    # Disarm or manual mode.
    (NavState.DOCKED, NavState.ONHOLD),
    (NavState.DOCKED, NavState.RECOVERY, NavState.ONHOLD),
])
def test_the_outcome_is_carried_by_the_next_beat(states):
    latch = NavOutcomeLatch()
    slot = SelectedTaskSlot(rt.RLock())
    builder = NavDisplayBuilder(latch, NavigationTaskState(), lambda: False)
    latch.record_docked()

    for state in states:
        bits = builder.build(state).state_bits()
        slot.set_participation(
            state is NavState.NAV, state is NavState.DOCKED, bits,
        )

    beat = decode_state_bits(_heartbeat_state(slot))
    assert beat.outcome == "docked"
    assert beat.phase == nav_phase(states[-1], False, False).name


# The state byte


@pytest.mark.parametrize("busy", [0, 1])
@pytest.mark.parametrize("display,code", [
    (NavDisplay(), 0x00),
    (NavDisplay(NavPhase.HOLD), 0x02),
    (NavDisplay(NavPhase.SEARCH), 0x04),
    (NavDisplay(NavPhase.TO_TASK), 0x06),
    (NavDisplay(NavPhase.HOMING), 0x08),
    (NavDisplay(NavPhase.CONFIRM), 0x0A),
    (NavDisplay(NavPhase.APPROACH), 0x0C),
    (NavDisplay(NavPhase.RECOVERING), 0x0E),
    (NavDisplay(NavPhase.HOLD, NavOutcome(True)), 0x12),
    (NavDisplay(NavPhase.HOLD, NavOutcome(False, "nav_fail")), 0x32),
    (NavDisplay(NavPhase.SEARCH, NavOutcome(False, "source_event_pump")), 0xB4),
    # An unknown reason is failed (2), never "no outcome" (0).
    (NavDisplay(NavPhase.SEARCH, NavOutcome(False, "made_up")), 0x24),
    (NavDisplay(NavPhase.SEARCH, NavOutcome(False, None)), 0x24),
])
def test_fixed_byte_values(display, code, busy):
    slot = SelectedTaskSlot(rt.RLock())
    slot.set_participation(bool(busy), False, display.state_bits())

    assert _heartbeat_state(slot) == code | busy


def test_a_held_task_sets_busy_beside_the_display_bits():
    slot = SelectedTaskSlot(rt.RLock())
    slot.set_participation(False, False, NavDisplay(NavPhase.SEARCH).state_bits())

    assert SwarmNodeState.from_code(_heartbeat_state(slot)) is SwarmNodeState.FREE
    slot.set_participation(True, False, NavDisplay(NavPhase.APPROACH).state_bits())
    assert SwarmNodeState.from_code(_heartbeat_state(slot)) is SwarmNodeState.BUSY


@pytest.mark.parametrize("code", range(256))
def test_from_code_reads_bit_zero_only(code):
    expected = SwarmNodeState.BUSY if code & 1 else SwarmNodeState.FREE
    assert SwarmNodeState.from_code(code) is expected


@pytest.mark.parametrize("name,index", [
    ("nav_fail", 1),
    ("no_detection", 2),
    ("missed_dock", 3),
    ("navigation_rejected", 4),
    ("source_command_stale", 5),
    ("discontinuity_source_missing", 6),
    ("final_approach_record_unavailable", 7),
    ("source_event_active_poi_missing", 8),
    ("source_event_pump", 9),
])
def test_reason_numbers_are_pinned(name, index):
    assert fail_reason.NAV_FAIL_REASON_INDEX[name] == index
    beat = decode_state_bits(NavDisplay(
        NavPhase.HOLD, NavOutcome(False, name),
    ).state_bits())
    assert (beat.outcome, beat.fail_reason) == ("failed", name)


def test_the_reason_table_has_no_other_entries():
    assert len(fail_reason.NAV_FAIL_REASON_INDEX) == 9
    assert max(fail_reason.NAV_FAIL_REASON_INDEX.values()) <= (
        fail_reason.MAX_REASON_INDEX
    )


@pytest.mark.parametrize("outcome_code", [2, 12, 13, 14, 15])
def test_index_zero_and_spare_indexes_decode_unknown(outcome_code):
    beat = decode_state_bits(outcome_code << 4)
    assert (beat.outcome, beat.fail_reason) == ("failed", "unknown")


def test_no_phase_decodes_none():
    assert decode_state_bits(0x01).phase is None


# Call sites and the import guard


_FAIL_CALLS = frozenset({
    "mark_failed", "_mark_failed", "_mark_failed_and_log", "latch_failure",
})


def test_every_nav_fail_call_site_passes_a_table_constant():
    literal = []
    for path in (SRC / "navpy").rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not node.args:
                continue
            name = getattr(node.func, "attr", getattr(node.func, "id", ""))
            first = node.args[0]
            if (
                name in _FAIL_CALLS
                and isinstance(first, ast.Constant)
                and isinstance(first.value, str)
            ):
                literal.append(f"{path.relative_to(ROOT).as_posix()}:{node.lineno}")
    assert literal == []


# The display is shown, never acted on (Pure Vision Approach Constraint):
# only the post-commit hook, NAV exit's latch, the heartbeat path and the
# GCS read it.
DISPLAY_IMPORTERS = frozenset({
    "src/navpy/core/flow/nav_display.py",
    "src/navpy/core/flow/nav_transition.py",
    "src/navpy/core/swarm/nav_network.py",
    "src/navpy/app/flow/nav_runtime_composition.py",
    "src/gcs/backend/companion_swarm_state.py",
})


def _imports_display(tree: ast.AST) -> bool:
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module and (
            node.module.endswith("nav_display")
        ):
            return True
        if isinstance(node, ast.Import) and any(
            alias.name.endswith("nav_display") for alias in node.names
        ):
            return True
    return False


def test_only_the_display_path_imports_nav_display():
    importers = {
        path.relative_to(ROOT).as_posix()
        for path in SRC.rglob("*.py")
        if _imports_display(ast.parse(path.read_text(encoding="utf-8")))
    }
    assert importers <= DISPLAY_IMPORTERS
    assert "src/navpy/core/swarm/nav_network.py" in importers
