"""Message registry census: which message classes each entry point registers.

Classes register on import (@register_msg), so a process only knows the
message modules something in it imports. Today the facades pull them all in;
without them the GCS would register 4 types instead of 11 (step 1, c01). Each
census imports one entry point first in a fresh interpreter and compares the
registered class names with the pinned set. Modules are found through the
step-1 manifest, so the census follows them as they move.
"""

from __future__ import annotations

import json

import pytest

from tests.guards.step1_fresh import error_tail, run_fresh
from tests.guards.step1_sources import REPO_ROOT, plan_module

sm = plan_module()
ROWS = sm.load_manifest(REPO_ROOT)

SWARM_MESSAGES = frozenset({
    "AvailableTaskRequestMsg", "AvailableTaskResponseMsg", "CheckInMsg", "CheckOutMsg",
    "SwarmAckMsg", "SwarmHeartbeatMsg", "SwarmRequestMsg", "TaskAssignRequestMsg",
    "TaskAssignResponseMsg", "TaskConfirmRequestMsg", "TaskConfirmResponseMsg",
})
GCS_MESSAGES = SWARM_MESSAGES                         # 11
COMPANION_MESSAGES = SWARM_MESSAGES | {"LogStatusMsg"}  # 12
REGISTRY_MODULE = "navpy.modules.comm.messages.msg_abc"
SWARM_SESSION_MODULE = "navpy.modules.nav.nav_network"  # the companion's swarm session

CENSUS = (
    "import importlib, json, sys\n"
    "importlib.import_module(sys.argv[1])\n"
    "registry = importlib.import_module(sys.argv[2]).MsgRegistry\n"
    "print(json.dumps(sorted(c.__name__ for c in registry._mav_registry.values())))\n"
)


def _now(module: str) -> str:
    """Current dotted name of a module named by its pre-step-1 name."""
    matches = [row for row in ROWS if sm.module_name(sm.NAVPY_ROOT + row.src) == module]
    assert len(matches) == 1, module
    location = sm.current_location(REPO_ROOT, matches[0])
    assert location is not None, f"{module} is in neither its old place nor its step-1 home"
    current = sm.module_name(location)
    assert current is not None
    return current


def _registered(entry_point: str) -> set[str]:
    result = run_fresh(CENSUS, entry_point, _now(REGISTRY_MODULE))
    assert result.returncode == 0, f"{entry_point}: {error_tail(result, 10)}"
    return set(json.loads(result.stdout.strip().splitlines()[-1]))


@pytest.mark.parametrize(("entry_point", "expected"), (
    ("gcs.backend.main", GCS_MESSAGES),
    ("navpy.main", COMPANION_MESSAGES),
))
def test_entry_point_registers_exactly_the_pinned_messages(
        entry_point: str, expected: frozenset[str]) -> None:
    registered = _registered(entry_point)

    assert registered == expected, (
        f"missing {sorted(expected - registered)}, extra {sorted(registered - expected)}")


def test_the_companion_swarm_session_registers_swarm_ack() -> None:
    """The swarm path imports swarm_ack_msg itself (see
    tests/modules/comm/test_swarm_ack_registration.py)."""
    assert "SwarmAckMsg" in _registered(_now(SWARM_SESSION_MODULE))
