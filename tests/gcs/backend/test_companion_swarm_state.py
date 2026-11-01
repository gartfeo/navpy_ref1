"""The GCS reads each vehicle's FREE/BUSY and NavPy display phase from its
own companion's beat (docs/design/gcs_mission_phase.md)."""
from unittest.mock import MagicMock

import pytest

from gcs.backend.companion_identity import COMPANION_COMPONENT_ID
from gcs.backend.companion_swarm_state import CompanionSwarmState
from gcs.backend.vehicle_manager import VehicleEntry
from navpy.core.link.msg_meta import MsgMeta
from navpy.core.link.swarm_heartbeat_msg import SwarmHeartbeatMsg
from navpy.core.flow.nav_display import NavDisplay, NavOutcome, NavPhase

SYS_ID = 2
AUTOPILOT_COMPONENT_ID = 1


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now


def _beat(state=0, boot=7, seq=1, ttl_ms=5000, sys_id=SYS_ID,
          component=COMPANION_COMPONENT_ID):
    msg = SwarmHeartbeatMsg(
        sender_id=sys_id,
        state=state,
        meta=MsgMeta(boot_id=boot, msg_seq=seq, time_ms=0, ttl_ms=ttl_ms),
    ).to_mavlink()
    msg.get_srcSystem = MagicMock(return_value=sys_id)
    msg.get_srcComponent = MagicMock(return_value=component)
    return msg


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def swarm(clock):
    return CompanionSwarmState(SYS_ID, monotonic_s=clock)


def test_no_state_before_the_first_beat(swarm):
    assert swarm.snapshot() is None


def test_own_companion_beat_sets_the_state(swarm):
    swarm.on_heartbeat(_beat(state=0, boot=7, seq=1))

    assert swarm.snapshot() == {
        "state": "FREE", "boot": 7, "seq": 1, "stale": False,
        "phase": None, "outcome": None, "fail_reason": None,
        "approach_seq": None,
    }


@pytest.mark.parametrize("code,state", [
    (1, "BUSY"), (5, "BUSY"), (0xFF, "BUSY"), (4, "FREE"), (0xFE, "FREE"),
])
def test_bit_zero_alone_reads_busy(swarm, code, state):
    swarm.on_heartbeat(_beat(state=code))

    assert swarm.snapshot()["state"] == state


@pytest.mark.parametrize("display,expected", [
    (NavDisplay(NavPhase.SEARCH), ("SEARCH", None, None)),
    (NavDisplay(NavPhase.HOMING), ("HOMING", None, None)),
    (NavDisplay(NavPhase.HOLD, NavOutcome(True)), ("HOLD", "docked", None)),
    (
        NavDisplay(NavPhase.HOLD, NavOutcome(False, "no_detection")),
        ("HOLD", "failed", "no_detection"),
    ),
    (
        NavDisplay(NavPhase.SEARCH, NavOutcome(False, "made_up")),
        ("SEARCH", "failed", "unknown"),
    ),
])
@pytest.mark.parametrize("busy", [0, 1])
def test_display_bits_reach_the_snapshot(swarm, display, expected, busy):
    swarm.on_heartbeat(_beat(state=busy | display.state_bits()))

    snapshot = swarm.snapshot()
    assert (
        snapshot["phase"], snapshot["outcome"], snapshot["fail_reason"],
    ) == expected
    assert snapshot["state"] == ("BUSY" if busy else "FREE")


def test_a_late_beat_from_a_retired_boot_is_dropped(swarm):
    failed = NavDisplay(NavPhase.HOLD, NavOutcome(False, "missed_dock"))
    swarm.on_heartbeat(_beat(state=failed.state_bits(), boot=7, seq=9))
    swarm.on_heartbeat(_beat(state=NavDisplay(NavPhase.HOLD).state_bits(),
                             boot=8, seq=1))

    swarm.on_heartbeat(_beat(state=failed.state_bits(), boot=7, seq=10))

    snapshot = swarm.snapshot()
    assert (snapshot["boot"], snapshot["outcome"]) == (8, None)


@pytest.mark.parametrize("source", [
    {"sys_id": 3},
    {"component": AUTOPILOT_COMPONENT_ID},
], ids=["other-companion", "own-autopilot"])
def test_beats_from_other_sources_are_ignored(swarm, source):
    swarm.on_heartbeat(_beat(state=1, **source))

    assert swarm.snapshot() is None


def test_older_or_repeated_beats_are_ignored(swarm):
    swarm.on_heartbeat(_beat(state=0, seq=5))

    swarm.on_heartbeat(_beat(state=1, seq=4))
    swarm.on_heartbeat(_beat(state=1, seq=5))

    assert swarm.snapshot() == {
        "state": "FREE", "boot": 7, "seq": 5, "stale": False,
        "phase": None, "outcome": None, "fail_reason": None,
        "approach_seq": None,
    }


def test_a_new_boot_replaces_the_record(swarm):
    swarm.on_heartbeat(_beat(state=0, boot=7, seq=50))

    swarm.on_heartbeat(_beat(state=1, boot=8, seq=1))

    assert swarm.snapshot() == {
        "state": "BUSY", "boot": 8, "seq": 1, "stale": False,
        "phase": None, "outcome": None, "fail_reason": None,
        "approach_seq": None,
    }


def test_state_is_stale_once_the_beat_outlives_its_ttl(swarm, clock):
    swarm.on_heartbeat(_beat(seq=1, ttl_ms=5000))

    clock.now += 5.0
    assert swarm.snapshot()["stale"] is False
    clock.now += 0.001
    assert swarm.snapshot()["stale"] is True

    swarm.on_heartbeat(_beat(seq=2, ttl_ms=5000))
    assert swarm.snapshot()["stale"] is False


def test_vehicle_entry_reports_its_companion_swarm_state():
    vehicle = MagicMock()
    callbacks = {}
    vehicle.on_message = lambda name, cb: callbacks.setdefault(name, []).append(cb)
    entry = VehicleEntry(SYS_ID, vehicle, "uav2")
    assert entry.snapshot()["swarm"] is None

    beat = _beat(state=1, boot=7, seq=3)
    for callback in callbacks[beat.get_type()]:
        callback(beat)

    swarm = entry.snapshot()["swarm"]
    assert (swarm["state"], swarm["boot"], swarm["seq"]) == ("BUSY", 7, 3)


def test_a_phase_change_alone_is_broadcast(swarm):
    from gcs.backend.telemetry_loop import _has_changed

    swarm.on_heartbeat(_beat(state=NavDisplay(NavPhase.SEARCH).state_bits()))
    before = swarm.snapshot()
    after = dict(before, phase="HOMING")

    assert _has_changed({"swarm": before}, {"swarm": after})
    assert not _has_changed({"swarm": after}, {"swarm": dict(after)})


def test_approach_seq_is_the_newest_approach_beat_of_this_boot(swarm):
    phase = lambda p: NavDisplay(p).state_bits()
    swarm.on_heartbeat(_beat(state=phase(NavPhase.SEARCH), boot=7, seq=1))
    assert swarm.snapshot()["approach_seq"] is None

    swarm.on_heartbeat(_beat(state=1 | phase(NavPhase.APPROACH), boot=7, seq=5))
    swarm.on_heartbeat(_beat(state=1 | phase(NavPhase.APPROACH), boot=7, seq=9))
    swarm.on_heartbeat(_beat(state=phase(NavPhase.TO_TASK), boot=7, seq=12))
    assert swarm.snapshot()["approach_seq"] == 9

    swarm.on_heartbeat(_beat(state=phase(NavPhase.SEARCH), boot=8, seq=1))
    assert swarm.snapshot()["approach_seq"] is None
