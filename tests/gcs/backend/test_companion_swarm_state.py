"""The GCS reads each vehicle's FREE/BUSY from its own companion's beat."""
from unittest.mock import MagicMock

import pytest

from gcs.backend.companion_identity import COMPANION_COMPONENT_ID
from gcs.backend.companion_swarm_state import CompanionSwarmState
from gcs.backend.vehicle_manager import VehicleEntry
from navpy.modules.comm.messages.msg_meta import MsgMeta
from navpy.modules.comm.messages.swarm_heartbeat_msg import SwarmHeartbeatMsg

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
    }


@pytest.mark.parametrize("code", [1, 5])
def test_any_non_zero_code_reads_busy(swarm, code):
    swarm.on_heartbeat(_beat(state=code))

    assert swarm.snapshot()["state"] == "BUSY"


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
    }


def test_a_new_boot_replaces_the_record(swarm):
    swarm.on_heartbeat(_beat(state=0, boot=7, seq=50))

    swarm.on_heartbeat(_beat(state=1, boot=8, seq=1))

    assert swarm.snapshot() == {
        "state": "BUSY", "boot": 8, "seq": 1, "stale": False,
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
