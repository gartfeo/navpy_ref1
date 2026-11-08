"""The companion heartbeat is paced in aircraft time (heartbeat_pacing.py; Demo's ruling on Codex round 2, step 7).

The autopilot's NavPy watchdog (scripts/lua/nav_watchdog.lua) uses a fixed timeout in aircraft time, so at a SITL
speed-up the heartbeat's wall period must shrink by it.
"""
from __future__ import annotations

import threading
from types import SimpleNamespace
from unittest.mock import MagicMock

from navpy.plugins.vehicle.ardupilot.heartbeat_pacing import SIM_SPEEDUP_PARAM, HeartbeatPacing
from navpy.plugins.vehicle.ardupilot.heartbeat_runtime import HeartbeatRuntime
from navpy.plugins.vehicle.ardupilot.link_state import HeartbeatState
from navpy.plugins.vehicle.ardupilot.parameter_repository import ParameterRepository
from navpy.plugins.vehicle.ardupilot.vehicle_protocol_composition import build_protocol_parts


class RecordingWake:
    """The pacing's wake event: records each wait's timeout and returns at once."""

    def __init__(self) -> None:
        self.timeouts: list[float] = []

    def clear(self) -> None:
        pass

    def set(self) -> None:
        pass

    def wait(self, timeout: float) -> bool:
        self.timeouts.append(round(timeout, 9))
        return False


def _pacing(speedup: object) -> tuple[HeartbeatPacing, RecordingWake]:
    pacing = HeartbeatPacing(lambda: speedup)
    wake = RecordingWake()
    pacing._wake = wake
    return pacing, wake


def test_the_wall_wait_is_the_period_divided_by_sim_speedup():
    pacing, wake = _pacing(10.0)
    pacing.wait(threading.Event(), 1.0)
    assert wake.timeouts == [0.1]


def test_without_sim_speedup_the_wait_is_the_period():
    """Real hardware has no SIM_SPEEDUP; a nonsense value counts as 1 too."""
    for value in (None, 0.0, -2.0, float("nan"), True):
        pacing, wake = _pacing(value)
        pacing.wait(threading.Event(), 1.0)
        assert wake.timeouts == [1.0]


def test_stop_ends_the_wait_at_once():
    stop = threading.Event()
    stop.set()
    pacing, wake = _pacing(1.0)
    pacing.wait(stop, 1.0)
    assert wake.timeouts == []


def test_a_speedup_change_ends_the_wait_at_once():
    """NAV raises SIM_SPEEDUP from 1 to 10 in the middle of a 1 s wait: the heartbeat goes out now, not after the
    rest of the old wait (which would space two heartbeats up to 10 s of aircraft time)."""
    pacing = HeartbeatPacing(lambda: 1.0)
    returned = threading.Event()
    waiter = threading.Thread(target=lambda: (pacing.wait(threading.Event(), 30.0), returned.set()))
    waiter.start()
    assert not returned.wait(0.1)
    pacing.wake()
    assert returned.wait(1.0)
    waiter.join(1.0)


def test_at_sim_speedup_the_heartbeat_is_sent_evenly_in_aircraft_time():
    """A running HeartbeatRuntime, 10 Hz (0.1 s of aircraft time) at SIM_SPEEDUP 20: sent every 5 ms of wall time,
    so about 40 times in 0.2 s; paced in wall time it would be 2 or 3."""
    sends: list[int] = []
    sent_enough = threading.Event()

    class Transport:
        def call(self, action):
            sends.append(1)
            if len(sends) >= 20:
                sent_enough.set()
            return action(SimpleNamespace(mav=MagicMock()))

    repository = ParameterRepository()
    pacing = HeartbeatPacing(lambda: repository.get(SIM_SPEEDUP_PARAM))
    repository.watch(SIM_SPEEDUP_PARAM, pacing.wake)
    runtime = HeartbeatRuntime(Transport(), SimpleNamespace(mav_type=18, target_system=7), HeartbeatState(7, 3.0),
                               SimpleNamespace(value=MagicMock()), 10.0, pacing)
    runtime.start()
    try:
        assert not sent_enough.wait(0.3)                  # speed-up 1: 0.1 s apart
        repository.update(SIM_SPEEDUP_PARAM, 20.0)        # the PARAM_VALUE NAV's speed-up lease brings back
        assert sent_enough.wait(1.0)
    finally:
        runtime.close()


def test_the_vehicle_paces_its_heartbeat_by_the_cached_sim_speedup():
    """The composition: the heartbeat reads SIM_SPEEDUP from the vehicle's parameter cache and wakes on its
    update."""
    repository = ParameterRepository()
    link = SimpleNamespace(transport=MagicMock(), identity=SimpleNamespace(target_system=7, mav_type=18),
                           logger_ref=SimpleNamespace(value=MagicMock()), active_bus=MagicMock(),
                           bus_lease=MagicMock())
    state = SimpleNamespace(messages=MagicMock(), mission_inbox=MagicMock(), heartbeat_state=HeartbeatState(7, 3.0),
                            callbacks=MagicMock(), parameter_repository=repository, packet_loss=MagicMock())
    request = SimpleNamespace(link=SimpleNamespace(target_system=7),
                              startup=SimpleNamespace(stop_requested=None, heartbeat_hz=1.0))
    pacing = build_protocol_parts(request, SimpleNamespace(link=link, state=state)).heartbeat_runtime._pacing
    assert pacing._read_speedup() is None                 # real hardware until SIM_SPEEDUP is seen
    repository.update(SIM_SPEEDUP_PARAM, 10.0)
    assert pacing._read_speedup() == 10.0 and pacing._wake.is_set()


def test_a_watched_parameter_update_calls_its_watcher_outside_the_lock():
    repository = ParameterRepository()
    seen: list[float | None] = []
    repository.watch("sim_speedup", lambda: seen.append(repository.get(SIM_SPEEDUP_PARAM)))   # re-enters the lock
    repository.update("SIM_SPEEDUP\x00", 4.0)
    repository.update("OTHER", 1.0)
    assert seen == [4.0]
