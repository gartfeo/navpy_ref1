"""The companion heartbeat is paced in aircraft time (heartbeat_pacing.py; Demo's ruling on Codex round 2, step 7).

The autopilot's NavPy watchdog (scripts/lua/nav_watchdog.lua) uses a fixed timeout in aircraft time, so at a SITL
speed-up the heartbeat's wall period must shrink by it.
"""
from __future__ import annotations

import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import pytest
from pymavlink.dialects.v20.ardupilotmega import MAV_TYPE_GCS, MAV_TYPE_ONBOARD_CONTROLLER

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


# ---------- Owner ruling 2026-11-08: the companion heartbeat is 4 Hz ----------


@pytest.mark.parametrize("mav_type, hz", [(MAV_TYPE_ONBOARD_CONTROLLER, 4.0), (MAV_TYPE_GCS, 1.0)])
def test_the_companion_heartbeats_at_4hz_and_the_gcs_at_1hz(mav_type, hz):
    """NavPy's heartbeat is what the autopilot's NavPy watchdog times; the GCS backend's own (also a VehicleMav)
    keeps 1 Hz on every radio's uplink."""
    from navpy.plugins.vehicle.ardupilot import vehicle_mav

    with patch.object(vehicle_mav, "build_vehicle_parts", side_effect=RuntimeError("stop")) as build:
        with pytest.raises(RuntimeError, match="stop"):
            vehicle_mav.VehicleMav("unused", 7, mav_type=mav_type)
    assert build.call_args.args[7] == hz


def test_the_heartbeat_timeout_warning_stays_once_a_second_at_any_rate():
    """The autopilot silent while the companion heartbeats at 100 Hz: one warning in 0.5 s, as at 1 Hz, not 50."""
    logger = MagicMock()
    sends = threading.Event()

    class Transport:
        def call(self, action):
            sends.set()
            return action(SimpleNamespace(mav=MagicMock()))

    state = HeartbeatState(7, 0.01)
    time.sleep(0.05)                                      # no autopilot heartbeat for longer than its timeout
    runtime = HeartbeatRuntime(Transport(), SimpleNamespace(mav_type=MAV_TYPE_ONBOARD_CONTROLLER, target_system=7),
                               state, SimpleNamespace(value=logger), 100.0)
    runtime.start()
    try:
        assert sends.wait(1.0)
        time.sleep(0.5)
    finally:
        runtime.close()
    warnings = [c.args[0] for c in logger.warning.call_args_list if "heartbeat timeout" in c.args[0]]
    assert len(warnings) == 1, warnings


# ---------- step 10, item 12: SIM_SPEEDUP read before the heartbeat starts ----------


def _activate(mav_type, probe=None):
    from navpy.plugins.vehicle.ardupilot.vehicle_build_transaction import VehicleBuildTransaction

    order: list[str] = []
    transaction = VehicleBuildTransaction.__new__(VehicleBuildTransaction)
    transaction._request = SimpleNamespace(
        startup=SimpleNamespace(wait_heartbeat=False, send_heartbeat=True, skip_mission_download=True),
        link=SimpleNamespace(target_system=1, mav_type=mav_type),
    )
    simulation = Mock(speedup=Mock(side_effect=probe or (lambda: order.append("probe") or 1.0)))
    lifecycle = Mock(start_heartbeat=Mock(side_effect=lambda: order.append("heartbeat")))
    transaction._activate(SimpleNamespace(lifecycle=lifecycle, router=Mock(), mission=Mock()),
                          SimpleNamespace(simulation=simulation))
    return order, simulation


def test_item12_a_companion_asks_for_sim_speedup_before_its_heartbeat_starts():
    """A NavPy restarted mid-flight in SITL: SIM_SPEEDUP is asked for (not waited for) before the first heartbeat,
    so the reply wakes the pacing within milliseconds instead of after the cadence's probe a second later."""
    order, simulation = _activate(MAV_TYPE_ONBOARD_CONTROLLER)
    assert order == ["probe", "heartbeat"]


def test_item12_the_gcs_does_not_ask_and_a_failed_ask_still_starts_the_heartbeat():
    order, simulation = _activate(MAV_TYPE_GCS)
    assert order == ["heartbeat"] and not simulation.speedup.called

    def fail():
        raise OSError("link down")

    order, _simulation = _activate(MAV_TYPE_ONBOARD_CONTROLLER, probe=fail)
    assert order == ["heartbeat"]


def test_item12_the_reply_to_the_ask_wakes_the_pacing():
    """The probe's PARAM_VALUE reaches the cache and wakes a heartbeat already waiting at speed-up 1."""
    repository = ParameterRepository()
    reading = threading.Event()

    def read_speedup():
        # The wait reads the speed-up after clearing its wake and before waiting: from here an update wakes it
        # (Codex round 1 on step 10, finding 4: the update must not come before the waiter read speed-up 1).
        value = repository.get(SIM_SPEEDUP_PARAM)
        reading.set()
        return value

    pacing = HeartbeatPacing(read_speedup)
    repository.watch(SIM_SPEEDUP_PARAM, pacing.wake)
    woke = threading.Event()
    waiter = threading.Thread(target=lambda: (pacing.wait(threading.Event(), 30.0), woke.set()))
    waiter.start()
    assert reading.wait(5.0)                                # the waiter read speed-up 1: a 30 s wait
    repository.update(SIM_SPEEDUP_PARAM, 10.0)              # the reply
    assert woke.wait(1.0)
    waiter.join(1.0)
