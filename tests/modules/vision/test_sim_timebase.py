"""Tests for frequency-only accelerated-SITL scheduler cadence."""

import pytest

from navpy.modules.common.scheduler_cadence import SchedulerCadence
from navpy.modules.common.sim_time import SimTimebase
from navpy.modules.vision.sim.sim_timebase import SimTimebase as VisionSimTimebase


class _FakeClock:
    def __init__(self, value: float) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value

    def advance(self, dt_s: float) -> None:
        self.value += dt_s


def test_scheduler_adapter_exposes_no_clock_and_accelerates_frequency() -> None:
    clock = _FakeClock(100.0)
    cadence = SchedulerCadence(lambda: 10.0, wall_clock=clock)

    clock.advance(0.1)

    assert not hasattr(cadence, "now")
    assert cadence.wall_period_for_scheduler_period(0.1) == pytest.approx(0.01)


def test_speed_change_only_changes_scheduler_period() -> None:
    clock = _FakeClock(100.0)
    speed = {"value": 10.0}
    cadence = SchedulerCadence(lambda: speed["value"], wall_clock=clock)

    clock.advance(0.1)
    speed["value"] = 2.0
    cadence.sync_speed()

    clock.advance(0.5)

    assert clock() == pytest.approx(100.6)
    assert cadence.wall_period_for_scheduler_period(0.2) == pytest.approx(0.1)


def test_invalid_speedup_falls_back_to_realtime() -> None:
    clock = _FakeClock(50.0)
    cadence = SchedulerCadence(lambda: None, wall_clock=clock)

    clock.advance(0.25)

    assert clock() == pytest.approx(50.25)
    assert cadence.wall_period_for_scheduler_period(0.1) == pytest.approx(0.1)


def test_uses_supplied_nonblocking_speedup_reader() -> None:
    clock = _FakeClock(20.0)
    cadence = SchedulerCadence(lambda: 5.0, wall_clock=clock)

    clock.advance(0.2)

    assert clock() == pytest.approx(20.2)
    assert cadence.wall_period_for_scheduler_period(0.1) == pytest.approx(0.02)


def test_wall_period_applies_cold_cache_speed_on_next_scheduler_tick() -> None:
    clock = _FakeClock(20.0)
    speed = {"value": 1.0}
    cadence = SchedulerCadence(lambda: speed["value"], wall_clock=clock)
    speed["value"] = 10.0

    assert cadence.wall_period_for_scheduler_period(0.02) == pytest.approx(0.002)
    assert cadence.current_speedup == pytest.approx(10.0)


def test_accelerated_sitl_requests_high_resolution_timer(monkeypatch) -> None:
    calls = []
    clock = _FakeClock(20.0)
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence._request_high_resolution_timer",
        lambda: calls.append("begin") or True,
    )
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence._release_high_resolution_timer",
        lambda: calls.append("end"),
    )

    cadence = SchedulerCadence(lambda: 10.0, wall_clock=clock)
    cadence.close()

    assert calls == ["begin", "end"]


def test_realtime_sitl_does_not_request_high_resolution_timer(monkeypatch) -> None:
    calls = []
    clock = _FakeClock(20.0)
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence._request_high_resolution_timer",
        lambda: calls.append("begin") or True,
    )
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence._release_high_resolution_timer",
        lambda: calls.append("end"),
    )

    cadence = SchedulerCadence(lambda: 1.0, wall_clock=clock)
    cadence.close()

    assert calls == []


def test_high_resolution_timer_released_when_speed_returns_realtime(monkeypatch) -> None:
    calls = []
    clock = _FakeClock(20.0)
    speed = {"value": 10.0}
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence._request_high_resolution_timer",
        lambda: calls.append("begin") or True,
    )
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence._release_high_resolution_timer",
        lambda: calls.append("end"),
    )

    cadence = SchedulerCadence(lambda: speed["value"], wall_clock=clock)
    speed["value"] = 1.0
    cadence.sync_speed()

    assert calls == ["begin", "end"]


def test_reader_failure_preserves_last_valid_speed() -> None:
    clock = _FakeClock(30.0)
    speed = {"value": 4.0}
    cadence = SchedulerCadence(lambda: speed["value"], wall_clock=clock)

    clock.advance(0.25)
    speed["value"] = None

    assert clock() == pytest.approx(30.25)
    assert cadence.wall_period_for_scheduler_period(0.2) == pytest.approx(0.05)


def test_accelerated_sitl_uses_live_proven_quantum_only_during_commands(
    monkeypatch,
) -> None:
    current = {"value": 0.005}
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence.sys.getswitchinterval",
        lambda: current["value"],
    )
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence.sys.setswitchinterval",
        lambda value: current.__setitem__("value", value),
    )

    cadence = SchedulerCadence(
        lambda: 10.0,
        wall_clock=_FakeClock(10.0),
        high_resolution_timer=False,
    )

    # Pre-navigation detector, MAVLink, and confirmation threads must retain the
    # normal interpreter quantum.  Tightening it process-wide from startup
    # overloaded the exact three-UAV run before any command was issued.
    assert current["value"] == pytest.approx(0.005)
    cadence.set_command_cadence_active(True)
    assert current["value"] == pytest.approx(0.001)
    cadence.set_command_cadence_active(False)
    assert current["value"] == pytest.approx(0.005)
    cadence.close()
    assert current["value"] == pytest.approx(0.005)


def test_interpreter_quantum_tracks_verified_speed_changes(monkeypatch) -> None:
    current = {"value": 0.005}
    speed = {"value": 10.0}
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence.sys.getswitchinterval",
        lambda: current["value"],
    )
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence.sys.setswitchinterval",
        lambda value: current.__setitem__("value", value),
    )
    cadence = SchedulerCadence(
        lambda: speed["value"],
        wall_clock=_FakeClock(10.0),
        high_resolution_timer=False,
    )
    cadence.set_command_cadence_active(True)

    speed["value"] = 2.0
    cadence.sync_speed()
    assert current["value"] == pytest.approx(0.001)
    speed["value"] = 1.0
    cadence.sync_speed()
    assert current["value"] == pytest.approx(0.005)
    cadence.close()


def test_multiple_cadence_owners_keep_the_smallest_quantum(monkeypatch) -> None:
    current = {"value": 0.005}
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence.sys.getswitchinterval",
        lambda: current["value"],
    )
    monkeypatch.setattr(
        "navpy.modules.common.scheduler_cadence.sys.setswitchinterval",
        lambda value: current.__setitem__("value", value),
    )
    fast = SchedulerCadence(
        lambda: 10.0,
        wall_clock=_FakeClock(10.0),
        high_resolution_timer=False,
    )
    medium = SchedulerCadence(
        lambda: 2.0,
        wall_clock=_FakeClock(10.0),
        high_resolution_timer=False,
    )
    fast.set_command_cadence_active(True)
    medium.set_command_cadence_active(True)

    assert current["value"] == pytest.approx(0.001)
    medium.close()
    assert current["value"] == pytest.approx(0.001)
    fast.close()
    assert current["value"] == pytest.approx(0.005)


def test_vision_sim_timebase_import_is_compatible() -> None:
    assert VisionSimTimebase is SimTimebase is SchedulerCadence
