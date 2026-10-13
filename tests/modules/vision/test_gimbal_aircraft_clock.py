"""The simulated SIYI gimbal runs in aircraft time, not wall-clock slots.

Run self-orbit-a: under CPU load the plant advanced one fixed step per wall
slot and ran about 7x slower than the 10x SITL, so the dock left the frame in
a banked turn.
"""

from __future__ import annotations

import threading
from types import SimpleNamespace

import pytest

from navpy.core.kernel.attitude import Attitude
from navpy.core.ports.gimbal_abc import GimbalData
from navpy.sim.gimbal.aircraft_step_clock import MAX_STEPS_PER_CALL, AircraftStepClock
from navpy.sim.gimbal.gimbal_attitude_reader import VehicleAttitudeReader
from navpy.sim.gimbal.siyi.gimbal_angles import GimbalAngularPlant
from navpy.sim.gimbal.siyi.gimbal_siyi_control import SiyiSimControl
from navpy.sim.gimbal.siyi.gimbal_siyi_state import SiyiSimState
from navpy.sim.gimbal.siyi.gimbal_siyi_step import SiyiSimStepper
from navpy.sim.gimbal.siyi.gimbal_zoom_plant import GimbalZoomPlant

# 50% of the plant's 90 deg/s full scale.
HALF_RATE_DEG_S = 45.0


def test_clock_counts_whole_steps_of_aircraft_time():
    clock = AircraftStepClock(20)
    assert clock.steps_for(1000) == 0  # first sample only syncs
    assert clock.steps_for(1200) == 10
    assert clock.steps_for(1215) == 0  # 15 ms owed
    assert clock.steps_for(1230) == 1  # 30 ms: one step, 10 ms owed
    assert clock.steps_for(1230) == 0  # no aircraft time passed: wait


def test_clock_resyncs_only_when_aircraft_time_goes_back():
    clock = AircraftStepClock(20)
    clock.steps_for(5000)
    assert clock.steps_for(100) == 0  # SITL restart
    assert clock.steps_for(140) == 2


def test_clock_keeps_long_debts_and_pays_them_over_later_calls():
    """A starved runner (at 10x, 100 ms wall = 1 s aircraft) must not drop time."""
    clock = AircraftStepClock(20, max_steps_per_call=50)
    clock.steps_for(0)
    paid = [clock.steps_for(3000)]  # 150 steps owed
    paid += [clock.steps_for(3000) for _ in range(3)]
    assert paid == [50, 50, 50, 0]


def test_clock_without_aircraft_time_steps_once_per_call():
    clock = AircraftStepClock(20)
    assert [clock.steps_for(None) for _ in range(3)] == [1, 1, 1]


def test_clock_rejects_non_positive_settings():
    with pytest.raises(ValueError):
        AircraftStepClock(0)
    with pytest.raises(ValueError):
        AircraftStepClock(20, max_steps_per_call=0)
    assert MAX_STEPS_PER_CALL > 0


def test_vehicle_reader_takes_attitude_and_time_from_one_sample():
    attitude = Attitude(5.0, 90.0, -40.0)
    vehicle = SimpleNamespace(
        attitude=Attitude(0.0, 0.0, 0.0),  # a different, older field
        attitude_sample=SimpleNamespace(attitude=attitude, time_boot_s=12.345),
    )
    assert VehicleAttitudeReader(vehicle).read_timed() == (attitude, 12345)
    no_sample = SimpleNamespace(attitude=attitude)
    assert VehicleAttitudeReader(no_sample).read_timed() is None


class _TimedReader:
    def __init__(self) -> None:
        self.boot_ms = 0

    def read(self) -> Attitude:
        return Attitude(0.0, 0.0, 0.0)

    def read_timed(self) -> tuple[Attitude, int]:
        return self.read(), self.boot_ms


def _rig(reader: object) -> tuple[SiyiSimState, SiyiSimStepper, SiyiSimControl]:
    state = SiyiSimState(
        data=GimbalData(att=Attitude(0.0, 0.0, 0.0)),
        angular=GimbalAngularPlant(0.0, 0.0),
        zoom=GimbalZoomPlant(1.0),
        lock=threading.Lock(),
        started=True,
    )
    stepper = SiyiSimStepper(state, reader)
    return state, stepper, SiyiSimControl(state, SimpleNamespace(), stepper.catch_up)


def test_stepper_advances_by_elapsed_aircraft_time():
    reader = _TimedReader()
    state, stepper, control = _rig(reader)
    stepper.advance()  # sync
    control.set_rate(50.0, 0.0)
    one_step_zoom = GimbalZoomPlant(1.0)
    one_step_zoom.start(1)
    one_step_zoom.advance(0.02)
    control.zoom_in()

    reader.boot_ms = 200  # one wall slot, but 200 ms of aircraft time
    stepper.advance()

    assert state.angular.snapshot().yaw == pytest.approx(HALF_RATE_DEG_S * 0.2)
    assert state.zoom.level - 1.0 == pytest.approx(10 * (one_step_zoom.level - 1.0))


def test_stepper_waits_when_aircraft_time_has_not_moved():
    reader = _TimedReader()
    state, stepper, control = _rig(reader)
    stepper.advance()
    control.set_rate(50.0, 0.0)

    for _ in range(5):
        stepper.advance()

    assert state.angular.snapshot().yaw == pytest.approx(0.0)


def test_a_new_command_does_not_act_on_time_before_it():
    """Rate 50% from 0-200 ms, then 0%: only the first 200 ms move the gimbal."""
    reader = _TimedReader()
    state, stepper, control = _rig(reader)
    stepper.advance()
    control.set_rate(50.0, 0.0)

    reader.boot_ms = 200  # the runner is late; the command arrives first
    control.set_rate(0.0, 0.0)
    reader.boot_ms = 400
    stepper.advance()

    assert state.angular.snapshot().yaw == pytest.approx(HALF_RATE_DEG_S * 0.2)


def test_a_command_after_a_long_gap_pays_the_whole_debt_first():
    """10% rate for 6 s (beyond one runner call's bound), then stop: 54 deg."""
    reader = _TimedReader()
    state, stepper, control = _rig(reader)
    stepper.advance()
    control.set_rate(10.0, 0.0)

    reader.boot_ms = 6000
    control.set_rate(0.0, 0.0)
    reader.boot_ms = 8000
    stepper.advance()

    assert state.angular.snapshot().yaw == pytest.approx(9.0 * 6.0)


def test_commands_without_an_aircraft_clock_do_not_move_the_gimbal():
    class _Untimed(_TimedReader):
        def read_timed(self) -> None:
            return None

    state, stepper, control = _rig(_Untimed())
    control.set_rate(50.0, 0.0)
    for _ in range(10):
        control.zoom_in()

    assert state.angular.snapshot().yaw == pytest.approx(0.0)


def test_lifecycle_stop_pays_owed_time_before_clearing_commands():
    from navpy.sim.gimbal.siyi.gimbal_siyi_lifecycle import SiyiSimLifecycle

    reader = _TimedReader()
    state, stepper, control = _rig(reader)
    stepper.advance()
    control.set_rate(10.0, 0.0)
    runner = SimpleNamespace(stop=lambda: True)
    lifecycle = SiyiSimLifecycle(state, runner, SimpleNamespace(), stepper.catch_up)

    reader.boot_ms = 200
    lifecycle.stop()

    assert state.angular.snapshot().yaw == pytest.approx(9.0 * 0.2)


def test_short_pulses_shorter_than_a_step_are_not_lost():
    """50% for 10 ms, then stop: the 10 ms belong to the 50% command."""
    reader = _TimedReader()
    state, stepper, control = _rig(reader)
    stepper.advance()
    control.set_rate(50.0, 0.0)

    reader.boot_ms = 10
    control.set_rate(0.0, 0.0)
    reader.boot_ms = 20
    stepper.advance()

    assert state.angular.snapshot().yaw == pytest.approx(HALF_RATE_DEG_S * 0.01)


def test_clock_settles_the_remainder_once():
    clock = AircraftStepClock(20)
    clock.steps_for(0)
    assert clock.steps_for(35, pay_all=True) == 1
    assert clock.settle_remainder_ms() == 15
    assert clock.settle_remainder_ms() == 0


def test_published_timestamps_never_go_backwards():
    reader = _TimedReader()
    state, stepper, control = _rig(reader)
    stamps = []
    for boot_ms in (0, 20, 40, 60):
        reader.boot_ms = boot_ms
        stepper.advance()
        control.set_rate(10.0, 0.0)
        stamps.append(state.data.timestamp_s)
    assert stamps == sorted(stamps)


def test_zoom_hold_keeps_the_zoom_already_owed():
    reader = _TimedReader()
    state, stepper, control = _rig(reader)
    stepper.advance()
    control.zoom_in()

    reader.boot_ms = 200
    control.zoom_hold()
    zoomed = state.zoom.level
    reader.boot_ms = 400
    stepper.advance()

    assert zoomed > 1.0
    assert state.zoom.level == pytest.approx(zoomed)
