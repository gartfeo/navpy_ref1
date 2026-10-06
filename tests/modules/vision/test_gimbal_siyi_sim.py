import threading
import time
from dataclasses import dataclass, field

import numpy as np
import pytest

from navpy.modules.common.models.attitude import Attitude
from navpy.modules.vision.camera_mount import CameraMount
from navpy.modules.vision.peripheral.camera_intrinsics import CameraIntrinsics
from navpy.modules.vision.peripheral.gimbal_abc import GimbalData
from navpy.modules.vision.peripheral.siyi.sim.gimbal_angles import (
    GimbalAngularPlant,
    MODE_FOLLOW,
    MODE_FPV,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_frame_transform import (
    zyx_to_matrix,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_control import (
    SiyiSimControl,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_lifecycle import (
    SiyiSimLifecycle,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_readback import (
    SiyiSimReadback,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_sim import (
    GimbalSiyiSim,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_state import (
    SiyiSimState,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_siyi_step import (
    SIYI_PHYSICS_STEP_S,
    SiyiSimStepper,
)
from navpy.modules.vision.peripheral.siyi.sim.gimbal_zoom_plant import (
    GimbalZoomPlant,
)
from navpy.utils.euler_utils import get_euler_by_sequence
from navpy.utils.simple_rotation import Rotation


@dataclass
class _AttitudeReader:
    attitude: Attitude | None = field(
        default_factory=lambda: Attitude(0.0, 0.0, 0.0)
    )

    def read(self) -> Attitude | None:
        return self.attitude


@dataclass
class _Logger:
    debug_messages: list[str] = field(default_factory=list)
    info_messages: list[str] = field(default_factory=list)

    def debug(self, message: str) -> None:
        self.debug_messages.append(message)

    def info(self, message: str) -> None:
        self.info_messages.append(message)

    def warning(self, message: str) -> None:
        pass


@dataclass
class _FakeRunner:
    running: bool = False
    start_result: bool = True
    start_calls: int = 0
    stop_calls: int = 0
    stop_result: bool = True

    @property
    def is_running(self) -> bool:
        return self.running

    def start(self) -> bool:
        self.start_calls += 1
        if self.start_result:
            self.running = True
        return self.start_result

    def stop(self) -> bool:
        self.stop_calls += 1
        if self.stop_result:
            self.running = False
        return self.stop_result


@dataclass
class _BlockingRunner:
    start_entered: threading.Event = field(default_factory=threading.Event)
    release_start: threading.Event = field(default_factory=threading.Event)
    running: bool = False
    start_calls: int = 0
    stop_calls: int = 0

    @property
    def is_running(self) -> bool:
        return self.running

    def start(self) -> bool:
        self.start_calls += 1
        self.start_entered.set()
        if not self.release_start.wait(2.0):
            raise TimeoutError("test did not release lifecycle start")
        self.running = True
        return True

    def stop(self) -> None:
        self.stop_calls += 1
        self.running = False


@dataclass
class _Rig:
    state: SiyiSimState
    reader: _AttitudeReader
    logger: _Logger
    control: SiyiSimControl
    readback: SiyiSimReadback
    stepper: SiyiSimStepper


def _rig(
    *,
    pitch: float = 0.0,
    yaw: float = 0.0,
    zoom: float = 1.0,
    attitude: Attitude | None = None,
) -> _Rig:
    reader = _AttitudeReader(
        Attitude(0.0, 0.0, 0.0) if attitude is None else attitude
    )
    logger = _Logger()
    state = SiyiSimState(
        data=GimbalData(att=Attitude(pitch, yaw, 0.0)),
        angular=GimbalAngularPlant(pitch, yaw),
        zoom=GimbalZoomPlant(zoom),
        lock=threading.Lock(),
        started=True,
    )
    return _Rig(
        state,
        reader,
        logger,
        SiyiSimControl(state, logger),
        SiyiSimReadback(state),
        SiyiSimStepper(state, reader),
    )


def _advance(rig: _Rig, count: int) -> None:
    for _ in range(count):
        rig.stepper.advance()


def _body_matrix(data: GimbalData) -> np.ndarray:
    return Rotation.from_euler(
        data.g_seq,
        get_euler_by_sequence(data.att, data.g_seq),
        degrees=data.degrees,
    ).as_matrix()


def test_stepper_uses_exact_fixed_physics_step():
    rig = _rig()
    rig.control.set_rate(50.0, 0.0)

    rig.stepper.advance()

    expected_yaw_deg = 0.5 * 90.0 * SIYI_PHYSICS_STEP_S
    assert rig.state.angular.snapshot().yaw == pytest.approx(expected_yaw_deg)


def test_stepper_owns_narrow_angular_and_zoom_plants():
    rig = _rig()

    assert isinstance(rig.state.angular, GimbalAngularPlant)
    assert isinstance(rig.state.zoom, GimbalZoomPlant)
    assert not hasattr(rig.stepper, "_physics")


def test_none_aircraft_attitude_preserves_last_valid_camera_state():
    rig = _rig(attitude=Attitude(5.0, 10.0, 3.0))
    rig.stepper.advance()
    first = rig.state.angular.snapshot().vehicle_attitude

    rig.reader.attitude = None
    rig.stepper.advance()

    assert rig.state.angular.snapshot().vehicle_attitude == first


def test_stepper_publishes_raw_wall_timestamp_and_monotonic_age(monkeypatch):
    rig = _rig()
    monotonic_s = [200.0]
    monkeypatch.setattr(time, "time", lambda: 123.5)
    monkeypatch.setattr(time, "monotonic", lambda: monotonic_s[0])

    rig.stepper.advance()
    monotonic_s[0] = 202.25
    data, zoom, age_s, sample_id = rig.readback.get_frame_state_sample()

    assert data.timestamp_s == pytest.approx(123.5)
    assert zoom == pytest.approx(1.0)
    assert age_s == pytest.approx(2.25)
    assert sample_id == 1


def test_set_attitude_converges_without_wall_clock_scaling():
    rig = _rig()
    rig.control.set_att(Attitude(-30.0, 45.0, 0.0))

    _advance(rig, 200)

    angular = rig.state.angular.snapshot()
    assert angular.yaw == pytest.approx(45.0, abs=0.2)
    assert angular.pitch == pytest.approx(-30.0, abs=0.2)


def test_follow_readback_counter_rotates_vehicle_roll():
    rig = _rig(pitch=-15.0, attitude=Attitude(0.0, 0.0, 20.0))
    rig.control.set_motion_mode(MODE_FOLLOW)

    rig.stepper.advance()

    data = rig.readback.get_data()
    assert data.roll_stabilize
    assert data.att.roll == pytest.approx(-20.0, abs=0.5)


def test_lock_readback_preserves_world_pose_during_vehicle_motion():
    rig = _rig()
    rig.control.set_att(Attitude(-10.0, 30.0, 0.0))
    _advance(rig, 200)
    rig.reader.attitude = Attitude(5.0, 10.0, 3.0)

    rig.stepper.advance()

    vehicle_to_world = zyx_to_matrix(10.0, 5.0, 3.0)
    world = vehicle_to_world @ _body_matrix(rig.readback.get_data())
    expected = zyx_to_matrix(30.0, -10.0, 0.0)
    assert np.allclose(world, expected, atol=1e-6)


def test_stepper_publishes_aircraft_attitude_paired_with_body_readback():
    reference = Attitude(5.0, 10.0, 3.0)
    rig = _rig(attitude=reference)

    rig.stepper.advance()

    assert rig.readback.get_data().reference_aircraft_attitude == reference


def test_rate_command_moves_yaw_one_step_at_a_time():
    rig = _rig()
    rig.control.set_rate(50.0, 0.0)

    _advance(rig, 10)

    assert rig.state.angular.snapshot().yaw == pytest.approx(9.0)


def test_angle_command_replaces_prior_rate_command():
    rig = _rig()
    rig.control.set_rate(-50.0, 0.0)
    _advance(rig, 10)
    assert rig.state.angular.snapshot().yaw < 0.0

    rig.control.set_att(Attitude(-30.0, 45.0, 0.0))
    _advance(rig, 200)

    assert rig.state.angular.snapshot().yaw == pytest.approx(45.0, abs=0.2)


@pytest.mark.parametrize("zoom", [5.0, "2"])
def test_valid_absolute_zoom_converges(zoom: float | str):
    rig = _rig()

    assert rig.control.set_zoom(zoom)
    _advance(rig, 300)

    assert rig.readback.get_zoom_level() == pytest.approx(float(zoom))


@pytest.mark.parametrize("zoom", [0.5, 31.0, "bad", None])
def test_invalid_zoom_is_rejected(zoom: object):
    rig = _rig()
    assert not rig.control.set_zoom(zoom)  # type: ignore[arg-type]


def test_incremental_zoom_and_hold_are_deterministic():
    rig = _rig(zoom=10.0)
    rig.control.zoom_out()
    _advance(rig, 10)
    moving_level = rig.readback.get_zoom_level()
    rig.control.zoom_hold()
    _advance(rig, 10)

    assert moving_level < 10.0
    assert rig.readback.get_zoom_level() == pytest.approx(moving_level)


def test_fpv_flags_roll_unstabilized_and_pitch_stabilized():
    rig = _rig()
    rig.control.set_motion_mode(MODE_FPV)

    rig.stepper.advance()

    data = rig.readback.get_data()
    assert not data.roll_stabilize
    assert data.pitch_stabilize


def test_zoom_sample_is_available_only_while_started():
    rig = _rig()
    rig.state.started = False
    rig.stepper.advance()
    assert rig.readback.get_zoom_level_sample() is None
    assert math_is_inf(rig.readback.get_zoom_level_age_s())

    rig.state.started = True
    assert rig.readback.get_zoom_level_sample() is not None


def math_is_inf(value: float) -> bool:
    return value == float("inf")


def test_lifecycle_start_is_idempotent():
    rig = _rig()
    rig.state.started = False
    runner = _FakeRunner()
    lifecycle = SiyiSimLifecycle(rig.state, runner, rig.logger)  # type: ignore[arg-type]

    lifecycle.start()
    lifecycle.start()

    assert runner.start_calls == 1
    assert lifecycle.is_connected()


def test_lifecycle_restarts_after_worker_has_died():
    rig = _rig()
    rig.state.started = False
    runner = _FakeRunner()
    lifecycle = SiyiSimLifecycle(rig.state, runner, rig.logger)  # type: ignore[arg-type]
    lifecycle.start()
    runner.running = False

    lifecycle.start()

    assert runner.start_calls == 2
    assert lifecycle.is_connected()


def test_lifecycle_rolls_back_started_state_when_runner_cannot_start():
    rig = _rig()
    rig.state.started = False
    runner = _FakeRunner(start_result=False)
    lifecycle = SiyiSimLifecycle(rig.state, runner, rig.logger)  # type: ignore[arg-type]

    with pytest.raises(RuntimeError, match="still active"):
        lifecycle.start()

    assert not rig.state.started
    assert not lifecycle.is_connected()


def test_lifecycle_clears_stale_commands_on_start_and_stop():
    rig = _rig(zoom=5.0)
    rig.state.started = False
    runner = _FakeRunner()
    lifecycle = SiyiSimLifecycle(rig.state, runner, rig.logger)  # type: ignore[arg-type]
    rig.control.set_rate(70.0, -30.0)
    rig.control.zoom_in()

    lifecycle.start()

    angular = rig.state.angular.snapshot()
    zoom = rig.state.zoom.snapshot()
    assert angular.yaw_command == 0.0
    assert angular.pitch_command == 0.0
    assert angular.target_yaw is None
    assert angular.target_pitch is None
    assert angular.motion_mode == MODE_FOLLOW
    assert zoom.direction == 0
    assert zoom.target is None

    rig.control.set_att(Attitude(-10.0, 20.0, 0.0))
    rig.control.zoom_out()
    lifecycle.stop()

    angular = rig.state.angular.snapshot()
    zoom = rig.state.zoom.snapshot()
    assert angular.target_yaw is None
    assert angular.target_pitch is None
    assert angular.yaw_command == 0.0
    assert angular.pitch_command == 0.0
    assert zoom.direction == 0
    assert zoom.target is None


def test_lifecycle_propagates_incomplete_runner_quiescence():
    rig = _rig()
    runner = _FakeRunner(running=True, stop_result=False)
    lifecycle = SiyiSimLifecycle(rig.state, runner, rig.logger)  # type: ignore[arg-type]

    assert lifecycle.stop() is False
    assert runner.running


def test_public_adapter_propagates_incomplete_lifecycle_quiescence():
    sim, _, _ = _make_public_sim()
    sim._parts.lifecycle.stop = lambda: False  # type: ignore[method-assign]

    assert sim.stop() is False


def test_lifecycle_stop_waits_for_inflight_start() -> None:
    rig = _rig()
    rig.state.started = False
    runner = _BlockingRunner()
    lifecycle = SiyiSimLifecycle(rig.state, runner, rig.logger)  # type: ignore[arg-type]
    stop_started = threading.Event()
    stop_done = threading.Event()

    start_thread = threading.Thread(target=lifecycle.start)

    def stop_lifecycle() -> None:
        stop_started.set()
        lifecycle.stop()
        stop_done.set()

    stop_thread = threading.Thread(target=stop_lifecycle)
    start_thread.start()
    assert runner.start_entered.wait(2.0)
    stop_thread.start()
    assert stop_started.wait(2.0)
    assert not stop_done.wait(0.05)
    runner.release_start.set()
    start_thread.join(2.0)
    stop_thread.join(2.0)

    assert not start_thread.is_alive()
    assert not stop_thread.is_alive()
    assert runner.start_calls == 1
    assert runner.stop_calls == 1
    assert not lifecycle.is_connected()


def _make_public_sim(
    *,
    pitch: float = 0.0,
    reader: _AttitudeReader | None = None,
) -> tuple[GimbalSiyiSim, _AttitudeReader, _Logger]:
    active_reader = reader or _AttitudeReader()
    logger = _Logger()
    sim = GimbalSiyiSim(
        GimbalData(att=Attitude(pitch, 0.0, 0.0)),
        active_reader,
        logger,
    )
    return sim, active_reader, logger


def test_public_adapter_start_stop_and_initial_pitch():
    sim, _, logger = _make_public_sim(pitch=-10.0)

    sim.start()
    try:
        deadline_s = time.monotonic() + 0.5
        while sim.get_data().timestamp_s <= 0.0 and time.monotonic() < deadline_s:
            time.sleep(0.005)
        assert sim.is_connected()
        assert sim.get_data().att.pitch == pytest.approx(-10.0, abs=0.5)
        assert logger.info_messages
    finally:
        sim.stop()
    assert not sim.is_connected()


def test_public_adapter_stop_before_start_is_safe():
    sim, _, _ = _make_public_sim()
    sim.stop()
    assert not sim.is_connected()


def test_public_adapter_does_not_need_vehicle_parameter_surface():
    sim, _, _ = _make_public_sim()

    sim.start()
    try:
        assert not hasattr(sim, "_sync_sim_speed")
        assert not hasattr(sim, "_physics")
    finally:
        sim.stop()


def test_camera_mount_captures_atomic_sim_gimbal_state():
    sim, _, _ = _make_public_sim()
    camera = CameraIntrinsics(
        zoom_map={
            "1": {"fx": 1000.0, "fy": 1000.0, "cx": 500.0, "cy": 250.0},
        },
        image_width=1000,
        image_height=500,
    )
    mount = CameraMount(name="sim-siyi", camera=camera, gimbal=sim)

    sim.start()
    try:
        deadline_s = time.monotonic() + 0.5
        state = None
        while state is None and time.monotonic() < deadline_s:
            state = mount.capture_frame_state(1000, 500)
            if state is None or state.zoom_sample_id is None:
                state = None
                time.sleep(0.005)
        assert state is not None
        assert not state.gimbal_is_static
        assert state.gimbal_timestamp_s > 0.0
        assert state.zoom_sample_id is not None
        assert float(state.k[0, 0]) == pytest.approx(1000.0)
    finally:
        sim.stop()
