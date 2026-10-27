"""NavPy's participant hooks on the participant loop, with a stand-in composition.

The loop is driven tick by tick as the coordinator drives it. NavPy's
composition is replaced by a run loop of the same shape (``RuntimeSession``
with the ``--stop-on-lifecycle`` stop hooks the real composition installs),
so each end is checked without a vehicle:

- a lifecycle ``stop`` ends the run loop; cleanup, then the ``stopped{id}``
  ack, and the ending ``DONE`` is flushed (the coordinator answers ``acked``);
- a cleanup failure sends no ack and ends unflushed (``missed`` and a host
  fault at the coordinator);
- the drain ``end`` stops it with no ack event, flushed;
- a stop that arrives before the host was built is held and applied;
- a composition failure before readiness ends unflushed, with no readiness.
"""

from __future__ import annotations

import pytest

from gcs.backend.navpy_process_launch import lockstep_navpy_argv
from navpy.app import runtime_ready
from navpy.app.lockstep import navpy_host
from navpy.app.lockstep.participant import NavpyHooks
from navpy.app.runtime_resources import RuntimeResourceStack, RuntimeSession
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging import logger_factory, pose_cadence_debug
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.coordinator_tick import READY_EVENT
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.part_wire import Lifecycle, Tick
from navpy.sim.lockstep.participant import LockstepIdentity, Participant
from navpy.sim.lockstep.participant_hooks import DEVICE_KEY, ParticipantContext, RequestIds
from navpy.sim.lockstep.participant_loop import ParticipantLoop

TICK_US = 20_000
SERIAL0 = "lockstep:serial0.1"
STOP_ID = (2, 7)


class _Composition:
    """``compose_runtime``'s stand-in: the same run loop, cleanup and stop hooks."""

    def __init__(self, *, cleanup_fails: bool = False, compose_fails: bool = False) -> None:
        self.cleanup_fails = cleanup_fails
        self.compose_fails = compose_fails
        self.args = None
        self.ports = None
        self.cleaned = False

    def __call__(self, args, ports) -> RuntimeSession:
        self.args = args
        self.ports = ports
        if self.compose_fails:
            raise RuntimeError("Vehicle 1 is not broadcasting heartbeats.")
        stack = RuntimeResourceStack()
        stack.own("controller", self._cleanup)
        stop = ports.lifecycle_stop
        return RuntimeSession(
            resources=stack, start_vision=lambda: None, start_controller=lambda: None,
            health_check=lambda: None, ui_step=lambda: True, sleep=rt.sleep,
            ready_signal=lambda: runtime_ready.emit_runtime_ready({"mission_items": 7}),
            stop_requested=stop.requested, acknowledge_stop=stop.acknowledge,
        )

    def _cleanup(self) -> None:
        if self.cleanup_fails:
            raise TimeoutError("navigation loop failed to terminate")
        self.cleaned = True


@pytest.fixture
def make_loop(monkeypatch):
    """A NavPy participant on a test-bound runtime; product globals restored after."""
    monkeypatch.setattr(runtime_ready, "_identity", runtime_ready._identity)
    monkeypatch.setattr(runtime_ready, "_publish", runtime_ready._publish)
    monkeypatch.setattr(logger_factory, "_run_log_dir", None)
    monkeypatch.setattr(pose_cadence_debug, "ENABLED", False)

    def make(composition: _Composition, argv: list[str] | None = None,
             camera_frames=None) -> tuple[ParticipantLoop, NavpyHooks]:
        baton = BatonRuntime()
        runtime = LockstepRuntime.create(baton, epoch_us=0, seed=1, ordinal=1)
        binding = rt.RuntimeBinding()
        binding.bind(runtime)
        monkeypatch.setattr(rt, "_binding", binding)
        monkeypatch.setattr(navpy_host, "compose_runtime", composition)
        participant = Participant(runtime, LockstepIdentity(1, 1), uninstalls=())
        argv = lockstep_navpy_argv(1, SERIAL0) if argv is None else argv
        hooks = NavpyHooks(ParticipantContext(participant, {DEVICE_KEY: SERIAL0, "argv": argv},
                                              RequestIds(1), camera_frames))
        return ParticipantLoop(participant, hooks, link=None), hooks

    return make


def _stop(k: int) -> Tick:
    return Tick(k, k * TICK_US, lifecycle=(Lifecycle("stop", (0, 40 + k), STOP_ID, 1, 1),))


def _run(loop: ParticipantLoop, ticks: dict[int, Tick], last: int = 12):
    dones = []
    for k in range(1, last + 1):
        done = loop.step(ticks.get(k, Tick(k, k * TICK_US)))
        dones.append(done)
        if done.ended is not None:
            break
    return dones


def _events(dones) -> list[tuple[int, str, dict]]:
    return [(done.k, event.kind, event.payload) for done in dones for event in done.events]


def test_a_lifecycle_stop_ends_with_the_ack_and_a_flushed_done(make_loop):
    composition = _Composition()
    loop, hooks = make_loop(composition)
    dones = _run(loop, {4: _stop(4)})
    assert composition.args.stop_on_lifecycle and composition.args.connection == SERIAL0
    assert logger_factory._run_log_dir == navpy_host.LOG_DIR  # run-relative logs
    events = _events(dones)
    assert events[0][:2] == (1, READY_EVENT)
    assert events[0][2] == {"role": "navpy", "mission_items": 7, "ordinal": 1, "generation": 1,
                            "process_pid": 1}
    assert events[1:] == [(4, "stopped", {"id": list(STOP_ID)})]
    assert dones[-1].k == 4 and dones[-1].ended is True and not dones[-1].faults
    assert composition.cleaned and hooks.exit_code(True) == 0
    assert composition.ports.camera_frames is None  # the camera model off


def test_the_sim_camera_schedules_reach_the_runtime_ports(make_loop):
    composition = _Composition()
    camera_frames = lambda _mount_index: None  # noqa: E731
    loop, _hooks = make_loop(composition, camera_frames=camera_frames)
    _run(loop, {4: _stop(4)})
    assert composition.ports.camera_frames is camera_frames


def test_a_cleanup_failure_sends_no_ack_and_ends_unflushed(make_loop):
    loop, _hooks = make_loop(_Composition(cleanup_fails=True))
    dones = _run(loop, {4: _stop(4)})
    assert [kind for _k, kind, _p in _events(dones)] == [READY_EVENT]
    assert dones[-1].k == 4 and dones[-1].ended is False
    assert not dones[-1].faults  # an unflushed end, not a protocol fault


def test_the_drain_end_stops_it_with_no_ack(make_loop):
    composition = _Composition()
    loop, _hooks = make_loop(composition)
    dones = _run(loop, {5: Tick(5, 5 * TICK_US, end=True)})
    assert [kind for _k, kind, _p in _events(dones)] == [READY_EVENT]
    assert dones[-1].k == 5 and dones[-1].ended is True and composition.cleaned


def test_a_stop_before_the_run_starts_ends_it_without_readiness(make_loop):
    """Todo 105: a stop during startup starts nothing (it was held until ready,
    past the stop budget); the stop is acked and the exit is a startup failure."""
    composition = _Composition()
    loop, _hooks = make_loop(composition)
    dones = _run(loop, {1: _stop(1)})  # the admission tick carries it
    assert [kind for _k, kind, _p in _events(dones)] == ["stopped"]
    assert dones[-1].k == 1 and dones[-1].ended is True


def test_a_composition_failure_before_readiness_ends_unflushed(make_loop):
    loop, _hooks = make_loop(_Composition(compose_fails=True))
    dones = _run(loop, {})
    assert _events(dones) == [] and dones[-1].k == 1 and dones[-1].ended is False


@pytest.mark.parametrize("argv, message", [
    (["-c", "tcp:127.0.0.1:5762", "-ss", "1", "--stop-on-lifecycle"], "connects to"),
    (["-c", SERIAL0, "-ss", "1", "--stop-on-stdin"], "lifecycle record only"),
    (["-c", SERIAL0, "-ss", "1", "--stop-on-lifecycle", "-gsu", "4"], "SIM_SPEEDUP"),
    *[(["-c", SERIAL0, "-ss", "1", "--stop-on-lifecycle", "-nt", wall], "wall-only")
      for wall in ("wifi", "serial", "serial_multi", "mav_multi")],
])
def test_a_command_line_off_the_lockstep_contract_is_refused(argv, message):
    with pytest.raises(ValueError, match=message):
        navpy_host.parse_lockstep_args(argv, SERIAL0)


@pytest.mark.parametrize("wall", ["wifi", "serial"])
def test_a_wall_peer_transport_fails_the_bootstrap_before_composition(make_loop, wall):
    """The refusal is the participant's bootstrap: nothing is composed (no socket,
    no port), no readiness, and the participant ends unflushed."""
    composition = _Composition()
    argv = [*lockstep_navpy_argv(1, SERIAL0), "-nt", wall]  # the last -nt wins
    loop, hooks = make_loop(composition, argv)
    dones = _run(loop, {})
    assert composition.args is None and hooks.host is None
    assert _events(dones) == [] and dones[-1].ended is False


@pytest.mark.parametrize("network", [None, "none", "mav"])
def test_the_vehicle_bus_or_no_peer_transport_is_accepted(network):
    extra = [] if network is None else ["-nt", network]
    args = navpy_host.parse_lockstep_args(["-c", SERIAL0, "-ss", "1", "--stop-on-lifecycle", *extra], SERIAL0)
    assert args.network_type in navpy_host.LOCKSTEP_NETWORK_TYPES
    # The launcher's lockstep command line is the vehicle bus.
    assert navpy_host.parse_lockstep_args(lockstep_navpy_argv(1, SERIAL0), SERIAL0).network_type.value == "mav"


def test_the_identity_cadence_is_sim_time_and_touches_no_process_state(monkeypatch):
    """``IdentitySchedulerCadence`` replaces the speedup: it reads no ``SIM_SPEEDUP``,
    requests no timer resolution and leaves the interpreter switch interval alone."""
    import sys

    from navpy.core.kernel import scheduler_cadence
    from navpy.core.kernel.scheduler_cadence import IdentitySchedulerCadence

    requested: list = []
    monkeypatch.setattr(scheduler_cadence, "_request_high_resolution_timer",
                        lambda: requested.append("timer") or True)
    switch = sys.getswitchinterval()
    cadence = IdentitySchedulerCadence()
    cadence.set_command_cadence_active(True)  # the command loop's hint
    assert cadence.current_speedup == 1.0 and cadence.sync_speed() == 1.0
    assert cadence.wall_period_for_scheduler_period(0.02) == 0.02
    assert sys.getswitchinterval() == switch and requested == []
    cadence.close()


def test_the_hooks_take_their_device_from_the_role_config(make_loop):
    """Devices are role data (``lockstep_fleet.md``, Links and names): the link
    is the configured device, and a role config without one is refused."""
    _loop, hooks = make_loop(_Composition())
    assert list(hooks.links) == [SERIAL0] and hooks.link.device == SERIAL0
    with pytest.raises(ValueError, match="names no 'device'"):
        _ = ParticipantContext(None, {"argv": []}, RequestIds(1)).device
