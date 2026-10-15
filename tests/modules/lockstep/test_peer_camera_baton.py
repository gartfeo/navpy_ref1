"""Peer camera transitions and the peer POI dispatch worker under the baton (10f).

``lockstep_fleet.md`` (Peer camera and image transfer; item 7): pre-NAV geo
pointing, the visual handoff and the NAV geo-session fence run on runtime
clocks and managed primitives; here each transition runs on a baton. The
parts are the product's: two cameras on real ``GimbalNavigation``, the
``GeoPointingFleet``, ``PeerGeoTracker``, ``PeerApproachStarter``,
``PeerGeoTick``, ``GeoReleasingTracking`` and ``NavEntry``. A nav loop, a
managed thread, ticks the peer geo update every 20 ms while a transition
lands on its own thread in a chosen tick. ``PeerPoiDispatchWorker``'s
coalescing, reset fence and stop run on baton threads, its waits on ticks.
"""

from __future__ import annotations

import random
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from unittest.mock import Mock

import pytest

from navpy.core.approach.nav_entry import NavEntry, NavEntryPorts
from navpy.core.command.gimbal.gimbal_navigation import GimbalNavigation
from navpy.core.command.gimbal.gimbal_navigation_state import GimbalLossPolicy, GimbalTrackingSetup
from navpy.core.flow.nav_state import GeoHoldState, NavigationTaskState, NavPhaseState, NavState
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.attitude import Attitude
from navpy.core.kernel.location import Location
from navpy.core.perception.geo_pointing_fleet import GeoPointingFleet
from navpy.core.perception.pointing.gimbal_rate_tracker import GimbalRateTrackerConfig
from navpy.core.perception.tracking_command_router import TrackingCommandRouter
from navpy.core.perception.tracking_geo_hand_off import GeoReleasingTracking
from navpy.core.skills.acquire.approach_strategy import ApproachKind, ApproachPlan
from navpy.core.skills.acquire.peer_approach_start import PeerApproachPorts, PeerApproachStarter
from navpy.core.skills.acquire.peer_geo import PeerGeoTracker
from navpy.core.skills.acquire.peer_geo_tick import DetectPointingPorts, PeerGeoTick
from navpy.core.swarm.peer_poi_dispatch import PeerPoiDispatchPorts, PeerPoiDispatchWorker
from navpy.sim.lockstep.baton import BatonRuntime
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from tests.detection_factory import make_detected_poi
from tests.modules.lockstep.test_lockstep_components import _inject_host_delays
from tests.modules.nav.test_peer_camera_tracking import _Camera
from tests.modules.navigation.test_gimbal_navigation import _FakeGeoRef, _make_mount

TICK_US = 20_000
EPOCH_US = 1_760_000_000_000_000
POI = Location(40.5, -74.5, 900.0, is_absolute=True)
UAV = Location(40.0, -74.0, 1500.0, is_absolute=True)


class _Baton:
    """One baton, its trace, and actions spawned at chosen ticks."""

    def __init__(self, baton: BatonRuntime) -> None:
        self.baton = baton
        self.trace: list[tuple[int, str]] = []

    @property
    def k(self) -> int:
        return self.baton.now_us // TICK_US

    def note(self, what: str) -> None:
        self.trace.append((self.k, what))

    def run(self, ticks: int, actions: dict[int, Callable[[], None]]) -> None:
        if 0 in actions:
            self.baton.spawn(actions[0], name="action0")
            self.baton.run_until_quiescent()
        for k in range(1, ticks + 1):
            self.baton.tick(k * TICK_US)
            if k in actions:
                self.baton.spawn(actions[k], name=f"action{k}")
                self.baton.run_until_quiescent()
        if self.baton.threads.errors:
            raise self.baton.threads.errors[0]

    def lines(self, prefix: str) -> list[int]:
        return [k for k, what in self.trace if what.startswith(prefix)]


@contextmanager
def _on_baton(jitter_seed: int | None = None) -> Iterator[_Baton]:
    baton = BatonRuntime()
    binding = rt.RuntimeBinding()
    binding.bind(LockstepRuntime.create(baton, epoch_us=EPOCH_US, seed=1, ordinal=1))
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(rt, "_binding", binding)
        if jitter_seed is not None:
            _inject_host_delays(mp, random.Random(jitter_seed))
        try:
            yield _Baton(baton)
        finally:
            baton.abort()


# -- the camera transitions ------------------------------------------------------

_GIMBAL_COMMANDS = ("set_att", "set_rate", "set_motion_mode")


def _command_args(args: tuple) -> str:
    return ", ".join(f"pitch {a.pitch:.1f} yaw {a.yaw:.1f}" if isinstance(a, Attitude) else str(a)
                     for a in args)


def _camera(name: str, note: Callable[[str], None]) -> tuple[_Camera, Mock]:
    """A real ``GimbalNavigation`` whose gimbal commands go into the trace."""
    mount, gimbal = _make_mount()
    mount.name = name
    for command in _GIMBAL_COMMANDS:
        getattr(gimbal, command).side_effect = (
            lambda *args, _c=command: note(f"{name} {_c} {_command_args(args)}"))
    logger = Mock()
    logger.is_enabled_for.return_value = False
    navigation = GimbalNavigation(
        mount, logger,
        tracking=GimbalTrackingSetup(
            GimbalRateTrackerConfig(correction_bw=1.0, max_rate=100.0, command_lead_time=0.15),
            GimbalLossPolicy(hold_sec=0.5, repoint_sec=2.0),
        ),
        zoom_config=None, neutral_pitch_deg=-20.0,
    )
    return _Camera(name, navigation), gimbal


class _PeerCamera:
    """The peer-navigation camera parts as ``nav_task_composition`` wires them."""

    def __init__(self, run: _Baton) -> None:
        self.run = run
        self.phase = NavPhaseState(current=NavState.DETECT)
        (self.cam_a, self.gimbal_a), (self.cam_b, self.gimbal_b) = (
            _camera("cam_a", run.note), _camera("cam_b", run.note))
        cameras = [self.cam_a, self.cam_b]
        self.fleet = GeoPointingFleet(cameras, Mock())
        self.geo_ref = _FakeGeoRef()
        geo_hold = GeoHoldState()
        in_nav = lambda: self.phase.is_current(NavState.NAV)  # noqa: E731
        tracker = PeerGeoTracker(self.fleet, geo_hold, self.geo_ref, lambda: UAV,
                                 lambda: Attitude(0.0, 0.0, 0.0), Mock())
        plan = ApproachPlan(kind=ApproachKind.ORBIT, approach_location=POI, offset_distance=0.0,
                            orbit_radius=686.0)
        self.starter = PeerApproachStarter(
            PeerApproachPorts(current_absolute=lambda: None, plan_orbit=lambda *_a, **_k: (plan, None),
                              request_guided=lambda: run.note("approach: guided"),
                              dispatch_approach=lambda *_a, **_k: run.note("approach: dispatched"),
                              save_loiter_radius=lambda *_a, **_k: True),
            NavigationTaskState(), tracker, in_nav, Mock())
        acquisition = Mock()
        acquisition.snapshot.return_value = None
        self.geo_tick = PeerGeoTick(
            NavigationTaskState(peer_navigation=True), geo_hold,
            DetectPointingPorts(
                is_detection_armed=lambda: any(c.is_detection_armed for c in cameras),
                prepare_geo_acquisition=self.fleet.prepare_geo_acquisition,
                update_geo=self.fleet.update_geo),
            acquisition, in_nav, lambda: UAV, lambda: Attitude(0.0, 0.0, 0.0))
        registry = Mock()
        registry.identity_for_task.return_value = Mock(source_name="cam_a", local_obj_id=7)
        self.hand_off = GeoReleasingTracking(TrackingCommandRouter(cameras, registry, Mock()),
                                             self.fleet.stop_geo_tracking)
        self.entry = NavEntry(NavEntryPorts(
            request_guided=lambda: run.note(f"nav entry: guided, geo armed {self.fleet.is_geo_armed}"),
            navigation_init=lambda: run.note("nav entry: init"),
            final_approach_active=lambda: True,
            arm_dock_contact=lambda: run.note("nav entry: dock contact armed"),
            stop_geo_tracking=self._stop_geo,
            is_geo_armed=lambda: self.fleet.is_geo_armed,
        ), NavigationTaskState(), Mock(), Mock())

    def _stop_geo(self) -> None:
        self.run.note("nav entry: geo stop")
        self.fleet.stop_geo_tracking()

    def nav_loop(self) -> None:
        """The NAV loop's peer geo tick, every 20 ms; notes each geo update."""
        while True:
            before = len(self.geo_ref.calls)
            self.geo_tick.update()
            if len(self.geo_ref.calls) > before:
                self.run.note("geo update")
            rt.sleep(TICK_US / 1e6)

    def start_peer_approach(self) -> None:
        self.starter.start(Mock(class_id=0), POI)
        self.run.note(f"approach started, geo armed {self.fleet.is_geo_armed}")

    def detect(self) -> None:
        self.hand_off.start_tracking(1)
        self.run.note(f"handoff: geo armed {self.fleet.is_geo_armed}, "
                      f"tracking cam_a {self.cam_a.is_detection_armed}, cam_b {self.cam_b.is_detection_armed}")

    def enter_nav(self) -> None:
        self.phase.request(NavState.NAV)
        self.entry.run()
        self.run.note(f"in NAV, geo armed {self.fleet.is_geo_armed}")


def _transitions(jitter_seed: int | None, *, handoff: bool) -> list[tuple[int, str]]:
    with _on_baton(jitter_seed) as run:
        parts: list[_PeerCamera] = []
        actions = {0: lambda: parts.append(_PeerCamera(run)),
                   1: lambda: rt.Thread(target=parts[0].nav_loop, name="nav-loop", daemon=True).start(),
                   3: lambda: parts[0].start_peer_approach(),
                   20: lambda: parts[0].enter_nav() if not handoff else parts[0].detect(),
                   40: lambda: parts[0].start_peer_approach()}  # in NAV or after detection: no geo
        if handoff:
            actions[30] = lambda: parts[0].enter_nav()
        run.run(50, actions)
    return run.trace


def _geo_updates(trace: list[tuple[int, str]]) -> list[int]:
    return [k for k, what in trace if what == "geo update"]


_POINT = "set_att pitch -15.0 yaw 45.0"  # the geo pointing at the POI
_NEUTRAL = ["set_motion_mode 1", "set_att pitch -20.0 yaw 0.0"]


def _commands(trace: list[tuple[int, str]], camera: str) -> list[tuple[int, str]]:
    return [(k, what[len(camera) + 1:]) for k, what in trace if what.startswith(f"{camera} ")]


def _pointing(updates: list[int]) -> list[tuple[int, str]]:
    """Arming locks and points at tick 3, then one pointing per geo update."""
    return [(3, "set_motion_mode 0"), (3, _POINT)] + [(k, _POINT) for k in updates]


def test_pre_nav_geo_pointing_then_the_nav_fence_on_ticks():
    trace = _transitions(None, handoff=False)
    assert (3, "approach started, geo armed True") in trace
    # The nav loop's 20 ms sleep restarts from each wake-up, so float drift
    # skips a tick now and then; it updates from the tick after arming up to
    # the fence's tick (it runs in the timer phase, before the transition).
    updates = _geo_updates(trace)
    assert updates[0] == 4 and updates[-1] == 20 and len(updates) > 10
    fence = [what for k, what in trace if k == 20 and what.startswith(("nav entry", "cam_"))]
    assert fence == ["cam_a " + _POINT, "cam_b " + _POINT,  # the tick's update, before the entry
                     "nav entry: geo stop", *(f"cam_a {c}" for c in _NEUTRAL),
                     *(f"cam_b {c}" for c in _NEUTRAL), "nav entry: guided, geo armed False",
                     "nav entry: init", "nav entry: dock contact armed"]
    for camera in ("cam_a", "cam_b"):  # pointed while armed, neutral at the fence, then nothing
        assert _commands(trace, camera) == _pointing(updates) + [(20, c) for c in _NEUTRAL]
    assert (20, "in NAV, geo armed False") in trace
    assert (40, "approach started, geo armed False") in trace  # never armed in NAV
    assert _transitions(3, handoff=False) == trace


def test_the_visual_handoff_releases_geo_pointing_on_every_camera_on_its_tick():
    trace = _transitions(None, handoff=True)
    assert (20, "handoff: geo armed False, tracking cam_a True, cam_b False") in trace
    updates = _geo_updates(trace)
    assert updates[0] == 4 and updates[-1] == 20  # none once the detection is armed
    # The tracking camera is held in lock, not sent to neutral; the other one is;
    # the NAV entry at 30 has nothing left to command.
    assert _commands(trace, "cam_a") == _pointing(updates) + [(20, "set_motion_mode 0")]
    assert _commands(trace, "cam_b") == _pointing(updates) + [(20, c) for c in _NEUTRAL]
    assert [what for k, what in trace if k == 30 and what.startswith("nav entry")][:2] == [
        "nav entry: geo stop", "nav entry: guided, geo armed False"]  # nothing left to fence
    assert (40, "approach started, geo armed False") in trace
    assert _transitions(3, handoff=True) == trace


# -- the peer POI dispatch worker ---------------------------------------------------

class _Dispatch:
    """The worker's ports; geo resolution and the network send take sim time."""

    def __init__(self, run: _Baton, resolve_s: float, notify_s: float) -> None:
        self.run = run
        self.statuses: set[int] = set()
        self._resolve_s = resolve_s
        self._notify_s = notify_s
        #: (task id, object id) of each POI the worker notified.
        self.notified: list[tuple[int, int]] = []

    def ports(self) -> PeerPoiDispatchPorts:
        return PeerPoiDispatchPorts(
            active_poi=lambda: None,
            resolve_location=self._resolve,
            status_exists=lambda poi: poi.identity.task_id in self.statuses,
            mark_notified=lambda poi: self.statuses.add(poi.identity.task_id),
            notify_pois=self._notify,
            warn=lambda text: self.run.note(f"warn: {text}"),
            error=lambda text, error: self.run.note(f"error: {text}: {error}"),
        )

    def _resolve(self, poi):
        rt.sleep(self._resolve_s)
        self.run.note(f"resolved {poi.identity.task_id}")
        return POI

    def _notify(self, pois) -> None:
        self.run.note(f"notify {[p.identity.task_id for p in pois]}")
        self.notified += [(p.identity.task_id, p.identity.obj_id) for p in pois]
        rt.sleep(self._notify_s)
        self.run.note("notify sent")


def _poi(task_id: int, obj_id: int | None = None):
    return make_detected_poi(obj_id=task_id if obj_id is None else obj_id, task_id=task_id)


def _worker_run(resolve_s: float, notify_s: float, actions: dict[int, Callable], ticks: int = 30,
                jitter_seed: int | None = None):
    with _on_baton(jitter_seed) as run:
        dispatch = run.dispatch = _Dispatch(run, resolve_s, notify_s)
        worker: list[PeerPoiDispatchWorker] = []

        def start() -> None:
            worker.append(PeerPoiDispatchWorker(dispatch.ports()))
            worker[0].start()

        def guarded(step: Callable[[PeerPoiDispatchWorker], None], label: str):
            def call() -> None:
                try:
                    step(worker[0])
                    run.note(f"{label} returned")
                except Exception as error:  # an outcome here
                    run.note(f"{label} raised {type(error).__name__}: {error}")
            return call

        run.run(ticks, {0: start, **{k: guarded(step, label) for k, (label, step) in actions.items()}})
    return run


def test_the_worker_batches_caps_pending_and_notifies_after_resolution_on_ticks():
    run = _worker_run(0.05, 0.0, {1: ("submit", lambda w: w.submit([_poi(1), _poi(2), _poi(3)])),
                                  10: ("stop", lambda w: w.stop())})
    # At most two POIs pend (3 is dropped). Submitted at 20 ms; each 50 ms
    # resolution ends on the first tick at or after it.
    assert run.trace == [
        (1, "submit returned"), (4, "resolved 1"), (7, "resolved 2"), (7, "notify [1, 2]"),
        (7, "notify sent"), (10, "stop returned")]
    assert _worker_run(0.05, 0.0, {1: ("submit", lambda w: w.submit([_poi(1), _poi(2), _poi(3)])),
                                   10: ("stop", lambda w: w.stop())}, jitter_seed=4).trace == run.trace


def test_a_newer_snapshot_of_a_pending_poi_replaces_the_older_one():
    """While task 1 resolves, task 2 pends twice: only its latest snapshot goes out."""
    def run_it(jitter_seed=None):
        return _worker_run(0.05, 0.0, {1: ("submit", lambda w: w.submit([_poi(1)])),
                                       2: ("submit", lambda w: w.submit([_poi(2, 20)])),
                                       3: ("submit", lambda w: w.submit([_poi(2, 21)])),
                                       12: ("stop", lambda w: w.stop())}, jitter_seed=jitter_seed)
    run = run_it()
    assert run.trace == [
        (1, "submit returned"), (2, "submit returned"), (3, "submit returned"),
        (4, "resolved 1"), (4, "notify [1]"), (4, "notify sent"),
        (7, "resolved 2"), (7, "notify [2]"), (7, "notify sent"), (12, "stop returned")]
    assert run.dispatch.notified == [(1, 1), (2, 21)]
    assert run_it(4).trace == run.trace


def test_a_reset_during_resolution_discards_the_old_generation():
    run = _worker_run(0.05, 0.0, {1: ("submit", lambda w: w.submit([_poi(1)])),
                                  2: ("reset", lambda w: w.reset()),
                                  10: ("stop", lambda w: w.stop())})
    assert run.trace == [(1, "submit returned"), (2, "reset returned"), (4, "resolved 1"),
                         (10, "stop returned")]


def test_a_reset_during_a_slow_send_fails_on_its_quiescence_deadline_tick():
    """The send holds the execution fence for 0.2 s; reset waits 0.1 s for it."""
    run = _worker_run(0.0, 0.2, {1: ("submit", lambda w: w.submit([_poi(1)])),
                                 2: ("reset", lambda w: w.reset()),
                                 3: ("health", lambda w: w.raise_if_failed()),
                                 20: ("health", lambda w: w.raise_if_failed())})
    failure = "TimeoutError: Peer POI dispatch did not quiesce for reset within 0.100s"
    # A zero sleep after the submit's tick wakes in the next tick's timer phase,
    # so the send starts at 40 ms; the reset after tick 2 fails 0.1 s later.
    # The send's 0.2 s from 0.04 s is past 0.24 s in float: tick 13, not 12.
    assert run.trace == [
        (1, "submit returned"), (2, "resolved 1"), (2, "notify [1]"), (3, "health returned"),
        (7, f"reset raised {failure}"), (13, "notify sent"), (20, f"health raised {failure}")]
