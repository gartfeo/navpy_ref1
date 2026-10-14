"""The LockstepBus contract: VehicleMav composes and runs on bytes alone.

A NavPy-style vehicle (``create_vehicle``, NavPy's factory) on
``lockstep:serial0.1`` and a backend-style one (``VehicleManager.add_vehicle``)
on ``lockstep:router`` run on one baton against a reference autopilot byte
stream: heartbeat admission, a mode change, a parameter set, a named
parameter read, a mission download and close. Construction also fails before
attach, fails after attach, and is cancelled by an aborted run.

The runtime is bound through a test-only fresh ``RuntimeBinding`` (the
process binding is once only). Sockets and ``mavlink_connection`` are refused
and the thread guard is installed, so the vehicles have nothing but the
link's bytes.
"""

from __future__ import annotations

import hashlib
import random
import socket
import threading
import time as host_time
from argparse import Namespace
from collections.abc import Callable, Iterator
from contextlib import contextmanager

import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from gcs.backend import full_params  # noqa: F401  (imported on the wall runtime: remove_vehicle uses it)
from gcs.backend.vehicle_manager import VehicleManager
from navpy.app.conn_args import ConnArgs
from navpy.app.vehicle_factory import create_vehicle
from navpy.core.kernel import runtime_time as rt
from navpy.core.kernel.logging.logger_api import ConsoleLogger
from navpy.core.ports.flight_mode import FlightMode
from navpy.plugins.vehicle.ardupilot import vehicle_build_transaction
from navpy.plugins.vehicle.ardupilot.mav_bus_heartbeats import MavBusHeartbeats
from navpy.plugins.vehicle.ardupilot.vehicle_foundation_composition import RESPONSE_HISTORY_DEPTH
from navpy.sim.lockstep.baton import BatonRuntime, audit
from navpy.sim.lockstep.lockstep_bus import LockstepConnection
from navpy.sim.lockstep.lockstep_link import LockstepLink
from navpy.sim.lockstep.lockstep_runtime import LockstepRuntime
from navpy.sim.lockstep.mavlink_wire import ByteSink, StreamDecoder, encoder
from navpy.sim.lockstep.thread_guard import install_thread_guard
from tests.modules.lockstep.lockstep_autopilot import MISSION, SYS_ID, ReferenceAutopilot
from tests.modules.lockstep.test_lockstep_components import _inject_host_delays

TICK_US = 20_000
EPOCH_US = 1_760_000_000_000_000
SERIAL0, SERIAL1 = "lockstep:serial0.1", "lockstep:router"
DEVICES = (SERIAL0, SERIAL1)
PLANE_GUIDED, PLANE_AUTO, COPTER_GUIDED = 15, 10, 4


def _refused(*_args, **_kwargs):
    raise AssertionError("a lockstep vehicle opened a host transport")


class _TraceLogger(ConsoleLogger):
    """The vehicle's ``ILogger``: lines go into the run trace with their tick."""

    def __init__(self, note: Callable[[str], None]) -> None:
        super().__init__()
        self._note = note

    def debug(self, msg: object) -> None:
        self._note(f"debug: {msg}")

    def info(self, msg: object, key: str = "", status: object = None, dest=None, check_interval=False) -> None:
        self._note(f"info: {msg}")

    def warning(self, msg: object, key: str = "", status: object = None, dest=None) -> None:
        self._note(f"warning: {msg}")

    def error(self, msg: object, ex: BaseException | None = None) -> None:
        self._note(f"error: {msg}")


class _Harness:
    """The coordinator: ticks, routes link bytes to the autopilot and back, journals."""

    def __init__(self, baton: BatonRuntime, autopilot: ReferenceAutopilot) -> None:
        self.baton = baton
        self.autopilot = autopilot
        self.links = {device: LockstepLink(baton, device) for device in DEVICES}
        self.trace: list[tuple[int, str, str]] = []
        self.journal: list[tuple[int, bytes, bytes, bytes, bytes]] = []
        self._products: list = []

    def note(self, who: str, what: str) -> None:
        self.trace.append((self.baton.now_us // TICK_US, who, what))

    def logger(self, who: str) -> _TraceLogger:
        return _TraceLogger(lambda line: self.note(who, line))

    def spawn(self, who: str, script: Callable[[], None]) -> None:
        def guarded() -> None:
            try:
                script()
            except Exception as error:  # a construction failure is an outcome here
                self.note(who, f"raised {type(error).__name__}: {error}")

        self._products.append(self.baton.spawn(guarded, name=who))

    def run(self, ticks: int, rng: random.Random | None, *, tick_us: int = TICK_US) -> None:
        baseline = set(threading.enumerate())
        pending = [b"", b""]
        for k in range(ticks + 1):
            def deliver(data=pending) -> None:
                for device, chunk in zip(DEVICES, data):
                    if chunk:
                        self.links[device].deliver(chunk)

            self.baton.tick(k * tick_us, deliver)
            audit(self.baton, baseline)
            egress = [self.links[device].drain_egress() for device in DEVICES]
            pending = self.autopilot.step(k * tick_us, egress)
            self.journal.append((k, *egress, *pending))
            if rng is not None:
                host_time.sleep(rng.random() / 1000)
            if all(product.done for product in self._products):
                return

    def alive(self) -> list[str]:
        return [thread.name for thread in self.baton.threads.alive()]

    def digest(self) -> str:
        digest = hashlib.sha256()
        for k, *streams in self.journal:
            digest.update(k.to_bytes(4, "little"))
            for stream in streams:
                digest.update(len(stream).to_bytes(4, "little") + stream)
        return digest.hexdigest()


@contextmanager
def _lockstep(jitter_seed: int | None) -> Iterator[tuple[BatonRuntime, random.Random | None]]:
    baton = BatonRuntime()
    binding = rt.RuntimeBinding()
    binding.bind(LockstepRuntime.create(baton, epoch_us=EPOCH_US, seed=3, ordinal=1))
    rng = None if jitter_seed is None else random.Random(jitter_seed)
    with pytest.MonkeyPatch.context() as mp:
        mp.setattr(rt, "_binding", binding)
        mp.setattr(socket, "socket", _refused)
        mp.setattr(mavutil, "mavlink_connection", _refused)
        if rng is not None:
            _inject_host_delays(mp, rng)
        uninstall_guard = install_thread_guard(baton)
        try:
            yield baton, rng
        finally:
            uninstall_guard()


def _navpy_vehicle(harness: _Harness):
    args = Namespace(connection=SERIAL0, baud=115200, source_system=SYS_ID)
    return create_vehicle(
        ConnArgs(args), harness.logger("navpy"), open_bus=harness.links[SERIAL0].open,
    )


def _backend_manager(harness: _Harness) -> VehicleManager:
    manager = VehicleManager(open_bus=harness.links[SERIAL1].open)
    # The connect-time mission probe is a backend producer (fenced in 9e); the
    # backend vehicle's mission download below is the call it makes.
    manager._probe_mission_async = lambda entry: None
    return manager


def _exercise(harness: _Harness, who: str, vehicle, mode: FlightMode, value: float) -> None:
    note = lambda what: harness.note(who, what)  # noqa: E731
    note(f"mode set: {vehicle.set_mode(mode)}")
    rt.sleep(1.0)  # the next autopilot heartbeat carries the mode
    note(f"mode now: {vehicle.get_mode}")
    note(f"param set: {vehicle.set_parameter('AAS_NAV_CGT', value)}")
    note(f"named read: {vehicle.get_parameter_fresh('SCHED_LOOP_RATE')}")


def _contract_run(jitter_seed: int | None) -> _Harness:
    with _lockstep(jitter_seed) as (baton, rng):
        harness = _Harness(baton, ReferenceAutopilot())

        def navpy() -> None:
            vehicle = _navpy_vehicle(harness)
            harness.note("navpy", f"constructed: {vehicle.mission_items_count} mission items")
            _exercise(harness, "navpy", vehicle, FlightMode.GUIDED, 20.0)
            vehicle.close()
            harness.note("navpy", f"closed: {harness.links[SERIAL0].state}")

        def backend() -> None:
            manager = _backend_manager(harness)
            entry = manager.add_vehicle(SERIAL1, SYS_ID)
            harness.note("backend", f"downloaded: {entry.vehicle.download_mission()} mission items")
            _exercise(harness, "backend", entry.vehicle, FlightMode.AUTO, 30.0)
            harness.note("backend", f"removed: {manager.remove_vehicle(SYS_ID)}")
            harness.note("backend", f"closed: {harness.links[SERIAL1].state}")

        harness.spawn("navpy", navpy)
        harness.spawn("backend", backend)
        harness.run(400, rng)
        harness.final_alive = harness.alive()
        if harness.final_alive:
            baton.abort()
    return harness


def _lines(harness: _Harness, who: str) -> list[str]:
    return [what for _tick, name, what in harness.trace if name == who]


def test_composed_vehicles_run_on_bytes_alone_and_repeat_under_host_delays():
    plain = _contract_run(None)
    for seed in (1, 2):
        jittered = _contract_run(seed)
        assert jittered.trace == plain.trace, seed
        assert jittered.digest() == plain.digest(), seed

    assert plain.final_alive == []
    assert {device: link.state for device, link in plain.links.items()} == {
        SERIAL0: "closed", SERIAL1: "closed",
    }
    navpy, backend = _lines(plain, "navpy"), _lines(plain, "backend")
    assert "info: Heartbeat from system 1 received." in navpy
    assert f"constructed: {len(MISSION)} mission items" in navpy
    assert f"downloaded: {len(MISSION)} mission items" in backend
    for lines in (navpy, backend):
        assert "mode set: True" in lines
        assert "param set: True" in lines
        assert "named read: 50.0" in lines
        assert not any(line.startswith("raised") for line in lines), lines
    assert "closed: closed" in navpy
    assert backend[-2:] == ["removed: [1]", "closed: closed"]
    # One shared autopilot: NavPy's GUIDED (after the backend's AUTO) is what
    # its next heartbeat reports to both vehicles; NavPy's 20.0 is set last.
    for lines in (navpy, backend):
        assert "mode now: FlightMode.GUIDED" in lines
    assert plain.autopilot.custom_mode == PLANE_GUIDED
    assert plain.autopilot.parameters["AAS_NAV_CGT"] == 20.0
    assert [link.lost_ingress_bytes for link in plain.links.values()] == [0, 0]

    # Each link carried its participant's own (sysid, compid): the product values.
    sources = {(port, source, component) for port, kind, source, component in plain.autopilot.received}
    assert sources == {
        (0, SYS_ID, mavlink.MAV_COMP_ID_ONBOARD_COMPUTER),
        (1, 255, 0),
    }
    kinds = {(port, kind) for port, kind, _s, _c in plain.autopilot.received}
    for port in (0, 1):
        assert {
            (port, "HEARTBEAT"), (port, "SET_MODE"), (port, "PARAM_SET"),
            (port, "PARAM_REQUEST_READ"), (port, "MISSION_REQUEST_LIST"),
            (port, "MISSION_REQUEST_INT"),
        } <= kinds


@pytest.mark.parametrize("style", ["navpy", "backend"])
def test_construction_failing_before_attach_releases_and_closes_the_link(monkeypatch, style):
    def broken(*_args):
        raise RuntimeError("capabilities failed")

    monkeypatch.setattr(vehicle_build_transaction, "build_capability_parts", broken)
    with _lockstep(None) as (baton, rng):
        harness = _Harness(baton, ReferenceAutopilot())
        device = SERIAL0 if style == "navpy" else SERIAL1
        if style == "navpy":
            harness.spawn(style, lambda: _navpy_vehicle(harness))
        else:
            harness.spawn(style, lambda: _backend_manager(harness).add_vehicle(SERIAL1, SYS_ID))
        harness.run(10, rng)
        assert harness.alive() == []
    assert _lines(harness, style) == ["raised RuntimeError: capabilities failed"]
    assert harness.links[device].state == "closed"
    assert harness.journal[0][1:3] == (b"", b"")  # it never sent a byte


def test_construction_failing_after_attach_releases_and_closes_the_link():
    """The autopilot answers but sends no heartbeat: construction fails after attach."""
    tick_us = 100_000
    with _lockstep(None) as (baton, rng):
        harness = _Harness(baton, ReferenceAutopilot(heartbeats=frozenset()))
        harness.spawn("navpy", lambda: _navpy_vehicle(harness))
        harness.run(400, rng, tick_us=tick_us)
        assert harness.alive() == []
    link = harness.links[SERIAL0]
    assert link.state == "closed"
    assert harness.trace[-2:] == [
        (30 * 1_000_000 // TICK_US, "navpy", "warning: No heartbeats received at all."),
        (30 * 1_000_000 // TICK_US, "navpy", "raised RuntimeError: Vehicle 1 is not broadcasting heartbeats."),
    ]


def test_gimbal_only_heartbeats_are_admitted_as_in_wall_mode():
    """The bus admits as ``MavBusHeartbeats`` does: a gimbal heartbeat counts for
    the system, so construction, backend discovery and seen ids match wall mode."""
    gimbal_only = frozenset({"gimbal"})
    wall = MavBusHeartbeats()
    for message in StreamDecoder().feed(ReferenceAutopilot(heartbeats=gimbal_only).step(0, [b"", b""])[0]):
        wall.observe(message)
    assert set(wall.vehicle) == {SYS_ID}

    with _lockstep(None) as (baton, rng):
        harness = _Harness(baton, ReferenceAutopilot(heartbeats=gimbal_only))

        def navpy() -> None:
            vehicle = _navpy_vehicle(harness)
            harness.note("navpy", f"constructed: {vehicle.mission_items_count} mission items")
            vehicle.close()

        def backend() -> None:
            manager = _backend_manager(harness)
            manager.add_vehicle(SERIAL1, SYS_ID)
            rt.sleep(1.0)
            harness.note("backend", f"seen: {manager.get_seen_ids()}")
            manager.remove_vehicle(SYS_ID)

        harness.spawn("navpy", navpy)
        harness.spawn("backend", backend)
        harness.run(200, rng)
        assert harness.alive() == []
    assert f"constructed: {len(MISSION)} mission items" in _lines(harness, "navpy")
    assert _lines(harness, "backend") == ["seen: [1]"]
    assert [link.state for link in harness.links.values()] == ["closed", "closed"]


def test_a_reply_burst_does_not_evict_the_answer_a_waiter_was_woken_for():
    """The reader yields after each packet, as on the wall: the woken reader of
    PARAM_VALUE takes its answer before a full history of unrelated replies."""
    with _lockstep(None) as (baton, rng):
        harness = _Harness(baton, ReferenceAutopilot(param_burst=RESPONSE_HISTORY_DEPTH))

        def navpy() -> None:
            vehicle = _navpy_vehicle(harness)
            try:
                harness.note("navpy", f"named read: {vehicle.get_parameter_fresh('SCHED_LOOP_RATE')}")
            finally:
                vehicle.close()

        harness.spawn("navpy", navpy)
        harness.run(100, rng)
        assert harness.alive() == []
    assert _lines(harness, "navpy")[-1] == "named read: 50.0"  # not MessageHistoryOverrun


def test_an_aborted_run_unwinds_a_parked_construction():
    """Cancellation (9c): the run aborts while construction waits for a heartbeat."""
    with _lockstep(None) as (baton, rng):
        harness = _Harness(baton, ReferenceAutopilot(heartbeats=frozenset()))
        harness.spawn("navpy", lambda: _navpy_vehicle(harness))
        harness.run(5, rng)
        assert sorted(harness.alive()) == ["Thread-1 (_run)", "navpy"]  # reader, construction
        baton.abort()  # LockstepAborted unwinds both; nothing is reported as a failure
        assert harness.alive() == []
    assert not any(who == "navpy" and what.startswith("raised") for _k, who, what in harness.trace)
    # The rollback's reader join was itself aborted, so the bus stays closing
    # (9e's shielded rollback completes it).
    assert harness.links[SERIAL0].state == "closing"


def test_the_link_owns_its_device_and_loses_bytes_with_no_reader():
    with _lockstep(None) as (baton, _rng):
        link = LockstepLink(baton, SERIAL0)
        with pytest.raises(ValueError, match="not a logical device"):
            LockstepLink(baton, "udp:127.0.0.1:14550")
        baton.tick(0, lambda: link.deliver(b"\xfd\x09"))
        assert (link.state, link.lost_ingress_bytes) == ("unopened", 2)
        opened = []

        def opener() -> None:
            with pytest.raises(ValueError, match="cannot open"):
                link.open(SERIAL1)
            opened.extend([link.open(SERIAL0, source_system=255), link.open(SERIAL0)])

        baton.spawn(opener, name="opener")
        baton.run_until_quiescent()
        assert opened[0] is opened[1] and link.state == "open"  # shared, as MavBus.get_or_create
        baton.spawn(opened[0].close, name="closer")
        baton.run_until_quiescent()
        assert link.state == "closed" and [t.name for t in baton.threads.alive()] == []


def test_mode_mapping_matches_a_mavfile_on_the_same_messages():
    """Fixed-wing before any heartbeat; then the latest vehicle heartbeat's type
    (a gimbal's is not one), as ``mavfile.post_message`` keeps it."""
    sink, wire = ByteSink(), ByteSink()
    connection = LockstepConnection(sink, 255, 0)
    oracle = mavutil.mavfile(None, "parity")
    gimbal, _ = encoder(SYS_ID, mavlink.MAV_COMP_ID_GIMBAL, wire)
    autopilot, _ = encoder(SYS_ID, mavlink.MAV_COMP_ID_AUTOPILOT1, wire)
    decoder = StreamDecoder()

    def observe(send: Callable[[], None]) -> dict[str, int] | None:
        send()
        for message in decoder.feed(wire.drain()):
            connection.observe(message)
            oracle.post_message(message)
        assert connection.mode_mapping() == oracle.mode_mapping()
        return connection.mode_mapping()

    assert observe(lambda: None)["GUIDED"] == PLANE_GUIDED
    assert observe(lambda: gimbal.heartbeat_send(mavlink.MAV_TYPE_GIMBAL, 8, 0, 0, 4))["GUIDED"] == PLANE_GUIDED
    assert observe(lambda: autopilot.heartbeat_send(mavlink.MAV_TYPE_FIXED_WING, 3, 1, 0, 3))["AUTO"] == PLANE_AUTO
    assert observe(lambda: autopilot.heartbeat_send(mavlink.MAV_TYPE_QUADROTOR, 3, 1, 0, 3))["GUIDED"] == COPTER_GUIDED
    connection.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, 8, 0, 0, 4)
    assert sink.drain()[0] == mavlink.PROTOCOL_MARKER_V2
