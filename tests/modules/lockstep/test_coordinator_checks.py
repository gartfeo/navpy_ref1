"""Coordinator checks without a full run: the frame grid, capabilities, registration."""

from __future__ import annotations

import dataclasses
import json
import socket
import threading
import time

import pytest

from gcs.backend.instance_registry_runtime import pid_alive
from navpy.sim.lockstep.coordinator_run import FAULT_HOST_FILE
from navpy.sim.lockstep.coordinator_spawn import (
    SpawnRecord,
    capability,
    hermetic_environment,
    validate_register,
)
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, HostFault, WallBudgets
from navpy.sim.lockstep.part_socket import PartListener
from navpy.sim.lockstep.part_wire import Register, decode
from tests.modules.lockstep.coordinator_doubles import ScriptedSitl, build, manifest
from tests.modules.lockstep.test_part_wire import GOLDEN


def _frames_fault(tmp_path, source_us, match):
    sitl = ScriptedSitl(source_us=source_us)
    plan = manifest(initial=(), mission_budget_us=200_000)
    coordinator, *_ = build(tmp_path, plan=plan, sitl=sitl)
    with pytest.raises(CoordinatorFault, match=match):
        coordinator.run()
    return coordinator, sitl


@pytest.mark.parametrize(
    ("source_us", "bad_tick"),
    [
        (lambda k: 20_000 * min(k, 4), 5),  # source_us repeats
        (lambda k: 20_000 * k - (30_000 if k == 6 else 0), 6),  # goes back
        (lambda k: 20_000 * k + (10_000 if k >= 7 else 0), 7),  # a 30 ms gap > Δ_max 25 ms
    ],
)
def test_the_grid_check_faults_on_a_frame_off_the_grid(tmp_path, source_us, bad_tick):
    coordinator, sitl = _frames_fault(tmp_path, source_us, f"grid check at tick {bad_tick}")
    assert sitl.stopped
    assert [r["k"] for r in coordinator.journal.records if "k" in r] == list(range(1, bad_tick))


def test_a_gap_of_exactly_the_maximum_passes(tmp_path):
    # Every frame steps exactly Δ_max: the mission times out (journaled), then
    # the operator (waiting for a hub that never came) misses finalization.
    coordinator, _sitl = _frames_fault(tmp_path, lambda k: 25_000 * k, "finalization passed its budget")
    assert {"timeout": "mission sim-time budget", "k": 12} in coordinator.journal.records


def test_relay_eof_is_a_coordinator_fault(tmp_path):
    sitl = ScriptedSitl()
    sitl.stopped = True
    coordinator, *_ = build(tmp_path, plan=manifest(initial=()), sitl=sitl)
    with pytest.raises(CoordinatorFault, match="SITL frame 1: relay closed"):
        coordinator.run()


def test_the_capability_is_deterministic_per_seed_and_spawn_id():
    assert capability(1, (0, 1)) == capability(1, (0, 1))
    assert len({capability(1, (0, 1)), capability(2, (0, 1)), capability(1, (0, 2)),
                capability(1, (2, 1))}) == 4


def test_a_register_must_match_its_recorded_spawn():
    register = decode(GOLDEN["register"][1])
    record = SpawnRecord((2, 3), "rover", 1, 1, register.capability, requester=2)
    validate_register(register, record)
    for change in ({"capability": "0" * 8}, {"role": "hub"}, {"ordinal": 2},
                   {"spawn_id": (2, 4)}, {"generation": 2}):
        with pytest.raises(HostFault, match="does not match spawn"):
            validate_register(dataclasses.replace(register, **change), record)
    assert isinstance(register, Register)


def test_a_partial_launch_stops_every_spawned_process(tmp_path):
    plan = manifest(initial=(("hub", 2), ("rover", 1)))
    broken = dataclasses.replace(plan.roles["rover"], hooks="tests.modules.lockstep.no_such:rover")
    plan = dataclasses.replace(plan, roles={**plan.roles, "rover": broken})
    coordinator, sitl, _operator, roster = build(
        tmp_path, plan=plan, budgets=WallBudgets(register_s=5, done_s=60, exit_s=30))
    with pytest.raises(HostFault, match=r"spawn \(0, 2\) did not register"):
        coordinator.run()
    assert [m.ordinal for m in roster.members] == [2]
    assert not any(pid_alive(m.child.pid) for m in roster.members)
    assert sitl.stopped and sitl.k == 0


def test_a_child_that_never_gets_go_exits_on_stdin_eof():
    import subprocess
    import sys

    from tests.modules.lockstep.coordinator_doubles import child_environment

    child = subprocess.Popen([sys.executable, "-m", "navpy.sim.lockstep.participant_main"],
                             stdin=subprocess.PIPE, env=child_environment())
    child.stdin.close()  # the coordinator died, or refused it, before go
    assert child.wait(timeout=60) == 4


class _RelayDiesAt(ScriptedSitl):
    """The relay connection drops at frame 6 with an OS message naming a host path."""

    def __init__(self, host_path) -> None:
        super().__init__()
        self._host_path = host_path

    def recv_frame(self) -> bytes:
        if self.k == 5:
            raise ConnectionError(f"relay socket {self._host_path} closed")
        return super().recv_frame()


def test_a_fault_journals_class_and_context_only_and_host_detail_goes_to_a_sidecar(tmp_path):
    digests = []
    for name in ("first-host", "second-host"):
        root = tmp_path / name
        coordinator, *_ = build(root, plan=manifest(initial=()), sitl=_RelayDiesAt(root))
        with pytest.raises(CoordinatorFault):
            coordinator.run()
        digests.append(coordinator.journal.digest)
        assert coordinator.journal.records[-1] == {
            "fault": "CoordinatorFault", "kind": "frame_invalid", "tick": 6, "ordinal": None}
        host = json.loads((root / "host" / FAULT_HOST_FILE).read_text())
        assert str(root) in host["message"] and host["tick"] == 6
    assert digests[0] == digests[1]


def test_a_trickling_peer_still_breaches_the_message_deadline():
    listener = PartListener()
    sender = socket.create_connection(listener.address)
    receiver = listener.accept(5.0)
    stop = threading.Event()

    message = (100).to_bytes(4, "little") + b"x" * 100  # 104 bytes, one every 50 ms

    def trickle() -> None:
        for byte in message:
            if stop.is_set():
                return
            try:
                sender.send(bytes([byte]))
            except OSError:
                return
            stop.wait(0.05)

    feeder = threading.Thread(target=trickle, daemon=True)
    feeder.start()
    started = time.monotonic()
    try:
        with pytest.raises(socket.timeout):
            receiver.recv(time.monotonic() + 0.5)
        assert time.monotonic() - started < 2.0  # each fragment did not restart the budget
    finally:
        stop.set()
        sender.close()
        receiver.close()
        listener.close()


def test_the_child_environment_is_hermetic(monkeypatch):
    monkeypatch.setenv("NAVPY_POSE_CADENCE_DEBUG", "x")
    monkeypatch.setenv("NAVPY_LOG_DIR", "y")
    environment = hermetic_environment(("a", "b"), "RECORD", "{}")
    assert set(environment) <= {"RECORD", "PYTHONPATH", "PYTHONHASHSEED", "PATH", "SYSTEMROOT"}
    assert environment["PYTHONHASHSEED"] == "0" and environment["RECORD"] == "{}"
