"""Step 10d: ``WslFleet``'s launch on the exchange deadline and its cleanup by ownership.

``lockstep_fleet.md``, Fleet barrier (Launch, steps 1-4) and package 9: the
root once, every instance, relays 1..3, then SITLs 1..3; SITL 1's ``go``
anchors frame 1's deadline and SITLs 2..3 launch on it. A failed or
interrupted launch, and a fault before the barrier, mid-mission or in drain,
stop all three SITLs and all three relays by ownership and every
participant. WSL is faked at its seams (root, instances, spawn, relay link,
ownership stop); the participants are the 10a toys, real processes.
"""

from __future__ import annotations

import json
import math
import subprocess

import pytest

from gcs.backend.instance_registry_runtime import pid_alive
from navpy.sim.lockstep.coordinator_deadline import ExchangeDeadline
from navpy.sim.lockstep.coordinator_types import CoordinatorFault, HostFault
from scripts import lockstep_wsl, lockstep_wsl_sitl, lockstep_wsl_spawn
from scripts.lockstep_wsl import OwnedProcess
from scripts.lockstep_wsl_sitl import FleetLaunch, WslFleet
from tests.modules.lockstep.fleet_doubles import (
    build_fleet,
    fleet_manifest,
    fleet_sitls,
    recorded_source_us,
)
from tests.modules.lockstep.test_coordinator_exchange import FakeClock

SLOT = 41
ROOT = "/home/u/navpy-eval-roots/lockstep-x"
HOMES = ("40.1,44.1,1294.86,0.0", "40.1,44.2,1294.86,0.0", "40.1,44.3,1294.86,0.0")
ALL_ROLES = ["relay1", "relay2", "relay3", "sitl1", "sitl2", "sitl3"]
REAL_SPAWN = lockstep_wsl_spawn.spawn


class _Child:
    def __init__(self, role: str) -> None:
        self.role = role
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _Link:
    """A relay link on a scripted SITL; closing it is the relay's end."""

    def __init__(self, sitl) -> None:
        self.sitl = sitl
        self.closed = False

    def recv_frame(self, timeout_s: float = math.inf) -> bytes:
        return self.sitl.recv_frame(timeout_s)

    def send_reply(self, reply: bytes, timeout_s: float = math.inf) -> None:
        self.sitl.send_reply(reply, timeout_s)

    def close(self) -> None:
        self.closed = True
        if self.sitl is not None:
            self.sitl.stop()


class FakeWsl:
    """WSL at ``WslFleet``'s seams. ``fail`` names a role whose spawn fails
    before it is recorded (``spawn`` abandons it); ``after`` runs after each
    recorded spawn (a clock cost, an interrupt); ``links`` serve the relays."""

    def __init__(self, monkeypatch, *, fail: str | None = None, after=None, links=None,
                 clock: FakeClock | None = None, spawn_cost_s: float = 0.0) -> None:
        self.events: list[tuple] = []
        self.children: list[_Child] = []
        self.stops: list[list[str]] = []
        self._fail, self._after, self._links = fail, after, list(links or [])
        self._clock, self._cost = clock, spawn_cost_s
        monkeypatch.setattr(lockstep_wsl, "create_root", self._root)
        monkeypatch.setattr(lockstep_wsl, "prepare_instance",
                            lambda root, inst, tiles: self.events.append(("instance", inst)) or f"{root}/{inst}")
        monkeypatch.setattr(lockstep_wsl, "wsl_path", lambda _path: "/mnt/relay.py")
        monkeypatch.setattr(lockstep_wsl, "stop_owned", self._stop_owned)
        monkeypatch.setattr(lockstep_wsl_sitl.lockstep, "record_child", lambda *_a: True)
        monkeypatch.setattr(lockstep_wsl_spawn, "spawn", self._spawn)
        monkeypatch.setattr(lockstep_wsl_sitl, "StepLink", self._link)

    def _root(self, _name: str, _defaults: str) -> str:
        self.events.append(("root",))
        return ROOT

    def _spawn(self, role, script, cwd, record, *, held, admit=None, ready=None, on_go=None,
               wait_budget=None):
        child = _Child(role)
        held.append(child)
        self.children.append(child)
        if self._clock is not None:  # pid, identity and admission take their time, on a budget
            allowed = (wait_budget or (lambda own: own))(60.0)
            self._clock.now += min(self._cost, allowed)
            if self._cost > allowed:
                child.closed = True  # spawn abandons it: never recorded, never had go
                raise TimeoutError(f"{role}: no output line within {allowed} s")
        if role == self._fail:
            child.closed = True  # spawn abandons it: never recorded, never had go
            raise RuntimeError(f"{role} failed to start")
        owned = OwnedProcess(role, 100 + len(self.children), 1, cwd)
        records = json.loads(record.read_text()) if record.exists() else []
        record.write_text(json.dumps(records + [owned.__dict__]))
        assert admit(owned)
        self.events.append(("spawn", role, script, cwd, ready))
        if on_go is not None:
            on_go()
        if self._after is not None:
            self._after(role)
        return child

    def _link(self, port: int, *, timeout_s: float):
        self.events.append(("link", port))
        return self._links.pop(0) if self._links else _Link(None)

    def _stop_owned(self, record, root):
        assert root == ROOT
        roles = [entry["role"] for entry in json.loads(record.read_text())]
        self.stops.append(roles)
        return [f"{role}: stopped" for role in roles]


def _fleet(tmp_path, *, delay_seed: int = 0) -> WslFleet:
    return WslFleet(tmp_path, SLOT, {"owner": "me"},
                    FleetLaunch(tuple(zip((1, 2, 3), HOMES)), 5, delay_seed, "D", {}))


def test_the_launch_prepares_every_instance_then_relays_then_sitls_on_frame_ones_deadline(
        tmp_path, monkeypatch):
    clock = FakeClock()
    fake = FakeWsl(monkeypatch, clock=clock, spawn_cost_s=1.0)
    deadline = ExchangeDeadline(30.0, 0.5, clock=clock)
    fleet = _fleet(tmp_path, delay_seed=7)
    fleet.launch(deadline)
    instances = [123, 124, 125]  # 3*slot + i-1
    ports = [18550 + 3 * SLOT + i for i in range(3)]
    assert fake.events[:4] == [("root",), *(("instance", i) for i in instances)]
    spawns = [e for e in fake.events if e[0] in ("spawn", "link")]
    assert [e[1] for e in spawns] == ["relay1", ports[0], "relay2", ports[1], "relay3", ports[2],
                                      "sitl1", "sitl2", "sitl3"]
    for index, (_kind, role, script, cwd, ready) in enumerate(e for e in spawns if e[0] == "spawn"):
        inst = instances[index % 3]
        assert cwd == f"{ROOT}/{inst}"
        if role.startswith("relay"):
            assert f"--port {ports[index]}" in script and "--delay-seed 7" in script
            assert ready == "LOCKSTEP_RELAY_READY"
        else:
            sys_id = index - 2
            assert f"-I{inst}" in script and f"--sysid {sys_id}" in script
            assert f"--home {HOMES[sys_id - 1]}" in script
            assert "NAVPY_STEP_BOOT_ID=5" in script and "NAVPY_STEP_HELPER_JITTER_SEED=7" in script
    # SITL 1's go came after 4 spawns of 1 s: frame 1's deadline counts from it.
    assert deadline.due == pytest.approx(4.0 + 30.0 - 0.5)
    assert [r["role"] for r in json.loads((tmp_path / lockstep_wsl.PROCESSES_FILE).read_text())] == ALL_ROLES


def test_sitl_launches_that_overrun_frame_ones_deadline_are_an_exchange_budget_fault(
        tmp_path, monkeypatch):
    clock = FakeClock()
    fake = FakeWsl(monkeypatch, clock=clock, spawn_cost_s=15.0)
    fleet = _fleet(tmp_path)
    with pytest.raises(HostFault, match="launch wait") as raised:
        fleet.launch(ExchangeDeadline(30.0, 0.5, clock=clock))
    assert raised.value.kind == "exchange_budget" and raised.value.tick == 1
    assert isinstance(raised.value.__cause__, TimeoutError)
    fleet.stop()  # SITL 2 took 15 s of frame 1's 29.5 s; SITL 3's spawn ran out inside its wait
    assert clock.now == pytest.approx(60.0 + 29.5)
    assert fake.stops == [["relay1", "relay2", "relay3", "sitl1", "sitl2"]]
    assert all(child.closed for child in fake.children)


def _interrupt_after_sitl1(role: str) -> None:
    if role == "sitl1":
        raise KeyboardInterrupt  # the coordinator dies between launches


@pytest.mark.parametrize(("case", "expected"), [
    ({"fail": "sitl2"}, ["relay1", "relay2", "relay3", "sitl1"]),
    ({"fail": "relay3"}, ["relay1", "relay2"]),
    ({"after": _interrupt_after_sitl1}, ["relay1", "relay2", "relay3", "sitl1"]),
], ids=["sitl2_fails", "relay3_fails", "coordinator_dies_between_launches"])
def test_a_partial_launch_stops_every_recorded_process_and_the_participants(
        tmp_path, monkeypatch, case, expected):
    fake = FakeWsl(monkeypatch, **case)
    fleet = _fleet(tmp_path)
    coordinator, _sitls, _operator, roster = build_fleet(tmp_path / "run", sitls=list(fleet.sitls),
                                                         launch=fleet.launch)
    with pytest.raises((RuntimeError, KeyboardInterrupt)):
        coordinator.run()
    assert fake.stops == [expected]  # one ownership stop, of everything recorded
    assert all(child.closed for child in fake.children)
    assert [m.ordinal for m in roster.members] == [4]  # the hub registered before any launch
    assert not any(pid_alive(m.child.pid) for m in roster.members)
    assert coordinator.journal.records[-1]["tick"] == 0
    fleet.stop()  # idempotent: nothing is stopped twice
    assert len(fake.stops) == 1


def _offset_from(k0: int):
    recorded = recorded_source_us()
    return {3: lambda k: recorded(k) - (833 if k >= k0 else 0)}


@pytest.mark.parametrize(("phase", "options", "fault"), [
    ("boot", {"quiet_from": (3, 10**9, 3), "plan": {"boot_budget_us": 100_000}}, "boot_barrier"),
    ("mission", {"source_us": _offset_from(60)}, "fleet_clock"),
    ("drain", {"plan": {"hub": {"stop_on_end": True}}}, "stop_in_drain"),
])
def test_a_fault_at_any_phase_stops_all_sitls_relays_and_participants(
        tmp_path, monkeypatch, phase, options, fault):
    sitls = fleet_sitls(quiet_from=options.get("quiet_from", (3, 3, 3)),
                        source_us=options.get("source_us"))
    links = [_Link(sitl) for sitl in sitls]
    fake = FakeWsl(monkeypatch, links=links)
    fleet = _fleet(tmp_path)
    coordinator, _sitls, _operator, roster = build_fleet(
        tmp_path / "run", plan=fleet_manifest(**options.get("plan", {})), sitls=list(fleet.sitls),
        launch=fleet.launch)
    with pytest.raises((CoordinatorFault, HostFault)) as raised:
        coordinator.run()
    end = coordinator.journal.records[-1]
    phases = {r["k"]: r["phase"] for r in coordinator.journal.records if "phase" in r}
    # The fault's tick, or the last journaled one before it (a frame never read is not journaled).
    assert phases.get(end["tick"], phases[end["tick"] - 1]) == phase
    if fault != "stop_in_drain":
        assert raised.value.kind == fault
    else:
        assert "during drain" in str(raised.value)
    assert fake.stops == [ALL_ROLES]  # one ownership stop: 3 relays and 3 SITLs
    assert all(child.closed for child in fake.children) and all(link.closed for link in links)
    assert len(roster.members) >= (1 if phase == "boot" else 2)
    assert not any(pid_alive(m.child.pid) for m in roster.members)


class _Stream:
    """A launcher's stdout: queued lines, EOF once its stdin closes."""

    def __init__(self) -> None:
        import queue

        self.lines: queue.Queue = queue.Queue()
        self.closed = False

    def readline(self) -> bytes:
        line = self.lines.get()
        if not line:
            self.lines.put(b"")
        return line

    def close(self) -> None:
        self.closed = True


class _Launcher:
    """``wsl.exe`` running the spawn launcher: it reports its pid and, for a
    relay, its ready line after ``go``; stdin EOF ends it. SITL 2's launcher
    stalls before its pid and stays alive after its stdin closes (a bash that
    never reads stdin) until the test ends it (:meth:`end`). ``log`` records
    the owner's cleanup steps in order."""

    pids = iter(range(500, 600))
    log: list = []
    stalled: list = []

    def __init__(self, argv, **_kwargs) -> None:
        self.stdout = _Stream()
        script = argv[-1]
        self.role = "relay" if "relay.py" in script else "sitl"
        self.stalls = "--sysid 2" in script
        self.ended = False
        if self.stalls:
            _Launcher.stalled.append(self)
        else:
            self.stdout.lines.put(f"LOCKSTEP_PID {next(self.pids)}\n".encode())
        outer = self

        class Stdin:
            closed = False

            def write(self, data: bytes) -> None:
                if data == b"go\n" and outer.role == "relay":
                    outer.stdout.lines.put(b"LOCKSTEP_RELAY_READY pid=1\n")

            def flush(self) -> None:
                pass

            def close(self) -> None:
                Stdin.closed = True
                if not outer.stalls:
                    outer.end()  # EOF: the launcher kills its held child and exits

        self.stdin = Stdin()

    def end(self) -> None:
        self.ended = True
        self.stdout.lines.put(b"")

    def wait(self, timeout=None) -> int:
        _Launcher.log.append(("reap", "stalled" if self.stalls else self.role))
        if not self.ended:
            raise subprocess.TimeoutExpired("wsl.exe", timeout)  # the bounded wait runs out
        return 0


def test_a_stalled_later_sitl_launch_is_cut_at_frame_ones_deadline_and_cleaned_up(
        tmp_path, monkeypatch):
    """Codex 10d rounds 1 and 2: every wait inside a later SITL's spawn draws
    on frame 1's deadline, and nothing in the spawn path blocks on cleanup.
    SITL 2's launcher never reports its pid and outlives its closed stdin:
    the launch fails with ``exchange_budget`` at the deadline. The owner's
    stop then stops every recorded process by ownership first, and only then
    reaps the launchers with bounded waits; the stalled one stays held, and a
    later stop closes it once it ended, without a second ownership stop."""
    import time

    fake = FakeWsl(monkeypatch)
    monkeypatch.setattr(lockstep_wsl_spawn, "spawn", REAL_SPAWN)
    monkeypatch.setattr(lockstep_wsl_spawn.subprocess, "Popen", _Launcher)
    monkeypatch.setattr(_Launcher, "log", [])
    monkeypatch.setattr(_Launcher, "stalled", [])
    monkeypatch.setattr(lockstep_wsl, "proc", lambda op, pid, **_kw: f"7 {ROOT}/12{pid % 3}")
    stop_owned = lockstep_wsl.stop_owned

    def logged_stop(record, root):
        _Launcher.log.append(("stop_owned",))
        return stop_owned(record, root)

    monkeypatch.setattr(lockstep_wsl, "stop_owned", logged_stop)
    fleet = _fleet(tmp_path)
    deadline = ExchangeDeadline(2.0, 0.5)  # frame 1's deadline: 1.5 s after SITL 1's go
    started = time.monotonic()
    with pytest.raises(HostFault, match="launch wait") as raised:
        fleet.launch(deadline)
    assert time.monotonic() - started < 3.0  # raised at the deadline: no reap in the spawn path
    assert raised.value.kind == "exchange_budget"
    assert isinstance(raised.value.__cause__, TimeoutError)
    assert _Launcher.log == []  # nothing reaped yet
    (stalled,) = _Launcher.stalled
    assert stalled.stdin.closed and not stalled.ended  # released, still alive
    with pytest.raises(subprocess.TimeoutExpired):
        fleet.stop()
    assert _Launcher.log[0] == ("stop_owned",)  # ownership first, then the bounded reaps
    assert ("reap", "stalled") in _Launcher.log[1:]
    assert fake.stops == [["relay1", "relay2", "relay3", "sitl1"]]
    assert [child.role for child in fleet._children] == ["sitl2"]  # held for the retry
    stalled.end()
    fleet.stop()
    assert fleet._children == [] and len(fake.stops) == 1
