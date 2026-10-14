"""Lockstep slot ownership on a temp registry, with real owner and child processes.

The design's ownership tests (``lockstep_slice.md``, 9d row): coordinator
exit, partial spawn failure, concurrent stop and reclaim, a delayed release
against a same-token replacement, owner death with live children, and a stop
racing the record-and-``go`` boundary. Every child here is held for ``go`` as
a participant is, and every owner is its own process, so ``gcs_stop`` kills
nothing of the test's.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

from gcs.backend import instance_ports as ip
from gcs.backend import instance_registry as reg
from gcs.backend import instance_registry_lockstep as lockstep
from navpy.sim.lockstep.coordinator_members import Roster
from navpy.sim.lockstep.coordinator_spawn import HostSidecar, LocalSpawner
from navpy.sim.lockstep.coordinator_types import HostFault, WallBudgets
from tests.modules.lockstep.coordinator_doubles import PYTHON_PATH, build, child_environment, manifest

ROOT = Path(__file__).resolve().parents[3]
HELD_CHILD = "import sys, time\nif sys.stdin.readline().strip() != 'go': sys.exit(4)\ntime.sleep(120)\n"
OWNER = """
import json, os, subprocess, sys
from gcs.backend import instance_registry_lockstep as lockstep
slot, identity = lockstep.claim(run_token="seed-1", clone="c", owner="dir:test", port_check=lambda n: True)
child = subprocess.Popen([sys.executable, "-c", sys.argv[1]], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL)
assert lockstep.record_child(slot, identity, lockstep.local_child(child.pid))
child.stdin.write(b"go\\n"); child.stdin.flush()
print(json.dumps({"slot": slot, "identity": identity, "child": child.pid}), flush=True)
command = sys.stdin.readline().strip()
if command == "die":
    os._exit(9)
child.kill(); child.wait()
print(lockstep.release_if_owner(slot, identity), flush=True)
"""


@pytest.fixture(autouse=True)
def registry(tmp_path, monkeypatch):
    monkeypatch.setenv("GCS_INSTANCE_REGISTRY", str(tmp_path / "instances.json"))
    monkeypatch.setattr(reg, "slot_ports_free", lambda n: True)


def _free(n: int) -> bool:
    return True


class Owner:
    """A coordinator stand-in in its own process: claims, records a child, then waits."""

    def __init__(self) -> None:
        self.process = subprocess.Popen(
            [sys.executable, "-c", OWNER, HELD_CHILD], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            env=child_environment(), text=True)
        info = json.loads(self.process.stdout.readline())
        self.slot, self.identity, self.child = info["slot"], info["identity"], info["child"]

    def send(self, command: str) -> str:
        self.process.stdin.write(command + "\n")
        self.process.stdin.flush()
        out = self.process.stdout.read().strip()
        self.process.wait(timeout=30)
        return out


def _gcs_stop():
    spec = importlib.util.spec_from_file_location("gcs_stop_lockstep_test", ROOT / "scripts" / "gcs_stop.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _claim(token: str = "seed-1", pid: int | None = None):
    return lockstep.claim(run_token=token, clone="c", owner="dir:test", pid=pid, port_check=_free)


def test_a_coordinator_exit_releases_its_slot(tmp_path):
    owner = Owner()
    assert reg.get(owner.slot)["lockstep"]["children"][0]["pid"] == owner.child
    assert owner.send("release") == "True"
    assert reg.get(owner.slot) is None and not reg.pid_matches(owner.child)


def test_owner_death_leaves_its_children_discoverable_until_gcs_stop():
    owner = Owner()
    owner.send("die")
    assert reg.pid_matches(owner.child)
    assert [e["chat_index"] for e in reg.live()] == [owner.slot]  # pruning keeps it
    other, _identity = _claim("seed-2")
    assert other != owner.slot  # not claimable
    sitl_eval = reg.claim(label="sitl-eval", owner="dir:test", lo=ip.EVAL_CHAT_MIN, launcher_pid=1)
    assert sitl_eval["chat_index"] not in (owner.slot, other)  # never taken over either
    _gcs_stop().stop_chat(owner.slot)
    assert reg.get(owner.slot) is None and not reg.pid_matches(owner.child)


def test_a_delayed_release_never_removes_a_same_token_replacement():
    slot, old = _claim()
    assert lockstep.release_if_owner(slot, old)
    again, new = _claim()
    assert again == slot and new["run_token"] == old["run_token"] and new["nonce"] != old["nonce"]
    assert not lockstep.release_if_owner(slot, old)  # the old claim's late release
    assert reg.get(slot)["lockstep"]["owner"] == new


def test_a_stop_and_a_concurrent_reclaim_leave_the_new_claim_intact():
    owner = Owner()
    reclaimed = []
    stop = _gcs_stop()

    def kill(pid):
        if not reclaimed:  # a reclaim lands while the stop is in progress
            reclaimed.append(_claim())
        stop._kill_tree(pid)

    from gcs_stop_lockstep import stop_lockstep

    assert stop_lockstep(owner.slot, kill=kill, report=lambda _line: None)
    (slot, identity), = reclaimed
    assert slot != owner.slot
    assert reg.get(slot)["lockstep"]["owner"] == identity
    assert reg.get(owner.slot) is None and not reg.pid_matches(owner.child)
    owner.process.wait(timeout=30)


def _roster(tmp_path, slot, identity):
    plan = manifest()
    return Roster(plan, LocalSpawner(tmp_path / "host", PYTHON_PATH),
                  lockstep.RegistryChildRecorder(slot, identity),
                  HostSidecar(tmp_path / "participants_host.json"), WallBudgets(register_s=60))


def test_a_stop_before_record_kills_the_held_child_before_go(tmp_path):
    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        slot, identity = _claim(pid=keeper.pid)
        roster = _roster(tmp_path, slot, identity)
        lockstep.mark_stopping(slot)  # gcs_stop lands between creation and record
        spawned = []
        original = LocalSpawner.spawn

        def capture(self, *args):
            spawned.append(original(self, *args))
            return spawned[-1]

        LocalSpawner.spawn = capture
        try:
            with pytest.raises(HostFault, match="refused before go"):
                roster.spawn("hub", 2, (0, 1), 0)
        finally:
            LocalSpawner.spawn = original
        assert not reg.pid_matches(spawned[0].pid)
        assert reg.get(slot)["lockstep"]["children"] == []
    finally:
        keeper.kill()


def test_a_stop_after_record_and_go_stops_the_child(tmp_path):
    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    slot, identity = _claim(pid=keeper.pid)
    roster = _roster(tmp_path, slot, identity)
    member = roster.spawn("hub", 2, (0, 1), 0)
    assert reg.get(slot)["lockstep"]["children"][0]["pid"] == member.child.pid
    _gcs_stop().stop_chat(slot)
    assert not reg.pid_matches(member.child.pid) and not reg.pid_matches(keeper.pid)
    assert reg.get(slot) is None


def test_a_partial_spawn_failure_stops_every_child_then_releases(tmp_path, monkeypatch):
    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        slot, identity = _claim(pid=keeper.pid)
        plan = manifest(initial=(("hub", 2), ("rover", 1)))
        broken = dataclasses.replace(plan.roles["rover"], hooks="tests.no_such:rover")
        plan = dataclasses.replace(plan, roles={**plan.roles, "rover": broken})
        coordinator, *_ = build(tmp_path, plan=plan, recorder=lockstep.RegistryChildRecorder(slot, identity),
                                budgets=WallBudgets(register_s=5))
        spawned = _capture_children(monkeypatch)
        with pytest.raises(HostFault, match="did not register"):
            coordinator.run()
        children = reg.get(slot)["lockstep"]["children"]
        assert [c["role"] for c in children] == ["hub"]  # the unadmitted rover's record is released
        assert len(spawned) == 2 and not any(reg.pid_matches(child.pid) for child in spawned)
        assert lockstep.release_if_owner(slot, identity)
    finally:
        keeper.kill()


def _capture_children(monkeypatch) -> list:
    spawned = []
    original = LocalSpawner.spawn

    def capture(self, *args):
        spawned.append(original(self, *args))
        return spawned[-1]

    monkeypatch.setattr(LocalSpawner, "spawn", capture)
    return spawned


def test_a_registry_save_failure_while_recording_kills_the_held_child(tmp_path, monkeypatch):
    from gcs.backend import instance_registry_store as store

    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        slot, identity = _claim(pid=keeper.pid)

        def failing_save(_data):
            raise OSError("disk full")

        broken = store.StoreAccess(locked=store.locked, load=store.load, save=failing_save)
        plan = manifest()
        roster = Roster(plan, LocalSpawner(tmp_path / "host", PYTHON_PATH),
                        lockstep.RegistryChildRecorder(slot, identity, access=broken),
                        HostSidecar(tmp_path / "participants_host.json"), WallBudgets(register_s=60))
        spawned = _capture_children(monkeypatch)
        with pytest.raises(OSError, match="disk full"):
            roster.spawn("hub", 2, (0, 1), 0)
        assert not reg.pid_matches(spawned[0].pid) and roster.members == []
        assert reg.get(slot)["lockstep"]["children"] == []
    finally:
        keeper.kill()


def test_a_failure_after_recording_kills_the_child_and_releases_its_record(tmp_path, monkeypatch):
    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        slot, identity = _claim(pid=keeper.pid)
        roster = _roster(tmp_path, slot, identity)
        spawned = _capture_children(monkeypatch)

        def failing_note(self, *_args, **_kwargs):
            raise OSError("sidecar unwritable")

        monkeypatch.setattr(HostSidecar, "note", failing_note)
        with pytest.raises(OSError, match="sidecar unwritable"):
            roster.spawn("hub", 2, (0, 1), 0)
        assert not reg.pid_matches(spawned[0].pid)
        assert reg.get(slot)["lockstep"]["children"] == []
    finally:
        keeper.kill()


def test_gcs_stop_keeps_the_entry_while_a_wsl_child_is_unconfirmed():
    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        slot, identity = _claim(pid=keeper.pid)
        wsl = {"kind": "wsl", "wsl_pid": 9, "start_ticks": 1, "cwd": "/r/0", "root": "/r"}
        assert lockstep.record_child(slot, identity, wsl)
        stop = _gcs_stop()
        from gcs_stop_lockstep import stop_lockstep

        assert not stop_lockstep(slot, kill=stop._kill_tree, stop_wsl=lambda child: False,
                                 report=lambda _line: None)
        assert reg.get(slot) is not None  # kept: still stoppable
        assert stop_lockstep(slot, kill=stop._kill_tree, stop_wsl=lambda child: True,
                             report=lambda _line: None)
        assert reg.get(slot) is None
    finally:
        keeper.kill()


def test_gcs_stop_keeps_the_entry_while_the_coordinator_survives_its_kill():
    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        slot, _identity = _claim(pid=keeper.pid)
        _gcs_stop()
        from gcs_stop_lockstep import stop_lockstep

        assert not stop_lockstep(slot, kill=lambda pid: None, report=lambda _line: None,
                                 confirm_budget_s=0.2)
        assert reg.get(slot) is not None and reg.pid_matches(keeper.pid)
    finally:
        keeper.kill()


@pytest.mark.parametrize(
    ("result", "alive", "confirmed"),
    [("stopped", True, True), ("gone", True, True), ("not-ours", True, True),
     ("killed", True, False), ("killed", False, True)],
)
def test_a_wsl_kill_counts_only_once_a_fresh_probe_confirms_it(monkeypatch, result, alive, confirmed):
    _gcs_stop()
    import gcs_stop_lockstep

    wsl = gcs_stop_lockstep._wsl_module()
    monkeypatch.setattr(wsl, "stop_recorded", lambda *args: result)
    monkeypatch.setattr(wsl, "recorded_alive", lambda *args: alive)
    child = {"kind": "wsl", "wsl_pid": 9, "start_ticks": 1, "cwd": "/r/0", "root": "/r"}
    assert gcs_stop_lockstep._stop_wsl(child) is confirmed


def test_an_unreachable_wsl_is_not_a_confirmation(monkeypatch):
    _gcs_stop()
    import gcs_stop_lockstep

    wsl = gcs_stop_lockstep._wsl_module()

    def unreachable(*_args):
        raise RuntimeError("WSL command failed")

    monkeypatch.setattr(wsl, "stop_recorded", unreachable)
    child = {"kind": "wsl", "wsl_pid": 9, "start_ticks": 1, "cwd": "/r/0", "root": "/r"}
    assert gcs_stop_lockstep._stop_wsl(child) is False


def test_the_wall_eval_launcher_never_resolves_or_takes_a_lockstep_slot(tmp_path):
    """swarm_run's path: resolve_chat (find_for_owner "sitl-eval") then begin_sitl_launch."""
    sys.path.insert(0, str(ROOT / "scripts"))
    try:
        import swarm_run_wsl
    finally:
        sys.path.remove(str(ROOT / "scripts"))
    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        directory = tmp_path / "clone"
        slot, identity = lockstep.claim(run_token="seed-1", clone=str(directory),
                                        owner=reg.owner_for(str(directory)), pid=keeper.pid,
                                        port_check=_free)
        chat = swarm_run_wsl.resolve_chat(None, eval_mode=True, root=directory, registry=reg)
        assert chat != slot  # a fresh sitl-eval slot, not the coordinator's
        assert reg.begin_sitl_launch(chat, supervisor_pid=keeper.pid)
        with pytest.raises(reg.SitlSupervisorActive, match="lockstep slot"):
            reg.begin_sitl_launch(slot, supervisor_pid=keeper.pid)
        assert reg.get(slot)["lockstep"]["owner"] == identity and not reg.get(slot)["sitl"]
    finally:
        keeper.kill()


def test_a_stop_landing_exactly_between_record_and_go_leaves_no_child(tmp_path, monkeypatch):
    """A recorder hook runs a whole gcs_stop right after the record commits, before go."""
    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        slot, identity = _claim(pid=keeper.pid)
        stop = _gcs_stop()
        from gcs_stop_lockstep import stop_lockstep

        recorder = lockstep.RegistryChildRecorder(slot, identity)
        stopped = []

        class StopAfterRecord:
            def record(self, child, spawn):
                recorded = recorder.record(child, spawn)
                stopped.append(stop_lockstep(slot, kill=stop._kill_tree, report=lambda _l: None))
                return recorded

            def forget(self, child, spawn):
                recorder.forget(child, spawn)

        roster = Roster(manifest(), LocalSpawner(tmp_path / "host", PYTHON_PATH), StopAfterRecord(),
                        HostSidecar(tmp_path / "participants_host.json"), WallBudgets(register_s=5))
        spawned = _capture_children(monkeypatch)
        with pytest.raises((HostFault, OSError)):
            roster.spawn("hub", 2, (0, 1), 0)  # go hits a killed child, or it never registers
        assert stopped == [True]  # the frozen inventory held the child; gcs_stop released
        assert not reg.pid_matches(spawned[0].pid) and not reg.pid_matches(keeper.pid)
        assert reg.get(slot) is None and roster.members == []
    finally:
        keeper.kill()


def test_a_wsl_timeout_leaves_the_child_unconfirmed_and_teardown_continues(monkeypatch):
    _gcs_stop()
    import gcs_stop_lockstep

    def times_out(*_args):
        raise subprocess.TimeoutExpired("wsl.exe", 60)

    wsl = gcs_stop_lockstep._wsl_module()
    monkeypatch.setattr(wsl, "stop_recorded", times_out)
    child = {"kind": "wsl", "wsl_pid": 9, "start_ticks": 1, "cwd": "/r/0", "root": "/r"}
    assert gcs_stop_lockstep._stop_wsl(child) is False

    owner = Owner()  # a coordinator with one local child; a WSL child is recorded after it
    wsl_child = dict(child)
    data = reg.get(owner.slot)
    assert lockstep.record_child(owner.slot, data["lockstep"]["owner"], wsl_child)
    stop = _gcs_stop()

    def raising_stop_wsl(_child):
        raise subprocess.TimeoutExpired("wsl.exe", 60)

    assert not gcs_stop_lockstep.stop_lockstep(owner.slot, kill=stop._kill_tree,
                                               stop_wsl=raising_stop_wsl, report=lambda _l: None)
    assert not reg.pid_matches(owner.child)  # the local child was still stopped
    owner.process.wait(timeout=30)  # and so was the coordinator
    assert reg.get(owner.slot) is not None  # the unconfirmed WSL child keeps the entry


def _stop_main_eval(stop) -> list[int]:
    from unittest.mock import patch

    with patch.object(stop, "stop_chat") as stop_chat, patch.object(sys, "argv", ["gcs_stop.py", "--eval"]):
        assert stop.main() == 0
    return [call.args[0] for call in stop_chat.call_args_list]


def test_gcs_stop_eval_prefers_the_wall_eval_slot_over_a_lockstep_run():
    stop = _gcs_stop()
    owner = reg.owner_for(str(stop.ROOT))
    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        slot, _identity = lockstep.claim(run_token="seed-1", clone=str(stop.ROOT), owner=owner,
                                         pid=keeper.pid, port_check=_free)
        wall = reg.claim(label="sitl-eval", owner=owner, lo=ip.EVAL_CHAT_MIN)["chat_index"]
        assert slot < wall  # the lockstep slot comes first in index order
        assert _stop_main_eval(stop) == [wall]
    finally:
        keeper.kill()


def test_gcs_stop_eval_falls_back_to_the_lockstep_slot_alone():
    stop = _gcs_stop()
    owner = reg.owner_for(str(stop.ROOT))
    keeper = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        slot, _identity = lockstep.claim(run_token="seed-1", clone=str(stop.ROOT), owner=owner,
                                         pid=keeper.pid, port_check=_free)
        assert _stop_main_eval(stop) == [slot]
    finally:
        keeper.kill()


def test_a_child_whose_kill_raises_does_not_stop_the_teardown():
    owner = Owner()  # its child is recorded first
    second = subprocess.Popen([sys.executable, "-c", HELD_CHILD], stdin=subprocess.PIPE)
    try:
        identity = reg.get(owner.slot)["lockstep"]["owner"]
        assert lockstep.record_child(owner.slot, identity, lockstep.local_child(second.pid))
        stop = _gcs_stop()
        from gcs_stop_lockstep import stop_lockstep

        def kill(pid):
            if pid == owner.child:
                raise OSError("access denied")
            stop._kill_tree(pid)

        assert not stop_lockstep(owner.slot, kill=kill, report=lambda _l: None, confirm_budget_s=1.0)
        assert second.wait(timeout=30) is not None  # the later child was stopped
        assert owner.process.wait(timeout=30) is not None  # and so was the coordinator
        assert reg.get(owner.slot) is not None  # the unconfirmed child keeps the entry
    finally:
        second.kill()
        subprocess.run(["taskkill", "/PID", str(owner.child), "/F"], capture_output=True)


class _Win32:
    def __init__(self, handle=7, error=0, result=0x0, started=99) -> None:
        self._open = (handle, error)
        self._result = result
        self._started = started
        self.closed = []

    def start_time(self, handle):
        return self._started

    def open(self, pid):
        return self._open

    def wait(self, handle, timeout_ms):
        return self._result

    def close(self, handle):
        self.closed.append(handle)


@pytest.mark.parametrize(
    ("win32", "still_recorded", "gone"),
    [
        (_Win32(result=0x0), True, True),            # WAIT_OBJECT_0 on a valid handle
        (_Win32(result=0x102), True, False),         # WAIT_TIMEOUT
        (_Win32(result=0xFFFFFFFF), True, False),    # WAIT_FAILED
        (_Win32(handle=0, error=87), False, True),   # no such pid, identity agrees
        (_Win32(handle=0, error=87), True, False),   # no such pid, but identity still matches
        (_Win32(handle=0, error=5), False, False),   # access denied: unknown
        (_Win32(result=0x102, started=123), True, True),   # a live process reusing the pid
        (_Win32(result=0x0, started=None), True, False),   # start time unreadable: unconfirmed
    ],
)
def test_wait_gone_is_fail_closed(monkeypatch, win32, still_recorded, gone):
    _gcs_stop()
    import gcs_stop_lockstep

    monkeypatch.setattr(gcs_stop_lockstep.reg, "pid_matches", lambda pid, start=None: still_recorded)
    assert gcs_stop_lockstep.wait_gone(1234, 99, 0.1, win32=win32) is gone
    if win32._open[0]:
        assert win32.closed == [win32._open[0]]
