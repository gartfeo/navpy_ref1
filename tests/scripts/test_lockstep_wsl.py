"""The WSL side of a lockstep run: commands, environment, ownership records."""

from __future__ import annotations

import json
import shlex
import subprocess
import threading

import pytest

from scripts import lockstep_wsl, lockstep_wsl_spawn


def test_the_sitl_runs_serial_step_on_both_ports_with_the_pinned_start_time():
    command = shlex.split(lockstep_wsl.sitl_command("/r", "40.1,44.2,500,0", 120, 1))
    assert command[:3] == ["setarch", "-R", "/r/bin/arduplane"]
    assert "--serial0=sim:navpystep" in command and "--serial1=sim:navpystep" in command
    assert "--serial2=none" in command
    assert command[command.index("--speedup") + 1] == "1"
    assert command[command.index("--start-time") + 1] == "1767225600"
    assert command[-1] == "-I120"


def test_the_sitl_environment_holds_only_serial_step_settings():
    env = shlex.split(lockstep_wsl.sitl_environment(5, 30_000_000, 0))
    assert env[:2] == ["env", "-i"]
    assert dict(item.split("=", 1) for item in env[2:]) == {
        "PATH": "/usr/bin:/bin",
        "NAVPY_SERIAL_STEP": "1",
        "NAVPY_STEP_BOOT_ID": "5",
        "NAVPY_STEP_DEADLINE_US": "30000000",
    }
    assert "NAVPY_STEP_HELPER_JITTER_SEED=9" in lockstep_wsl.sitl_environment(5, 1, 9)


def test_lockstep_defaults_turn_off_the_logger_and_keep_terrain_on():
    """Terrain is on as in the wall eval (todo 46: with it off the SITL ground was
    flat at home altitude, 3.46 m above the dock); SITL reads tiles on its main
    thread, so it is no helper (lockstep_contracts.md, Firmware helpers)."""
    overrides = lockstep_wsl.LOCKSTEP_DEFAULT_OVERRIDES
    assert overrides["LOG_BACKEND_TYPE"] == 0.0
    assert overrides["TERRAIN_ENABLE"] == 1.0
    assert overrides["SIM_BAUDLIMIT_EN"] == 0.0


def test_prepare_root_seeds_the_pinned_terrain_tiles_into_the_instance(monkeypatch):
    """Each tile goes to ``<root>/<instance>/terrain/`` (SITL's terrain directory,
    relative to its cwd) and is checked by its sha256 in WSL."""
    import hashlib

    from scripts.eval_gcs_demo_sitl_root import terrain_tiles

    monkeypatch.setattr(lockstep_wsl, "wsl", lambda script, **_k: "/home/u")
    calls = []
    monkeypatch.setattr(lockstep_wsl.subprocess, "run",
                        lambda argv, **kwargs: calls.append((argv[-1], kwargs.get("input"))))
    tiles = terrain_tiles()
    root = lockstep_wsl.prepare_root("r", 120, "X 1\n", tiles)
    assert root == "/home/u/navpy-eval-roots/lockstep-r"
    seeded = [(script, data) for script, data in calls if "/terrain/" in script]
    assert len(seeded) == len(tiles) == 1
    script, data = seeded[0]
    assert data == tiles["N40E044.DAT"]
    assert f"cat > {root}/120/terrain/N40E044.DAT;" in script
    assert hashlib.sha256(data).hexdigest() in script  # verified after the write


def test_the_firmware_lock_pins_a_binary():
    lock = lockstep_wsl.load_firmware_lock()
    assert len(lock["binary_sha256"]) == 64 and lock["branch"] == "feat/navpy-serial-step"


def test_stop_refuses_a_record_outside_the_run_root(tmp_path, monkeypatch):
    record = tmp_path / lockstep_wsl.PROCESSES_FILE
    record.write_text(json.dumps([
        {"role": "sitl", "wsl_pid": 7, "start_ticks": 1, "cwd": "/home/x/other/1"},
    ]))
    monkeypatch.setattr(lockstep_wsl, "wsl", lambda *_a, **_k: pytest.fail("must not kill"))
    with pytest.raises(RuntimeError, match="outside the run root"):
        lockstep_wsl.stop_owned(record, "/home/x/navpy-eval-roots/lockstep-a")


def test_stop_goes_through_the_pidfd_helper_with_the_recorded_identity(tmp_path, monkeypatch):
    record = tmp_path / lockstep_wsl.PROCESSES_FILE
    root = "/home/x/navpy-eval-roots/lockstep-a"
    record.write_text(json.dumps([
        {"role": "relay", "wsl_pid": 41, "start_ticks": 1234, "cwd": f"{root}/120"},
    ]))
    calls = []
    monkeypatch.setattr(lockstep_wsl, "proc", lambda *args: calls.append(args) or "stopped")
    assert lockstep_wsl.stop_owned(record, root) == ["relay 41: stopped"]
    assert calls == [("stop", 41, 1234, f"{root}/120")]


def test_the_proc_helper_reads_start_ticks_past_a_comm_with_spaces(tmp_path, monkeypatch):
    from scripts import lockstep_wsl_proc as proc

    stat = "41 (my (odd) name) S 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 777 19 20"
    monkeypatch.setattr("builtins.open", lambda *_a, **_k: __import__("io").StringIO(stat))
    assert proc.start_ticks(41) == "777"


class _HeldWslChild:
    """A double of the ``wsl.exe`` launcher: reports its pid, then waits for ``go`` or EOF."""

    instances: list = []

    def __init__(self, *_args, **_kwargs) -> None:
        import io

        self.stdout = io.BytesIO(b"LOCKSTEP_PID 321\n")
        self.written = bytearray()
        self.closed = False
        outer = self

        class Stdin:
            def write(self, data: bytes) -> None:
                outer.written += data

            def flush(self) -> None:
                pass

            def close(self) -> None:
                outer.closed = True  # EOF: the launcher's bash kills the held child

            @property
            def closed(self) -> bool:
                return outer.closed

        self.stdin = Stdin()
        _HeldWslChild.instances.append(self)

    def wait(self, timeout=None) -> int:
        return 137

    @property
    def child_runs(self) -> bool:
        return b"go\n" in self.written


def _fake_wsl(monkeypatch):
    _HeldWslChild.instances = []
    monkeypatch.setattr(lockstep_wsl.subprocess, "Popen", _HeldWslChild)
    monkeypatch.setattr(lockstep_wsl, "proc", lambda *args: "777 /root/lockstep-a/0")


def test_spawn_admits_before_go_and_a_refusal_never_sends_go(tmp_path, monkeypatch):
    _fake_wsl(monkeypatch)
    seen = []

    def refuse(owned):
        seen.append(owned)
        assert not _HeldWslChild.instances[0].written  # admitted after recording, before go
        return False

    with pytest.raises(RuntimeError, match="stopping"):
        lockstep_wsl_spawn.spawn("sitl", "x", "/root/lockstep-a/0", tmp_path / "p.json", held=[],
                                 admit=refuse)
    (child,) = _HeldWslChild.instances
    assert not child.child_runs and child.closed  # killed unstarted
    assert seen[0].wsl_pid == 321 and seen[0].start_ticks == 777


def test_a_stop_racing_record_and_go_leaves_no_running_child(tmp_path, monkeypatch):
    """gcs_stop marks the slot stopping between the run record and go."""
    _fake_wsl(monkeypatch)
    slot = {"phase": "active"}

    def admit(_owned):
        slot["phase"] = "stopping"  # the stop lands in the interval after recording
        return slot["phase"] == "active"

    with pytest.raises(RuntimeError):
        lockstep_wsl_spawn.spawn("relay", "x", "/root/lockstep-a/0", tmp_path / "p.json", held=[],
                                 admit=admit)
    assert not _HeldWslChild.instances[0].child_runs
    _HeldWslChild.instances = []
    child = lockstep_wsl_spawn.spawn("relay", "x", "/root/lockstep-a/0", tmp_path / "q.json", held=[],
                               admit=lambda _owned: True)
    assert child.process.child_runs and child.owned.wsl_pid == 321
    child.close()


def test_stop_owned_tries_every_record_when_one_stop_fails(tmp_path, monkeypatch):
    record = tmp_path / lockstep_wsl.PROCESSES_FILE
    root = "/home/x/navpy-eval-roots/lockstep-a"
    record.write_text(json.dumps([
        {"role": "relay", "wsl_pid": 41, "start_ticks": 1, "cwd": f"{root}/120"},
        {"role": "sitl", "wsl_pid": 42, "start_ticks": 2, "cwd": f"{root}/120"},
    ]))
    calls = []

    def proc(*args):
        calls.append(args[1])
        if args[1] == 41:
            raise subprocess.TimeoutExpired("wsl.exe", 60)
        return "stopped"

    monkeypatch.setattr(lockstep_wsl, "proc", proc)
    with pytest.raises(RuntimeError, match=r"relay 41: TimeoutExpired.*sitl 42: stopped"):
        lockstep_wsl.stop_owned(record, root)
    assert calls == [41, 42]  # the SITL was still stopped


class _PipedWslChild:
    """A ``wsl.exe`` double whose stdout is a real OS pipe: the test writes the
    child's output into it, as WSL does, and a closed read end fails the write.
    ``first`` is what the launcher wrote before spawn reads (its pid line, ...);
    ``wait_timeouts`` waits run out before one returns."""

    first = b""
    close_after_first = False
    wait_timeouts = 0
    write_fd = -1  # the latest double's: only the fd, so the test holds no pipe reader
    latest: list = []  # the latest double's stdin and stdout (cleanup checks), not the double

    def __init__(self, *_args, **_kwargs) -> None:
        import io
        import os

        read, write = os.pipe()
        self.stdout = os.fdopen(read, "rb")
        self.stdin = io.BytesIO()
        self.wait_timeouts = _PipedWslChild.wait_timeouts
        os.write(write, b"LOCKSTEP_PID 321\n" + _PipedWslChild.first)
        if _PipedWslChild.close_after_first:
            os.close(write)  # the child exits after its first output
        _PipedWslChild.write_fd = write
        _PipedWslChild.latest = [self.stdin, self.stdout]

    def wait(self, timeout=None) -> int:
        if self.wait_timeouts:
            self.wait_timeouts -= 1
            raise subprocess.TimeoutExpired("wsl.exe", timeout)
        return 0


def _piped_wsl(monkeypatch, first: bytes = b"", *, close_after_first: bool = False,
               wait_timeouts: int = 0, stop_fails: bool = False) -> list:
    """Install the double; return the ``proc`` calls."""
    calls: list = []

    def proc(*args):
        calls.append(args)
        if args[0] == "stop" and stop_fails:
            raise RuntimeError("WSL command failed")
        return "777 /root/lockstep-a/0"

    monkeypatch.setattr(lockstep_wsl.subprocess, "Popen", _PipedWslChild)
    monkeypatch.setattr(_PipedWslChild, "first", first)
    monkeypatch.setattr(_PipedWslChild, "close_after_first", close_after_first)
    monkeypatch.setattr(_PipedWslChild, "wait_timeouts", wait_timeouts)
    monkeypatch.setattr(lockstep_wsl, "proc", proc)
    return calls


def test_a_child_that_writes_after_spawn_keeps_its_pipe_and_its_output_lands_in_the_sidecar(
        tmp_path, monkeypatch):
    """Todo 46 ``relay closed``: the caller drops what spawn returns, as
    ``WslSitl.start`` once did with the SITL's ``Popen``. The pipe's read end
    must stay open (a closed one is a ``SIGPIPE`` for the WSL child, which then
    dies), and every line written after spawn reaches the ``*_host`` sidecar."""
    import gc
    import os

    _piped_wsl(monkeypatch)
    record = tmp_path / lockstep_wsl.PROCESSES_FILE
    lockstep_wsl_spawn.spawn("sitl", "x", "/root/lockstep-a/0", record, held=[])  # all dropped
    _PipedWslChild.latest = []
    gc.collect()
    lines = [b"Debug line %d after boot\n" % k for k in range(3)]
    for line in lines:
        os.write(_PipedWslChild.write_fd, line)  # fails if the read end were closed
    os.close(_PipedWslChild.write_fd)  # the child exits: EOF ends the drain
    for thread in threading.enumerate():
        if thread.name == "lockstep-wsl-sitl-output":
            thread.join(5)
    sidecar = lockstep_wsl_spawn.output_sidecar(record, "sitl")
    assert sidecar.name == "lockstep_wsl_sitl_host.log"
    assert sidecar.read_bytes() == b"".join(lines)


def test_spawn_waits_for_the_ready_line_and_keeps_it_in_the_sidecar(tmp_path, monkeypatch):
    import os

    _piped_wsl(monkeypatch, b"boot\nLOCKSTEP_RELAY_READY pid=5\n")
    record = tmp_path / lockstep_wsl.PROCESSES_FILE
    held: list = []
    child = lockstep_wsl_spawn.spawn("relay", "x", "/root/lockstep-a/0", record, held=held,
                                     ready="LOCKSTEP_RELAY_READY")
    assert held == [child]
    os.write(_PipedWslChild.write_fd, b"LOCKSTEP_RELAY_DONE frames=3\n")
    os.close(_PipedWslChild.write_fd)
    child.close(timeout_s=5)
    assert child.closed and child.process.stdout.closed
    assert lockstep_wsl_spawn.output_sidecar(record, "relay").read_bytes() == (
        b"boot\nLOCKSTEP_RELAY_READY pid=5\nLOCKSTEP_RELAY_DONE frames=3\n")


def test_a_failed_spawn_after_go_stops_the_child_and_releases_every_host_resource(
        tmp_path, monkeypatch):
    """Codex 9h round 1 P2-2: the relay exits before its ready line. Spawn stops
    the recorded child, reaps ``wsl.exe``, closes its streams and the sidecar,
    then raises; the output so far is in the sidecar."""
    calls = _piped_wsl(monkeypatch, b"Traceback: bind failed\n", close_after_first=True)
    record = tmp_path / lockstep_wsl.PROCESSES_FILE
    held: list = []
    with pytest.raises(RuntimeError, match="relay exited before LOCKSTEP_RELAY_READY"):
        lockstep_wsl_spawn.spawn("relay", "x", "/root/lockstep-a/0", record, held=held,
                                 ready="LOCKSTEP_RELAY_READY")
    (child,) = held
    stdin, stdout = _PipedWslChild.latest
    assert ("stop", 321, 777, "/root/lockstep-a/0") in calls  # by its record
    assert child.closed and stdin.closed and stdout.closed and child.sink.closed
    assert b"bind failed" in lockstep_wsl_spawn.output_sidecar(record, "relay").read_bytes()


def test_an_interrupt_after_go_is_delivered_stops_the_child_by_its_record(tmp_path, monkeypatch):
    """Codex 9h round 2 P2-1: ``go`` reaches the launcher, then an interrupt.
    Its bash is in ``wait`` now and EOF no longer kills the child, so spawn
    stops it by its record, closes everything, and the interrupt is raised."""
    import io

    calls = _piped_wsl(monkeypatch)

    class InterruptAfterGo(io.BytesIO):
        def flush(self) -> None:
            super().flush()
            assert self.getvalue() == b"go\n"  # delivered
            raise KeyboardInterrupt

    real_init = _PipedWslChild.__init__

    def init(self, *args, **kwargs):
        real_init(self, *args, **kwargs)
        self.stdin = InterruptAfterGo()

    monkeypatch.setattr(_PipedWslChild, "__init__", init)
    held: list = []
    with pytest.raises(KeyboardInterrupt):
        lockstep_wsl_spawn.spawn("sitl", "x", "/root/lockstep-a/0",
                                 tmp_path / lockstep_wsl.PROCESSES_FILE, held=held)
    (child,) = held
    assert ("stop", 321, 777, "/root/lockstep-a/0") in calls
    assert child.closed and child.process.stdout.closed and child.sink.closed


@pytest.mark.parametrize("failure", ["stop_fails", "wait_times_out"])
def test_abandoning_with_a_running_drain_never_blocks_and_leaves_the_rest_for_a_retry(
        tmp_path, monkeypatch, failure):
    """Codex 9h round 2 P2-2: the drain already runs when spawn fails (an
    interrupt out of ``Thread.start``). The stop by record fails, or the
    launcher outlives its bounded wait: spawn still raises its own error at
    once, stdout stays open under the live drain, and the child stays in
    ``held``. Once the child exits, the owner's retry closes it."""
    import os

    _piped_wsl(monkeypatch, stop_fails=failure == "stop_fails",
               wait_timeouts=1 if failure == "wait_times_out" else 0)
    real_start = lockstep_wsl_spawn.WslChild.start_drain

    def start_then_interrupt(self):
        real_start(self)
        raise KeyboardInterrupt  # after the thread launched

    monkeypatch.setattr(lockstep_wsl_spawn.WslChild, "start_drain", start_then_interrupt)
    held: list = []
    with pytest.raises(KeyboardInterrupt):
        lockstep_wsl_spawn.spawn("sitl", "x", "/root/lockstep-a/0",
                                 tmp_path / lockstep_wsl.PROCESSES_FILE, held=held)
    (child,) = held
    assert not child.closed and not child.process.stdout.closed  # never under the live drain
    os.write(_PipedWslChild.write_fd, b"late line\n")
    os.close(_PipedWslChild.write_fd)  # the child exits (stopped by the owner's stop)
    child.close(timeout_s=5)
    assert child.closed and child.process.stdout.closed
    sidecar = lockstep_wsl_spawn.output_sidecar(tmp_path / lockstep_wsl.PROCESSES_FILE, "sitl")
    assert sidecar.read_bytes() == b"late line\n"


def test_a_failing_sidecar_never_stops_the_drain_and_close_raises_its_error():
    """Codex 9h round 1 P2-1: the sink fails on its first write. The output
    (more than a pipe holds) is still read to EOF, so the child never blocks;
    ``close`` reaps, closes and then raises the sink's error."""
    import os

    process = _PipedWslChild()
    process.stdout.readline()  # the pid line spawn would have read

    class FailingSink:
        closed = False

        def write(self, _data):
            raise OSError(28, "No space left on device")

        def flush(self):
            pass

        def close(self):
            self.closed = True

    sink = FailingSink()
    child = lockstep_wsl_spawn.WslChild(process, "sitl")
    child.sink = sink
    child.start_drain()
    big = b"x" * (4 << 20)  # far beyond any pipe buffer

    def child_writes():
        os.write(_PipedWslChild.write_fd, big)
        os.close(_PipedWslChild.write_fd)

    writer = threading.Thread(target=child_writes)
    writer.start()
    writer.join(20)
    assert not writer.is_alive()  # never blocked on a full pipe
    with pytest.raises(RuntimeError, match="output sidecar failed: OSError"):
        child.close(timeout_s=5)
    assert child.closed and sink.closed and process.stdout.closed
    child.close()  # closed: a second close is a no-op


class _HostChild:
    """A ``WslChild`` double for ``WslSitl.stop``: ``fails`` closes raise first."""

    def __init__(self, role: str, fails: int = 0) -> None:
        self.role = role
        self.fails = fails
        self.closed = False
        self.attempts = 0

    def close(self) -> None:
        self.attempts += 1
        if self.fails:
            self.fails -= 1
            raise subprocess.TimeoutExpired("wsl.exe", 30)
        self.closed = True


def _wsl_sitl(tmp_path):
    from scripts.lockstep_wsl_sitl import WslSitl

    sitl = WslSitl(tmp_path, 40, {}, seed=1, delay_seed=0, defaults="", tiles={})
    sitl.root = "/home/x/navpy-eval-roots/lockstep-a"
    return sitl


def test_wsl_sitl_stop_retries_a_failed_ownership_stop_and_only_then_closes(tmp_path, monkeypatch):
    sitl = _wsl_sitl(tmp_path)
    relay, sitl_child = _HostChild("relay"), _HostChild("sitl")
    sitl._children = [relay, sitl_child]
    outcomes = [RuntimeError("relay 41: TimeoutExpired"), None]

    def stop_owned(_record, _root):
        outcome = outcomes.pop(0)
        if outcome is not None:
            raise outcome
        return []

    monkeypatch.setattr(lockstep_wsl, "stop_owned", stop_owned)
    with pytest.raises(RuntimeError, match="TimeoutExpired"):
        sitl.stop()
    assert relay.attempts == sitl_child.attempts == 0  # launchers still run their children
    sitl.stop()
    assert relay.closed and sitl_child.closed and sitl._children == []


def test_wsl_sitl_stop_keeps_a_child_whose_close_failed_and_a_retry_closes_it(tmp_path, monkeypatch):
    """Codex 9h round 1 P2-3: the ownership stop succeeds once; a close that
    times out keeps its child, and the next stop closes it without stopping again."""
    sitl = _wsl_sitl(tmp_path)
    relay, sitl_child = _HostChild("relay"), _HostChild("sitl", fails=1)
    sitl._children = [relay, sitl_child]
    stops = []
    monkeypatch.setattr(lockstep_wsl, "stop_owned", lambda *_a: stops.append(1) or [])
    with pytest.raises(subprocess.TimeoutExpired):
        sitl.stop()
    assert relay.closed and sitl._children == [sitl_child]
    sitl.stop()
    assert sitl_child.closed and sitl._children == [] and stops == [1]
    assert relay.attempts == 1
