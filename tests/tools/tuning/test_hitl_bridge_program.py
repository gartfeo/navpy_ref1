"""The JSBSim program process and its input socket (tools/tuning/hitl_bridge/jsbsim_program.py), without JSBSim."""
import io
import signal
import socket
import statistics
import subprocess
import threading
import time

import pytest

from tools.tuning.hitl_bridge import jsbsim_program as jprog
from tools.tuning.hitl_bridge.jsbsim_files import JSBSimError
from tools.tuning.hitl_bridge.jsbsim_protocol import ReplyReader, batch_replies, expect_replies

# --- the input socket: replies written as separate small sends with Nagle on ---------------
# JSBSim (FGfdmSocket) writes each reply line and each prompt with its own send() and no
# TCP_NODELAY. On Linux the receiver's delayed ACK then holds every next segment for at least
# TCP_DELACK_MIN = 40 ms (measured in WSL without the fix: median 50 ms per batch). With
# TCP_QUICKACK re-armed after each receive the same exchange took a median 0.06 ms.
REPLY_BATCH_BOUND_S = 0.02             # half of Linux's minimum delayed-ACK stall; ~300x the fixed median
BATCHES = 40                           # Linux starts a connection in quick-ACK mode for its first segments
REPLY_PIECES = [b"set successful\n", b"JSBSim> "] * 4 + [b"Iterations performed\n", b"JSBSim> "]
BATCH = b"set a 1\nset b 2\nset c 3\nset d 4\niterate 6\n"


def _serve_replies_like_jsbsim(server: socket.socket) -> None:
    conn, _ = server.accept()                       # Nagle stays on, as in JSBSim
    buffer = b""
    try:
        for _ in range(BATCHES):
            while b"iterate" not in buffer:
                buffer += conn.recv(4096)
            buffer = buffer.split(b"iterate", 1)[1].split(b"\n", 1)[1]
            for piece in REPLY_PIECES:
                conn.send(piece)
    finally:
        conn.close()


def test_replies_sent_as_separate_segments_with_nagle_on_are_not_held_by_delayed_acks():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    thread = threading.Thread(target=_serve_replies_like_jsbsim, args=(server,), daemon=True)
    thread.start()
    control = jprog.ControlSocket(socket.create_connection(server.getsockname(), timeout=5))
    reader = ReplyReader()
    durations = []
    try:
        for _ in range(BATCHES):
            start = time.perf_counter()
            control.send(BATCH)
            expect_replies(control.read_replies, reader, batch_replies(["a", "b", "c", "d"]),
                           deadline=time.monotonic() + 5.0, clock=time.monotonic)
            durations.append(time.perf_counter() - start)
    finally:
        control.close()
        thread.join(timeout=5)
        server.close()
    assert statistics.median(durations) < REPLY_BATCH_BOUND_S, durations


def test_no_reply_yet_is_empty_and_a_closed_input_socket_is_an_error():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 0))
    server.listen(1)
    control = jprog.ControlSocket(socket.create_connection(server.getsockname(), timeout=5))
    conn, _ = server.accept()
    try:
        assert control.read_replies(0.05) == b""
        conn.close()                                # the program ended
        with pytest.raises(JSBSimError, match="closed its input socket"):
            control.read_replies(1.0)
    finally:
        control.close()
        server.close()


# --- the program process: refusals and stop by pid ------------------------------------------
class FakeProc:
    def __init__(self, events, console=b"JSBSim startup beginning ...\nJSBSim Execution beginning ...\n",
                 exits_on=(signal.SIGTERM,), exited=False):
        self.pid = 4242
        self.events = events
        self.stdout = io.BytesIO(console)
        self.exits_on = exits_on
        self.returncode = 1 if exited else None

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if self.returncode is None:
            raise subprocess.TimeoutExpired("JSBSim", timeout)
        return self.returncode

    def kill_by_group(self, pgid, signum):
        self.events.append(("killpg", pgid, signum))
        if signum in self.exits_on:
            self.returncode = -signum


class FakeControl:
    def __init__(self, events):
        self.events = events

    def setsockopt(self, *args):
        pass

    def close(self):
        self.events.append(("close control",))


def start_program(tmp_path, events, proc=None, processes=(), connect_error=None, check_other_programs=True):
    proc = proc or FakeProc(events)
    launched = []

    def popen(cmd, **kwargs):
        launched.append((cmd, kwargs))
        return proc

    def connect(address, timeout):
        if connect_error:
            raise connect_error
        events.append(("connect", address))
        return FakeControl(events)

    config = jprog.ProgramConfig(program="/usr/games/JSBSim", control_port=0, state_port=0, run_root=tmp_path)
    program = jprog.JSBSimProgram(config, tmp_path, 1200.0, 1, check_other_programs, popen=popen,
                                  killpg=proc.kill_by_group, processes=lambda: list(processes), connect=connect)
    return program, proc, launched


def test_another_jsbsim_process_is_refused_before_anything_starts(tmp_path):
    events = []
    for argv in (["/usr/games/JSBSim", "--script=x"], ["JSBSim", "--suspend"]):     # ours, or ArduPilot's child
        with pytest.raises(JSBSimError, match="pid 77"):
            start_program(tmp_path, events, processes=[(1, ["bash"]), (77, argv)])
    assert events == []
    assert jprog.jsbsim_pids([(5, ["python3", "-m", "x", "JSBSim"]), (6, ["/opt/JSBSim2"])],
                             "/usr/games/JSBSim") == []


def test_a_restart_does_not_look_for_other_jsbsim_programs(tmp_path):
    events = []
    program, _, launched = start_program(tmp_path, events, processes=[(77, ["JSBSim", "--suspend"])],
                                         check_other_programs=False)
    assert launched and program.running()
    program.stop()


def test_busy_control_or_state_ports_are_refused(tmp_path):
    taken_tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    taken_tcp.bind(("0.0.0.0", 0))
    taken_tcp.listen(1)
    taken_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    taken_udp.bind(("127.0.0.1", 0))
    try:
        with pytest.raises(JSBSimError, match="TCP port .* busy"):
            jprog.check_tcp_port_free(taken_tcp.getsockname()[1])
        with pytest.raises(JSBSimError, match="UDP port .* busy"):
            jprog.bind_state_socket(taken_udp.getsockname()[1])
    finally:
        taken_tcp.close()
        taken_udp.close()


def test_the_program_runs_in_its_own_group_and_stop_closes_the_socket_then_signals_its_pid(tmp_path):
    events = []
    program, proc, launched = start_program(tmp_path, events)
    cmd, kwargs = launched[0]
    assert cmd[1] == "--suspend" and kwargs["start_new_session"] is True
    assert kwargs["stdin"] == subprocess.DEVNULL and kwargs["cwd"] == str(tmp_path)
    assert program.running() and program.pid == 4242
    program.stop()
    program.stop()                                               # idempotent
    assert events == [("connect", ("127.0.0.1", 0)), ("close control",), ("killpg", 4242, signal.SIGTERM)]
    assert "JSBSim Execution beginning" in (tmp_path / "jsbsim_1.log").read_text()


def test_a_program_ignoring_sigterm_is_killed_by_its_pid(tmp_path):
    events = []
    sigkill = getattr(signal, "SIGKILL", signal.SIGTERM)
    proc = FakeProc(events, exits_on=(sigkill,) if sigkill != signal.SIGTERM else (signal.SIGTERM,))
    program, _, _ = start_program(tmp_path, events, proc=proc)
    if sigkill != signal.SIGTERM:
        original_wait = proc.wait
        waits = iter([True])

        def wait(timeout=None):                                  # SIGTERM ignored: the first wait times out
            if next(waits, False):
                raise subprocess.TimeoutExpired("JSBSim", timeout)
            return original_wait(timeout)
        proc.wait = wait
    program.stop()
    kills = [e for e in events if e[0] == "killpg"]
    assert kills[-1] == ("killpg", 4242, sigkill) and all(e[1] == 4242 for e in kills)


def test_a_program_that_exits_during_start_up_is_reported_and_not_signalled(tmp_path):
    events = []
    proc = FakeProc(events, console=b"Could not open file: start.xml\n", exited=True)
    with pytest.raises(JSBSimError, match="exited") as info:
        start_program(tmp_path, events, proc=proc)
    assert "Could not open file" in str(info.value)
    assert not [e for e in events if e[0] == "killpg"]


def test_a_failure_after_the_launch_stops_the_program_by_its_pid(tmp_path):
    events = []
    with pytest.raises(ConnectionRefusedError):
        start_program(tmp_path, events, connect_error=ConnectionRefusedError("no input socket"))
    assert events == [("killpg", 4242, signal.SIGTERM)]


def test_the_state_socket_is_released_on_stop(tmp_path):
    events = []
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    proc = FakeProc(events)
    config = jprog.ProgramConfig(control_port=0, state_port=port, run_root=tmp_path)
    program = jprog.JSBSimProgram(config, tmp_path, 1200.0, 1, popen=lambda cmd, **kw: proc,
                                  killpg=proc.kill_by_group, processes=list,
                                  connect=lambda address, timeout: FakeControl(events))
    with pytest.raises(JSBSimError):
        jprog.bind_state_socket(port)                            # held while the program runs
    program.stop()
    jprog.bind_state_socket(port).close()                        # free again
