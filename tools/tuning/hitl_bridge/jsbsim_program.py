"""The JSBSim program process: refusals, start, its sockets, stop by its own pid.

- Another JSBSim program (ours or ArduPilot SITL's child "JSBSim") is refused on the first
  start of a bridge, before anything starts; a restart after a board reboot checks only
  the ports (busy control or state port refused), which is the real conflict.
- The program runs in its own session (process group = its pid) and, on Linux, gets SIGTERM
  from the kernel when the bridge dies, even by SIGKILL (PR_SET_PDEATHSIG).
- The bridge closes its input-socket connection before stopping the program: if the program
  closes first, its listening port stays in TIME_WAIT and the next start cannot listen on it
  (probe: "Connection refused" on the restart).
- TCP_QUICKACK on the input socket, re-armed after every receive (Linux): JSBSim writes each
  reply and each prompt as a separate small send with Nagle on, so the next one waits for our
  ACK, which Linux delays by at least 40 ms (TCP_DELACK_MIN). Measured in WSL: a batch's
  five replies took a median 50 ms without it, 0.06 ms with it (hitl/bridge_fix/nagle_check.py).
"""
from __future__ import annotations

import os
import select
import signal
import socket
import subprocess
import sys
import tempfile
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional, Protocol, Sequence

from tools.tuning.hitl_bridge.jsbsim_files import LOCALHOST, JSBSimError, jsbsim_command
from tools.tuning.hitl_bridge.jsbsim_protocol import EXECUTION_BEGINNING

JSBSIM_PROGRAM = "/usr/games/JSBSim"
JSBSIM_PROCESS_NAME = "JSBSim"          # argv[0] base name, also of ArduPilot's SITL child (execlp "JSBSim")
# Our own ports, outside ArduPilot's JSBSim ports (5504/5505 + 10*instance), the GCS bands
# and the bench's 17000-19999.
DEFAULT_CONTROL_PORT = 9621             # the program's input socket (TCP, it listens)
DEFAULT_STATE_PORT = 9622               # the program's state output (UDP, we listen)
ANY_ADDRESS = "0.0.0.0"                 # where the program listens; the busy check binds the same
STARTUP_TIMEOUT_S = 10.0                # model load took 0.05 s in the probe
STOP_TIMEOUT_S = 3.0                    # SIGTERM to exit, then SIGKILL
STATE_RCVBUF_BYTES = 1 << 20            # asked for; Linux caps it at net.core.rmem_max
RECV_BYTES = 65536                      # larger than any reply burst or state datagram
CONSOLE_TAIL_LINES = 15
PR_SET_PDEATHSIG = 1                    # linux/prctl.h
TCP_QUICKACK = getattr(socket, "TCP_QUICKACK", None)     # Linux only (see the module docstring)


def list_processes() -> list[tuple[int, list[str]]]:
    """(pid, argv) of every process readable in /proc; empty where there is no /proc."""
    procs = []
    proc_root = Path("/proc")
    if not proc_root.is_dir():
        return procs
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
        except OSError:
            continue
        argv = [part.decode(errors="replace") for part in raw.split(b"\0") if part]
        if argv:
            procs.append((int(entry.name), argv))
    return procs


def jsbsim_pids(processes: Sequence[tuple[int, list[str]]], program: str) -> list[int]:
    return [pid for pid, argv in processes
            if argv[0] == program or os.path.basename(argv[0]) == JSBSIM_PROCESS_NAME]


def check_tcp_port_free(port: int, host: str = ANY_ADDRESS) -> None:
    """Refuse a port another socket uses, TIME_WAIT included (the program does not reuse)."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        probe.bind((host, port))
    except OSError as exc:
        raise JSBSimError(f"TCP port {port} is busy ({exc}); pick another --jsbsim-control-port") from exc
    finally:
        probe.close()


def bind_state_socket(port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((LOCALHOST, port))
    except OSError as exc:
        sock.close()
        raise JSBSimError(f"UDP port {port} is busy ({exc}); pick another --jsbsim-state-port") from exc
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, STATE_RCVBUF_BYTES)
    return sock


class ControlSocket:
    """The program's input socket (TCP, the program listens): command batches out, replies in."""

    def __init__(self, sock: socket.socket) -> None:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = sock
        self._quick_ack()

    def _quick_ack(self) -> None:
        """ACK the next segment at once (Linux leaves quick-ACK mode again after a receive)."""
        if TCP_QUICKACK is not None:
            self._sock.setsockopt(socket.IPPROTO_TCP, TCP_QUICKACK, 1)

    def send(self, data: bytes) -> None:
        try:
            self._sock.sendall(data)
        except OSError as exc:
            raise JSBSimError(f"JSBSim input socket failed ({exc})") from exc

    def read_replies(self, timeout_s: float) -> bytes:
        """Reply bytes, b'' when none came within timeout_s; JSBSimError if the socket closed."""
        ready, _, _ = select.select([self._sock], [], [], timeout_s)
        if not ready:
            return b""
        try:
            data = self._sock.recv(RECV_BYTES)
        except OSError as exc:
            raise JSBSimError(f"JSBSim input socket failed ({exc})") from exc
        if not data:
            raise JSBSimError("JSBSim closed its input socket")
        self._quick_ack()
        return data

    def close(self) -> None:
        self._sock.close()


class ProgramIO(Protocol):
    """A started JSBSim program: its input socket, its state output, its process."""

    def send(self, data: bytes) -> None: ...

    def read_replies(self, timeout_s: float) -> bytes:
        """Reply bytes, b'' when none came within timeout_s; JSBSimError if the socket closed."""

    def read_datagram(self, timeout_s: float) -> Optional[bytes]:
        """One state datagram, or None when none came within timeout_s (0: do not wait)."""

    def running(self) -> bool: ...

    def console_tail(self) -> str: ...

    def stop(self) -> None:
        """Close the input socket first, then stop the program by its pid; idempotent."""


@dataclass(frozen=True)
class ProgramConfig:
    program: str = JSBSIM_PROGRAM
    control_port: int = DEFAULT_CONTROL_PORT
    state_port: int = DEFAULT_STATE_PORT
    run_root: Path = field(default_factory=lambda: Path(tempfile.gettempdir()) / "navpy-hitl-jsbsim")


def _parent_death_signal() -> Optional[Callable[[], None]]:
    """A preexec_fn (Linux) that makes the kernel send SIGTERM to the program when the
    bridge dies, even by SIGKILL, so it never keeps spinning on its own. The libc function
    is looked up here in the parent; the forked child only calls it."""
    if not sys.platform.startswith("linux"):
        return None
    import ctypes  # noqa: PLC0415 - Linux only
    prctl = ctypes.CDLL(None, use_errno=True).prctl
    sigterm = int(signal.SIGTERM)
    return lambda: prctl(PR_SET_PDEATHSIG, sigterm)


class JSBSimProgram:
    """The JSBSim program in its own process group, its sockets and its console log."""

    def __init__(self, config: ProgramConfig, run_dir: Path, rate_hz: float, start_index: int,
                 check_other_programs: bool = True,
                 popen: Callable[..., subprocess.Popen] = subprocess.Popen,
                 killpg: Optional[Callable[[int, int], None]] = getattr(os, "killpg", None),
                 processes: Callable[[], list] = list_processes,
                 connect: Callable[..., socket.socket] = socket.create_connection) -> None:
        self.config = config
        self.log_path = run_dir / f"jsbsim_{start_index}.log"
        self._killpg = killpg
        self._proc: Optional[subprocess.Popen] = None
        self._control: Optional[ControlSocket] = None
        self._state: Optional[socket.socket] = None
        self._reader: Optional[threading.Thread] = None
        self._console_event = threading.Event()     # set on EXECUTION_BEGINNING or at the console's end
        self._begun = False
        self._console: list[str] = []
        others = jsbsim_pids(processes(), config.program) if check_other_programs else []
        if others:
            raise JSBSimError(f"another JSBSim program is running (pid {', '.join(map(str, others))}); "
                              "stop it first (this bridge stops only the program it started)")
        check_tcp_port_free(config.control_port)
        self._state = bind_state_socket(config.state_port)
        try:
            self._launch(run_dir, rate_hz, popen)
            self._control = ControlSocket(connect((LOCALHOST, config.control_port), timeout=STARTUP_TIMEOUT_S))
        except BaseException:
            self.stop()
            raise

    @property
    def pid(self) -> Optional[int]:
        return None if self._proc is None else self._proc.pid

    def _launch(self, run_dir: Path, rate_hz: float, popen: Callable[..., subprocess.Popen]) -> None:
        cmd = jsbsim_command(self.config.program, run_dir, rate_hz)
        try:
            self._proc = popen(cmd, cwd=str(run_dir), stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, start_new_session=True,
                               preexec_fn=_parent_death_signal())
        except OSError as exc:
            raise JSBSimError(f"cannot start {self.config.program}: {exc}") from exc
        self._reader = threading.Thread(target=self._read_console, name="jsbsim-console", daemon=True)
        self._reader.start()
        self._console_event.wait(STARTUP_TIMEOUT_S)
        if not self._begun:
            state = "exited" if self._proc.poll() is not None else f"did not start within {STARTUP_TIMEOUT_S} s"
            raise JSBSimError(f"JSBSim {state} (pid {self._proc.pid}):\n{self.console_tail()}")

    def _read_console(self) -> None:
        """Copy the console to the log (so the pipe never fills) and watch for the start."""
        assert self._proc is not None and self._proc.stdout is not None
        try:
            with open(self.log_path, "w", encoding="utf-8") as log:
                for raw in self._proc.stdout:
                    line = raw.decode(errors="replace").rstrip()
                    log.write(line + "\n")
                    log.flush()
                    self._console.append(line)
                    del self._console[:-CONSOLE_TAIL_LINES]
                    if EXECUTION_BEGINNING in line:
                        self._begun = True
                        self._console_event.set()
        except (OSError, ValueError):       # the pipe was closed under us at stop()
            pass
        finally:
            self._console_event.set()

    def console_tail(self) -> str:
        return "\n".join(self._console[-CONSOLE_TAIL_LINES:])

    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def send(self, data: bytes) -> None:
        assert self._control is not None
        self._control.send(data)

    def read_replies(self, timeout_s: float) -> bytes:
        assert self._control is not None
        return self._control.read_replies(timeout_s)

    def read_datagram(self, timeout_s: float) -> Optional[bytes]:
        assert self._state is not None
        ready, _, _ = select.select([self._state], [], [], timeout_s)
        return self._state.recv(RECV_BYTES) if ready else None

    def stop(self) -> None:
        if self._control is not None:          # first, so TIME_WAIT is on our side, not on its port
            self._control.close()
            self._control = None
        if self._proc is not None and self._proc.poll() is None:
            self._signal(signal.SIGTERM)
            try:
                self._proc.wait(timeout=STOP_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self._signal(getattr(signal, "SIGKILL", signal.SIGTERM))
                self._proc.wait(timeout=STOP_TIMEOUT_S)
        if self._reader is not None:            # ends at the console's end, after the exit
            self._reader.join(timeout=STOP_TIMEOUT_S)
            self._reader = None
        if self._proc is not None and self._proc.stdout is not None:
            self._proc.stdout.close()
        if self._state is not None:
            self._state.close()
            self._state = None

    def _signal(self, signum: int) -> None:
        """To the program's own process group, whose id is its pid (start_new_session)."""
        assert self._proc is not None
        try:
            if self._killpg is not None:
                self._killpg(self._proc.pid, signum)
            else:
                self._proc.send_signal(signum)
        except ProcessLookupError:
            pass


Launcher = Callable[[ProgramConfig, Path, float, int, bool], ProgramIO]


def launch_program(config: ProgramConfig, run_dir: Path, rate_hz: float, start_index: int,
                   check_other_programs: bool) -> ProgramIO:
    return JSBSimProgram(config, run_dir, rate_hz, start_index, check_other_programs)
