"""The HITL processes on the PC: the JSBSim bridge, and with a WSL bridge the Windows serial relay.

Direct (no --relay, the recommended setup): the bridge runs on Windows next to the board, opens the board's HIL
USB port itself (--device COM11) and steps the jsbsim package in-process (--plant package). It counts as up once
its first console line ("bridge: <device> <-> <model> ...", printed after the physics is loaded) is in its output.
A WSL bridge cannot meet the 5 ms loop: Windows-WSL TCP hops stalled 40-160 ms and the WSL physics fell behind
under load (first board runs, 2026-10-13).

With a relay (a WSL bridge), in this order: the bridge (it listens on a TCP port
inside WSL, reached from Windows through WSL's localhost forwarding), then, once that port
accepts connections, the relay that joins the board's HIL USB port (COM11, OTG2) to it.
Stopped in reverse order: the relay first, after which the bridge stops itself and its
JSBSim (its relay connection stays closed past the bridge's grace time, or its input ends:
the runner closes it); the runner then waits for the bridge process to end. Processes are
only ever stopped through their own handles, never by name. Both outputs go to the run folder.

The two command lines are templates; the runner fills in {python} (this interpreter),
{port}, {lat} {lon} {alt} {hdg} (the origin, for the bridge's --origin-lat/--origin-lon/
--origin-alt/--heading), {link_csv}, {run_dir} and {repo} (Windows paths) and {link_csv_wsl}
{run_dir_wsl} {repo_wsl} (the same paths as WSL sees them). The bridge must write its link
log (--log-csv) to {link_csv} or {link_csv_wsl}: the run's link-health gate reads it.
"""
from __future__ import annotations

import os
import re
import shlex
import socket
import subprocess
import sys
import time
from pathlib import Path, PureWindowsPath
from typing import IO, Callable

from tools.tuning.hitl_params import Refused

REPO_ROOT = Path(__file__).resolve().parents[2]
LOOPBACK = "127.0.0.1"
LISTEN_TIMEOUT_S = 90.0        # wsl.exe start, JSBSim loading the model, the bridge binding its port (or, direct, its first line)
LISTEN_POLL_S = 0.5            # Windows gives no event when a port inside WSL starts listening
PROBE_TIMEOUT_S = 1.0          # one connection attempt to the bridge's port
RELAY_STOP_TIMEOUT_S = 10.0    # the relay holds no state: terminate, then wait this long before kill
# With --exit-on-stdin-eof the bridge ends as soon as its input closes; without it, after its
# --client-grace-s (default 30 s, hitl_bridge/bridge.py DEFAULT_CLIENT_GRACE_S) with no relay
# connection. This covers that default grace plus the JSBSim program's shutdown.
BRIDGE_EXIT_TIMEOUT_S = 45.0
BRIDGE_OUTPUT = "bridge_stdout.txt"
RELAY_OUTPUT = "relay_stdout.txt"
LINK_CSV = "link.csv"          # the bridge's link log (--log-csv) in the run folder
ORIGIN_FIELDS = ("lat", "lon", "alt", "hdg")
LINK_CSV_FIELDS = ("link_csv", "link_csv_wsl")
FIELDS = ("python", "port", *ORIGIN_FIELDS, *LINK_CSV_FIELDS, "run_dir", "repo", "run_dir_wsl", "repo_wsl")
PLACEHOLDER = re.compile(r"\{([a-z_]+)\}")
# bridge.py _serve's first line, printed (flushed) once the physics is up and the board port is set up
BRIDGE_READY = re.compile(r"^bridge: .+ <-> ", re.MULTILINE)
TCP_LISTEN_DEVICE = "--device tcpin:"


def wsl_path(path: Path) -> str:
    """The folder as WSL sees it: C:\\a\\b -> /mnt/c/a/b; a POSIX path stays as it is."""
    win = PureWindowsPath(str(path))
    if not win.drive:
        return Path(path).as_posix()
    if not re.fullmatch(r"[A-Za-z]:", win.drive):
        raise ValueError(f"{path}: not on a drive letter, so WSL cannot reach it under /mnt")
    return "/".join(["/mnt", win.drive[0].lower(), *win.parts[1:]])


def command_fields(origin: str, port: int, run_dir: Path) -> dict[str, str]:
    lat, lon, alt, hdg = (p.strip() for p in origin.split(","))
    return {"python": sys.executable, "port": str(port), "lat": lat, "lon": lon, "alt": alt, "hdg": hdg,
            "link_csv": str(run_dir / LINK_CSV), "link_csv_wsl": f"{wsl_path(run_dir)}/{LINK_CSV}",
            "run_dir": str(run_dir), "repo": str(REPO_ROOT), "run_dir_wsl": wsl_path(run_dir),
            "repo_wsl": wsl_path(REPO_ROOT)}


def fill(template: str, fields: dict[str, str]) -> str:
    unknown = sorted(set(PLACEHOLDER.findall(template)) - set(fields))
    if unknown:
        raise ValueError(f"unknown placeholder {unknown} in {template!r}; known: {', '.join(FIELDS)}")
    return PLACEHOLDER.sub(lambda m: fields[m.group(1)], template)


def template_errors(bridge: str, relay: str | None) -> list[str]:
    """What is wrong with the command templates (relay None: the bridge opens the board's port itself)."""
    errors = []
    for name, template in (("--bridge", bridge), ("--relay", relay)):
        if template is None:
            continue
        used = set(PLACEHOLDER.findall(template))
        if used - set(FIELDS):
            errors.append(f"{name}: unknown placeholder {sorted(used - set(FIELDS))}; known: {', '.join(FIELDS)}")
        if relay is not None and "port" not in used:
            errors.append(f"{name} must use {{port}}, the bridge's TCP port")
    if relay is None and TCP_LISTEN_DEVICE in bridge:
        errors.append("--bridge without --relay must open the board's HIL port itself (--device COM11), not listen "
                      "for a relay (tcpin:)")
    used = set(PLACEHOLDER.findall(bridge))
    if not set(ORIGIN_FIELDS) <= used:
        errors.append("--bridge must take the origin from {lat} {lon} {alt} {hdg} (--origin-lat {lat} --origin-lon "
                      "{lon} --origin-alt {alt} --heading {hdg}), so the bridge starts where the board's SIM_OPOS_* say")
    if not set(LINK_CSV_FIELDS) & used:
        errors.append("--bridge must write its link log to {link_csv_wsl} or {link_csv} (--log-csv): the run's "
                      "link-health gate reads it")
    return errors


def port_listens(port: int) -> bool:
    try:
        with socket.create_connection((LOOPBACK, port), timeout=PROBE_TIMEOUT_S):
            return True
    except OSError:
        return False


def _args(command: str) -> str | list[str]:
    """CreateProcess parses a command string on Windows; elsewhere it is split like a shell would."""
    return command if os.name == "nt" else shlex.split(command)


class HitlProcesses:
    """The bridge and the relay of one HITL run."""

    def __init__(self, bridge: str, relay: str | None, port: int, run_dir: Path, origin: str,
                 popen: Callable[..., subprocess.Popen] = subprocess.Popen,
                 listening: Callable[[int], bool] = port_listens) -> None:
        self.run_dir = Path(run_dir)
        self.port = port
        self.link_csv = self.run_dir / LINK_CSV
        fields = command_fields(origin, port, self.run_dir)
        self.bridge_cmd = fill(bridge, fields)
        self.relay_cmd = fill(relay, fields) if relay is not None else None
        self._popen = popen
        self._listening = listening
        self._outputs: list[IO[str]] = []
        self.bridge: subprocess.Popen | None = None
        self.relay: subprocess.Popen | None = None

    def start(self) -> dict:
        if self.relay_cmd is None:
            self.bridge = self._spawn(self.bridge_cmd, BRIDGE_OUTPUT, subprocess.PIPE)   # its input: stop on EOF
            self._wait_until(self._bridge_ready, "printing its first line", "print its first line (physics up)")
            return {"bridge": self.bridge_cmd, "bridge_pid": self.bridge.pid, "relay": None}
        if self._listening(self.port):
            raise Refused(f"TCP port {self.port} already accepts connections: a bridge from an earlier run may "
                          "still run in WSL; stop it by its pid first")
        self.bridge = self._spawn(self.bridge_cmd, BRIDGE_OUTPUT, subprocess.PIPE)   # its input: stop on EOF
        self._wait_until(lambda: self._listening(self.port), f"listening on port {self.port}",
                         f"listen on port {self.port}")
        self.relay = self._spawn(self.relay_cmd, RELAY_OUTPUT, subprocess.DEVNULL)
        return {"bridge": self.bridge_cmd, "bridge_pid": self.bridge.pid, "relay": self.relay_cmd,
                "relay_pid": self.relay.pid, "port": self.port}

    def _spawn(self, command: str, output: str, stdin: int) -> subprocess.Popen:
        out = open(self.run_dir / output, "w")
        self._outputs.append(out)
        return self._popen(_args(command), stdin=stdin, stdout=out, stderr=subprocess.STDOUT)

    def _bridge_ready(self) -> bool:
        for out in self._outputs:
            out.flush()
        try:
            text = (self.run_dir / BRIDGE_OUTPUT).read_text(encoding="utf-8", errors="replace")
        except OSError:
            return False
        return BRIDGE_READY.search(text) is not None

    def _wait_until(self, ready: Callable[[], bool], before: str, failed: str) -> None:
        deadline = time.time() + LISTEN_TIMEOUT_S
        while not ready():
            if self.bridge.poll() is not None:
                raise RuntimeError(f"the bridge exited with code {self.bridge.returncode} before {before}; "
                                   f"see {BRIDGE_OUTPUT}")
            if time.time() > deadline:
                raise RuntimeError(f"the bridge did not {failed} within {LISTEN_TIMEOUT_S:g} s; see {BRIDGE_OUTPUT}")
            time.sleep(LISTEN_POLL_S)

    def check(self) -> None:
        """Raise if the bridge or the relay has ended: the board would fly on stale state."""
        for name, proc, output in (("bridge", self.bridge, BRIDGE_OUTPUT), ("relay", self.relay, RELAY_OUTPUT)):
            if proc is not None and proc.poll() is not None:
                raise RuntimeError(f"the {name} exited with code {proc.returncode}; see {output}")

    def stop(self) -> dict:
        """Relay first, then wait for the bridge to stop itself. Raises if it does not."""
        record = {}
        try:
            if self.relay is not None:
                record["relay_exit"] = self._end_relay()
            if self.bridge is not None:
                record["bridge_exit"] = self._end_bridge()
        finally:
            for out in self._outputs:
                out.close()
            self._outputs = []
        return record

    def _end_relay(self) -> int:
        if self.relay.poll() is None:
            self.relay.terminate()
            try:
                self.relay.wait(timeout=RELAY_STOP_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                self.relay.kill()
                self.relay.wait()
        return self.relay.returncode

    def _end_bridge(self) -> int:
        if self.bridge.stdin is not None:
            try:
                self.bridge.stdin.close()
            except OSError:          # the pipe broke because the bridge already ended
                pass
        try:
            return self.bridge.wait(timeout=BRIDGE_EXIT_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            raise RuntimeError(f"the bridge (pid {self.bridge.pid}) did not exit within {BRIDGE_EXIT_TIMEOUT_S:g} s "
                               "after the relay stopped and its input closed; its JSBSim may still run in WSL: "
                               "find it there (ps) and stop it by its pid") from None
