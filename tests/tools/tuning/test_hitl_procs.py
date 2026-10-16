"""The HITL processes (tools/tuning/hitl_procs.py): the bridge starts first and must listen
before the relay starts; the relay stops first, then the bridge must end by itself. Fake
processes only."""
from __future__ import annotations

import re
import subprocess
from pathlib import Path

import pytest

from tools.tuning import hitl, hitl_procs
from tools.tuning.hitl_bridge import relay as hitl_relay
from tools.tuning.hitl_bridge.bridge import build_parser as bridge_parser
from tools.tuning.hitl_params import Refused
from tools.tuning.hitl_procs import (BRIDGE_OUTPUT, LINK_CSV, RELAY_OUTPUT, REPO_ROOT, HitlProcesses, command_fields,
                                     fill, template_errors, wsl_path)

ORIGIN = "40.2,44.4,1300,270"
BRIDGE = ("wsl.exe --cd {repo_wsl} -- python3 -m tools.tuning.hitl_bridge --device tcpin:{port} --origin-lat {lat} "
          "--origin-lon {lon} --origin-alt {alt} --heading {hdg} --exit-on-stdin-eof --log-csv {link_csv_wsl}")
RELAY = "{python} -m tools.tuning.hitl_bridge.relay --device COM11 --bridge 127.0.0.1:{port}"


class FakeProc:
    def __init__(self, log: list[str], name: str, exits_on_eof: bool = True) -> None:
        self.log, self.name, self.exits_on_eof = log, name, exits_on_eof
        self.returncode: int | None = None
        self.pid = 1000 + len(log)
        self.stdin = self if name == "bridge" else None

    def poll(self) -> int | None:
        return self.returncode

    def close(self) -> None:              # the bridge's stdin
        self.log.append("bridge stdin closed")
        if self.exits_on_eof:
            self.returncode = 0

    def terminate(self) -> None:
        self.log.append(f"{self.name} terminated")
        self.returncode = 1

    def kill(self) -> None:
        self.log.append(f"{self.name} killed")
        self.returncode = -9

    def wait(self, timeout: float | None = None) -> int:
        if self.returncode is None:
            raise subprocess.TimeoutExpired(self.name, timeout)
        return self.returncode


class World:
    """Records what starts and stops in which order; the port listens once the bridge has run."""

    def __init__(self, listen_after_polls: int = 2, exits_on_eof: bool = True, already_listening: bool = False):
        self.log: list[str] = []
        self.procs: dict[str, FakeProc] = {}
        self.polls = 0
        self.listen_after_polls = listen_after_polls
        self.exits_on_eof = exits_on_eof
        self.already_listening = already_listening

    def popen(self, args: object, **kwargs: object) -> FakeProc:
        name = "bridge" if "--device tcpin:" in str(args) else "relay"
        self.log.append(f"{name} started")
        self.procs[name] = FakeProc(self.log, name, self.exits_on_eof)
        self.procs[name].kwargs = kwargs
        return self.procs[name]

    def listening(self, port: int) -> bool:
        if "bridge" not in self.procs:
            return self.already_listening
        self.polls += 1
        listens = self.polls > self.listen_after_polls
        if listens and "listening" not in self.log:
            self.log.append("listening")
        return listens


def _procs(world: World, tmp_path: Path) -> HitlProcesses:
    return HitlProcesses(BRIDGE, RELAY, 18570, tmp_path, ORIGIN, popen=world.popen, listening=world.listening)


@pytest.fixture(autouse=True)
def _fast(monkeypatch):
    monkeypatch.setattr(hitl_procs, "LISTEN_POLL_S", 0.0)


def test_bridge_listens_before_the_relay_starts_and_the_relay_stops_first(tmp_path):
    world = World()
    procs = _procs(world, tmp_path)
    record = procs.start()
    assert world.log == ["bridge started", "listening", "relay started"]
    assert world.procs["bridge"].kwargs["stdin"] == subprocess.PIPE      # the bridge also stops on EOF
    assert record["port"] == 18570 and record["bridge_pid"] != record["relay_pid"]
    procs.check()
    assert procs.stop() == {"relay_exit": 1, "bridge_exit": 0}
    assert world.log[3:] == ["relay terminated", "bridge stdin closed"]
    assert (tmp_path / BRIDGE_OUTPUT).exists() and (tmp_path / RELAY_OUTPUT).exists()


def test_commands_get_the_origin_port_and_both_path_forms(tmp_path):
    procs = _procs(World(), tmp_path / "run")
    assert "tcpin:18570 --origin-lat 40.2 --origin-lon 44.4 --origin-alt 1300 --heading 270" in procs.bridge_cmd
    assert f"--cd {wsl_path(REPO_ROOT)} " in procs.bridge_cmd
    assert f"--log-csv {wsl_path(tmp_path / 'run')}/link.csv" in procs.bridge_cmd
    assert procs.link_csv == tmp_path / "run" / LINK_CSV          # where the link-health gate reads it
    assert procs.relay_cmd.endswith("--bridge 127.0.0.1:18570") and "{" not in procs.relay_cmd


def test_refuses_when_the_port_already_listens_before_the_bridge_starts(tmp_path):
    world = World(already_listening=True)
    with pytest.raises(Refused, match="already accepts connections"):
        _procs(world, tmp_path).start()
    assert world.log == []


def test_a_bridge_that_exits_before_listening_stops_the_start(tmp_path):
    world = World(listen_after_polls=10**6)
    procs = _procs(world, tmp_path)
    original = world.listening

    def listening(port: int) -> bool:
        if world.polls == 3:
            world.procs["bridge"].returncode = 2
        return original(port)

    procs._listening = listening
    with pytest.raises(RuntimeError, match="exited with code 2 before listening"):
        procs.start()
    assert "relay started" not in world.log


def test_a_bridge_that_never_listens_times_out(tmp_path, monkeypatch):
    monkeypatch.setattr(hitl_procs, "LISTEN_TIMEOUT_S", 0.0)
    world = World(listen_after_polls=10**6)
    with pytest.raises(RuntimeError, match="did not listen on port 18570"):
        _procs(world, tmp_path).start()
    assert "relay started" not in world.log


def test_check_fails_when_either_process_ends(tmp_path):
    for name in ("bridge", "relay"):
        world = World()
        procs = _procs(world, tmp_path / name)
        (tmp_path / name).mkdir()
        procs.start()
        world.procs[name].returncode = 3
        with pytest.raises(RuntimeError, match=f"the {name} exited with code 3"):
            procs.check()


def test_a_bridge_that_does_not_end_is_reported_not_killed(tmp_path, monkeypatch):
    monkeypatch.setattr(hitl_procs, "BRIDGE_EXIT_TIMEOUT_S", 0.0)
    world = World(exits_on_eof=False)
    procs = _procs(world, tmp_path)
    procs.start()
    with pytest.raises(RuntimeError, match=r"did not exit within 0 s .* stop it by its pid"):
        procs.stop()
    assert "bridge killed" not in world.log and "bridge terminated" not in world.log
    assert world.log[-2:] == ["relay terminated", "bridge stdin closed"]


def test_stop_before_the_relay_started_only_ends_the_bridge(tmp_path, monkeypatch):
    monkeypatch.setattr(hitl_procs, "LISTEN_TIMEOUT_S", 0.0)
    world = World(listen_after_polls=10**6)
    procs = _procs(world, tmp_path)
    with pytest.raises(RuntimeError):
        procs.start()
    assert procs.stop() == {"bridge_exit": 0}


def test_wsl_path_maps_drive_paths_and_keeps_posix_paths():
    assert wsl_path(Path(r"C:\Users\someone\src\navpy")) == "/mnt/c/Users/someone/src/navpy"
    assert wsl_path(Path("D:/runs/a b")) == "/mnt/d/runs/a b"
    assert wsl_path("/home/someone/tune") == "/home/someone/tune"
    with pytest.raises(ValueError, match="drive letter"):
        wsl_path(r"\\wsl.localhost\Ubuntu-24.04\home\someone")


def test_templates_must_use_the_port_and_the_origin_and_known_placeholders():
    assert template_errors(BRIDGE, RELAY) == []
    errors = template_errors("bridge --origin-lat {lat}", "relay --bridge 127.0.0.1:{prot}")
    assert any("--bridge must use {port}" in e for e in errors)
    assert any("unknown placeholder ['prot']" in e for e in errors)
    assert any("SIM_OPOS_" in e for e in errors)
    with pytest.raises(ValueError, match="unknown placeholder"):
        fill("{nope}", {"port": "1"})


def test_the_bridge_takes_the_origin_only_from_its_four_options():
    # the bridge has --origin-lat/--origin-lon/--origin-alt/--heading and no --origin option
    # (hitl_bridge/bridge.py build_parser): an {origin} template would exit with code 2
    errors = template_errors("bridge --device tcpin:{port} --origin {origin} --log-csv {link_csv_wsl}", RELAY)
    assert any("unknown placeholder ['origin']" in e for e in errors)
    assert any("{lat} {lon} {alt} {hdg}" in e for e in errors)


def test_the_bridge_must_write_its_link_log_where_the_gate_reads_it():
    without_log = BRIDGE.replace(" --log-csv {link_csv_wsl}", "")
    (error,) = template_errors(without_log, RELAY)
    assert "{link_csv_wsl}" in error
    assert template_errors(without_log + " --log-csv {link_csv}", RELAY) == []


def _documented(option: str) -> str:
    """The --bridge or --relay template of the usage in tools/tuning/hitl.py, on one line."""
    return " ".join(re.search(rf'{option} "([^"]+)"', hitl.__doc__).group(1).split())


def test_the_documented_templates_parse_with_the_bridges_and_relays_own_options(tmp_path):
    bridge, relay = _documented("--bridge"), _documented("--relay")
    assert template_errors(bridge, relay) == []
    fields = command_fields(ORIGIN, hitl.DEFAULT_BRIDGE_PORT, tmp_path)
    words = fill(bridge, fields).split()
    args = bridge_parser().parse_args(words[words.index("tools.tuning.hitl_bridge") + 1:])
    assert args.device == f"tcpin:{hitl.DEFAULT_BRIDGE_PORT}" and args.exit_on_stdin_eof
    assert (args.origin_lat, args.origin_lon, args.origin_alt, args.heading) == (40.2, 44.4, 1300, 270)
    assert args.log_csv.as_posix() == f"{wsl_path(tmp_path)}/{LINK_CSV}" and args.model_dir.name == "Peron15"
    words = fill(relay, fields).split()
    relay_args = hitl_relay.build_parser().parse_args(words[words.index("tools.tuning.hitl_bridge.relay") + 1:])
    assert relay_args.bridge == f"127.0.0.1:{hitl.DEFAULT_BRIDGE_PORT}" == hitl_relay.DEFAULT_BRIDGE


DIRECT = ("{python} -m tools.tuning.hitl_bridge --device COM11 --plant package --origin-lat {lat} --origin-lon {lon} "
          "--origin-alt {alt} --heading {hdg} --exit-on-stdin-eof --log-csv {link_csv}")


def test_a_direct_bridge_counts_as_up_on_its_first_line_and_no_relay_runs(tmp_path):
    log: list[str] = []
    lines = ["jsbsim banner\n", "bridge: COM11 <-> Peron15_hitl3 (package), log link.csv\n"]

    def popen(args, **kwargs):
        log.append("bridge started")
        proc = FakeProc(log, "bridge")
        proc.out = kwargs["stdout"]
        return proc

    polls = []

    def sleep_writes_next_line(_s):
        polls.append(1)
        procs.bridge.out.write(lines.pop(0))

    procs = HitlProcesses(DIRECT, None, 18570, tmp_path, ORIGIN, popen=popen,
                          listening=lambda port: pytest.fail("a direct bridge has no TCP port"))
    hitl_procs.time.sleep, real_sleep = sleep_writes_next_line, hitl_procs.time.sleep
    try:
        record = procs.start()
    finally:
        hitl_procs.time.sleep = real_sleep
    assert len(polls) == 2 and record["relay"] is None and "--device COM11" in record["bridge"]
    procs.check()
    assert procs.stop() == {"bridge_exit": 0} and log == ["bridge started", "bridge stdin closed"]
    assert template_errors(DIRECT, None) == []


def test_a_direct_bridge_that_exits_before_its_first_line_stops_the_start(tmp_path):
    def popen(args, **kwargs):
        proc = FakeProc([], "bridge")
        proc.returncode = 2
        return proc
    procs = HitlProcesses(DIRECT, None, 18570, tmp_path, ORIGIN, popen=popen, listening=lambda port: False)
    with pytest.raises(RuntimeError, match="exited with code 2 before printing its first line"):
        procs.start()
