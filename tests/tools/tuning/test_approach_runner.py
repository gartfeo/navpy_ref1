"""Step 13's approach runner (tools/tuning/approach*.py): each role once, ports and owners checked before anything
starts, ready lines awaited, every owned process stopped on every exit, orphans named. Fake processes only."""
from __future__ import annotations

import json
import os
import socket
from dataclasses import replace
from pathlib import Path

import pytest

from tools.hitl.media_fault_spec import NO_FAULT
from tools.tuning import approach, approach_chain as chain, approach_handles as handles
from tools.tuning.approach_lane import FIRST_LANE_SEQ, LANE_REPEATS, build_lane, lane_items
from tools.tuning.approach_procs import STOP_EOF, STOP_SIGINT, ProcessGroup, ProcSpec, RunnerError
from tools.tuning.dock_pass import M, Point
from tools.tuning.hitl_params import ParamFile


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeHandle:
    """ready: the line it writes on start; exits_on: which step ends it (stop, terminate, kill or never)."""

    def __init__(self, log: list, role: str, exits_on: str = "stop", alive_after: bool = False) -> None:
        self.log, self.role, self.exits_on, self.alive_after = log, role, exits_on, alive_after
        self.pid = 100 + len(log)
        self.code = None

    def running(self) -> bool:
        return self.code is None

    def exit_code(self):
        return self.code

    def _step(self, step: str) -> None:
        self.log.append(f"{self.role} {step}")
        if self.exits_on == step:
            self.code = 0 if step == "stop" else -15

    def request_stop(self) -> None:
        self._step("stop")

    def terminate(self) -> None:
        self._step("terminate")

    def kill(self) -> None:
        self._step("kill")

    def group_alive(self) -> bool:
        return self.running() or self.alive_after

    def confirm_ready(self) -> None:
        """Nothing more to learn at ready."""


def _spec(role: str, ready: str | None = "ready", ports=(), refuse=None, timeout_s: float = 5.0) -> ProcSpec:
    return ProcSpec(role, (role,), ready, timeout_s, STOP_SIGINT, stop_wait_s=1.0, ports=ports, refuse=refuse)


def _group(tmp_path, log, busy=(), behaviour=None, writes_ready=True):
    clock = FakeClock()
    handles = {}

    def spawn(spec, output):
        log.append(f"spawn {spec.role}")
        if writes_ready:
            output.write("starting\nready\n")
        handle = handles[spec.role] = FakeHandle(log, spec.role, **(behaviour or {}).get(spec.role, {}))
        return handle

    group = ProcessGroup(tmp_path, {"posix": spawn}, lambda protocol, port: (protocol, port) not in busy,
                         clock=clock, sleep=clock.sleep)
    return group, handles, clock


def test_roles_start_in_order_and_stop_in_reverse(tmp_path):
    log: list = []
    group, _, _ = _group(tmp_path, log)
    for role in ("backing", "router", "navpy"):
        group.start(_spec(role))
    group.stop_all()
    assert [line for line in log if line.startswith("spawn")] == ["spawn backing", "spawn router", "spawn navpy"]
    assert [line for line in log if line.endswith("stop")] == ["navpy stop", "router stop", "backing stop"]
    assert all(item["ended_after_stop"] and item["how"] == "stop" for item in group.producers().values())
    assert group.orphans() == []


def test_a_second_owner_is_refused_before_anything_starts(tmp_path):
    log: list = []
    group, _, _ = _group(tmp_path, log, busy={("tcp", 18554)})
    specs = [_spec("router"), _spec("router"), _spec("publisher", ports=(("tcp", 18554),)),
             _spec("unreal", refuse=lambda: "UnrealEditor.exe pid 9 already runs")]
    problems = group.refusals(specs)
    assert problems == ["router: a second router in one run", "publisher: tcp port 18554 is in use (another owner)",
                        "unreal: UnrealEditor.exe pid 9 already runs"]
    group.start(specs[0])
    with pytest.raises(RunnerError, match="a second router"):
        group.start(specs[1])
    with pytest.raises(RunnerError, match="a second router"):
        group.adopt("router", FakeHandle(log, "router"))
    assert log == ["spawn router"]


def test_an_exit_before_ready_fails_the_start_and_stop_all_still_stops_the_rest(tmp_path):
    log: list = []
    group, handles, _ = _group(tmp_path, log, writes_ready=False)
    group.adopt("backing", FakeHandle(log, "backing"))

    def dies(spec, output):
        handle = FakeHandle(log, spec.role)
        handle.code = 3
        return handle

    group._spawners["posix"] = dies
    with pytest.raises(RunnerError, match="router exited with code 3 before it was ready"):
        group.start(_spec("router"))
    group.stop_all()
    producers = group.producers()
    assert producers["router"]["how"] == "already exited" and not producers["router"]["ended_after_stop"]
    assert producers["backing"]["ended_after_stop"] and "backing stop" in log


def test_no_ready_line_in_time_fails_the_start(tmp_path):
    log: list = []
    group, _, clock = _group(tmp_path, log, writes_ready=False)
    with pytest.raises(RunnerError, match="publisher not ready within 5 s"):
        group.start(_spec("publisher", timeout_s=5.0))
    assert clock.now > 5.0
    group.stop_all()
    assert "publisher stop" in log


def test_a_ready_file_other_than_the_output_is_awaited(tmp_path):
    log: list = []
    group, _, _ = _group(tmp_path, log, writes_ready=False)
    status = tmp_path / "publisher.json"
    status.write_text(json.dumps({"ready_monotonic_ns": 5}), encoding="utf-8")
    spec = ProcSpec("publisher", ("p",), r'"ready_monotonic_ns"', 5.0, ready_file=status)
    group.start(spec)


def _unreal_log(tmp_path, monkeypatch, *lines):
    monkeypatch.setattr(chain, "win_path", str)
    log = tmp_path / "unreal.log"
    log.write_text("".join(f"LogTemp: {line}\n" for line in lines), encoding="utf-8")
    paths = chain.ChainPaths(tmp_path, Path("/repo"), "py", Path("/aas"), tmp_path / "p.json",
                             Path("/mnt/c/repos/Aas/Aas.uproject"), "172.0.0.2", "172.0.0.1")
    spec = chain.unreal(paths, Path("/mnt/c/lens.json"), Path("/mnt/c/s/unreal.json"), "e" * 64)
    return replace(spec, ready_timeout_s=5.0, kind="posix", refuse=None)


@pytest.mark.parametrize("lines, error", [
    (("NavPyCameraMount ok",), "unreal not ready within 5 s"),                    # terrain sample not back yet
    (("NavPyCameraMount ok", "NavPyScenario applied scene_hash=" + "f" * 64), "not ready within"),
    (("NavPyScenario applied scene_hash=" + "e" * 64,), "not ready within"),     # no camera yet
    (("NavPyCameraMount ok", "Error: NavPyScenario refused: the dock board was not placed"),
     "unreal refused to start: NavPyScenario refused: the dock board was not placed"),
    (("NavPyCameraMount ok", "NavPyScenario applied scene_hash=" + "e" * 64), None),
])
def test_unreal_is_ready_only_once_it_built_this_runs_scene(tmp_path, monkeypatch, lines, error):
    log: list = []
    group, _, _ = _group(tmp_path, log, writes_ready=False)
    spec = _unreal_log(tmp_path, monkeypatch, *lines)
    if error is None:
        group.start(spec)
    else:
        with pytest.raises(RunnerError, match=error):
            group.start(spec)
    group.stop_all()


def test_stop_escalates_and_names_orphans(tmp_path):
    log: list = []
    behaviour = {"emulator": {"exits_on": "terminate"}, "unreal": {"exits_on": "kill", "alive_after": True},
                 "navpy": {"exits_on": "never"}}
    group, _, _ = _group(tmp_path, log, behaviour=behaviour)
    for role in ("emulator", "unreal", "navpy"):
        group.start(_spec(role))
    group.stop_all()
    producers = group.producers()
    assert producers["emulator"]["how"] == "terminate" and not producers["emulator"]["ended_after_stop"]
    assert producers["unreal"]["how"] == "still running after kill"      # its leader died; a member is left
    assert producers["navpy"]["how"] == "still running after kill"
    assert group.orphans() == ["unreal", "navpy"]


def test_stop_all_never_raises_and_an_exited_role_fails_the_check(tmp_path):
    log: list = []
    group, handles, _ = _group(tmp_path, log)
    group.start(_spec("router"))
    group.start(_spec("navpy"))
    handles["router"].code = 1
    with pytest.raises(RunnerError, match="router exited with code 1 during the run"):
        group.check()

    def broken():
        raise OSError("gone")

    handles["navpy"].request_stop = broken
    handles["navpy"].exits_on = "terminate"
    group.stop_all()
    navpy = group.producers()["navpy"]
    assert navpy["how"] == "terminate" and navpy["errors"] == ["stop: OSError: gone"]
    assert not navpy["ended_after_stop"] and "navpy terminate" in log and group.orphans() == []


def test_the_adopted_bridge_is_marked_stopped_by_its_owner(tmp_path):
    log: list = []
    group, _, _ = _group(tmp_path, log)
    bridge = FakeHandle(log, "bridge")
    group.adopt("bridge", bridge, STOP_EOF)
    bridge.code = 0                     # Board.stop closed its input; it exited
    group.mark_stopped("bridge", 0)
    group.stop_all()
    assert group.producers()["bridge"]["ended_after_stop"] and "bridge stop" not in log


def test_the_lane_is_fixed_from_home_and_repeats():
    lane = build_lane(Point(40.0, 44.0))
    items = lane_items(lane)
    assert [item[0] for item in items] == list(range(len(items)))
    start, end, jump = items[FIRST_LANE_SEQ], items[FIRST_LANE_SEQ + 1], items[FIRST_LANE_SEQ + 2]
    assert start[2] == end[2] == M.MAV_CMD_NAV_WAYPOINT and start[8] == items[0][8]   # due north of home
    assert end[7] > start[7] > items[0][7]
    assert jump[2] == M.MAV_CMD_DO_JUMP and jump[3:5] == (FIRST_LANE_SEQ, LANE_REPEATS)
    assert items[-1][2] == M.MAV_CMD_NAV_LOITER_UNLIM


def test_the_navpy_spec_carries_the_timing_inputs_and_the_link_identity(tmp_path):
    paths = chain.ChainPaths(tmp_path, Path("/repo"), "/venv/python", Path("/aas"), tmp_path / "p.json", Path("/mnt/c/u/A.uproject"),
                             "172.0.0.2", "172.0.0.1")
    spec = chain.navpy(paths, tmp_path / "approach_record.json", FIRST_LANE_SEQ)
    argv = list(spec.argv)
    assert argv[argv.index("--integration-timing") + 1] == str(tmp_path / "approach_record.json")
    assert argv[argv.index("-mwp") + 1] == str(FIRST_LANE_SEQ)
    assert chain.navpy_link_id() == f"{argv[argv.index('-c') + 1]}#{argv[argv.index('-ss') + 1]}"
    backing = list(chain.backing(paths, "40,44,1300,0", "/fw/sim-hil/arduplane").argv)
    assert backing[backing.index("--binary") + 1] == "/fw/sim-hil/arduplane" and "hil" in backing


@pytest.mark.parametrize("board_stop_fails", [False, True])
def test_a_failed_start_still_stops_the_board_and_every_process(tmp_path, monkeypatch, board_stop_fails):
    log: list = []

    class FakeBoard:
        def __init__(self, *args, **kwargs) -> None:
            self.workdir, self.record = Path(args[1]), {"hitl": {"bridge_exit": 0}}

        def start(self):
            log.append("board start")
            raise RuntimeError("bridge exited")

        def stop(self):
            log.append("board stop")
            if board_stop_fails:
                raise OSError("link close failed")

        def report(self):
            return {"link_health": {"ok": True}}

    def spawn(spec, output):
        log.append(f"spawn {spec.role}")
        output.write(READY_LINES[spec.role] + "\n")
        return FakeHandle(log, spec.role)

    monkeypatch.setattr(approach, "_record", lambda args, run_id, repo, scenario: {"run": {"run_id": run_id},
                                                                         "commissioning": NO_FAULT.record()})
    monkeypatch.setattr(approach.scenario_run, "service", fake_service)
    monkeypatch.setattr(approach.approach_records, "write_record",
                        lambda record, run_dir: _written(run_dir / "approach_record.json"))
    monkeypatch.setattr(approach, "_addresses", lambda: ("172.0.0.2", "172.0.0.1"))
    monkeypatch.setattr(approach, "spawn_posix", spawn)
    monkeypatch.setattr(approach.chain, "port_free", lambda protocol, port: True)
    monkeypatch.setattr(approach.chain, "unreal_running", lambda: None)
    monkeypatch.setattr(approach.owners, "census", lambda *a, **k: {"ok": True, "roles": {}, "problems": []})
    monkeypatch.setattr(approach.handles, "windows_udp_in_use", lambda ports: None)
    monkeypatch.setattr(approach.chain, "publisher_profile", lambda paths: tmp_path / "profile.json")
    monkeypatch.setattr(approach.chain, "win_path", lambda path: "C:/x")
    monkeypatch.setattr(approach, "Board", FakeBoard)
    monkeypatch.setattr(approach, "board_param_files", lambda *args: [])
    monkeypatch.setattr(approach, "_orbit_check", lambda args, scenario: {"feasible": True})
    args = approach_args(tmp_path)
    assert approach.approach(args) == 1
    run_dir = next((tmp_path / "runs").iterdir())
    run = json.loads((run_dir / "approach_run.json").read_text(encoding="utf-8"))
    assert run["error"] == "RuntimeError: bridge exited"
    assert log[:3] == ["spawn backing", "spawn router", "board start"]
    assert log[3:] == ["board stop", "router stop", "backing stop"] and run["orphans"] == []
    assert (run_dir / "run_manifest.json").exists()
    assert run.get("board_stop_error") == ("OSError: link close failed" if board_stop_fails else None)


READY_LINES = {"backing": "[backing] SITL instance 240 (hil)", "router": "[router] tcp:127.0.0.1:8160 <-> 3 clients"}


def _written(path: Path) -> Path:
    path.write_text("{}", encoding="utf-8")
    return path


def approach_args(tmp_path):
    parser = approach.argparse.ArgumentParser()
    approach.add_parser(parser.add_subparsers())
    return parser.parse_args(["approach", "--mode", "pc-sitl", "--run-root", str(tmp_path / "runs"),
                              "--model-dir", str(tmp_path), "--model-input", "mi.json", "--media-runtime", "mr.json",
                              "--publisher-profile", "p.json", "--aas-hitl", "/aas",
                              "--uproject", "/mnt/c/u/A.uproject", "--windows-python", "/mnt/c/py/python.exe",
                              "--scenario-series", "series.json", "--seed-index", "0", "--board-height-m", "10"])


PUBLIC = {"schema_version": 1, "series_id": "s1", "series_sha256": "d" * 64, "seed_index": 0,
          "scene_hash": "e" * 64, "consumers": ["unreal", "scoring"],
          "lane": {"bearing_deg": 0.0, "start_m": 400.0, "length_m": 2000.0, "height_m": 80.0, "repeats": 30},
          "limits": {"observe_s": 660.0, "min_height_m": 30.0}}


def fake_service(command, *options):
    """The scenario service's printed line: prepare writes the public half where asked; verify a verdict."""
    if command == "prepare":
        if "--out" in options:
            out = Path(options[options.index("--out") + 1])
            out.mkdir(parents=True)
            (out / "scenario.json").write_text(json.dumps(PUBLIC), encoding="utf-8")
        return dict(PUBLIC)
    return {"reproduced": False, "problems": ["no Unreal log"]}


def test_unreal_gets_windows_paths_for_drive_and_wsl_disk_files():
    assert chain.win_path(Path("/mnt/c/Users/a/unreal.log")) == r"C:\Users\a\unreal.log"
    assert chain.win_path(Path("/home/gart/repo/x.json"), "Ubuntu-24.04") == \
        r"\\wsl.localhost\Ubuntu-24.04\home\gart\repo\x.json"
    with pytest.raises(ValueError):
        chain.win_path(Path("relative/x.json"), "Ubuntu-24.04")


def test_unreal_gets_its_project_as_a_windows_path(tmp_path):
    paths = chain.ChainPaths(Path("/mnt/c/runs/r1"), Path("/repo"), "py", Path("/aas"), tmp_path / "p.json",
                             Path("/mnt/c/repos/Aas/Aas.uproject"), "172.0.0.2", "172.0.0.1")
    argv = chain.unreal(paths, Path("/mnt/c/lens.json"), Path("/mnt/c/run/scenario/private/unreal.json"),
                        "e" * 64).argv
    assert argv[1] == r"C:\repos\Aas\Aas.uproject" and r"-abslog=C:\runs\r1\unreal.log" in argv


def test_a_listener_owns_its_port():
    with socket.socket() as listener:
        listener.bind(("0.0.0.0", 0))
        listener.listen()
        assert not chain.port_free("tcp", listener.getsockname()[1])


@pytest.mark.skipif(os.name == "nt", reason="the runner runs in WSL; Linux TIME_WAIT bind rules")
def test_the_time_wait_a_stopped_run_leaves_is_not_an_owner():
    with socket.socket() as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)     # as MediaMTX's Go listener does
        listener.bind(("0.0.0.0", 0))
        listener.listen()
        port = listener.getsockname()[1]
        client = socket.create_connection(("127.0.0.1", port))
        served, _ = listener.accept()
        served.close()                 # the server closes first, so its port keeps the TIME_WAIT
        client.recv(1)
        client.close()
    assert chain.port_free("tcp", port)


def test_the_board_gets_navpys_parameter_script_and_scripting(tmp_path):
    script = tmp_path / "repo" / approach.AAS_SCRIPT
    script.parent.mkdir(parents=True)
    script.write_bytes(b"param:add_table(88)\r\nreturn\r\n")
    approach._board_scripts(tmp_path / "repo", tmp_path / "run")
    # SITL loads Lua from its workdir (the backing's), with LF ends like the aircraft deploy step.
    assert (tmp_path / "run" / "backing" / "scripts" / "aas_params.lua").read_bytes() == b"param:add_table(88)\nreturn\n"
    assert (tmp_path / "run" / approach.BOARD_SCRIPTING).read_text(encoding="utf-8") == "SCR_ENABLE 1\n"


def _windows_paths(tmp_path, windows_python="/mnt/c/py/python.exe"):
    return chain.ChainPaths(Path("/mnt/c/runs/r1"), Path("/mnt/c/repo"), "/venv/python", Path("/aas"),
                            tmp_path / "p.json", Path("/mnt/c/u/A.uproject"), "172.0.0.2", "172.0.0.1", windows_python)


def test_the_emulator_runs_on_windows_beside_unreal_and_stops_on_end_of_input(tmp_path):
    spec = chain.emulator(_windows_paths(tmp_path))
    argv = list(spec.argv)
    assert argv[:4] == ["/mnt/c/py/python.exe", "-s", "-m", "tools.hitl.zr10_emulator"]
    assert argv[argv.index("--vehicle-state") + 1] == f"172.0.0.1:{chain.VEHICLE_STATE_PORT}"
    assert argv[argv.index("--siyi") + 1] == f"172.0.0.1:{chain.SIYI_PORT}"
    assert argv[argv.index("--unreal") + 1] == f"127.0.0.1:{chain.GIMBAL_PORT}"
    assert argv[argv.index("--log") + 1] == r"C:\runs\r1\emulator.jsonl" and "--exit-on-stdin-eof" in argv
    assert (spec.stop, spec.kind, spec.ports) == (STOP_EOF, "interop", ())
    assert spec.env["PYTHONPATH"] == r"C:\repo\src;C:\repo"
    assert {"PYTHONPATH", "PYTHONUNBUFFERED"} <= set(spec.env["WSLENV"].split(":"))
    with pytest.raises(RunnerError, match="--windows-python"):
        chain.emulator(_windows_paths(tmp_path, ""))


def test_the_bridge_copy_and_navpys_siyi_link_go_to_the_windows_emulator(tmp_path):
    assert chain.bridge_template(Path("/m"), 0.1, "172.0.0.1").endswith(
        f"--vehicle-state-copy 172.0.0.1:{chain.VEHICLE_STATE_PORT}")
    argv = list(chain.navpy(_windows_paths(tmp_path), tmp_path / "r.json", FIRST_LANE_SEQ).argv)
    assert argv[argv.index("--siyi-ip") + 1] == "172.0.0.1"


NETSTAT = """
Active Connections

  Proto  Local Address          Foreign Address        State           PID
  UDP    0.0.0.0:53             *:*                                    4060
  UDP    172.23.112.1:37260     *:*                                    29860
  UDP    172.23.112.1:17113     *:*                                    29860
  UDP    0.0.0.0:137260         *:*                                    5
  UDP    [::]:17113             *:*                                    77
"""


def test_the_windows_owners_of_the_emulators_ports_are_found():
    assert handles.windows_udp_owners(chain.EMULATOR_PORTS, NETSTAT) == {29860, 77}
    assert handles.windows_udp_owners((9999,), NETSTAT) == set()


class _Proxy:
    pid = 1


READY = ("ZR10 emulator: SIYI ('172.0.0.1', 37260), vehicle_state x, Unreal y, non-ideal none, pid 29860, "
         "started 134062000000000000\r\n")


def _interop(tmp_path, monkeypatch, started: dict, owners: set, ready: str = READY):
    """*started*: Windows pid -> its start time now, or an exception its query raises; a matching kill clears it."""
    killed: list = []
    monkeypatch.setattr(handles.PosixHandle, "group_alive", lambda self: False)     # the interop proxy is gone
    monkeypatch.setattr(handles.PosixHandle, "kill", lambda self: killed.append("proxy"))
    monkeypatch.setattr(handles, "windows_udp_owners", lambda ports: set(owners))

    def windows_process(pid, kill_if=""):
        now = started.get(pid, handles.GONE)
        if isinstance(now, Exception):
            raise now
        if kill_if and now == kill_if:
            killed.append(pid)
            started.pop(pid)
            return handles.GONE
        return now

    monkeypatch.setattr(handles, "windows_process", windows_process)
    output = tmp_path / "emulator.out"
    output.write_text(ready, encoding="utf-8")
    return handles.InteropHandle(_Proxy(), STOP_EOF, chain.EMULATOR_PORTS, output, chain.EMULATOR_PID_LINE), killed


def test_an_interop_role_is_alive_while_its_own_windows_python_runs_and_a_kill_ends_only_that(tmp_path, monkeypatch):
    started = {29860: "134062000000000000", 77: "134061000000000000"}
    handle, killed = _interop(tmp_path, monkeypatch, started, {29860, 77})
    handle.confirm_ready()
    assert handle.windows_pid == 29860 and handle.group_alive()
    handle.kill()
    assert killed == ["proxy", 29860] and 77 in started and not handle.group_alive()


def test_a_pid_reused_by_another_process_is_neither_alive_nor_killed(tmp_path, monkeypatch):
    started = {29860: "134062000000000000"}
    handle, killed = _interop(tmp_path, monkeypatch, started, {29860})
    handle.confirm_ready()
    started[29860] = "134062000099990000"          # the emulator exited; Windows gave its pid to a new process
    assert not handle.group_alive()
    handle.kill()
    assert killed == ["proxy"] and 29860 in started


def test_a_failed_windows_query_raises_so_the_role_counts_as_alive(tmp_path, monkeypatch):
    started: dict = {29860: "134062000000000000"}
    handle, _ = _interop(tmp_path, monkeypatch, started, set())
    handle.confirm_ready()
    started[29860] = OSError("Windows process query failed (exit 1)")
    with pytest.raises(OSError):
        handle.group_alive()


def test_the_windows_kill_holds_the_handle_its_start_time_was_checked_on(monkeypatch):
    scripts: list = []

    class Done:
        returncode, stdout, stderr = 0, "gone\r\n", ""

    monkeypatch.setattr(handles.subprocess, "run", lambda argv, **kwargs: (scripts.append(argv[-1]), Done())[1])
    assert handles.windows_process(29860, "134062000000000000") == handles.GONE
    script = scripts[0]
    # StartTime and Kill each open their own handle by pid unless $p.Handle opened one first (.NET Framework)
    assert "Get-Process -Id 29860 " in script
    assert script.index("$null = $p.Handle;") < script.index("StartTime") < script.index("$p.Kill()")
    assert "-eq 134062000000000000" in script


def test_the_windows_query_answer_is_a_start_time_gone_or_a_failure():
    assert handles.parse_process(0, "134062000000000000\r\n", "") == "134062000000000000"
    assert handles.parse_process(0, "gone\r\n", "") == handles.GONE
    for returncode, stdout, stderr in ((1, "", "Get-Process : access denied"), (0, "", ""), (0, "1\r\n", "warn"),
                                       (0, "Exception calling Kill", "")):
        with pytest.raises(OSError, match="Windows process query failed"):
            handles.parse_process(returncode, stdout, stderr)


def test_a_ready_line_without_a_running_pid_fails_the_start(tmp_path, monkeypatch):
    handle, _ = _interop(tmp_path, monkeypatch, {}, set())
    with pytest.raises(RunnerError, match="names no running Windows pid with its start time"):
        handle.confirm_ready()
    handle, _ = _interop(tmp_path, monkeypatch, {29860: "134062000000000000"}, set(), ready="ZR10 emulator: SIYI a\n")
    with pytest.raises(RunnerError, match="names no running Windows pid with its start time"):
        handle.confirm_ready()
    # the emulator exited and its pid went to another process before the runner looked: never adopted
    handle, killed = _interop(tmp_path, monkeypatch, {29860: "134062000099990000"}, {29860})
    with pytest.raises(RunnerError, match="names no running Windows pid with its start time"):
        handle.confirm_ready()
    assert handle.windows_pid is None
    handle.kill()
    assert killed == ["proxy"]


def test_before_ready_a_port_owner_counts_as_alive_but_is_never_killed(tmp_path, monkeypatch):
    handle, killed = _interop(tmp_path, monkeypatch, {77: "1"}, {77}, "starting\n")
    assert handle.windows_pid is None and handle.group_alive()
    handle.kill()
    assert killed == ["proxy"] and handle.group_alive()


def test_a_failing_alive_check_counts_as_alive_and_the_stop_goes_on(tmp_path):
    log: list = []
    group, handles_, _ = _group(tmp_path, log, behaviour={"emulator": {"exits_on": "terminate"}})
    group.start(_spec("emulator"))
    emulator, calls = handles_["emulator"], []

    def flaky() -> bool:
        calls.append(1)
        if len(calls) == 1:
            raise OSError("tasklist timed out")
        return emulator.running()

    def failing_running() -> bool:
        if len(calls) < 3:
            raise OSError("tasklist timed out")
        return emulator.code is None

    emulator.group_alive, emulator.running = flaky, failing_running
    group.stop_all()
    stopped = group.producers()["emulator"]
    assert stopped["how"] == "terminate" and log[-2:] == ["emulator stop", "emulator terminate"]
    assert group.orphans() == []


class _Members(FakeHandle):
    """A leader whose process group keeps members until *members_end_on* (terminate or kill) reaches them."""

    def __init__(self, log: list, role: str, members_end_on: str) -> None:
        super().__init__(log, role, exits_on="never")
        self.members, self.members_end_on = True, members_end_on

    def _step(self, step: str) -> None:
        super()._step(step)
        self.members = self.members and step != self.members_end_on

    def group_alive(self) -> bool:
        return self.running() or self.members


def test_a_group_whose_leader_exited_first_is_still_stopped_to_its_last_member(tmp_path):
    log: list = []
    group, _, _ = _group(tmp_path, log)
    leader = _Members(log, "publisher", "terminate")
    group.adopt("publisher", leader, STOP_SIGINT)
    leader.code = 0                             # the leader exited during the run; MediaMTX is still there
    group.stop_all()
    stopped = group.producers()["publisher"]
    assert stopped["how"] == "terminate" and not stopped["ended_after_stop"] and group.orphans() == []
    assert log == ["publisher stop", "publisher terminate"]


def test_members_left_after_the_handoff_stop_are_terminated(tmp_path):
    log: list = []
    group, _, _ = _group(tmp_path, log)
    navpy = _Members(log, "navpy", "kill")
    group.adopt("navpy", navpy)
    navpy.code = 0
    approach.stop._handed_off(group, "navpy", {"stop": "requested", "returncode": 0}, "requested")
    group.stop_all()
    stopped = group.producers()["navpy"]
    assert stopped["leftover"]["how"] == "kill" and not stopped["ended_after_stop"] and group.orphans() == []
    assert log == ["navpy terminate", "navpy kill"]


@pytest.mark.parametrize("result, clean, ended", [({"stop": "requested", "returncode": 0}, "requested", True),
                                                  ({"stop": "timed out", "ended": "killed", "returncode": -9},
                                                   "requested", False),
                                                  ({"ended": "terminated", "returncode": -15}, None, False)])
def test_a_role_the_handoff_stopped_is_stopped_not_already_exited(tmp_path, result, clean, ended):
    log: list = []
    group, handles, _ = _group(tmp_path, log)
    group.start(_spec("navpy"))
    handles["navpy"].code = result["returncode"]           # the handoff stopped it
    assert approach.stop._handed_off(group, "navpy", result, clean) is result
    group.stop_all()
    stopped = group.producers()["navpy"]
    assert stopped["how"] == "stop" and stopped["ended_after_stop"] is ended and "navpy stop" not in log


def test_the_publisher_stops_by_drain_then_stop_on_its_control_socket(tmp_path, monkeypatch):
    sent, signals = [], []
    monkeypatch.setattr(handles, "control", lambda path, command: sent.append((path, command)) or {"ok": True})
    monkeypatch.setattr(handles.PosixHandle, "_signal", lambda self, number: signals.append(number))

    class Process:
        pid = 1

    status = tmp_path / "publisher.json"
    handle = handles.PublisherHandle(Process(), STOP_SIGINT, status)
    handle.request_stop()                                   # no status yet: it never got that far
    assert sent == [] and signals == [handles.signal.SIGINT]
    status.write_text(json.dumps({"control_socket": "/tmp/p/control.sock"}), encoding="utf-8")
    handle.request_stop()
    assert sent == [("/tmp/p/control.sock", "drain"), ("/tmp/p/control.sock", "stop")] and len(signals) == 1
    assert chain.publisher(_windows_paths(tmp_path), tmp_path / "p.json", tmp_path / "lens.json").kind == "publisher"


def test_the_frame_consumers_start_below_the_link_roles(tmp_path):
    """Section 8: the bridge and router above navpy and the publisher; WSL only lets a user lower a priority."""
    paths = _windows_paths(tmp_path)
    for spec in (chain.navpy(paths, tmp_path / "r.json", FIRST_LANE_SEQ),
                 chain.publisher(paths, tmp_path / "p.json", tmp_path / "lens.json")):
        assert spec.argv[:3] == ("nice", "-n", "10")
    for spec in (chain.backing(paths, "40,44,1300,0", "/fw/arduplane"), chain.router(paths)):
        assert spec.argv[0] == "/venv/python"

# --- the orbit check (step 23) ---------------------------------------------------------------------------------

def _orbit_inputs(tmp_path, monkeypatch, pitch_line="PTCH_LIM_MIN_DEG -45\n"):
    params = tmp_path / "aircraft.parm"
    params.write_text("AIRSPEED_CRUISE 24\nROLL_LIMIT_DEG 45\n" + pitch_line, encoding="utf-8")
    model = tmp_path / "mi.json"
    model.write_text(json.dumps({"envelope": {"airspeed_m_s": {"min": 14.0, "max": 35.0}}}), encoding="utf-8")
    monkeypatch.setattr(approach, "board_param_files", lambda *args: [ParamFile(params, True)])
    args = approach_args(tmp_path)
    args.model_input = str(model)
    return args


def test_the_runner_refuses_an_infeasible_orbit_before_its_run_folder_exists(tmp_path, monkeypatch):
    """The owner's 300 m radius over the series default (80 m lane, 30 m floor, a 10 m board) fails the check:
    the run is refused before the scenario folder, the record or any process."""
    args = _orbit_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(approach.scenario_run, "service", fake_service)
    monkeypatch.setattr(approach, "_record", lambda *a: pytest.fail("the record was built"))
    with pytest.raises(SystemExit, match=r"HITL orbit is infeasible \(.*reader.*\)"):
        approach.approach(args)
    assert not (tmp_path / "runs").exists()


def test_a_feasible_orbit_passes_and_is_kept_for_the_run(tmp_path, monkeypatch):
    args = _orbit_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(approach.orbit, "final_approach_orbit_setting", lambda profile: 180.0)
    scenario = {**PUBLIC, "lane": {**PUBLIC["lane"], "height_m": 100.0}, "limits": {**PUBLIC["limits"],
                                                                                    "min_height_m": 1.0}}
    result = approach._orbit_check(args, scenario)
    assert result["feasible"] is True and len(result["checks"]) == len(approach.orbit.CHECKS)


def test_unreadable_orbit_inputs_refuse_the_run(tmp_path, monkeypatch):
    args = _orbit_inputs(tmp_path, monkeypatch, pitch_line="")
    with pytest.raises(SystemExit, match="orbit check cannot run: .*PTCH_LIM_MIN_DEG"):
        approach._orbit_check(args, PUBLIC)
