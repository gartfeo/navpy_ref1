"""Step 13's approach runner (tools/tuning/approach*.py): each role once, ports and owners checked before anything
starts, ready lines awaited, every owned process stopped on every exit, orphans named. Fake processes only."""
from __future__ import annotations

import json
import os
import socket
from pathlib import Path

import pytest

from tools.tuning import approach, approach_chain as chain
from tools.tuning.approach_lane import FIRST_LANE_SEQ, LANE_REPEATS, build_lane, lane_items
from tools.tuning.approach_procs import STOP_EOF, STOP_SIGINT, ProcessGroup, ProcSpec, RunnerError
from tools.tuning.dock_pass import M, Point


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
    assert producers["unreal"]["how"] == "kill"
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
    group.stop_all()
    assert group.producers()["navpy"] == {**group.producers()["navpy"], "ended_after_stop": False,
                                          "error": "OSError: gone"}


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


def test_a_failed_start_still_stops_the_board_and_every_process(tmp_path, monkeypatch):
    log: list = []

    class FakeBoard:
        def __init__(self, *args, **kwargs) -> None:
            self.workdir, self.record = Path(args[1]), {"hitl": {"bridge_exit": 0}}

        def start(self):
            log.append("board start")
            raise RuntimeError("bridge exited")

        def stop(self):
            log.append("board stop")

        def report(self):
            return {"link_health": {"ok": True}}

    def spawn(spec, output):
        log.append(f"spawn {spec.role}")
        output.write(READY_LINES[spec.role] + "\n")
        return FakeHandle(log, spec.role)

    monkeypatch.setattr(approach, "_record", lambda args, run_id, repo: {"run": {"run_id": run_id}})
    monkeypatch.setattr(approach.approach_records, "write_record",
                        lambda record, run_dir: _written(run_dir / "approach_record.json"))
    monkeypatch.setattr(approach, "_addresses", lambda: ("172.0.0.2", "172.0.0.1"))
    monkeypatch.setattr(approach, "spawn_posix", spawn)
    monkeypatch.setattr(approach.chain, "port_free", lambda protocol, port: True)
    monkeypatch.setattr(approach.chain, "unreal_running", lambda: None)
    monkeypatch.setattr(approach.chain, "publisher_profile", lambda paths: tmp_path / "profile.json")
    monkeypatch.setattr(approach.chain, "win_path", lambda path: "C:/x")
    monkeypatch.setattr(approach, "Board", FakeBoard)
    monkeypatch.setattr(approach, "board_param_files", lambda *args: [])
    args = approach_args(tmp_path)
    assert approach.approach(args) == 1
    run_dir = next((tmp_path / "runs").iterdir())
    run = json.loads((run_dir / "approach_run.json").read_text(encoding="utf-8"))
    assert run["error"] == "RuntimeError: bridge exited"
    assert log[:3] == ["spawn backing", "spawn router", "board start"]
    assert log[3:] == ["board stop", "router stop", "backing stop"] and run["orphans"] == []
    assert (run_dir / "run_manifest.json").exists()


READY_LINES = {"backing": "[backing] SITL instance 240 (hil)", "router": "[router] tcp:127.0.0.1:8160 <-> 3 clients"}


def _written(path: Path) -> Path:
    path.write_text("{}", encoding="utf-8")
    return path


def approach_args(tmp_path):
    parser = approach.argparse.ArgumentParser()
    approach.add_parser(parser.add_subparsers())
    return parser.parse_args(["approach", "--mode", "pc-sitl", "--run-root", str(tmp_path / "runs"),
                              "--model-dir", str(tmp_path), "--model-input", "mi.json", "--media-runtime", "mr.json",
                              "--publisher-profile", "p.json", "--aas-hitl", "/aas", "--uproject", "/mnt/c/u/A.uproject"])


def test_unreal_gets_windows_paths_for_drive_and_wsl_disk_files():
    assert chain.win_path(Path("/mnt/c/Users/a/unreal.log")) == r"C:\Users\a\unreal.log"
    assert chain.win_path(Path("/home/gart/repo/x.json"), "Ubuntu-24.04") == \
        r"\\wsl.localhost\Ubuntu-24.04\home\gart\repo\x.json"
    with pytest.raises(ValueError):
        chain.win_path(Path("relative/x.json"), "Ubuntu-24.04")


def test_unreal_gets_its_project_as_a_windows_path(tmp_path):
    paths = chain.ChainPaths(Path("/mnt/c/runs/r1"), Path("/repo"), "py", Path("/aas"), tmp_path / "p.json",
                             Path("/mnt/c/repos/Aas/Aas.uproject"), "172.0.0.2", "172.0.0.1")
    argv = chain.unreal(paths, Path("/mnt/c/lens.json")).argv
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
