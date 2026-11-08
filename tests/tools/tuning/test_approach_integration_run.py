"""Step 27's runner (hitl_final_approach.md section 9): the contact wiring (link, service, the bridge's pose copy,
navpy's --hitl-contact), command mode (navpy a router client, bounded by its drain), the command audit, the host-load
sampler and the after-run timing check. Fake processes, a real router on loopback UDP, synthetic files."""
from __future__ import annotations

import io
import json
from pathlib import Path

import pytest
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

from tests.tools.tuning.test_approach_observation import READS_AND_REPORTS, WRITES
from tests.tools.tuning.test_mav_router import NAVPY, RUNNER, GatedBoard, UdpClient, receivers_current, wait_until
from tests.tools.tuning import test_approach_runner as rig
from tools.hitl import host_load
from tools.hitl.contact_loop import READY_LINE
from tools.tuning import approach, approach_chain as chain, approach_contact as contact, approach_finish as finish
from tools.tuning import approach_owners as owners
from tools.tuning.approach_quiesce import audit
from tools.tuning.approach_verdict import problems
from tools.tuning.mav_router import build_parser
from tools.tuning.mav_router_core import MavRouter
from tools.tuning.mav_router_endpoint import Client

M = mavlink2


def _paths(tmp_path):
    return chain.ChainPaths(tmp_path, Path("/repo"), "/venv/python", Path("/aas"), tmp_path / "p.json",
                            Path("/mnt/c/u/A.uproject"), "172.0.0.2", "172.0.0.1", "/mnt/c/py/python.exe", None)


# ---- the contact wiring ---------------------------------------------------------------------------------------
def test_the_contact_service_runs_on_the_loopback_with_the_links_endpoints_and_its_own_evidence(tmp_path):
    spec = contact.service("/venv/python", Path("/repo"), tmp_path, tmp_path / "link.json", tmp_path / "scenario")
    argv = list(spec.argv)
    assert argv[:5] == ["/venv/python", "-s", "-m", "tools.hitl.contact_service", "serve"]
    assert argv[argv.index("--link") + 1] == str(tmp_path / "link.json")
    assert argv[argv.index("--scenario") + 1] == str(tmp_path / "scenario")
    assert argv[argv.index("--poses") + 1] == contact.pose_copy() == f"127.0.0.1:{contact.POSE_PORT}"
    assert argv[argv.index("--evidence") + 1] == str(tmp_path / contact.EVIDENCE_FILE)
    assert spec.ready == "^" + READY_LINE and set(spec.ports) == {("udp", contact.SERVICE_PORT),
                                                                   ("udp", contact.POSE_PORT)}
    assert len({contact.SERVICE_PORT, contact.NAVPY_PORT, contact.POSE_PORT, chain.VEHICLE_STATE_PORT,
                chain.POSE_PORT, chain.GIMBAL_PORT, chain.SIYI_PORT}) == 7


def test_navpy_gets_the_link_and_the_bridge_sends_the_service_its_own_pose_copy(tmp_path):
    argv = list(chain.navpy(_paths(tmp_path), tmp_path / "r.json", 2, extra=contact.navpy_args(tmp_path / "l.json")).argv)
    assert argv[argv.index("--hitl-contact") + 1] == str(tmp_path / "l.json")
    template = chain.bridge_template(Path("/m"), 0.03, "172.0.0.1", contact.pose_copy())
    assert template.count("--vehicle-state-copy") == 2 and f"--vehicle-state-copy {contact.pose_copy()}" in template


def test_the_link_is_written_by_the_services_own_process_in_the_private_folder(tmp_path, monkeypatch):
    monkeypatch.setenv("NAVPY_HITL_TOOLS_DIR", str(tmp_path))
    run_dir = tmp_path / "runs" / "approach-1"
    run_dir.mkdir(parents=True)
    record = run_dir / "approach_record.json"
    record.write_text(json.dumps(_record_json("approach-1")), encoding="utf-8")
    monkeypatch.setattr(contact.subprocess, "run", _real_link_run(monkeypatch))
    link = contact.link(run_dir, record)
    data = json.loads(link.read_text(encoding="utf-8"))
    assert data["run_id"] == "approach-1" and data["mode"] == "pc-sitl"
    with pytest.raises(SystemExit, match="contact link was refused"):
        contact.link(run_dir, record)                      # one link per run


def test_a_link_outside_the_private_folder_refuses_the_run(tmp_path, monkeypatch):
    monkeypatch.setenv("NAVPY_HITL_TOOLS_DIR", str(tmp_path / "private"))
    record = tmp_path / "approach_record.json"
    record.write_text(json.dumps(_record_json("approach-2")), encoding="utf-8")
    monkeypatch.setattr(contact.subprocess, "run", _real_link_run(monkeypatch))
    with pytest.raises(SystemExit, match="private folder"):
        contact.link(tmp_path, record)


def _record_json(run_id: str) -> dict:
    return {"run": {"run_id": run_id, "mode": "pc-sitl"}}


def _real_link_run(monkeypatch):
    """The service's ``link`` in this process (the record read is faked: these records are partial on purpose)."""
    from tools.hitl import approach_records, contact_service

    monkeypatch.setattr(approach_records, "read_record", lambda path: json.loads(Path(path).read_text("utf-8")))

    def run(argv, **kwargs):
        out = io.StringIO()
        import contextlib
        with contextlib.redirect_stdout(out):
            code = contact_service.main(argv[argv.index("link"):])
        return type("Done", (), {"returncode": code, "stdout": out.getvalue(), "stderr": ""})()
    return run


def test_the_contact_service_is_a_role_with_one_owner():
    assert owners.ROLE_OWNER[contact.ROLE] == contact.ROLE
    assert owners.program_roles(("python", "-s", "-m", "tools.hitl.contact_service", "serve")) == (contact.ROLE,)
    ports = {(rule.proto, rule.port) for rule in owners.PORT_RULES if rule.role == contact.ROLE}
    assert ports == {("udp", contact.SERVICE_PORT), ("udp", contact.POSE_PORT)}


# ---- command mode -----------------------------------------------------------------------------------------------
@pytest.mark.parametrize("command, flag", [(False, "--reader"), (True, "--client")])
def test_the_router_takes_navpy_as_a_reader_in_observation_and_a_client_in_command(tmp_path, command, flag):
    argv = list(chain.router(_paths(tmp_path), command).argv)
    navpy = f"navpy={chain.NAVPY_PORT}"
    assert navpy in [argv[i + 1] for i, word in enumerate(argv) if word == flag]
    other = "--reader" if command else "--client"
    assert navpy not in [argv[i + 1] for i, word in enumerate(argv) if word == other]
    args = build_parser().parse_args(argv[argv.index("--board"):])
    assert (("navpy", chain.NAVPY_PORT) in args.client) is command


def test_command_mode_needs_a_timing_fit_before_anything_starts(tmp_path, monkeypatch):
    args = rig.approach_args(tmp_path)
    args.command = True
    monkeypatch.setattr(approach.scenario_run, "service", lambda *a: pytest.fail("the scenario was prepared"))
    with pytest.raises(SystemExit, match="--command needs --timing-constants"):
        approach.approach(args)
    assert not (tmp_path / "runs").exists()


@pytest.fixture
def commanding(tmp_path):
    """The router with navpy as a full client (command mode) and the runner, writing its log to a file."""
    board, navpy, runner = GatedBoard(), UdpClient(NAVPY), UdpClient(RUNNER)
    log_path = tmp_path / "router_writes.csv"
    log = log_path.open("w", encoding="utf-8", buffering=1)
    router = MavRouter(board, [Client("navpy", navpy.port, False), Client("runner", runner.port, False)],
                       announce=lambda line: None, control_port=0, write_log=log).start()
    ports = {name: c.sock.getsockname() for name, c in router.clients.items()}
    navpy.peer, runner.peer = ports["navpy"], ports["runner"]
    wait_until(lambda: receivers_current(router))
    yield board, navpy, runner, router, log_path
    board.gate.set()
    router.close()
    log.close()
    navpy.sock.close()
    runner.sock.close()


def test_the_bounded_command_test_navpys_commands_reach_the_board_until_its_drain_and_none_after(commanding):
    """Step 27's one bounded command test at the board boundary: in command mode navpy's attitude, mode and mission
    writes reach the board; the quiesce's drain closes its endpoint; every later navpy write is refused, the
    runner still flies the RTL, and the command audit passes with 0 writes after the drain."""
    board, navpy, runner, router, log_path = commanding
    for msg in WRITES + READS_AND_REPORTS:
        navpy.send(msg)
    wait_until(lambda: len(board.written) == len(WRITES) + len(READS_AND_REPORTS))
    ack = router.close_endpoint("navpy")
    for msg in WRITES:
        navpy.send(msg)
    wait_until(lambda: _refused(router) == len(WRITES))         # every late write dropped, none written
    runner.send(M.MAVLink_command_long_message(1, 1, M.MAV_CMD_DO_SET_MODE, 0, 1, 11, 0, 0, 0, 0, 0))
    wait_until(lambda: len(board.written) == len(WRITES) + len(READS_AND_REPORTS) + 1)
    result = audit(log_path, {"barrier": {"ack": ack}}, [], command_mode=True)
    assert result["authority"] == "command" and result["forbidden"] == 0
    assert result["navpy_writes"] == len(WRITES) + len(READS_AND_REPORTS)
    assert result["after_drain"] == 0 and result["ok"]
    observed = audit(log_path, {"barrier": {"ack": ack}}, [])          # the same log under observation's rules
    assert observed["forbidden"] == len(WRITES) and not observed["ok"]


def _refused(router) -> int:
    stats = router.report()["stats"]
    return stats.get("refused_closed_navpy", 0) + stats.get("dropped_stale_navpy", 0)


def test_the_command_audit_still_fails_a_write_after_the_drain_or_without_one(tmp_path):
    path = tmp_path / "router_writes.csv"
    path.write_text(f"1.0,navpy,{M.MAVLINK_MSG_ID_SET_ATTITUDE_TARGET},\n2.5,navpy,0,\n", encoding="utf-8")
    ack = {"barrier": {"ack": {"barrier_mono_s": 2.0}}}
    late = audit(path, ack, [], command_mode=True)
    assert late["forbidden"] == 0 and late["after_drain"] == 1 and not late["ok"]
    assert not audit(path, {"barrier": {"ack": None}}, [], command_mode=True)["ok"]


# ---- the runner wires it ----------------------------------------------------------------------------------------
def test_a_command_run_starts_the_sampler_first_and_the_service_before_navpy(tmp_path, monkeypatch):
    seen = _command_run(tmp_path, monkeypatch)
    roles = [spec.role for spec in seen["specs"]]
    assert roles == ["host_load", "backing", "router", "emulator", "publisher", "unreal", "contact", "navpy"]
    by_role = {spec.role: list(spec.argv) for spec in seen["specs"]}
    assert f"navpy={chain.NAVPY_PORT}" in [by_role["router"][i + 1] for i, w in enumerate(by_role["router"])
                                           if w == "--client"]
    assert by_role["navpy"][by_role["navpy"].index("--hitl-contact") + 1] == str(seen["link"])
    assert by_role["contact"][by_role["contact"].index("--link") + 1] == str(seen["link"])
    assert seen["run"]["authority"] == "command" and seen["run"]["observation"]["authority"] == "command"
    assert seen["run"]["timing"]["failures"] == ["fit refused"]
    assert seen["run"]["host_load"] == {"ok": False, "problems": ["no host load summary"]}


def _command_run(tmp_path, monkeypatch) -> dict:
    seen: dict = {}

    class Group(approach.ProcessGroup):
        def refusals(self, specs):
            seen["specs"] = specs
            raise approach.RunnerError("stop after the specs")

    constants = tmp_path / "fit.json"
    constants.write_text("{}", encoding="utf-8")

    def link(run_dir, record_path):
        seen["link"] = rig._written(run_dir / contact.LINK_FILE)
        return seen["link"]

    monkeypatch.setattr(approach, "ProcessGroup", Group)
    monkeypatch.setattr(approach, "_record", lambda *a: {"run": {}, "commissioning": rig.NO_FAULT.record()})
    monkeypatch.setattr(approach.scenario_run, "service", rig.fake_service)
    monkeypatch.setattr(approach.approach_records, "write_record",
                        lambda record, run_dir: rig._written(run_dir / "approach_record.json"))
    monkeypatch.setattr(approach.contact, "link", link)
    monkeypatch.setattr(approach, "_addresses", lambda: ("172.0.0.2", "172.0.0.1"))
    monkeypatch.setattr(approach, "Board", _Board)
    monkeypatch.setattr(approach, "board_param_files", lambda *a: [])
    monkeypatch.setattr(approach, "orbit_check", lambda *a: {"feasible": True})
    monkeypatch.setattr(approach.chain, "publisher_profile", lambda paths: tmp_path / "profile.json")
    monkeypatch.setattr(approach.chain, "win_path", lambda path: "C:/x")
    monkeypatch.setattr(approach.finish, "timing", lambda *a: {"passed": False, "failures": ["fit refused"]})
    args = rig.approach_args(tmp_path)
    args.command, args.timing_constants = True, str(constants)
    assert approach.approach(args) == 1
    run_dir = next((tmp_path / "runs").iterdir())
    seen["run"] = json.loads((run_dir / "approach_run.json").read_text(encoding="utf-8"))
    return seen


class _Board:
    def __init__(self, *args, **kwargs) -> None:
        self.workdir, self.record = Path(args[1]), {"hitl": {"bridge_exit": 0}}

    def start(self):
        raise AssertionError("not reached")

    def stop(self):
        pass

    def report(self):
        return {"link_health": {"ok": True}}


# ---- after the run --------------------------------------------------------------------------------------------
def test_a_calibration_only_run_has_no_timing_validity(tmp_path):
    assert finish.timing(tmp_path, tmp_path / "r.json", tmp_path / "m.json", None, False) == {
        "passed": False, "failures": [finish.NO_CONSTANTS]}


def test_unreadable_timing_inputs_fail_and_never_count(tmp_path):
    constants = tmp_path / "fit.json"
    constants.write_text(json.dumps({"schema": "x", "passed": True}), encoding="utf-8")
    (tmp_path / "publisher").mkdir()
    (tmp_path / "publisher" / "frames-001.csv").write_text("", encoding="utf-8")
    result = finish.timing(tmp_path, tmp_path / "r.json", tmp_path / "m.json", str(constants), False)
    assert result["passed"] is False and "could not read its inputs" in result["failures"][0]


@pytest.mark.parametrize("observe_s, command, expected", [(600.0, False, True), (599.0, False, False),
                                                          (900.0, True, False)])
def test_only_a_long_observation_run_gets_the_ten_minute_coverage_rules(observe_s, command, expected):
    assert finish.observation_run({"limits": {"observe_s": observe_s}}, command) is expected


def test_the_host_load_sampler_is_a_run_role_with_its_own_files(tmp_path):
    spec = finish.host_load(_paths(tmp_path))
    argv = list(spec.argv)
    assert argv[3] == "tools.hitl.host_load" and spec.ready == "^" + host_load.READY_LINE
    assert argv[argv.index("--out-json") + 1] == str(tmp_path / finish.HOST_LOAD_JSON)


def test_the_run_verdict_lists_every_failed_section():
    assert problems({}, None)[:3] == ["not every process ended after its stop", "orphans None",
                                      "the scene was not reproduced"]
    assert "error: boom" in problems({}, "boom")


# ---- isolation: the runner reaches truth only through other processes -------------------------------------------
TRUTH_MODULES = {"tools.hitl.contact_service", "tools.hitl.contact_scene", "tools.hitl.approach_truth",
                 "tools.hitl.approach_run_checks", "tools.hitl.approach_report", "tools.hitl.scenario_scoring",
                 "tools.hitl.scenario_service", "navpy.sim"}


def _imported(path: Path) -> set[str]:
    import ast
    names: set[str] = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            names |= {alias.name for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            names |= {node.module, *(f"{node.module}.{alias.name}" for alias in node.names)}
    return names


def test_the_runner_imports_no_truth_reader_and_no_scorer():
    repo = Path(__file__).resolve().parents[3]
    runner = sorted((repo / "tools" / "tuning").glob("approach*.py")) + [repo / "tools" / "hitl" / "host_load.py"]
    hits = [f"{path.name}: {name}" for path in runner for name in _imported(path)
            if any(name == bad or name.startswith(bad + ".") for bad in TRUTH_MODULES)]
    assert hits == []


def _clean_run() -> dict:
    ok = {"ok": True, "problems": []}
    return {"producers": {"navpy": {"ended_after_stop": True}}, "orphans": [],
            "scene": {"reproduced": True, "scene_hash": "e"}, "scenario": {"scene_hash": "e"},
            "board": {"link_health": {"ok": True}}, "observation": {"ok": True}, "outcome": {"problems": []},
            "airspeed": {"problems": []}, "owners": {p: ok for p in ("preflight", "ready", "end")},
            "sources": ok, "timing": {"passed": True, "failures": []}, "host_load": {"ok": True}}


def test_a_held_out_run_fails_on_its_timing_or_host_load_and_a_calibration_run_has_no_timing_to_pass():
    """Codex round 1: the run's own exit agrees with the series report on timing and host load."""
    assert problems(_clean_run(), None) == []
    assert problems({**_clean_run(), "timing": {"passed": False, "failures": [finish.NO_CONSTANTS]}}, None) == []
    assert problems({**_clean_run(), "timing": {"passed": False, "failures": ["gimbal: over 8 ms"]}}, None) == [
        "timing: gimbal: over 8 ms"]
    assert problems({**_clean_run(), "timing": None}, None) == ["timing: no timing check"]
    assert problems({**_clean_run(), "host_load": {"ok": False}}, None) == ["host load: {'ok': False}"]


def test_the_run_records_the_parameter_files_as_flown(tmp_path):
    from tools.tuning.hitl_params import ParamFile
    present = tmp_path / "a.parm"
    present.write_text("X 1\n", encoding="utf-8")
    digests = finish.param_digests([ParamFile(present, True), ParamFile(tmp_path / "gone.parm", False)])
    assert digests[0]["sha256"] and digests[1] == {"path": str(tmp_path / "gone.parm"), "sha256": None}
