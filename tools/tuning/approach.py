"""``sitl_driver approach``: one approach run of the HITL chain (hitl_final_approach.md sections 1 and 9, steps 13, 16, 27).

Mode pc-sitl, from WSL in the repository root, with the run's private inputs:

    python -s -m tools.tuning.sitl_driver approach --mode pc-sitl --model-dir M --model-input MI.json
        --media-runtime MR.json --publisher-profile P.json --aas-hitl A --uproject U.uproject
        --params-file F.parm --scenario-series SERIES.json --seed-index N --board-height-m H
        [--timing-constants FIT.json [--command]]

The scenario service (tools/hitl/scenario_service.py, its own process: approach_scenario.py) prepares the seed's
scenario folder; the runner reads only its public half (identity, lane, limits: scenario_public.py) and hands Unreal the
path of its private file unread. It refuses an infeasible HITL orbit (approach_orbit.py, step 23), then writes the
run's record (approach_records.py) and its contact link (approach_contact.py) into a new private run folder, refuses
at once when a role's port or Unreal is taken or anything holds a role (approach_owners.py), then starts the host-load
sampler -> backing -> router -> board setup (parameters, the bridge, the boot on the bridge) -> emulator -> publisher ->
Unreal -> contact service -> navpy (``--hitl-contact``), each on its ready line (the publisher first: Unreal connects
to it), and counts each role's owners again. Takeoff, the scenario's search lane (approach_lane.py) in AUTO with
navpy's detection gated to it; navpy only observes (the router's reader: none of its writes to the aircraft reaches the
board), or with ``--command`` (step 27; needs a timing fit) commands as a router client until its drain; it waits for navpy's terminal record or the
scenario's observe_s, then stops navpy and closes its router endpoint (navpy_handoff.py) and only then flies RTL; a
breach of the scenario's floor takes the same path (HeightFloor). A crash or an abort (Ctrl+C, SIGTERM, SIGHUP) runs the
same idempotent quiesce (approach_quiesce.py) before the board's stop, with no RTL. Every owned process is stopped on
every exit; scoring then checks that Unreal built the delivered scene (scenario_scoring.py: a verdict, no pose), and the
manifest (tools/hitl/run_manifest.py), the offline timing check (approach_finish.py) and the orphan check
follow; the series report (tools/hitl/approach_report.py) scores the runs. A second signal waits for the cleanup. Exit 0 only
when the flight, the handoff, every stop, the scene, the link health, the orphan check, the write audit (0
forbidden navpy writes in observation, 0 after the drain ack), one owner per role in each census and in the logs, and navpy's
airspeed source (pinned before takeoff, admitted, never revoked: approach_airspeed.py) all passed.
``--commission-fault`` (SITL only) runs navpy through media_fault_injector.py: never admitted.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from collections.abc import Callable
from pathlib import Path
from typing import Any

from navpy.core.kernel.config.navigation_args import NavigationAlgorithm
from navpy.core.perception.camera.integration_timing import IntegrationTimingConfig, IntegrationTimingLimits
from tools.hitl import approach_records, media_fault_spec
from tools.hitl.model_input import read_model_input
from tools.hitl.source_identity import source_identity
from tools.hitl.timing_fit import FIT_SCHEMA
from tools.tuning import approach_chain as chain, approach_handles as handles, approach_scenario as scenario_run
from tools.tuning import approach_airspeed as airspeed, approach_owners as owners, approach_quiesce as stop
from tools.tuning import approach_contact as contact, approach_finish as finish, approach_sources
from tools.tuning.approach_orbit import orbit_check
from tools.tuning.approach_lane import FIRST_LANE_SEQ, LaneSpec, build_lane, fly_lane
from tools.tuning.approach_procs import STOP_EOF, PosixHandle, ProcessGroup, RunnerError, spawn_posix
from tools.tuning.approach_verdict import verdict
from tools.tuning.board import Board
from tools.tuning.dock_pass import END_RTL_S, Point, home_position
from tools.tuning.flight_floor import HeightFloor
from tools.tuning.hitl import board_param_files
from tools.tuning.hitl_bridge.board_clock import DEFAULT_LOOP_RATE_HZ
from tools.tuning.hitl_bridge.bridge import DEFAULT_INTEGRATION_DT_S, PLANT_PROGRAM
from tools.tuning.hitl_procs import HitlProcesses
from tools.tuning.private_data import private_dir

MODE = "pc-sitl"
# The SITL binary speaking the bridge's HIL link contract v2 (the protocol-100 port, sitl_sim HIL_SERIAL);
# sitl-backing's default flight-tune build speaks v1 on a fixed SERIAL8 (SIM_HIL.h). My choice for step 13.
SIM_HIL_BINARY = Path.home() / "fw-worktrees" / "ardupilot-sim-hil" / "build" / "sitl" / "bin" / "arduplane"
YEGHVARD_ORIGIN = "40.3117412,44.4552112,1292.8,0"      # hitl_unreal_camera.md: the Unreal level's origin
SERVO_SPEED_S = 0.03            # the bridge's servo lag in the dock-pass runs (hitl_unreal_camera.md)
JSBSIM = "/usr/games/JSBSim"
AAS_SCRIPT = Path("scripts") / "lua" / "aas_params.lua"     # navpy's AAS_* parameters, as on the aircraft's SD card
BOARD_SCRIPTING = "board_scripting.parm"                   # the run's parameter file: SCR_ENABLE 1


def _jsbsim_version() -> str:
    text = subprocess.run([JSBSIM, "--version"], capture_output=True, text=True, timeout=30, check=False).stdout
    return next(line.strip() for line in text.splitlines() if "JSBSim Version" in line)


def _addresses() -> tuple[str, str]:
    """WSL's own address and the Windows host's (WSL's default gateway, NAT networking)."""
    own = subprocess.run(["hostname", "-I"], capture_output=True, text=True, check=True).stdout.split()[0]
    route = subprocess.run(["ip", "route", "show", "default"], capture_output=True, text=True, check=True).stdout
    return own, route.split()[2]


def _board_scripts(repo: Path, run_dir: Path) -> None:
    """The board's SD card in SITL: aas_params.lua where SITL loads Lua (its workdir, the backing's), CRLF stripped
    like the aircraft deploy step, and the run's parameter file enabling scripting; the board setup's reboot starts it.
    Without it navpy waits 6 s for each AAS_* parameter the board never answers."""
    scripts = run_dir / "backing" / "scripts"
    scripts.mkdir(parents=True, exist_ok=True)
    text = (repo / AAS_SCRIPT).read_text(encoding="utf-8").replace("\r\n", "\n")
    (scripts / AAS_SCRIPT.name).write_text(text, encoding="utf-8", newline="\n")
    (run_dir / BOARD_SCRIPTING).write_text("SCR_ENABLE 1\n", encoding="utf-8")


def _source_id(repo: Path) -> str:
    profiles = json.loads((repo / "src/navpy/core/perception/profile/vision_profiles.json").read_text(encoding="utf-8"))
    return profiles["profiles"][chain.VISION_PROFILE]["detector"]["camera_index"]


def _binary(path: str) -> dict:
    data = Path(path).read_bytes() if Path(path).is_file() else b""
    return {"path": path, "sha256": hashlib.sha256(data).hexdigest() if data else None}


def _corrections(path: str | None) -> tuple[float, float]:
    """The fit's constants for a held-out run; 0 for a calibration-only run (the fit never reads them)."""
    if not path:
        return 0.0, 0.0
    fit = json.loads(Path(path).read_text(encoding="utf-8"))
    if fit.get("schema") != FIT_SCHEMA or not fit.get("passed"):
        raise SystemExit(f"error: {path} is not a passing {FIT_SCHEMA}")
    return float(fit["attitude"]["correction_s"]), float(fit["gimbal"]["correction_s"])


def _record(args: argparse.Namespace, run_id: str, repo: Path, scenario: dict) -> dict:
    attitude_c, gimbal_c = _corrections(args.timing_constants)
    timing = IntegrationTimingConfig(run_id, MODE, _source_id(repo), chain.navpy_link_id(), IntegrationTimingLimits(),
                                     attitude_c, gimbal_c)
    bridge = approach_records.BridgeRun(PLANT_PROGRAM, _jsbsim_version(), DEFAULT_INTEGRATION_DT_S,
                                        DEFAULT_LOOP_RATE_HZ, SERVO_SPEED_S)
    return approach_records.new_record(
        run_id=run_id, mode=MODE, created=datetime.now(timezone.utc), model_input=read_model_input(args.model_input),
        model_dir=Path(args.model_dir), bridge=bridge, law=NavigationAlgorithm.VISION_NAV_PN.value,
        sources={"navpy": source_identity(repo)},
        media_runtime=json.loads(Path(args.media_runtime).read_text(encoding="utf-8")), integration_timing=timing,
        scenario=scenario, commissioning=args.commission_fault)


def _adopting_popen(group: ProcessGroup) -> Callable[..., subprocess.Popen]:
    def popen(argv: list[str], **kwargs: Any) -> subprocess.Popen:
        process = subprocess.Popen(argv, start_new_session=True, **kwargs)
        group.adopt("bridge", PosixHandle(process, STOP_EOF))
        return process
    return popen


def _fly(board: Board, group: ProcessGroup, run: dict, quiesce: stop.Quiesce) -> None:
    from tools.tuning.sitl_driver import takeoff        # sitl_driver imports this module
    limits = run["scenario"]["limits"]
    run["airspeed_pins"] = airspeed.pin_board(board, run["mode"], run["sitl_binary"])
    takeoff(board)
    flight = run["flight"] = {}
    with HeightFloor(board, limits["min_height_m"], flight, before_recovery=lambda: quiesce.run(stop.FLOOR)):
        lat, lon, _ = home_position(board)
        lane = build_lane(Point(lat, lon), LaneSpec.of(run["scenario"]["lane"]))
        flight["lane"] = lane.describe()
        fly_lane(board, lane)
        flight["lane_reached_utc"] = datetime.now(timezone.utc).isoformat()
        flight["ended_by"] = stop.observe(board, group, group.output_path("navpy"), limits["observe_s"])
    # quiesce first: navpy stopped, its endpoint closed
    record = quiesce.run(stop.TERMINAL if flight["ended_by"] == stop.ENDED_BY_TERMINAL else stop.DEADLINE)
    if record.get("rtl") != "direct":
        board.set_mode("RTL")
        board.fly_for(END_RTL_S)
    if not record.get("barrier", {}).get("ok"):
        raise RunnerError(f"navpy handoff failed: {record.get('failed')}")


def approach(args: argparse.Namespace) -> int:
    """One approach run of the HITL chain (mode pc-sitl): see tools/tuning/approach.py."""
    repo = Path.cwd()
    run_id = "approach-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = Path(args.run_root or private_dir() / "runs" / "approach") / run_id
    if args.command and not args.timing_constants:
        raise SystemExit("error: --command needs --timing-constants (a held-out run whose timing can be checked)")
    scenario = scenario_run.public(args.scenario_series, args.seed_index)     # refuses a bad series or seed first
    orbit_check_result = orbit_check(args, scenario)        # refuses an infeasible orbit before the run folder
    record = _record(args, run_id, repo, scenario)          # refuses bad inputs before the run folder exists
    run_dir.mkdir(parents=True, exist_ok=False)
    scenario_dir = run_dir / scenario_run.SCENARIO_DIR
    if scenario_run.public(args.scenario_series, args.seed_index, scenario_dir) != scenario:
        raise SystemExit("error: the scenario service prepared another scenario than its dry run")
    record_path = approach_records.write_record(record, run_dir)
    link_file = contact.link(run_dir, record_path)            # the contact key: private, one per run
    wsl_ip, windows_ip = _addresses()
    paths = chain.ChainPaths(run_dir, repo, args.python, Path(args.aas_hitl), Path(args.publisher_profile),
                             Path(args.uproject), wsl_ip, windows_ip, args.windows_python, args.unreal_affinity)
    lens = repo / "src/navpy/core/perception/profile/vision_profiles.json"
    group = ProcessGroup(run_dir, {"posix": spawn_posix, "windows": chain.spawn_unreal(args.unreal_affinity),
                                   "interop": handles.spawn_interop(chain.EMULATOR_PORTS, chain.EMULATOR_PID_LINE),
                                   "publisher": handles.spawn_publisher}, chain.port_free)
    run: dict = {"run_id": run_id, "mode": MODE, "record": str(record_path), "scenario": scenario,
                 "addresses": [wsl_ip, windows_ip], "commissioning": record["commissioning"],
                 "admission_problems": media_fault_spec.admission_problems(record),
                 "sitl_binary": _binary(args.sitl_binary), "orbit_check": orbit_check_result,
                 "authority": "command" if args.command else "observation"}
    hitl = HitlProcesses(chain.bridge_template(Path(args.model_dir), SERVO_SPEED_S, windows_ip, contact.pose_copy()),
                         None, 0,
                         run_dir / "board", args.origin, popen=_adopting_popen(group))
    files = board_param_files([run_dir / BOARD_SCRIPTING], [Path(p) for p in args.params_file or []], True, False,
                              args.hitl_data)
    board = Board(f"udpin:127.0.0.1:{chain.RUNNER_PORT}", run_dir / "board", files, origin=args.origin, hitl=hitl,
                  log_fetch=False)
    failed, reason, quiesce, handlers = None, stop.EXIT, stop.Quiesce.of(group), {}
    try:
        stop.arm(handlers)
        specs = [finish.host_load(paths), chain.backing(paths, args.origin, args.sitl_binary),
                 chain.router(paths, args.command), chain.emulator(paths),
                 chain.publisher(paths, chain.publisher_profile(paths), lens),
                 chain.unreal(paths, lens, scenario_run.unreal_file(run_dir), run["scenario"]["scene_hash"]),
                 contact.service(args.python, repo, run_dir, link_file, scenario_dir),
                 chain.navpy(paths, record_path, FIRST_LANE_SEQ, args.commission_fault.active,
                             contact.navpy_args(link_file))]
        problems = group.refusals(specs)
        if problems:
            raise RunnerError("; ".join(problems))
        owners.check(run, group, owners.PREFLIGHT)
        for spec in specs[:3]:
            group.start(spec)
        _board_scripts(repo, run_dir)                # after sitl-backing wiped its workdir, before the reboot
        board.start()
        for spec in specs[3:]:
            group.start(spec)
        owners.check(run, group, owners.READY)
        _fly(board, group, run, quiesce)
    except BaseException as exc:      # noqa: BLE001 - recorded; the quiesce and every stop still run below
        reason = stop.ABORT if isinstance(exc, (KeyboardInterrupt, stop.Aborted)) else stop.CRASH
        failed = run["error"] = f"{type(exc).__name__}: {exc}"
        print(f"[approach] run FAILED ({reason}): {failed}", flush=True)
    finally:
        while True:                             # a later signal waits for the cleanup
            try:
                stop.shield(handlers)
                break
            except (KeyboardInterrupt, stop.Aborted):
                continue                        # one that arrived as shield() began: shield again
        try:
            run["handoff"] = quiesce.end(group, reason)     # before the board's stop: navpy stopped, endpoint closed
            owners.check(run, group, owners.END)            # once the chain came up: still one owner each
            try:
                board.stop()
            except Exception as exc:      # noqa: BLE001 - recorded; the owned processes must still stop
                run["board_stop_error"] = f"{type(exc).__name__}: {exc}"
                failed = failed or run["board_stop_error"]
            group.mark_stopped("bridge", board.record.get("hitl", {}).get("bridge_exit"))
            group.stop_all()
            run["producers"], run["orphans"] = group.producers(), group.orphans()
            records = stop.navpy_records(group.output_path("navpy"))
            run["quiesce"] = quiesce.describe()
            run["outcome"] = stop.outcome(records, run.get("flight", {}).get("ended_by"))
            run["airspeed"] = airspeed.outcome(group.output_path("navpy"))
            run["observation"] = stop.audit(run_dir / "router_writes.csv", quiesce.record, records, args.command)
            run["scene"] = scenario_run.verify(scenario_dir, run_dir / "unreal.log")
            run["sources"] = approach_sources.sources(run_dir / "unreal.log", run_dir / "emulator.jsonl")
            finish.copy_logs(run_dir, board.workdir)
            run["board"] = board.report()
            manifest = finish.manifest(run_dir, run_id, record_path, group)
            run["manifest"] = str(manifest)
            run["timing"] = finish.timing(run_dir, record_path, manifest, args.timing_constants,
                                          finish.observation_run(scenario, args.command))
            run["host_load"] = finish.host_load_summary(run_dir)
            (run_dir / "approach_run.json").write_text(json.dumps(run, indent=1, default=str) + "\n", "utf-8")
        finally:
            stop.restore(handlers)
    return verdict(run, run_id, run_dir, failed)

def add_parser(sub: argparse._SubParsersAction) -> None:
    a = sub.add_parser("approach", help=approach.__doc__)
    a.add_argument("--mode", choices=(MODE,), required=True)
    a.add_argument("--run-root", help="folder for the run folder (default: the private folder's runs/approach)")
    a.add_argument("--origin", default=YEGHVARD_ORIGIN, help="lat,lon,alt,hdg (default: the Unreal level's)")
    a.add_argument("--sitl-binary", default=str(SIM_HIL_BINARY), help="the backing's arduplane (default: sim-hil)")
    a.add_argument("--model-dir", required=True, help="the JSBSim model folder (WSL path)")
    a.add_argument("--model-input", required=True, help="the model folder's declaration (tools/hitl/model_input.py)")
    a.add_argument("--media-runtime", required=True, help="a passing check_media_runtime.py result for pc-wsl")
    a.add_argument("--timing-constants", help="a passing step 12 fit: its corrections go into a held-out run")
    a.add_argument("--publisher-profile", required=True, help="the aas-hitl publisher profile template")
    a.add_argument("--aas-hitl", required=True, help="the aas-hitl checkout the publisher runs from (WSL path)")
    a.add_argument("--uproject", required=True, help="WSL path of the Unreal project (/mnt/c/...)")
    a.add_argument("--unreal-affinity", type=int, help="Unreal's processor mask (the CPU share granted)")
    a.add_argument("--python", default=sys.executable, help="the Python navpy and the tools run with")
    a.add_argument("--windows-python", required=True, help="WSL path of the Windows Python the emulator runs with")
    a.add_argument("--params-file", action="append", help="starting parameters, as dock-pass")
    a.add_argument("--board-height-m", type=float, required=True,
                   help="the board's design height above home (m), for the orbit check")
    a.add_argument("--hitl-data", type=Path, help="the aircraft's private HITL files (default: the private folder)")
    a.add_argument("--scenario-series", required=True,
                   help="the private series file (tools/hitl/scenario_service.py new-series); read only by the service")
    a.add_argument("--seed-index", type=int, required=True, help="the seed of the series this run flies (0-9)")
    a.add_argument("--command", action="store_true",
                   help="navpy commands the aircraft (a router client, step 27); needs --timing-constants")
    a.add_argument("--commission-fault", type=media_fault_spec.parse, default=media_fault_spec.NO_FAULT,
                   metavar="FAULT", help="SITL commissioning only: delay:N[@K] or loss[@K] (media_fault_spec.py)")
    a.set_defaults(func=approach)
