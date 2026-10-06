"""SCRATCH: fly the plant baseline matrix, in parallel, and tabulate it.

This is the BASELINE the navigation law gets compared against: what the airframe
does when told exactly what to do, with no vision and no law in the loop. Any
error the law shows beyond what appears here is the law's, not the aircraft's.

Flights are staircases of held attitudes (see `scratch_plant_matrix_child.py`),
so one climb buys many measurements. Concurrency is bounded by
`--max-parallel`, which DEFAULTS TO 1 and must stay there for a single worktree
AS CURRENTLY IMPLEMENTED: eval slots are keyed by worktree path
(`instance_registry_store.owner_for`), so concurrent launches from one directory
all resolve to the same slot and the losers are refused.

That is an implementation limit, not a fixed property. The worktree key exists to
identify WHICH code is flying; it is not meant to cap how many instances may fly
from it. One REGISTRY SLOT is what is scarce, not one aircraft -- VEHICLES_PER_CHAT
is 3 (`instance_ports.py:52`) and `swarm_run_runner.py:145` launches
`args.instances` sysids under a single supervisor, so up to three cases could
share one slot on distinct sysids. Lifting the limit properly means either
batching cases onto those sysids or letting one worktree own several slots.

Every child independently checks its own simulator clock -- both rate and drift
-- and fails itself rather than contribute time-scaled numbers.

Delete with the other plant scratch harnesses once the baseline is done.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

WORKTREE = Path(__file__).resolve().parent.parent
SCRIPTS = Path(__file__).resolve().parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from eval_direct_pixel_pn import (  # noqa: E402
    DEFAULT_HOME_COORDS,
    _home_lat_lon,
    _start_mission,
    _start_swarm,
    _terminate,
    _wait_ready,
)
from eval_navigation_cases import (  # noqa: E402
    download_mission,
    resolve_home_abs_alt_m,
    resolve_target_expectation,
    set_param,
    stop_own_stack,
    wait_for_heartbeat,
)
from gcs.backend import instance_ports as ip  # noqa: E402
from upload_north_line_mission import (  # noqa: E402
    DEFAULT_GATE_OFFSET_M,
    DEFAULT_WAYPOINT_OFFSET_M,
    upload_north_line,
)

EVAL_CHAT_LO, EVAL_CHAT_HI = ip.eval_band()

# Measured from the plant-ID runs: the airframe settles near this airspeed at
# 0.55 throttle, and its angle of attack is stable at about -3.3 deg. Used ONLY
# to predict the altitude a flight will consume so a staircase can be rejected
# before it flies into the floor -- never to interpret a result.
NOMINAL_AIRSPEED_MPS = 27.4
NOMINAL_AOA_DEG = -3.3
ALTITUDE_FLOOR_M = 60.0
ALTITUDE_MARGIN_M = 40.0

#: Baseline matrix. `alt` is the mission altitude the staircase needs.
FLIGHTS: dict[str, dict] = {
    # A. static map: commanded pitch -> achieved pitch, gamma, AoA, airspeed.
    "A1-shallow": {
        "segments": "-5:0:6,-10:0:6,-20:0:6,-30:0:6",
        "alt": 400.0,
    },
    "A2-steep": {
        "segments": "-40:0:4,-50:0:4,-60:0:4,-70:0:4",
        "alt": 700.0,
    },
    # A3. roll coupling at a fixed pitch.
    "A3-roll": {
        "segments": "-20:0:6,-20:15:6,-20:30:6,-20:45:6",
        "alt": 500.0,
    },
    # B. wind. The plant is air-relative, so this mainly proves gamma-over-
    # ground and ground speed move while the attitude map does NOT.
    "B1-head10": {"segments": "-20:0:8,-30:0:8", "alt": 400.0,
                  "wind_speed": 10.0, "wind_dir": 0.0},
    "B2-tail10": {"segments": "-20:0:8,-30:0:8", "alt": 400.0,
                  "wind_speed": 10.0, "wind_dir": 180.0},
    "B3-cross10": {"segments": "-20:0:8,-30:0:8", "alt": 400.0,
                   "wind_speed": 10.0, "wind_dir": 90.0},
    "B4-tail5": {"segments": "-20:0:8,-30:0:8", "alt": 400.0,
                 "wind_speed": 5.0, "wind_dir": 180.0},
    # C. speed, via throttle. Airspeed changes the turn/pull authority the law
    # implicitly assumes.
    "C1-thr35": {"segments": "-20:0:8,-45:0:5", "alt": 500.0, "throttle": 0.35},
    "C2-thr75": {"segments": "-20:0:8,-45:0:5", "alt": 500.0, "throttle": 0.75},
    # D. SMALL-SIGNAL. The law commands increments of a fraction of a degree to
    # a few degrees, never 20 deg steps, so this is the regime that actually
    # governs terminal accuracy -- and the one the single-step plant ID missed.
    "D1-small-20": {
        "segments": "-20:0:5,-19:0:3,-20:0:3,-18:0:3,-20:0:3,"
                    "-15:0:3,-20:0:3,-10:0:3,-20:0:3",
        "alt": 600.0,
    },
    "D2-small-45": {
        "segments": "-45:0:4,-44:0:3,-45:0:3,-42:0:3,-45:0:3,-35:0:3,-45:0:3",
        "alt": 900.0,
    },
}


def _predicted_drop_m(segments: str) -> float:
    """Altitude a staircase will consume, from the measured pitch->gamma map."""
    total = 0.0
    for piece in segments.split(","):
        pitch, _roll, hold = (float(part) for part in piece.strip().split(":"))
        gamma = pitch - NOMINAL_AOA_DEG
        total += max(0.0, -NOMINAL_AIRSPEED_MPS * math.sin(math.radians(gamma))) * hold
    return total


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument(
        "--flights",
        default="all",
        help="comma-separated flight names, or 'all'",
    )
    parser.add_argument("--max-parallel", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--home", default=DEFAULT_HOME_COORDS)
    parser.add_argument("--engage-wp", type=int, default=2, dest='scoring_start_wp')
    parser.add_argument(
        "--allow-floor-risk",
        action="store_true",
        help="fly staircases predicted to reach the altitude floor",
    )
    return parser


def _launch_command(
    python: Path,
    case_dir: Path,
    *,
    device: str,
    sysid: int,
    scoring_start_seq: int,
    flight: dict,
    timeout_s: float,
) -> list[str]:
    """Exact child argv. Shared with the pre-flight probe so the probe cannot
    validate a command that differs from the one that flies."""
    # `--opt=value`, NOT `--opt value`.
    #
    # A segment spec starts with '-' (e.g. "-5:0:6,..."). argparse only accepts
    # a leading-dash token as a VALUE when it matches its negative-number
    # pattern, so "-20.0" is fine but "-5:0:6" is parsed as an unknown OPTION
    # and the child dies with "argument --segments: expected one argument"
    # before it flies anything. The `=` form removes the ambiguity for every
    # argument, so no future value can fall into the same trap.
    command = [
        str(python),
        str(SCRIPTS / "scratch_plant_matrix_child.py"),
        f"--connection={device}",
        f"--sysid={sysid}",
        f"--engage-seq={scoring_start_seq}",
        f"--timeout={timeout_s!r}",
        f"--result={case_dir / 'result.json'}",
        f"--engaged={case_dir / 'engaged.marker'}",
        f"--segments={flight['segments']}",
        f"--throttle={float(flight.get('throttle', 0.55))!r}",
        f"--floor-rel-alt-m={ALTITUDE_FLOOR_M!r}",
    ]
    return command


def _launch_child(
    python: Path,
    case_dir: Path,
    *,
    device: str,
    sysid: int,
    scoring_start_seq: int,
    flight: dict,
    timeout_s: float,
) -> subprocess.Popen[bytes]:
    import os

    command = _launch_command(
        python, case_dir, device=device, sysid=sysid,
        scoring_start_seq=scoring_start_seq, flight=flight, timeout_s=timeout_s,
    )
    (case_dir / "child.cmd.json").write_text(
        json.dumps(command, indent=2), encoding="utf-8"
    )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(WORKTREE / "src")
    environment["PYTHONUTF8"] = "1"
    environment["PYTHONIOENCODING"] = "utf-8"
    environment["NAVPY_LOG_DIR"] = str(case_dir / "navpy-logs")
    return subprocess.Popen(
        command,
        cwd=str(WORKTREE),
        env=environment,
        stdout=(case_dir / "child.out.log").open("wb"),
        stderr=(case_dir / "child.err.log").open("wb"),
    )


def run_flight(
    python: Path,
    root: Path,
    name: str,
    flight: dict,
    args: argparse.Namespace,
    chat: int | None = None,
) -> dict[str, object]:
    case_dir = root / name
    case_dir.mkdir(parents=True)
    swarm = None
    child = None
    master = None
    # Slot this flight OWNS, set only once the launcher confirms the grant.
    # Never pre-seeded with the REQUESTED slot: `gcs_stop.py --chat N` kills
    # whatever the registry holds on chat N without checking the caller, so if
    # the launch failed because another session owns that slot, tearing it down
    # would kill their stack.
    granted_chat: int | None = None
    try:
        # Explicit slot, and no unscoped stop. `--eval` alone picks the first
        # free chat, so concurrent launches all choose the SAME one and every
        # loser dies with "chat N already has a live SITL supervisor"; an
        # unscoped stop would instead tear down every sibling.
        swarm, verdict = _start_swarm(python, case_dir, 1.0, instances=1,
                                      home=args.home, chat=chat)
        granted_chat = verdict.chat
        chat = granted_chat
        sysid = ip.sysids_for_chat(chat)[0]
        lat, lon = _home_lat_lon(args.home)
        upload_north_line(
            ip.monitor_device(chat),
            sysid,
            home=(lat, lon),
            gate_offset=DEFAULT_GATE_OFFSET_M,
            waypoint_offset=DEFAULT_WAYPOINT_OFFSET_M,
            alt_m=float(flight["alt"]),
            echo=lambda line: (case_dir / "mission.log").open(
                "a", encoding="utf-8"
            ).write(f"{line}\n"),
        )
        master = wait_for_heartbeat(ip.monitor_device(chat), 180.0)
        if master is None:
            raise RuntimeError("no evaluator heartbeat")
        mission = download_mission(master)
        home_alt = resolve_home_abs_alt_m(master, timeout_s=30.0)
        run_navigation_episode = resolve_target_expectation(
            mission,
            target_wp=args.scoring_start_wp,
            target_rel_alt_m=60.0,
            home_abs_alt_m=home_alt,
        )
        for param, value in (
            ("ARMING_CHECK", 0.0),
            ("SIM_WIND_SPD", float(flight.get("wind_speed", 0.0))),
            ("SIM_WIND_DIR", float(flight.get("wind_dir", 0.0))),
        ):
            if not set_param(master, param, value):
                raise RuntimeError(f"parameter echo failed: {param}")
        child = _launch_child(
            python,
            case_dir,
            device=ip.companion_device(sysid),
            sysid=sysid,
            scoring_start_seq=run_navigation_episode.mission_seq,
            flight=flight,
            timeout_s=args.timeout,
        )
        _wait_ready(
            case_dir / "child.out.log", child, 300.0, marker="PLANT_MATRIX_READY"
        )
        _start_mission(master)
        deadline_s = time.monotonic() + args.timeout
        while time.monotonic() < deadline_s:
            if child.poll() is not None:
                break
            time.sleep(0.2)
        result_path = case_dir / "result.json"
        if not result_path.exists():
            return {"passed": False, "errors": ["child produced no result"],
                    "flight": name}
        payload = json.loads(result_path.read_text(encoding="utf-8"))
        payload["flight"] = name
        return payload
    except Exception as error:  # noqa: BLE001 - one bad flight must not kill the matrix
        return {"passed": False, "errors": [f"{type(error).__name__}: {error}"],
                "flight": name}
    finally:
        _terminate(child)
        if master is not None:
            master.close()
        _terminate(swarm)
        if granted_chat is not None:
            stop_own_stack(python, chat=granted_chat)


def _print_tables(rows: list[dict]) -> None:
    print("\n=== STATIC BASELINE: commanded attitude -> what the airframe does ===")
    print(f"{'flight':14}{'cmd_p':>7}{'cmd_r':>7}{'act_p':>8}{'err':>7}"
          f"{'gamma':>8}{'AoA':>7}{'air':>7}{'sink':>7}")
    for row in rows:
        if row.get("passed") is not True:
            continue
        for report in row.get("segment_reports", []):
            s = report["steady"]
            def f(value: float | None, spec: str = ".2f") -> str:
                return "--" if value is None else format(value, spec)
            print(f"{row['flight']:14}{s['cmd_pitch_deg']:>7.0f}"
                  f"{s['cmd_roll_deg']:>7.0f}{f(s['act_pitch_deg']):>8}"
                  f"{f(s['pitch_tracking_error_deg']):>7}{f(s['gamma_deg']):>8}"
                  f"{f(s['implied_aoa_deg']):>7}{f(s['air_speed_mps'],'.1f'):>7}"
                  f"{f(s['sink_m_s'],'.1f'):>7}")

    print("\n=== RESPONSE: how fast, and what range it costs ===")
    print(f"{'flight':14}{'step':>7}{'p_t90':>8}{'g_t90':>8}{'g_settle':>9}"
          f"{'band':>6}{'over':>7}{'m@25':>7}{'m@39':>7}")
    unsettled = 0
    for row in rows:
        if row.get("passed") is not True:
            continue
        for report in row.get("segment_reports", []):
            r = report.get("response")
            if not r or r.get("gamma_t90_s") is None:
                continue
            settle = r.get("gamma_settle_s")
            def f(value: float | None, spec: str = ".3f") -> str:
                return "--" if value is None else format(value, spec)
            if settle is None:
                unsettled += 1
                span = ">seg"
            else:
                span = ""
            # Range is computed from SETTLING, not from the first 90% crossing.
            # The response overshoots, so the first crossing is not completion.
            #
            # `is None`, NOT truthiness: a settle time of 0.0 is a real
            # measurement (the signal never left the band) and its range is
            # 0.0 m. Testing `if settle` would print it as missing data, in the
            # same "--" form used for a segment that never settled at all, while
            # not counting it in `unsettled` -- so a zero would read as absent.
            reach = (
                f"{'--':>7}{'--':>7}" if settle is None
                else f"{settle*25.0:>7.1f}{settle*39.1:>7.1f}"
            )
            print(f"{row['flight']:14}{r['step_pitch_deg']:>7.0f}"
                  f"{f(r.get('pitch_t90_s')):>8}{f(r.get('gamma_t90_s')):>8}"
                  f"{(f(settle)+span):>9}"
                  f"{f(r.get('gamma_settle_band_deg'),'.2f'):>6}"
                  f"{f(r.get('gamma_overshoot_frac'),'.2f'):>7}{reach}")
    print("\nm@25 / m@39 = range consumed before gamma SETTLES, at the closing")
    print("speeds measured in the terminal phase. Computed from g_settle, not")
    print("from g_t90: the response overshoots, so the first 90% crossing")
    print("happens on the way up and is not a completion time.")
    print("band = settling band in degrees, max(10% of step, 0.20 deg floor).")
    if unsettled:
        print(f"\n{unsettled} segment(s) marked '>seg' never stayed in band before")
        print("the segment ended. Their settling time is longer than this flight")
        print("can measure -- treat every range above as a LOWER bound.")


def main() -> int:
    args = _parser().parse_args()
    names = (
        list(FLIGHTS)
        if args.flights.strip() == "all"
        else [n.strip() for n in args.flights.split(",") if n.strip()]
    )
    unknown = [name for name in names if name not in FLIGHTS]
    if unknown:
        raise SystemExit(f"unknown flight(s): {unknown}; known: {list(FLIGHTS)}")

    # Dry-run every child command BEFORE any SITL launch. The previous run
    # burned its whole matrix discovering at flight time that argparse rejected
    # the segment spec; a launch that cannot parse its own arguments must fail
    # in the first second, not after a climb to altitude.
    import subprocess as _sp
    import tempfile as _tf
    root_probe = Path(_tf.mkdtemp(prefix="plant-matrix-probe-"))
    for name in names:
        # The REAL command plus --check-args, so the probe exercises exactly
        # what will fly. A `--help` probe short-circuits before argparse checks
        # required arguments and passed a command that then died in flight.
        probe = _sp.run(
            _launch_command(
                args.python.resolve(),
                root_probe,
                device="udp:0.0.0.0:1",
                sysid=1,
                scoring_start_seq=1,
                flight=FLIGHTS[name],
                timeout_s=args.timeout,
            ) + ["--check-args"],
            capture_output=True, text=True, timeout=60,
        )
        if probe.returncode != 0:
            raise SystemExit(
                f"child rejects arguments for {name}: "
                f"{(probe.stderr or probe.stdout).strip().splitlines()[-1]}"
            )

    risky = []
    for name in names:
        flight = FLIGHTS[name]
        drop = _predicted_drop_m(flight["segments"])
        usable = float(flight["alt"]) - ALTITUDE_FLOOR_M - ALTITUDE_MARGIN_M
        if drop > usable:
            risky.append((name, drop, usable))
    if risky and not args.allow_floor_risk:
        for name, drop, usable in risky:
            print(f"REFUSE {name}: staircase needs ~{drop:.0f} m, only {usable:.0f} m "
                  f"usable above the floor. Raise `alt` or shorten holds.",
                  file=sys.stderr)
        raise SystemExit("altitude budget exceeded; pass --allow-floor-risk to fly anyway")

    # Refused, not merely defaulted. Eval slots are keyed by worktree path
    # (`instance_registry_store.owner_for`), so concurrent launches from one
    # directory all resolve to the same slot and the losers are refused mid-run
    # -- which looks like flaky flights, not like a configuration error. The
    # refusal guards TODAY's slot allocation; see the module docstring for why
    # this is an implementation limit rather than a fixed one.
    if args.max_parallel > 1:
        raise SystemExit(
            f"--max-parallel={args.max_parallel} is not supported from a single "
            "worktree today: eval slots are keyed by worktree path, so parallel "
            "launches here all resolve to the same slot. Use one worktree per "
            "concurrent flight, batch cases onto distinct sysids within one "
            "slot, or keep --max-parallel=1."
        )
    root = WORKTREE / ".sitl-runs" / f"plant-matrix-{time.strftime('%Y%m%d-%H%M%S')}"
    root.mkdir(parents=True)
    python = args.python.resolve()
    print(f"flights: {names}\nmax parallel: {args.max_parallel}", flush=True)
    rows: list[dict] = []
    with ThreadPoolExecutor(max_workers=max(1, args.max_parallel)) as pool:
        futures = {
            pool.submit(
                run_flight, python, root, name, FLIGHTS[name], args,
            ): name
            for name in names
        }
        for future in futures:
            rows.append(future.result())
            print(f"  done {futures[future]}", flush=True)
    rows.sort(key=lambda row: names.index(row["flight"]))
    (root / "summary.json").write_text(
        json.dumps({"flights": rows}, indent=2), encoding="utf-8"
    )
    _print_tables(rows)
    failed = [row for row in rows if row.get("passed") is not True]
    for row in failed:
        detail = row.get("missing") or row.get("errors") or row.get("aborted")
        print(f"FAILED {row['flight']}: {detail}", file=sys.stderr)
    print(f"\nARTIFACT {root}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
