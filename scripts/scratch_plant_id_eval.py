"""SCRATCH: run the open-loop plant-identification cases against live SITL.

One launch per case, calm air, same north-line mission and gate the navigation
sweeps use -- so the plant is characterised in the geometry the navigation
question is actually asked in, not in a different one.

Delete with `scratch_plant_id_child.py` once plant ID is closed out.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
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
from scratch_plant_id_child import OVERSHOOT_FIRST_ORDER_MAX  # noqa: E402
from upload_north_line_mission import (  # noqa: E402
    DEFAULT_ALT_M,
    DEFAULT_GATE_OFFSET_M,
    DEFAULT_WAYPOINT_OFFSET_M,
    upload_north_line,
)

# The plant is characterised in still air on purpose: wind would change the
# achieved flight path without changing the commanded attitude, which is the
# one relationship this experiment exists to measure.
WIND_SPEED_MPS = 0.0
WIND_DIR_DEG = 0.0


def _cases(raw: str) -> list[tuple[float, float]]:
    """Parse `pitch:roll,pitch:roll` into commanded attitude pairs."""
    cases: list[tuple[float, float]] = []
    for piece in raw.split(","):
        piece = piece.strip()
        if not piece:
            continue
        pitch, _, roll = piece.partition(":")
        cases.append((float(pitch), float(roll or 0.0)))
    return cases


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument(
        "--cases",
        default="-20:0,-20:30",
        help="commanded pitch:roll pairs in degrees",
    )
    parser.add_argument("--throttle", type=float, default=0.55)
    parser.add_argument("--level-s", type=float, default=3.0)
    parser.add_argument("--hold-s", type=float, default=20.0)
    parser.add_argument("--mission-alt", type=float, default=DEFAULT_ALT_M)
    parser.add_argument("--gate-offset", type=float, default=DEFAULT_GATE_OFFSET_M)
    parser.add_argument(
        "--target-offset", type=float, default=DEFAULT_WAYPOINT_OFFSET_M
    )
    parser.add_argument("--engage-wp", type=int, default=2, dest='scoring_start_wp')
    parser.add_argument("--home", default=DEFAULT_HOME_COORDS)
    parser.add_argument("--timeout", type=float, default=400.0)
    return parser


def _launch_child(
    python: Path,
    case_dir: Path,
    *,
    device: str,
    sysid: int,
    scoring_start_seq: int,
    pitch_deg: float,
    roll_deg: float,
    args: argparse.Namespace,
) -> subprocess.Popen[bytes]:
    import os

    command = [
        str(python),
        str(SCRIPTS / "scratch_plant_id_child.py"),
        "--connection", device,
        "--sysid", str(sysid),
        "--engage-seq", str(scoring_start_seq),
        "--timeout", repr(args.timeout),
        "--result", str(case_dir / "result.json"),
        "--engaged", str(case_dir / "engaged.marker"),
        "--pitch-deg", repr(pitch_deg),
        "--roll-deg", repr(roll_deg),
        "--throttle", repr(args.throttle),
        "--level-s", repr(args.level_s),
        "--hold-s", repr(args.hold_s),
    ]
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


def run_case(
    python: Path,
    root: Path,
    pitch_deg: float,
    roll_deg: float,
    args: argparse.Namespace,
    *,
    suffix: str = "",
    chat: int | None = None,
) -> dict[str, object]:
    # `suffix` keeps concurrent repeats of the SAME commanded attitude in
    # separate directories; without it parallel siblings collide on mkdir.
    case_dir = root / f"pitch{pitch_deg:g}-roll{roll_deg:g}{suffix}"
    case_dir.mkdir(parents=True)
    swarm = None
    child = None
    master = None
    # Slot this case OWNS, set only once the launcher confirms the grant.
    #
    # It must NOT start as the requested slot. `gcs_stop.py --chat N` kills
    # whatever the registry has on chat N without checking who is asking, so if
    # the launch failed precisely BECAUSE another session already holds that
    # slot, tearing down the requested chat would kill their stack.
    granted_chat: int | None = None
    try:
        # NO unscoped stop here. `stop_own_stack(python)` with no chat runs
        # `gcs_stop.py --eval`, which stops the WHOLE eval band -- every
        # sibling case running in parallel from this worktree. A case only ever
        # stops the one slot it owns.
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
            gate_offset=args.gate_offset,
            waypoint_offset=args.target_offset,
            alt_m=args.mission_alt,
            echo=lambda line: (case_dir / "mission.log").open(
                "a", encoding="utf-8"
            ).write(f"{line}\n"),
        )
        master = wait_for_heartbeat(ip.monitor_device(chat), 120.0)
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
        for name, value in (
            ("ARMING_CHECK", 0.0),
            ("SIM_WIND_SPD", WIND_SPEED_MPS),
            ("SIM_WIND_DIR", WIND_DIR_DEG),
        ):
            if not set_param(master, name, value):
                raise RuntimeError(f"parameter echo failed: {name}")
        child = _launch_child(
            python,
            case_dir,
            device=ip.companion_device(sysid),
            sysid=sysid,
            scoring_start_seq=run_navigation_episode.mission_seq,
            pitch_deg=pitch_deg,
            roll_deg=roll_deg,
            args=args,
        )
        _wait_ready(
            case_dir / "child.out.log", child, 180.0, marker="PLANT_ID_READY"
        )
        _start_mission(master)
        deadline_s = time.monotonic() + args.timeout
        while time.monotonic() < deadline_s:
            if child.poll() is not None:
                break
            time.sleep(0.2)
        result_path = case_dir / "result.json"
        if not result_path.exists():
            return {"passed": False, "errors": ["child produced no result"]}
        return json.loads(result_path.read_text(encoding="utf-8"))
    except Exception as error:  # noqa: BLE001 - one bad case must not kill the sweep
        return {"passed": False, "errors": [f"{type(error).__name__}: {error}"]}
    finally:
        _terminate(child)
        if master is not None:
            master.close()
        _terminate(swarm)
        # Scoped to the slot this case owns. Unscoped would kill siblings.
        if granted_chat is not None:
            stop_own_stack(python, chat=granted_chat)


def main() -> int:
    args = _parser().parse_args()
    cases = _cases(args.cases)
    root = WORKTREE / ".sitl-runs" / f"plant-id-{time.strftime('%Y%m%d-%H%M%S')}"
    root.mkdir(parents=True)
    rows: list[dict[str, object]] = []
    for pitch_deg, roll_deg in cases:
        print(f"case pitch={pitch_deg:g} roll={roll_deg:g}", flush=True)
        payload = run_case(args.python.resolve(), root, pitch_deg, roll_deg, args)
        payload["case"] = {"pitch_deg": pitch_deg, "roll_deg": roll_deg}
        rows.append(payload)
        print(f"  {json.dumps(payload.get('steady') or payload)}", flush=True)
    (root / "summary.json").write_text(
        json.dumps({"cases": rows}, indent=2), encoding="utf-8"
    )
    print()
    header = (
        f"{'cmd_pitch':>10}{'cmd_roll':>10}{'act_pitch':>11}{'gamma':>9}"
        f"{'AoA':>8}{'air_mps':>9}{'att_hz':>8}{'oshoot':>8}{'tau_s':>10}"
    )
    print(header)
    second_order = False
    for row in rows:
        case = row["case"]
        steady = row.get("steady") or {}
        step = row.get("step_response") or {}

        def _fmt(value: float | None, width: int, spec: str = ".2f") -> str:
            return f"{'--':>{width}}" if value is None else f"{value:>{width}{spec}}"

        # Judge from `overshoot_frac` rather than the child's `first_order_valid`
        # flag, so results recorded by an older child are judged too instead of
        # defaulting to "valid" on a missing key.
        overshoot = step.get("overshoot_frac")
        valid = overshoot is not None and overshoot <= OVERSHOOT_FIRST_ORDER_MAX
        if step.get("tau_s") is None:
            tau = f"{'--':>10}"
        elif valid:
            tau = f"{step['tau_s']:>10.3f}"
        else:
            # The crossing exists but is not a plant time constant. Printing it
            # bare next to clean steady-state numbers is what made it look like
            # one, so it does not get printed bare.
            tau = f"{step['tau_s']:.3f}!"
            tau = f"{tau:>10}"
            second_order = True

        print(
            f"{case['pitch_deg']:>10.1f}{case['roll_deg']:>10.1f}"
            f"{_fmt(steady.get('act_pitch_deg'), 11)}"
            f"{_fmt(steady.get('gamma_deg'), 9)}"
            f"{_fmt(steady.get('implied_aoa_deg'), 8)}"
            f"{_fmt(steady.get('air_speed_mps'), 9)}"
            f"{_fmt(row.get('attitude_update_rate_hz'), 8, '.1f')}"
            f"{_fmt(overshoot, 8, '.3f')}"
            f"{tau}"
        )
    if second_order:
        print(
            f"\n! tau_s is a 63.2% crossing and is NOT a plant time constant for"
            f" these cases:\n"
            f"  overshoot exceeds {OVERSHOOT_FIRST_ORDER_MAX:.0%}, so the response"
            f" is second order, not first order.\n"
            f"  Use peak_pitch_deg / time_to_peak_s / overshoot_frac in"
            f" result.json instead."
        )
    print(f"\nARTIFACT {root}")
    failed = [row for row in rows if row.get("passed") is not True]
    for row in failed:
        case = row["case"]
        detail = row.get("errors") or row.get("missing") or row.get("aborted")
        print(
            f"FAILED pitch={case['pitch_deg']:g} roll={case['roll_deg']:g}: {detail}",
            file=sys.stderr,
        )
    # A sweep whose cases failed must not look like a clean sweep to a caller.
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
