"""Put every aircraft in a fleet into the state its cell calls for.

Each aircraft needs its own mission, its own wind and its own speed plan, and
each of those is set by selecting that sysid on the shared link and talking to
it. Doing that for one aircraft is a few lines; doing it for thirty-six is the
bulk of a launch, which is why it sits apart from the run itself.

Every step here also doubles as a liveness check, and that is deliberate. The
shared launcher only verifies `sysids_for_chat(chat)[:instances]` -- never more
than three aircraft -- so nothing upstream confirms that aircraft four and up
ever came up. An aircraft that is not answering cannot have its mission read
back or its parameters echoed, so it fails here, by name, before anything flies.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
WORKTREE = SCRIPTS.parent
sys.path[:0] = [str(WORKTREE / "src"), str(WORKTREE), str(SCRIPTS)]

from pymavlink import mavutil  # noqa: E402

from scripts import eval_direct_pixel_pn as one  # noqa: E402
from scripts import eval_direct_pixel_pn_three_uav as many  # noqa: E402
from scripts.eval_direct_pixel_verdict import (  # noqa: E402
    SCORING_POLICY_SITL_TRUTH,
)
from scripts.eval_navigation_cases import (  # noqa: E402
    download_mission,
    request_coordinate_score_stream,
    require_nav_solution,
    resolve_home_abs_alt_m,
    resolve_target_expectation,
    set_param,
)
from scripts.eval_navigation_telemetry import (  # noqa: E402
    request_message_interval_stream,
)
from scripts.eval_navigation_truth import TRUTH_SCORE_RATE_HZ  # noqa: E402
from scripts.eval_sim_parameters import sim_parameters  # noqa: E402
from scripts.eval_source_identity import (  # noqa: E402
    script_import_closure, source_identity,
)
import eval_preboot_profile as preboot_profile  # noqa: E402
from scripts.pixel_pn_case_manifest import write_case_manifest  # noqa: E402
from scripts.pixel_pn_terminal_speed import terminal_speed_plan  # noqa: E402
from scripts.upload_north_line_mission import upload_north_line  # noqa: E402

# The single-case identity closure does NOT reach the fleet harness: it is
# derived from the flight child and the one-aircraft evaluator, so an edit to
# THIS path could fly unrecorded -- an old fleet and a rewired one would carry
# the same sha. The fleet closure starts from the fleet evaluator, so it
# contains every eval_fleet_* module plus the whole single-case closure it
# imports.
#
# The pre-boot profile is in here as well, and it is not a .py: the closure
# follows imports and source_identity globs *.py, so the file that decides
# every aircraft's GPS and height-delay timing would otherwise sit outside the
# hash. An edit between repetitions could then change the experiment while
# every source-consistency check still passed.
FLEET_IDENTITY_ROOTS = tuple(dict.fromkeys((
    *one.SOURCE_IDENTITY_ROOTS,
    *script_import_closure(
        Path(__file__).resolve().parent / "eval_direct_pixel_pn_fleet.py",
        SCRIPTS,
    ),
    preboot_profile.PROFILE,
)))


def fleet_source_identity() -> dict[str, object]:
    """The content hash of everything that decides how a FLEET flies."""
    return source_identity(FLEET_IDENTITY_ROOTS, WORKTREE)


@dataclass(frozen=True)
class FleetPlan:
    """Each aircraft's reference location, scoring start sequence, and wind."""

    targets: dict[int, object] = field(default_factory=dict)
    scoring_start_sequences: dict[int, int] = field(default_factory=dict)
    speed_plans: dict[int, object] = field(default_factory=dict)
    winds: dict[int, tuple[float, float]] = field(default_factory=dict)
    # TruthRecorder needs the home the truth stream is relative to, per
    # aircraft: co-located starts make them equal today, but reading it per
    # vehicle costs nothing and stops a later grid layout from silently
    # scoring every aircraft against aircraft one's home.
    home_alts: dict[int, float] = field(default_factory=dict)
    # None = never requested (estimate policy); a bool records the ACK.  The
    # ACK does not prove delivery -- the recorder certifies the stream.
    truth_stream_accepted: dict[int, bool | None] = field(default_factory=dict)


def upload_missions(
    device: str, sys_ids: list[int], args: argparse.Namespace, case_dir: Path
) -> None:
    """Give every aircraft the north line.

    Upload before the evaluator opens its own link -- both bind the same
    monitor port -- and before anything is told to fly. Without this the fleet
    would fly whatever route SITL booted with, the launcher's coverage plan,
    whose legs turn; on a turning route the wind direction relative to the
    ground track changes mid-run and a wind cell stops meaning anything.
    """
    for sys_id in sys_ids:
        upload_north_line(
            device,
            sys_id,
            home=one._home_lat_lon(args.home),
            loiter_offset=args.loiter_offset,
            gate_offset=args.gate_offset,
            waypoint_offset=args.target_offset,
            alt_m=args.mission_alt,
            echo=lambda line: (case_dir / "mission.log").open(
                "a", encoding="utf-8").write(f"{line}\n"),
        )


def require_same_source(identity: dict[str, object], when: str) -> None:
    """Void the case when the code under test changed mid-launch.

    Launching and readying a fleet takes minutes, and each child re-reads the
    law from disk as it starts. An edit landing inside that window would be
    flown but not recorded -- or worse, flown by only SOME of the children,
    a mixture invisible in the result. Raised as `SourceChangedError` so the
    matrix driver aborts the remaining repetitions instead of degrading this
    into one failed case among comparable ones.
    """
    flown = fleet_source_identity()
    if flown != identity:
        raise one.SourceChangedError(
            f"source changed {when} "
            f"({identity['sha256'][:12]} -> {flown['sha256'][:12]}); "
            "the fleet flown would not be the fleet recorded, so this case "
            "is void"
        )


@dataclass(frozen=True)
class AircraftManifest:
    """The case-constant inputs of each aircraft's `case.json`.

    The per-aircraft manifest is the certified single-case one, written by
    the same writer: the SIM_CPA scorer reads its expected target back from
    `case_dir/case.json`, and an archived aircraft directory must say what
    that aircraft flew -- its cell's EFFECTIVE parameter pushes included,
    which the fleet-level manifest cannot carry per aircraft.
    """

    identity: dict[str, object]
    speedup: float
    launch_speedup: float
    repetition: int


def cell_arguments(
    args: argparse.Namespace, speed: float, direction: float
) -> argparse.Namespace:
    """The shared CLI arguments with ONE cell's wind in place.

    `sim_parameters` reads the wind from the args it is given, which is right
    for the one-aircraft harness where the CLI wind IS the run's wind. In a
    fleet the wind is the cell's, so the certified tuple is built from a copy
    -- never by mutating the shared namespace, which would leak one aircraft's
    cell into the next aircraft's parameters.
    """
    return argparse.Namespace(
        **{**vars(args), "wind_speed": speed, "wind_dir": direction}
    )


def prepare_aircraft(
    master: object,
    sys_ids: list[int],
    winds: list[tuple[float, float]],
    *,
    args: argparse.Namespace,
    speedup: float,
    case_dirs: dict[int, Path] | None = None,
    sim_cpa: dict[int, object] | None = None,
    manifest: AircraftManifest | None = None,
) -> FleetPlan:
    """Read each aircraft's mission back and set the state its cell calls for.

    The parameters pushed are `sim_parameters`, the SAME certified tuple the
    one-aircraft harness pushes -- fence isolation and the 1000 Hz physics
    clock included -- not a hand-picked subset. The subset is what killed the
    first truth-scoring sweep: wind and ARMING_CHECK were pushed here while the
    cloned eeprom's geofence stayed live and RTL'd the climb.
    """
    plan = FleetPlan()
    for sys_id, (speed, direction) in zip(sys_ids, winds):
        many._select(master, sys_id)
        if not request_coordinate_score_stream(master):
            raise RuntimeError(f"coordinate stream not acknowledged by {sys_id}")
        # Absorb the EKF-origin wait before the timed children exist; the
        # arm-site re-check then normally clears on the first report.
        require_nav_solution(master, sys_id)
        truth_accepted: bool | None = None
        if args.scoring_policy == SCORING_POLICY_SITL_TRUTH:
            # Per-vehicle request on the shared link, after _select so the
            # interval lands on THIS aircraft. The ACK does not prove
            # delivery -- the recorder certifies the observed stream.
            truth_accepted = request_message_interval_stream(
                master,
                mavutil.mavlink.MAVLINK_MSG_ID_SIM_STATE,
                TRUTH_SCORE_RATE_HZ,
            )
        plan.truth_stream_accepted[sys_id] = truth_accepted
        mission = download_mission(master)
        home_alt = resolve_home_abs_alt_m(master, timeout_s=30.0)
        plan.home_alts[sys_id] = home_alt
        plan.targets[sys_id] = resolve_target_expectation(
            mission, target_wp=args.target_wp,
            target_rel_alt_m=args.target_alt, home_abs_alt_m=home_alt,
        ).location
        plan.scoring_start_sequences[sys_id] = resolve_target_expectation(
            mission, target_wp=args.scoring_start_wp,
            target_rel_alt_m=args.target_alt, home_abs_alt_m=home_alt,
        ).mission_seq
        plan.speed_plans[sys_id] = terminal_speed_plan(
            args, speedup, mission, home_alt
        )
        plan.winds[sys_id] = (speed, direction)
        cell = cell_arguments(args, speed, direction)
        if manifest is not None and case_dirs is not None:
            # BEFORE any parameter push, like the single case: an aircraft
            # that dies on an unserved parameter must still leave the record
            # of what it was configured to fly -- and the SIM_CPA scorer
            # later reads its expected target back from this file.
            write_case_manifest(
                case_dirs[sys_id],
                speedup=manifest.speedup,
                launch_speedup=manifest.launch_speedup,
                repetition=manifest.repetition,
                target=plan.targets[sys_id],
                scoring_start_seq=plan.scoring_start_sequences[sys_id],
                identity=manifest.identity,
                speed_plan=plan.speed_plans[sys_id],
                sitl_params=args.sitl_param or [],
                sitl_params_pushed=sim_parameters(cell),
            )
        for name, value in sim_parameters(cell):
            if not set_param(master, name, value):
                raise RuntimeError(f"sysid={sys_id} parameter echo failed: {name}")
        if sim_cpa is not None and case_dirs is not None:
            # After the parameter pushes and before any child exists -- the
            # same lifecycle point the one-aircraft harness binds at.  The
            # stage never raises; a cross-check fault flies stream-scored.
            sim_cpa[sys_id].pre_flight(
                master,
                sysid=sys_id,
                target=plan.targets[sys_id],
                case_dir=case_dirs[sys_id],
            )
    return plan


__all__ = [
    "AircraftManifest",
    "FLEET_IDENTITY_ROOTS",
    "FleetPlan",
    "cell_arguments",
    "fleet_source_identity",
    "prepare_aircraft",
    "require_same_source",
    "upload_missions",
]
