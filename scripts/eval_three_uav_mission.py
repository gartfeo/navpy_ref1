"""Mission setup for the three-UAV direct-pixel harness.

Split out of ``scripts/eval_direct_pixel_pn_three_uav.py``: that harness sits
on a frozen line-size debt, and the upload it was missing does not fit beside
the per-vehicle read-back it already had.  The two are one step -- put a known
mission on every vehicle, then read back where each vehicle's POI is -- so
they move together rather than leaving half the step behind.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable

from gcs.backend import instance_ports as ip
from scripts.eval_navigation_cases import (
    download_mission,
    request_coordinate_score_stream,
    require_nav_solution,
    resolve_home_abs_alt_m,
    resolve_poi_expectation,
    set_param,
)
from scripts.upload_north_line_mission import upload_north_line


# Metres between vehicles at launch.  Zero because `upload_north_line` verifies
# each vehicle's own home against the requested one within 25 m
# (`MAX_HOME_DRIFT_M`), and the launcher's default 50 m grid puts vehicles 2 and
# 3 outside that.  It also decides what the harness measures -- co-located
# replicates isolate concurrency, spaced starts would sample distinct geometries
# -- so it is named here rather than left as a literal at the call site, and it
# goes into every recorded result.
LAUNCH_SPACING_M = 0


def geometry_record(args: object) -> dict[str, object]:
    """The geometry a result was produced under, for the run artifact.

    Two runs of this harness used to be indistinguishable in `summary.json`
    even when they flew different missions from different start points: the
    recorded fields were pass/fail, speedup, repetition and wind only.  That is
    how the 2026-08-17 change of `--home` went unnoticed for four days while
    the mission it was scored against stayed where the launcher template had
    seeded it.  Stamping the geometry makes results that are not comparable say
    so, instead of looking alike.
    """
    return {
        "wind_speed_mps": args.wind_speed,
        "wind_dir_deg": args.wind_dir,
        "home": args.home,
        "launch_spacing_m": LAUNCH_SPACING_M,
        "loiter_offset_m": args.loiter_offset,
        "gate_offset_m": args.gate_offset,
        "poi_offset_m": args.poi_offset,
        "mission_alt_m": args.mission_alt,
        "poi_alt_m": args.poi_alt,
    }


def _echo(path: Path) -> Callable[[str], None]:
    """Append the uploader's transcript to a per-vehicle log."""

    def write(line: str) -> None:
        with path.open("a", encoding="utf-8") as handle:
            handle.write(f"{line}\n")

    return write


def upload_missions(
    chat: int,
    sys_ids: list[int],
    *,
    home: tuple[float, float],
    loiter_offset: float,
    gate_offset: float,
    waypoint_offset: float,
    alt_m: float,
    case_dir: Path,
) -> None:
    """Put the north line on every vehicle in the swarm.

    Call this before the evaluator opens its own link: both bind the same
    monitor port.  Without an upload each vehicle flies whatever mission its
    launcher template happens to hold, which is hand-seeded state inside WSL
    with two failure modes the harness cannot see:

    * the template's waypoints are absolute coordinates, so they stay where
      they were seeded no matter what ``--home`` the run launches at.  Since
      home defaulted to open water the downloaded POI sat about 1000 km
      from the vehicles, and the run scored a POI it could never reach.
    * ``run_swarm.sh`` copies each template's ``eeprom.bin`` into every
      non-template instance on every launch, so one launch on a template slot
      that clears the stored mission count propagates to every clone of that
      template.  Measured on 2026-08-21: template 1's ``MIS_TOTAL`` read 0, so
      every sysid congruent to 1 mod 3 -- the first vehicle of every eval trio
      -- booted with no waypoints and POI discovery raised before any
      navigation ran.

    Uploading here removes the dependency on that state entirely, and matches
    what the single-UAV harness has always done.
    """
    device = ip.monitor_device(chat)
    for sys_id in sys_ids:
        upload_north_line(
            device,
            sys_id,
            home=home,
            loiter_offset=loiter_offset,
            gate_offset=gate_offset,
            waypoint_offset=waypoint_offset,
            alt_m=alt_m,
            echo=_echo(case_dir / f"mission-{sys_id}.log"),
        )


def prepare_vehicles(
    master: object,
    sys_ids: list[int],
    *,
    select: Callable[[object, int], None],
    poi_wp: int,
    scoring_start_wp: int,
    poi_rel_alt_m: float,
    parameters: tuple[tuple[str, float], ...],
) -> tuple[dict[int, object], dict[int, int]]:
    """Read every vehicle's POI back and pin its simulator parameters.

    The read-back is what makes the upload verifiable end to end: the POI
    the children are given comes from what the vehicle actually stored, not
    from what the uploader believes it sent.
    """
    pois: dict[int, object] = {}
    scoring_start_sequences: dict[int, int] = {}
    for sys_id in sys_ids:
        select(master, sys_id)
        if not request_coordinate_score_stream(master):
            raise RuntimeError(f"coordinate stream not acknowledged by {sys_id}")
        # Absorb the EKF-origin wait here, before the timed children exist;
        # the arm-site re-check then normally clears on the first report
        # instead of spending its timeout inside the children's deadlines.
        require_nav_solution(master, sys_id)
        mission = download_mission(master)
        home_alt = resolve_home_abs_alt_m(master, timeout_s=30.0)
        pois[sys_id] = resolve_poi_expectation(
            mission,
            poi_wp=poi_wp,
            poi_rel_alt_m=poi_rel_alt_m,
            home_abs_alt_m=home_alt,
        ).location
        scoring_start_sequences[sys_id] = resolve_poi_expectation(
            mission,
            poi_wp=scoring_start_wp,
            poi_rel_alt_m=poi_rel_alt_m,
            home_abs_alt_m=home_alt,
        ).mission_seq
        for name, value in parameters:
            if not set_param(master, name, value):
                raise RuntimeError(f"sysid={sys_id} parameter echo failed: {name}")
    return pois, scoring_start_sequences
