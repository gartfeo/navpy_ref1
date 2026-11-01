"""DOCKED, DOCK_FAILED and DOCK_ARMED evidence rows (physical_docking.md).

Docking counts only from a DOCKED row, the sim contact sensor's event,
matched to the episode's own DOCK_ARMED row and its command time span.
DOCK_FAILED is the NAV exit of an armed episode that ended without contact
(missed_dock.md); it is matched to its DOCK_ARMED row by identity only, since
a failure may come before any command.
"""

from __future__ import annotations

from dataclasses import dataclass

from navpy.core.approach.vision_nav.command_freshness import (
    FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S,
)
from scripts.eval_gcs_demo_models import (
    DockArmed,
    DockedEvidence,
    FinalApproachCommand,
)
from scripts.eval_gcs_demo_tokens import (
    canonical_integer as _integer,
    finite_number as _float,
    payload_fields as _payload,
)
from scripts.eval_gcs_demo_tokens import seconds_of_day

_DOCKED_FIELDS = (
    "vehicle",
    "task",
    "dock",
    "episode",
    "aircraft_ts",
    "closest_3d_m",
    "closest_h_m",
    "closest_v_m",
)
_DOCK_ARMED_FIELDS = ("vehicle", "task", "dock", "episode")
# Current shape: the legacy fields plus the NAV-entry ATTITUDE boot time.
_DOCK_ARMED_SOURCE_FIELDS = (*_DOCK_ARMED_FIELDS, "source_ts")
# Then the epoch wall time at NAV entry (the observation rows' clock).
_DOCK_ARMED_WALL_FIELDS = (*_DOCK_ARMED_SOURCE_FIELDS, "wall_s")
# A held command expires FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S of aircraft
# time after its frame (command_freshness.py) and the approach then ends as
# NAV_FAIL, so a genuine contact cannot come later than that after the last
# commanded frame.
DOCKED_AFTER_LAST_FRAME_MAX_S = FINAL_APPROACH_COMMAND_MAX_SOURCE_AGE_S
# The sim net radius (navpy.sim.world.dock_net_radius): contact is
# strictly inside it, so a DOCKED row outside it contradicts its own decision.
DOCK_NET_RADIUS_M = 0.5


def parse_docked(row: list[str], line_number: int) -> DockedEvidence:
    if len(row) < 3:
        raise ValueError(f"DOCKED row {line_number} has fewer than 3 columns")
    payload = _payload(row[2], _DOCKED_FIELDS, "DOCKED")
    distances = {
        name: _float(name, payload[name])
        for name in ("closest_3d_m", "closest_h_m", "closest_v_m")
    }
    if any(value is None or value < 0.0 for value in distances.values()):
        raise ValueError("DOCKED closest distances must be non-negative")
    if not distances["closest_3d_m"] < DOCK_NET_RADIUS_M:  # type: ignore[operator]
        raise ValueError(
            f"DOCKED closest_3d_m {distances['closest_3d_m']:g} is not inside "
            f"the {DOCK_NET_RADIUS_M:g} m net"
        )
    return DockedEvidence(
        wall_s=seconds_of_day(row[0]),
        vehicle=_integer("vehicle", payload["vehicle"], allow_zero=False),
        task=_integer("task", payload["task"], allow_zero=False),
        dock=_integer("dock", payload["dock"], allow_zero=True),
        episode=_integer("episode", payload["episode"], allow_zero=False),
        aircraft_ts=_float("aircraft_ts", payload["aircraft_ts"]),  # type: ignore[arg-type]
        closest_3d_m=distances["closest_3d_m"],  # type: ignore[arg-type]
        closest_h_m=distances["closest_h_m"],  # type: ignore[arg-type]
        closest_v_m=distances["closest_v_m"],  # type: ignore[arg-type]
    )


@dataclass(frozen=True)
class DockFailedEvidence:
    """One DOCK_FAILED row: the episode ended without contact, and why."""

    vehicle: int
    task: int
    dock: int
    episode: int
    reason: str

    @property
    def identity(self) -> tuple[int, int, int, int]:
        return self.vehicle, self.task, self.dock, self.episode


_DOCK_FAILED_FIELDS = (*_DOCK_ARMED_FIELDS, "reason")


def parse_dock_failed(row: list[str], line_number: int) -> DockFailedEvidence:
    if len(row) < 3:
        raise ValueError(f"DOCK_FAILED row {line_number} has fewer than 3 columns")
    payload = _payload(row[2], _DOCK_FAILED_FIELDS, "DOCK_FAILED")
    if not payload["reason"]:
        raise ValueError(f"DOCK_FAILED row {line_number} has no reason")
    return DockFailedEvidence(
        vehicle=_integer("vehicle", payload["vehicle"], allow_zero=False),
        task=_integer("task", payload["task"], allow_zero=False),
        dock=_integer("dock", payload["dock"], allow_zero=True),
        episode=_integer("episode", payload["episode"], allow_zero=False),
        reason=payload["reason"],
    )


def dock_failed_errors(
    failed: DockFailedEvidence,
    armed: DockArmed | None,
    docked: bool,
    vehicle_id: int | None,
) -> str | None:
    """Why a DOCK_FAILED row is invalid for the episode that just closed."""
    if armed is None:
        return "DOCK_FAILED without a DOCK_ARMED row for its episode"
    if failed.identity != armed.identity:
        return f"DOCK_FAILED {failed.identity} is not the armed episode {armed.identity}"
    if vehicle_id is not None and failed.vehicle != vehicle_id:
        return f"DOCK_FAILED for vehicle {failed.vehicle}, not this UAV {vehicle_id}"
    if docked:
        return f"DOCK_FAILED and DOCKED for one episode {failed.identity}"
    return None


def parse_dock_armed(row: list[str], line_number: int) -> DockArmed:
    if len(row) < 3:
        raise ValueError(f"DOCK_ARMED row {line_number} has fewer than 3 columns")
    if ";wall_s=" in row[2]:
        fields = _DOCK_ARMED_WALL_FIELDS
    elif ";source_ts=" in row[2]:
        fields = _DOCK_ARMED_SOURCE_FIELDS
    else:
        fields = _DOCK_ARMED_FIELDS
    payload = _payload(row[2], fields, "DOCK_ARMED")
    return DockArmed(
        vehicle=_integer("vehicle", payload["vehicle"], allow_zero=False),
        task=_integer("task", payload["task"], allow_zero=False),
        dock=_integer("dock", payload["dock"], allow_zero=True),
        episode=_integer("episode", payload["episode"], allow_zero=False),
        source_ts=_float("source_ts", payload.get("source_ts", ""), optional=True),
        wall_s=_float("wall_s", payload.get("wall_s", ""), optional=True),
    )


def docked_errors(
    docked: DockedEvidence,
    armed: DockArmed | None,
    current: list[FinalApproachCommand],
    vehicle_id: int | None,
    last_episode: int | None,
) -> str | None:
    """Why a DOCKED row does not count, or None when it does."""
    if not current:
        return "DOCKED outside a final-approach episode"
    if armed is None:
        return "DOCKED without a DOCK_ARMED row for its episode"
    if docked.identity != armed.identity:
        return (
            f"DOCKED {docked.identity} is not the armed episode {armed.identity}"
        )
    if vehicle_id is not None and docked.vehicle != vehicle_id:
        return f"DOCKED for vehicle {docked.vehicle}, not this UAV {vehicle_id}"
    task, obj = current[-1].task, current[-1].obj
    if docked.task != task:
        return f"DOCKED task {docked.task} is not the episode's task {task}"
    if docked.dock != obj:
        return f"DOCKED dock {docked.dock} is not the UAV's own dock {obj}"
    if last_episode is not None and docked.episode <= last_episode:
        return f"stale DOCKED episode {docked.episode} (after {last_episode})"
    first_ts = min(command.obs_ts for command in current)
    last_ts = max(command.obs_ts for command in current)
    if not first_ts <= docked.aircraft_ts <= last_ts + DOCKED_AFTER_LAST_FRAME_MAX_S:
        return (
            f"DOCKED aircraft_ts {docked.aircraft_ts:g} is outside the episode's "
            f"command span [{first_ts:g}, {last_ts:g}] + "
            f"{DOCKED_AFTER_LAST_FRAME_MAX_S:g} s"
        )
    return None


__all__ = [
    "DOCKED_AFTER_LAST_FRAME_MAX_S",
    "DOCK_NET_RADIUS_M",
    "DockFailedEvidence",
    "dock_failed_errors",
    "docked_errors",
    "parse_dock_armed",
    "parse_dock_failed",
    "parse_docked",
]
