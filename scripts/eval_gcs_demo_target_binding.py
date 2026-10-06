"""Bind terminal command episodes to their assigned physical targets."""

from __future__ import annotations

import math
import re
from pathlib import Path

from scripts.eval_gcs_demo_evidence import (
    mission_navigation_segment,
    parse_terminal_command_episodes,
)
from scripts.eval_gcs_demo_scenario import DemoMissionPlan
from scripts.eval_gcs_demo_vehicle import navigation_log


MAX_TARGET_COORD_ERROR_M = 1.0
_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
_TARGET_RE = re.compile(
    r"TARGET:\s*T(?P<task>[1-9]\d*)\s*"
    r"\(tracking obj_id=(?P<obj>0|[1-9]\d*)\)"
)
_TARGET_LOCATION_RE = re.compile(
    rf"\bt_l\s*\([^)]*\):\s*"
    rf"(?P<lat>{_NUMBER}),\s*(?P<lon>{_NUMBER}),"
)
_CONFIRMING_RE = re.compile(r"\bCONFIRMING:\s*T(?P<task>[1-9]\d*)\b")
_CONFIRMED_RE = re.compile(
    r"\bTarget\s+(?P<task>[1-9]\d*)\s+confirmed by ground station\."
)


def terminal_target_binding_errors(
    log_dir: Path,
    plan: DemoMissionPlan,
    global_assignments: dict[int, int],
    approved_tasks: dict[int, int],
) -> list[str]:
    """Certify selection -> command identity -> truth-coordinate continuity.

    The target coordinate is simulator-only scoring evidence read after the
    command is formed.  It is never supplied to the final approach law.
    """
    expected_by_id = {
        target.task_id: (target.lat, target.lon)
        for target in plan.targets
    }
    errors: list[str] = []
    for vehicle in plan.vehicles:
        sys_id = vehicle.sys_id
        assigned_task = global_assignments.get(sys_id)
        expected = expected_by_id.get(assigned_task)
        if expected is None:
            errors.append(f"vehicle {sys_id} has no resolved assigned target")
            continue
        try:
            text = mission_navigation_segment(
                navigation_log(log_dir, sys_id).read_text(
                    encoding="utf-8",
                    errors="replace",
                )
            )
            episodes = parse_terminal_command_episodes(
                log_dir / f"uav_{sys_id}_navigation_debug.csv"
            )
        except (OSError, UnicodeError, ValueError) as error:
            errors.append(
                f"vehicle {sys_id} has invalid terminal-target evidence: {error}"
            )
            continue
        errors.extend(_vehicle_binding_errors(
            sys_id,
            text,
            episodes,
            expected,
            approved_tasks.get(sys_id),
        ))
    return errors


def _vehicle_binding_errors(
    sys_id: int,
    navigation_text: str,
    episodes: list,
    expected: tuple[float, float],
    approved_task: int | None,
) -> list[str]:
    errors: list[str] = []
    snap_at = navigation_text.find("SNAP(VISION-NAV")
    selection_window = (
        navigation_text if snap_at < 0 else navigation_text[:snap_at]
    )
    selections = list(_TARGET_RE.finditer(selection_window))
    confirming = list(_CONFIRMING_RE.finditer(selection_window))
    confirmed = list(_CONFIRMED_RE.finditer(selection_window))
    if approved_task is None:
        errors.append(
            f"vehicle {sys_id} has no unique audited approval task identity"
        )
    if len(confirming) != 1:
        errors.append(
            f"vehicle {sys_id} expected one CONFIRMING target before SNAP, "
            f"found {len(confirming)}"
        )
    if len(confirmed) != 1:
        errors.append(
            f"vehicle {sys_id} expected one ground-station confirmed target "
            f"before SNAP, found {len(confirmed)}"
        )
    if len(selections) != 1:
        errors.append(
            f"vehicle {sys_id} expected one selected TARGET before SNAP, "
            f"found {len(selections)}"
        )
        return errors
    selected = selections[0]
    selected_identity = (
        int(selected.group("task")),
        int(selected.group("obj")),
    )
    identity_chain = {
        "approval": approved_task,
        "CONFIRMING": (
            int(confirming[0].group("task")) if len(confirming) == 1 else None
        ),
        "confirmed": (
            int(confirmed[0].group("task")) if len(confirmed) == 1 else None
        ),
        "TARGET": selected_identity[0],
    }
    known_identities = {
        task for task in identity_chain.values() if task is not None
    }
    if len(known_identities) != 1 or len(known_identities) != len(
        set(identity_chain.values())
    ):
        errors.append(
            f"vehicle {sys_id} local target identity chain differs: "
            + ", ".join(
                f"{stage}=T{task}" if task is not None else f"{stage}=missing"
                for stage, task in identity_chain.items()
            )
        )
    if len(episodes) != 1:
        return [
            f"vehicle {sys_id} expected one terminal command episode for "
            f"selected T{selected_identity[0]}/obj{selected_identity[1]}, "
            f"found {len(episodes)}"
        ]
    commands = episodes[0]
    identities = {(command.task, command.obj) for command in commands}
    if identities != {selected_identity}:
        errors.append(
            f"vehicle {sys_id} terminal command identity {sorted(identities)} "
            f"does not match selected T{selected_identity[0]}/"
            f"obj{selected_identity[1]}"
        )
    issued_count = sum(command.issued for command in commands)
    episode_text = selection_window[selected.end():]
    locations = [
        (float(match.group("lat")), float(match.group("lon")))
        for match in _TARGET_LOCATION_RE.finditer(episode_text)
    ]
    if issued_count == 0:
        errors.append(f"vehicle {sys_id} selected target has no issued commands")
    if len(locations) != issued_count:
        errors.append(
            f"vehicle {sys_id} selected target has {issued_count} issued "
            f"commands but {len(locations)} truth-target samples"
        )
    wrong = [
        _distance_m(location, expected)
        for location in locations
        if _distance_m(location, expected) > MAX_TARGET_COORD_ERROR_M
    ]
    if wrong:
        errors.append(
            f"vehicle {sys_id} terminal truth target differs from assigned "
            f"coordinate by up to {max(wrong):.3f}m"
        )
    return errors


def _distance_m(
    left: tuple[float, float],
    right: tuple[float, float],
) -> float:
    if not _valid_coordinate(left) or not _valid_coordinate(right):
        return math.inf
    radius_m = 6_371_000.0
    phi_a, phi_b = math.radians(left[0]), math.radians(right[0])
    delta_phi = phi_b - phi_a
    delta_lon = math.radians(right[1] - left[1])
    haversine = (
        math.sin(delta_phi / 2.0) ** 2
        + math.cos(phi_a) * math.cos(phi_b) * math.sin(delta_lon / 2.0) ** 2
    )
    return 2.0 * radius_m * math.asin(min(1.0, math.sqrt(haversine)))


def _valid_coordinate(coordinate: tuple[float, float]) -> bool:
    lat, lon = coordinate
    return (
        math.isfinite(lat)
        and math.isfinite(lon)
        and -90.0 <= lat <= 90.0
        and -180.0 <= lon <= 180.0
    )


__all__ = ["MAX_TARGET_COORD_ERROR_M", "terminal_target_binding_errors"]
