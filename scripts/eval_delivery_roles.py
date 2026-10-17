"""Resolve the delivery run's roles from the NavPy logs (e4).

- **Offerer:** the fixture's offerer zone (A sees d1 and d2).
- **Winner:** the other UAV that flew a peer task ("Peer navigation started").
- **Home:** the other UAV that flew to its home dock and no peer task.

Either non-offerer may win: the swarm picks by reported ETA, not by zone, so
the run decides (docs/design/delivery_eval_scenario.md, "The scenario"). The
checks are by role, never by sysid; the role gates are e6.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from scripts.eval_gcs_demo_api import navigation_log_path
from scripts.eval_gcs_demo_ports import JsonValue
from scripts.eval_gcs_demo_role_gates import PEER_APPROACH_MARKER

OFFERER, WINNER, HOME = "offerer", "winner", "home"
HOME_DOCK_MARKER = "Default delivery hub navigation:"


@dataclass(frozen=True)
class DeliveryRoles:
    #: sysid -> role, for every UAV whose role resolved.
    roles: dict[int, str]
    #: sysid -> the world name of its own home dock (zone order).
    home_docks: dict[int, str]
    errors: list[str] = field(default_factory=list)

    def sys_id(self, role: str) -> int | None:
        return next((sys_id for sys_id, name in self.roles.items() if name == role), None)


def resolve_roles(
    sys_ids: Sequence[int],
    navigation_texts: Mapping[int, str],
    expected: Mapping[str, JsonValue],
) -> DeliveryRoles:
    """*sys_ids* in zone order; *expected* is the fixture's ``expected`` block."""
    home_names = list(expected["home_dock_by_zone"])  # type: ignore[arg-type]
    if len(home_names) != len(sys_ids):
        raise ValueError(f"{len(sys_ids)} UAVs but {len(home_names)} home docks")
    home_docks = dict(zip(sys_ids, (str(name) for name in home_names)))
    offerer = sys_ids[int(expected["offerer_zone_index"])]  # type: ignore[arg-type]
    errors: list[str] = []
    flew_peer = {sys_id for sys_id in sys_ids if PEER_APPROACH_MARKER in navigation_texts[sys_id]}
    homed = {sys_id for sys_id in sys_ids if HOME_DOCK_MARKER in navigation_texts[sys_id]}
    roles = {offerer: OFFERER}
    if offerer in flew_peer:
        errors.append(f"offerer {offerer} flew a peer task")
    others = [sys_id for sys_id in sys_ids if sys_id != offerer]
    winners = [sys_id for sys_id in others if sys_id in flew_peer]
    homes = [sys_id for sys_id in others if sys_id in homed and sys_id not in flew_peer]
    for role, found in ((WINNER, winners), (HOME, homes)):
        if len(found) == 1:
            roles[found[0]] = role
        else:
            errors.append(f"expected exactly one {role} among {others}, found {found}")
    return DeliveryRoles(roles, home_docks, errors)


def delivery_roles(
    log_dir: Path,
    sys_ids: Sequence[int],
    expected: Mapping[str, JsonValue],
) -> DeliveryRoles:
    texts = {
        sys_id: navigation_log_path(log_dir, sys_id).read_text(
            encoding="utf-8", errors="replace",
        )
        for sys_id in sys_ids
    }
    return resolve_roles(sys_ids, texts, expected)


__all__ = [
    "HOME",
    "HOME_DOCK_MARKER",
    "OFFERER",
    "WINNER",
    "DeliveryRoles",
    "delivery_roles",
    "resolve_roles",
]
