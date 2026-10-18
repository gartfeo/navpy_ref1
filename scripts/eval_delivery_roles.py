"""Resolve the delivery run's roles (e4, e6).

- **Offerer:** the fixture's offerer zone (A sees d1 and d2).
- **Winner:** the helper of the run's one logical APPLIED assignment in the
  GCS backend log, which must name the offerer as owner.
- **Home:** the remaining UAV.

Either non-offerer may win: the swarm picks by reported ETA, not by zone, so
the run decides (docs/design/delivery_eval_scenario.md, "The scenario"). The
roles come from the backend's assignment evidence, never from a NavPy log
marker, so a winner that fails to log its approach is not taken for home.
"""

from __future__ import annotations

from collections.abc import Collection, Mapping, Sequence
from dataclasses import dataclass, field

from scripts.eval_gcs_demo_ports import JsonValue

OFFERER, WINNER, HOME = "offerer", "winner", "home"
HOME_DOCK_MARKER = "Default delivery hub navigation:"
#: (owner, helper, auction task) of one APPLIED assignment.
Applied = tuple[int, int, int]


@dataclass(frozen=True)
class DeliveryRoles:
    #: sysid -> role, for every UAV whose role resolved.
    roles: dict[int, str]
    #: sysid -> the world name of its own home dock (zone order).
    home_docks: dict[int, str]
    #: The one logical assignment, when it resolved.
    applied: Applied | None = None
    errors: list[str] = field(default_factory=list)

    def sys_id(self, role: str) -> int | None:
        return next((sys_id for sys_id, name in self.roles.items() if name == role), None)


def resolve_roles(
    sys_ids: Sequence[int],
    expected: Mapping[str, JsonValue],
    applied: Collection[Applied],
) -> DeliveryRoles:
    """*sys_ids* in zone order; *expected* is the fixture's ``expected`` block;
    *applied* holds every distinct APPLIED tuple of the run."""
    home_names = list(expected["home_dock_by_zone"])  # type: ignore[arg-type]
    if len(home_names) != len(sys_ids):
        raise ValueError(f"{len(sys_ids)} UAVs but {len(home_names)} home docks")
    home_docks = dict(zip(sys_ids, (str(name) for name in home_names)))
    offerer = sys_ids[int(expected["offerer_zone_index"])]  # type: ignore[arg-type]
    roles = {offerer: OFFERER}
    if len(applied) != 1:
        return DeliveryRoles(roles, home_docks, None, [
            f"expected exactly one logical APPLIED assignment, found {sorted(applied)}"
        ])
    (owner, helper, task), = applied
    errors = []
    if owner != offerer:
        errors.append(f"APPLIED owner {owner} is not the offerer {offerer}")
    if helper == offerer or helper not in sys_ids:
        errors.append(f"APPLIED helper {helper} is not another UAV of {list(sys_ids)}")
    if errors:
        return DeliveryRoles(roles, home_docks, None, errors)
    roles[helper] = WINNER
    for sys_id in sys_ids:
        roles.setdefault(sys_id, HOME)
    return DeliveryRoles(roles, home_docks, (owner, helper, task))


__all__ = [
    "HOME",
    "HOME_DOCK_MARKER",
    "OFFERER",
    "WINNER",
    "Applied",
    "DeliveryRoles",
    "resolve_roles",
]
