"""Scheduled faults: which ones a run injects, drawn from its seed (step 7).

``docs/design/deterministic_dev_sim.md``, section 2 (Faults) and section 4. A
fault is a swarm-drawn model of the sim profile (``sim_profile``): the profile
gives its ``start_s`` and ``duration_s`` ranges, and the target comes from its
own stream. At most one fault of each kind per run, and none in a one-UAV fleet,
whose gates know no peer. ``coordinator_faults`` runs the plans.

- ``link_drop``: one radio endpoint (a SITL's serial1 or the backend's router
  device) hears nothing and is heard by nobody.
- ``radio_partition``: one UAV and its peers hear nothing of each other; the
  ground station's link stays up.
- ``late_heartbeat``: one NavPy freezes, long enough to pass the product's
  liveness timeouts: the 5 s peer silence and companion window.
- ``navpy_restart``: the coordinator stops one NavPy and respawns it with
  ``generation + 1``.

A fault's clock starts at its anchor, the first NavPy line of a mission phase:
the whole timeline moves with the wind and the line rate, so a fixed sim time
would land in a different phase per seed. Setup before the launch (parameters,
mission upload) is outside every fault's window, and a partition before the
first check-in would cut nothing.

Everything is healed at the latest at the fence, the first confirm request
(:data:`FENCE_MARKER`), and nothing starts after it. The mission's last phase,
guidance, starts only when the operator answers, which is the eval's
confirmation delay later, so each recovery deadline falls before it (the gates
check that on every run).

A restart has no early end, so it may start only until every UAV has checked in.
The first detection comes tens of seconds after that, so the approval exchange
never meets a NavPy coming back (the gates check that on every run too).
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from navpy.sim.lockstep.sim_profile import Realization, model_stream

LINK_DROP = "link_drop"
RADIO_PARTITION = "radio_partition"
LATE_HEARTBEAT = "late_heartbeat"
NAVPY_RESTART = "navpy_restart"
FAULT_KINDS = (LINK_DROP, RADIO_PARTITION, LATE_HEARTBEAT, NAVPY_RESTART)

#: A link drop's target, besides a SITL (the router's endpoint names).
BACKEND_TARGET = "backend"
#: Launch started: the first vehicle is armed and in its mission (``navigation_decision``).
LAUNCH_MARKER = "ONHOLD: WAYPOINT"
#: The first swarm presence (``task_messaging``); before it no NAVLINK crosses between UAVs.
PRESENCE_MARKER = "CheckIn"
#: The first confirm request (``eval_gcs_demo_slice_gates.WORKFLOW_MARKERS``).
FENCE_MARKER = "Sending confirm request for P"
#: Why a planned fault did not run: the fence came, or every UAV had checked in (a restart only).
SKIP_AT_FENCE = "fence"
SKIP_AT_PRESENCE = "presence"
_ANCHORS: Mapping[str, str] = {LINK_DROP: LAUNCH_MARKER, RADIO_PARTITION: PRESENCE_MARKER,
                               LATE_HEARTBEAT: LAUNCH_MARKER, NAVPY_RESTART: LAUNCH_MARKER}

#: After a cut or a freeze heals, how long a peer or the ground station may stay unheard:
#: one heartbeat TTL, the product's own silence limit (``PEER_SILENCE_EXPIRY_S``).
PEER_RECOVERY_S = 5.0
#: After a respawn, how long the new generation may take to be back in service and to meet its
#: peers: measured at most 4.3 s to its first flow line, then one heartbeat TTL, the rest margin.
RESTART_RECOVERY_S = 15.0
_RECOVERY: Mapping[str, float] = {LINK_DROP: PEER_RECOVERY_S, RADIO_PARTITION: PEER_RECOVERY_S,
                                  LATE_HEARTBEAT: PEER_RECOVERY_S, NAVPY_RESTART: RESTART_RECOVERY_S}


@dataclass(frozen=True)
class FaultPlan:
    kind: str
    #: ``sitl<i>`` or ``backend`` for a link drop, else ``uav<i>``.
    target: str
    #: From the anchor to the start.
    start_s: float
    #: From the start to the heal; a restart heals when it respawned.
    duration_s: float

    @property
    def id(self) -> str:
        return f"{self.kind}:{self.target}"

    @property
    def anchor(self) -> str:
        return _ANCHORS[self.kind]

    @property
    def recovery_s(self) -> float:
        return _RECOVERY[self.kind]

    @property
    def ordinal(self) -> int:
        """The target UAV's sysid, SITL and NavPy ordinal (``FleetLayout``)."""
        return int(self.target.removeprefix("uav").removeprefix("sitl"))

    def to_json(self) -> dict[str, object]:
        return {"id": self.id, "kind": self.kind, "target": self.target, "anchor": self.anchor,
                "start_s": self.start_s, "duration_s": self.duration_s, "recovery_s": self.recovery_s}


def _targets(kind: str, fleet_size: int) -> list[str]:
    if kind == LINK_DROP:
        return [*(f"sitl{i}" for i in range(1, fleet_size + 1)), BACKEND_TARGET]
    return [f"uav{i}" for i in range(1, fleet_size + 1)]


def _target(seed: int, kind: str, fleet_size: int) -> str:
    targets = _targets(kind, fleet_size)
    return targets[math.floor(model_stream(seed, kind, "target").random() * len(targets))]


def fault_plans(realization: Realization, fleet_size: int) -> tuple[FaultPlan, ...]:
    """The run's faults, in :data:`FAULT_KINDS` order."""
    if fleet_size < 2:
        return ()
    plans = []
    for kind in FAULT_KINDS:
        draw = realization.model(kind)
        if draw is not None and draw.on:
            plans.append(FaultPlan(kind, _target(realization.seed, kind, fleet_size),
                                   float(draw.params["start_s"]),
                                   float(draw.params.get("duration_s", 0.0))))
    return tuple(plans)


__all__ = [
    "BACKEND_TARGET", "FAULT_KINDS", "FENCE_MARKER", "FaultPlan", "LATE_HEARTBEAT", "LAUNCH_MARKER",
    "LINK_DROP", "NAVPY_RESTART", "PEER_RECOVERY_S", "PRESENCE_MARKER", "RADIO_PARTITION",
    "RESTART_RECOVERY_S", "SKIP_AT_FENCE", "SKIP_AT_PRESENCE", "fault_plans",
]
