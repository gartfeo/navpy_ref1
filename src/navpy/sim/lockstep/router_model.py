"""Reference model of the lockstep router (mavlink-router's forwarding rules).

Emulates mavlink-router ``d59ef94`` (the ``/usr/bin/mavlink-routerd`` that
``run_swarm.sh`` starts), ``Endpoint::accept_msg`` and
``Mainloop::route_msg``, in the configuration ``run_swarm.sh`` gives it:
``-s 255`` (sniffer sysid 255, the GCS), no endpoint groups, no
deduplication period (so no dedup), and one filter: each SITL endpoint's
``BlockSrcCompOut`` (:data:`SITL_BLOCK_SRC_COMP_OUT`,
``docs/design/radio_budget.md``).

The coordinator replaces mavlink-router on the serial1 fan-in (contract in
``docs/design/lockstep_contracts.md``, Router; ``lockstep_fleet.md``,
Router and delivery). Packets come framed by
:class:`~navpy.sim.lockstep.router_framer.RouterFramer`, so their targets
come from the router entry table, never from NavPy's dialect. Rules, as
mavlink-router applies them:

- An endpoint learns every ``(sysid, compid)`` it receives from, before the
  packet is routed.
- No endpoint gets a packet whose source ``(sysid, compid)`` it has seen
  (this excludes the endpoint it came from, and stops loops).
- Then the filter: an endpoint does not get a packet from a component in its
  ``BlockSrcCompOut``, whatever its target or id (``read_msg`` sets every
  packet's id from its header, so ``accept_msg``'s unknown-id skip never
  applies). A filtered endpoint skips the target and sniffer rules.
- No target (an id outside the entry table, or an entry without a target
  field) or target system 0 is a broadcast: every other endpoint.
- A nonzero target component goes to endpoints that have seen that
  ``(sysid, compid)``; a target component of 0 (or none) to endpoints that
  have seen the sysid.
- Sniffer: an endpoint that has seen the sniffer sysid gets every packet
  the rules above did not already send it.
- A packet no endpoint accepts is dropped. It counts as mavlink-router's
  ``msg_to_unknown`` only when no endpoint filtered it either
  (:attr:`Route.filtered`).
- The original packet bytes are forwarded unchanged; destinations are
  visited in the fixed endpoint order, so delivery order is deterministic.

``test_router_parity`` pins these rules against the binary on recorded streams.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from navpy.sim.lockstep.router_framer import NO_TARGET, RouterPacket

SNIFFER_SYSID = 255
#: Each SITL endpoint's ``BlockSrcCompOut``: no peer autopilot (1) or camera (100) traffic
#: reaches an autopilot (``docs/design/radio_budget.md``, item 2). The backend has no filter.
SITL_BLOCK_SRC_COMP_OUT = (1, 100)


class Accept(Enum):
    """``Endpoint::AcceptState``."""

    ACCEPTED = "accepted"
    FILTERED = "filtered"
    REJECTED = "rejected"


@dataclass
class RouterEndpoint:
    name: str
    seen: set[tuple[int, int]] = field(default_factory=set)
    block_src_comp_out: tuple[int, ...] = ()

    def has_system(self, system: int) -> bool:
        return any(sys_id == system for sys_id, _comp in self.seen)

    def accept(self, packet: RouterPacket, sniffer_sysid: int) -> Accept:
        """``Endpoint::accept_msg``: the loop check, the filter, then the target rules."""
        if packet.source in self.seen:
            return Accept.REJECTED
        if packet.source[1] in self.block_src_comp_out:
            return Accept.FILTERED
        system, component = packet.target
        if system in (0, NO_TARGET):
            return Accept.ACCEPTED
        if component > 0 and (system, component) in self.seen:
            return Accept.ACCEPTED
        if component in (0, NO_TARGET) and self.has_system(system):
            return Accept.ACCEPTED
        if sniffer_sysid != 0 and self.has_system(sniffer_sysid):
            return Accept.ACCEPTED
        return Accept.REJECTED


@dataclass(frozen=True)
class Route:
    #: The endpoints that get the packet, in fixed order.
    destinations: tuple[str, ...]
    #: Some endpoint filtered it: with no destination, not ``msg_to_unknown``.
    filtered: bool


class ReferenceRouter:
    def __init__(self, endpoint_names: list[str], sniffer_sysid: int = SNIFFER_SYSID,
                 block_src_comp_out: dict[str, tuple[int, ...]] | None = None) -> None:
        filters = block_src_comp_out or {}
        if not set(filters) <= set(endpoint_names):
            raise ValueError(f"filters name unknown endpoints {sorted(set(filters) - set(endpoint_names))}")
        self._endpoints = [RouterEndpoint(name, block_src_comp_out=filters.get(name, ()))
                           for name in endpoint_names]
        self._sniffer_sysid = sniffer_sysid
        self._by_name = {endpoint.name: endpoint for endpoint in self._endpoints}
        self.dropped_unknown_target = 0

    @property
    def endpoint_names(self) -> list[str]:
        return [endpoint.name for endpoint in self._endpoints]

    def route(self, source: str, packet: RouterPacket) -> Route:
        """Learn from ``packet`` and return its route; destinations in fixed order."""
        self._by_name[source].seen.add(packet.source)
        results = [(endpoint.name, endpoint.accept(packet, self._sniffer_sysid))
                   for endpoint in self._endpoints]
        route = Route(tuple(name for name, result in results if result is Accept.ACCEPTED),
                      any(result is Accept.FILTERED for _name, result in results))
        if not route.destinations and not route.filtered:
            self.dropped_unknown_target += 1
        return route


__all__ = ["Accept", "ReferenceRouter", "Route", "RouterEndpoint", "SITL_BLOCK_SRC_COMP_OUT",
           "SNIFFER_SYSID"]
