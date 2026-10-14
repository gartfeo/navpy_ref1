"""Reference model of the lockstep router (mavlink-router's forwarding rules).

Emulates mavlink-router ``d59ef94`` (the ``/usr/bin/mavlink-routerd`` that
``run_swarm.sh`` starts), ``Endpoint::accept_msg`` and
``Mainloop::route_msg``, in the configuration ``run_swarm.sh`` gives it:
``-s 255`` (sniffer sysid 255, the GCS), no message filters or endpoint
groups, and no deduplication period (so no dedup).

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
- No target (an id outside the entry table, or an entry without a target
  field) or target system 0 is a broadcast: every other endpoint.
- A nonzero target component goes to endpoints that have seen that
  ``(sysid, compid)``; a target component of 0 (or none) to endpoints that
  have seen the sysid.
- Sniffer: an endpoint that has seen the sniffer sysid gets every packet
  the rules above did not already send it.
- A packet no endpoint accepts is dropped and counted (mavlink-router's
  ``msg_to_unknown``), never broadcast.
- The original packet bytes are forwarded unchanged; destinations are
  visited in the fixed endpoint order, so delivery order is deterministic.

``test_router_parity`` pins these rules against the binary on recorded streams.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from navpy.sim.lockstep.router_framer import NO_TARGET, RouterPacket

SNIFFER_SYSID = 255


@dataclass
class RouterEndpoint:
    name: str
    seen: set[tuple[int, int]] = field(default_factory=set)

    def has_system(self, system: int) -> bool:
        return any(sys_id == system for sys_id, _comp in self.seen)

    def accepts(self, packet: RouterPacket, sniffer_sysid: int) -> bool:
        """``Endpoint::accept_msg`` without filters."""
        if packet.source in self.seen:
            return False
        system, component = packet.target
        if system in (0, NO_TARGET):
            return True
        if component > 0 and (system, component) in self.seen:
            return True
        if component in (0, NO_TARGET) and self.has_system(system):
            return True
        return sniffer_sysid != 0 and self.has_system(sniffer_sysid)


class ReferenceRouter:
    def __init__(self, endpoint_names: list[str], sniffer_sysid: int = SNIFFER_SYSID) -> None:
        self._endpoints = [RouterEndpoint(name) for name in endpoint_names]
        self._sniffer_sysid = sniffer_sysid
        self._by_name = {endpoint.name: endpoint for endpoint in self._endpoints}
        self.dropped_unknown_target = 0

    @property
    def endpoint_names(self) -> list[str]:
        return [endpoint.name for endpoint in self._endpoints]

    def route(self, source: str, packet: RouterPacket) -> list[str]:
        """Learn from ``packet`` and return its destinations, in fixed order."""
        self._by_name[source].seen.add(packet.source)
        targets = [endpoint.name for endpoint in self._endpoints
                   if endpoint.accepts(packet, self._sniffer_sysid)]
        if not targets:
            self.dropped_unknown_target += 1
        return targets


__all__ = ["ReferenceRouter", "RouterEndpoint", "SNIFFER_SYSID"]
