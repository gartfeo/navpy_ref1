"""Reference model of the lockstep router (mavlink-router's forwarding rules).

Emulates mavlink-router ``d59ef94`` (the ``/usr/bin/mavlink-routerd`` that
``run_swarm.sh`` starts), ``Endpoint::accept_msg``, in the configuration
``run_swarm.sh`` gives it: ``-s 255`` (sniffer sysid 255, the GCS), no
message filters or endpoint groups, and no deduplication period (so no
dedup).

The coordinator replaces mavlink-router on the serial1 fan-in (contract in
``docs/design/lockstep_contracts.md``, Router). Rules, as mavlink-router
applies them:

- An endpoint learns every ``(sysid, compid)`` it receives from.
- No endpoint gets a message whose source ``(sysid, compid)`` it has seen
  (this excludes the endpoint it came from, and stops loops).
- Sniffer: an endpoint that has seen the sniffer sysid gets every other
  message, targeted or not.
- A message with no target, or target system 0, is a broadcast: it goes to
  every other endpoint.
- A targeted message goes only to endpoints that have seen the target system
  (and component, when it is nonzero). An unknown target is dropped and
  counted, not broadcast.
- The original packet bytes are forwarded unchanged; destinations are visited
  in the fixed endpoint order, so delivery order is deterministic.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from pymavlink.dialects.v20.ardupilotmega import MAVLink_message


@dataclass
class RouterEndpoint:
    name: str
    seen: set[tuple[int, int]] = field(default_factory=set)

    def has_seen(self, system: int, component: int) -> bool:
        if component == 0:
            return any(sys_id == system for sys_id, _comp in self.seen)
        return (system, component) in self.seen


class ReferenceRouter:
    def __init__(self, endpoint_names: list[str], sniffer_sysid: int = 255) -> None:
        self._endpoints = [RouterEndpoint(name) for name in endpoint_names]
        self._sniffer_sysid = sniffer_sysid
        self._by_name = {endpoint.name: endpoint for endpoint in self._endpoints}
        self.dropped_unknown_target = 0

    def route(self, source: str, message: MAVLink_message) -> list[str]:
        """Learn from ``message`` and return its destinations, in fixed order."""
        origin = self._by_name[source]
        src = (message.get_srcSystem(), message.get_srcComponent())
        origin.seen.add(src)
        system = getattr(message, "target_system", 0)
        component = getattr(message, "target_component", 0)
        others = [endpoint for endpoint in self._endpoints if src not in endpoint.seen]
        if system == 0:
            return [endpoint.name for endpoint in others]
        targets = [
            endpoint.name for endpoint in others
            if endpoint.has_seen(system, component)
            or endpoint.has_seen(self._sniffer_sysid, 0)
        ]
        if not targets:
            self.dropped_unknown_target += 1
        return targets


__all__ = ["ReferenceRouter", "RouterEndpoint"]
