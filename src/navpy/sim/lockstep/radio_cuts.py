"""Which radio paths scheduled faults cut now (``coordinator_faults``).

The router forwards whole packets, so a cut is whole packets too
(``RoutedPorts._route``): a packet from a cut source ends before the router sees
it, and a copy on a cut path ends when it is made. Bytes queued toward a SITL
before the cut still arrive, as a radio's own buffer does. A path is
``(source endpoint, destination endpoint)`` (``router_endpoints``). With no cut,
which is every fault-free run, nothing changes.
"""

from __future__ import annotations

from collections.abc import Iterable


class RadioCuts:
    def __init__(self) -> None:
        #: Each active fault's id, in start order, and its cut sources and paths.
        self._cuts: dict[str, tuple[frozenset[str], frozenset[tuple[str, str]]]] = {}

    def cut(self, fault: str, sources: Iterable[str], paths: Iterable[tuple[str, str]]) -> None:
        self._cuts[fault] = (frozenset(sources), frozenset(paths))

    def heal(self, fault: str) -> None:
        del self._cuts[fault]

    def source(self, endpoint: str) -> str | None:
        """The fault that cuts everything ``endpoint`` sends, if any."""
        return next((fault for fault, (sources, _paths) in self._cuts.items() if endpoint in sources), None)

    def path(self, source: str, destination: str) -> str | None:
        """The fault that cuts this copy, if any."""
        return next((fault for fault, (_sources, paths) in self._cuts.items()
                     if (source, destination) in paths), None)


__all__ = ["RadioCuts"]
