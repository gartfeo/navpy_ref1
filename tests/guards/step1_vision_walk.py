"""The pure-vision walk (docs/design/step1_separation.md, "Pure-vision guard").

From each flagged core/plugins/products file, follow runtime imports through
core, plugins and products files. Reaching geo, world or sim code fails unless the hop is one of
the named guard=vision edges in exceptions.tsv; a named edge no flagged file
reaches any more is stale.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from tests.guards.step1_vision_graph import Hop, VisionGraph

FORBIDDEN_HOMES = (
    "core/perception/geo/", "core/world/", "sim/", "core/approach/geo/",
    "core/skills/geo_approach/", "plugins/laws/geo/",
)
WALKABLE_HOMES = ("core/", "plugins/", "products/")


@dataclass(frozen=True)
class WalkResult:
    roots: tuple[str, ...]                  # flagged core/plugins/products files walked from
    violations: tuple[str, ...]             # one path per unnamed hop into forbidden code
    reached: frozenset[tuple[str, str]]     # named (file, imports) hops some root reaches


def walkable(home: str) -> bool:
    return home.startswith(WALKABLE_HOMES) and not home.startswith(FORBIDDEN_HOMES)


def walk(graph: VisionGraph, named: frozenset[tuple[str, str]]) -> WalkResult:
    roots = tuple(sorted(
        key for key, node in graph.nodes.items() if node.flagged and walkable(node.home)))
    violations: dict[tuple[str, int, str], str] = {}
    reached: set[tuple[str, str]] = set()
    for root in roots:
        via: dict[str, tuple[str, Hop] | None] = {root: None}
        queue = deque([root])
        while queue:
            key = queue.popleft()
            for hop in graph.hops.get(key, ()):
                pair = (graph.nodes[key].home, graph.nodes[hop.target].home)
                if pair in named:
                    reached.add(pair)
                elif pair[1].startswith(FORBIDDEN_HOMES):
                    violations.setdefault((key, hop.line, hop.target), _path(via, key, hop))
                elif hop.target not in via and walkable(pair[1]):
                    via[hop.target] = (key, hop)
                    queue.append(hop.target)
    return WalkResult(roots, tuple(violations[k] for k in sorted(violations)), frozenset(reached))


def _path(via: dict[str, tuple[str, Hop] | None], key: str, hop: Hop) -> str:
    """root:line -> ... -> key:line -> forbidden target ((fn): function-level
    import; key(pkg): the package init loading key runs first)."""
    steps = [hop.target]
    step: tuple[str, Hop] | None = (key, hop)
    while step is not None:
        source, outgoing = step
        steps.append(f"{source}(pkg)" if outgoing.where == "package" else
                     f"{source}:{outgoing.line}{'(fn)' if outgoing.where == 'function' else ''}")
        step = via[source]
    return " -> ".join(reversed(steps))
