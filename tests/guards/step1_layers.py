"""File-to-file import graph of src/navpy and the step-1 layer rules over it.

Edges are what the code imports today: `from m import n` lands on submodule
m.n when one exists, otherwise on m. A file still in the old tree is in layer
"old", which the rules do not judge until it lands (step1_manifest.allowed).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping

from tests.guards.step1_sources import ImportRef, SourceFile, plan_module

sm = plan_module()


@dataclass(frozen=True)
class Edge:
    importer: str          # path below src/navpy
    target: str            # path below src/navpy, or "gcs:<module>" for src/gcs
    line: int
    where: str
    type_checking: bool


def is_navpy(module: str) -> bool:
    return module == "navpy" or module.startswith("navpy.")


def is_gcs(module: str) -> bool:
    return module == "gcs" or module.startswith("gcs.")


def module_paths(sources: Mapping[str, SourceFile]) -> dict[str, str]:
    """Dotted module -> path below src/navpy."""
    return {source.module: source.path for source in sources.values()}


def resolve_ref(ref: ImportRef, modules: Mapping[str, str]) -> str | None:
    """File the import loads; "gcs:<module>" for src/gcs; None if unresolved."""
    if is_gcs(ref.module):
        return sm.GCS_HOME + ref.module
    if ref.name not in (None, "*"):
        submodule = modules.get(f"{ref.module}.{ref.name}")
        if submodule is not None:
            return submodule
    return modules.get(ref.module)


def navpy_edges(sources: Mapping[str, SourceFile]) -> list[Edge]:
    modules = module_paths(sources)
    edges: list[Edge] = []
    for source in sources.values():
        for ref in source.refs:
            if not (is_navpy(ref.module) or is_gcs(ref.module)):
                continue
            target = resolve_ref(ref, modules)
            if target is not None and target != source.path:
                edges.append(Edge(source.path, target, ref.line, ref.where, ref.type_checking))
    return edges


def wrong_way(edge: Edge) -> bool:
    return not sm.allowed(sm.layer_of(edge.importer), sm.layer_of(edge.target))


def unlisted_wrong_way(edges: Iterable[Edge], listed: set[tuple[str, str]]) -> list[str]:
    """Wrong-way imports between landed files that exceptions.tsv does not list."""
    return sorted(
        f"{edge.importer}:{edge.line} -> {edge.target} "
        f"({sm.layer_of(edge.importer)} -> {sm.layer_of(edge.target)}, {edge.where}"
        f"{', TYPE_CHECKING' if edge.type_checking else ''})"
        for edge in edges
        if wrong_way(edge) and (edge.importer, edge.target) not in listed
    )


def stale_exceptions(edges: Iterable[Edge], listed: set[tuple[str, str]],
                     existing: set[str]) -> list[str]:
    """Listed pairs whose two files have landed but no longer import each other."""
    present = {(edge.importer, edge.target) for edge in edges}
    return sorted(
        f"{importer} -> {target}"
        for importer, target in listed
        if importer in existing and target in existing and (importer, target) not in present
    )
