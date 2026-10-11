"""Layer guard and ratchet for refactor step 1 (docs/design/step1_separation.md).

Once both ends of an import have landed in the new layout, the layout's import
rules apply: a wrong-way import fails unless docs/design/step1/exceptions.tsv
lists the file pair (guard=layer), and a listed pair whose two files have
landed but no longer import each other fails until its row is removed. Files
still in the old tree are not judged.
"""

from __future__ import annotations

from functools import cache

from tests.guards.step1_layers import (
    Edge,
    navpy_edges,
    stale_exceptions,
    unlisted_wrong_way,
    wrong_way,
)
from tests.guards.step1_sources import REPO_ROOT, navpy_sources, plan_module

sm = plan_module()
LISTED = {(row.file, row.imports) for row in sm.load_exceptions(REPO_ROOT) if row.guard == "layer"}


@cache
def _edges() -> tuple[Edge, ...]:
    return tuple(navpy_edges(navpy_sources(REPO_ROOT)))


def test_the_graph_is_read_from_the_tree() -> None:
    importers = {edge.importer for edge in _edges()}

    assert len(_edges()) > 3000
    assert len(importers) > 500


def test_landed_files_import_no_unlisted_wrong_way_layer() -> None:
    violations = unlisted_wrong_way(_edges(), LISTED)

    assert not violations, (
        "wrong-way imports (fix them, or list a planned one in docs/design/step1/"
        "exceptions.tsv with its fixing step):\n" + "\n".join(violations))


def test_listed_layer_exceptions_are_still_needed() -> None:
    stale = stale_exceptions(_edges(), LISTED, set(navpy_sources(REPO_ROOT)))

    assert not stale, (
        "remove these rows from docs/design/step1/exceptions.tsv, the import is gone:\n"
        + "\n".join(stale))


def test_listed_layer_exceptions_are_wrong_way_imports() -> None:
    not_wrong_way = [
        f"{importer} -> {target}" for importer, target in sorted(LISTED)
        if not wrong_way(Edge(importer, target, 0, "top", False))
    ]

    assert not not_wrong_way, "\n".join(not_wrong_way)


def test_the_ratchet_fails_new_and_stale_edges_and_skips_the_old_tree() -> None:
    listed = {("core/flow/a.py", "plugins/vehicle/ardupilot/b.py"),
              ("core/ports/c.py", "core/link/d.py")}
    edges = [
        Edge("core/flow/a.py", "plugins/vehicle/ardupilot/b.py", 3, "top", False),
        Edge("core/kernel/e.py", "core/flow/f.py", 4, "function", False),
        Edge("plugins/radio/g.py", "app/h.py", 5, "top", True),
        Edge("core/kernel/e.py", "sim/i.py", 6, "top", False),
        Edge("core/kernel/e.py", "gcs:gcs.backend.j", 7, "top", False),
        Edge("modules/nav/old.py", "app/h.py", 8, "top", False),
        Edge("core/flow/a.py", "modules/vision/old.py", 9, "top", False),
        Edge("sim/i.py", "plugins/radio/g.py", 10, "top", False),
        Edge("main.py", "core/flow/a.py", 11, "top", False),
    ]
    existing = {"core/flow/a.py", "plugins/vehicle/ardupilot/b.py", "core/ports/c.py",
                "core/link/d.py"}

    assert unlisted_wrong_way(edges, listed) == [
        "core/kernel/e.py:4 -> core/flow/f.py (core/kernel -> core/flow, function)",
        "core/kernel/e.py:6 -> sim/i.py (core/kernel -> sim, top)",
        "core/kernel/e.py:7 -> gcs:gcs.backend.j (core/kernel -> gcs, top)",
        "plugins/radio/g.py:5 -> app/h.py (plugins/radio -> app, top, TYPE_CHECKING)",
    ]
    assert stale_exceptions(edges, listed, existing) == ["core/ports/c.py -> core/link/d.py"]
    assert stale_exceptions(edges, listed, existing - {"core/link/d.py"}) == []
