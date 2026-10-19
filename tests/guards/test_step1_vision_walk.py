"""Pure-vision guard for refactor step 1 (docs/design/step1_separation.md).

The flagged set is every manifest row with vision_path=y plus the flagged
split parts of splits.json, wherever each file is now. From each flagged
core/plugins/products file, runtime imports are followed through
core/plugins/products files;
reaching core/perception/geo, core/world, sim, core/approach/geo,
core/skills/geo_approach or plugins/laws/geo fails unless the hop is a named
guard=vision edge of exceptions.tsv, and a named edge nothing reaches fails.
"""

from __future__ import annotations

import ast
from functools import cache

from tests.guards.step1_sources import (
    REPO_ROOT,
    SourceFile,
    import_refs,
    module_of,
    navpy_sources,
    package_of,
    plan_module,
)
from tests.guards.step1_vision_graph import VisionGraph, build_vision_graph
from tests.guards.step1_vision_walk import WalkResult, walk

sm = plan_module()
ROWS = sm.load_manifest(REPO_ROOT)
SPLITS = sm.load_splits(REPO_ROOT)
NAMED = frozenset(
    (row.file, row.imports) for row in sm.load_exceptions(REPO_ROOT) if row.guard == "vision")
DROPPED = frozenset((row.src, row.line, row.name) for row in sm.load_reexports(REPO_ROOT))
# 161 flagged rows and 8 flagged split parts, two of them sharing
# core/ports/detection.py: 168 files, 163 of them in core, plugins or products.
MIN_FLAGGED = 150


@cache
def _graph() -> VisionGraph:
    return build_vision_graph(navpy_sources(REPO_ROOT), ROWS, SPLITS, DROPPED)


@cache
def _walk() -> WalkResult:
    return walk(_graph(), NAMED)


def test_every_navpy_file_has_a_step1_home() -> None:
    assert _graph().unplaced == (), "files missing from docs/design/step1/manifest.tsv"


def test_every_flagged_file_and_part_is_located() -> None:
    rows = [row.src for row in ROWS
            if row.vision_path and sm.current_location(REPO_ROOT, row) is None]
    parts = [part.step1 for parts in SPLITS.values() for part in parts
             if part.vision_path and part.step1 not in _graph().nodes]

    assert not rows and not parts, f"flagged files not found: {rows + parts}"


def test_the_flagged_set_is_the_vision_path() -> None:
    flagged = [key for key, node in _graph().nodes.items() if node.flagged]

    assert len(flagged) >= MIN_FLAGGED
    assert len(_walk().roots) >= MIN_FLAGGED
    assert set(_walk().roots) <= set(flagged)


def test_the_delivery_flow_is_a_root() -> None:
    # Delivery's decision runs the approach and the POI review, so the walk
    # starts there too (delivery_product.md, c6).
    assert "products/delivery/flow.py" in _walk().roots


def test_flagged_files_reach_no_geo_world_or_sim_code() -> None:
    assert not _walk().violations, (
        "vision-path files reach geo, world or sim code (path: file:line hops):\n"
        + "\n".join(_walk().violations))


def test_every_named_vision_edge_is_still_reached() -> None:
    stale = sorted(NAMED - _walk().reached)

    assert not stale, (
        "no flagged file reaches these guard=vision rows of docs/design/step1/"
        "exceptions.tsv any more; remove them:\n" + "\n".join(map(str, stale)))


# --- the walk on a synthetic tree -------------------------------------------

def _source(path: str, text: str) -> SourceFile:
    tree = ast.parse(text)
    relative = f"navpy/{path}"
    return SourceFile(path, module_of(relative), tree,
                      tuple(import_refs(tree, package_of(relative))))


def _row(src: str, step1: str, vision: bool, action: str = "move") -> sm.ManifestRow:
    return sm.ManifestRow(src, step1, step1, action, vision, "")


SYNTHETIC_SOURCES = {path: _source(path, text) for path, text in {
    "__init__.py": "",
    "old/__init__.py": "from navpy.old.geo import distance\n",
    "old/geo.py": "def distance(): ...\n",
    "old/law.py": "from navpy.old import distance\n",
    "old/types.py": (
        "from navpy.old.geo import distance\n"
        "class Command: ...\n"
        "def helper():\n"
        "    return distance()\n"),
    "old/engine.py": "from navpy.old.types import Command\n",
    "old/runner.py": "from navpy.old.types import helper\n",
    "old/host.py": "from navpy.old.geo import distance\nVALUE = 1\n",
    "old/reader.py": "from navpy.old.host import VALUE\n",
    "old/typed.py": (
        "from typing import TYPE_CHECKING\n"
        "if TYPE_CHECKING:\n"
        "    from navpy.old.geo import distance\n"),
    "core/approach/landed.py": "def late():\n    from navpy.old.geo import distance\n",
}.items()}
SYNTHETIC_ROWS = (
    _row("__init__.py", "__init__.py", False, "stay"),
    _row("old/__init__.py", sm.DELETE, False, "delete"),
    _row("old/geo.py", "core/perception/geo/geo.py", False),
    _row("old/law.py", "core/approach/law.py", True),
    _row("old/types.py", "plugins/vehicle/ardupilot/types.py", False, "split"),
    _row("old/engine.py", "core/approach/engine.py", True),
    _row("old/runner.py", "core/approach/runner.py", True),
    _row("old/host.py", "core/approach/host.py", False),
    _row("old/reader.py", "core/approach/reader.py", True),
    _row("old/typed.py", "core/approach/typed.py", True),
    _row("old/landed.py", "core/approach/landed.py", True),
)
SYNTHETIC_SPLITS = {"old/types.py": (
    sm.SplitPart(("Command",), "core/ports/command.py", "core/ports/command.py", True),)}
SYNTHETIC_DROPPED = frozenset({("old/host.py", 1, "distance")})
SYNTHETIC_NAMED = frozenset({
    ("core/approach/law.py", "core/perception/geo/geo.py"),
    ("core/approach/engine.py", "core/perception/geo/geo.py"),
})


def test_the_walk_follows_facades_splits_and_landed_files() -> None:
    graph = build_vision_graph(SYNTHETIC_SOURCES, SYNTHETIC_ROWS, SYNTHETIC_SPLITS,
                               SYNTHETIC_DROPPED)
    result = walk(graph, SYNTHETIC_NAMED)

    assert graph.nodes["core/ports/command.py"].flagged
    assert graph.nodes["old/__init__.py"].home == sm.DELETE
    assert result.roots == (
        "core/approach/engine.py", "core/approach/landed.py", "core/approach/law.py",
        "core/approach/reader.py", "core/approach/runner.py", "core/approach/typed.py",
        "core/ports/command.py",
    )
    assert result.violations == (
        "core/approach/landed.py:2(fn) -> core/perception/geo/geo.py",
        "core/approach/runner.py:1 -> plugins/vehicle/ardupilot/types.py:1"
        " -> core/perception/geo/geo.py",
    )
    assert result.reached == {("core/approach/law.py", "core/perception/geo/geo.py")}


def test_the_walk_keeps_a_re_export_line_c01_does_not_strip() -> None:
    graph = build_vision_graph(SYNTHETIC_SOURCES, SYNTHETIC_ROWS, SYNTHETIC_SPLITS, frozenset())

    assert ("core/approach/reader.py:1 -> core/approach/host.py:1 -> core/perception/geo/geo.py"
            in walk(graph, SYNTHETIC_NAMED).violations)


def test_the_walk_loads_a_kept_module_an_import_names() -> None:
    """Only c01's re-exports are followed past: an import of Command from a kept
    module that imports it for itself still loads that module and its geo import."""
    sources = {**SYNTHETIC_SOURCES, **{path: _source(path, text) for path, text in {
        "old/relay.py": ("from navpy.old.geo import distance\n"
                         "from navpy.old.types import Command\n"),
        "old/consumer.py": "from navpy.old.relay import Command\n",
    }.items()}}
    rows = (*SYNTHETIC_ROWS,
            _row("old/relay.py", "core/approach/relay.py", False),
            _row("old/consumer.py", "core/approach/consumer.py", True))
    graph = build_vision_graph(sources, rows, SYNTHETIC_SPLITS, SYNTHETIC_DROPPED)

    assert ("core/approach/consumer.py:1 -> core/approach/relay.py:1 -> core/perception/geo/geo.py"
            in walk(graph, SYNTHETIC_NAMED).violations)


def test_the_walk_runs_the_package_inits_above_an_imported_module() -> None:
    """Importing pkg/leaf runs pkg/__init__ first: its geo import counts,
    unless c01 deletes or empties that init (a delete row)."""
    sources = {**SYNTHETIC_SOURCES, **{path: _source(path, text) for path, text in {
        "foo/__init__.py": "from navpy.old.geo import distance\n",
        "foo/leaf.py": "VALUE = 1\n",
        "old/foo_user.py": "from navpy.foo.leaf import VALUE\n",
    }.items()}}
    rows = (*SYNTHETIC_ROWS,
            _row("foo/__init__.py", "plugins/foo/__init__.py", False),
            _row("foo/leaf.py", "plugins/foo/leaf.py", False),
            _row("old/foo_user.py", "core/approach/foo_user.py", True))
    deleted_init = tuple(
        _row(row.src, sm.DELETE, False, "delete") if row.src == "foo/__init__.py" else row
        for row in rows)

    def violations(plan: tuple[sm.ManifestRow, ...]) -> tuple[str, ...]:
        graph = build_vision_graph(sources, plan, SYNTHETIC_SPLITS, SYNTHETIC_DROPPED)
        return walk(graph, SYNTHETIC_NAMED).violations

    assert ("core/approach/foo_user.py:1 -> plugins/foo/leaf.py(pkg)"
            " -> plugins/foo/__init__.py:1 -> core/perception/geo/geo.py") in violations(rows)
    assert not any("foo" in path for path in violations(deleted_init))
