"""Pure Vision guard for the sim dock contact sensor (physical_docking.md).

The contact sensor compares the aircraft's telemetry position with the
dock's true position. Only one correlated DOCKED event may leave it: no
position, range or contact distance may reach frames, the navigation law or
the command path.
"""

from __future__ import annotations

import ast
from dataclasses import fields
from pathlib import Path

import pytest

from navpy.core.ports.dock_contact import DockContactEvent
from navpy.core.ports.frame import FinalApproachVisionFrame

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src/navpy"
CONTACT_PORT = "navpy.core.ports.dock_contact"
# Event values that carry truth geometry or the aircraft clock.
CONTACT_VALUES = frozenset(
    {"closest_3d_m", "closest_h_m", "closest_v_m", "aircraft_time_s"}
)
# The only readers of those values: the producer, its port, the DOCKED
# evidence row and the operator status line. None of them commands.
CONTACT_VALUE_READERS = frozenset({
    "src/navpy/sim/world/dock_contact_sensor.py",
    # The producer's closest-pass helper (gcs_sim_closest_approach.md).
    "src/navpy/sim/world/closest_approach_tracker.py",
    "src/navpy/sim/world/dock_contact_feed.py",
    "src/navpy/core/ports/dock_contact.py",
    "src/navpy/core/approach/vision_nav/diagnostics.py",
    "src/navpy/core/approach/dock_contact_episode.py",
    # The DOCKED state line, after the command fence (docked_state.md).
    "src/navpy/core/flow/nav_transition.py",
})
# The final-approach command path: frames, projection, admission, the law
# and every file that can reach the actuator.
COMMAND_PATH = (
    "src/navpy/core/ports/frame.py",
    "src/navpy/core/approach/vision_nav/frame_projection.py",
    "src/navpy/core/approach/vision_nav/ingress.py",
    "src/navpy/core/approach/vision_nav/confirmation.py",
    "src/navpy/core/approach/vision_nav/command_transaction.py",
    "src/navpy/core/approach/vision_nav/command_executor.py",
    "src/navpy/core/approach/vision_nav/command_hold.py",
    "src/navpy/core/approach/vision_nav/command_freshness.py",
    "src/navpy/core/approach/vision_nav/visual_pass.py",
)
LAW_DIR = SRC / "plugins/laws"


def _relative(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def _imports(tree: ast.AST) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    return names


def _contact_value_reads(tree: ast.AST) -> list[str]:
    return [
        f"{node.attr}@{node.lineno}"
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute) and node.attr in CONTACT_VALUES
    ]


def _command_path_files() -> list[Path]:
    files = [ROOT / relative for relative in COMMAND_PATH]
    files += sorted(LAW_DIR.rglob("*.py"))
    assert all(path.is_file() for path in files)
    return files


def test_the_event_carries_its_values_under_the_guarded_names():
    assert {field.name for field in fields(DockContactEvent)} >= CONTACT_VALUES


def test_frames_carry_no_contact_dock_or_distance_field():
    names = {field.name for field in fields(FinalApproachVisionFrame)}
    assert not {
        name for name in names
        if any(word in name for word in ("dock", "contact", "closest", "dist"))
    }


def test_command_path_never_imports_the_contact_port_or_the_simulator():
    hits = []
    for path in _command_path_files():
        imports = _imports(ast.parse(path.read_text("utf-8")))
        hits += [
            f"{_relative(path)}: {name}" for name in sorted(imports)
            if name == CONTACT_PORT or name.startswith("navpy.sim")
        ]
    assert not hits, hits


def test_contact_values_are_read_only_by_evidence_and_the_producer():
    hits = []
    for path in sorted(SRC.rglob("*.py")):
        relative = _relative(path)
        if relative in CONTACT_VALUE_READERS:
            continue
        hits += [
            f"{relative}:{hit}"
            for hit in _contact_value_reads(ast.parse(path.read_text("utf-8")))
        ]
    assert not hits, hits


def test_command_path_never_reads_the_docked_state_gate():
    """The DOCKED state's contact and the failed-dock terminal (missed_dock.md)
    are read after the fence, never a command input."""
    gate = {"own_dock_contact", "dock_terminal", "dock_outcome", "terminal_snapshot"}
    hits = []
    for path in _command_path_files():
        hits += [
            f"{_relative(path)}:{node.attr}@{node.lineno}"
            for node in ast.walk(ast.parse(path.read_text("utf-8")))
            if isinstance(node, ast.Attribute) and node.attr in gate
        ]
    assert not hits, hits


def test_the_scanner_catches_a_contact_value_reaching_a_command():
    bad = ast.parse(
        "from navpy.core.ports.dock_contact import DockContactEvent\n"
        "def command(event):\n"
        "    return event.closest_3d_m * 2.0\n"
    )
    assert CONTACT_PORT in _imports(bad)
    assert _contact_value_reads(bad) == ["closest_3d_m@3"]


# The closest-approach display record (gcs_sim_closest_approach.md): sim truth
# geometry for the GCS map. Only its producer, its composition and the GCS
# backend reader may import it, and the command path may not read its values
# by attribute, by literal subscript or by ``.get``.
RECORD_MODULE = "navpy.sim.world.closest_approach_record"
RECORD_IMPORTERS = frozenset({
    "src/navpy/sim/world/closest_approach_record.py",
    "src/navpy/sim/world/closest_approach_tracker.py",
    "src/navpy/app/perception/sim_detector_factory.py",
    "src/gcs/backend/sim_closest_approach_store.py",
})
# Record attributes (nested parts and their values) and flat wire keys.
RECORD_VALUES = frozenset({
    "point", "dock", "d3_m", "h_m", "v_m", "alt_rel_m", "alt_amsl_m",
    "dock_lat", "dock_lon", "dock_alt_rel_m", "dock_alt_amsl_m",
})


def _absolute_module(node: ast.ImportFrom, package: str) -> str:
    """The module an ImportFrom names, relative imports resolved."""
    if not node.level:
        return node.module or ""
    parts = package.split(".")
    base = parts[: len(parts) - (node.level - 1)] if node.level > 1 else parts
    return ".".join([*base, *([node.module] if node.module else [])])


def _record_imports(tree: ast.AST, package: str = "") -> list[str]:
    """``package`` is the scanned module's package (relative imports)."""
    hits = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            hits += [a.name for a in node.names if a.name == RECORD_MODULE]
        elif isinstance(node, ast.ImportFrom):
            module = _absolute_module(node, package)
            if module == RECORD_MODULE:
                hits.append(module)
            elif module == RECORD_MODULE.rpartition(".")[0]:
                hits += [
                    f"{module}.{a.name}" for a in node.names
                    if a.name == RECORD_MODULE.rpartition(".")[2]
                ]
    return hits


def _package(path: Path) -> str:
    return ".".join(path.relative_to(ROOT / "src").parent.parts)


def _record_value_reads(tree: ast.AST) -> list[str]:
    hits = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and node.attr in RECORD_VALUES:
            hits.append(f"{node.attr}@{node.lineno}")
        elif (
            isinstance(node, ast.Subscript)
            and isinstance(node.slice, ast.Constant)
            and node.slice.value in RECORD_VALUES
        ):
            hits.append(f"[{node.slice.value}]@{node.lineno}")
        elif (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "get"
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value in RECORD_VALUES
        ):
            hits.append(f"get({node.args[0].value})@{node.lineno}")
    return hits


def test_the_record_carries_its_geometry_under_the_guarded_names():
    from navpy.sim.world.closest_approach_record import (
        ApproachPoint,
        ClosestApproach,
        MissDistance,
    )

    attributes = {
        field.name
        for record in (ClosestApproach, ApproachPoint, MissDistance)
        for field in fields(record)
    }
    wire = {"dock_lat", "dock_lon", "dock_alt_rel_m", "dock_alt_amsl_m"}
    assert attributes | wire >= RECORD_VALUES


def test_only_the_producer_and_the_gcs_reader_import_the_record():
    hits = []
    for base in (SRC, ROOT / "src/gcs/backend"):
        for path in sorted(base.rglob("*.py")):
            relative = _relative(path)
            if relative in RECORD_IMPORTERS:
                continue
            hits += [
                f"{relative}: {name}"
                for name in _record_imports(
                    ast.parse(path.read_text("utf-8")), _package(path),
                )
            ]
    assert not hits, hits


def test_command_path_never_reads_record_values():
    hits = []
    for path in _command_path_files():
        hits += [
            f"{_relative(path)}:{hit}"
            for hit in _record_value_reads(ast.parse(path.read_text("utf-8")))
        ]
    assert not hits, hits


@pytest.mark.parametrize(
    "source, expected",
    [
        ("def f(r):\n    return r.d3_m\n", ["d3_m@2"]),
        ("def f(r):\n    return r['h_m']\n", ["[h_m]@2"]),
        ("def f(r):\n    return r.get('dock_lat', 0.0)\n", ["get(dock_lat)@2"]),
    ],
)
def test_the_scanner_catches_record_values_in_every_form(source, expected):
    assert _record_value_reads(ast.parse(source)) == expected


@pytest.mark.parametrize(
    "source",
    [
        "import navpy.sim.world.closest_approach_record\n",
        "from navpy.sim.world.closest_approach_record import ClosestApproach\n",
        "from navpy.sim.world import closest_approach_record\n",
    ],
)
def test_the_scanner_catches_every_record_import(source):
    assert _record_imports(ast.parse(source))


@pytest.mark.parametrize(
    "source",
    [
        "from ....sim.world.closest_approach_record import ClosestApproach\n",
        "from ....sim.world import closest_approach_record\n",
    ],
)
def test_the_scanner_resolves_relative_record_imports(source):
    package = "navpy.core.approach.vision_nav"  # frame_projection.py
    assert _record_imports(ast.parse(source), package)
    assert not _record_imports(ast.parse("from .closest_approach_record import X\n"), package)
