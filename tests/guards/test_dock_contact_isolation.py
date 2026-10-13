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
    "src/navpy/sim/world/dock_contact_feed.py",
    "src/navpy/core/ports/dock_contact.py",
    "src/navpy/core/approach/vision_nav/diagnostics.py",
    "src/navpy/core/approach/dock_contact_episode.py",
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


def test_the_scanner_catches_a_contact_value_reaching_a_command():
    bad = ast.parse(
        "from navpy.core.ports.dock_contact import DockContactEvent\n"
        "def command(event):\n"
        "    return event.closest_3d_m * 2.0\n"
    )
    assert CONTACT_PORT in _imports(bad)
    assert _contact_value_reads(bad) == ["closest_3d_m@3"]
