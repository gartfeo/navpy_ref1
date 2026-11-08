"""Pure Vision guard for the HITL contact channel (hitl_final_approach.md section 9, step 26).

The contact service holds the dock pose and the plant's true pose. navpy's side of the channel (plugins/contact) may
only accept a contact for its armed episode: it reads no simulator module, no position and no pose, and no file of
the command path imports it. Only the HITL wiring opens it, and only the HITL vision profile allows it.
"""
from __future__ import annotations

import ast
import json
from pathlib import Path

from navpy.plugins.contact import hitl_contact_wire as wire

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src/navpy"
CHANNEL = "navpy.plugins.contact"
CHANNEL_DIR = SRC / "plugins/contact"
# The only navpy modules that may import the channel: the channel itself and the HITL wiring.
CHANNEL_IMPORTERS = frozenset({"src/navpy/app/perception/hitl_contact_input.py"})
WIRING_IMPORTERS = frozenset({"src/navpy/app/perception/vision_controller_composition.py"})
# What the channel may never name: simulator truth, positions, poses, ranges, the pose wire.
TRUTH_ATTRIBUTES = frozenset({
    "location", "g_loc", "position", "position_sample", "position_lla", "ground_speed", "ground_speed_ned",
    "velocity", "altitude", "relative_altitude", "attitude", "heading", "yaw", "closest_3d_m", "closest_h_m",
    "closest_v_m",
})
GEOMETRY_WORDS = ("lat", "lon", "lng", "alt", "north", "east", "height", "position", "pose", "dist", "closest",
                  "range", "heading", "yaw")
# The final-approach command path (as tests/guards/test_dock_contact_isolation.py names it) and the laws.
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
PROFILES = SRC / "core/perception/profile/vision_profiles.json"
HITL_PROFILE = "siyi_zr10_hitl_encoded"


def _relative(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def imports(tree: ast.AST) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
            names.update(f"{node.module}.{alias.name}" for alias in node.names)
    return names


def truth_reads(tree: ast.AST) -> list[str]:
    hits = [f"{node.attr}@{node.lineno}" for node in ast.walk(tree)
            if isinstance(node, ast.Attribute) and node.attr in TRUTH_ATTRIBUTES]
    hits += [f"import {name}" for name in sorted(imports(tree))
             if name.startswith("navpy.sim") or name.startswith("tools") or name.endswith(".location")
             or name.endswith("vehicle_contract")]
    hits += [f"names {node.value!r}@{node.lineno}" for node in ast.walk(tree)
             if isinstance(node, ast.Constant) and isinstance(node.value, str)
             and any(marker in node.value for marker in ("vehicle_state", "position_lla", "unreal_applied"))]
    return hits


def _channel_files() -> list[Path]:
    files = sorted(CHANNEL_DIR.rglob("*.py"))
    assert len(files) >= 4
    return files


def test_the_channel_reads_no_truth_position_or_pose():
    hits = []
    for path in _channel_files():
        hits += [f"{_relative(path)}: {hit}" for hit in truth_reads(ast.parse(path.read_text("utf-8")))]
    assert not hits, hits


def test_no_wire_kind_carries_a_coordinate_or_a_distance():
    names = {name for fields in wire.FIELDS.values() for name in fields}
    assert not [name for name in names if any(word in name for word in GEOMETRY_WORDS)]
    assert set(wire.FIELDS) == {wire.KIND_ARM, wire.KIND_DISARM, wire.KIND_CONTACT}


def _importers(prefix: str) -> set[str]:
    found = set()
    for path in sorted(SRC.rglob("*.py")):
        if path.is_relative_to(CHANNEL_DIR):
            continue
        if any(name == prefix or name.startswith(prefix + ".") for name in imports(ast.parse(path.read_text("utf-8")))):
            found.add(_relative(path))
    return found


def test_only_the_hitl_wiring_imports_the_channel():
    assert _importers(CHANNEL) == CHANNEL_IMPORTERS
    assert _importers("navpy.app.perception.hitl_contact_input") == WIRING_IMPORTERS


def test_the_command_path_and_the_laws_name_no_hitl_contact():
    files = [ROOT / relative for relative in COMMAND_PATH] + sorted((SRC / "plugins/laws").rglob("*.py"))
    hits = [f"{_relative(path)}: {word}" for path in files
            for word in ("hitl_contact", "plugins.contact", "ClosingContactSensor")
            if word in path.read_text("utf-8")]
    assert not hits, hits


def test_only_the_hitl_profile_turns_the_contact_on():
    profiles = json.loads(PROFILES.read_text(encoding="utf-8"))["profiles"]
    assert sorted(name for name, profile in profiles.items() if "hitl_contact" in profile) == [HITL_PROFILE]
    assert profiles[HITL_PROFILE]["hitl_contact"] is True


def test_only_a_pc_sitl_run_may_open_the_contact():
    from navpy.app.perception.hitl_contact_input import HITL_CONTACT_MODES

    assert HITL_CONTACT_MODES == frozenset({"pc-sitl"})


def test_the_scanner_catches_truth_in_every_form():
    bad = ast.parse(
        "from navpy.sim.world.dock_contact_sensor import PositionSample\n"
        "from navpy.core.kernel.location import Location\n"
        "def f(vehicle, body):\n"
        "    return vehicle.location, body['position_lla'], vehicle.ground_speed\n"
    )
    hits = truth_reads(bad)
    assert "location@4" in hits and "ground_speed@4" in hits
    assert any("navpy.sim" in hit for hit in hits) and any("names 'position_lla'" in hit for hit in hits)
    assert any(hit.endswith(".location") for hit in hits)
