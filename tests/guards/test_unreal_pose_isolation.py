"""Pure Vision guard for the HITL Unreal camera (docs/design/hitl_unreal_camera.md).

The HITL bridge sends the plant's true pose to Unreal for rendering only. navpy gets pixels and the
board TIME of the rendered pose, never the pose: no source file may import the bridge's sender or
speak its vehicle_state schema, and the NPYC frame header carries no position or attitude field.
"""

from __future__ import annotations

import ast
from dataclasses import fields
from pathlib import Path

from navpy.plugins.camera.unreal.npyc_codec import NpycHeader

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
SENDER_MODULE = "tools.tuning.hitl_bridge.unreal_pose"
POSE_WIRE_NAMES = ("navpy.unreal.vehicle_state", "position_lla", "attitude_deg")


def _imports(tree: ast.AST) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
    return names


def test_no_source_file_imports_the_bridge_pose_sender_or_speaks_its_pose_schema():
    offenders = []
    for path in SRC.rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        tree = ast.parse(text)
        if any(name == SENDER_MODULE or name.startswith("tools.") for name in _imports(tree)):
            offenders.append(f"{path.relative_to(ROOT)} imports the bridge tools")
        for node in ast.walk(tree):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                if any(wire in node.value for wire in POSE_WIRE_NAMES):
                    offenders.append(f"{path.relative_to(ROOT)} names {node.value!r}")
    assert offenders == []


def test_the_frame_header_carries_timing_and_identity_only():
    pose_like = {"lat", "lon", "alt", "roll", "pitch", "yaw", "position", "attitude", "heading", "range"}
    names = {f.name for f in fields(NpycHeader)}
    assert not {name for name in names if any(part in name for part in pose_like)}
    assert {"board_time_us", "pose_sequence", "pose_flags"} <= names
