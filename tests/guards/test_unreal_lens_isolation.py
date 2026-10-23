"""Pure Vision guard for the Unreal lens table (HITL stage 1 step 5, docs/design/hitl_final_approach.md section 9).

The ZR10 emulator tells Unreal the true lens state (reading, the setting it was reached by, settled) so the camera
renders the right reading_intrinsics row, and the camera logs which row each frame rendered with. That is simulator
truth for rendering and offline scoring: navpy gets pixels and the wire's zoom reading, never the gimbal_state
datagram, the lens object, the capture-slot log or the emulator's step log.
"""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TRUTH_NAMES = ("navpy.unreal.gimbal_state", "lens_settled", "lens_command", "lens_set_by", "lens_basis",
               "NavPyCaptureSlot", "NavPyLensTable", "NavPyGimbalState")


def test_no_source_file_reads_the_simulated_lens_truth():
    offenders = []
    for path in (ROOT / "src").rglob("*.py"):
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Constant) and isinstance(node.value, str):
                offenders += [f"{path.relative_to(ROOT)} names {name!r}" for name in TRUTH_NAMES if name in node.value]
            elif isinstance(node, (ast.Attribute, ast.Name)):
                name = node.attr if isinstance(node, ast.Attribute) else node.id
                if name in ("lens_settled", "lens_command"):
                    offenders.append(f"{path.relative_to(ROOT)} uses {name}")
    assert offenders == []
