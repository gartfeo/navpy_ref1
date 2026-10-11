"""Physical SOLID limits for broad, compatibility-critical public adapters."""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PUBLIC_ADAPTERS = {
    "src/navpy/modules/vehicle/vehicle_mav.py": "VehicleMav",
    "src/navpy/modules/vision/detector.py": "Detector",
    "src/navpy/modules/vision/sim/detector_sim.py": "DetectorSim",
    "src/navpy/modules/vision/detection_coordinator.py": "DetectionCoordinator",
    "src/navpy/modules/vision/peripheral/siyi/sim/gimbal_physics.py": "GimbalPhysics",
    "src/navpy/modules/vision/peripheral/siyi/sim/gimbal_siyi_sim.py": "GimbalSiyiSim",
    "src/navpy/modules/vision/poi_zoom_orchestrator.py": "PoiZoomTracker",
}
# Top-level *Facet classes in src/navpy on 2026-10-11. The scan covers the
# whole package so moved files stay in it; finding fewer means one was lost.
MIN_FACET_CLASSES = 61


def _public_class(relative: str, name: str) -> ast.ClassDef:
    path = ROOT / relative
    assert path.is_file(), f"listed adapter is missing: {relative}"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == name
    )


def test_public_adapters_only_construct_focused_capability_facets() -> None:
    violations = []
    for relative, name in PUBLIC_ADAPTERS.items():
        owner = _public_class(relative, name)
        methods = [
            node.name
            for node in owner.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        ]
        if methods != ["__init__"]:
            violations.append(f"{relative}:{name} directly owns {methods}")
        fields = {
            target.attr
            for node in ast.walk(owner)
            if isinstance(node, (ast.Assign, ast.AnnAssign))
            for target in (
                node.targets if isinstance(node, ast.Assign) else [node.target]
            )
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
            )
        }
        if fields != {"_parts"}:
            violations.append(f"{relative}:{name} owns fields {sorted(fields)}")
        for statement in owner.body:
            if not isinstance(statement, (ast.Assign, ast.AnnAssign)):
                continue
            value = statement.value
            if isinstance(value, (ast.Call, ast.Lambda, ast.Name, ast.Attribute)):
                violations.append(
                    f"{relative}:{name}:{statement.lineno} uses callable rebinding"
                )
    assert not violations, "\n".join(violations)


def test_public_adapter_facets_are_stateless_and_physically_bounded() -> None:
    violations = []
    facets = 0
    for path in sorted((ROOT / "src/navpy").rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for owner in (node for node in tree.body if isinstance(node, ast.ClassDef)):
            if not owner.name.endswith("Facet"):
                continue
            facets += 1
            methods = [
                node
                for node in owner.body
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            ]
            if len(methods) > 15:
                violations.append(f"{path.name}:{owner.name} has {len(methods)} methods")
            if int(owner.end_lineno) - owner.lineno + 1 > 200:
                violations.append(f"{path.name}:{owner.name} exceeds 200 lines")
            if any(method.name == "__init__" for method in methods):
                violations.append(f"{path.name}:{owner.name} has __init__")
            fields = [
                node
                for node in ast.walk(owner)
                if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign))
                and any(
                    isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "self"
                    for target in (
                        node.targets
                        if isinstance(node, ast.Assign)
                        else [node.target]
                    )
                )
            ]
            if fields:
                violations.append(f"{path.name}:{owner.name} owns mutable state")
    assert facets >= MIN_FACET_CLASSES, f"found {facets} facet classes"
    assert not violations, "\n".join(violations)
