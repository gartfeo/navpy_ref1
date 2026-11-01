"""Camera health is operator display only (docs/design/gcs_preflight_camera_row.md).

The detector camera's readiness goes out in the camera HEARTBEAT and nowhere
else. Only the wiring from the frame store to the camera publisher may name
the port or the readiness reading, so nothing on the navigation command path
can read it, by import or through an object it already holds.
"""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src/navpy"
HEALTH_MODULE = "navpy.core.ports.camera_health"
# Names a reader would have to write to reach the readiness reading.
HEALTH_NAMES = frozenset({
    "CameraHealth",
    "camera_health",
    "has_fresh_frame",
    "DETECTOR_FRAME_FRESH_S",
})
# The wiring: frame store -> frame provider -> detector diagnostics ->
# detector factories -> composition -> camera publisher and its messages.
WIRING = frozenset({
    "src/navpy/core/ports/camera_health.py",
    "src/navpy/core/perception/camera/frame_publication.py",
    "src/navpy/core/perception/camera/frame_provider.py",
    "src/navpy/core/perception/pipeline/real_detector_diagnostics.py",
    "src/navpy/core/perception/pipeline/detector_public_diagnostics.py",
    "src/navpy/app/perception/vision_detector_factory_ports.py",
    "src/navpy/app/perception/real_detector_factory.py",
    "src/navpy/app/perception/sim_detector_factory.py",
    "src/navpy/app/perception/camera_stream_composition.py",
    "src/navpy/app/perception/vision_controller_composition.py",
    "src/navpy/plugins/vehicle/ardupilot/camera_stream_messages.py",
    "src/navpy/plugins/vehicle/ardupilot/camera_stream_publisher.py",
})


def _references(tree: ast.AST) -> list[str]:
    hits = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            hits += [alias.name for alias in node.names if alias.name == HEALTH_MODULE]
        elif isinstance(node, ast.ImportFrom) and node.module:
            if node.module == HEALTH_MODULE:
                hits.append(node.module)
            hits += [alias.name for alias in node.names if alias.name in HEALTH_NAMES]
            if node.module == "navpy.core.ports":
                hits += [
                    alias.name for alias in node.names if alias.name == "camera_health"
                ]
        elif isinstance(node, ast.Name) and node.id in HEALTH_NAMES:
            hits.append(f"{node.id}@{node.lineno}")
        elif isinstance(node, ast.Attribute) and node.attr in HEALTH_NAMES:
            hits.append(f"{node.attr}@{node.lineno}")
        elif isinstance(node, ast.keyword) and node.arg in HEALTH_NAMES:
            hits.append(f"{node.arg}=@{node.value.lineno}")
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and (
            node.name in HEALTH_NAMES
        ):
            hits.append(f"def {node.name}@{node.lineno}")
        elif isinstance(node, ast.Constant) and node.value == HEALTH_MODULE:
            hits.append(f"'{HEALTH_MODULE}'@{node.lineno}")  # importlib by name
    return hits


def test_wiring_modules_exist():
    missing = [path for path in sorted(WIRING) if not (ROOT / path).is_file()]
    assert not missing, missing


def test_only_the_wiring_names_the_camera_health():
    offenders = {}
    for path in sorted(SRC.rglob("*.py")):
        relative = path.relative_to(ROOT).as_posix()
        if relative in WIRING:
            continue
        hits = _references(ast.parse(path.read_text(encoding="utf-8")))
        if hits:
            offenders[relative] = hits
    assert offenders == {}


def test_every_wiring_module_still_needs_its_place():
    unused = [
        path for path in sorted(WIRING)
        if not _references(ast.parse((ROOT / path).read_text(encoding="utf-8")))
    ]
    assert not unused, unused


def test_guard_sees_every_way_to_reach_the_reading():
    for source in (
        "import navpy.core.ports.camera_health\n",
        "from navpy.core.ports.camera_health import CameraHealth\n",
        "from navpy.core.ports import camera_health\n",
        "ready = detector.has_fresh_frame()\n",
        "port = node.camera_health\n",
        "Node(camera_health=port)\n",
        "importlib.import_module('navpy.core.ports.camera_health')\n",
        "limit = DETECTOR_FRAME_FRESH_S\n",
        "def has_fresh_frame(self):\n    return True\n",
    ):
        assert _references(ast.parse(source)), source
