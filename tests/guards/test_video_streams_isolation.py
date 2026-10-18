"""Video is operator display only (docs/design/gcs_video_streams.md).

No NavPy module may import the GCS video-stream discovery code, so stream
state can never reach navigation or the final-approach command path.
"""

from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src/navpy"
VIDEO_MODULES = frozenset({
    "gcs.backend.video_camera_discovery",
    "gcs.backend.video_stream_policy",
    "gcs.backend.video_stream_uri",
    "gcs.backend.video_streams",
})


def _imported_modules(tree: ast.AST) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            names.add(node.module)
            names.update(f"{node.module}.{alias.name}" for alias in node.names)
    return names


def _video_imports(tree: ast.AST) -> set[str]:
    return {
        name for name in _imported_modules(tree)
        if any(name == module or name.startswith(f"{module}.") for module in VIDEO_MODULES)
    }


def test_video_modules_exist() -> None:
    for module in VIDEO_MODULES:
        path = ROOT / "src" / (module.replace(".", "/") + ".py")
        assert path.is_file(), module


def test_no_navpy_module_imports_video_discovery() -> None:
    offenders = {}
    for path in sorted(SRC.rglob("*.py")):
        found = _video_imports(ast.parse(path.read_text(encoding="utf-8")))
        if found:
            offenders[path.relative_to(ROOT).as_posix()] = sorted(found)
    assert offenders == {}


def test_guard_detects_both_import_forms() -> None:
    for source in (
        "import gcs.backend.video_streams\n",
        "from gcs.backend.video_streams import VehicleVideoStreams\n",
        "from gcs.backend import video_stream_uri\n",
    ):
        assert _video_imports(ast.parse(source)), source
