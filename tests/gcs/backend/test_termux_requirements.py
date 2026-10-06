"""The Termux requirement set must track the desktop backend dependencies."""
from __future__ import annotations

import re
import tomllib
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]

# Deliberate differences, explained in src/gcs/requirements-termux.txt.
_FROM_TERMUX_PACKAGES = {"numpy", "opencv-python-headless"}
_TEST_ONLY = {"pytest"}


def _name(requirement: str) -> str:
    """Distribution name without extras, version, URL or comment."""
    return re.split(r"[\s\[<>=!~@;#]", requirement.strip(), maxsplit=1)[0].lower()


def _file_names(path: Path) -> set[str]:
    lines = path.read_text(encoding="utf-8").splitlines()
    return {_name(line) for line in lines if line.strip() and not line.lstrip().startswith("#")}


def test_termux_requirements_cover_backend_and_navpy_dependencies():
    pyproject = tomllib.loads((_REPO / "pyproject.toml").read_text(encoding="utf-8"))
    desktop = _file_names(_REPO / "src" / "gcs" / "requirements.txt")
    desktop |= {_name(dep) for dep in pyproject["project"]["dependencies"]}
    expected = desktop - _FROM_TERMUX_PACKAGES - _TEST_ONLY

    termux = _file_names(_REPO / "src" / "gcs" / "requirements-termux.txt")

    assert termux == expected


def test_termux_uvicorn_has_no_native_extras():
    text = (_REPO / "src" / "gcs" / "requirements-termux.txt").read_text(encoding="utf-8")
    assert "uvicorn[" not in text
