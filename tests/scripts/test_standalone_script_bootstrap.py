from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    "script_relative_path",
    (
        "scripts/vision_static_point_mass.py",
        "scripts/python/run_gimbal_tracker.py",
        "scripts/python/run_detector.py",
        "tools/cam/gimbal_controller.py",
    ),
)
def test_standalone_script_prefers_current_checkout_src(
    tmp_path: Path,
    script_relative_path: str,
) -> None:
    shadow_package = tmp_path / "navpy"
    shadow_package.mkdir()
    (shadow_package / "__init__.py").write_text(
        'raise RuntimeError("loaded shadow navpy")\n',
        encoding="utf-8",
    )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(tmp_path), str(REPOSITORY_ROOT / "src"))
    )

    result = subprocess.run(
        [sys.executable, script_relative_path, "--help"],
        cwd=REPOSITORY_ROOT,
        env=environment,
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "loaded shadow navpy" not in result.stderr
