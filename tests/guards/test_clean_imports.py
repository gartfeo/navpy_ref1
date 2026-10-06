"""Clean-process import guards for package-initialization cycles."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    "statement",
    (
        "import navpy.main",
        (
            "from navpy.modules.navigation.geo import GeoRefCalc; "
            "from navpy.modules.vision import DetectorSim"
        ),
    ),
)
def test_navpy_imports_succeed_in_a_clean_process(statement: str) -> None:
    env = os.environ.copy()
    python_path = str(REPO_ROOT / "src")
    if env.get("PYTHONPATH"):
        python_path += os.pathsep + env["PYTHONPATH"]
    env["PYTHONPATH"] = python_path

    result = subprocess.run(
        [sys.executable, "-c", statement],
        cwd=REPO_ROOT,
        env=env,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )

    assert result.returncode == 0, result.stderr


def test_core_composition_modules_do_not_import_compatibility_facades() -> None:
    roll_composition = (
        REPO_ROOT
        / "src/navpy/modules/navigation/nav/roll_l1_composition.py"
    ).read_text(encoding="utf-8")
    roll_law = (
        REPO_ROOT
        / "src/navpy/modules/navigation/nav/roll_l1_pitch_nav_law.py"
    ).read_text(encoding="utf-8")
    detector_factory_ports = (
        REPO_ROOT
        / "src/navpy/modules/vision/vision_detector_factory_ports.py"
    ).read_text(encoding="utf-8")
    bus_registry = (
        REPO_ROOT
        / "src/navpy/modules/vehicle/mav_bus_registry.py"
    ).read_text(encoding="utf-8")

    assert "roll_l1_pitch_nav import" not in roll_composition
    assert "roll_l1_pitch_nav import" not in roll_law
    assert "vision_profiles import CameraMountSpec" not in detector_factory_ports
    assert "._lifecycle" not in bus_registry
    assert "._membership" not in bus_registry


def test_internal_modules_do_not_depend_on_compatibility_facades() -> None:
    source_root = REPO_ROOT / "src/navpy"
    facade_imports = (
        "from navpy.logger.navigation_snap import",
        "from navpy.modules.navigation.gimbal_geo_tracking import",
        "from navpy.modules.navigation.gimbal_tracking_session import",
        "from navpy.modules.vision.vision_profiles import",
        "from navpy.modules.vision import vision_profiles",
        "from navpy.modules.vision.target_zoom_tracker import",
        "from navpy.modules.comm.messages.available_task_msg import",
        "from navpy.modules.swarm.task_actor_state import",
        "from navpy.modules.nav.confirmation_workflow import",
        "from navpy.modules.nav.navigation_task import",
        "from navpy.modules.nav.nav_confirmation_composition import",
        "from navpy.modules.nav.navigation_transitions import",
        "from navpy.modules.nav.target_confirmation import",
        "from navpy.modules.nav.terminal_navigation import",
        "from navpy.modules.vehicle.mission_protocol import",
        "from navpy.modules.vehicle import pose_cadence_debug",
    )
    allowed_imports = {
        (
            source_root / "modules/vision/__init__.py",
            "from navpy.modules.vision.target_zoom_tracker import",
        ),
    }
    violations: list[str] = []
    for path in source_root.rglob("*.py"):
        source = path.read_text(encoding="utf-8")
        for import_text in facade_imports:
            if (
                import_text in source
                and (path, import_text) not in allowed_imports
            ):
                violations.append(
                    f"{path.relative_to(REPO_ROOT).as_posix()}: {import_text}"
                )

    assert not violations, "\n".join(violations)
