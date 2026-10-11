"""Clean-process import guards for package-initialization cycles."""

from __future__ import annotations

import ast
import os
import subprocess
import sys
from pathlib import Path

import pytest

from tests.guards.step1_sources import (
    SourceFile,
    import_refs,
    module_of,
    navpy_sources,
    plan_module,
)


REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    "statement",
    (
        "import navpy.main",
        (
            "from navpy.modules.navigation.geo.geo_ref_calc import GeoRefCalc; "
            "from navpy.modules.vision.sim.detector_sim import DetectorSim"
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


def compatibility_facades() -> frozenset[str]:
    """Modules of the shims/facades step 1 deletes: the manifest's delete
    rows that are not package inits."""
    plan = plan_module()
    return frozenset(
        f"navpy.{module_of(row.src)}"
        for row in plan.load_manifest(REPO_ROOT)
        if row.action == plan.DELETE and not row.src.endswith("__init__.py")
    )


def facade_imports(
    sources: dict[str, SourceFile], facades: frozenset[str]
) -> list[str]:
    """`path:line module[.name]` for every import that names a facade module,
    as `import facade`, `from facade import x` or `from package import facade`."""
    violations: list[str] = []
    for below, source in sorted(sources.items()):
        for ref in source.refs:
            named = f"{ref.module}.{ref.name}" if ref.name else ref.module
            if ref.module in facades or named in facades:
                violations.append(f"src/navpy/{below}:{ref.line} {named}")
    return violations


def test_compatibility_facades_are_the_twenty_deleted_shims() -> None:
    facades = compatibility_facades()

    assert len(facades) == 20
    assert not any(module.endswith("__init__") for module in facades)
    assert not [
        module for module in facades
        if (REPO_ROOT / "src" / f"{module.replace('.', '/')}.py").exists()
    ]


def test_facade_import_scan_finds_each_import_form() -> None:
    facade = "navpy.pkg.old_facade"
    source = (
        "import navpy.pkg.old_facade",
        "from navpy.pkg.old_facade import Thing",
        "from navpy.pkg import old_facade",
        "from navpy.pkg import real_module",
        "def lazy():",
        "    from navpy.pkg.old_facade import Other",
    )
    tree = ast.parse("\n".join(source))
    sources = {
        "pkg/user.py": SourceFile(
            "pkg/user.py", "navpy.pkg.user", tree,
            tuple(import_refs(tree, "navpy.pkg")),
        ),
    }

    assert facade_imports(sources, frozenset({facade})) == [
        "src/navpy/pkg/user.py:1 navpy.pkg.old_facade",
        "src/navpy/pkg/user.py:2 navpy.pkg.old_facade.Thing",
        "src/navpy/pkg/user.py:3 navpy.pkg.old_facade",
        "src/navpy/pkg/user.py:6 navpy.pkg.old_facade.Other",
    ]


def test_internal_modules_do_not_depend_on_compatibility_facades() -> None:
    violations = facade_imports(navpy_sources(REPO_ROOT), compatibility_facades())

    source_root = REPO_ROOT / "src/navpy"
    package_import = "from navpy.modules.vehicle import pose_cadence_debug"
    for path in source_root.rglob("*.py"):
        if package_import in path.read_text(encoding="utf-8"):
            violations.append(
                f"{path.relative_to(REPO_ROOT).as_posix()}: {package_import}"
            )

    assert not violations, "\n".join(violations)
