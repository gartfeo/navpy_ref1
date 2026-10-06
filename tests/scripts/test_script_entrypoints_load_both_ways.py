"""Scripts that are BOTH run directly and imported by tests must load both ways.

`scripts/siyi_pixel_pn_child.py` is launched as a subprocess by the eval harness
(`eval_direct_pixel_pn.py`, which sets PYTHONPATH to `src` only) and imported by
`tests/modules/vision/test_siyi_geo_pixel_source.py` as `scripts.siyi_pixel_pn_child`.

Each mode needs a different sys.path root, and fixing one broke the other twice:

  * a bare `from pixel_pn_flight_control_trace import ...` resolved when run and
    failed on import -- which made the WHOLE repository suite fail to COLLECT,
    so nothing outside tests/scripts was verified at all;
  * qualifying it to `scripts.pixel_pn_flight_control_trace` fixed collection
    and broke the harness launch with `No module named 'scripts'`.

Neither failure is visible from the other mode, which is why both are pinned.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

WORKTREE = Path(__file__).resolve().parents[2]

DUAL_MODE_SCRIPTS = [
    "siyi_pixel_pn_child.py",
    # The navigation bench: the parent is run directly and imported by tests, the
    # child is launched by PATH from a possibly-different source tree, and the
    # three helper modules are reached from both. Splitting the parent into
    # modules broke the package path in exactly the way documented above --
    # `import scripts.scratch_navigation_arms` raised ModuleNotFoundError for its
    # sibling while every test still passed, because another module had already
    # put `scripts/` on the path.
    "scratch_navigation_sitl.py",
    "scratch_navigation_uav.py",
    # The live eval harnesses: an operator runs them by path, and the tests
    # import them as `scripts.eval_direct_pixel_pn`. The three-UAV one had the
    # package imports without the path bootstrap its sibling carries, so
    # running it by path died on `No module named 'scripts'` before argparse --
    # invisible from the import side, where every test passed.
    "eval_direct_pixel_pn.py",
    "eval_direct_pixel_pn_three_uav.py",
    # The flight child: launched by path with PYTHONPATH=src, and imported by
    # tests for its speed step-down. It reached its siblings by relying on the
    # directory Python adds only for a script, so the import side died on
    # `No module named pixel_pn_flight_control_trace`.
    "direct_pixel_pn_child.py",
    # The verdict: run by path on a case's directory, and imported by
    # tests for its exit codes and the verdict it prints.
    "pixel_pn_determinism_verdict.py",
]

# Imported, never run: no `_parser`, so they are checked for import only.
IMPORT_ONLY_MODULES = [
    "eval_param_float32",
    "eval_direct_pixel_cli",
    "eval_fleet_cli",
    "eval_fleet_result",
    "swarm_run_cli",
    "eval_sim_parameters",
    "eval_source_identity",
    "pixel_pn_admission_ledger",
    "pixel_pn_case_manifest",
    "pixel_pn_child_teardown",
    "pixel_pn_determinism_evidence",
    "pixel_pn_determinism_summary",
    "pixel_pn_failure_report",
    "pixel_pn_run_identity",
    "pixel_pn_terminal_speed",
    "scratch_navigation_arms",
    "scratch_navigation_cells",
    "scratch_navigation_launch",
    "scratch_navigation_result",
]


@pytest.mark.parametrize("script", DUAL_MODE_SCRIPTS)
def test_the_script_runs_as_the_harness_launches_it(script):
    """Direct execution with PYTHONPATH=src, the harness's exact environment."""
    completed = subprocess.run(
        [sys.executable, str(WORKTREE / "scripts" / script), "--help"],
        cwd=str(WORKTREE),
        env={**dict(__import__("os").environ),
             "PYTHONPATH": str(WORKTREE / "src")},
        capture_output=True, text=True, timeout=120,
    )
    assert completed.returncode == 0, (
        f"{script} cannot start the way the harness launches it:\n"
        f"{completed.stderr[-2000:]}"
    )
    assert "usage:" in completed.stdout


@pytest.mark.parametrize("script", DUAL_MODE_SCRIPTS)
def test_the_script_imports_as_pytest_loads_it(script):
    """Package import, which is how the test suite reaches the same file."""
    module = __import__(
        f"scripts.{script[:-3]}", fromlist=["_parser"],
    )
    assert hasattr(module, "_parser")


@pytest.mark.parametrize("module_name", IMPORT_ONLY_MODULES)
def test_a_helper_module_imports_by_package_path_alone(module_name):
    """In a FRESH interpreter, so no other import can have fixed the path.

    Importing in-process proves nothing here: whichever module ran first may
    already have inserted `scripts/`, which is precisely how a broken package
    path passed a full green suite.
    """
    completed = subprocess.run(
        [sys.executable, "-c", f"import scripts.{module_name}"],
        cwd=str(WORKTREE),
        env={**dict(__import__("os").environ),
             "PYTHONPATH": str(WORKTREE / "src")},
        capture_output=True, text=True, timeout=120,
    )
    assert completed.returncode == 0, (
        f"scripts.{module_name} cannot be imported by package path:\n"
        f"{completed.stderr[-1500:]}"
    )
