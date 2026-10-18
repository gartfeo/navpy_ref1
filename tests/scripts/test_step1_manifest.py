"""The step-1 plan reader every c00 guard and the move tool share.

The commit counts pin docs/design/step1/commits.tsv to the plan's Commits
table (docs/design/step1_separation.md); the exceptions and final-violation
rows pin layer_of/allowed to the planning checker that produced them.
"""

from __future__ import annotations

import csv
import importlib.util
import sys
from collections import Counter
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / "scripts" / "step1_manifest.py"

spec = importlib.util.spec_from_file_location("step1_manifest", SCRIPT_PATH)
sm = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = sm
spec.loader.exec_module(sm)

# The plan's Commits table: files each commit moves (move, gcs and split rows).
PLAN_COMMIT_FILES = {
    "c03": 39, "c04": 23, "c05": 37, "c06": 70, "c07": 31, "c08": 28, "c09": 49, "c10": 54,
    "c11": 39, "c12": 41, "c13": 44, "c14": 55, "c15": 35, "c16": 19, "c17": 116, "c18": 39,
}
MANIFEST = sm.load_manifest(REPO_ROOT)
ROWS = {row.src: row for row in MANIFEST}
PREFIXES = sm.load_commits(REPO_ROOT)


def _row(src: str, action: str = "move", step1: str = "core/kernel/x.py") -> sm.ManifestRow:
    return sm.ManifestRow(src, step1, step1, action, False, "")


# --- commits ---------------------------------------------------------------

def test_commit_file_counts_match_the_plan_table() -> None:
    counts = Counter(sm.commit_of(row, PREFIXES) for row in MANIFEST)
    moving = [row for row in MANIFEST if row.action in sm.MOVING_ACTIONS]

    assert {c: n for c, n in counts.items() if c is not None} == PLAN_COMMIT_FILES
    assert len(moving) == sum(PLAN_COMMIT_FILES.values()) == 719
    for commit, files in PLAN_COMMIT_FILES.items():
        assert len(sm.rows_for_commit(MANIFEST, PREFIXES, commit)) == files, commit


def test_every_moving_row_has_one_commit_and_no_other_row_has_one() -> None:
    for row in MANIFEST:
        commit = sm.commit_of(row, PREFIXES)
        assert (commit is not None) == (row.action in sm.MOVING_ACTIONS), row.src


def test_every_commit_prefix_claims_a_row() -> None:
    claimed = Counter()
    for row in MANIFEST:
        matches = [p for p in PREFIXES if row.step1_dest.startswith(p.step1_prefix)]
        if row.action in sm.MOVING_ACTIONS and matches:
            claimed[max(matches, key=lambda p: len(p.step1_prefix))] += 1
    assert PREFIXES
    assert [p for p in PREFIXES if not claimed[p]] == []


def test_longest_prefix_wins() -> None:
    assert sm.commit_of(_row("a.py", step1="sim/radio/x.py"), PREFIXES) == "c05"
    assert sm.commit_of(_row("a.py", step1="sim/camera/x.py"), PREFIXES) == "c17"
    assert sm.commit_of(_row("a.py", step1="core/perception/x.py"), PREFIXES) == "c10"
    assert sm.commit_of(_row("a.py", step1="core/perception/zoom/x.py"), PREFIXES) == "c09"
    assert sm.commit_of(_row("a.py", step1="gcs:backend/x.py", action="gcs"), PREFIXES) == "c05"
    assert sm.commit_of(_row("a.py", step1="nowhere/x.py"), PREFIXES) is None
    assert sm.commit_of(_row("a.py", step1="delete", action="delete"), PREFIXES) is None


# --- loaders ---------------------------------------------------------------

def test_manifest_rows_parse() -> None:
    actions = Counter(row.action for row in MANIFEST)
    assert actions["delete"] == 40
    assert set(actions) == sm.ACTIONS
    assert len(ROWS) == len(MANIFEST)
    assert sum(row.vision_path for row in MANIFEST) == 161


def test_splits_match_the_split_rows() -> None:
    splits = sm.load_splits(REPO_ROOT)
    assert set(splits) == {row.src for row in MANIFEST if row.action == "split"}
    assert len(splits) == 13
    assert sum(len(parts) for parts in splits.values()) == 14
    assert all(part.symbols for parts in splits.values() for part in parts)


def test_reexports_match_the_plan() -> None:
    reexports = sm.load_reexports(REPO_ROOT)
    assert len(reexports) == 95
    assert len({(r.src, r.line) for r in reexports}) == 43
    assert len({r.src for r in reexports}) == 29
    assert {r.src for r in reexports} <= set(ROWS)


def test_exceptions_parse() -> None:
    guards = Counter(e.guard for e in sm.load_exceptions(REPO_ROOT))
    assert guards == {"layer": 27, "vision": 15}


@pytest.mark.parametrize("bad_line, message", [
    ("modules/a.py\tdelete\tdelete\tteleport\tn\tx\n", "unknown action"),
    ("modules/a.py\tdelete\tdelete\tdelete\tmaybe\tx\n", "expected 'y' or 'n'"),
    ("modules/a.py\tdelete\tdelete\tdelete\tn\n", "expected 6 fields"),
    ("modules/a.py\tdelete\tdelete\tdelete\tn\tx\textra\n", "expected 6 fields"),
    ("modules/b.py\tdelete\tdelete\tdelete\tn\tx\n" * 2, "duplicate src"),
])
def test_malformed_manifest_is_rejected(tmp_path: Path, bad_line: str, message: str) -> None:
    data = tmp_path / sm.STEP1_DATA
    data.mkdir(parents=True)
    (data / "manifest.tsv").write_text("\t".join(sm.MANIFEST_COLUMNS) + "\n" + bad_line, encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        sm.load_manifest(tmp_path)


def test_wrong_columns_are_rejected(tmp_path: Path) -> None:
    data = tmp_path / sm.STEP1_DATA
    data.mkdir(parents=True)
    (data / "commits.tsv").write_text("commit\tprefix\nc03\tcore/kernel/\n", encoding="utf-8")
    with pytest.raises(ValueError, match="columns"):
        sm.load_commits(tmp_path)


def test_unknown_stage_is_rejected() -> None:
    with pytest.raises(KeyError, match="step2"):
        MANIFEST[0].dest("step2")
    with pytest.raises(KeyError, match="step2"):
        sm.split_symbol_map(sm.load_splits(REPO_ROOT), "step2")


# --- path and module maps ----------------------------------------------------

def test_path_map_covers_every_row() -> None:
    step1 = sm.path_map(MANIFEST, "step1")
    final = sm.path_map(MANIFEST, "final")
    assert len(step1) == len(final) == len(MANIFEST)
    assert step1["src/navpy/modules/comm/esp32_trigger.py"] == "src/gcs/backend/esp32_trigger.py"
    assert step1["src/navpy/modules/vision/__init__.py"] is None
    assert step1["src/navpy/main.py"] == "src/navpy/main.py"
    assert step1["src/navpy/args/nav_args.py"] == "src/navpy/core/kernel/config/nav_args.py"
    assert (step1["src/navpy/modules/nav/mission_catalog.py"], final["src/navpy/modules/nav/mission_catalog.py"]) == (
        "src/navpy/core/flow/mission_catalog.py", "src/navpy/products/delivery/mission_catalog.py")


def test_module_map_names_packages_and_skips_data_files() -> None:
    step1 = sm.module_map(MANIFEST, "step1")
    assert step1["navpy.modules.comm.messages"] == "navpy.core.link"
    assert step1["navpy.modules.vehicle"] == "navpy.plugins.vehicle.ardupilot"
    assert step1["navpy.modules.comm.esp32_trigger"] == "gcs.backend.esp32_trigger"
    assert step1["navpy.args.nav_args"] == "navpy.core.kernel.config.nav_args"
    assert step1["navpy.modules.vision"] is None
    assert step1["navpy"] == "navpy"
    assert len(step1) == sum(row.src.endswith(".py") for row in MANIFEST)
    assert sm.module_map(MANIFEST, "final")["navpy.modules.nav.mission_catalog"] == (
        "navpy.products.delivery.mission_catalog")


def test_split_symbol_map_follows_each_part() -> None:
    splits = sm.load_splits(REPO_ROOT)
    step1 = sm.split_symbol_map(splits, "step1")
    final = sm.split_symbol_map(splits, "final")
    assert step1[("navpy.modules.vision.yolo_detector", "Detection")] == "navpy.core.ports.detection"
    assert step1[("navpy.modules.vision.real_detector_ports", "FrameDetector")] == "navpy.core.ports.detection"
    key = ("navpy.modules.nav.mission_navigation", "FallbackMissionNavigation")
    assert (step1[key], final[key]) == (
        "navpy.core.flow.fallback_mission_navigation", "navpy.products.delivery.fallback_mission_navigation")
    assert len(step1) == sum(len(p.symbols) for parts in splits.values() for p in parts)


# --- current location --------------------------------------------------------

def _touch(root: Path, relative: str) -> None:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("", encoding="utf-8")


def test_current_location_follows_the_move(tmp_path: Path) -> None:
    row = ROWS["args/nav_args.py"]
    assert sm.current_location(tmp_path, row) is None
    _touch(tmp_path, "src/navpy/args/nav_args.py")
    assert sm.current_location(tmp_path, row) == "src/navpy/args/nav_args.py"
    _touch(tmp_path, "src/navpy/core/kernel/config/nav_args.py")
    assert sm.current_location(tmp_path, row) == "src/navpy/args/nav_args.py"
    (tmp_path / "src/navpy/args/nav_args.py").unlink()
    assert sm.current_location(tmp_path, row) == "src/navpy/core/kernel/config/nav_args.py"


def test_current_location_of_gcs_and_deleted_rows(tmp_path: Path) -> None:
    _touch(tmp_path, "src/gcs/backend/esp32_trigger.py")
    assert sm.current_location(tmp_path, ROWS["modules/comm/esp32_trigger.py"]) == (
        "src/gcs/backend/esp32_trigger.py")
    assert sm.current_location(tmp_path, ROWS["modules/vision/__init__.py"]) is None
    _touch(tmp_path, "src/navpy/modules/vision/__init__.py")
    assert sm.current_location(tmp_path, ROWS["modules/vision/__init__.py"]) == (
        "src/navpy/modules/vision/__init__.py")


# --- layers ------------------------------------------------------------------

@pytest.mark.parametrize("path, layer", [
    ("core/kernel/config/nav_args.py", "core/kernel"),
    ("core/ports/detection.py", "core/ports"),
    ("core/approach/vision_nav/frame.py", "core/approach"),
    ("core/__init__.py", "core"),
    ("plugins/vehicle/ardupilot/vehicle_mav.py", "plugins/vehicle"),
    ("products/delivery/mission_catalog.py", "products/delivery"),
    ("sim/detection/detector_sim.py", "sim"),
    ("app/perception/frame_capture_factory.py", "app"),
    ("main.py", "app"),
    ("main_gui.py", "app"),
    ("__init__.py", "app"),
    ("modules/vision/detector.py", "old"),
    ("logger/log.py", "old"),
    ("args/nav_args.py", "old"),
    ("utils/geometry.py", "old"),
    ("tools/esp32_simulator.py", "old"),
    ("runtime_composition.py", "old"),
    ("gcs:backend/esp32_trigger.py", "gcs"),
    ("delete", "deleted"),
    ("core\\kernel\\x.py", "core/kernel"),
])
def test_layer_of(path: str, layer: str) -> None:
    assert str(sm.layer_of(path)) == layer


def test_layer_of_rejects_repo_relative_paths() -> None:
    with pytest.raises(ValueError, match="under src/navpy"):
        sm.layer_of("src/navpy/core/kernel/x.py")


@pytest.mark.parametrize("importer, imported, expected", [
    ("core/kernel/a.py", "core/kernel/b.py", True),
    ("core/kernel/a.py", "core/ports/b.py", False),
    ("core/ports/a.py", "core/kernel/b.py", True),
    ("core/ports/a.py", "core/link/b.py", False),
    ("core/flow/a.py", "core/skills/b.py", True),
    ("core/flow/a.py", "plugins/vehicle/ardupilot/b.py", False),
    ("plugins/radio/serial/a.py", "plugins/radio/wifi/b.py", True),
    ("plugins/radio/serial/a.py", "core/ports/b.py", True),
    ("plugins/radio/serial/a.py", "core/link/b.py", False),
    ("plugins/radio/serial/a.py", "plugins/vehicle/ardupilot/b.py", False),
    ("products/delivery/a.py", "core/flow/b.py", True),
    ("products/delivery/a.py", "products/pickup/b.py", False),
    ("products/delivery/a.py", "plugins/laws/geo/b.py", False),
    ("products/delivery/a.py", "sim/b.py", False),
    ("core/flow/a.py", "products/delivery/b.py", False),
    ("plugins/vehicle/ardupilot/a.py", "products/delivery/b.py", False),
    ("sim/a.py", "products/delivery/b.py", False),
    ("sim/a.py", "plugins/gimbal/siyi/b.py", True),
    ("sim/a.py", "app/b.py", False),
    ("app/a.py", "sim/b.py", True),
    ("main.py", "plugins/vehicle/ardupilot/b.py", True),
    ("core/flow/a.py", "main.py", False),
    ("core/flow/a.py", "delete", False),
    ("core/flow/a.py", "gcs:backend/b.py", False),
    ("modules/nav/a.py", "core/kernel/b.py", True),
    ("core/kernel/a.py", "modules/nav/b.py", True),
])
def test_allowed(importer: str, imported: str, expected: bool) -> None:
    assert sm.allowed(sm.layer_of(importer), sm.layer_of(imported)) is expected


def test_layer_exceptions_are_wrong_way_edges_under_these_rules() -> None:
    exceptions = sm.load_exceptions(REPO_ROOT)
    assert exceptions
    for e in exceptions:
        importer, imported = sm.layer_of(e.file), sm.layer_of(e.imports)
        assert (str(importer), str(imported)) == (e.from_layer, e.to_layer), e
        if e.guard == "layer":
            assert not sm.allowed(importer, imported), e


def test_final_violations_are_wrong_way_edges_under_these_rules() -> None:
    def final_home(key: str) -> str:
        return key.split("#", 1)[1] if "#" in key else ROWS[key].final_dest

    with (REPO_ROOT / sm.STEP1_DATA / "final_violations.tsv").open(encoding="utf-8", newline="") as handle:
        records = list(csv.DictReader(handle, delimiter="\t"))
    assert len(records) == 35
    for record in records:
        importer, imported = sm.layer_of(final_home(record["src"])), sm.layer_of(final_home(record["imports"]))
        assert (str(importer), str(imported)) == (record["from_layer"], record["to_layer"]), record
        assert not sm.allowed(importer, imported), record
