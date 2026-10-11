"""Patch targets on split sources follow the part that looks the name up.

Read from the step-1 base commit, so the check keeps its meaning after c02
splits the sources and rewrites the targets: the 16 mock.patch targets on
real_detector_models in test_detector_deep_search.py must go to the builder
part, implicit concatenations included.
"""

from __future__ import annotations

import ast
import importlib.util
import subprocess
import sys
from pathlib import Path
from types import ModuleType

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = REPO_ROOT / "scripts"
STEP1_BASE = "2c887d9"
PATCHING_TEST = "tests/modules/vision/test_detector_deep_search.py"
OLD_MODELS = "navpy.modules.vision.real_detector_models"
BUILDER = "navpy.app.perception.real_detector_model_builder"
PATCHED_NAMES = {"YoloDetector", "create_tracker_backend", "create_appearance_embedder",
                 "AsyncAppearanceEmbedder", "deep_search_config_from_settings",
                 "DeepSearchDetector"}


def _load(name: str) -> ModuleType:
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


for _name in ("step1_manifest", "step1_rewrite_source", "step1_rewrite_maps",
              "step1_rewrite_strings", "step1_rewrite_imports"):
    _load(_name)
sm = _load("step1_manifest")
maps_module = _load("step1_rewrite_maps")
lookups_module = _load("step1_lookups")
rewrite = _load("step1_rewrite")


def _at_base(path: str) -> str:
    result = subprocess.run(["git", "show", f"{STEP1_BASE}:{path}"], cwd=REPO_ROOT,
                            capture_output=True, check=True)
    return result.stdout.decode("utf-8")


def _maps() -> object:
    rows = sm.load_manifest(REPO_ROOT)
    splits = sm.load_splits(REPO_ROOT)
    sources = {src: _at_base(sm.repo_path(src)) for src in splits}
    return maps_module.RewriteMaps(
        modules={}, known=frozenset(sm.module_map(rows, "step1")), paths={},
        symbols=sm.split_symbol_map(splits, "step1"),
        lookups=lookups_module.split_lookups(sources, splits, "step1"),
    )


def _patch_targets(text: str) -> list[str]:
    return [node.value for node in ast.walk(ast.parse(text))
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
            and node.value.rpartition(".")[2] in PATCHED_NAMES]


def test_real_detector_model_patches_go_to_the_builder_part() -> None:
    before = _at_base(PATCHING_TEST)
    assert sorted(_patch_targets(before)) == sorted(
        f"{OLD_MODELS}.{name}" for name in PATCHED_NAMES for _ in range(
            {"YoloDetector": 3, "create_tracker_backend": 3, "create_appearance_embedder": 3,
             "deep_search_config_from_settings": 3, "AsyncAppearanceEmbedder": 2,
             "DeepSearchDetector": 2}[name]))

    result = rewrite.rewrite_source(before, _maps())

    assert result.problems == ()
    after = _patch_targets(result.text)
    assert len(after) == 16
    assert all(target.startswith(f"{BUILDER}.") for target in after), after


def test_a_name_the_rest_and_a_part_both_call_is_not_routed() -> None:
    splits = {"attic/models.py": (sm.SplitPart(("build",), "app/builder.py", "app/builder.py",
                                               False),)}
    source = (
        "from navpy.attic.yolo import Yolo\n"
        "from navpy.attic.math import clamp\n"
        "from navpy.attic.kinds import Kind\n"
        "ALIAS = Kind\n"
        "def close(value: Yolo) -> int:\n    return clamp(value)\n"
        "def build() -> Yolo:\n    return Yolo(clamp(1))\n"
    )

    lookups = lookups_module.split_lookups({"attic/models.py": source}, splits, "step1")

    assert lookups == {("navpy.attic.models", "Yolo"): "navpy.app.builder",
                       ("navpy.attic.models", "clamp"): None}
