"""The step-1 leftover scan: references to old modules, files and folders
that no longer exist fail; old paths still present (moving later) pass.

The synthetic plan has navpy/legacy/{alpha,beta}.py; alpha has moved, beta
has not, so every reference is checked both ways.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

import pytest

SCRIPTS = Path(__file__).resolve().parents[2] / "scripts"


def _load(name: str) -> ModuleType:
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


for _name in ("step1_manifest", "step1_rewrite_source", "step1_rewrite_maps",
              "step1_rewrite_strings", "step1_rewrite_imports", "step1_rewrite",
              "step1_leftovers_segments"):
    _load(_name)
manifest = _load("step1_manifest")
imports = _load("step1_rewrite_imports")
leftovers = _load("step1_leftovers")

ROWS = (
    manifest.ManifestRow("__init__.py", "__init__.py", "__init__.py", "stay", False, ""),
    manifest.ManifestRow("legacy/__init__.py", "delete", "delete", "delete", False, ""),
    manifest.ManifestRow("legacy/alpha.py", "fresh/alpha.py", "fresh/alpha.py", "move", False, ""),
    manifest.ManifestRow("legacy/beta.py", "fresh/beta.py", "fresh/beta.py", "move", False, ""),
    manifest.ManifestRow("legacy/deep/gamma.py", "fresh/gamma.py", "fresh/gamma.py", "move",
                         False, ""),
)
# alpha and the deep folder have moved; legacy/ still holds beta.
FILES = ("src/navpy/__init__.py", "src/navpy/legacy/__init__.py", "src/navpy/legacy/beta.py",
         "src/navpy/fresh/alpha.py", "src/navpy/fresh/gamma.py")
STATE = leftovers.tree_state(ROWS, FILES)


def _scan(text: str, importer: object = None) -> list[tuple[int, str, str]]:
    return [(item.line, item.kind, item.detail)
            for item in leftovers.scan_leftovers("tests/t.py", text, STATE, importer)]


def test_code_references_to_a_gone_module_are_leftovers() -> None:
    found = _scan(
        "import navpy.legacy.alpha\n"
        "from navpy.legacy import alpha, beta\n"
        "from navpy.legacy.alpha import A\n"
        "value = navpy.legacy.beta.B\n"
    )

    assert found == [
        (1, "module reference", "navpy.legacy.alpha (navpy.legacy.alpha is gone)"),
        (2, "import", "navpy.legacy.alpha (navpy.legacy.alpha is gone)"),
        (3, "import", "navpy.legacy.alpha.A (navpy.legacy.alpha is gone)"),
    ]


def test_dotted_strings_and_patch_prefixes_split_over_lines_are_leftovers() -> None:
    found = _scan(
        "patch(\n    'navpy.legacy.'\n    'alpha.A')\n"
        "PREFIX = 'navpy.legacy.alpha.'\n"
        "KEPT = 'navpy.legacy.beta.B'\n"
        "NEW = 'navpy.fresh.alpha.A'\n"
    )

    assert [(line, kind) for line, kind, _ in found] == [(2, "dotted string"), (4, "dotted string")]


def test_old_folder_and_file_strings_are_leftovers_once_gone() -> None:
    found = _scan(
        "A = 'src/navpy/legacy/alpha.py'\n"
        "B = 'src/navpy/legacy/deep/'\n"
        "C = 'src/navpy/legacy'\n"
        "D = 'legacy/deep/gamma.py'\n"
        "E = 'legacy/beta.py'\n"
        "F = f'src/navpy/legacy/deep/{name}'\n"
    )

    assert found == [
        (1, "path string", "navpy/legacy/alpha.py"),
        (2, "path string", "navpy/legacy/deep/"),
        (4, "relative path string", "legacy/deep/gamma.py"),
        (6, "path string", "navpy/legacy/deep/"),
    ]


def test_anything_inside_a_gone_old_folder_is_a_leftover() -> None:
    found = _scan(
        "A = 'src/navpy/legacy/deep/new.json'\n"
        "B = 'legacy/deep/notes.txt'\n"
        "C = 'src/navpy/legacy/other.json'\n"
        "D = 'legacy/other.json'\n"
        "E = 'see legacy/deep/notes.txt'\n"
        "F = os.path.join(ROOT, 'src', 'navpy', 'legacy', 'deep', 'new.json')\n"
    )

    assert found == [
        (1, "path string", "navpy/legacy/deep/new.json"),
        (2, "relative path string", "legacy/deep/notes.txt"),
        (6, "segment-built path", "navpy/legacy/deep/new.json"),
    ]


def test_paths_inside_a_gone_top_level_old_folder_are_leftovers() -> None:
    """legacy/ is a single-segment old folder: once it is gone, a path under
    src/navpy or after a "navpy" segment is a leftover. A bare relative
    "legacy/..." string is not judged: "utils/geo.js", "tools/cam/x.py" and
    "modules/mavlink_c_library_v2/..." name other trees in this repository."""
    files = ("src/navpy/__init__.py", "src/navpy/fresh/alpha.py", "src/navpy/fresh/beta.py",
             "src/navpy/fresh/gamma.py")
    state = leftovers.tree_state(ROWS, files)
    text = (
        "A = 'src/navpy/legacy/new.json'\n"
        "B = os.path.join(ROOT, 'src', 'navpy', 'legacy', 'new.json')\n"
        "C = ROOT / 'src' / 'navpy' / 'legacy'\n"
        "D = 'legacy'\n"
        "E = 'legacy/new.json'\n"
    )

    found = [(item.line, item.kind)
             for item in leftovers.scan_leftovers("tests/t.py", text, state, None)]

    assert found == [(1, "path string"), (2, "segment-built path"), (3, "segment-built path")]


def test_a_patch_target_joined_from_an_f_string_and_a_literal_is_a_leftover() -> None:
    found = _scan(
        "patch(f'navpy.legacy.' 'alpha.A')\n"
        "patch(f'navpy.{kind}.' 'alpha.A')\n"
        "patch(f'navpy.legacy.' 'beta.B')\n"
    )

    assert found == [(1, "dotted string", "navpy.legacy.alpha.A (navpy.legacy.alpha is gone)")]


@pytest.mark.parametrize("built", [
    'os.path.join(ROOT, "src", "navpy", "legacy", "alpha.py")',
    'Path(__file__).parents[3] / "src" / "navpy" / "legacy" / "deep"',
    'ROOT.joinpath("src/navpy", "legacy", "alpha.py")',
    'Path(ROOT, "src", "navpy", "legacy", "deep", name)',
    'Path(ROOT, "src", "navpy").joinpath("legacy", "alpha.py")',
    'Path(ROOT, "src", "navpy") / "legacy" / "alpha.py"',
    '(ROOT / "src" / "navpy").joinpath("legacy", "deep")',
    'os.path.join(Path(ROOT, "src", "navpy"), "legacy", "alpha.py")',
])
def test_segment_built_paths_to_gone_locations_are_leftovers(built: str) -> None:
    found = _scan(f"PATH = {built}\n")

    assert [(line, kind) for line, kind, _ in found] == [(1, "segment-built path")]


@pytest.mark.parametrize("built", [
    'os.path.join(ROOT, "src", "navpy", "legacy", "beta.py")',
    'ROOT / "src" / "navpy" / "legacy"',
    'ROOT / "src" / "navpy" / folder / "alpha.py"',
    '"/".join(["src", "navpy", "legacy", "alpha.py"])',
])
def test_segment_built_paths_still_present_or_not_literal_pass(built: str) -> None:
    assert _scan(f"PATH = {built}\n") == []


def test_relative_imports_are_judged_from_the_file_location() -> None:
    importer = imports.Importer("navpy.legacy.beta", "navpy.legacy.beta", False)

    found = _scan("from .alpha import A\nfrom . import beta\n", importer)

    assert found == [(1, "import", "navpy.legacy.alpha.A (navpy.legacy.alpha is gone)")]


def test_unparsable_files_naming_navpy_are_reported() -> None:
    assert _scan("def broken(:\n    navpy\n")[0][1] == "unparsable"
    assert _scan("def broken(:\n") == []
