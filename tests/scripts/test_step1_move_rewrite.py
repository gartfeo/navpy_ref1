"""The step-1 rewriter on small sources: every reference form a move renames.

Synthetic modules live under navpy.legacy (old) and navpy.fresh (new); no
real manifest module is named, so a real step-1 run leaves this file alone
(the step-1 tools skip their own tests anyway).
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


for _name in ("step1_rewrite_source", "step1_rewrite_maps", "step1_rewrite_strings",
              "step1_rewrite_imports"):
    _load(_name)
maps_module = _load("step1_rewrite_maps")
strings = _load("step1_rewrite_strings")
imports = _load("step1_rewrite_imports")
rewrite = _load("step1_rewrite")

KNOWN = frozenset({
    "navpy", "navpy.legacy", "navpy.legacy.alpha", "navpy.legacy.beta", "navpy.legacy.gamma",
    "navpy.legacy.pkg", "navpy.legacy.pkg.inner", "navpy.legacy.pkg.side",
})
MAPS = maps_module.RewriteMaps(
    modules={"navpy.legacy.alpha": "navpy.fresh.kernel.alpha",
             "navpy.legacy.gamma": "navpy.fresh.kernel.gamma2"},
    known=KNOWN,
    paths={"navpy/legacy/alpha.py": "navpy/fresh/kernel/alpha.py"},
)


def _rewrite(text: str, maps: object = MAPS, importer: object = None) -> str:
    result = rewrite.rewrite_source(text, maps, importer)
    assert result.problems == ()
    return result.text


def test_imports_and_dotted_code_names_follow_the_moved_module() -> None:
    source = (
        "import navpy.legacy.alpha\n"
        "import navpy.legacy.alpha as a_mod\n"
        "from navpy.legacy.alpha import A, B as C\n"
        "value = navpy.legacy.alpha.A.method()\n"
        "other = navpy.legacy.beta.B\n"
    )

    assert _rewrite(source) == (
        "import navpy.fresh.kernel.alpha\n"
        "import navpy.fresh.kernel.alpha as a_mod\n"
        "from navpy.fresh.kernel.alpha import A, B as C\n"
        "value = navpy.fresh.kernel.alpha.A.method()\n"
        "other = navpy.legacy.beta.B\n"
    )


def test_crlf_line_endings_and_untouched_text_are_kept_byte_for_byte() -> None:
    source = "# keep\r\nfrom navpy.legacy.alpha import A  # trailing\r\n\r\nx = 'navpy.legacy.beta'\r\n"

    assert _rewrite(source) == (
        "# keep\r\nfrom navpy.fresh.kernel.alpha import A  # trailing\r\n\r\nx = 'navpy.legacy.beta'\r\n"
    )


def test_from_package_import_follows_each_submodule() -> None:
    source = (
        "def build() -> None:\n"
        "    from navpy.legacy import alpha, beta as b\n"
        "    from navpy.legacy import gamma\n"
        "from navpy.legacy import (\n"
        "    alpha,\n"
        "    beta,\n"
        "    gamma,\n"
        ")\n"
    )

    assert _rewrite(source) == (
        "def build() -> None:\n"
        "    from navpy.fresh.kernel import alpha\n"
        "    from navpy.legacy import beta as b\n"
        "    from navpy.fresh.kernel import gamma2 as gamma\n"
        "from navpy.fresh.kernel import alpha\n"
        "from navpy.legacy import beta\n"
        "from navpy.fresh.kernel import gamma2 as gamma\n"
    )


def test_a_split_import_keeps_the_binding_and_import_order() -> None:
    """alpha and gamma share a new home, beta stays: the last `x` is still
    gamma's, and the modules still load alpha, beta, gamma."""
    source = (
        "from navpy.legacy import alpha as x, beta as x, gamma as x\n"
        "from navpy.legacy import (\n"
        "    alpha,\n"
        "    gamma,\n"
        "    beta,\n"
        ")\n"
    )

    assert _rewrite(source) == (
        "from navpy.fresh.kernel import alpha as x\n"
        "from navpy.legacy import beta as x\n"
        "from navpy.fresh.kernel import gamma2 as x\n"
        "from navpy.fresh.kernel import (\n"
        "    alpha,\n"
        "    gamma2 as gamma,\n"
        ")\n"
        "from navpy.legacy import beta\n"
    )


def test_an_import_right_after_a_dedent_keeps_its_from_keyword() -> None:
    source = "def f() -> None:\n    if x:\n        y = 1\n    from navpy.legacy.alpha import A\n"

    assert _rewrite(source) == (
        "def f() -> None:\n    if x:\n        y = 1\n    from navpy.fresh.kernel.alpha import A\n"
    )


def test_a_split_import_on_a_shared_line_stays_on_it() -> None:
    assert _rewrite("if True: from navpy.legacy import alpha, beta\n") == (
        "if True: from navpy.fresh.kernel import alpha; from navpy.legacy import beta\n"
    )


def test_a_submodule_moving_later_keeps_its_name_while_its_package_moves() -> None:
    maps = maps_module.RewriteMaps(modules={"navpy.legacy.pkg": "navpy.fresh.pkg"}, known=KNOWN,
                                   paths={})
    source = (
        "from navpy.legacy.pkg import inner, Thing\n"
        "import navpy.legacy.pkg.side\n"
        "patch('navpy.legacy.pkg.Thing')\n"
        "patch('navpy.legacy.pkg.inner.Thing')\n"
    )

    assert _rewrite(source, maps) == (
        "from navpy.legacy.pkg import inner\n"
        "from navpy.fresh.pkg import Thing\n"
        "import navpy.legacy.pkg.side\n"
        "patch('navpy.fresh.pkg.Thing')\n"
        "patch('navpy.legacy.pkg.inner.Thing')\n"
    )


def test_patch_target_split_over_lines_is_renamed_across_its_pieces() -> None:
    source = (
        "with patch(\n"
        "    \"navpy.legacy.\"\n"
        "    \"alpha.TIMEOUT_S\",\n"
        "    0.01,\n"
        "), patch(\"navpy.leg\" \"acy.alpha.x\"):\n"
        "    pass\n"
    )

    assert _rewrite(source) == (
        "with patch(\n"
        "    \"navpy.fresh.kernel.\"\n"
        "    \"alpha.TIMEOUT_S\",\n"
        "    0.01,\n"
        "), patch(\"navpy.fresh.kernel\" \".alpha.x\"):\n"
        "    pass\n"
    )


def test_escaped_raw_and_docstring_literals_are_edited_in_place() -> None:
    source = (
        '"""Run as python -m navpy.legacy.alpha."""\n'
        "a = 'tab\\tnavpy.legacy.alpha.X'\n"
        "b = r'raw\\navpy.legacy.alpha'\n"
        "c = 'x\\u00e9 navpy.legacy.alpha'\n"
    )

    assert _rewrite(source) == (
        '"""Run as python -m navpy.fresh.kernel.alpha."""\n'
        "a = 'tab\\tnavpy.fresh.kernel.alpha.X'\n"
        "b = r'raw\\navpy.legacy.alpha'\n"
        "c = 'x\\u00e9 navpy.fresh.kernel.alpha'\n"
    )


def test_bytes_and_f_strings_are_never_edited() -> None:
    source = "a = f'navpy.legacy.alpha.{x}'\nb = b'navpy.legacy.alpha'\n"

    assert _rewrite(source) == source


def test_path_strings_and_comments_follow_the_moved_file() -> None:
    source = (
        "# see navpy.legacy.alpha in src/navpy/legacy/alpha.py\n"
        "PATH = 'src/navpy/legacy/alpha.py::test_x'\n"
        "FOLDER = 'src/navpy/legacy/'\n"
    )

    assert _rewrite(source) == (
        "# see navpy.fresh.kernel.alpha in src/navpy/fresh/kernel/alpha.py\n"
        "PATH = 'src/navpy/fresh/kernel/alpha.py::test_x'\n"
        "FOLDER = 'src/navpy/legacy/'\n"
    )


def test_relative_imports_stay_while_they_resolve_and_turn_absolute_otherwise() -> None:
    maps = maps_module.RewriteMaps(
        modules={"navpy.legacy.pkg.inner": "navpy.fresh.pkg.inner",
                 "navpy.legacy.pkg.side": "navpy.fresh.pkg.side",
                 "navpy.legacy.alpha": "navpy.fresh.kernel.alpha"},
        known=KNOWN, paths={},
    )
    moved = imports.Importer("navpy.legacy.pkg.inner", "navpy.fresh.pkg.inner", False)
    source = "from .side import S\nfrom ..alpha import A\nfrom ..beta import B\nfrom . import side\n"

    assert _rewrite(source, maps, moved) == (
        "from .side import S\n"
        "from navpy.fresh.kernel.alpha import A\n"
        "from navpy.legacy.beta import B\n"
        "from . import side\n"
    )


def test_a_relative_import_of_a_moved_module_from_a_staying_file_turns_absolute() -> None:
    staying = imports.Importer("navpy.legacy.pkg", "navpy.legacy.pkg", True)
    source = "from .. import alpha\nfrom .side import S\nfrom ..beta import B\n"

    assert _rewrite(source, MAPS, staying) == (
        "from navpy.fresh.kernel import alpha\nfrom .side import S\nfrom ..beta import B\n"
    )


def test_symbols_follow_their_defining_module_for_splits_and_facades() -> None:
    maps = maps_module.RewriteMaps(
        modules={}, known=KNOWN, paths={},
        symbols={("navpy.legacy.beta", "Detection"): "navpy.fresh.ports.detection"},
    )
    source = (
        "from navpy.legacy.beta import Detection, Yolo\n"
        "patch('navpy.legacy.beta.Detection.score')\n"
        "patch('navpy.legacy.beta.Yolo')\n"
    )

    assert _rewrite(source, maps) == (
        "from navpy.fresh.ports.detection import Detection\n"
        "from navpy.legacy.beta import Yolo\n"
        "patch('navpy.fresh.ports.detection.Detection.score')\n"
        "patch('navpy.legacy.beta.Yolo')\n"
    )


def test_a_patch_target_follows_the_split_part_that_looks_the_name_up() -> None:
    maps = maps_module.RewriteMaps(
        modules={}, known=KNOWN, paths={},
        symbols={("navpy.legacy.beta", "build"): "navpy.fresh.builder"},
        lookups={("navpy.legacy.beta", "Yolo"): "navpy.fresh.builder",
                 ("navpy.legacy.beta", "np"): None},
    )
    source = (
        "from navpy.legacy.beta import Yolo, build\n"
        "patch('navpy.legacy.beta.Yolo')\n"
        "patch('navpy.legacy.beta.'\n"
        "      'Yolo', return_value=None)\n"
        "patch('navpy.legacy.beta.build')\n"
    )

    assert _rewrite(source, maps) == (
        "from navpy.legacy.beta import Yolo\n"
        "from navpy.fresh.builder import build\n"
        "patch('navpy.fresh.builder.Yolo')\n"
        "patch('navpy.fresh.builder.'\n"
        "      'Yolo', return_value=None)\n"
        "patch('navpy.fresh.builder.build')\n"
    )
    unroutable = rewrite.rewrite_source("patch('navpy.legacy.beta.np')\n", maps)
    assert unroutable.text == "patch('navpy.legacy.beta.np')\n"
    assert len(unroutable.problems) == 1 and "navpy.legacy.beta.np" in unroutable.problems[0]


def test_a_qualified_reference_to_a_moved_name_is_left_for_a_hand_fix() -> None:
    maps = maps_module.RewriteMaps(
        modules={"navpy.legacy.beta": "navpy.fresh.beta"}, known=KNOWN, paths={},
        symbols={("navpy.legacy.beta", "Detection"): "navpy.fresh.ports.detection"},
    )
    source = "import navpy.legacy.beta\nvalue = navpy.legacy.beta.Detection\n"

    result = rewrite.rewrite_source(source, maps)

    assert result.text == "import navpy.fresh.beta\nvalue = navpy.legacy.beta.Detection\n"
    assert len(result.problems) == 1 and "line 2" in result.problems[0]
    assert _rewrite("value = navpy.legacy.beta.Other\n", maps) == (
        "value = navpy.fresh.beta.Other\n")


def test_a_moved_name_through_a_module_alias_is_left_for_a_hand_fix() -> None:
    maps = maps_module.RewriteMaps(
        modules={"navpy.legacy.beta": "navpy.fresh.beta"}, known=KNOWN, paths={},
        symbols={("navpy.legacy.beta", "Detection"): "navpy.fresh.ports.detection"},
        lookups={("navpy.legacy.beta", "np"): None},
    )
    source = (
        "import navpy.legacy.beta as det\n"
        "from navpy.legacy import beta\n"
        "from navpy.legacy import beta as b2\n"
        "one = det.Detection\n"
        "two = beta.Detection.score\n"
        "three = b2.np\n"
        "kept = det.Other\n"
    )

    result = rewrite.rewrite_source(source, maps)

    assert result.text == (
        "import navpy.fresh.beta as det\n"
        "from navpy.fresh import beta\n"
        "from navpy.fresh import beta as b2\n"
        "one = det.Detection\n"
        "two = beta.Detection.score\n"
        "three = b2.np\n"
        "kept = det.Other\n"
    )
    assert [problem.split(":")[0] for problem in result.problems] == [
        "line 4", "line 5", "line 6"]
    assert "navpy.legacy.beta.Detection" in result.problems[0]


def test_an_import_whose_split_would_drop_a_comment_is_left_for_a_hand_fix() -> None:
    source = "from navpy.legacy import (\n    alpha,  # moved\n    beta,\n)\n"

    result = rewrite.rewrite_source(source, MAPS)

    assert result.text == source
    assert len(result.problems) == 1 and "line 1" in result.problems[0]


def test_a_rewrite_that_would_not_parse_is_left_for_a_hand_fix() -> None:
    maps = maps_module.RewriteMaps(modules={"navpy.legacy.alpha": "navpy.fresh.1bad"},
                                   known=KNOWN, paths={})
    source = "import navpy.legacy.alpha\n"

    result = rewrite.rewrite_source(source, maps)

    assert result.text == source
    assert result.edits == 0 and "would not parse" in result.problems[0]


def test_unparsable_text_is_a_problem_only_when_it_names_navpy() -> None:
    assert rewrite.rewrite_source("def broken(:\n  navpy.legacy.alpha\n", MAPS).problems
    assert rewrite.rewrite_source("def broken(:\n", MAPS).problems == ()


def test_maps_reject_moving_modules_outside_the_known_set() -> None:
    with pytest.raises(ValueError, match="navpy.elsewhere"):
        maps_module.RewriteMaps(modules={"navpy.elsewhere": "navpy.fresh"}, known=KNOWN, paths={})


@pytest.mark.parametrize(("old", "new", "cuts", "parts"), [
    ("navpy.legacy.alpha.A", "navpy.fresh.kernel.alpha.A", [13],
     ["navpy.fresh.kernel.", "alpha.A"]),
    ("navpy.legacy.alpha.A", "navpy.fresh.kernel.alpha.A", [3], ["nav", "py.fresh.kernel.alpha.A"]),
    ("navpy.legacy.alpha", "navpy.fresh.kernel.alpha", [9], ["navpy.fresh.kernel", ".alpha"]),
    ("abc", "xyz", [1, 2], ["xyz", "", ""]),
])
def test_split_replacement_keeps_shared_ends_with_their_piece(
    old: str, new: str, cuts: list[int], parts: list[str]
) -> None:
    assert strings.split_replacement(old, new, cuts) == parts
    assert "".join(parts) == new
