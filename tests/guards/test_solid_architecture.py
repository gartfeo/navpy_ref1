"""Repository-level SOLID architecture gates."""

from __future__ import annotations

import ast
from functools import cache
from pathlib import Path

from tests.guards.solid_architecture_guard import (
    format_violations,
    read_current_sources,
    read_pinned_sources,
    scan_global_class_limits,
    scan_material_limits,
    scan_new_definition_annotations,
)
from tests.guards.solid_step1_normalizer import MovedBaseline
from tests.guards.solid_step1_relocation import Relocation, load_relocation, module_level


REPO_ROOT = Path(__file__).resolve().parents[2]


@cache
def _current() -> dict[str, str]:
    return read_current_sources(REPO_ROOT)


@cache
def _pinned() -> dict[str, str]:
    return read_pinned_sources(REPO_ROOT)


@cache
def _relocation() -> Relocation:
    return load_relocation(REPO_ROOT)


def test_all_navpy_classes_stay_within_solid_limits() -> None:
    violations = scan_global_class_limits(_current())
    assert not violations, format_violations(violations)


def test_material_modules_and_top_level_functions_stay_bounded() -> None:
    violations = scan_material_limits(
        _current(), _pinned(), relocation=_relocation()
    )
    assert not violations, format_violations(violations)


def test_new_production_definitions_are_fully_annotated() -> None:
    violations = scan_new_definition_annotations(
        _current(), _pinned(), relocation=_relocation()
    )
    assert not violations, format_violations(violations)


def test_passive_record_allowance_rejects_mutation_escapes() -> None:
    """The decorator is not the contract -- the body must not mutate.

    Found by review: a `@property` on a frozen passive record could call
    `object.__setattr__` and the allowance still applied, because only the
    decorator was inspected. A record with a reachable mutation escape must
    fall back to the strict field limit.
    """
    fields = "\n".join(f"    field_{index}: float" for index in range(15))
    source = (
        "from dataclasses import dataclass\n"
        "@dataclass(frozen=True)\n"
        "class FinalApproachVisionFrame:\n"
        f"{fields}\n"
        "    @property\n"
        "    def sneaky(self) -> float:\n"
        '        object.__setattr__(self, "field_0", 999.0)\n'
        "        return self.field_0\n"
    )
    path = "src/navpy/modules/navigation/nav/vision_nav/frame.py"
    violations = scan_global_class_limits({path: source})
    assert any(violation.code == "owned-fields" for violation in violations)


def test_step1_relocation_covers_the_whole_plan() -> None:
    """The move map the material and annotation gates compare through holds
    every moved row, split part and stripped re-export of the step-1 plan."""
    relocation = _relocation()
    assert len(relocation.paths) == 719  # 705 move, 12 split, 2 gcs rows
    assert len(relocation.modules) == 718  # all but vision_profiles.json
    assert len(relocation.parts) == 12
    assert sum(len(pieces) for pieces in relocation.parts.values()) == 13
    assert len(relocation.stripped) == 29
    assert sum(len(names) for names in relocation.stripped.values()) == 95
    assert len(relocation.directories) == 15


def test_every_listed_re_export_is_stripped_from_its_pinned_module() -> None:
    """Each reexports.tsv row matches a module-level import of the pinned
    base by name and origin, so the normalizer strips exactly it."""
    baseline = MovedBaseline(_pinned(), _relocation())
    kept = []
    for path, bindings in _relocation().stripped.items():
        tree = baseline._canonical(_pinned()[path], path, bindings)
        bound = {
            alias.asname or alias.name
            for statement in module_level(tree.body)
            if isinstance(statement, ast.ImportFrom)
            for alias in statement.names
        }
        kept += [f"{path}: {name}" for name, _ in bindings if name in bound]
    assert not kept, kept
