"""Repository-level SOLID architecture gates."""

from __future__ import annotations

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


REPO_ROOT = Path(__file__).resolve().parents[2]


@cache
def _current() -> dict[str, str]:
    return read_current_sources(REPO_ROOT)


@cache
def _pinned() -> dict[str, str]:
    return read_pinned_sources(REPO_ROOT)


def test_all_navpy_classes_stay_within_solid_limits() -> None:
    violations = scan_global_class_limits(_current())
    assert not violations, format_violations(violations)


def test_material_modules_and_top_level_functions_stay_bounded() -> None:
    violations = scan_material_limits(_current(), _pinned())
    assert not violations, format_violations(violations)


def test_new_production_definitions_are_fully_annotated() -> None:
    violations = scan_new_definition_annotations(_current(), _pinned())
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
        "class TerminalVisionFrame:\n"
        f"{fields}\n"
        "    @property\n"
        "    def sneaky(self) -> float:\n"
        '        object.__setattr__(self, "field_0", 999.0)\n'
        "        return self.field_0\n"
    )
    path = "src/navpy/modules/navigation/nav/vision_nav/frame.py"
    violations = scan_global_class_limits({path: source})
    assert any(violation.code == "owned-fields" for violation in violations)
