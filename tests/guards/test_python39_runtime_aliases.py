"""Repository-wide Python 3.9 guards for evaluated typing expressions."""

from __future__ import annotations

import ast
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_ROOTS = (
    "src/navpy",
    "src/gcs",
    "scripts",
    "tools",
)


def _runtime_paths() -> tuple[Path, ...]:
    return tuple(sorted(
        path
        for relative_root in RUNTIME_ROOTS
        for path in (REPO_ROOT / relative_root).rglob("*.py")
    ))


def _has_postponed_annotations(tree: ast.Module) -> bool:
    return any(
        isinstance(statement, ast.ImportFrom)
        and statement.module == "__future__"
        and any(alias.name == "annotations" for alias in statement.names)
        for statement in tree.body
    )


def _annotation_nodes(tree: ast.Module) -> list[ast.AST]:
    annotations: list[ast.AST] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.arg) and node.annotation is not None:
            annotations.append(node.annotation)
        elif (
            isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.returns is not None
        ):
            annotations.append(node.returns)
        elif isinstance(node, ast.AnnAssign):
            annotations.append(node.annotation)
    return annotations


def _pep604_lines(nodes: list[ast.AST]) -> list[int]:
    return sorted({
        candidate.lineno
        for node in nodes
        for candidate in ast.walk(node)
        if isinstance(candidate, ast.BinOp)
        and isinstance(candidate.op, ast.BitOr)
    })


def _assigned_names(statement: ast.Assign | ast.AnnAssign) -> list[str]:
    targets = (
        statement.targets
        if isinstance(statement, ast.Assign)
        else [statement.target]
    )
    return [target.id for target in targets if isinstance(target, ast.Name)]


def _runtime_union_aliases(tree: ast.Module) -> list[tuple[int, str]]:
    aliases: list[tuple[int, str]] = []
    for statement in tree.body:
        if not isinstance(statement, (ast.Assign, ast.AnnAssign)):
            continue
        value = statement.value
        if value is None or not _pep604_lines([value]):
            continue
        names = _assigned_names(statement)
        if names and all(name.isupper() for name in names):
            # Uppercase assignments are runtime bitmask/constants, not aliases.
            continue
        aliases.extend((statement.lineno, name) for name in names)
    return aliases


def test_runtime_typing_expressions_support_python39() -> None:
    violations: list[str] = []
    for path in _runtime_paths():
        relative = path.relative_to(REPO_ROOT).as_posix()
        tree = ast.parse(
            path.read_text(encoding="utf-8-sig"),
            filename=relative,
        )
        type_alias_imports = [
            alias
            for statement in tree.body
            if isinstance(statement, ast.ImportFrom)
            and statement.module == "typing"
            for alias in statement.names
            if alias.name == "TypeAlias"
        ]
        violations.extend(
            f"{relative}:{alias.lineno} imports typing.TypeAlias"
            for alias in type_alias_imports
        )
        violations.extend(
            f"{relative}:{line} evaluates PEP 604 alias {name}"
            for line, name in _runtime_union_aliases(tree)
        )
        if not _has_postponed_annotations(tree):
            violations.extend(
                f"{relative}:{line} evaluates a PEP 604 annotation"
                for line in _pep604_lines(_annotation_nodes(tree))
            )

    assert not violations, (
        "Python 3.9-incompatible runtime typing expressions:\n  "
        + "\n  ".join(violations)
    )


def test_runtime_alias_scan_distinguishes_type_alias_from_bitmask() -> None:
    source = "Alias = str | None\nMASK_VALUE = LEFT | RIGHT\n"

    assert _runtime_union_aliases(ast.parse(source)) == [(1, "Alias")]
