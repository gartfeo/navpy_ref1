"""Pure-move split check for the step-1 SOLID normalizer.

A split part, or the rest of its source, holds only old code when its
statements are exactly the base statements it owns, in their base order, no
more, no fewer and none twice: a part owns the statements that define its
symbols; the rest owns every statement no existing part claims (all of them
before c02 makes the parts). A part taking code from several sources holds
it source by source, in the order splits.json lists the sources. It keeps
`from __future__ import annotations` when a source had it, imports only names
its sources imported or defined, and still binds every such name its code
reads to what the base bound it to. The module docstring and `__all__` are not
compared.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass
from typing import Collection, Dict, Iterable, List, Mapping, Set

from tests.guards.solid_step1_relocation import Piece

FUTURE_ANNOTATIONS = "__future__.annotations"


@dataclass(frozen=True)
class SplitView:
    """A module's imports, as (dump, imported dotted name, bound name), and
    its other top-level statements in order; `if TYPE_CHECKING:` imports count
    as imports."""

    imports: frozenset[tuple[str, str, str]]
    statements: tuple[ast.stmt, ...]

    @property
    def has_future_annotations(self) -> bool:
        return any(target == FUTURE_ANNOTATIONS for _, target, _ in self.imports)

    def bindings(self) -> Dict[str, Set[str]]:
        """Bound name -> the dotted names its imports bind it to."""
        bound: Dict[str, Set[str]] = {}
        for _, target, name in self.imports:
            bound.setdefault(name, set()).add(target)
        return bound


def split_view(tree: ast.Module) -> SplitView:
    imports: set[tuple[str, str, str]] = set()
    statements: list[ast.stmt] = []
    for index, statement in enumerate(tree.body):
        if (index == 0 and _is_docstring(statement)) or "__all__" in _assigned(statement):
            continue
        block = _imports_of(statement)
        if block is None:
            statements.append(statement)
            continue
        for item in block:
            alias = item.names[0]
            if isinstance(item, ast.ImportFrom):
                target, bound = f"{item.module}.{alias.name}", alias.asname or alias.name
            else:
                target, bound = alias.name, alias.asname or alias.name.split(".")[0]
            imports.add((ast.dump(item), target, bound))
    return SplitView(frozenset(imports), tuple(statements))


def claimed_symbols(
    parts: Mapping[str, tuple[Piece, ...]], old_path: str, present: Collection[str]
) -> frozenset[str]:
    """The top-level names the split parts present in the tree took from old_path."""
    return frozenset(
        symbol
        for path, pieces in parts.items()
        if path in present
        for piece in pieces
        if piece.old_path == old_path and piece.symbols is not None
        for symbol in piece.symbols
    )


def owned_statements(piece: Piece, base: SplitView, claimed: frozenset[str]) -> List[str]:
    """Dumps of the base statements the piece's file must hold, in base order."""
    symbols = claimed if piece.symbols is None else piece.symbols
    return dumps(s for s in base.statements
                 if bool(defined_names(s) & symbols) == (piece.symbols is not None))


def dumps(statements: Iterable[ast.stmt]) -> List[str]:
    return [ast.dump(statement) for statement in statements]


def defined_names(statement: ast.stmt) -> set[str]:
    if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
        return {statement.name}
    return _assigned(statement)


def _assigned(statement: ast.stmt) -> set[str]:
    targets = (
        statement.targets if isinstance(statement, ast.Assign)
        else [statement.target] if isinstance(statement, (ast.AnnAssign, ast.AugAssign))
        else []
    )
    return {node.id for target in targets for node in ast.walk(target)
            if isinstance(node, ast.Name)}


def _imports_of(statement: ast.stmt) -> list[ast.Import | ast.ImportFrom] | None:
    if isinstance(statement, (ast.Import, ast.ImportFrom)):
        return [statement]
    if (
        isinstance(statement, ast.If)
        and _dotted(statement.test) in {"TYPE_CHECKING", "typing.TYPE_CHECKING"}
        and not statement.orelse
        and all(isinstance(item, (ast.Import, ast.ImportFrom)) for item in statement.body)
    ):
        return list(statement.body)
    return None


def _is_docstring(statement: ast.stmt) -> bool:
    return (
        isinstance(statement, ast.Expr)
        and isinstance(statement.value, ast.Constant)
        and isinstance(statement.value.value, str)
    )


def _dotted(node: ast.AST) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        prefix = _dotted(node.value)
        return f"{prefix}.{node.attr}" if prefix else None
    return None
