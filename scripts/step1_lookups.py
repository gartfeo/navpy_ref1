"""Where a split part looks up the names its source imported: patch targets.

mock.patch replaces a name in the module that looks it up. When c02 moves
code out of a split source, a name that source imported is looked up by the
output whose code uses it: `real_detector_models.create_tracker_backend` must
be patched on the builder part once `build_models` lives there, while an
import of the same name still follows its defining module. These lookups feed
RewriteMaps.lookups; a name several outputs use cannot be routed (None).
"""

from __future__ import annotations

import ast
from typing import Dict, Iterable, Mapping, Optional, Tuple

from step1_manifest import SplitPart, Stage, module_name, repo_path

Lookups = Dict[Tuple[str, str], Optional[str]]
_ANNOTATION_FIELDS = frozenset({"annotation", "returns"})


def split_lookups(sources: Mapping[str, str], splits: Mapping[str, tuple[SplitPart, ...]],
                  stage: Stage) -> Lookups:
    """(old module, imported name) -> module of the split part whose code
    uses it, or None when several outputs (parts or the rest) use it. Names
    only the rest uses keep their module and are left out. sources: text of
    each split source, keyed by its manifest path."""
    lookups: Lookups = {}
    for src, parts in splits.items():
        old = module_name(repo_path(src) or "")
        if old is None:
            raise ValueError(f"split source {src} is not a module")
        owners = {symbol: module_name(repo_path(part.dest(stage)) or "") or ""
                  for part in parts for symbol in part.symbols}
        tree = ast.parse(sources[src])
        users = _users(tree, owners)
        for name in _imported_names(tree.body):
            found = users.get(name, set())
            if found and found != {None}:
                lookups[(old, name)] = next(iter(found)) if len(found) == 1 else None
    return lookups


def _users(tree: ast.Module, owners: Mapping[str, str]) -> dict[str, set[str | None]]:
    """Name -> the outputs whose functions read it (None: the rest)."""
    users: dict[str, set[str | None]] = {}
    for statement in tree.body:
        if isinstance(statement, (ast.Import, ast.ImportFrom)) or _is_import_block(statement):
            continue
        homes = {owners[name] for name in _defined(statement) if name in owners}
        owner = homes.pop() if len(homes) == 1 else None
        for name in _call_time_reads(statement):
            users.setdefault(name, set()).add(owner)
    return users


def _call_time_reads(node: ast.AST, inside: bool = False) -> Iterable[str]:
    """Names the code reads while a function runs, which is when a patch is
    in place; import-time reads (aliases, bases, defaults, decorators) and
    annotations are not."""
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)) and not inside:
        body = node.body if isinstance(node.body, list) else [node.body]
        for child in body:
            yield from _call_time_reads(child, True)
        return
    if inside and isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load):
        yield node.id
    for field, value in ast.iter_fields(node):
        if field in _ANNOTATION_FIELDS:
            continue
        for child in value if isinstance(value, list) else [value]:
            if isinstance(child, ast.AST):
                yield from _call_time_reads(child, inside)


def _defined(statement: ast.stmt) -> set[str]:
    if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
        return {statement.name}
    targets: list[ast.expr] = []
    if isinstance(statement, ast.Assign):
        targets = list(statement.targets)
    elif isinstance(statement, (ast.AnnAssign, ast.AugAssign)):
        targets = [statement.target]
    return {node.id for target in targets for node in ast.walk(target)
            if isinstance(node, ast.Name)}


def _is_import_block(statement: ast.stmt) -> bool:
    body = getattr(statement, "body", None)
    return isinstance(statement, (ast.If, ast.Try)) and bool(body) and all(
        isinstance(child, (ast.Import, ast.ImportFrom)) for child in body)


def _imported_names(statements: Iterable[ast.stmt]) -> Iterable[str]:
    """Names bound by module-level imports, through if/try blocks."""
    for statement in statements:
        if isinstance(statement, ast.ImportFrom):
            yield from (alias.asname or alias.name for alias in statement.names)
        elif isinstance(statement, ast.Import):
            yield from (alias.asname or alias.name.split(".")[0] for alias in statement.names)
        elif isinstance(statement, (ast.If, ast.Try)):
            handlers = [child for handler in getattr(statement, "handlers", [])
                        for child in handler.body]
            yield from _imported_names([*statement.body, *statement.orelse, *handlers])
