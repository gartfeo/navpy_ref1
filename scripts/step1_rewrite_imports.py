"""from-imports for the step-1 rewriter, absolute and relative.

Every imported name is followed on its own: a submodule to its new package,
another name to the module that defines it now. When all names still come
from one module, only the module text changes; otherwise the statement is
rewritten as one statement per run of consecutive names from one module, so
the names keep their binding order. A relative import stays as it
is while it still resolves to the same modules from the file's new package,
and becomes absolute when it does not.
"""

from __future__ import annotations

import ast
import tokenize
from dataclasses import dataclass
from itertools import groupby
from operator import attrgetter

from step1_rewrite_maps import ROOT_PACKAGE, RewriteMaps, rename_module
from step1_rewrite_source import Edit, Source


@dataclass(frozen=True)
class Importer:
    """The rewritten file as a module: its dotted name before and after the
    run, and whether it is a package __init__."""

    old: str
    new: str
    is_package: bool

    def package(self, stage: str) -> str:
        module = self.old if stage == "old" else self.new
        return module if self.is_package else module.rpartition(".")[0]


@dataclass(frozen=True)
class _Binding:
    module: str
    name: str
    asname: str | None


def resolve_relative(package: str, level: int, module: str | None) -> str | None:
    """Absolute module of `from <level dots><module> import` in package."""
    parts = package.split(".") if package else []
    if level - 1 >= len(parts):
        return None
    base = parts[:len(parts) - (level - 1)]
    return ".".join(base + ([module] if module else []))


def import_target(node: ast.ImportFrom, importer: Importer | None, stage: str) -> str | None:
    """The module a from-import names, resolved for relative imports."""
    if node.level == 0:
        return node.module
    if importer is None:
        return None
    return resolve_relative(importer.package(stage), node.level, node.module)


def import_edits(
    source: Source, importer: Importer | None, maps: RewriteMaps
) -> tuple[list[Edit], list[str]]:
    """Edits for every navpy from-import the maps change; problems found."""
    edits: list[Edit] = []
    problems: list[str] = []
    for node in ast.walk(source.tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        target = import_target(node, importer, "old")
        if target is None or target.split(".")[0] != ROOT_PACKAGE:
            continue
        bindings = [_binding(target, alias, maps) for alias in node.names]
        written = import_target(node, importer, "new")
        if _unchanged(node, written, bindings):
            continue
        edit = _edit(source, node, bindings)
        if edit is None:
            problems.append(f"line {node.lineno}: from-import with comments needs a split by hand")
        else:
            edits.append(edit)
    return edits, problems


def _binding(target: str, alias: ast.alias, maps: RewriteMaps) -> _Binding:
    full = f"{target}.{alias.name}"
    if alias.name != "*" and full in maps.known:
        parent, _, leaf = rename_module(full, maps).rpartition(".")
        bound = alias.asname or alias.name
        return _Binding(parent, leaf, None if bound == leaf else bound)
    home = maps.symbols.get((target, alias.name)) or rename_module(target, maps)
    return _Binding(home, alias.name, _asname(alias))


def _asname(alias: ast.alias) -> str | None:
    return None if alias.asname == alias.name else alias.asname


def _unchanged(node: ast.ImportFrom, written: str | None, bindings: list[_Binding]) -> bool:
    return all(
        (binding.module, binding.name, binding.asname) == (written, alias.name, _asname(alias))
        for binding, alias in zip(bindings, node.names)
    )


def _edit(source: Source, node: ast.ImportFrom, bindings: list[_Binding]) -> Edit | None:
    start, end = source.span(node)
    tokens = source.tokens_in(start, end)
    modules = {binding.module for binding in bindings}
    same_names = all(
        (binding.name, binding.asname) == (alias.name, _asname(alias))
        for binding, alias in zip(bindings, node.names)
    )
    if len(modules) == 1 and same_names:
        # DEDENT tokens may share the statement's start offset, so find the keywords.
        words = [k for k, token in enumerate(tokens)
                 if token.type == tokenize.NAME and token.string in ("from", "import")]
        return Edit(tokens[words[0] + 1].start, tokens[words[1] - 1].end, bindings[0].module)
    if any(token.type == tokenize.COMMENT for token in tokens):
        return None
    head = source.line_head(start)
    separator = source.newline + head if not head.strip() else "; "
    # One statement per run of names from one module, in the original order,
    # so a name bound twice ends bound as before.
    statements = [
        _render(module, list(group), head if not head.strip() else "", source.newline,
                node.lineno != node.end_lineno)
        for module, group in groupby(bindings, key=attrgetter("module"))
    ]
    return Edit(start, end, separator.join(statements))


def _render(module: str, bindings: list[_Binding], indent: str, newline: str,
            multiline: bool) -> str:
    names = [binding.name + (f" as {binding.asname}" if binding.asname else "")
             for binding in bindings]
    if not multiline or len(names) == 1:
        return f"from {module} import {', '.join(names)}"
    lines = "".join(f"{indent}    {name},{newline}" for name in names)
    return f"from {module} import ({newline}{lines}{indent})"
