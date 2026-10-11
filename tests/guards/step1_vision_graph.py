"""Runtime import graph of src/navpy as step 1 leaves it, for the pure-vision walk.

Each file is a node at its step-1 home (manifest step1_dest while it waits in
the old tree, its own path once landed), so the walk means the same before and
after every move. The graph also does now what c01 and c02 will do:
- an imported name is followed to the module that defines it only through a
  file c01 deletes or empties (a delete row) or a re-export line listed in
  reexports.tsv (c01 makes importers name the defining module); any other
  module an import names is loaded at runtime, so the hop ends there;
- a re-export line listed in reexports.tsv is dropped (c01 strips it);
- a split source that still holds a split part's symbols is cut into a node per
  part (c02): its imports belong to the parts whose code uses them, and its
  parts reference each other.
Loading a module first runs the __init__.py of every package above it, so each
node also has a "package" hop to every ancestor package init that is a node
(at its step-1 home); an init c01 deletes or empties is a delete-row node and
ends the walk there.
TYPE_CHECKING imports are left out: they do not run.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass
from typing import Iterable, Mapping

from tests.guards.step1_layers import is_navpy, module_paths
from tests.guards.step1_sources import ImportRef, SourceFile, plan_module

sm = plan_module()


@dataclass(frozen=True)
class Node:
    home: str          # step-1 home below src/navpy; "delete" for a deleted file
    flagged: bool      # on the pure-vision path (manifest or split-part flag)


@dataclass(frozen=True)
class Hop:
    target: str        # node key
    line: int          # 0 for a package hop
    where: str         # "top", "function" or "package"


@dataclass(frozen=True)
class VisionGraph:
    nodes: dict[str, Node]               # key: the home; a deleted file keeps its path
    hops: dict[str, tuple[Hop, ...]]
    unplaced: tuple[str, ...]            # old-tree files the manifest does not place


class _Placement:
    """Step-1 home and vision flag of every file and split part."""

    def __init__(self, rows: tuple[sm.ManifestRow, ...],
                 splits: Mapping[str, tuple[sm.SplitPart, ...]]) -> None:
        self.rows = {row.src: row for row in rows}
        self.splits = splits
        self.flags: dict[str, bool] = {}
        for row in rows:
            if row.step1_dest != sm.DELETE:
                self.flags[row.step1_dest] = self.flags.get(row.step1_dest, False) or row.vision_path
        for parts in splits.values():
            for part in parts:
                self.flags[part.step1] = self.flags.get(part.step1, False) or part.vision_path

    def node(self, path: str) -> tuple[str, Node] | None:
        """(key, node) of a file; None for an old-tree file with no manifest row."""
        row = self.rows.get(path)
        if row is not None:
            key = path if row.step1_dest == sm.DELETE else row.step1_dest
            return key, Node(row.step1_dest, row.vision_path)
        if sm.layer_of(path).kind == "old":
            return None
        return path, Node(path, self.flags.get(path, False))

    def deleted(self, path: str) -> bool:
        """A file c01 deletes or empties (a manifest delete row)."""
        row = self.rows.get(path)
        return row is not None and row.step1_dest == sm.DELETE

    def virtual_parts(self, source: SourceFile, defined: set[str]) -> dict[str, str]:
        """symbol -> part home, for parts still defined in their source file."""
        if source.path not in self.rows:
            return {}
        parts: dict[str, str] = {}
        for part in self.splits.get(source.path, ()):
            if defined.intersection(part.symbols):
                parts.update(dict.fromkeys(part.symbols, part.step1))
        return parts


def build_vision_graph(sources: Mapping[str, SourceFile], rows: tuple[sm.ManifestRow, ...],
                       splits: Mapping[str, tuple[sm.SplitPart, ...]],
                       dropped: frozenset[tuple[str, int, str]]) -> VisionGraph:
    """dropped: (path, line, bound name) re-export lines c01 strips."""
    return _Builder(sources, _Placement(rows, splits), dropped).build()


class _Builder:
    def __init__(self, sources: Mapping[str, SourceFile], placement: _Placement,
                 dropped: frozenset[tuple[str, int, str]]) -> None:
        self.sources = sources
        self.placement = placement
        self.dropped = dropped
        self.modules = module_paths(sources)
        self.defs = {path: _top_level_defs(source.tree) for path, source in sources.items()}
        self.binds = {path: {ref.bound: ref for ref in source.refs if ref.where == "top"}
                      for path, source in sources.items()}
        self.parts = {path: placement.virtual_parts(source, self.defs[path])
                      for path, source in sources.items()}

    def build(self) -> VisionGraph:
        nodes: dict[str, Node] = {}
        unplaced: list[str] = []
        hops: dict[str, list[Hop]] = {}
        for path, source in sorted(self.sources.items()):
            placed = self.placement.node(path)
            if placed is None:
                unplaced.append(path)
                nodes[path] = Node(path, False)
                continue
            nodes.setdefault(*placed)
            for symbol_home in set(self.parts[path].values()):
                nodes.setdefault(symbol_home, Node(symbol_home, self.placement.flags[symbol_home]))
            for owner, hop in self._file_hops(source, placed[0]):
                if owner != hop.target:
                    hops.setdefault(owner, []).append(hop)
        for key, node in nodes.items():
            hops.setdefault(key, []).extend(_package_hops(node, nodes))
        return VisionGraph(nodes, {k: tuple(v) for k, v in hops.items() if v}, tuple(unplaced))

    def key(self, path: str, symbol: str | None = None) -> str:
        if symbol is not None and symbol in self.parts.get(path, {}):
            return self.parts[path][symbol]
        placed = self.placement.node(path)
        return path if placed is None else placed[0]

    def _file_hops(self, source: SourceFile, own: str) -> Iterable[tuple[str, Hop]]:
        parts = self.parts[source.path]
        usage = _users_by_owner(source.tree, parts, own) if parts else {}
        for ref in source.refs:
            if ref.type_checking or not is_navpy(ref.module):
                continue
            if not parts and ref.name and ref.where == "top" and (
                    source.path, ref.line, ref.bound) in self.dropped:
                continue
            target = self._target(ref)
            if target is None:
                continue
            owners = {own} if not parts else (
                usage.get(ref.bound, set()) if ref.where == "top"
                else {_enclosing_owner(source.tree, ref.line, parts, own)})
            for owner in owners:
                yield owner, Hop(target, ref.line, ref.where)
        if parts:
            yield from _part_references(source.tree, parts, own, self.defs[source.path])

    def _target(self, ref: ImportRef) -> str | None:
        if ref.name is None:
            path = self._longest_module(ref.module)
            return None if path is None else self.key(path)
        submodule = self.modules.get(f"{ref.module}.{ref.name}")
        if submodule is not None:
            return self.key(submodule)
        path = self._longest_module(ref.module)
        if path is None:
            return None
        origin, name = self._origin(path, ref.name, set())
        return self.key(origin, name)

    def _origin(self, path: str, name: str, seen: set[tuple[str, str]]) -> tuple[str, str | None]:
        """(file the import loads, name): followed to the defining file only
        through re-exports c01 removes (delete rows and listed lines)."""
        if (path, name) in seen or name in self.defs[path]:
            return path, name
        seen.add((path, name))
        bind = self.binds[path].get(name)
        if bind is None or bind.name is None or not is_navpy(bind.module):
            return path, name
        if not self.placement.deleted(path) and (path, bind.line, name) not in self.dropped:
            return path, name
        submodule = self.modules.get(f"{bind.module}.{bind.name}")
        if submodule is not None:
            return submodule, None
        defining = self._longest_module(bind.module)
        return (path, name) if defining is None else self._origin(defining, bind.name, seen)

    def _longest_module(self, dotted: str) -> str | None:
        parts = dotted.split(".")
        while parts:
            path = self.modules.get(".".join(parts))
            if path is not None:
                return path
            parts.pop()
        return None


def _package_hops(node: Node, nodes: Mapping[str, Node]) -> list[Hop]:
    """Hops to the package inits loading the node's module runs first."""
    if node.home == sm.DELETE:
        return []
    folders = node.home.split("/")[:-1]
    if node.home.rpartition("/")[2] == "__init__.py":
        folders = folders[:-1] if folders else None
    if folders is None:
        return []
    inits = ["/".join([*folders[:depth], "__init__.py"]) for depth in range(len(folders) + 1)]
    return [Hop(init, 0, "package") for init in inits if init in nodes and init != node.home]


def _top_level_defs(tree: ast.Module) -> set[str]:
    return {name for node in tree.body for name in _statement_symbols(node)}


def _statement_symbols(node: ast.stmt) -> list[str]:
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
        return [node.name]
    if isinstance(node, (ast.Assign, ast.AnnAssign)):
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        return [n.id for target in targets for n in ast.walk(target) if isinstance(n, ast.Name)]
    return []


def _owner(node: ast.stmt, parts: Mapping[str, str], own: str) -> str:
    homes = [parts[symbol] for symbol in _statement_symbols(node) if symbol in parts]
    return homes[-1] if homes else own


def _code_statements(tree: ast.Module) -> Iterable[ast.stmt]:
    """Top-level statements that are code: not imports, import blocks or __all__."""
    for node in tree.body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            continue
        if isinstance(node, ast.If) and all(
                isinstance(child, (ast.Import, ast.ImportFrom)) for child in node.body):
            continue
        if _statement_symbols(node) == ["__all__"]:
            continue
        yield node


def _users_by_owner(tree: ast.Module, parts: Mapping[str, str], own: str) -> dict[str, set[str]]:
    """bound name -> owners (part homes, or own) of the code that uses it."""
    users: dict[str, set[str]] = {}
    for node in _code_statements(tree):
        for name in _names_used(node):
            users.setdefault(name, set()).add(_owner(node, parts, own))
    return users


def _enclosing_owner(tree: ast.Module, line: int, parts: Mapping[str, str], own: str) -> str:
    for node in tree.body:
        if node.lineno <= line <= (node.end_lineno or node.lineno):
            return _owner(node, parts, own)
    return own


def _part_references(tree: ast.Module, parts: Mapping[str, str], own: str,
                     defined: set[str]) -> Iterable[tuple[str, Hop]]:
    """Hops between the parts of one split source that use each other's names."""
    for node in _code_statements(tree):
        owner = _owner(node, parts, own)
        symbols = set(_statement_symbols(node))
        for name in (_names_used(node) & defined) - symbols:
            target = parts.get(name, own)
            if target != owner:
                yield owner, Hop(target, node.lineno, "top")


def _names_used(node: ast.AST) -> set[str]:
    """Names a statement reads: Name nodes and names in string annotations
    (docstrings and other strings are not references)."""
    used = {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}
    for annotation in _annotations(node):
        for constant in ast.walk(annotation):
            if isinstance(constant, ast.Constant) and isinstance(constant.value, str):
                try:
                    expression = ast.parse(constant.value, mode="eval")
                except SyntaxError:
                    continue
                used.update(n.id for n in ast.walk(expression) if isinstance(n, ast.Name))
    return used


def _annotations(node: ast.AST) -> Iterable[ast.expr]:
    for child in ast.walk(node):
        if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
            args = child.args
            for arg in [*args.posonlyargs, *args.args, *args.kwonlyargs, args.vararg, args.kwarg]:
                if arg is not None and arg.annotation is not None:
                    yield arg.annotation
            if child.returns is not None:
                yield child.returns
        elif isinstance(child, ast.AnnAssign):
            yield child.annotation
