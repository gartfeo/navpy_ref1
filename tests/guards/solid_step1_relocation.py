"""Step-1 relocation for the SOLID guard: where a file's code sat at the base.

Refactor step 1 (docs/design/step1_separation.md) moves the files of
src/navpy, splits 12 of them and rewrites importers to name the defining
module, while the guard's pinned base keeps the old layout. This module reads
the move map from docs/design/step1/ through scripts/step1_manifest.py and
spells every name both sides use one way: the step-1 home of the module that
defines it at the base. c18 re-pins the base and deletes this module together
with solid_step1_normalizer.py.
"""

from __future__ import annotations

import ast
import importlib.util
import sys
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from types import ModuleType
from typing import Iterator, Mapping

# Top-level packages under src/ whose names the move map rewrites.
NAME_ROOTS = frozenset({"navpy", "gcs"})
_MANIFEST_SCRIPT = Path("scripts", "step1_manifest.py")


@dataclass(frozen=True)
class Piece:
    """Code a current file holds from one file at the base: all of it
    (`symbols` is None) or the top-level names a split part took from it."""

    old_path: str
    symbols: frozenset[str] | None = None

    def holds(self, qualname: str) -> bool:
        return self.symbols is None or qualname.split(".", 1)[0] in self.symbols


@dataclass(frozen=True)
class Relocation:
    """The step-1 move map. Paths are repo-relative, modules dotted.

    paths, modules: old -> step-1 home of every moved, split or gcs row.
    symbol_modules: (old module, symbol) -> module of the split part defining it.
    parts: split part path -> the pieces it took from split sources.
    stripped: old path -> (bound name, origin module) of each module-level
    re-export binding c01 strips from it (reexports.tsv).
    directories: old directory -> new directory, when the directory's files
    all land in one.
    """

    paths: Mapping[str, str]
    modules: Mapping[str, str]
    symbol_modules: Mapping[tuple[str, str], str] = field(default_factory=dict)
    parts: Mapping[str, tuple[Piece, ...]] = field(default_factory=dict)
    stripped: Mapping[str, frozenset[tuple[str, str]]] = field(default_factory=dict)
    directories: Mapping[str, str] = field(default_factory=dict)

    @cached_property
    def _old_paths(self) -> dict[str, str]:
        return {new: old for old, new in self.paths.items()}

    @cached_property
    def _split_sources(self) -> frozenset[str]:
        return frozenset(
            piece.old_path for pieces in self.parts.values() for piece in pieces
        )

    def pieces(self, relative: str) -> tuple[Piece, ...]:
        """What the file at `relative` holds from the base. A step-1 home
        maps to its old path whether or not the row has moved yet."""
        part = self.parts.get(relative)
        if part is not None:
            return part
        return (Piece(self._old_paths.get(relative, relative)),)

    def is_split(self, relative: str) -> bool:
        """A split part, or the rest of a split source at either location."""
        return (
            relative in self.parts
            or self.pieces(relative)[0].old_path in self._split_sources
        )


def load_relocation(repo_root: Path) -> Relocation:
    manifest = _step1_manifest(repo_root)
    rows = manifest.load_manifest(repo_root)
    splits = manifest.load_splits(repo_root)
    parts: dict[str, list[Piece]] = {}
    for src, split_parts in splits.items():
        for part in split_parts:
            parts.setdefault(manifest.repo_path(part.step1), []).append(
                Piece(manifest.repo_path(src), frozenset(part.symbols))
            )
    stripped: dict[str, set[tuple[str, str]]] = {}
    for reexport in manifest.load_reexports(repo_root):
        stripped.setdefault(manifest.repo_path(reexport.src), set()).add(
            (reexport.name, reexport.imported_from))
    homes = {
        old: new
        for old, new in manifest.path_map(rows, "step1").items()
        if new is not None
    }
    return Relocation(
        paths=_moved(homes),
        modules=_moved(manifest.module_map(rows, "step1")),
        symbol_modules=manifest.split_symbol_map(splits, "step1"),
        parts={path: tuple(pieces) for path, pieces in parts.items()},
        stripped={path: frozenset(bindings) for path, bindings in stripped.items()},
        directories=_directory_map(homes),
    )


class BaseNames:
    """Top-level definitions and import bindings of every module at the base."""

    def __init__(self, pinned: Mapping[str, str]) -> None:
        self.modules: set[str] = set()
        self._defs: dict[str, set[str]] = {}
        self._binds: dict[str, dict[str, tuple[str, str | None]]] = {}
        for relative, source in pinned.items():
            module = module_of(relative)
            if module is not None:
                self.modules.add(module)
                self._index(module, relative, parse(source, relative))

    def defined(self, module: str) -> frozenset[str]:
        return frozenset(self._defs.get(module, ()))

    def resolve(self, module: str, name: str) -> tuple[str, str | None]:
        """(defining module, name) of `name` looked up in `module`, through
        re-exports; (submodule, None) for a module; unchanged when unknown."""
        seen: set[tuple[str, str]] = set()
        while module in self._defs and (module, name) not in seen:
            seen.add((module, name))
            if name in self._defs[module]:
                return module, name
            bound = self._binds[module].get(name)
            if bound is None:
                break
            module, original = bound
            if original is None:
                return module, None
            name = original
        if f"{module}.{name}" in self.modules:
            return f"{module}.{name}", None
        return module, name

    def _index(self, module: str, relative: str, tree: ast.Module) -> None:
        defs: set[str] = set()
        binds: dict[str, tuple[str, str | None]] = {}
        package = relative.endswith("/__init__.py")
        for statement in module_level(tree.body):
            if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                defs.add(statement.name)
            elif isinstance(statement, (ast.Assign, ast.AnnAssign)):
                targets = statement.targets if isinstance(statement, ast.Assign) else [statement.target]
                defs.update(
                    node.id for target in targets for node in ast.walk(target)
                    if isinstance(node, ast.Name)
                )
            elif isinstance(statement, ast.ImportFrom):
                source = absolute_module(module, package, statement.level, statement.module)
                if source is not None:
                    binds.update(
                        (alias.asname or alias.name, (source, alias.name))
                        for alias in statement.names
                    )
            elif isinstance(statement, ast.Import):
                binds.update(
                    (alias.asname, (alias.name, None))
                    for alias in statement.names if alias.asname
                )
        self._defs[module] = defs
        self._binds[module] = binds


class Namer:
    """Canonical spellings: a name's home is the step-1 home of the module
    that defines it at the base, so old and step-1 spellings agree."""

    def __init__(self, relocation: Relocation, base: BaseNames) -> None:
        self._relocation = relocation
        self._base = base
        self._old_modules = {new: old for old, new in relocation.modules.items()}
        self._owners = {
            (home, symbol): old
            for (old, symbol), home in relocation.symbol_modules.items()
        }
        self._part_modules = frozenset(relocation.symbol_modules.values())

    def module(self, dotted: str) -> str:
        parts = self._old_modules.get(dotted, dotted).split(".")
        for end in range(len(parts), 0, -1):
            home = self._relocation.modules.get(".".join(parts[:end]))
            if home is not None:
                return ".".join([home, *parts[end:]])
        return ".".join(parts)

    def imported(self, module: str, name: str) -> str:
        """Canonical dotted name of what `from module import name` binds."""
        owner = self._owners.get((module, name), self._old_modules.get(module, module))
        target, original = self._base.resolve(owner, name)
        if original is None:
            return self.module(target)
        home = self._relocation.symbol_modules.get((target, original))
        return f"{home or self.module(target)}.{original}"

    def dotted(self, text: str) -> str:
        """A dotted reference: the longest module prefix, then attributes."""
        parts = text.split(".")
        for end in range(len(parts), 0, -1):
            head = ".".join(parts[:end])
            if self._old_modules.get(head, head) in self._base.modules or head in self._part_modules:
                rest = parts[end:]
                if not rest:
                    return self.module(head)
                return ".".join([self.imported(head, rest[0]), *rest[1:]])
        return text

    def path(self, text: str) -> str:
        """A repo-relative path: a moved file, else its longest moved directory."""
        home = self._relocation.paths.get(text)
        if home is not None:
            return home
        parts = text.split("/")
        for end in range(len(parts), 0, -1):
            directory = self._relocation.directories.get("/".join(parts[:end]))
            if directory is not None:
                return "/".join([directory, *parts[end:]])
        return text


def module_of(relative: str) -> str | None:
    """Dotted module of a repo-relative .py path under src/, else None."""
    if not relative.startswith("src/") or not relative.endswith(".py"):
        return None
    parts = relative[len("src/"):-len(".py")].split("/")
    return ".".join(parts[:-1] if parts[-1] == "__init__" else parts)


def absolute_module(
    module: str | None, package: bool, level: int, name: str | None
) -> str | None:
    """Absolute module of an import in `module`; None when unknowable."""
    if level == 0:
        return name
    if module is None:
        return None
    parts = module.split(".")
    keep = len(parts) - (level - 1 if package else level)
    if keep <= 0:
        return None
    return ".".join([*parts[:keep], *([name] if name else [])])


def parse(source: str, relative: str) -> ast.Module:
    return ast.parse(source.removeprefix("﻿"), filename=relative)


def module_level(body: list[ast.stmt]) -> Iterator[ast.stmt]:
    """Module-level statements, including those under if/try blocks."""
    for statement in body:
        yield statement
        if isinstance(statement, ast.If):
            yield from module_level(statement.body + statement.orelse)
        elif isinstance(statement, ast.Try):
            nested = statement.body + statement.orelse + statement.finalbody
            for handler in statement.handlers:
                nested += handler.body
            yield from module_level(nested)


def _moved(mapping: Mapping[str, str | None]) -> dict[str, str]:
    return {old: new for old, new in mapping.items() if new is not None and new != old}


def _directory_map(homes: Mapping[str, str]) -> dict[str, str]:
    targets: dict[str, set[str]] = {}
    for old, new in homes.items():
        targets.setdefault(old.rpartition("/")[0], set()).add(new.rpartition("/")[0])
    return {
        old: next(iter(new))
        for old, new in targets.items()
        if len(new) == 1 and new != {old}
    }


def _step1_manifest(repo_root: Path) -> ModuleType:
    """scripts/step1_manifest.py, registered first: its dataclasses need it."""
    path = repo_root / _MANIFEST_SCRIPT
    spec = importlib.util.spec_from_file_location("step1_manifest", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module
