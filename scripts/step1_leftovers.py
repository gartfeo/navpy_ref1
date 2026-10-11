"""Leftover references to old step-1 locations: the move tool's gate.

A reference is a leftover when the old module, file or folder it names no
longer exists, or it names anything inside an old folder that is gone: a
dotted name in code or a from-import, a dotted name or src/navpy path in a
string (adjacent literals joined, f-string literal parts included, f-strings
also scanned raw), a path string relative to src/navpy, or a path built from
literal segments after a "navpy" segment (os.path.join, Path(...), "/"
chains, joinpath, nested in each other). An old path that still exists is no
leftover: it moves in a later commit.
"""

from __future__ import annotations

import ast
from dataclasses import dataclass
from typing import Iterable, Iterator, Mapping

from step1_leftovers_segments import segment_paths
from step1_manifest import NAVPY_ROOT, ManifestRow, module_name
from step1_rewrite import dotted_chains
from step1_rewrite_imports import Importer, import_target
from step1_rewrite_maps import DOTTED_NAME, NAVPY_PATH, ROOT_PACKAGE, known_prefix
from step1_rewrite_source import Source, SourceError, parse_source
from step1_rewrite_strings import string_groups, string_piece

_Finding = tuple[int, str, str]
_FIELD = "\0"


@dataclass(frozen=True)
class Leftover:
    path: str
    line: int
    kind: str
    detail: str

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: {self.kind}: {self.detail}"


@dataclass(frozen=True)
class TreeState:
    """The tree a scan judges: files and folders under src/ that exist, and
    the plan's old modules (with their old paths) and old files and folders,
    src-relative ("navpy/a/b.py") and src/navpy-relative ("a/b.py")."""

    files: frozenset[str]
    folders: frozenset[str]
    module_paths: Mapping[str, str]
    known: frozenset[str]
    old_paths: frozenset[str]
    relative_paths: frozenset[str]

    def exists(self, repo_relative: str) -> bool:
        return repo_relative in self.files or repo_relative in self.folders

    def stale_module(self, name: str) -> str | None:
        """The gone old module a dotted name starts with, if any."""
        prefix = known_prefix(name, self.known)
        if prefix is None or prefix == ROOT_PACKAGE or self.exists(self.module_paths[prefix]):
            return None
        return prefix

    def stale_path(self, src_relative: str) -> bool:
        return self._gone(src_relative.rstrip("/"), self.old_paths, "src/")

    def stale_relative(self, value: str) -> bool:
        path = value.strip().rstrip("/")
        return not any(char.isspace() for char in path) and self._gone(
            path, self.relative_paths, NAVPY_ROOT)

    def _gone(self, path: str, old: frozenset[str], root: str) -> bool:
        """The longest old path that path starts with no longer exists."""
        parts = path.split("/")
        for count in range(len(parts), 0, -1):
            prefix = "/".join(parts[:count])
            if prefix in old:
                return not self.exists(root + prefix)
        return False


def tree_state(rows: tuple[ManifestRow, ...], files: Iterable[str]) -> TreeState:
    module_paths: dict[str, str] = {}
    old_paths: set[str] = set()
    for row in rows:
        old = NAVPY_ROOT + row.src
        module = module_name(old)
        if module is not None:
            module_paths[module] = old
        pieces = row.src.split("/")
        old_paths.update("/".join(pieces[:count]) for count in range(1, len(pieces) + 1))
    existing = frozenset(files)
    folders = frozenset(path.rsplit("/", depth)[0] for path in existing
                        for depth in range(1, path.count("/") + 1))
    return TreeState(existing, folders, module_paths, frozenset(module_paths),
                     frozenset(f"{ROOT_PACKAGE}/{path}" for path in old_paths),
                     frozenset(path for path in old_paths if "/" in path))


def scan_leftovers(path: str, text: str, state: TreeState,
                   importer: Importer | None) -> list[Leftover]:
    """Leftovers in one file; importer places it for relative imports."""
    try:
        source = parse_source(text)
    except SourceError as error:
        return [Leftover(path, 0, "unparsable", str(error))] if ROOT_PACKAGE in text else []
    findings = [*_code_findings(source, state, importer), *_string_findings(source, state)]
    findings += [(line, "segment-built path", segments)
                 for line, segments in segment_paths(source.tree) if state.stale_path(segments)]
    unique = {(line, detail): kind for line, kind, detail in findings}
    return [Leftover(path, line, kind, detail) for (line, detail), kind in sorted(unique.items())]


def _code_findings(source: Source, state: TreeState,
                   importer: Importer | None) -> Iterator[_Finding]:
    spans: list[tuple[int, int]] = []
    for node in ast.walk(source.tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        spans.append(source.span(node))
        target = import_target(node, importer, "new")
        if target is None or target.split(".")[0] != ROOT_PACKAGE:
            continue
        for alias in node.names:
            name = target if alias.name == "*" else f"{target}.{alias.name}"
            yield from _module_finding(node.lineno, "import", name, state)
    for first, _, dotted in dotted_chains(source.tokens):
        if not any(start <= first.start < end for start, end in spans):
            yield from _module_finding(first.line, "module reference", dotted, state)


def _string_findings(source: Source, state: TreeState) -> Iterator[_Finding]:
    for group in string_groups(source.tokens):
        pieces = [string_piece(token) for token in group]
        values = [(group[0].line, "".join(
            piece.value if piece else _literal_parts(token.string)
            for token, piece in zip(group, pieces)))]
        values += [(token.line, token.string)
                   for token, piece in zip(group, pieces) if piece is None]
        for line, value in values:
            for match in DOTTED_NAME.finditer(value):
                yield from _module_finding(line, "dotted string", match.group(0), state)
            for match in NAVPY_PATH.finditer(value):
                if state.stale_path(match.group(0)):
                    yield line, "path string", match.group(0)
            if state.stale_relative(value):
                yield line, "relative path string", value


def _literal_parts(literal: str) -> str:
    """An f-string's literal text, each replacement field as a NUL that ends
    any name or path; a bytes literal is one NUL."""
    node = ast.parse(literal, mode="eval").body
    if not isinstance(node, ast.JoinedStr):
        return _FIELD
    return "".join(part.value if isinstance(part, ast.Constant) else _FIELD
                   for part in node.values)


def _module_finding(line: int, kind: str, name: str, state: TreeState) -> Iterator[_Finding]:
    stale = state.stale_module(name)
    if stale is not None:
        yield line, kind, f"{name} ({stale} is gone)"
