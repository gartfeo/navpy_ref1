"""The renames one step-1 rewrite applies: dotted modules, symbols and paths.

A dotted name is renamed by its longest prefix the plan knows as an old
module, so a submodule that moves in another commit keeps its name even when
its package moves now. Paths are src-relative ("navpy/a/b.py") and renamed
only when they name a moving file exactly; folders are never rewritten.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Mapping

ROOT_PACKAGE = "navpy"
# A dotted navpy name in text: "navpy.a.b", not inside a longer name or path.
DOTTED_NAME = re.compile(r"(?<![\w./\\-])navpy(?:\.[A-Za-z_]\w*)+")
# A src-relative navpy path in text: "src/navpy/a/b.py" holds "navpy/a/b.py".
NAVPY_PATH = re.compile(r"(?<![\w.\\-])navpy/[\w./-]*")


@dataclass(frozen=True)
class RewriteMaps:
    """modules: old dotted module -> new name, for the modules moving now.
    known: every old module of the plan; bounds the longest-prefix match.
    paths: src-relative old path -> new one, for the files moving now.
    symbols: (old module, name) -> module that defines name now (facades
    removed by c01, parts split by c02); used by from-imports and strings.
    lookups: (old module, imported name) -> module that looks the name up
    now (a split part whose code uses an import of its source; see
    step1_lookups), None when several do; strings (mock.patch targets)
    follow it before symbols."""

    modules: Mapping[str, str]
    known: frozenset[str]
    paths: Mapping[str, str]
    symbols: Mapping[tuple[str, str], str] = field(default_factory=dict)
    lookups: Mapping[tuple[str, str], str | None] = field(default_factory=dict)

    def __post_init__(self) -> None:
        unknown = sorted(set(self.modules) - self.known)
        if unknown:
            raise ValueError(f"moving modules missing from known: {unknown}")


def known_prefix(name: str, known: frozenset[str]) -> str | None:
    """Longest leading part of a dotted name that is a known module."""
    parts = name.split(".")
    for count in range(len(parts), 0, -1):
        prefix = ".".join(parts[:count])
        if prefix in known:
            return prefix
    return None


def rename_module(name: str, maps: RewriteMaps) -> str:
    """The dotted name with its known module prefix renamed, if that moves."""
    prefix = known_prefix(name, maps.known)
    if prefix is None or prefix not in maps.modules:
        return name
    return maps.modules[prefix] + name[len(prefix):]


def _symbol_key(name: str, maps: RewriteMaps) -> tuple[str, str] | None:
    """(known module, first name after it) of a dotted name."""
    prefix = known_prefix(name, maps.known)
    if prefix is None or len(name) == len(prefix):
        return None
    return prefix, name[len(prefix) + 1:].split(".", 1)[0]


def rename_reference(name: str, maps: RewriteMaps) -> str:
    """rename_module, except that a name the change moves follows it: to the
    module that looks it up (lookups), else to its defining module (symbols)."""
    key = _symbol_key(name, maps)
    if key is not None:
        home = maps.lookups.get(key) or maps.symbols.get(key)
        if home is not None:
            return home + name[len(key[0]):]
    return rename_module(name, maps)


def unroutable(name: str, maps: RewriteMaps) -> bool:
    """A dotted name no single module looks up any more (a hand fix)."""
    key = _symbol_key(name, maps)
    return key is not None and key in maps.lookups and maps.lookups[key] is None


def rename_path(path: str, maps: RewriteMaps) -> str:
    """The src-relative path renamed when it names a moving file."""
    stem = path.rstrip("/")
    new = maps.paths.get(stem)
    return path if new is None else new + path[len(stem):]


def text_renames(text: str, maps: RewriteMaps) -> list[tuple[int, int, str]]:
    """(start, end, replacement) for every dotted name and path in text that
    the maps rename, in text order."""
    renames: list[tuple[int, int, str]] = []
    for match in DOTTED_NAME.finditer(text):
        new = rename_reference(match.group(0), maps)
        if new != match.group(0):
            renames.append((match.start(), match.end(), new))
    for match in NAVPY_PATH.finditer(text):
        new = rename_path(match.group(0), maps)
        if new != match.group(0):
            renames.append((match.start(), match.end(), new))
    return sorted(renames)
