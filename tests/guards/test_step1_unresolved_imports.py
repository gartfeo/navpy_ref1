"""Every navpy import in the repository resolves to code on disk.

Refactor step 1 moves most of src/navpy and rewrites every reference; an
import left pointing at an old path, or at a name the module no longer
defines, fails here with file:line. Read from source (never find_spec), so an
installed copy of navpy in site-packages cannot hide a broken import.
"""

from __future__ import annotations

import ast
from functools import cache
from pathlib import Path
from typing import Iterable, Mapping

from tests.guards.step1_layers import is_navpy
from tests.guards.step1_sources import (
    REPO_ROOT,
    ImportRef,
    SourceFile,
    import_refs,
    module_bindings,
    module_of,
    navpy_sources,
    python_files,
)

# src/navpy is read with its package context (relative imports resolve there).
OTHER_AREAS = ("src/gcs", "scripts", "tests", "tools")


class NavpyIndex:
    """Modules, package directories and module-scope names of src/navpy."""

    def __init__(self, sources: Mapping[str, SourceFile]) -> None:
        self.sources = {source.module: source for source in sources.values()}
        self.packages = {
            ".".join(["navpy", *parts[:depth]])
            for parts in (Path(path).parent.parts for path in sources)
            for depth in range(len(parts) + 1)
        }
        self.names: dict[str, frozenset[str] | None] = {}

    def has_module(self, dotted: str) -> bool:
        return dotted in self.sources or dotted in self.packages

    def problem(self, ref: ImportRef) -> str | None:
        """Why the import does not resolve; None when it does."""
        if not self.has_module(ref.module):
            return f"no module {ref.module}"
        if ref.name in (None, "*") or self.has_module(f"{ref.module}.{ref.name}"):
            return None
        names = self._names(ref.module)
        if names is None or ref.name in names:
            return None
        return f"{ref.module} has no {ref.name}"

    def problems(self, where: str, refs: Iterable[ImportRef]) -> list[str]:
        found = []
        for ref in refs:
            problem = self.problem(ref) if is_navpy(ref.module) else None
            if problem is not None:
                found.append(f"{where}:{ref.line}: {problem}")
        return found

    def _names(self, module: str) -> frozenset[str] | None:
        """Module-scope names (None: open); a directory without __init__ offers none."""
        if module not in self.names:
            source = self.sources.get(module)
            self.names[module] = frozenset() if source is None else module_bindings(source.tree)
        return self.names[module]


@cache
def _index() -> NavpyIndex:
    return NavpyIndex(navpy_sources(REPO_ROOT))


def _source(path: str, text: str) -> SourceFile:
    tree = ast.parse(text)
    return SourceFile(path, module_of(f"navpy/{path}"), tree, tuple(import_refs(tree, None)))


def test_src_navpy_imports_resolve() -> None:
    sources = navpy_sources(REPO_ROOT)
    problems = [problem for path, source in sources.items()
                for problem in _index().problems(f"src/navpy/{path}", source.refs)]

    assert len(sources) > 500
    assert sum(is_navpy(ref.module) for s in sources.values() for ref in s.refs) > 3000
    assert not problems, "\n".join(problems)


def test_navpy_imports_outside_src_navpy_resolve() -> None:
    problems: list[str] = []
    checked = 0
    for area in OTHER_AREAS:
        for path in python_files(REPO_ROOT / area):
            refs = import_refs(ast.parse(path.read_bytes(), filename=str(path)), None)
            checked += sum(is_navpy(ref.module) for ref in refs)
            problems += _index().problems(path.relative_to(REPO_ROOT).as_posix(), refs)

    assert checked > 3000
    assert not problems, "\n".join(problems)


def test_the_check_reports_missing_modules_and_names() -> None:
    index = NavpyIndex({
        "__init__.py": _source("__init__.py", ""),
        "pkg/__init__.py": _source("pkg/__init__.py", "from navpy.pkg.mod import Kept\n"),
        "pkg/mod.py": _source("pkg/mod.py", "class Kept: ...\nif True:\n    Late = 1\n"),
        "spaces/sub/leaf.py": _source("spaces/sub/leaf.py", ""),
    })
    refs = import_refs(ast.parse(
        "from navpy.pkg import mod, Kept\n"
        "from navpy.pkg.mod import Kept, Late\n"
        "import navpy.spaces.sub.leaf\n"
        "from navpy.spaces import sub\n"
        "from navpy.old.mod import Kept\n"
        "from navpy.pkg import Gone\n"
        "from navpy.spaces import Gone\n"
        "from navpy.pkg.mod import *\n"
    ), None)

    assert index.problems("x.py", refs) == [
        "x.py:5: no module navpy.old.mod",
        "x.py:6: navpy.pkg has no Gone",
        "x.py:7: navpy.spaces has no Gone",
    ]
