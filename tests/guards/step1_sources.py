"""Source reading for the step-1 guards: files, modules and import records.

Everything is read from the files with ast (never importlib.util.find_spec), so
a guard judges the tree on disk, whatever is installed in site-packages.
"""

from __future__ import annotations

import ast
import importlib.util
import sys
from dataclasses import dataclass
from functools import cache
from pathlib import Path
from types import ModuleType
from typing import Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
SKIP_DIRS = frozenset({"__pycache__", "node_modules", ".venv", ".git"})
_FUNCTION_NODES = (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)


def plan_module() -> ModuleType:
    """scripts/step1_manifest.py, loaded once per process (the dataclasses
    need it registered in sys.modules)."""
    name = "step1_manifest"
    path = REPO_ROOT / "scripts" / "step1_manifest.py"
    loaded = sys.modules.get(name)
    if loaded is not None and Path(loaded.__file__ or "").resolve() == path:
        return loaded
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class ImportRef:
    """One imported name: `from module import name` or `import module`."""

    line: int
    module: str           # absolute dotted module the statement names
    name: str | None      # None for `import module`
    bound: str            # name the statement binds in the importer
    where: str            # "top" or "function"
    type_checking: bool   # inside an `if TYPE_CHECKING:` body


@dataclass(frozen=True)
class SourceFile:
    path: str             # posix path relative to the scanned src root
    module: str
    tree: ast.Module
    refs: tuple[ImportRef, ...]


def python_files(directory: Path) -> list[Path]:
    """Every .py file under directory, sorted, skipping caches and vendored trees."""
    if not directory.is_dir():
        return []
    return sorted(
        path for path in directory.rglob("*.py")
        if not SKIP_DIRS.intersection(path.relative_to(directory).parts)
    )


def module_of(relative: str) -> str:
    """Dotted module of a posix .py path relative to a src root."""
    parts = relative[: -len(".py")].split("/")
    if parts[-1] == "__init__":
        parts.pop()
    return ".".join(parts)


def package_of(relative: str) -> str:
    """The package relative imports in this file resolve against."""
    module = module_of(relative)
    return module if relative.endswith("/__init__.py") else module.rpartition(".")[0]


def read_package(src_root: Path, package: str) -> dict[str, SourceFile]:
    """Parse every file of a package under src_root; keyed by path below the package."""
    base = src_root / package
    files: dict[str, SourceFile] = {}
    for path in python_files(base):
        relative = path.relative_to(src_root).as_posix()
        tree = ast.parse(path.read_bytes(), filename=str(path))
        below = relative[len(package) + 1:]
        files[below] = SourceFile(below, module_of(relative), tree,
                                  tuple(import_refs(tree, package_of(relative))))
    return files


@cache
def navpy_sources(repo_root: Path = REPO_ROOT) -> dict[str, SourceFile]:
    """Every file under src/navpy, keyed by its path below src/navpy."""
    return read_package(repo_root / "src", "navpy")


def import_refs(tree: ast.Module, package: str | None) -> list[ImportRef]:
    """Import records of a module; package None drops relative imports."""
    refs: list[ImportRef] = []
    _collect(tree.body, package, False, False, refs)
    return refs


def _collect(nodes: Iterable[ast.AST], package: str | None, in_function: bool,
             type_checking: bool, refs: list[ImportRef]) -> None:
    for node in nodes:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            refs.extend(_refs(node, package, "function" if in_function else "top", type_checking))
        elif isinstance(node, ast.If) and _is_type_checking(node.test):
            _collect(node.body, package, in_function, True, refs)
            _collect(node.orelse, package, in_function, type_checking, refs)
        else:
            inner = in_function or isinstance(node, _FUNCTION_NODES)
            _collect(ast.iter_child_nodes(node), package, inner, type_checking, refs)


def _is_type_checking(test: ast.expr) -> bool:
    return (isinstance(test, ast.Name) and test.id == "TYPE_CHECKING") or (
        isinstance(test, ast.Attribute) and test.attr == "TYPE_CHECKING")


def _refs(node: ast.Import | ast.ImportFrom, package: str | None, where: str,
          type_checking: bool) -> list[ImportRef]:
    if isinstance(node, ast.Import):
        return [ImportRef(node.lineno, alias.name, None, alias.asname or alias.name.split(".")[0],
                          where, type_checking) for alias in node.names]
    module = absolute_module(node, package)
    if module is None:
        return []
    return [ImportRef(node.lineno, module, alias.name, alias.asname or alias.name,
                      where, type_checking) for alias in node.names]


def absolute_module(node: ast.ImportFrom, package: str | None) -> str | None:
    """The module a from-import names, relative levels resolved."""
    if not node.level:
        return node.module or ""
    if not package:
        return None
    parts = package.split(".")
    if node.level - 1 >= len(parts):
        return None
    base = parts[: len(parts) - node.level + 1]
    return ".".join(base + ([node.module] if node.module else []))


def module_bindings(tree: ast.Module) -> frozenset[str] | None:
    """Names a module binds at module scope; None when it is open (a star
    import or a module __getattr__ can supply any name)."""
    names: set[str] = set()
    for node in _module_scope(tree.body):
        if isinstance(node, ast.ImportFrom) and any(a.name == "*" for a in node.names):
            return None
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            if node.name == "__getattr__":
                return None
            names.add(node.name)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            names.update(a.asname or a.name.split(".")[0] for a in node.names)
        else:
            names.update(_stored_names(node))
    return frozenset(names)


def _module_scope(nodes: list[ast.stmt]) -> Iterable[ast.stmt]:
    """Statements run at module scope, through if/try/with/for/while blocks."""
    for node in nodes:
        yield node
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        for field in ("body", "orelse", "finalbody"):
            yield from _module_scope(getattr(node, field, []))
        for handler in getattr(node, "handlers", []):
            yield from _module_scope(handler.body)


def _stored_names(node: ast.stmt) -> set[str]:
    targets: list[ast.AST] = []
    if isinstance(node, ast.Assign):
        targets = list(node.targets)
    elif isinstance(node, (ast.AnnAssign, ast.AugAssign, ast.For, ast.AsyncFor)):
        targets = [node.target]
    elif isinstance(node, (ast.With, ast.AsyncWith)):
        targets = [item.optional_vars for item in node.items if item.optional_vars]
    elif isinstance(node, ast.Try):
        return {handler.name for handler in node.handlers if handler.name}
    return {n.id for target in targets for n in ast.walk(target) if isinstance(n, ast.Name)}
