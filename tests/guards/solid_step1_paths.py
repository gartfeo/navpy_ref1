"""Path builders the step-1 normalizer may rewrite, found by their bindings.

Only calls and operators that really build a path are rewritten: os.path.join
(also posixpath/ntpath, under any import spelling), the pathlib classes, and
`joinpath(...)` or `/` on a value known to be a pathlib path (a pathlib call,
`.parent`, `.parents[i]`, `.resolve()` and the like of one, or a module-level
name only ever assigned one). Any other call keeps its arguments as written.
"""

from __future__ import annotations

import ast
from typing import Dict, Optional, Set

JOINERS = frozenset({"os.path.join", "posixpath.join", "ntpath.join"})
PATH_CLASSES = frozenset({
    "pathlib.Path", "pathlib.PurePath", "pathlib.PurePosixPath",
    "pathlib.PureWindowsPath", "pathlib.PosixPath", "pathlib.WindowsPath",
})
# Methods and properties of a pathlib path that return a path.
_PATH_METHODS = frozenset({"joinpath", "resolve", "absolute", "expanduser", "with_name",
                           "with_suffix", "with_stem", "relative_to"})
_PATH_PROPERTIES = frozenset({"parent"})


class PathBuilders:
    """The path-building names of one module, read before it is rewritten."""

    def __init__(self, tree: ast.Module) -> None:
        self._imports: Dict[str, str] = {}
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    bound = alias.asname or alias.name.split(".")[0]
                    self._imports[bound] = alias.name if alias.asname else bound
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                for alias in node.names:
                    self._imports[alias.asname or alias.name] = f"{node.module}.{alias.name}"
        self._paths: Set[str] = set()
        self._others: Set[str] = set()
        for statement in tree.body:
            if isinstance(statement, (ast.Assign, ast.AnnAssign)) and statement.value is not None:
                targets = statement.targets if isinstance(statement, ast.Assign) else [statement.target]
                path = self.is_path(statement.value)
                for target in targets:
                    if isinstance(target, ast.Name):
                        (self._paths if path else self._others).add(target.id)
        self._paths -= self._others

    def qualified(self, node: ast.expr) -> Optional[str]:
        """The imported dotted name an expression spells, if any."""
        if isinstance(node, ast.Name):
            return self._imports.get(node.id)
        if isinstance(node, ast.Attribute):
            prefix = self.qualified(node.value)
            return None if prefix is None else f"{prefix}.{node.attr}"
        return None

    def builds_path(self, call: ast.Call) -> bool:
        """os.path.join(...), a pathlib class, or joinpath on a known path."""
        func = call.func
        if self.qualified(func) in JOINERS | PATH_CLASSES:
            return True
        return isinstance(func, ast.Attribute) and func.attr == "joinpath" and self.is_path(func.value)

    def is_path(self, node: ast.expr) -> bool:
        """An expression known to be a pathlib path."""
        if isinstance(node, ast.Name):
            return node.id in self._paths
        if isinstance(node, ast.Call):
            func = node.func
            return self.qualified(func) in PATH_CLASSES or (
                isinstance(func, ast.Attribute) and func.attr in _PATH_METHODS
                and self.is_path(func.value))
        if isinstance(node, ast.BinOp):
            return isinstance(node.op, ast.Div) and self.is_path(node.left)
        if isinstance(node, ast.Attribute):
            return node.attr in _PATH_PROPERTIES and self.is_path(node.value)
        if isinstance(node, ast.Subscript):
            value = node.value
            return (isinstance(value, ast.Attribute) and value.attr == "parents"
                    and self.is_path(value.value))
        return False
