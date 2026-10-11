"""Paths built from literal segments, which a text rewrite cannot follow.

os.path.join(root, "src", "navpy", "modules", "x.py"), Path(...) and
joinpath calls and "/" chains, nested in each other, are read as their
literal segments; the run of literal segments after a "navpy" segment is the
src-relative path built.
"""

from __future__ import annotations

import ast

from step1_rewrite_maps import ROOT_PACKAGE

_PATH_CALLS = frozenset({"join", "joinpath", "Path", "PurePath", "PurePosixPath", "PureWindowsPath"})


def segment_paths(tree: ast.AST) -> list[tuple[int, str]]:
    """(line, "navpy/<literal segments>") for every path built this way."""
    found: list[tuple[int, str]] = []
    inner: set[int] = set()
    for node in ast.walk(tree):  # breadth first: a builder before its parts
        if id(node) in inner or not _builds_path(node):
            continue
        path = _navpy_segments(_operands(node, inner))
        if path is not None:
            found.append((node.lineno, path))
    return found


def _builds_path(node: ast.AST) -> bool:
    if isinstance(node, ast.BinOp):
        return isinstance(node.op, ast.Div)
    if not isinstance(node, ast.Call):
        return False
    func = node.func
    name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", None)
    return name in _PATH_CALLS


def _operands(node: ast.BinOp | ast.Call, inner: set[int]) -> list[ast.expr]:
    """The operands of a path builder left to right, nested builders (a "/"
    chain, a joinpath receiver, a Path(...) argument) expanded in place."""
    inner.add(id(node))
    if isinstance(node, ast.BinOp):
        parts = [node.left, node.right]
    else:
        func = node.func
        receiver = [func.value] if isinstance(func, ast.Attribute) and func.attr == "joinpath" else []
        parts = [*receiver, *node.args]
    operands: list[ast.expr] = []
    for part in parts:
        if _builds_path(part):
            operands += _operands(part, inner)
        else:
            operands.append(part)
    return operands


def _navpy_segments(operands: list[ast.expr]) -> str | None:
    segments: list[str | None] = []
    for operand in operands:
        if isinstance(operand, ast.Constant) and isinstance(operand.value, str):
            segments += [part for part in operand.value.replace("\\", "/").split("/") if part]
        else:
            segments.append(None)
    for index, segment in enumerate(segments):
        if segment != ROOT_PACKAGE:
            continue
        tail: list[str] = []
        for part in segments[index + 1:]:
            if part is None:
                break
            tail.append(part)
        if tail:
            return "/".join([ROOT_PACKAGE, *tail])
    return None
