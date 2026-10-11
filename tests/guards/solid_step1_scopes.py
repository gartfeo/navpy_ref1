"""Module-level names a block of statements reads, for the step-1 split check.

A name read inside a function is a module-level name unless the function
binds it (a parameter, an assignment, an import, a nested def or class) and
does not declare it global. A dotted Name the normalizer made from a `navpy.`
chain reads its first segment. Names in string annotations count as read.
"""

from __future__ import annotations

import ast
from typing import Iterable, List, Set, Union

_Function = Union[ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda]


def free_names(statements: Iterable[ast.stmt]) -> Set[str]:
    used: Set[str] = set()
    for statement in statements:
        _collect(statement, frozenset(), used, False)
    return used


def _collect(node: ast.AST, local: frozenset[str], used: Set[str], annotation: bool) -> None:
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda)):
        _collect_function(node, local, used)
    elif isinstance(node, ast.Name):
        head = node.id.split(".")[0]
        if isinstance(node.ctx, ast.Load) and head not in local:
            used.add(head)
    elif isinstance(node, ast.Constant):
        if annotation and isinstance(node.value, str):
            used.update(_string_names(node.value) - local)
    elif isinstance(node, ast.AnnAssign):
        _collect(node.target, local, used, False)
        _collect(node.annotation, local, used, True)
        if node.value is not None:
            _collect(node.value, local, used, False)
    else:
        for child in ast.iter_child_nodes(node):
            _collect(child, local, used, annotation)


def _collect_function(node: _Function, local: frozenset[str], used: Set[str]) -> None:
    """Decorators, defaults and annotations read the enclosing scope; the
    body reads it except for the names the function binds."""
    args = node.args
    every_arg = [arg for arg in (*args.posonlyargs, *args.args, *args.kwonlyargs,
                                 args.vararg, args.kwarg) if arg is not None]
    for expression in (*getattr(node, "decorator_list", ()), *args.defaults,
                       *(default for default in args.kw_defaults if default is not None)):
        _collect(expression, local, used, False)
    annotations = [arg.annotation for arg in every_arg if arg.annotation is not None]
    returns = getattr(node, "returns", None)
    for expression in [*annotations, *([returns] if returns is not None else [])]:
        _collect(expression, local, used, True)
    body = node.body if isinstance(node.body, list) else [node.body]
    inner = local | {arg.arg for arg in every_arg} | _bound_in(body)
    for statement in body:
        _collect(statement, inner, used, False)


def _bound_in(body: List[ast.AST]) -> Set[str]:
    """Names a function body binds, nested scopes left out, globals excluded."""
    bound: Set[str] = set()
    declared: Set[str] = set()
    stack = list(body)
    while stack:
        node = stack.pop()
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            bound.add(node.name)
            continue
        if isinstance(node, ast.Lambda):
            continue
        if isinstance(node, ast.Name) and isinstance(node.ctx, (ast.Store, ast.Del)):
            bound.add(node.id)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            bound.update((alias.asname or alias.name).split(".")[0] for alias in node.names)
        elif isinstance(node, (ast.Global, ast.Nonlocal)):
            declared.update(node.names)
        elif isinstance(node, ast.ExceptHandler) and node.name:
            bound.add(node.name)
        stack.extend(ast.iter_child_nodes(node))
    return bound - declared


def _string_names(text: str) -> Set[str]:
    try:
        expression = ast.parse(text, mode="eval")
    except SyntaxError:
        return set()
    return {node.id for node in ast.walk(expression) if isinstance(node, ast.Name)}
