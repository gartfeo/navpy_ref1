"""Step-1 normalizer for the SOLID guard: is a moved file materially new?

A file is compared with its old path at the pinned base (solid_step1_relocation)
after both sides are rewritten to one form: each import names the step-1 home of
the defining module, one name per statement, sorted within its block; `navpy.`
chains, dotted strings, `navpy/` paths and `"src", "navpy", ...` segments of
real path builders (os.path.join, pathlib paths, their joinpath and `/`;
solid_step1_paths) follow the move map; the module-level re-export bindings
c01 strips (reexports.tsv: name and origin module) leave imports and
`__all__`. A pure move or facade rewrite then dumps as its old form; other
changes do not. A split part or split rest is compared with the base
statements it owns (solid_step1_splits).
"""

from __future__ import annotations

import ast
import re
from functools import cached_property
from typing import Collection, Dict, Iterable, List, Mapping, Set

from tests.guards.solid_step1_relocation import (
    NAME_ROOTS,
    BaseNames,
    Namer,
    Piece,
    Relocation,
    absolute_module,
    module_level,
    module_of,
    parse,
)
from tests.guards.solid_step1_paths import PathBuilders
from tests.guards.solid_step1_scopes import free_names
from tests.guards.solid_step1_splits import (
    claimed_symbols,
    defined_names,
    dumps,
    owned_statements,
    split_view,
)

_DOTTED = re.compile(r"(?<![\w.])(?:navpy|gcs)(?:\.[A-Za-z_]\w*)+")
_PATH = re.compile(r"(?<![\w.-])(?:navpy|gcs)/[\w./-]*")
_SEGMENTED = re.compile(r"(?:src/)?(?:navpy|gcs)(?:/|$)")


class MovedBaseline:
    """The pinned base seen through the step-1 move map."""

    def __init__(
        self,
        pinned: Mapping[str, str],
        relocation: Relocation,
        present: Collection[str] = (),
    ) -> None:
        """present: the current tree's paths (which split parts exist)."""
        self._pinned = pinned
        self._relocation = relocation
        self._present = frozenset(present)

    @cached_property
    def _base(self) -> BaseNames:
        return BaseNames(self._pinned)

    @cached_property
    def _namer(self) -> Namer:
        return Namer(self._relocation, self._base)

    def is_material(self, relative: str, source: str) -> bool:
        pieces = self._relocation.pieces(relative)
        if any(piece.old_path not in self._pinned for piece in pieces):
            return True
        if self._relocation.is_split(relative):
            return not self._holds_only_old_code(relative, source, pieces)
        old = pieces[0].old_path
        stripped = self._stripped(pieces)
        return ast.dump(self._canonical(source, relative, stripped)) != ast.dump(
            self._canonical(self._pinned[old], old, stripped)
        )

    def _holds_only_old_code(
        self, relative: str, source: str, pieces: tuple[Piece, ...]
    ) -> bool:
        current = split_view(self._canonical(source, relative, self._stripped(pieces)))
        expected: List[str] = []
        old_imports: Set[str] = set()
        own_names: Set[str] = set()
        old_bindings: Dict[str, Set[str]] = {}
        future = False
        for piece in pieces:
            base = split_view(
                self._canonical(
                    self._pinned[piece.old_path],
                    piece.old_path,
                    self._stripped((piece,)),
                )
            )
            old_imports.update(dump for dump, _, _ in base.imports)
            for name, targets in base.bindings().items():
                old_bindings.setdefault(name, set()).update(targets)
            future = future or base.has_future_annotations
            claimed = claimed_symbols(self._relocation.parts, piece.old_path, self._present)
            expected.extend(owned_statements(piece, base, claimed))
            module = module_of(piece.old_path)
            if module is not None:
                for name in self._base.defined(module):
                    own = self._namer.imported(module, name)
                    own_names.add(own)
                    old_bindings.setdefault(name, set()).add(own)
        return (
            dumps(current.statements) == expected
            and (current.has_future_annotations or not future)
            and all(dump in old_imports or target in own_names
                    for dump, target, _ in current.imports)
            and not _unbound(current.statements, current.bindings(), old_bindings)
        )

    def _stripped(self, pieces: Iterable[Piece]) -> frozenset[tuple[str, str]]:
        return frozenset().union(
            *(self._relocation.stripped.get(piece.old_path, ()) for piece in pieces)
        )

    def _canonical(
        self, source: str, relative: str, stripped: frozenset[tuple[str, str]]
    ) -> ast.Module:
        tree = parse(source, relative)
        tree = _Canonical(self._namer, relative, stripped, tree).visit(tree)
        _tidy(tree)
        return tree


def _unbound(statements: Iterable[ast.stmt], bindings: Mapping[str, Set[str]],
             old_bindings: Mapping[str, Set[str]]) -> List[str]:
    """Names the statements read that the base bound by import or definition
    and the file no longer binds the same way (an import dropped or changed)."""
    statements = list(statements)
    defined = set().union(*(defined_names(statement) for statement in statements))
    return sorted(
        name for name in free_names(statements) - defined
        if name in old_bindings and not bindings.get(name, set()) & old_bindings[name]
    )


class _Canonical(ast.NodeTransformer):
    """Rewrites one module into the canonical step-1 spelling."""

    def __init__(
        self,
        namer: Namer,
        relative: str,
        stripped: frozenset[tuple[str, str]],
        tree: ast.Module,
    ) -> None:
        self._namer = namer
        self._module = module_of(relative)
        self._package = relative.endswith("/__init__.py")
        self._stripped = stripped
        self._stripped_names = frozenset(name for name, _ in stripped)
        self._top = frozenset(id(statement) for statement in module_level(tree.body))
        self._paths = PathBuilders(tree)

    def visit_ImportFrom(self, node: ast.ImportFrom) -> list[ast.stmt]:
        source = absolute_module(self._module, self._package, node.level, node.module)
        ours = source is not None and source.split(".")[0] in NAME_ROOTS
        statements: list[ast.stmt] = []
        for alias in node.names:
            bound = alias.asname or alias.name
            if self._strips(node, alias, source):
                continue
            module, name, level = node.module, alias.name, node.level
            if ours and alias.name == "*":
                module, level = self._namer.module(source), 0
            elif ours:
                module, _, name = self._namer.imported(source, alias.name).rpartition(".")
                level = 0
            statements.append(
                ast.ImportFrom(module=module, names=[ast.alias(name, bound)], level=level)
            )
        return statements

    def visit_Import(self, node: ast.Import) -> list[ast.stmt]:
        return [
            ast.Import(names=[ast.alias(self._module_name(alias.name), alias.asname)])
            for alias in node.names
        ]

    def visit_Attribute(self, node: ast.Attribute) -> ast.expr:
        dotted = _dotted(node)
        if dotted is None or dotted.split(".")[0] not in NAME_ROOTS:
            return self.generic_visit(node)
        return ast.Name(id=self._namer.dotted(dotted), ctx=node.ctx)

    def visit_Constant(self, node: ast.Constant) -> ast.Constant:
        if isinstance(node.value, str):
            text = _PATH.sub(lambda match: self._path(match.group(0)), node.value)
            node.value = _DOTTED.sub(lambda match: self._namer.dotted(match.group(0)), text)
        return node

    def visit_Call(self, node: ast.Call) -> ast.Call:
        builds_path = self._paths.builds_path(node)
        self.generic_visit(node)
        if builds_path:
            node.args = self._joined_path_segments(node.args)
        return node

    def visit_BinOp(self, node: ast.BinOp) -> ast.expr:
        """A `path / "src" / "navpy" / ...` chain, joined as one path."""
        if not (isinstance(node.op, ast.Div) and self._paths.is_path(node)):
            return self.generic_visit(node)
        operands: list[ast.expr] = []
        left: ast.expr = node
        while isinstance(left, ast.BinOp) and isinstance(left.op, ast.Div):
            operands.insert(0, left.right)
            left = left.left
        operands.insert(0, left)
        joined = self._joined_path_segments([self.visit(operand) for operand in operands])
        chain = joined[0]
        for operand in joined[1:]:
            chain = ast.BinOp(left=chain, op=ast.Div(), right=operand)
        return chain

    def visit_Assign(self, node: ast.Assign) -> ast.stmt | None:
        return self._strip_all(node, node.targets)

    def visit_AnnAssign(self, node: ast.AnnAssign) -> ast.stmt | None:
        return self._strip_all(node, [node.target])

    def _strips(self, node: ast.ImportFrom, alias: ast.alias, source: str | None) -> bool:
        """A module-level binding reexports.tsv lists: its name, and what it
        binds resolves to the same definition as the listed origin (the
        imported module or the module defining the name)."""
        if source is None or alias.name == "*" or id(node) not in self._top:
            return False
        bound = alias.asname or alias.name
        target = self._namer.imported(source, alias.name)
        return any(
            name == bound and self._namer.imported(origin, alias.name) == target
            for name, origin in self._stripped
        )

    def _module_name(self, dotted: str) -> str:
        return self._namer.module(dotted) if dotted.split(".")[0] in NAME_ROOTS else dotted

    def _path(self, text: str) -> str:
        core = text.rstrip(".")
        return self._namer.path(f"src/{core}").removeprefix("src/") + text[len(core):]

    def _joined_path_segments(self, args: list[ast.expr]) -> list[ast.expr]:
        """`"src", "navpy", "a", "b.py"` call arguments as one canonical path."""
        joined: list[ast.expr] = []
        run: list[str] = []
        for argument in [*args, None]:
            if isinstance(argument, ast.Constant) and isinstance(argument.value, str):
                run.append(argument.value)
                continue
            path = "/".join(run)
            if len(run) > 1 and _SEGMENTED.match(path):
                joined.append(ast.Constant(self._namer.path(path) if path.startswith("src/")
                                           else self._path(path)))
            else:
                joined.extend(ast.Constant(value) for value in run)
            run = []
            if argument is not None:
                joined.append(argument)
        return joined

    def _strip_all(self, node: ast.Assign | ast.AnnAssign, targets: list[ast.expr]) -> ast.stmt | None:
        """Drops stripped re-exports from a module-level `__all__`, and an
        `__all__` left empty: c01 may delete it or leave `[]`."""
        value = node.value
        if (
            self._stripped
            and id(node) in self._top
            and any(isinstance(target, ast.Name) and target.id == "__all__" for target in targets)
            and isinstance(value, (ast.List, ast.Tuple))
        ):
            value.elts = [
                element for element in value.elts
                if not (isinstance(element, ast.Constant)
                        and element.value in self._stripped_names)
            ]
            if not value.elts:
                return None
        return self.generic_visit(node)


_NESTED_BODIES = ("body", "orelse", "finalbody", "handlers", "cases")


def _tidy(node: ast.AST) -> None:
    """Sorts each block of imports; drops an `if` that stripping emptied."""
    for name in _NESTED_BODIES:
        items = getattr(node, name, None)
        if not isinstance(items, list) or not items or isinstance(items[0], ast.expr):
            continue
        for item in items:
            _tidy(item)
        if isinstance(items[0], ast.stmt):
            setattr(node, name, _sorted_imports([
                item for item in items
                if not (isinstance(item, ast.If) and not item.body and not item.orelse)
            ]))


def _sorted_imports(statements: list[ast.stmt]) -> list[ast.stmt]:
    result: list[ast.stmt] = []
    block: list[ast.stmt] = []
    for statement in [*statements, None]:
        if isinstance(statement, (ast.Import, ast.ImportFrom)):
            block.append(statement)
            continue
        result.extend(sorted(block, key=ast.dump) if len(block) > 1 else block)
        block = []
        if statement is not None:
            result.append(statement)
    return result


def _dotted(node: ast.AST) -> str | None:
    if isinstance(node, ast.Name):
        return node.id
    if isinstance(node, ast.Attribute):
        prefix = _dotted(node.value)
        return f"{prefix}.{node.attr}" if prefix else None
    return None
