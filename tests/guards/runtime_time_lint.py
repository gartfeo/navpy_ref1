"""Rules of the RuntimeTime port (``navpy.core.kernel.runtime_time``).

Covered product code reads clocks, waits, makes threads, queues, random
numbers and ids only through the port, and declares no set whose elements
hash by identity: such a set iterates in memory-address order, so a lockstep
replay could not reproduce it (``deterministic_demo_eval.md`` step 8).

The lint is the first net. The lockstep runtime's thread audit and the
per-tick journal catch what reaches a byte by a path it cannot see.
"""

from __future__ import annotations

import ast
import collections.abc
import dataclasses
import enum
import importlib
import sys
import types
import typing
import weakref
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
COVERED = ("src/navpy", "src/gcs/backend")
# Path prefix -> why it may use the host directly.
EXEMPT = {
    "src/navpy/core/kernel/runtime_time.py": "the port and its wall implementation",
    "src/navpy/sim/lockstep/": (
        "the lockstep runtime and its host-side harness: they implement the "
        "primitives on host threads and talk to the WSL relay"
    ),
    "src/navpy/plugins/vehicle/ardupilot/_mavftp/": (
        "vendored pymavlink MAVFTP: serial-step refuses FTP, and lockstep reads "
        "command bounds by parameter name"
    ),
    "src/navpy/plugins/gimbal/siyi/siyi_sdk.py": (
        "vendored SIYI SDK, pinned by hash: it drives the real gimbal over UDP "
        "and never runs in the simulated stack"
    ),
    "src/gcs/backend/instance_registry": (
        "the machine-wide slot registry: host process liveness and claim "
        "times, never a product outcome"
    ),
}
# The operator side of the GCS demo eval: its clock is StackContext.clock.
OPERATOR_GLOB = "scripts/eval_gcs_demo_*.py"
# Operator file -> (host module it may import, the one definition that may use it).
OPERATOR_SCOPED = {
    "scripts/eval_gcs_demo_execution.py": ("time", "SystemClock"),  # the wall ClockPort
    "scripts/eval_gcs_demo_process.py": ("secrets", "launch_stack"),  # the registry-slot token
}
# The composition roots that create the operator's wall clock.
SYSTEM_CLOCK_ROOTS = frozenset({"scripts/eval_gcs_demo_runtime.py"})

# Modules, and members, whose every use is a host clock, thread or entropy.
BANNED_MODULES = frozenset({
    "time", "timeit", "threading", "_thread", "queue", "random", "uuid", "secrets",
    "concurrent.futures", "numpy.random", "os.urandom",
})
WALL_DATETIME = frozenset({
    "datetime.datetime.now", "datetime.datetime.today", "datetime.datetime.utcnow",
    "datetime.date.today",
})
PORT = "navpy.core.kernel.runtime_time"
# Building the wall runtime directly would bypass the bound one.
PORT_INTERNALS = frozenset(
    f"{PORT}.{name}"
    for name in ("WallClock", "WallThreads", "WallEntropy", "wall_runtime", "RuntimeBinding", "_binding")
)
IMPORTERS = frozenset({"__import__", "importlib.import_module"})
# A dynamic import of these may stay: they are the covered packages themselves.
FIRST_PARTY_PREFIXES = ("navpy.", "gcs.")
VALUE_HASHED = (int, str, float, bytes, bool, complex, type(None))


@dataclass(frozen=True)
class Finding:
    path: str
    line: int
    rule: str

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: {self.rule}"


def covered_files() -> list[tuple[str, Path]]:
    files = []
    for root in COVERED:
        for path in sorted((ROOT / root).rglob("*.py")):
            relative = path.relative_to(ROOT).as_posix()
            if not any(relative.startswith(prefix) for prefix in EXEMPT):
                files.append((relative, path))
    return files


def operator_files() -> list[tuple[str, Path]]:
    return [
        (path.relative_to(ROOT).as_posix(), path)
        for path in sorted(ROOT.glob(OPERATOR_GLOB))
    ]


def module_name(path: str) -> str:
    return path.removeprefix("src/").removesuffix(".py").replace("/", ".").removesuffix(".__init__")


def _package_of(path: str) -> str:
    """The package a relative import in this file starts from."""
    module = module_name(path)
    return module if path.endswith("__init__.py") else module.rpartition(".")[0]


def _is_type_checking(test: ast.expr) -> bool:
    """Exactly ``TYPE_CHECKING`` or ``typing.TYPE_CHECKING``, never a negation."""
    if isinstance(test, ast.Name):
        return test.id == "TYPE_CHECKING"
    return (
        isinstance(test, ast.Attribute)
        and test.attr == "TYPE_CHECKING"
        and isinstance(test.value, ast.Name)
        and test.value.id == "typing"
    )


def _type_checking_nodes(tree: ast.AST) -> set[int]:
    """ids of nodes in the body of ``if TYPE_CHECKING:`` (annotation-only)."""
    inside: set[int] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.If) and _is_type_checking(node.test):
            for child in node.body:
                inside.update(id(sub) for sub in ast.walk(child))
    return inside


def _banned(symbol: str) -> bool:
    return any(symbol == name or symbol.startswith(name + ".") for name in BANNED_MODULES)


def _stands_for(symbols: set[str], module: str) -> bool:
    return any(symbol == module or symbol.startswith(module + ".") for symbol in symbols)


class _Symbols:
    """What each name of one module may stand for, as qualified symbols.

    Imports bind names (``import datetime as dt``: ``dt`` is ``datetime``;
    ``from importlib import import_module``: ``importlib.import_module``;
    a relative import keeps its dots). Assignments alias them: names,
    attribute chains (``self.kind = datetime``) and tuple unpacking.
    """

    _ROUNDS = 8  # alias chains are short; this bounds self-referencing ones

    def __init__(self, tree: ast.AST, path: str | None = None) -> None:
        self._package = _package_of(path) if path else None
        self._bound: dict[str, set[str]] = {"__import__": {"__import__"}}  # a builtin
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    name = alias.asname or alias.name.split(".")[0]
                    self._bind(name, {alias.name if alias.asname else name})
            elif isinstance(node, ast.ImportFrom):
                base = self.import_base(node)
                for alias in node.names:
                    if alias.name != "*":
                        self._bind(alias.asname or alias.name, {f"{base}.{alias.name}"})
        pairs = []
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):  # ``clock = rt`` read as ``self.clock``
                for statement in node.body:
                    if isinstance(statement, (ast.Assign, ast.AnnAssign)) and statement.value is not None:
                        targets = statement.targets if isinstance(statement, ast.Assign) else [statement.target]
                        for target in targets:
                            if isinstance(target, ast.Name):
                                pairs += [(f"{owner}.{target.id}", statement.value) for owner in ("self", "cls")]
            if isinstance(node, (ast.Assign, ast.AnnAssign)) and node.value is not None:
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                for target in targets:
                    pairs += self._pairs(target, node.value)
            elif isinstance(node, ast.arguments):  # defaults: ``def f(clock=datetime)``
                positional = node.posonlyargs + node.args
                for arg, default in zip(positional[len(positional) - len(node.defaults):], node.defaults):
                    pairs.append((arg.arg, default))
                for arg, default in zip(node.kwonlyargs, node.kw_defaults):
                    if default is not None:
                        pairs.append((arg.arg, default))
        for _ in range(self._ROUNDS):
            grown = False
            for key, value in pairs:
                grown |= self._bind(key, self.resolve(value, bound_only=True))
            if not grown:
                break

    def import_base(self, node: ast.ImportFrom) -> str:
        """The absolute module a ``from ... import`` reads; relative ones
        resolve against the file's package (unknown package: dots kept)."""
        if node.level == 0:
            return node.module or ""
        if self._package is None:
            return "." * node.level + (node.module or "")
        parts = self._package.split(".")
        parts = parts[: len(parts) - (node.level - 1)]
        return ".".join(parts + ([node.module] if node.module else []))

    @staticmethod
    def _pairs(target: ast.expr, value: ast.expr) -> list[tuple[str, ast.expr]]:
        if isinstance(target, (ast.Tuple, ast.List)) and isinstance(value, (ast.Tuple, ast.List)):
            return [
                pair
                for inner_target, inner_value in zip(target.elts, value.elts)
                for pair in _Symbols._pairs(inner_target, inner_value)
            ]
        if isinstance(target, (ast.Name, ast.Attribute)):
            return [(ast.unparse(target), value)]
        return []

    def _bind(self, name: str, symbols: set[str]) -> bool:
        known = self._bound.setdefault(name, set())
        new = symbols - known
        known |= new
        return bool(new)

    def resolve(self, expression: ast.expr, *, bound_only: bool = False) -> set[str]:
        """Qualified symbols; with ``bound_only``, nothing for a name nothing binds."""
        if isinstance(expression, ast.Name):
            bound = self._bound.get(expression.id)
            return set(bound) if bound else (set() if bound_only else {expression.id})
        if isinstance(expression, ast.Attribute):
            symbols = set(self._bound.get(ast.unparse(expression), ()))
            return symbols | {
                f"{owner}.{expression.attr}"
                for owner in self.resolve(expression.value, bound_only=bound_only)
            }
        if isinstance(expression, ast.Call):  # import_module("datetime") is datetime
            importer = self.resolve(expression.func) & IMPORTERS
            target = expression.args[0] if expression.args else next(
                (keyword.value for keyword in expression.keywords if keyword.arg == "name"), None,
            )
            if importer and isinstance(target, ast.Constant) and isinstance(target.value, str):
                name = target.value
                return {name.split(".")[0] if importer == {"__import__"} else name}
        return set()


def outermost_references(tree: ast.AST) -> list[ast.expr]:
    """Loaded names and attribute chains, each chain once at its outermost node."""
    owners = {id(node.value) for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    return [
        node for node in ast.walk(tree)
        if isinstance(node, (ast.Name, ast.Attribute)) and isinstance(node.ctx, ast.Load)
        and id(node) not in owners
    ]


def _dynamic_import(node: ast.Call, symbols: _Symbols) -> str | None:
    """The module a dynamic import may load, unless it is first-party."""
    if not symbols.resolve(node.func) & IMPORTERS:
        return None
    target = node.args[0] if node.args else next(
        (keyword.value for keyword in node.keywords if keyword.arg == "name"), None,
    )
    if isinstance(target, ast.Constant) and isinstance(target.value, str):
        return target.value if _banned(target.value) else None
    if (
        isinstance(target, ast.JoinedStr)
        and target.values
        and isinstance(target.values[0], ast.Constant)
        and str(target.values[0].value).startswith(FIRST_PARTY_PREFIXES)
    ):
        return None
    return "a module named at run time"


def _import_findings(path: str, node: ast.AST, symbols: _Symbols) -> list[Finding]:
    if isinstance(node, ast.Import):
        return [
            Finding(path, node.lineno, f"import {alias.name}: use runtime_time")
            for alias in node.names if _banned(alias.name)
        ]
    if not isinstance(node, ast.ImportFrom):
        return []
    if any(alias.name == "*" for alias in node.names):
        return [Finding(path, node.lineno, f"from {node.module} import *: import names explicitly")]
    base = symbols.import_base(node)
    if _banned(base) or any(_banned(f"{base}.{alias.name}") for alias in node.names):
        return [Finding(path, node.lineno, f"from {base} import: use runtime_time")]
    return []


def _scope_names(tree: ast.AST) -> dict[int, set[str]]:
    """Each node id -> names of the classes and functions enclosing it."""
    scopes: dict[int, set[str]] = {}

    def visit(node: ast.AST, enclosing: frozenset[str]) -> None:
        scopes[id(node)] = set(enclosing)
        if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            enclosing = enclosing | {node.name}
        for child in ast.iter_child_nodes(node):
            visit(child, enclosing)

    visit(tree, frozenset())
    return scopes


def host_use_findings(
    path: str, source: str, allowed: tuple[str, str] | None = None,
) -> list[Finding]:
    """Banned imports and references (static or dynamic), wall ``datetime``
    reads and the port's internals.

    ``allowed`` = (module, owner) exempts references to that module inside
    the class or function named owner.
    """
    tree = ast.parse(source)
    annotation_only = _type_checking_nodes(tree)
    symbols = _Symbols(tree, path)
    scopes = _scope_names(tree)
    findings = []
    for node in ast.walk(tree):
        if id(node) in annotation_only:
            continue
        findings += _import_findings(path, node, symbols)
        if isinstance(node, ast.Call):
            dynamic = _dynamic_import(node, symbols)
            if dynamic is not None:
                findings.append(Finding(path, node.lineno, f"dynamic import of {dynamic}: use runtime_time"))
    for node in outermost_references(tree):
        if id(node) in annotation_only:
            continue
        resolved = symbols.resolve(node, bound_only=True)
        wall = resolved & WALL_DATETIME
        internals = (resolved & PORT_INTERNALS) | {
            symbol for symbol in resolved if symbol.startswith(f"{PORT}._")
        }
        banned = {symbol for symbol in resolved if _banned(symbol)}
        if allowed is not None and allowed[1] in scopes[id(node)]:
            banned = {symbol for symbol in banned if not _stands_for({symbol}, allowed[0])}
        if wall:
            findings.append(Finding(path, node.lineno, f"{min(wall)}: use runtime_time.now"))
        elif internals:
            findings.append(Finding(path, node.lineno, f"{min(internals)}: use the bound runtime (rt)"))
        elif banned:
            findings.append(Finding(path, node.lineno, f"{min(banned)}: use runtime_time"))
    return findings


def operator_findings(path: str, source: str) -> list[Finding]:
    """Host use in an operator file: one module, imported plainly, used by one
    owner; and no runtime port: the operator's clock is StackContext.clock."""
    scoped = OPERATOR_SCOPED.get(path)
    tree = ast.parse(source)
    findings = host_use_findings(path, source, scoped)
    if scoped is not None:
        module = scoped[0]
        plain_imports = {
            node.lineno for node in tree.body
            if isinstance(node, ast.Import)
            and [(alias.name, alias.asname) for alias in node.names] == [(module, None)]
        }
        findings = [
            f for f in findings
            if not (f.rule == f"import {module}: use runtime_time" and f.line in plain_imports)
        ]
    symbols = _Symbols(tree, path)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and symbols.resolve(node.func) & IMPORTERS:
            findings.append(Finding(path, node.lineno, "dynamic import in the operator: import statically"))
        imported = (
            {alias.name for alias in node.names} if isinstance(node, ast.Import)
            else {f"{symbols.import_base(node)}.{alias.name}" for alias in node.names}
            | {symbols.import_base(node)}
            if isinstance(node, ast.ImportFrom) else set()
        )
        if _stands_for(imported, PORT):
            findings.append(Finding(path, node.lineno, "runtime_time in the operator: take StackContext.clock"))
    for node in outermost_references(tree):
        if _stands_for(symbols.resolve(node, bound_only=True), PORT):
            findings.append(Finding(path, node.lineno, "runtime_time in the operator: take StackContext.clock"))
    return findings


def system_clock_findings(path: str, source: str) -> list[Finding]:
    """Any reference to SystemClock, under any name, outside the composition root."""
    if path in SYSTEM_CLOCK_ROOTS:
        return []
    tree = ast.parse(source)
    symbols = _Symbols(tree, path)
    return [
        Finding(path, node.lineno, "SystemClock: take StackContext.clock")
        for node in ast.walk(tree)
        if isinstance(node, (ast.Name, ast.Attribute)) and isinstance(node.ctx, ast.Load)
        and any(symbol.rsplit(".", 1)[-1] == "SystemClock" for symbol in symbols.resolve(node))
    ]


# -- identity-hashed sets -----------------------------------------------------
#
# The rule is on declarations, not on iteration: covered code must not
# declare a set (in any annotation: attributes, fields, parameters, returns,
# locals, aliases, nested in containers or typing wrappers) whose element
# type hashes by object identity. Without such a set there is nothing whose
# iteration order could follow memory addresses, whatever path it would take.
# A set built with no annotation at all is outside this first net; the
# lockstep journal catches the order it produces at the first tick it
# reaches a byte.

SET_ORIGINS = (set, frozenset, collections.abc.Set)
# A hand-written __hash__ in this repository may hash an identity; the
# standard library's and third parties' value types are trusted.
def _hand_written_first_party(hash_fn: object) -> bool:
    """A Python ``__hash__`` written in this repository (not dataclass code,
    not the standard library or a third party, whose value types are trusted)."""
    if not isinstance(hash_fn, types.FunctionType):
        return False
    source = hash_fn.__code__.co_filename  # "<string>" for dataclass-generated code
    if source == "<string>":
        return False
    path = Path(source).resolve()
    return path.is_relative_to(ROOT) and not {".venv", "site-packages"} & set(path.parts)


def _forward_chain(kind: object, namespace: dict) -> tuple[object, list[str]]:
    """Resolve a forward reference (a string or ForwardRef) through its whole
    alias chain: (the type or None if a step fails or the chain cycles, every
    text met on the way)."""
    texts: list[str] = []
    while isinstance(kind, (str, typing.ForwardRef)):
        text = kind if isinstance(kind, str) else kind.__forward_arg__
        if text in texts:
            return None, texts  # a cycle
        texts.append(text)
        try:
            kind = eval(text, dict(namespace))  # noqa: S307 - our own annotations
        except Exception:
            return None, texts
    return kind, texts


def _forward(kind: object, namespace: dict) -> object:
    return _forward_chain(kind, namespace)[0]


def _arguments(kind: object) -> list[object]:
    """Type arguments, with Callable's parameter list flattened."""
    found = []
    for arg in typing.get_args(kind):
        if arg is Ellipsis:
            continue
        found += list(arg) if isinstance(arg, (list, tuple)) else [arg]
    return found


def id_hashed(kind: object, namespace: dict | None = None,
              seen: frozenset[int] = frozenset()) -> bool:
    """Whether instances of ``kind`` hash by object identity (unknown: yes)."""
    namespace = namespace or {}
    if isinstance(kind, (str, typing.ForwardRef)):
        kind = _forward(kind, namespace)
    if kind is None or id(kind) in seen:
        return kind is None
    seen = seen | {id(kind)}
    origin = typing.get_origin(kind)
    if origin is not None:
        args = _arguments(kind)
        if origin is typing.Literal:  # values: classify each by its own type
            return any(id_hashed(type(value), namespace, seen) for value in typing.get_args(kind))
        if origin is typing.Annotated:
            return id_hashed(args[0], namespace, seen)
        if origin in (typing.Union, types.UnionType, tuple, frozenset):  # contents decide
            return any(id_hashed(arg, namespace, seen) for arg in args)
        if isinstance(origin, type):  # a generic class: its own hash, then its arguments
            return id_hashed(origin, namespace, seen) or any(
                id_hashed(arg, namespace, seen) for arg in args
            )
        return True
    if hasattr(kind, "__supertype__"):  # NewType
        return id_hashed(kind.__supertype__, namespace, seen)
    if isinstance(kind, types.FunctionType):  # runtime_time factories: Event, Thread, ...
        return True
    if not isinstance(kind, type):  # TypeVars and the like
        return True
    if kind in VALUE_HASHED:
        return False
    if getattr(kind, "_is_protocol", False):  # any structural implementation
        return True
    if (
        (issubclass(kind, (tuple, frozenset)) and not hasattr(kind, "_fields"))
        or issubclass(kind, weakref.ref)
    ):  # a bare container, or a weak reference: the contents decide, and are unknown
        return True
    hash_fn = kind.__hash__
    if hash_fn is None:  # unhashable: never a set element
        return False
    if (
        hash_fn is object.__hash__
        or getattr(hash_fn, "__isabstractmethod__", False)  # Hashable: any hash at all
        or _hand_written_first_party(hash_fn)
    ):
        return True
    if dataclasses.is_dataclass(kind) or (issubclass(kind, tuple) and hasattr(kind, "_fields")):
        return _fields_id_hashed(kind, namespace, seen)
    return False  # a value hash from the standard library or a third party


def _fields_id_hashed(kind: type, namespace: dict, seen: frozenset[int]) -> bool:
    """The fields a generated hash covers: dataclass fields with ``hash``
    (default: ``compare``) set, or every namedtuple field. A field with no
    type (an untyped namedtuple) is unknown."""
    try:
        hints = typing.get_type_hints(kind)
    except Exception:  # resolve the hashed fields one by one
        hints = {}
        module = vars(importlib.import_module(kind.__module__)) if kind.__module__ in sys.modules else {}
        for name, annotation in getattr(kind, "__annotations__", {}).items():
            resolved = _forward(annotation, {**module, **namespace})
            if resolved is not None:
                hints[name] = resolved
    if dataclasses.is_dataclass(kind):
        names = [
            field.name for field in dataclasses.fields(kind)
            if (field.hash if field.hash is not None else field.compare)
        ]
    else:
        names = list(kind._fields)
    if any(name not in hints for name in names):
        return True
    return any(id_hashed(hints[name], namespace, seen) for name in names)


def identity_sets(kind: object, namespace: dict | None = None,
                  seen: frozenset[int] = frozenset()) -> list[str]:
    """Every set type inside ``kind`` whose elements hash by identity, and
    every forward reference naming a set that does not resolve."""
    namespace = namespace or {}
    if isinstance(kind, (str, typing.ForwardRef)):
        resolved, texts = _forward_chain(kind, namespace)
        if resolved is None:  # fail closed when any step of the chain named a set
            return [f"unresolved {texts[-1]!r}"] if any(map(_mentions_a_set, texts)) else []
        kind = resolved
    if hasattr(kind, "__supertype__"):  # NewType("Clients", set[Client])
        return identity_sets(kind.__supertype__, namespace, seen)
    if hasattr(kind, "__value__") and not isinstance(kind, type):  # a TypeAliasType
        return identity_sets(kind.__value__, namespace, seen)
    if id(kind) in seen:
        return []
    seen = seen | {id(kind)}
    origin = typing.get_origin(kind)
    if origin is typing.Literal:  # values, not types
        return []
    args = _arguments(kind)
    found = []
    container = origin if origin is not None else kind
    if isinstance(container, type) and issubclass(container, SET_ORIGINS):
        if id_hashed(args[0] if args else None, namespace):  # a bare set: unknown elements
            found.append(repr(kind))
    for arg in args if origin is not typing.Annotated else args[:1]:
        found += identity_sets(arg, namespace, seen)
    return found


def _module_statements(tree: ast.Module) -> list[ast.stmt]:
    """Module-level statements, including those inside if/try/with blocks."""
    found: list[ast.stmt] = []
    pending = list(tree.body)
    while pending:
        node = pending.pop(0)
        found.append(node)
        if isinstance(node, (ast.If, ast.Try, ast.With)):
            for block in ("body", "orelse", "finalbody"):
                pending += getattr(node, block, [])
            for handler in getattr(node, "handlers", []):
                pending += handler.body
    return found


def _set_words(tree: ast.Module) -> set[str]:
    """Set type names, plus the names a module imports them under."""
    return set(SET_WORDS) | {
        alias.asname
        for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
        for alias in node.names if alias.name in SET_WORDS and alias.asname
    }


def set_alias_names(sources: dict[str, str]) -> set[str]:
    """Module-level names assigned a type that mentions a set, anywhere."""
    names = set()
    for source in sources.values():
        tree = ast.parse(source)
        words = _set_words(tree)
        for node in _module_statements(tree):
            if (
                isinstance(node, (ast.Assign, ast.AnnAssign)) and node.value is not None
                and _words(ast.unparse(node.value)) & words
            ):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                names |= {target.id for target in targets if isinstance(target, ast.Name)}
    return names


SET_WORDS = frozenset({"set", "frozenset", "Set", "FrozenSet", "AbstractSet", "MutableSet"})


def _words(text: str) -> set[str]:
    for mark in "[](),|.'\"":
        text = text.replace(mark, " ")
    return set(text.split())


def _mentions_a_set(text: str) -> bool:
    return bool(_words(text) & SET_WORDS)


def _annotations(tree: ast.Module) -> list[tuple[int, ast.expr]]:
    """Every annotation in the module (assignments, parameters, returns), and
    every module-level alias value that names a set (``Clients = set[X]``,
    ``Clients: TypeAlias = ...``)."""
    found = []
    words = _set_words(tree)
    for node in _module_statements(tree):
        if (
            isinstance(node, (ast.Assign, ast.AnnAssign)) and node.value is not None
            and _words(ast.unparse(node.value)) & words
        ):
            found.append((node.lineno, node.value))
    for node in ast.walk(tree):
        if isinstance(node, ast.AnnAssign):
            found.append((node.lineno, node.annotation))
        elif isinstance(node, ast.arg) and node.annotation is not None:
            found.append((node.lineno, node.annotation))
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.returns is not None:
            found.append((node.lineno, node.returns))
    return found


def _annotation_text(annotation: ast.expr) -> str:
    if isinstance(annotation, ast.Constant) and isinstance(annotation.value, str):
        return annotation.value  # a quoted annotation
    return ast.unparse(annotation)


def identity_set_findings(path: str, source: str, module: str,
                          aliases: set[str] = frozenset(),
                          namespace: dict | None = None) -> list[Finding]:
    """Declared sets whose elements hash by identity. Every annotation is
    evaluated in the module's namespace (``namespace`` overrides importing
    ``module``); one that names a set and does not resolve is a finding."""
    tree = ast.parse(source)
    annotations = _annotations(tree)
    if not annotations:
        return []
    namespace = dict(namespace if namespace is not None else vars(importlib.import_module(module)))
    for node in ast.walk(tree):  # names declared for annotations only
        if isinstance(node, ast.If) and _is_type_checking(node.test):
            for statement in node.body:
                if isinstance(statement, (ast.Import, ast.ImportFrom, ast.Assign, ast.AnnAssign)):
                    try:
                        exec(compile(ast.Module([statement], []), path, "exec"), namespace)  # noqa: S102
                    except Exception:
                        pass
    words = _set_words(tree) | set(aliases)
    findings = []
    for line, annotation in annotations:
        text = _annotation_text(annotation)
        try:
            kind = eval(text, namespace)  # noqa: S307 - our own annotations
        except Exception:
            if _words(text) & words:
                findings.append(Finding(path, line, f"cannot resolve {text!r}: declare the set with resolvable types"))
            continue
        for found in identity_sets(kind, namespace):
            findings.append(Finding(path, line, f"{found} orders by object identity: keep a dict"))
    return findings
