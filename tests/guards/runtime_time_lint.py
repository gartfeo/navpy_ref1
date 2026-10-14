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
import sysconfig
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
    "src/navpy/sim/radio/serial_sim_server.py": (
        "the integration-test serial radio simulator: its own host process "
        "(scripts/tests_integration), never part of the demo eval stack"
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
    # The lockstep run's host wall time, for its *_host sidecar only.
    "scripts/eval_gcs_demo_lockstep.py": ("time", "execute_lockstep_run"),
}
# The composition roots that create the operator's wall clock.
SYSTEM_CLOCK_ROOTS = frozenset({"scripts/eval_gcs_demo_runtime.py"})

# Modules, and members, whose every use is a host clock, thread or entropy.
BANNED_MODULES = frozenset({
    "time", "timeit", "threading", "_thread", "queue", "random", "uuid", "secrets",
    "concurrent.futures", "numpy.random", "os.urandom",
    "multiprocessing", "email.utils",  # host processes and pools; formatdate, localtime, make_msgid
})
# Host clocks, entropy and workers in the public API of reviewed modules.
BANNED_MEMBERS = frozenset({
    "cv2.getTickCount", "cv2.getTickFrequency", "cv2.getCPUTickCount",
    "torch.rand", "torch.rand_like", "torch.randn", "torch.randn_like",
    "torch.randint", "torch.randint_like", "torch.randperm", "torch.bernoulli",
    "torch.multinomial", "torch.normal", "torch.poisson", "torch.manual_seed",
    "torch.seed", "torch.random", "torch.cuda.manual_seed", "torch.Generator",
    "logging.handlers.QueueListener", "os.getrandom", "os.times",
    "socket.getdefaulttimeout",
})
# Every module outside navpy/gcs that covered code imports, reviewed: a new
# one fails until it is added here. Several are host-capable boundaries
# (sockets, serial, MAVLink transport, asyncio, logging, ML runtimes): the
# lint cannot see their internal clocks and threads; step 9 replaces the
# transports and its thread audit and journal catch the rest.
EXTERNAL_IMPORTS = frozenset({
    "__future__", "abc", "anyio.to_thread", "argparse", "asyncio", "atexit", "base64", "bisect",
    "boxmot.motion.cmc", "boxmot.reid.core", "boxmot.trackers.bbox.botsort.botsort",
    "boxmot.trackers.bbox.strongsort.strongsort", "builtins", "collections",
    "collections.abc", "contextlib", "contextvars", "copy", "csv", "ctypes", "cv2",
    "dataclasses", "datetime", "enum", "fastapi", "fastapi.middleware.cors",
    "fastapi.responses", "fastapi.staticfiles", "geopy.distance", "geopy.point",
    "glob", "hashlib", "http.server", "importlib", "io", "itertools", "json",
    "logging", "logging.handlers", "math", "numbers", "numpy", "orjson", "os",
    "pathlib", "pydantic", "pymap3d", "pymavlink", "pymavlink.dialects.v20.ardupilotmega",
    "pymavlink.generator.mavcrc", "pymavlink.mavextra", "pymavlink.mavutil",
    "pymavlink.mavwp", "re", "requests", "serial", "shutil", "socket",
    "starlette.background", "starlette.types", "struct", "subprocess", "sys",
    "tempfile", "tkinter", "torch", "traceback", "types", "typing", "ultralytics",
    "urllib.parse", "weakref", "zipfile", "zmq",
})
WALL_DATETIME = frozenset({
    "datetime.datetime.now", "datetime.datetime.today", "datetime.datetime.utcnow",
    "datetime.date.today",
})
PRODUCT = ("navpy", "gcs")
FIRST_PARTY = (*PRODUCT, "scripts")  # runtime identity: the operator's modules too
CLOCK_CLASSES = frozenset(symbol.rsplit(".", 1)[0] for symbol in WALL_DATETIME)
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
# Third-party types reviewed to hash by value (none needed yet).
REVIEWED_VALUE_HASHES: frozenset[type] = frozenset()
# Bound methods hash their receiver, whose hash is unknown.
RECEIVER_HASHED = (types.MethodType, types.BuiltinMethodType, types.MethodWrapperType)
SET_WORDS = frozenset({"set", "frozenset", "Set", "FrozenSet", "AbstractSet", "MutableSet"})
UNIONS = (typing.Union, getattr(types, "UnionType", typing.Union))  # 3.9: no UnionType


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


def external_imports(tree: ast.AST) -> set[str]:
    """Modules outside navpy/gcs a file imports (absolute imports)."""
    found = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found |= {alias.name for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            found.add(node.module)
    return {name for name in found if name.split(".", 1)[0] not in PRODUCT}


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
    return any(
        symbol == name or symbol.startswith(name + ".") for name in BANNED_MODULES | BANNED_MEMBERS
    )


LOCAL = "<local>"  # the symbols of a module's own classes


def _class_statements(node: ast.ClassDef) -> list[ast.stmt]:
    """The statements a class body runs, in any nested block (if, for, try,
    with, ...), but not those of its methods or nested classes."""
    found: list[ast.stmt] = []
    pending = list(node.body)
    while pending:
        statement = pending.pop(0)
        found.append(statement)
        if isinstance(statement, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            continue
        pending += [child for child in ast.iter_child_nodes(statement) if isinstance(child, ast.stmt)]
        for handler in getattr(statement, "handlers", []):
            pending += handler.body
        for case in getattr(statement, "cases", []):
            pending += case.body
    return found


def _stands_for(symbols: set[str], module: str) -> bool:
    return any(symbol == module or symbol.startswith(module + ".") for symbol in symbols)


class _Symbols:
    """What each name of one module may stand for, as qualified symbols.

    Imports bind names (``import datetime as dt``: ``dt`` is ``datetime``;
    ``from importlib import import_module``: ``importlib.import_module``;
    a relative import keeps its dots). Assignments alias them: names,
    attribute chains (``self.kind = datetime``) and tuple unpacking. A class
    of the module is the symbol ``<local>.Name``, standing also for its
    bases; its attributes are ``<local>.Name.attr``, read through the class,
    a subclass, ``self`` or ``cls``.
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
            elif isinstance(node, ast.ClassDef):
                self._bind(node.name, {f"{LOCAL}.{node.name}"})
        pairs = []
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                # ``class Stamp(datetime)``: ``Stamp.now`` is ``datetime.now``
                pairs += [(node.name, base) for base in node.bases]
                pairs += [("super()", base) for base in node.bases]  # ``super().now()``
                pairs += [(owner, ast.Name(node.name, ast.Load())) for owner in ("self", "cls")]
                for statement in _class_statements(node):  # ``clock = rt``: ``Hub.clock``
                    if isinstance(statement, (ast.Assign, ast.AnnAssign)) and statement.value is not None:
                        targets = statement.targets if isinstance(statement, ast.Assign) else [statement.target]
                        for target in targets:
                            pairs += [
                                (f"{LOCAL}.{node.name}.{key}", value)
                                for key, value in self._pairs(target, statement.value)
                                if key.isidentifier()
                            ]
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
            for owner in self.resolve(expression.value, bound_only=bound_only):
                member = f"{owner}.{expression.attr}"
                symbols |= {member} | self._bound.get(member, set())
            return symbols
        if isinstance(expression, ast.BoolOp):  # ``provider or datetime``: either
            return set().union(*(self.resolve(value, bound_only=bound_only) for value in expression.values))
        if isinstance(expression, ast.IfExp):  # ``a if test else b``: either
            return self.resolve(expression.body, bound_only=bound_only) | self.resolve(
                expression.orelse, bound_only=bound_only,
            )
        if isinstance(expression, ast.Call):  # import_module("datetime") is datetime
            importer = self.resolve(expression.func) & IMPORTERS
            target = expression.args[0] if expression.args else next(
                (keyword.value for keyword in expression.keywords if keyword.arg == "name"), None,
            )
            if importer and isinstance(target, ast.Constant) and isinstance(target.value, str):
                name = target.value
                return {name.split(".")[0] if importer == {"__import__"} else name}
            if isinstance(expression.func, ast.Name) and expression.func.id == "super":
                return set(self._bound.get("super()", ()))
            # an instance carries its class's members: ``API().port``, ``datetime(...).now``
            called = self.resolve(expression.func, bound_only=bound_only)
            return {
                symbol for symbol in called | reexported(called)
                if symbol.startswith(f"{LOCAL}.") or symbol in CLOCK_CLASSES
            }
        return set()


_MISSING = object()


def runtime_object(symbol: str) -> object:
    """The object a first-party qualified symbol names at runtime (importing
    its module), or a sentinel when it is not first-party or not found."""
    parts = symbol.split(".")
    if parts[0] not in FIRST_PARTY:
        return _MISSING
    found: object = importlib.import_module(parts[0])
    for index, attribute in enumerate(parts[1:], start=2):
        member = getattr(found, attribute, _MISSING)
        if member is _MISSING and isinstance(found, types.ModuleType):
            try:  # a submodule not imported yet
                member = importlib.import_module(".".join(parts[:index]))
            except Exception:
                return _MISSING
        if member is _MISSING:
            return _MISSING
        found = member
    return found


def reexported(symbols: set[str]) -> set[str]:
    """What first-party symbols are at runtime, as host symbols: a module, a
    function or class by its home module (``helper.clock`` re-exporting
    ``runtime_time`` is the port), a bound wall ``datetime`` read."""
    found = set()
    for held in map(runtime_object, symbols):
        if held is _MISSING:
            continue
        if isinstance(held, types.ModuleType):
            found.add(held.__name__)
            continue
        if isinstance(held, type):  # a class stands for its bases: Timestamp(datetime)
            found |= {f"{base.__module__}.{base.__qualname__}" for base in held.__mro__}
        owner = getattr(held, "__self__", None)  # a bound method: datetime.now
        if isinstance(owner, type):  # named by the class defining it (Timestamp.now: datetime.now)
            method = getattr(held, "__name__", "")
            definer = next((base for base in owner.__mro__ if method in base.__dict__), owner)
            found.add(f"{definer.__module__}.{definer.__qualname__}.{method}")
        home, name = getattr(held, "__module__", None), getattr(held, "__qualname__", None)
        if isinstance(home, str) and isinstance(name, str):
            found.add(f"{home}.{name}")
        elif not callable(held):  # an instance matters as a clock: a datetime, a SystemClock
            found |= {
                symbol for base in type(held).__mro__
                for symbol in [f"{base.__module__}.{base.__qualname__}"]
                if symbol in CLOCK_CLASSES or base.__name__ == "SystemClock"
            }
    return found


def outermost_references(tree: ast.AST) -> list[ast.expr]:
    """Loaded names and attribute chains, each chain once at its outermost node."""
    owners = {id(node.value) for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
    return [
        node for node in ast.walk(tree)
        if isinstance(node, (ast.Name, ast.Attribute)) and isinstance(node.ctx, ast.Load)
        and id(node) not in owners
    ]


def _dynamic_import(node: ast.Call, symbols: _Symbols) -> str | None:
    """The module a dynamic import may load, unless it is first-party or a
    reviewed external module."""
    if not symbols.resolve(node.func) & IMPORTERS:
        return None
    target = node.args[0] if node.args else next(
        (keyword.value for keyword in node.keywords if keyword.arg == "name"), None,
    )
    if isinstance(target, ast.Constant) and isinstance(target.value, str):
        name = target.value
        reviewed = name.split(".", 1)[0] in PRODUCT or name in EXTERNAL_IMPORTS
        return name if _banned(name) or not reviewed else None
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
    typed = _annotation_node_ids(tree)
    parents = {id(child): node for node in ast.walk(tree) for child in ast.iter_child_nodes(node)}
    for node in outermost_references(tree):
        if id(node) in annotation_only:
            continue
        resolved = symbols.resolve(node, bound_only=True)
        resolved |= reexported(resolved)
        held_module = _held_module(node, resolved, parents)
        if held_module is not None:
            findings.append(Finding(path, node.lineno, f"module {held_module} held as a value: name its members"))
        if resolved & CLOCK_CLASSES and id(node) not in typed and not _uses_a_class(node, parents):
            findings.append(Finding(path, node.lineno, f"{min(resolved & CLOCK_CLASSES)} held as a value: read time through rt.now"))
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


def _held_module(node: ast.expr, resolved: set[str], parents: dict[int, ast.AST]) -> str | None:
    """An external module held as a value: it would carry its clocks and
    entropy wherever it flows (``providers = {"wall": datetime}``). Reading
    a constant member (``getattr(subprocess, "CREATE_NO_WINDOW", 0)``) is
    not holding it; that member is checked against the bans instead."""
    modules = {
        symbol for symbol in resolved
        if symbol.split(".", 1)[0] not in FIRST_PARTY and isinstance(sys.modules.get(symbol), types.ModuleType)
    }
    if not modules:
        return None
    parent = parents.get(id(node))
    if (
        isinstance(parent, ast.Call) and isinstance(parent.func, ast.Name)
        and parent.func.id in ("getattr", "hasattr") and parent.args and parent.args[0] is node
    ):  # a computed name is reflection, outside this net
        name = parent.args[1] if len(parent.args) > 1 else None
        if not (isinstance(name, ast.Constant) and isinstance(name.value, str)):
            return None
        member = {f"{module}.{name.value}" for module in modules}
        return min(member) if any(map(_banned, member)) else None
    return min(modules)


def _annotation_node_ids(tree: ast.AST) -> set[int]:
    found = set()
    for node in ast.walk(tree):
        for annotation in (getattr(node, "annotation", None), getattr(node, "returns", None)):
            if annotation is not None:
                found |= {id(part) for part in ast.walk(annotation)}
    return found


_TYPE_TESTS = frozenset({"isinstance", "issubclass"})


def _uses_a_class(node: ast.expr, parents: dict[int, ast.AST]) -> bool:
    """A class read in place: called (``datetime(2020, 1, 1)``), subclassed,
    or tested (``isinstance(x, (date, datetime))``), never held as a value."""
    parent = parents.get(id(node))
    if isinstance(parent, ast.Call) and parent.func is node:
        return True
    if isinstance(parent, ast.ClassDef) and node in parent.bases:
        return True
    if isinstance(parent, ast.Tuple):  # isinstance(x, (date, datetime))
        node, parent = parent, parents.get(id(parent))
    return (
        isinstance(parent, ast.Call) and isinstance(parent.func, ast.Name)
        and parent.func.id in _TYPE_TESTS and node in parent.args[1:]
    )


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
        resolved = symbols.resolve(node, bound_only=True)
        if _stands_for(resolved | reexported(resolved), PORT):
            findings.append(Finding(path, node.lineno, "runtime_time in the operator: take StackContext.clock"))
    return findings


def system_clock_findings(path: str, source: str) -> list[Finding]:
    """Any reference to SystemClock (the class, an instance, a method), under
    any name, outside the composition root."""
    if path in SYSTEM_CLOCK_ROOTS:
        return []
    tree = ast.parse(source)
    symbols = _Symbols(tree, path)
    return [
        Finding(path, node.lineno, "SystemClock: take StackContext.clock")
        for node in ast.walk(tree)
        if isinstance(node, (ast.Name, ast.Attribute)) and isinstance(node.ctx, ast.Load)
        and any(  # the class, an instance or a method of it, under any name
            "SystemClock" in symbol.split(".")
            for symbol in symbols.resolve(node) | reexported(symbols.resolve(node))
        )
    ]


# -- identity-hashed sets -----------------------------------------------------
#
# The rule is on declarations, not on iteration: covered code must not
# declare a set (in any annotation: attributes, fields, parameters, returns,
# locals, aliases, nested in containers or typing wrappers) whose element
# type hashes by object identity. Without such a set there is nothing whose
# iteration order could follow memory addresses, whatever path it would take.
# The check fails closed: every annotation (and ``# type:`` comment) must
# resolve from its module and enclosing classes, so an alias the lint does
# not collect cannot hide a set behind a name it cannot read.
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


class _Unions(ast.NodeTransformer):
    def visit_BinOp(self, node: ast.BinOp) -> ast.expr:
        self.generic_visit(node)
        if not isinstance(node.op, ast.BitOr):
            return node
        union = ast.Name(UNION_NAME, ast.Load())
        return ast.Subscript(union, ast.Tuple([node.left, node.right], ast.Load()), ast.Load())


UNION_NAME = "__lint_union__"


def evaluate(text: str, namespace: dict) -> object:
    """Evaluate a type expression, reading ``X | Y`` as ``Union[X, Y]``
    (Python 3.9 cannot evaluate ``|`` between types)."""
    tree = ast.fix_missing_locations(_Unions().visit(ast.parse(text.strip(), mode="eval")))
    return eval(compile(tree, "<annotation>", "eval"), {**namespace, UNION_NAME: typing.Union})  # noqa: S307


def _standard_library(top: str) -> bool:
    """Whether a top-level module is the standard library (Python 3.9 has no
    ``sys.stdlib_module_names``: read where the module lives instead)."""
    names = getattr(sys, "stdlib_module_names", None)
    if names is not None:
        return top in names or top == "builtins"
    module = sys.modules.get(top)
    source = getattr(module, "__file__", None)
    if module is None or source is None:
        return module is not None  # a built-in module
    path = Path(source).resolve()
    return path.is_relative_to(Path(sysconfig.get_paths()["stdlib"]).resolve()) and "site-packages" not in path.parts


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
        names = _definers(text, namespace)
        if names is None:
            return None, texts
        try:
            kind = evaluate(text, names)
        except Exception:
            return None, texts
    return kind, texts


def _definers(text: str, namespace: dict) -> dict | None:
    """What each name in a forward reference stands for, wherever the
    reference came from: the one object the first-party modules and
    ``namespace`` bind it to (``JsonValue = Union[..., list["JsonValue"]]``
    resolves where it is defined, wherever it is imported). A name bound to
    different objects in different modules is ambiguous: None, unresolved.
    A name nobody binds is left to the builtins."""
    try:
        names = {node.id for node in ast.walk(ast.parse(text.strip(), mode="eval")) if isinstance(node, ast.Name)}
    except SyntaxError:
        return None
    homes = [namespace] + [
        vars(module) for name, module in list(sys.modules.items())
        if name.split(".", 1)[0] in PRODUCT and hasattr(module, "__dict__")
    ]
    found = {}
    for name in names:
        objects = {id(home[name]): home[name] for home in homes if name in home}
        if len(objects) > 1:
            return None
        found.update({name: value for value in objects.values()})
    return found


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
        if origin in (*UNIONS, tuple, frozenset):  # contents decide; none given: unknown
            return not args or any(id_hashed(arg, namespace, seen) for arg in args)
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
        or issubclass(kind, (weakref.ref, *RECEIVER_HASHED))
    ):  # a bare container, a weak reference, a bound method: what it holds decides
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
    owner = next((base for base in kind.__mro__ if "__hash__" in base.__dict__), kind)
    home = owner.__module__.split(".", 1)[0]
    # the standard library's value types are trusted; a third party's hash is
    # unknown (torch.Tensor hashes by id) unless reviewed
    return not (_standard_library(home) or owner in REVIEWED_VALUE_HASHES)


def _fields_id_hashed(kind: type, namespace: dict, seen: frozenset[int]) -> bool:
    """The fields a generated hash covers: dataclass fields with ``hash``
    (default: ``compare``) set, or every namedtuple field. A field with no
    type (an untyped namedtuple) is unknown."""
    try:
        hints = typing.get_type_hints(kind)
    except Exception:  # resolve the hashed fields one by one
        hints = {}
        for owner in reversed(kind.__mro__):  # inherited fields first, each in its module
            module = vars(sys.modules[owner.__module__]) if owner.__module__ in sys.modules else {}
            for name, annotation in owner.__dict__.get("__annotations__", {}).items():
                resolved = _forward(annotation, module or namespace)
                if resolved is None:
                    hints.pop(name, None)
                else:
                    hints[name] = resolved
    owner = next((base for base in kind.__mro__ if "__hash__" in base.__dict__), kind)
    if dataclasses.is_dataclass(owner):  # the class whose generated hash this is
        names = [
            field.name for field in dataclasses.fields(owner)
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
    every forward reference that does not resolve."""
    namespace = namespace or {}
    if isinstance(kind, (str, typing.ForwardRef)):
        resolved, texts = _forward_chain(kind, namespace)
        if resolved is None:  # fail closed: what it names is unknown
            return [f"unresolved {texts[-1]!r}"]
        kind = resolved
    if isinstance(kind, tuple):  # ``Clients, Names = set[Client], set[str]``
        return [found for item in kind for found in identity_sets(item, namespace, seen)]
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


def _set_words(tree: ast.Module) -> set[str]:
    """Set type names, plus the names a module imports them under."""
    return set(SET_WORDS) | {
        alias.asname
        for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
        for alias in node.names if alias.name in SET_WORDS and alias.asname
    }


def _scoped_nodes(tree: ast.Module) -> list[tuple[tuple[str, ...], ast.AST]]:
    """Every node, with the classes enclosing it."""
    found = []
    pending = [((), tree)]
    while pending:
        scope, node = pending.pop()
        for child in ast.iter_child_nodes(node):
            found.append((scope, child))
            pending.append((scope + (child.name,) if isinstance(child, ast.ClassDef) else scope, child))
    return found


def _type_expression(node: ast.expr) -> bool:
    """Whether a value is spelled like a type: names, subscripts, unions,
    tuples of them, a quoted subscript."""
    if isinstance(node, (ast.Name, ast.Attribute)):
        return True
    if isinstance(node, ast.Subscript):
        return _type_expression(node.value)
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.BitOr):
        return _type_expression(node.left) and _type_expression(node.right)
    if isinstance(node, ast.Tuple):
        return all(map(_type_expression, node.elts))
    if isinstance(node, ast.Constant):
        return node.value is None or (isinstance(node.value, str) and "[" in node.value)
    return False


TYPE_WRAPPERS = frozenset(
    f"{module}.{name}" for module in ("typing", "typing_extensions") for name in ("NewType", "TypeAliasType")
)


def _alias_parts(value: ast.expr, symbols: _Symbols) -> list[ast.expr]:
    """The parts of an assigned value that may each be a type: the elements
    of a tuple or list, every argument of a type wrapper under any name
    (``NewType("X", set)``), and the arguments of another call other than
    bare names (``field(default_factory=set)`` builds a set, it declares no
    type)."""
    if isinstance(value, (ast.Tuple, ast.List)):
        return [part for element in value.elts for part in _alias_parts(element, symbols)]
    if isinstance(value, ast.Call):
        arguments = [*value.args, *(keyword.value for keyword in value.keywords)]
        wrapper = symbols.resolve(value.func)
        if not (wrapper | reexported(wrapper)) & TYPE_WRAPPERS:
            arguments = [argument for argument in arguments if not isinstance(argument, (ast.Name, ast.Attribute))]
        return [part for argument in arguments for part in _alias_parts(argument, symbols)]
    return [value]


_TYPE_STATEMENT = getattr(ast, "TypeAlias", ())  # ``type X = ...`` (Python 3.12+)


def _set_aliases(tree: ast.Module, path: str | None = None) -> list[tuple[tuple[str, ...], int, ast.expr]]:
    """Values assigned anywhere (module, class, function) that are types
    mentioning a set: each is checked where it is declared."""
    words = _set_words(tree)
    symbols = _Symbols(tree, path)
    return [
        (scope, node.lineno, part) for scope, node in _scoped_nodes(tree)
        if isinstance(node, (ast.Assign, ast.AnnAssign, _TYPE_STATEMENT)) and node.value is not None
        for part in _alias_parts(node.value, symbols)
        if _type_expression(part) and _spelled_words(part) & words
    ]


def _spelled_words(node: ast.expr) -> set[str]:
    """The names a value spells, and the words of quoted types in it
    (``row['set']`` spells ``row``; ``Optional['Bag[X]']`` spells ``Bag``)."""
    words = set()
    for part in ast.walk(node):
        if isinstance(part, ast.Name):
            words.add(part.id)
        elif isinstance(part, ast.Attribute):
            words.add(part.attr)
        elif isinstance(part, ast.Constant) and isinstance(part.value, str) and (
            part is node or "[" in part.value
        ):
            words |= _words(part.value)
    return words



def _words(text: str) -> set[str]:
    for mark in "[](),|.'\"":
        text = text.replace(mark, " ")
    return set(text.split())


def _annotations(tree: ast.Module, path: str | None = None) -> list[tuple[int, ast.expr, tuple[str, ...]]]:
    """Every annotation in the module (assignments, parameters, returns), and
    ``# type:`` comments, and every alias value that names a set (``Clients
    = set[X]``, ``Clients: TypeAlias = ...``), with the classes enclosing it.
    Annotations are evaluated without a function's locals: one inside a
    function must resolve from the module and its classes."""
    found = [(line, part, scope) for scope, line, part in _set_aliases(tree, path)]
    for scope, child in _scoped_nodes(tree):
        if isinstance(child, ast.AnnAssign):
            found.append((child.lineno, child.annotation, scope))
        elif isinstance(child, ast.arg) and child.annotation is not None:
            found.append((child.lineno, child.annotation, scope))
        elif isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)) and child.returns is not None:
            found.append((child.lineno, child.returns, scope))
        comment = getattr(child, "type_comment", None)
        if comment and isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
            signature = ast.parse(comment, mode="func_type")  # ``(int, str) -> set[X]``
            found += [(child.lineno, part, scope) for part in [*signature.argtypes, signature.returns]]
        elif comment:
            found.append((child.lineno, ast.Constant(comment), scope))
    return sorted(found, key=lambda item: item[0])


def _scope_namespace(namespace: dict, scope: tuple[str, ...]) -> dict:
    """The module namespace with the names of each enclosing class added."""
    merged, owner = dict(namespace), None
    for name in scope:
        owner = namespace.get(name) if owner is None else getattr(owner, name, None)
        if not isinstance(owner, type):
            break  # a class built inside a function: only the module is known
        merged.update(vars(owner))
    return merged


def _annotation_text(annotation: ast.expr) -> str:
    if isinstance(annotation, ast.Constant) and isinstance(annotation.value, str):
        return annotation.value  # a quoted annotation
    return ast.unparse(annotation)


def identity_set_findings(path: str, source: str, module: str,
                          namespace: dict | None = None) -> list[Finding]:
    """Declared sets whose elements hash by identity. Every annotation is
    evaluated in the module's namespace (``namespace`` overrides importing
    ``module``); one that names a set and does not resolve is a finding."""
    tree = ast.parse(source, type_comments=True)
    annotations = _annotations(tree, path)
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
    findings = []
    for line, annotation, scope in annotations:
        text = _annotation_text(annotation)
        scoped = _scope_namespace(namespace, scope)
        try:
            kind = evaluate(text, scoped)
        except Exception:  # fail closed: what it names is unknown
            findings.append(Finding(path, line, f"cannot resolve {text!r}: annotations must resolve from the module"))
            continue
        for found in identity_sets(kind, scoped):
            findings.append(Finding(path, line, f"{found} orders by object identity: keep a dict"))
    return findings
