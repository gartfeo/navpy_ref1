"""The RuntimeTime lint over the covered packages, and its negative controls."""

from __future__ import annotations

import ast
import dataclasses
import datetime as _datetime
import enum
import sys
import time as _time
import types
from typing import Generic, Optional, Protocol, TypeVar

import pytest

from navpy.core.kernel import runtime_time as rt
from tests.guards.runtime_time_lint import (
    ROOT,
    covered_files,
    external_imports,
    EXTERNAL_IMPORTS,
    evaluate,
    host_use_findings,
    id_hashed,
    identity_set_findings,
    module_name,
    operator_files,
    operator_findings,
    system_clock_findings,
)

if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))


def _read(path) -> str:
    return path.read_text(encoding="utf-8")


def _lines(findings) -> list[int]:
    return sorted(finding.line for finding in findings)


def _reads_and_held(findings) -> tuple[list[int], list[int]]:
    """Lines of clock reads and of datetime classes held as values."""
    held = [finding for finding in findings if "held as a value" in finding.rule]
    return _lines(f for f in findings if f not in held), _lines(held)


def test_covered_packages_use_only_the_runtime_port() -> None:
    findings = [
        str(finding)
        for relative, path in covered_files()
        for finding in host_use_findings(relative, _read(path))
    ]
    assert findings == []


def test_operator_takes_its_clock_from_the_stack_context() -> None:
    findings = []
    for relative, path in operator_files():
        source = _read(path)
        findings += system_clock_findings(relative, source)
        findings += operator_findings(relative, source)
    assert [str(finding) for finding in findings] == []


def test_covered_code_declares_no_identity_hashed_set() -> None:
    sources = {relative: _read(path) for relative, path in covered_files()}
    findings = [
        str(finding)
        for relative, source in sources.items()
        for finding in identity_set_findings(relative, source, module_name(relative))
    ]
    assert findings == []


# -- negative controls: host clocks, threads and entropy ------------------------

def test_an_unseeded_random_call_is_caught() -> None:
    source = "import random\n\ndef jitter():\n    return random.random()\n"
    assert [f.rule for f in host_use_findings("x.py", source)] == [
        "import random: use runtime_time", "random.random: use runtime_time",
    ]


def test_direct_clocks_threads_and_wall_datetimes_are_caught() -> None:
    source = (
        "from time import monotonic\n"
        "import threading as t\n"
        "from concurrent.futures import ThreadPoolExecutor\n"
        "from datetime import datetime\n"
        "def f():\n"
        "    import uuid\n"
        "    return datetime.now()\n"
    )
    assert _lines(host_use_findings("x.py", source)) == [1, 2, 3, 6, 7]


def test_other_host_sources_star_imports_and_port_internals_are_caught() -> None:
    source = (
        "import os\n"
        "import timeit\n"
        "import numpy as np\n"
        "from navpy.core.kernel import runtime_time as rt\n"
        "from x import *\n"
        "a = os.urandom(8)\n"
        "b = np.random.default_rng()\n"
        "c = rt.WallClock()\n"
        "d = rt.wall_runtime().threads\n"
        "e = rt.monotonic()\n"
    )
    assert _lines(host_use_findings("x.py", source)) == [2, 5, 6, 7, 8, 9]


def test_dynamic_imports_and_aliased_datetimes_are_caught() -> None:
    source = (
        "import datetime\n"
        "import importlib\n"
        "from datetime import date as day, datetime as dt\n"
        "a = __import__('time').time()\n"
        "b = importlib.import_module('random')\n"
        "c = datetime.datetime.now()\n"
        "d = dt.utcnow()\n"
        "e = day.today()\n"
        "f = importlib.import_module('gcs.backend.routes.x')\n"
    )
    # line 4: the dynamic import, and the time.time it reads
    assert _lines(host_use_findings("x.py", source)) == [4, 4, 5, 6, 7, 8]


def test_assigned_aliases_and_runtime_named_imports_are_caught() -> None:
    source = (
        "import importlib\n"
        "from datetime import datetime\n"
        "from importlib import import_module as load\n"
        "D = datetime\n"
        "a = D.now()\n"
        "now = datetime.now\n"
        "b = load('time')\n"
        "c = importlib.import_module(name='secrets')\n"
        "def f(name):\n"
        "    return importlib.import_module(name)\n"
        "g = importlib.import_module(f'navpy.plugins.{1}')\n"
    )
    assert _reads_and_held(host_use_findings("x.py", source)) == ([5, 6, 7, 8, 10], [4])


def test_qualified_attribute_and_tuple_aliases_are_caught() -> None:
    source = (
        "import datetime as dt\n"
        "import importlib\n"
        "import concurrent\n"
        "from concurrent import futures\n"
        "D = dt.datetime\n"
        "a = D.now()\n"
        "load = importlib.import_module\n"
        "b = load(name='random')\n"
        "class Stamp:\n"
        "    def __init__(self):\n"
        "        self.kind = dt.datetime\n"
        "    def now(self):\n"
        "        return self.kind.now()\n"
        "first, second = dt.datetime, dt.date\n"
        "c = second.today()\n"
        "pool = concurrent.futures.ThreadPoolExecutor()\n"
        "def stamp(time):\n"
        "    return time + 1\n"
    )
    assert _reads_and_held(host_use_findings("x.py", source)) == ([4, 6, 8, 13, 15, 16], [5, 11, 14, 14])


def test_only_the_type_checking_branch_is_annotation_only() -> None:
    allowed = (
        "import typing\n"
        "from typing import TYPE_CHECKING\n"
        "if TYPE_CHECKING:\n"
        "    import threading\n"
        "if typing.TYPE_CHECKING:\n"
        "    import queue\n"
    )
    assert host_use_findings("x.py", allowed) == []
    runtime = (
        "from typing import TYPE_CHECKING\n"
        "if not TYPE_CHECKING:\n"
        "    import threading\n"
        "if TYPE_CHECKING:\n"
        "    pass\n"
        "else:\n"
        "    import time\n"
    )
    assert _lines(host_use_findings("x.py", runtime)) == [3, 7]


# -- negative controls: the operator's clock ------------------------------------

def test_system_clock_is_referenced_only_by_the_composition_root() -> None:
    source = (
        "from scripts.eval_gcs_demo_execution import SystemClock as Clock\n"
        "from scripts import eval_gcs_demo_execution as execution\n"
        "from .eval_gcs_demo_execution import SystemClock as Relative\n"
        "C = Clock\n"
        "a = Clock()\n"
        "b = execution.SystemClock()\n"
        "c = C()\n"
        "factory = execution.SystemClock\n"
        "d = Relative()\n"
    )
    assert _lines(system_clock_findings("scripts/eval_gcs_demo_mission.py", source)) == [4, 5, 6, 7, 8, 9]
    assert system_clock_findings("scripts/eval_gcs_demo_runtime.py", source) == []


def test_operator_host_modules_stay_inside_their_one_owner() -> None:
    source = (
        "import time\n"
        "class SystemClock:\n"
        "    def monotonic(self):\n"
        "        return time.monotonic()\n"
        "def wait_until(clock):\n"
        "    time.sleep(1)\n"
    )
    findings = operator_findings("scripts/eval_gcs_demo_execution.py", source)
    assert [(f.line, f.rule) for f in findings] == [(6, "time.sleep: use runtime_time")]
    aliased = "import time as wall\nclass SystemClock:\n    pass\nwall.sleep(1)\n"
    assert _lines(operator_findings("scripts/eval_gcs_demo_execution.py", aliased)) == [1, 4]
    other = operator_findings("scripts/eval_gcs_demo_process.py", "import threading\n")
    assert [f.rule for f in other] == ["import threading: use runtime_time"]


def test_the_operator_never_uses_the_runtime_port() -> None:
    source = (
        "from navpy.core.kernel import runtime_time as rt\n"
        "def wait(deadline):\n"
        "    return rt.monotonic() < deadline\n"
    )
    assert _lines(operator_findings("scripts/eval_gcs_demo_mission.py", source)) == [1, 3]


# -- negative controls: identity-hashed sets ------------------------------------

class _Plain:
    pass


_PlainSet = set[_Plain]  # imported for annotations only in a negative control


T = TypeVar("T")


class _Generic(Generic[T]):
    pass


@dataclasses.dataclass(frozen=True)
class _ValueFieldsOwnHash:
    index: int

    def __hash__(self) -> int:  # hand-written: may hash an identity
        return id(self)


class _Kind(enum.Enum):
    A = 1

    def __hash__(self) -> int:
        return id(self)


class _OwnHash:
    def __init__(self, event) -> None:
        self.event = event

    def __hash__(self) -> int:
        return hash(self.event)


@dataclasses.dataclass(frozen=True)
class _ByValue:
    poi: Optional[int]
    name: str


@dataclasses.dataclass(frozen=True)
class _HoldsEvent:
    poi: int
    cancel: rt.Event  # a runtime factory: identity-hashed


@dataclasses.dataclass(eq=False)
class _Slot:
    index: int


def test_identity_hashing_is_resolved_through_dataclass_fields() -> None:
    assert not id_hashed(_ByValue)
    assert id_hashed(_HoldsEvent)
    assert not id_hashed(tuple[int, int])
    assert not id_hashed(str)
    assert id_hashed(_Plain)
    assert id_hashed(_Slot)
    assert id_hashed(_OwnHash)  # a hand-written __hash__ may hash an identity
    assert id_hashed(rt.Event)
    assert id_hashed(None)  # unresolved: assume identity


def test_identity_hashed_sets_are_caught_in_every_declaration_form() -> None:
    source = (
        "from typing import Annotated, ClassVar, Final, Optional, Set as ClientSet\n"
        "Clients = set[_Plain]\n"
        "class Hub:\n"
        "    shared: ClassVar[set[_Plain]] = set()\n"
        "    fixed: Final[frozenset[_Slot]]\n"
        "    def __init__(self):\n"
        "        self._clients: set[_Plain] = set()\n"
        "    def send(self, clients: Clients, more: 'set[_Plain]') -> set[_OwnHash]:\n"
        "        grouped: dict[int, set[_HoldsEvent]] = {}\n"
        "        maybe: Optional[ClientSet[_Plain]] = None\n"
        "        tagged: Annotated[set[_Plain], 'x'] = set()\n"
        "        ids: set[int] = set()\n"
        "        values: set[_ByValue] = set()\n"
        "        ghost: set[Missing] = set()\n"
        "        return set()\n"
    )
    namespace = {name: globals()[name] for name in ("_Plain", "_Slot", "_OwnHash", "_HoldsEvent", "_ByValue")}
    exec(compile(source, "x.py", "exec"), namespace)  # the source is its own module
    findings = identity_set_findings("x.py", source, "x", namespace)
    assert _lines(findings) == [2, 4, 5, 7, 8, 8, 8, 9, 10, 11, 14]  # 2: the alias itself
    assert [f.line for f in findings if "cannot resolve" in f.rule] == [14]


def test_relative_imports_resolve_against_the_package() -> None:
    source = (
        "from ..kernel import runtime_time as port\n"
        "from .. import kernel\n"
        "a = port.wall_runtime()\n"
        "b = kernel.runtime_time.WallClock()\n"
    )
    assert _lines(host_use_findings("src/navpy/core/approach/x.py", source)) == [3, 4]


def test_generic_custom_hash_bare_and_nested_forward_sets_are_caught() -> None:
    source = (
        "from typing import Optional\n"
        "Clients = set[_Plain]\n"
        "Group = Clients\n"
        "def f(a: Group, b: set[_Generic[int]], c: set[_ValueFieldsOwnHash],\n"
        "      d: set[_Kind], e: set, g: Optional['set[_Plain]'], h: list['set[Missing]'],\n"
        "      i: frozenset[_ByValue]):\n"
        "    pass\n"
    )
    namespace = {name: globals()[name] for name in (
        "_Plain", "_Generic", "_ValueFieldsOwnHash", "_Kind", "_ByValue")}
    exec(compile(source, "x.py", "exec"), namespace)
    findings = identity_set_findings("x.py", source, "x", namespace)
    rules = sorted(f.rule.split(" orders")[0] for f in findings)
    assert len(findings) == 8, rules  # with the Clients alias declaration
    assert any("unresolved 'set[Missing]'" in rule for rule in rules)


def test_imported_set_aliases_resolve_through_the_module() -> None:
    model = {"Clients": set[_Plain]}
    source = "def f(clients: Connections):\n    pass\n"
    namespace = {"Connections": model["Clients"]}
    assert _lines(identity_set_findings("x.py", source, "x", namespace)) == [1]


def test_the_operator_never_imports_dynamically() -> None:
    source = (
        "from importlib import import_module\n"
        "rt = import_module('navpy.core.kernel.runtime_time')\n"
        "rt.sleep(1)\n"
    )
    findings = operator_findings("scripts/eval_gcs_demo_mission.py", source)
    assert "dynamic import in the operator: import statically" in [f.rule for f in findings]


def test_dynamic_imports_of_parent_modules_bind_their_names() -> None:
    source = (
        "import importlib\n"
        "dt = importlib.import_module('datetime')\n"
        "a = dt.datetime.now()\n"
        "np = importlib.import_module('numpy')\n"
        "b = np.random.default_rng()\n"
        "c = __import__('concurrent.futures').futures.ThreadPoolExecutor()\n"
    )
    assert _lines(host_use_findings("x.py", source)) == [3, 5, 6, 6]  # 6: import and use


def test_callables_annotation_only_aliases_and_forward_chains_are_resolved() -> None:
    source = (
        "from typing import TYPE_CHECKING, Callable, Hashable, Literal, Optional\n"
        "if TYPE_CHECKING:\n"
        "    from tests.guards.test_runtime_time_lint import _PlainSet as Renamed\n"
        "Members = 'set[_Plain]'\n"
        "def f(a: Callable[[set[_Plain]], None], b: Optional['Renamed'],\n"
        "      c: list['Members'], d: set[Hashable],\n"
        "      e: Literal['set', 'get'], g: set[tuple['int', 'str']]):\n"
        "    pass\n"
    )
    namespace = {"_Plain": _Plain, "__name__": "x", "__package__": None}
    exec(compile(source, "x.py", "exec"), namespace)
    findings = identity_set_findings("x.py", source, "x", namespace)
    assert len(findings) == 5, [f.rule for f in findings]  # Members, a, b, c, d; never e or g


class _Equality(Protocol):
    def __eq__(self, other: object) -> bool: ...


def test_aliases_newtypes_bare_keys_protocols_and_literal_members_are_caught() -> None:
    source = (
        "import collections\n"
        "from typing_extensions import Literal, NewType, TypeAlias\n"
        "Declared: TypeAlias = set[_Plain]\n"
        "Wrapped = NewType('Wrapped', set[_Plain])\n"
        "Key = collections.namedtuple('Key', 'client')\n"
        "Members = 'set[Missing]'\n"
        "def f(a: Wrapped, b: set[tuple], c: set[Key], d: set[_Equality],\n"
        "      e: set[Literal[_Kind.A]], g: list['Members'], h: set[Literal['x', 1]]):\n"
        "    pass\n"
    )
    namespace = {"_Plain": _Plain, "_Equality": _Equality, "_Kind": _Kind}
    exec(compile(source, "x.py", "exec"), namespace)
    findings = identity_set_findings("x.py", source, "x", namespace)
    # 3: TypeAlias; 4: the NewType; 6: the unresolvable alias; 7: a-d; 8: e, g
    assert _lines(findings) == [3, 4, 6, 7, 7, 7, 7, 8, 8], [f.rule for f in findings]


def test_parameter_defaults_alias_like_assignments() -> None:
    source = (
        "from datetime import datetime\n"
        "def stamp(clock=datetime, *, other=datetime):\n"
        "    return clock.now(), other.today()\n"
    )
    assert _reads_and_held(host_use_findings("x.py", source)) == ([3, 3], [2, 2])


def test_dataclass_hashes_cover_only_their_hashed_fields() -> None:
    @dataclasses.dataclass(frozen=True)
    class Generation:
        value: int
        stop: rt.Event = dataclasses.field(compare=False)

    assert not id_hashed(Generation)


def test_renamed_sets_guarded_aliases_tuple_keys_and_weakrefs_are_caught() -> None:
    source = (
        "import weakref\n"
        "from typing import TYPE_CHECKING, Set as Bag; from typing_extensions import TypeAlias\n"
        "class Key(tuple):\n"
        "    pass\n"
        "Declared: TypeAlias = Bag[_Plain]\n"
        "if TYPE_CHECKING:\n"
        "    Clients: TypeAlias = set[_Plain]\n"
        "def f(a: 'Clients', b: 'Bag[Missing]', c: set[Key], d: set[weakref.ReferenceType],\n"
        "      e: set[frozenset[str]]):\n"
        "    pass\n"
    )
    namespace = {"_Plain": _Plain}
    exec(compile(source, "x.py", "exec"), namespace)
    findings = identity_set_findings("x.py", source, "x", namespace)
    # 5: the renamed-set alias; 7: the guarded alias; 8: a, b, c, d; never e
    assert _lines(findings) == [5, 7, 8, 8, 8, 8], [f.rule for f in findings]


def test_class_attribute_aliases_resolve_through_self() -> None:
    source = (
        "from datetime import datetime\n"
        "class Stamp:\n"
        "    kind = datetime\n"
        "    def now(self):\n"
        "        return self.kind.now()\n"
    )
    assert _reads_and_held(host_use_findings("x.py", source)) == ([5], [3])


def test_class_qualified_aliases_and_clock_subclasses_are_caught() -> None:
    source = (
        "from datetime import datetime\n"
        "class Stamp:\n"
        "    kind = datetime\n"
        "class Timestamp(datetime):\n"
        "    pass\n"
        "a = Stamp.kind.now()\n"
        "b = Timestamp.now()\n"
    )
    assert _reads_and_held(host_use_findings("x.py", source)) == ([6, 7], [3])


def test_bound_methods_bare_tuples_renamed_forwards_and_class_aliases_are_caught() -> None:
    source = (
        "import types\n"
        "from typing import Optional, Set as Bag, Tuple\n"
        "class Hub:\n"
        "    Members = set[_Plain]\n"
        "    Names = frozenset[str]\n"
        "    def send(self, members: Members, names: Names, a: set[types.MethodType],\n"
        "             b: set[Tuple], c: Optional['Bag[Missing]'], d: set[Tuple[int, str]]):\n"
        "        pass\n"
    )
    namespace = {"_Plain": _Plain}
    exec(compile(source, "x.py", "exec"), namespace)
    findings = identity_set_findings("x.py", source, "x", namespace)
    # 4: the class alias; 6: members, a; 7: b, c; never names or d
    assert _lines(findings) == [4, 6, 6, 7, 7], [f.rule for f in findings]


@dataclasses.dataclass(frozen=True)
class _Base:
    value: int


@dataclasses.dataclass(frozen=True)
class _Unresolved(_Base):
    stop: Missing = dataclasses.field(default=None, compare=False)  # noqa: F821


def test_dataclass_field_fallback_reads_inherited_fields() -> None:
    assert not id_hashed(_Unresolved)


def test_clock_subclasses_read_through_cls_and_self() -> None:
    source = (
        "from datetime import datetime\n"
        "class Stamp(datetime):\n"
        "    @classmethod\n"
        "    def current(cls):\n"
        "        return cls.now()\n"
        "    def again(self):\n"
        "        return self.today()\n"
    )
    assert _lines(host_use_findings("x.py", source)) == [5, 7]


def test_set_aliases_in_class_blocks_functions_and_tuples_are_caught() -> None:
    source = (
        "class Hub:\n"
        "    if True:\n"
        "        Members = set[_Plain]\n"
        "def run():\n"
        "    Local = set[_Plain]\n"
        "    clients: Local = set()\n"
        "    Pair, Names = set[_Plain], set[str]\n"
        "    names: 'set[int] | None' = None\n"
        "    row = {'set': 1}\n"
        "    kind = row['set']\n"
    )
    namespace = {"_Plain": _Plain}
    exec(compile(source, "x.py", "exec"), namespace)
    findings = identity_set_findings("x.py", source, "x", namespace)
    # 3: the class alias; 5: the local alias; 6: its unresolvable use; 7: Pair;
    # never the value-hashed union or a subscript that only quotes "set"
    assert _lines(findings) == [3, 5, 6, 7], [f.rule for f in findings]


def test_pep604_unions_evaluate_as_typing_unions() -> None:
    assert evaluate("set[int] | None", {}) == Optional[set[int]]


@dataclasses.dataclass(frozen=True)
class _HashOwner:
    index: int


@dataclasses.dataclass(frozen=True, eq=False)
class _InheritsHash(_HashOwner):
    cancel: rt.Event = None


def test_an_inherited_generated_hash_covers_its_owner_fields() -> None:
    assert _InheritsHash.__hash__ is _HashOwner.__hash__
    assert not id_hashed(_InheritsHash)


def test_unpacked_looped_and_instance_clock_members_are_caught() -> None:
    source = (
        "from datetime import datetime\n"
        "class Stamp:\n"
        "    kind, other = datetime, None\n"
        "    for _ in range(1):\n"
        "        looped = datetime\n"
        "    def read(self):\n"
        "        return self.kind.now(), self.looped.today()\n"
        "made = datetime(2020, 1, 1)\n"
        "late = made.now()\n"
    )
    assert _reads_and_held(host_use_findings("x.py", source)) == ([7, 7, 9], [3, 5])


def test_wrapped_unpacked_commented_and_unresolvable_declarations_fail_closed() -> None:
    source = (
        "from typing import NewType as Nominal\n"
        "from typing_extensions import TypeAliasType\n"
        "Wrapped = Nominal('Wrapped', set[_Plain])\n"
        "Aliased = TypeAliasType('Aliased', set[_Plain])\n"
        "def run():\n"
        "    Members, retries = set[_Plain], 3\n"
        "    members: Members = set()\n"
        "    listed = [set[_Plain]]\n"
        "    legacy = set()  # type: set[_Plain]\n"
        "def f(a):\n"
        "    # type: (set[_Plain]) -> None\n"
        "    pass\n"
        "def g(b: Unknown):\n"
        "    pass\n"
    )
    namespace = {"_Plain": _Plain}
    exec(compile(source, "x.py", "exec"), namespace)
    findings = identity_set_findings("x.py", source, "x", namespace)
    # 3, 4: wrapped aliases; 6: the unpacked alias; 7: its unresolvable use;
    # 8: a listed type; 9: a type comment; 10: a signature comment;
    # 13: any annotation that does not resolve, set or not
    assert _lines(findings) == [3, 4, 6, 7, 8, 9, 10, 13], [f.rule for f in findings]


def test_keyword_wrappers_and_forward_references_resolve_at_their_definer(monkeypatch) -> None:
    provider = types.ModuleType("navpy._set_probe")
    exec(
        "class ProbeClient:\n"
        "    pass\n"
        "class ProbeUnique(str):\n"
        "    pass\n"
        "Key = tuple['ProbeClient']\n"
        "Safe = tuple['ProbeUnique | None']\n",
        provider.__dict__,
    )
    monkeypatch.setitem(sys.modules, provider.__name__, provider)
    source = (
        "from typing import NewType\n"
        "Clients = NewType(name='Clients', tp=set[_Plain])\n"
        "ProbeClient = int\n"
        "def f(a: set[Key], b: set[Safe]):\n"
        "    pass\n"
    )
    namespace = {"_Plain": _Plain, "Key": provider.Key, "Safe": provider.Safe}
    exec(compile(source, "x.py", "exec"), namespace)
    findings = identity_set_findings("x.py", source, "x", namespace)
    # 2: a keyword-built alias; 4 (twice: the set and its reference): Key's
    # "ProbeClient" is ambiguous, never the consumer's int; never Safe's union
    assert _lines(findings) == [2, 4, 4], [f.rule for f in findings]


def test_datetime_providers_and_first_party_reexports_are_caught(monkeypatch) -> None:
    helper = types.ModuleType("navpy._wall_probe")
    helper.dt, helper.wall, helper.tm = _datetime.datetime, rt.wall_runtime, _time
    monkeypatch.setitem(sys.modules, helper.__name__, helper)
    source = (
        "from datetime import datetime\n"
        "from navpy._wall_probe import dt, wall, tm\n"
        "def f(supplied=None):\n"
        "    reader = supplied or datetime\n"
        "    return reader.now()\n"
        "class API:\n"
        "    def __init__(self):\n"
        "        self.kind = datetime\n"
        "a = dt.now()\n"
        "b = wall()\n"
        "c = tm.monotonic()\n"
        "d = isinstance(a, (datetime, int))\n"
        "e = datetime(2020, 1, 1)\n"
    )
    # 5: the read through the selected provider; 9-11: re-exported wall reads,
    # wall runtime and time; 4, 8: datetime held as a value; never the
    # isinstance test or the constructor
    assert _reads_and_held(host_use_findings("x.py", source)) == ([5, 9, 10, 11], [4, 8])


def test_operator_rejects_a_reexported_port(monkeypatch) -> None:
    helper = types.ModuleType("navpy._port_probe")
    helper.clock = rt
    monkeypatch.setitem(sys.modules, helper.__name__, helper)
    source = "from navpy._port_probe import clock\nlate = clock.monotonic()\n"
    findings = operator_findings("scripts/eval_gcs_demo_probe.py", source)
    assert [f.line for f in findings if "runtime_time in the operator" in f.rule] == [2]


def test_bare_set_wrappers_are_declarations_and_set_factories_are_not() -> None:
    source = (
        "from typing import NewType as Nominal\n"
        "from dataclasses import field\n"
        "Clients = Nominal('Clients', set)\n"
        "Factory = field(default_factory=set)\n"
    )
    namespace: dict = {}
    exec(compile(source, "x.py", "exec"), namespace)
    assert _lines(identity_set_findings("x.py", source, "x", namespace)) == [3]


@pytest.mark.skipif(sys.version_info < (3, 12), reason="type statements are Python 3.12+")
def test_type_statements_are_declarations() -> None:
    source = "type Clients = set[_Plain]\n"
    namespace = {"_Plain": _Plain}
    exec(compile(source, "x.py", "exec"), namespace)
    assert _lines(identity_set_findings("x.py", source, "x", namespace)) == [1]


def test_inherited_and_super_datetime_reads_are_caught(monkeypatch) -> None:
    provider = types.ModuleType("navpy._timestamp_probe")
    exec("from datetime import datetime\nclass Timestamp(datetime):\n    pass\n", provider.__dict__)
    monkeypatch.setitem(sys.modules, provider.__name__, provider)
    imported = "from navpy._timestamp_probe import Timestamp\nstamp = Timestamp.now()\n"
    assert _reads_and_held(host_use_findings("x.py", imported)) == ([2], [])
    overridden = (
        "from datetime import datetime\n"
        "class Stamp(datetime):\n"
        "    @classmethod\n"
        "    def now(cls, tz=None):\n"
        "        return super().now(tz)\n"
    )
    assert _reads_and_held(host_use_findings("x.py", overridden)) == ([5], [])


def test_reexported_type_wrappers_declare_with_every_argument(monkeypatch) -> None:
    import typing as _typing

    helper = types.ModuleType("navpy._typing_probe")
    helper.NewType = _typing.NewType
    monkeypatch.setitem(sys.modules, helper.__name__, helper)
    source = "from navpy._typing_probe import NewType\nClients = NewType('Clients', set)\n"
    namespace: dict = {}
    exec(compile(source, "x.py", "exec"), namespace)
    assert _lines(identity_set_findings("x.py", source, "x", namespace)) == [2]


def test_imported_datetime_subclasses_are_clock_classes(monkeypatch) -> None:
    provider = types.ModuleType("navpy._subclass_probe")
    exec("from datetime import datetime\nclass Timestamp(datetime):\n    pass\n", provider.__dict__)
    monkeypatch.setitem(sys.modules, provider.__name__, provider)
    source = (
        "from navpy._subclass_probe import Timestamp\n"
        "def f(supplied=None):\n"
        "    reader = supplied or Timestamp\n"
        "    return reader.now()\n"
        "late = Timestamp(2020, 1, 1).today()\n"
    )
    # 3: the subclass held as a value; 4: the read through it; 5: a read
    # through its instance
    assert _reads_and_held(host_use_findings("x.py", source)) == ([4, 5], [3])


def test_relative_reexported_type_wrappers_declare(monkeypatch) -> None:
    import typing as _typing

    package = types.ModuleType("navpy._wrapper_pkg")
    package.__path__ = []
    helper = types.ModuleType("navpy._wrapper_pkg.typing_helpers")
    helper.NewType = _typing.NewType
    monkeypatch.setitem(sys.modules, package.__name__, package)
    monkeypatch.setitem(sys.modules, helper.__name__, helper)
    source = "from .typing_helpers import NewType\nClients = NewType('Clients', set)\n"
    namespace = {"NewType": _typing.NewType}
    findings = identity_set_findings("src/navpy/_wrapper_pkg/consumer.py", source, "x", namespace)
    assert _lines(findings) == [2]


def test_covered_code_imports_only_reviewed_external_modules() -> None:
    found = {
        name: relative
        for relative, path in covered_files()
        for name in external_imports(ast.parse(_read(path)))
    }
    assert {name: where for name, where in found.items() if name not in EXTERNAL_IMPORTS} == {}


def test_selected_providers_and_host_helpers_are_caught() -> None:
    source = (
        "import datetime\n"
        "import numpy as np\n"
        "from multiprocessing.pool import ThreadPool\n"
        "from email.utils import formatdate\n"
        "def stamp(provider=None, supplied=None, flag=False):\n"
        "    clock = provider or datetime\n"
        "    xp = np if flag else supplied\n"
        "    return clock.datetime.now(), xp.random.default_rng()\n"
        "pool = ThreadPool(2)\n"
        "date = formatdate(usegmt=True)\n"
    )
    # 3, 4: the imports; 8: both selected reads; 9, 10: their uses; 6, 7:
    # the datetime and numpy modules held as values
    assert _reads_and_held(host_use_findings("x.py", source)) == ([3, 4, 8, 8, 9, 10], [6, 7])


def test_operator_rejects_a_renamed_system_clock_reexport(monkeypatch) -> None:
    helper = types.ModuleType("scripts._clock_probe")

    class SystemClock:
        pass

    helper.Clock = SystemClock
    monkeypatch.setitem(sys.modules, helper.__name__, helper)
    source = "from scripts._clock_probe import Clock\nclock = Clock()\n"
    assert _lines(system_clock_findings("scripts/eval_gcs_demo_probe.py", source)) == [2]


def test_third_party_hashes_are_unknown_unless_reviewed() -> None:
    external = type("External", (int,), {"__hash__": int.__hash__, "__module__": "fakelib"})
    assert id_hashed(external)
    assert not id_hashed(int)


def test_reviewed_modules_host_members_dynamic_imports_and_held_modules() -> None:
    source = (
        "import cv2, importlib, datetime\n"
        "from logging.handlers import QueueListener\n"
        "ticks = cv2.getTickCount()\n"
        "scheduler = importlib.import_module('sched')\n"
        "providers = {'wall': datetime}\n"
        "flag = getattr(cv2, 'getTickFrequency')\n"
        "name = getattr(cv2, 'IMREAD_COLOR')\n"
    )
    # 2: QueueListener; 3: a host clock; 4: an unreviewed module; 5: a module
    # held as a value; 6: a banned member read by name; never a plain constant
    findings = host_use_findings("x.py", source)
    assert _lines(findings) == [2, 3, 4, 5, 6], [f.rule for f in findings]


def test_operator_rejects_system_clock_instances_and_methods(monkeypatch) -> None:
    helper = types.ModuleType("scripts._clock_instance_probe")

    class SystemClock:
        def monotonic(self) -> float:
            return 0.0

    helper.clock, helper.read = SystemClock(), SystemClock.monotonic
    monkeypatch.setitem(sys.modules, helper.__name__, helper)
    source = "from scripts._clock_instance_probe import clock, read\na = clock.monotonic()\nb = read(clock)\n"
    # every reference node: clock and clock.monotonic, read and clock
    assert _lines(system_clock_findings("scripts/eval_gcs_demo_probe.py", source)) == [2, 2, 3, 3]


def test_standard_library_detection_works_without_stdlib_module_names(monkeypatch) -> None:
    from tests.guards import runtime_time_lint

    monkeypatch.delattr(sys, "stdlib_module_names", raising=False)
    assert runtime_time_lint._standard_library("datetime")
    assert runtime_time_lint._standard_library("builtins")
    assert not runtime_time_lint._standard_library("numpy")
