"""The producer inventory is complete, and the fence stops each producer.

``lockstep_slice.md``, "Shutdown in drain": the backend keeps an ownership
inventory of everything that can make work or events. This compares it with
the code: every site that starts a task or a thread or schedules a coroutine
from another thread is either tracked through ``producers`` under a known
producer, or listed below with the producer that owns it; and the lockstep
fence names a stop for every producer. A new producer fails here until it is
added to the inventory and fenced.
"""

from __future__ import annotations

import ast
import asyncio
import threading
from pathlib import Path

import pytest

from gcs.backend.broadcast import ConnectionManager
from gcs.backend.lockstep.fence import FENCE, fence
from gcs.backend.producer_inventory import PRODUCERS, ProducerFenced, ProducerInventory

ROOT = Path(__file__).resolve().parents[3]
BACKEND = ROOT / "src" / "gcs" / "backend"
#: Calls that start work: ``asyncio.<name>`` and ``rt.<name>``.
_ASYNCIO_STARTS = {"create_task", "ensure_future", "run_coroutine_threadsafe"}
_RT_STARTS = {"Thread", "Timer"}
#: Calls that register a callback a transport runs later (a subscription).
_SUBSCRIBES = {"on_message", "on_admission", "subscribe", "register_rc_channel"}
#: Raw sites that do not go through ``producers``: (file, function) -> producer.
#: Both are host transports that lockstep replaces with ports, never runs.
RAW_SITES = {
    ("esp32_simulator.py", "Esp32RequestHandler._handle_launch"): "esp32_simulator",
    ("esp32_simulator.py", "Esp32Simulator.start"): "esp32_simulator",
    ("navpy_process_launch.py", "launch_navpy_instance"): "companion_processes",
}
#: A producer with no call site: requests come through the server.
NO_SITE = {"http_routes"}


class _Sites(ast.NodeVisitor):
    def __init__(self, relative: str) -> None:
        self.relative = relative
        self.scope: list[str] = []
        self.raw: list[tuple[str, str, str]] = []
        self.tracked: list[tuple[str, str]] = []
        self.subscribes: list[tuple[str, int]] = []
        self.kept: set[int] = set()

    def _named(self, node) -> None:
        self.scope.append(node.name)
        self.generic_visit(node)
        self.scope.pop()

    visit_FunctionDef = visit_AsyncFunctionDef = visit_ClassDef = _named

    def visit_Call(self, node: ast.Call) -> None:
        func = node.func
        if isinstance(func, ast.Attribute):
            owner = func.value.id if isinstance(func.value, ast.Name) else None
            where = _qualified(self.scope) if self.scope else "<module>"
            if func.attr in _SUBSCRIBES:
                self.subscribes.append((f"{self.relative}:{node.lineno}", id(node)))
            if owner == "producers":
                if func.attr == "keep_subscription" and len(node.args) > 1:
                    self.kept.add(id(node.args[1]))
                first = node.args[0] if node.args else None
                name = first.value if isinstance(first, ast.Constant) else None
                if name is None and isinstance(first, ast.Name):
                    name = _module_constant(self.relative, first.id)
                self.tracked.append((f"{self.relative}:{where}", name))
            elif (owner == "asyncio" and func.attr in _ASYNCIO_STARTS) or (
                    owner == "rt" and func.attr in _RT_STARTS) or (
                    func.attr == "create_task" and owner not in ("producers", "asyncio")):
                self.raw.append((self.relative, _qualified(self.scope), func.attr))
        self.generic_visit(node)


def _qualified(scope: list[str]) -> str:
    """``Class.method`` or the top-level function enclosing the call."""
    return ".".join(scope[:2]) if len(scope) > 1 and scope[0][:1].isupper() else scope[0]


def _module_constant(relative: str, name: str) -> str | None:
    tree = ast.parse((BACKEND / relative).read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == name for t in node.targets):
            return node.value.value if isinstance(node.value, ast.Constant) else None
    return None


def _scan() -> _Sites:
    total = _Sites("")
    for path in sorted(BACKEND.rglob("*.py")):
        relative = path.relative_to(BACKEND).as_posix()
        if relative.startswith("lockstep/") or relative == "producer_inventory.py":
            continue
        sites = _Sites(relative)
        sites.visit(ast.parse(path.read_text(encoding="utf-8")))
        total.raw += sites.raw
        total.tracked += sites.tracked
        total.subscribes += [where for where, node in sites.subscribes if node not in sites.kept]
    return total


def test_every_work_starting_site_is_in_the_inventory():
    sites = _scan()
    raw = {(relative, where) for relative, where, _call in sites.raw}
    assert raw == set(RAW_SITES), "a new producer: track it through producers, or list it"
    unknown = [(where, name) for where, name in sites.tracked if name not in PRODUCERS]
    assert not unknown, f"tracked under an unknown producer: {unknown}"
    named = {name for _where, name in sites.tracked} | set(RAW_SITES.values()) | NO_SITE
    assert named == set(PRODUCERS), "an inventory producer with no site, or a site with none"


def test_every_subscription_is_held_by_the_inventory():
    """A callback a transport runs later is a producer too: the fence cancels it."""
    assert _scan().subscribes == []


def test_the_fence_stops_every_inventory_producer():
    assert set(FENCE) == set(PRODUCERS)


def test_the_fence_lands_between_scheduling_a_broadcast_and_running_it():
    """Admitted work runs and is journaled; new work is refused."""
    inventory, manager = ProducerInventory(), ConnectionManager()
    journal, delivered = [], []

    class Client:
        async def accept(self) -> None:
            pass

        async def send_bytes(self, data: bytes) -> None:
            delivered.append(data)

    manager.observer = journal.append

    async def main() -> None:
        await manager.connect(Client())
        loop = asyncio.get_running_loop()
        scheduled = []
        worker = threading.Thread(target=lambda: scheduled.append(inventory.broadcast(
            "route_broadcasts", manager.broadcast({"stage": "uploading"}), loop)))
        worker.start()
        worker.join()  # scheduled from a job's thread; the loop has not run it yet
        await fence(inventory)
        with pytest.raises(ProducerFenced):
            inventory.broadcast("route_broadcasts", manager.broadcast({"stage": "late"}), loop)
        assert inventory.futures() == scheduled  # retire awaits it
        await asyncio.wrap_future(scheduled[0])

    asyncio.run(main())
    assert journal == [{"stage": "uploading"}] and delivered == [b'{"stage":"uploading"}']
    assert not inventory.admitting


def test_wall_tasks_never_pass_a_context_to_create_task():
    """``create_task(context=)`` is Python 3.11+; the wall must run on 3.9."""
    inventory = ProducerInventory()
    seen = []

    async def work() -> str:
        return "ran"

    async def main() -> str:
        loop = asyncio.get_running_loop()
        plain = loop.create_task

        def create_task_39(coro):  # the 3.9 signature: no ``context``
            seen.append(coro)
            return plain(coro)

        loop.create_task = create_task_39
        return await inventory.create_task("autoconnect", work())

    assert asyncio.run(main()) == "ran" and seen[0].cr_code.co_name == "work"


def test_vehicle_and_param_observer_subscriptions_are_held_and_fenced(monkeypatch):
    """Under lockstep the fence cancels them; on the wall none is held."""
    from contextlib import nullcontext

    from gcs.backend import producer_inventory
    from gcs.backend.full_param_store import FullParamStore
    from gcs.backend.vehicle_entry import VehicleEntry
    from navpy.core.ports.subscription import Subscription

    class Vehicle:
        def __init__(self) -> None:
            self.live: set[str] = set()

        def on_message(self, name, _callback):
            self.live.add(name)
            return Subscription(lambda: self.live.discard(name))

    class Scopes:
        def context(self, _name):
            return None

        def entered(self, _name):
            return nullcontext()

    wall = ProducerInventory()
    monkeypatch.setattr(producer_inventory, "producers", wall)
    for module in ("gcs.backend.vehicle_entry", "gcs.backend.full_param_store"):
        monkeypatch.setattr(f"{module}.producers", wall)
    VehicleEntry(1, Vehicle(), "s1-u1")
    assert wall.unsubscribe("vehicle_entry_messages") == 0  # the wall holds none

    lockstep = ProducerInventory()
    lockstep.scopes = Scopes()
    for module in ("gcs.backend.vehicle_entry", "gcs.backend.full_param_store"):
        monkeypatch.setattr(f"{module}.producers", lockstep)
    vehicle = Vehicle()
    VehicleEntry(1, vehicle, "s1-u1")
    FullParamStore()._ensure_subscribed(1, vehicle)
    assert {"STATUSTEXT", "RC_CHANNELS", "SWARM_HEARTBEAT", "PARAM_VALUE"} <= vehicle.live
    asyncio.run(fence(lockstep))
    assert vehicle.live == set()


#: Component packages the backend builds inside requests (vehicles, buses, loggers).
COMPONENTS = ("src/navpy/plugins/vehicle/ardupilot", "src/navpy/core/kernel/logging")


def test_every_component_worker_starts_detached_from_its_creating_request():
    """A worker a vehicle, bus or logger owns outlives the request that built
    it: it starts under ``rt.detached()``, so a request's cancel cannot stop
    it (the request runner faults any other thread left in its scope)."""
    missing = []
    for root in COMPONENTS:
        for path in sorted((ROOT / root).rglob("*.py")):
            tree = ast.parse(path.read_text(encoding="utf-8"))
            starts = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
                      and isinstance(n.func, ast.Attribute) and n.func.attr == "Thread"
                      and isinstance(n.func.value, ast.Name) and n.func.value.id == "rt"]
            detached = [n for n in ast.walk(tree) if isinstance(n, ast.withitem)
                        and isinstance(n.context_expr, ast.Call)
                        and getattr(n.context_expr.func, "attr", None) == "detached"]
            if starts and len(detached) < len(starts):
                missing.append(path.relative_to(ROOT).as_posix())
    assert missing == []
