"""Every navpy module imports as the first import of a new interpreter.

An in-process import loop hides init-order cycles: a module that only imports
after something else has loaded (the vehicle-init cycle) passes there. Here
each module is imported first in its own interpreter, in parallel.

Importing a.b.c first runs package a.b's import to the end before c's own
code. When that import of a.b, itself first in a fresh interpreter, already
loaded c, importing c first is exactly the same run, so it is not repeated.
Modules go in waves by depth to know that; today's facade packages load most
modules, and once c01 empties them every module gets its own interpreter.
"""

from __future__ import annotations

import json
import os
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor
from typing import Callable, Iterable

from tests.guards.step1_fresh import error_tail, run_fresh
from tests.guards.step1_sources import REPO_ROOT, navpy_sources

# Modules known to fail as the first import, listed so the guard holds the rest
# (none since c01 emptied the comm facade). A listed module that imports again
# fails the test until its entry is removed.
KNOWN_FAILURES: dict[str, str] = {}
IMPORT_ALONE = (
    "import importlib, json, sys\n"
    "importlib.import_module(sys.argv[1])\n"
    "print(json.dumps([m for m in sys.modules if m == 'navpy' or m.startswith('navpy.')]))\n"
)
ImportResult = tuple[str, frozenset[str] | None, str]   # module, navpy modules loaded, error


def _import_alone(module: str) -> ImportResult:
    """Import module first in a fresh interpreter; loaded is None when it fails."""
    result = run_fresh(IMPORT_ALONE, module)
    if result.returncode:
        return module, None, error_tail(result)
    return module, frozenset(json.loads(result.stdout.strip().splitlines()[-1])), ""


def fresh_import_failures(
        modules: Iterable[str],
        import_alone: Callable[[str], ImportResult] = _import_alone,
) -> tuple[dict[str, str], int]:
    """(module -> error for each that fails first in a fresh interpreter,
    number of interpreters started)."""
    waves: dict[int, list[str]] = defaultdict(list)
    for module in modules:
        waves[module.count(".")].append(module)
    loaded: dict[str, frozenset[str]] = {}
    failures: dict[str, str] = {}
    started = 0
    with ThreadPoolExecutor(max_workers=os.cpu_count() or 4) as pool:
        for depth in sorted(waves):
            alone = []
            for module in waves[depth]:
                parent = module.rpartition(".")[0]
                if module in loaded.get(parent, ()):
                    loaded[module] = loaded[parent]
                else:
                    alone.append(module)
            started += len(alone)
            for module, names, error in pool.map(import_alone, alone):
                if names is None:
                    failures[module] = error
                else:
                    loaded[module] = names
    return failures, started


def test_every_navpy_module_imports_first_in_a_fresh_interpreter() -> None:
    modules = sorted(source.module for source in navpy_sources(REPO_ROOT).values())
    failures, started = fresh_import_failures(modules)
    unexpected = [f"{module}:\n    {error}" for module, error in sorted(failures.items())
                  if module not in KNOWN_FAILURES]
    fixed = sorted(set(KNOWN_FAILURES) - set(failures))

    assert len(modules) > 500 and started > 0
    assert not unexpected, "modules that fail as the first import:\n" + "\n".join(unexpected)
    assert not fixed, f"these import now; remove them from KNOWN_FAILURES: {fixed}"


def test_a_module_its_package_already_loaded_gets_no_interpreter_of_its_own() -> None:
    loads = {
        "navpy": {"navpy"},
        "navpy.a": {"navpy", "navpy.a", "navpy.a.b"},
        "navpy.a.d": {"navpy", "navpy.a", "navpy.a.b", "navpy.a.d"},
        "navpy.c": None,
        "navpy.c.e": None,
    }
    started: list[str] = []

    def import_alone(module: str) -> ImportResult:
        started.append(module)
        names = loads[module]
        return module, None if names is None else frozenset(names), "" if names else "boom"

    failures, count = fresh_import_failures(
        ["navpy", "navpy.a", "navpy.a.b", "navpy.a.d", "navpy.c", "navpy.c.e"], import_alone)

    assert sorted(started) == ["navpy", "navpy.a", "navpy.a.d", "navpy.c", "navpy.c.e"]
    assert count == 5
    assert failures == {"navpy.c": "boom", "navpy.c.e": "boom"}
