"""Atomic persistence and cross-process locks for the instance registry."""

from __future__ import annotations

import json
import os
import time
from contextlib import AbstractContextManager, contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterator

from gcs.backend import instance_registry_runtime as runtime


LOCK_TIMEOUT_S = 10.0
LOCK_STALE_S = 30.0


def registry_path() -> Path:
    """Return the machine-shared registry path."""
    override = os.environ.get("GCS_INSTANCE_REGISTRY")
    if override:
        return Path(override)
    return Path.home() / ".gcs" / "instances.json"


def owner_for(clone: str) -> str:
    """Return a stable directory-based owner identity."""
    return f"dir:{os.path.normcase(os.path.abspath(clone))}"


def lock_path() -> Path:
    """Return the registry transaction lock path."""
    return registry_path().with_suffix(".lock")


def lock_holder_alive(lock: Path) -> bool:
    """Return whether the PID recorded in *lock* is still alive."""
    try:
        return runtime.pid_alive(int(lock.read_text().split()[0]))
    except (OSError, ValueError, IndexError):
        return False


def unlink_if_owner(lock: Path) -> None:
    """Remove *lock* only when this process still owns that exact file."""
    try:
        if lock.read_text().split()[0] == str(os.getpid()):
            lock.unlink()
    except (OSError, ValueError, IndexError):
        pass


@contextmanager
def locked() -> Iterator[None]:
    """Hold the registry's exclusive lock for one atomic transaction."""
    lock = lock_path()
    lock.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.time() + LOCK_TIMEOUT_S
    while True:
        try:
            descriptor = os.open(
                str(lock),
                os.O_CREAT | os.O_EXCL | os.O_WRONLY,
            )
            os.write(descriptor, str(os.getpid()).encode())
            os.close(descriptor)
            break
        except (FileExistsError, PermissionError) as error:
            # PermissionError as well as FileExistsError: on Windows a lock file
            # another process has just unlinked lingers in a delete-pending
            # state, and O_CREAT|O_EXCL against it raises EACCES rather than
            # EEXIST. That is ordinary contention and must queue like
            # contention; catching only FileExistsError killed the loser of two
            # simultaneous launches outright.
            broke = _break_dead_stale_lock(lock)
            # Deadline checked on EVERY pass, including the one that just broke
            # a lock. `_break_dead_stale_lock` also answers True when the file is
            # simply absent, so an unwritable directory -- a PermissionError that
            # will never clear -- would otherwise retry forever behind `continue`
            # and never reach this check.
            if time.time() > deadline:
                raise TimeoutError(
                    f"could not acquire registry lock {lock}: {error}"
                ) from error
            if not broke:
                time.sleep(0.1)
    try:
        yield
    finally:
        unlink_if_owner(lock)


def _break_dead_stale_lock(lock: Path) -> bool:
    try:
        if (
            time.time() - lock.stat().st_mtime > LOCK_STALE_S
            and not lock_holder_alive(lock)
        ):
            lock.unlink()
            return True
    except FileNotFoundError:
        return True
    except OSError:
        # Present but not inspectable/removable (delete-pending, or held open).
        # Not ours to break: report it as still held.
        return False
    return False


def sitl_launch_lock_path() -> Path:
    """Return the machine-wide SITL launch-serialization lock path."""
    return registry_path().with_name("sitl_launch.lock")


@contextmanager
def sitl_launch_lock(
    timeout: float = 300.0,
    stale_after: float = 120.0,
) -> Iterator[bool]:
    """Serialize swarm initialization, yielding whether the lock was acquired."""
    lock = sitl_launch_lock_path()
    lock.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.time() + timeout
    acquired = False
    while True:
        try:
            descriptor = os.open(
                str(lock),
                os.O_CREAT | os.O_EXCL | os.O_WRONLY,
            )
            os.write(descriptor, f"{os.getpid()} {now()}".encode())
            os.close(descriptor)
            acquired = True
            break
        except (FileExistsError, PermissionError):
            # Delete-pending EACCES on Windows is contention here too; see
            # `locked`. Deadline checked every pass for the same reason.
            broke = _break_old_launch_lock(lock, stale_after=stale_after)
            if time.time() > deadline:
                break
            if not broke:
                time.sleep(0.5)
    try:
        yield acquired
    finally:
        if acquired:
            try:
                lock.unlink()
            except FileNotFoundError:
                pass


def _break_old_launch_lock(lock: Path, *, stale_after: float) -> bool:
    try:
        if time.time() - lock.stat().st_mtime > stale_after:
            lock.unlink()
            return True
    except FileNotFoundError:
        return True
    except OSError:
        return False
    return False


def load() -> dict[str, Any]:
    """Load a valid registry document or return an empty one."""
    try:
        data = json.loads(registry_path().read_text(encoding="utf-8"))
        if isinstance(data, dict) and isinstance(data.get("instances"), dict):
            return data
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        pass
    return {"instances": {}}


def save(data: dict[str, Any]) -> None:
    """Atomically replace the registry document."""
    path = registry_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(data, indent=2), encoding="utf-8")
    temporary.replace(path)


@dataclass(frozen=True)
class StoreAccess:
    """Injectable atomic-store boundary used by focused registry services."""

    locked: Callable[[], AbstractContextManager[None]]
    load: Callable[[], dict[str, Any]]
    save: Callable[[dict[str, Any]], None]


DEFAULT_ACCESS = StoreAccess(locked=locked, load=load, save=save)


def now() -> str:
    """Return the current UTC timestamp in registry format."""
    return datetime.now(timezone.utc).isoformat()
