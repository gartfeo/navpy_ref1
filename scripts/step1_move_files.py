"""Files and git for the step-1 move tool: listing, vendored check, changes.

Paths are repo-relative with forward slashes. Every change goes through git
(mv, add, rm) so the index holds the whole commit.
"""

from __future__ import annotations

import ast
import hashlib
import os
import re
import subprocess
from pathlib import Path
from typing import Callable, Iterable, Iterator, List, Mapping, Tuple

from step1_move_plan import INIT, Move

SCAN_ROOTS = ("src", "tests", "scripts", "tools")
SOLID_GUARD = Path("tests", "guards", "solid_architecture_guard.py")
_CACHE_FOLDER = "__pycache__"
_CACHE_SUFFIXES = (".pyc", ".pyo")
_SKIPPED_FOLDERS = frozenset({_CACHE_FOLDER, "node_modules"})
# The step-1 tools and their tests name old modules and paths on purpose.
_PLAN_TOOLING = re.compile(r"(?:test_)?step1_\w+\.py")
_GIT_BATCH = 100


def git(repo: Path, *args: str) -> str:
    result = subprocess.run(["git", *args], cwd=repo, capture_output=True, text=True, check=False)
    if result.returncode:
        raise RuntimeError(f"git {' '.join(args[:3])}...: {result.stderr.strip()}")
    return result.stdout


def walk_files(repo: Path, root: str) -> Iterator[str]:
    """Files under root, skipping caches, node_modules and dot folders."""
    for folder, folders, names in os.walk(repo / root):
        folders[:] = sorted(name for name in folders
                            if name not in _SKIPPED_FOLDERS and not name.startswith("."))
        base = Path(folder).relative_to(repo).as_posix()
        yield from (f"{base}/{name}" for name in sorted(names))


def python_files(repo: Path) -> list[str]:
    """The .py files a move rewrites and the leftover scan reads."""
    return [path for root in SCAN_ROOTS for path in walk_files(repo, root)
            if path.endswith(".py") and not _PLAN_TOOLING.fullmatch(path.rpartition("/")[2])]


def src_files(repo: Path) -> frozenset[str]:
    return frozenset(walk_files(repo, "src"))


def present_folders(repo: Path, folders: Iterable[str]) -> frozenset[str]:
    return frozenset(folder for folder in folders if (repo / folder).is_dir())


def vendored_hashes(repo: Path) -> frozenset[str]:
    """SHA256 (upper hex) of the vendored files the SOLID guard lists."""
    guard = repo / SOLID_GUARD
    if not guard.is_file():
        return frozenset()
    for node in ast.parse(guard.read_text(encoding="utf-8")).body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "VENDORED_SHA256" for target in node.targets
        ):
            return frozenset(value.upper() for value in ast.literal_eval(node.value).values())
    return frozenset()


def is_vendored(data: bytes, hashes: frozenset[str]) -> bool:
    return hashlib.sha256(data).hexdigest().upper() in hashes


def is_placeholder(repo: Path, path: str) -> bool:
    """A package init that holds nothing, or one still to be made."""
    file = repo / path
    return not file.exists() or not file.read_text(encoding="utf-8").strip()


def move_files(repo: Path, moves: Iterable[Move]) -> None:
    """git mv each file; an empty package init at the destination gives way."""
    for move in moves:
        target = repo / move.new
        if target.exists():
            _delete(repo, move.new)
        target.parent.mkdir(parents=True, exist_ok=True)
        git(repo, "mv", move.old, move.new)


def write_texts(repo: Path, texts: Mapping[str, str]) -> None:
    for path, text in texts.items():
        (repo / path).write_bytes(text.encode("utf-8"))
    stage(repo, texts)


def create_empty(repo: Path, paths: Iterable[str]) -> None:
    created = list(paths)
    for path in created:
        (repo / path).parent.mkdir(parents=True, exist_ok=True)
        (repo / path).write_bytes(b"")
    stage(repo, created)


def removable_init(repo: Path, path: str, delete_rows: frozenset[str]) -> bool:
    """A package init an emptied old folder may lose: a delete row's, even
    when it is not empty, or one that holds nothing."""
    return path.endswith("/" + INIT) and (path in delete_rows or is_placeholder(repo, path))


def folder_entries(repo: Path, folder: str) -> Iterator[Tuple[str, bool]]:
    """Every file and link under folder, hidden ones included, and whether it
    is bytecode cache (inside __pycache__, or a .pyc/.pyo file)."""
    for base, folders, names in os.walk(repo / folder):
        relative = Path(base).relative_to(repo).as_posix()
        in_cache = _CACHE_FOLDER in relative.split("/")
        links = [name for name in folders if (Path(base) / name).is_symlink()]
        yield from ((f"{relative}/{name}", False) for name in links)
        yield from ((f"{relative}/{name}", in_cache or name.endswith(_CACHE_SUFFIXES))
                    for name in names)


def removal_blockers(repo: Path, folders: Iterable[str],
                     removable: Callable[[str], bool]) -> List[str]:
    """What keeps the folders from being removed: every entry that is
    neither bytecode cache nor removable."""
    return [path for folder in folders if (repo / folder).is_dir()
            for path, cache in folder_entries(repo, folder) if not cache and not removable(path)]


def remove_folders(repo: Path, folders: Iterable[str], delete_rows: frozenset[str]) -> None:
    """Remove emptied old folders: git rm their package inits, delete their
    bytecode caches, then the folders left empty. Refuses, deleting nothing,
    while any folder holds anything else (hidden files included)."""
    folders = [folder for folder in folders if (repo / folder).is_dir()]
    blockers = removal_blockers(repo, folders, lambda path: removable_init(repo, path, delete_rows))
    if blockers:
        raise RuntimeError("emptied old folders still hold files that are neither caches nor"
                           f" removable package inits; nothing removed: {', '.join(blockers)}")
    for folder in folders:
        for path, cache in list(folder_entries(repo, folder)):
            if cache:
                (repo / path).unlink()
            else:
                _delete(repo, path)
        for base, _, _ in os.walk(repo / folder, topdown=False):
            os.rmdir(base)  # empty by now; fails rather than delete anything else


def stage(repo: Path, paths: Iterable[str]) -> None:
    batch = list(paths)
    for start in range(0, len(batch), _GIT_BATCH):
        git(repo, "add", "--", *batch[start:start + _GIT_BATCH])


def _delete(repo: Path, path: str) -> None:
    if git(repo, "ls-files", "--", path).strip():
        git(repo, "rm", "-q", "-f", "--", path)
    else:
        (repo / path).unlink()
