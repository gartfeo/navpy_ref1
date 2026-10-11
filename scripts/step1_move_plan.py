"""One step-1 commit's move, planned from the manifest on a file listing.

Pure: the caller lists files; nothing here touches the tree. The plan moves
the commit's rows, gives every new folder an empty __init__.py unless a row
brings one, keeps an old folder a regular package with an empty __init__.py
when its own init moves out before its other files, and removes old folders
left holding nothing but removable package inits (and __pycache__).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from step1_manifest import (NAVPY_ROOT, CommitPrefix, ManifestRow, module_map, module_name,
                            repo_path, rows_for_commit)
from step1_rewrite_maps import RewriteMaps

INIT = "__init__.py"


class PlanError(ValueError):
    """The commit cannot be planned or applied to this tree."""


@dataclass(frozen=True)
class Move:
    old: str
    new: str


@dataclass(frozen=True)
class CommitPlan:
    commit: str
    moves: tuple[Move, ...]
    maps: RewriteMaps

    def new_path(self, path: str) -> str:
        """Where a repo path lives after the move."""
        return next((move.new for move in self.moves if move.old == path), path)


@dataclass(frozen=True)
class TreeChange:
    """created: empty package inits to add. removed: old folders to delete,
    deepest first. files: the files under src/ afterwards."""

    created: tuple[str, ...]
    removed: tuple[str, ...]
    files: frozenset[str]


def build_plan(rows: tuple[ManifestRow, ...], prefixes: tuple[CommitPrefix, ...],
               commit: str) -> CommitPlan:
    selected = rows_for_commit(rows, prefixes, commit)
    if not selected:
        raise PlanError(f"no manifest row moves in {commit}")
    moves = tuple(Move(NAVPY_ROOT + row.src, _home(row)) for row in selected)
    modules: dict[str, str] = {}
    for move in moves:
        old, new = module_name(move.old), module_name(move.new)
        if old is not None and new is not None:
            modules[old] = new
    paths = {move.old[len("src/"):]: move.new[len("src/"):] for move in moves}
    known = frozenset(module_map(rows, "step1"))
    return CommitPlan(commit, moves, RewriteMaps(modules, known, paths))


def preflight(plan: CommitPlan, files: frozenset[str],
              placeholder: Callable[[str], bool]) -> list[str]:
    """Why the moves cannot run on this tree; empty when they can."""
    errors: list[str] = []
    for move in plan.moves:
        if move.old not in files:
            state = "already moved" if move.new in files else "missing"
            errors.append(f"{move.old}: {state}")
        elif move.new in files and not (move.new.endswith("/" + INIT) and placeholder(move.new)):
            errors.append(f"{move.new}: exists and is not an empty package init")
    return errors


def tree_change(plan: CommitPlan, rows: tuple[ManifestRow, ...], files: frozenset[str],
                present_folders: frozenset[str], placeholder: Callable[[str], bool]) -> TreeChange:
    """The tree after the moves. present_folders: old folders on disk now;
    placeholder: whether a package init holds nothing (or is yet to be made)."""
    olds = {move.old for move in plan.moves}
    moved = (files - olds) | {move.new for move in plan.moves}
    created = [*_new_package_inits(plan.moves, moved), *_kept_package_inits(plan.moves, moved)]
    after = moved | set(created)
    removed = _emptied_folders(rows, after, present_folders, placeholder)
    final = frozenset(path for path in after if not any(path.startswith(f + "/") for f in removed))
    return TreeChange(tuple(created), tuple(removed), final)


def old_folders(rows: tuple[ManifestRow, ...]) -> frozenset[str]:
    """Repo paths of every folder holding an old manifest file."""
    folders: set[str] = set()
    for row in rows:
        pieces = row.src.split("/")[:-1]
        folders.update(NAVPY_ROOT + "/".join(pieces[:count]) for count in range(1, len(pieces) + 1))
    return frozenset(folders)


def removable_init(path: str, rows_by_src: dict[str, ManifestRow], files: frozenset[str],
                   placeholder: Callable[[str], bool]) -> bool:
    """A package init an emptied old folder may lose: a delete row, or the
    empty stand-in left after its row moved (its step-1 home exists)."""
    row = rows_by_src.get(path[len(NAVPY_ROOT):]) if path.startswith(NAVPY_ROOT) else None
    if row is None or not path.endswith("/" + INIT):
        return False
    return row.action == "delete" or (_home(row) in files and placeholder(path))


def _home(row: ManifestRow) -> str:
    home = repo_path(row.step1_dest)
    if home is None:
        raise PlanError(f"{row.src}: no step-1 home")
    return home


def _new_package_inits(moves: tuple[Move, ...], files: set[str]) -> list[str]:
    """Empty inits for new folders no row of this commit brings one to."""
    created: list[str] = []
    for move in moves:
        folder = move.new.rpartition("/")[0]
        while folder.count("/") >= 2:
            init = f"{folder}/{INIT}"
            if init not in files and init not in created:
                created.append(init)
            folder = folder.rpartition("/")[0]
    return created


def _kept_package_inits(moves: tuple[Move, ...], files: set[str]) -> list[str]:
    """Empty stand-ins for package inits moving out of folders that keep files."""
    return [
        move.old for move in moves
        if move.old.endswith("/" + INIT) and move.old not in files
        and any(path.startswith(move.old[:-len(INIT)]) for path in files)
    ]


def _emptied_folders(rows: tuple[ManifestRow, ...], files: set[str],
                     present_folders: frozenset[str], placeholder: Callable[[str], bool]
                     ) -> list[str]:
    rows_by_src = {row.src: row for row in rows}
    frozen = frozenset(files)
    emptied: list[str] = []
    for folder in sorted(present_folders, key=lambda path: (-path.count("/"), path)):
        inside = [path for path in files if path.startswith(folder + "/")]
        if all(removable_init(path, rows_by_src, frozen, placeholder) for path in inside):
            emptied.append(folder)
    return emptied
