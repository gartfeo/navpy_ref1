"""Whole-tree passes of the step-1 move tool: rewrite and leftover scan.

Both read every .py file under src/, tests/, scripts/ and tools/ (the
step-1 tools and their tests excepted); vendored files are scanned, never
edited.
"""

from __future__ import annotations

from pathlib import Path
from typing import Iterable

from step1_leftovers import Leftover, TreeState, scan_leftovers, tree_state
from step1_manifest import ManifestRow, module_name
from step1_move_files import is_vendored, python_files, src_files, vendored_hashes
from step1_move_plan import INIT, CommitPlan, TreeChange
from step1_rewrite import rewrite_source
from step1_rewrite_imports import Importer


def importer_for(old: str, new: str) -> Importer | None:
    """The file at old (at new after the move) as a module; None outside src/."""
    old_module, new_module = module_name(old), module_name(new)
    if old_module is None or new_module is None:
        return None
    return Importer(old_module, new_module, old.endswith("/" + INIT))


def rewrite_tree(repo: Path, plan: CommitPlan, change: TreeChange) -> tuple[dict[str, str], list[str]]:
    """New text of each file the move changes, keyed by its path after the
    move, and the problems left for a hand fix."""
    hashes = vendored_hashes(repo)
    texts: dict[str, str] = {}
    problems: list[str] = []
    for path in python_files(repo):
        new_path = plan.new_path(path)
        data = (repo / path).read_bytes()
        text = _decoded(data)
        if _removed(new_path, change) or text is None:
            continue
        result = rewrite_source(text, plan.maps, importer_for(path, new_path))
        problems += [f"{new_path}: {problem}" for problem in result.problems]
        if result.text == text:
            continue
        if is_vendored(data, hashes):
            problems.append(f"{new_path}: vendored, needs {result.edits} edits by hand")
        else:
            texts[new_path] = result.text
    return texts, problems


def planned_leftovers(repo: Path, rows: tuple[ManifestRow, ...], plan: CommitPlan,
                      change: TreeChange, texts: dict[str, str]) -> list[Leftover]:
    """Leftovers the move would leave, judged on the planned tree."""
    files: list[tuple[str, str]] = []
    for path in python_files(repo):
        new_path = plan.new_path(path)
        text = texts[new_path] if new_path in texts else _decoded((repo / path).read_bytes())
        if not _removed(new_path, change) and text is not None:
            files.append((new_path, text))
    return scan_tree(files, tree_state(rows, change.files))


def check_tree(repo: Path, rows: tuple[ManifestRow, ...]) -> list[Leftover]:
    """Leftovers in the tree as it is on disk."""
    files: list[tuple[str, str]] = []
    for path in python_files(repo):
        text = _decoded((repo / path).read_bytes())
        if text is not None:
            files.append((path, text))
    return scan_tree(files, tree_state(rows, src_files(repo)))


def scan_tree(files: Iterable[tuple[str, str]], state: TreeState) -> list[Leftover]:
    return [leftover for path, text in files
            for leftover in scan_leftovers(path, text, state, importer_for(path, path))]


def _removed(path: str, change: TreeChange) -> bool:
    return path.startswith("src/") and path not in change.files


def _decoded(data: bytes) -> str | None:
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return None
