"""Move one refactor step-1 commit's files and rewrite every reference to them.

    python scripts/step1_move.py --commit c05 [--dry-run] [--repo ROOT]
    python scripts/step1_move.py --check [--repo ROOT]

--commit git-moves the commit's manifest rows (docs/design/step1/), adds an
empty __init__.py to each new folder no row brings one to, keeps an old
folder a package while it still holds files, removes emptied old folders,
and rewrites imports, dotted names, patch targets and src/navpy paths in
every .py file under src/, tests/, scripts/ and tools/. Everything it
changes is staged; nothing is committed. --dry-run prints the same plan
and touches nothing. --check only scans for leftovers.

Exit 0: clean. Exit 1: leftovers or hand fixes remain (listed; after a real
run the moves are applied, fix them and rerun --check). Exit 2: the commit
cannot run on this tree; nothing was touched.
"""

from __future__ import annotations

import argparse
import sys
from functools import partial
from pathlib import Path
from typing import List, Sequence, TextIO

from step1_leftovers import Leftover
from step1_manifest import NAVPY_ROOT, ManifestRow, load_commits, load_manifest
from step1_move_files import (create_empty, is_placeholder, move_files, present_folders,
                              removable_init, removal_blockers, remove_folders, src_files,
                              write_texts)
from step1_move_plan import (CommitPlan, PlanError, TreeChange, build_plan, old_folders,
                             preflight, tree_change)
from step1_move_tree import check_tree, planned_leftovers, rewrite_tree

REPO_ROOT = Path(__file__).resolve().parents[1]


def main(argv: Sequence[str] | None = None, out: TextIO = sys.stdout) -> int:
    args = _parser().parse_args(argv)
    repo = Path(args.repo).resolve()
    rows = load_manifest(repo)
    if args.check:
        return _report([], check_tree(repo, rows), out)
    try:
        plan = build_plan(rows, load_commits(repo), args.commit)
    except PlanError as error:
        print(f"step1_move: {error}", file=out)
        return 2
    return run_commit(repo, rows, plan, args.dry_run, out)


def run_commit(repo: Path, rows: tuple[ManifestRow, ...], plan: CommitPlan, dry_run: bool,
               out: TextIO) -> int:
    files = src_files(repo)
    placeholder = partial(is_placeholder, repo)
    delete_rows = frozenset(NAVPY_ROOT + row.src for row in rows if row.action == "delete")
    errors = preflight(plan, files, placeholder)
    if not errors:
        change = tree_change(plan, rows, files, present_folders(repo, old_folders(rows)),
                             placeholder)
        errors = _removal_errors(repo, plan, change, delete_rows)
    if errors:
        print(f"step1_move {plan.commit}: cannot move, nothing touched:", file=out)
        print("".join(f"  {error}\n" for error in errors), end="", file=out)
        return 2
    texts, problems = rewrite_tree(repo, plan, change)
    _print_plan(plan, change, texts, dry_run, out)
    if dry_run:
        return _report(problems, planned_leftovers(repo, rows, plan, change, texts), out)
    move_files(repo, plan.moves)
    write_texts(repo, texts)
    create_empty(repo, change.created)
    remove_folders(repo, change.removed, delete_rows)
    return _report(problems, check_tree(repo, rows), out)


def _removal_errors(repo: Path, plan: CommitPlan, change: TreeChange,
                    delete_rows: frozenset[str]) -> List[str]:
    """Files, hidden ones included, that would stay in an old folder the
    commit empties: neither moved, bytecode cache nor a removable init."""
    moving = frozenset(move.old for move in plan.moves)
    blockers = removal_blockers(repo, change.removed, lambda path: path in moving
                                or removable_init(repo, path, delete_rows))
    return [f"{path}: left in a folder this commit empties; move or delete it first"
            for path in blockers]


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    task = parser.add_mutually_exclusive_group(required=True)
    task.add_argument("--commit", help="step-1 commit to move, as in commits.tsv (c03..c18)")
    task.add_argument("--check", action="store_true", help="only scan the tree for leftovers")
    parser.add_argument("--dry-run", action="store_true", help="print the plan, touch nothing")
    parser.add_argument("--repo", default=str(REPO_ROOT), help="repository root")
    return parser


def _print_plan(plan: CommitPlan, change: TreeChange, texts: dict[str, str], dry_run: bool,
                out: TextIO) -> None:
    mode = " (dry run, nothing touched)" if dry_run else ""
    print(f"step1_move {plan.commit}{mode}: {len(plan.moves)} moves, {len(texts)} files rewritten",
          file=out)
    lines = [f"move {move.old} -> {move.new}" for move in plan.moves]
    lines += [f"create {path}" for path in change.created]
    lines += [f"remove {folder}" for folder in change.removed]
    lines += [f"rewrite {path}" for path in sorted(texts)]
    print("".join(f"  {line}\n" for line in lines), end="", file=out)


def _report(problems: list[str], leftovers: list[Leftover], out: TextIO) -> int:
    if problems:
        print(f"hand fixes ({len(problems)}):", file=out)
        print("".join(f"  {problem}\n" for problem in problems), end="", file=out)
    if leftovers:
        print(f"leftovers ({len(leftovers)}):", file=out)
        print("".join(f"  {leftover}\n" for leftover in leftovers), end="", file=out)
    if problems or leftovers:
        return 1
    print("no leftovers", file=out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
