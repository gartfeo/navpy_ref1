"""Board parameter backup and restore (the hitl.py `backup` and `restore` commands)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from tools.tuning.board import Board
from tools.tuning.hitl_params import board_owned, changes, check_disarmed, fetch_params, read_param_file, write_backup


def restore(board: Board, backup: Path) -> dict:
    """Write a backup file back (e.g. after the flight firmware is reflashed): every value the board
    has under that name, except the firmware-owned ones; reboots until a fresh boot agrees."""
    values = read_param_file(backup)
    check_disarmed(board.connect())
    desired = {n: v for n, v in values.items() if not board_owned(n)}
    current = board.load(desired)
    return {"restored_from": str(backup), "device": board.device, "written": len(desired),
            "skipped": sorted(n for n in values if board_owned(n)),
            "absent": sorted(n for n in desired if n not in current),
            "differing": sorted(changes(desired, current)),
            "refused_sets": board.record.get("refused_sets", []), "load_rounds": board.record.get("load_rounds")}


def backup(args: argparse.Namespace) -> None:
    """Save every parameter of the board to a new file (read only)."""
    board = Board(args.board, Path(args.out).parent, [], baud=args.baud)
    try:
        board.connect()
        params = fetch_params(board)
        print(f"{len(params)} parameters saved to {write_backup(Path(args.out), params, args.board)}")
    finally:
        board.close_link()


def restore_cmd(args: argparse.Namespace) -> None:
    """Write a backup file back to the board, e.g. after the flight firmware is reflashed."""
    board = Board(args.board, Path(args.file).parent, [], baud=args.baud)
    try:
        result = restore(board, Path(args.file))
    finally:
        board.close_link()
    print(json.dumps(result, indent=1))
    sys.exit(1 if result["differing"] or result["absent"] else 0)
