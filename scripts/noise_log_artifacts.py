"""Inventory and copy a single new DataFlash log from an inactive owned instance."""

import argparse
import hashlib
import json
from pathlib import Path
from simtime_step_artifacts import owned_directory


def inventory(directory: Path) -> dict:
    logs = directory / "logs"
    if logs.is_symlink() or (logs.exists() and not logs.is_dir()):
        raise ValueError("unexpected logs directory")
    entries = {}
    if logs.exists():
        for path in logs.iterdir():
            if path.suffix.upper() != ".BIN":
                continue
            if path.is_symlink() or not path.is_file():
                raise ValueError("unexpected BIN entry")
            stat = path.stat()
            entries[path.name] = dict(size=stat.st_size, mtime_ns=stat.st_mtime_ns, inode=stat.st_ino)
    marker = logs / "LASTLOG.TXT"
    if marker.is_symlink() or (marker.exists() and not marker.is_file()):
        raise ValueError("unexpected log number marker")
    if entries and not marker.exists():
        raise ValueError("existing logs have no number marker")
    return dict(directory=str(directory), entries=entries,
                lastlog=int(marker.read_text().strip()) if marker.exists() else 0)


def new_log(before: dict, after: dict) -> str:
    if before["directory"] != after["directory"]:
        raise ValueError("instance identity changed")
    if any(after["entries"].get(name) != meta for name, meta in before["entries"].items()):
        raise ValueError("existing BIN changed or disappeared")
    added = set(after["entries"]) - set(before["entries"])
    if len(added) != 1:
        raise ValueError("expected exactly one new BIN")
    name = added.pop()
    if after["lastlog"] != before["lastlog"] + 1 or int(Path(name).stem) != after["lastlog"]:
        raise ValueError("new BIN does not match incremented log number")
    return name


def run(operation: str, root: Path, vehicle: int, case: Path) -> dict:
    directory = owned_directory(root, vehicle)
    current = inventory(directory)
    with (case / f"logs-{operation}.json").open("x") as output:
        json.dump(current, output, indent=2)
    if operation == "before":
        return current
    before = json.loads((case / "logs-before.json").read_text())
    name = new_log(before, current)
    source = directory / "logs" / name
    payload = source.read_bytes()
    with (case / "flight.BIN").open("xb") as output:
        output.write(payload)
    if inventory(directory) != current or (case / "flight.BIN").read_bytes() != payload:
        raise ValueError("BIN changed while copying")
    result = dict(source=str(source), sha256=hashlib.sha256(payload).hexdigest(),
                  binding="one new log in inactive owned instance under exclusive evaluator lock")
    (case / "bin-binding.json").write_text(json.dumps(result, indent=2))
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("before", "after"))
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--vehicle", required=True, type=int)
    parser.add_argument("--case", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(run(args.operation, args.root, args.vehicle, args.case)))


if __name__ == "__main__":
    main()
