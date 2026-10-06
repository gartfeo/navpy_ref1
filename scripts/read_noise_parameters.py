"""Read all occurrences of the experimental parameters, using the peer environment."""

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys


def summarize(records: list[dict], expected: dict) -> dict:
    result = {}
    for name, value in expected.items():
        rows = [row for row in records if row["name"] == name]
        if not rows or any(row["value"] != value for row in rows):
            raise ValueError(f"missing or conflicting PARM records: {name}")
        result[name] = dict(value=value, count=len(rows), first_time_us=rows[0]["time_us"])
    return result


def read(case: Path, expected: dict) -> dict:
    from pymavlink import DFReader
    runtime = json.loads((case / "identity.json").read_text())["runtime"]
    if (sys.version != runtime["python"] or
            {name: importlib.metadata.version(name) for name in runtime["packages"]} != runtime["packages"]):
        raise ValueError("parameter reader must use recorded peer environment")
    path = case / "flight.BIN"
    reader = DFReader.DFReader_binary(str(path))
    records, bad = [], 0
    while True:
        message = reader.recv_msg()
        if message is None:
            break
        if message.get_type() == "BAD_DATA":
            bad += 1
        if message.get_type() == "PARM":
            name = message.Name
            if isinstance(name, bytes):
                name = name.decode("ascii")
            name = name.rstrip("\0")
            if name in expected:
                records.append(dict(name=name, value=float(message.Value), time_us=int(message.TimeUS)))
    return dict(parameters=summarize(records, expected), bad_data_records=bad,
                binary_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                reader_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), runtime=runtime)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", type=Path)
    parser.add_argument("expected", help="JSON parameter mapping")
    args = parser.parse_args()
    print(json.dumps(read(args.case, json.loads(args.expected))))


if __name__ == "__main__":
    main()
