"""List the machine's CPU shares (``bounded_cpu_share.md``); it changes nothing.

Usage: python scripts/cpu_share.py --list
"""

from __future__ import annotations

import argparse
import datetime as dt
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from gcs.backend.cpu_share import ShareRecord  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--list", action="store_true", required=True,
                        help="print every share with its state")
    parser.parse_args(argv)
    record = ShareRecord()
    try:
        rows = record.listing()
    except ValueError as error:
        print(f"{record.path}: {error}", file=sys.stderr)
        return 1
    print(f"{record.path}: {len(rows)} share(s)")
    for share, state in rows:
        expiry = dt.datetime.fromtimestamp(share.expires_at).isoformat(timespec="seconds")
        print(f"  pid {share.pid} CPUs {','.join(map(str, share.cpus))} until {expiry}: "
              f"{state} [{share.label}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
