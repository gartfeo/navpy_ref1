"""Say whether a traced case is ELIGIBLE, from its files (D8).

Run by path with the directory a case's determinism files were written to
(``pixel_pn_determinism_evidence``), the one its result is published in::

    python scripts/pixel_pn_determinism_verdict.py <case directory>

It prints the verdict as JSON: the status, the failed rules, and each
rule's reason. It exits 0 when the case is ELIGIBLE and 1 when it is
INSPECTABLE, so a harness can gate on it. The reading and the rules are
``navpy.modules.vision.sim.determinism_eligibility``'s, D8 of the
LANDING2 step-1 plan; this is only their
command line.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from navpy.modules.vision.sim.determinism_eligibility import (
    Verdict,
    case_verdict,
)

ELIGIBLE_EXIT = 0
INSPECTABLE_EXIT = 1


def verdict_payload(verdict: Verdict) -> dict[str, Any]:
    """The verdict as the command line prints it."""
    return {
        "status": verdict.status,
        "failed_rules": list(verdict.failed_rules),
        "reasons": [
            {"rule": reason.rule, "text": reason.text}
            for reason in verdict.reasons
        ],
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Judge a traced case by D8: ELIGIBLE or INSPECTABLE."
    )
    parser.add_argument(
        "directory",
        type=Path,
        help="the case's directory, holding determinism_manifest.json",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Print the case's verdict; its exit code says which it is."""
    options = _parser().parse_args(argv)
    verdict = case_verdict(options.directory)
    print(json.dumps(verdict_payload(verdict), indent=2))
    return ELIGIBLE_EXIT if verdict.eligible else INSPECTABLE_EXIT


if __name__ == "__main__":
    sys.exit(main())


__all__ = [
    "ELIGIBLE_EXIT",
    "INSPECTABLE_EXIT",
    "main",
    "verdict_payload",
]
