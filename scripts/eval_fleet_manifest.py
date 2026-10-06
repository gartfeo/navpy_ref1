"""What a fleet case records about itself, so a result can be read back later.

Archived runs have been found differing by 10x with nothing in the artefacts to
say why, and a fleet adds more ways for that to happen than a single-aircraft
run: two clocks, a randomised cell assignment, a sysid span that reaches past
the chat it launched on. None of those are visible in a miss distance, so each
one is written down beside it.
"""

from __future__ import annotations

import json
from pathlib import Path

MANIFEST_NAME = "case.json"


def write_case_manifest(
    case_dir: Path,
    *,
    speedup: float,
    launch_speedup: float,
    repetition: int,
    chat: int,
    span: list[int],
    assign_seed: int,
    winds: dict[int, tuple[float, float]],
    clock_rates: dict[int, float | None],
    identity: dict[str, object],
) -> None:
    """Record the case beside its own artefacts, before anything flies."""
    (case_dir / MANIFEST_NAME).write_text(
        json.dumps(
            {
                "speedup": speedup,
                "launch_speedup": launch_speedup,
                "repetition": repetition,
                "chat": chat,
                "span": span,
                "assign_seed": assign_seed,
                "assignment": {
                    str(sys_id): list(wind) for sys_id, wind in winds.items()
                },
                "clock_rates": {
                    str(sys_id): rate for sys_id, rate in clock_rates.items()
                },
                "source_identity": identity,
            },
            indent=2,
        ),
        encoding="utf-8",
    )


__all__ = ["MANIFEST_NAME", "write_case_manifest"]
