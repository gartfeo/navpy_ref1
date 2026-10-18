"""Core and plugins name no mission product (docs/design/delivery_product.md).

Owner ruling 2026-10-12: core and swarm should not know which missions exist.
Each refused word names a product or its parts; a new product adds its words.
The scan is lexical: identifiers split on ``_``, ``-`` and camelCase, strings
and comments, whole words, any case. A refused compound also counts when an
identifier splits it (pick_up, PickUpLatch); prose with a space ("pick up") is
two plain words. It checks words, not meaning; each review still checks that
no product rule hides behind a neutral name.
"""

from __future__ import annotations

import re
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCANNED = ("src/navpy/core", "src/navpy/plugins")
VENDORED = "_mavftp"
REFUSED = frozenset({
    "delivery", "deliveries", "ddh", "hub", "hubs", "pickup", "pick-up", "catcher",
    "medicine", "survey", "spraying", "surveillance", "patrol",
})
_ARDUPILOT = "src/navpy/plugins/vehicle/ardupilot/"
# (path, word, count, reason). A missing or stale row fails.
ALLOWED = (
    (_ARDUPILOT + "mission_encoding.py", "delivery", 3, "the wire codec the GCS imports"),
    (_ARDUPILOT + "mission_encoding.py", "hub", 3, "the wire codec the GCS imports"),
    ("src/navpy/core/approach/vision_nav/command_hold.py", "delivery", 1,
     "names the design file final_approach_delivery_fix"),
)
_RUN = re.compile(r"[A-Za-z0-9_]+(?:-[A-Za-z0-9_]+)*")
_JOINER = re.compile(r"[-_]")
_PART = re.compile(r"[A-Z]+(?=[A-Z][a-z])|[A-Z]?[a-z]+|[A-Z]+|[0-9]+")

Hits = dict[tuple[str, str], int]


def _split(run: str) -> list[str]:
    """A refused run counts whole, in any case (such as pick-up); else its parts."""
    if run.lower() in REFUSED:
        return [run]
    pieces = [piece for piece in _JOINER.split(run) if piece]
    if pieces != [run]:  # any joiner, even a leading or trailing one
        return [word for piece in pieces for word in _split(piece)]
    return _PART.findall(run)


def words(text: str) -> Counter[str]:
    found: Counter[str] = Counter()
    for run in _RUN.findall(text):
        parts = [part.lower() for part in _split(run)]
        index = 0
        while index < len(parts):
            pair = "".join(parts[index:index + 2])
            if index + 1 < len(parts) and pair in REFUSED:  # such as pick_up
                found[pair] += 1
                index += 2
                continue
            if parts[index] in REFUSED:
                found[parts[index]] += 1
            index += 1
    return found


def scan() -> Hits:
    hits: Hits = {}
    for top in SCANNED:
        for path in sorted((ROOT / top).rglob("*")):
            if path.suffix not in (".py", ".json") or VENDORED in path.parts:
                continue
            rel = path.relative_to(ROOT).as_posix()
            for word, count in words(path.read_text(encoding="utf-8")).items():
                hits[(rel, word)] = count
    return hits


def check(hits: Hits, allowed: Hits) -> tuple[Hits, Hits]:
    """Words without an allow row, and allow rows whose count does not match."""
    unexpected = {key: n for key, n in hits.items() if key not in allowed}
    stale = {key: n for key, n in allowed.items() if hits.get(key) != n}
    return unexpected, stale


def test_core_and_plugins_name_no_product() -> None:
    hits = scan()
    unexpected, stale = check(hits, {(p, w): n for p, w, n, _ in ALLOWED})
    assert not unexpected, f"product words in core or plugins: {unexpected}"
    assert not stale, f"stale allow rows {stale}; found {[hits.get(k) for k in stale]}"


def test_the_scan_finds_every_form_and_passes_neutral_words() -> None:
    sample = '''
default_delivery_hub = DeliveryFlow()  # the DDH route
class DDHReturn: "a pick-up or a Pickup"
DELIVERY_HUBS = "deliveries"
deliver(); delivered = AAS_DEL_PITCH; hubert = 1
mark = "HuB"  # dElIvErY by PickUp, not a re-hub-bed
pick_up_altitude = PICK_UP; class PickUpLatch: "pick up the task"
_dElIvErY = __HuB__; pIcKuP_ = 0
'''
    assert words(sample) == Counter({
        "delivery": 5, "hub": 4, "ddh": 2, "pick-up": 1, "pickup": 6,
        "hubs": 1, "deliveries": 1,
    })


def test_an_allow_row_escapes_only_its_exact_count() -> None:
    hits = {("a.py", "hub"): 2, ("b.py", "ddh"): 1}
    assert check(hits, {("a.py", "hub"): 2}) == ({("b.py", "ddh"): 1}, {})
    assert check(hits, {("a.py", "hub"): 3, ("b.py", "ddh"): 1}) == (
        {}, {("a.py", "hub"): 3},
    )
    assert check({}, {("a.py", "hub"): 2}) == ({}, {("a.py", "hub"): 2})
