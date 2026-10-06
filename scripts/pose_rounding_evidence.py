"""Validate lossless, same-conversion pose pairs before offline attribution."""

from dataclasses import dataclass
import csv
import hashlib
import json
import math
from pathlib import Path
import struct

from simtime_navigation_protocol import Snapshot, WINDOW_SIZE

FIELDS = ("version,boot0,boot1,vehicle,step,tick,source_us,truth_us,publish_us,conversion_us,"
          "sequence,physics_sequence,expected_physics_sequence,branch,ftype_size,valid,"
          "origin_lat,origin_lng,home_alt,dlat_hex,dlng_hex,alt_cm_hex,"
          "rounded_lat,rounded_lng,rounded_alt").split(",")


@dataclass(frozen=True)
class PosePair:
    rounded: tuple[float, float, float]
    precast: tuple[float, float, float]


def validate_pairs(rows: list[dict], snapshots: list[Snapshot]) -> list[PosePair]:
    if not rows or len(rows) != len(snapshots):
        raise ValueError("incomplete pose pairs")
    pairs = []
    previous = None
    for row, snapshot in zip(rows, snapshots):
        if set(row) != set(FIELDS) or any(value is None for value in row.values()):
            raise ValueError("pose schema mismatch")
        ints = {key: int(value) for key, value in row.items() if not key.endswith("_hex")}
        if any(row[key] != str(value) for key, value in ints.items()):
            raise ValueError("noncanonical pose integer")
        identity = snapshot.identity
        expected = dict(version=1, boot0=identity.boot[0], boot1=identity.boot[1],
            vehicle=identity.vehicle, step=identity.step, tick=identity.tick,
            source_us=identity.source_us, truth_us=snapshot.truth_us,
            publish_us=snapshot.truth_us, valid=1, ftype_size=8)
        if any(ints[key] != value for key, value in expected.items()):
            raise ValueError(f"pose identity/validity mismatch at step {identity.step}")
        # This experiment is explicitly bound to SIM_RATE_HZ=1000.
        age = ints["publish_us"] - ints["conversion_us"]
        if ints["branch"] not in (1, 2, 3) or age != (0 if ints["branch"] == 2 else 1000):
            raise ValueError("pose conversion branch/time mismatch")
        if (ints["physics_sequence"] != ints["expected_physics_sequence"] or
                ints["sequence"] < ints["physics_sequence"] or ints["physics_sequence"] <= 0):
            raise ValueError("stale pose conversion")
        if previous is not None:
            dt = ints["publish_us"] - previous["publish_us"]
            if (dt != 20000 or ints["physics_sequence"] - previous["physics_sequence"] != dt // 1000
                    or ints["sequence"] <= previous["sequence"]):
                raise ValueError("pose sequence/cadence mismatch")
        previous = ints
        dlat, dlng, alt_cm = (float.fromhex(row[key]) for key in ("dlat_hex", "dlng_hex", "alt_cm_hex"))
        if not all(math.isfinite(value) for value in (dlat, dlng, alt_cm)):
            raise ValueError("nonfinite pose")
        if not (-2**31 <= dlat <= 2**31-1 and -3600000000 <= dlng <= 3600000000
                and -2**31 <= alt_cm <= 2**31-1 and -2**31 <= ints["home_alt"] <= 2**31-1):
            raise ValueError("unsupported pose conversion range")
        lat = ints["origin_lat"] + int(dlat)
        lng = ints["origin_lng"] + int(dlng)
        altitude = int(alt_cm)
        if not (-900000000 <= lat <= 900000000 and -1800000000 <= lng <= 1800000000
                and -900000000 <= ints["origin_lat"] <= 900000000
                and -1800000000 <= ints["origin_lng"] <= 1800000000):
            raise ValueError("latitude limit/longitude wrap unsupported")
        if (lat, lng, altitude) != tuple(ints[key] for key in ("rounded_lat", "rounded_lng", "rounded_alt")):
            raise ValueError("pose integer shadow mismatch")
        rounded = (lat * 1e-7, lng * 1e-7, altitude * 1e-2)
        original = (snapshot.truth.latitude, snapshot.truth.longitude, snapshot.truth.altitude)
        if struct.pack("<3d", *rounded) != struct.pack("<3d", *original):
            raise ValueError("pose legacy truth bits mismatch")
        precast = ((float(ints["origin_lat"]) + dlat) * 1e-7,
                   (float(ints["origin_lng"]) + dlng) * 1e-7, alt_cm * 1e-2)
        pairs.append(PosePair(rounded, precast))
    return pairs


def read_pairs(case: Path, peer: dict) -> tuple[list[dict], list[PosePair]]:
    manifest = json.loads((case.parent / "fleet.json").read_text())
    if manifest.get("pose_capture") is not True or manifest.get("noise_profile") != "noise-off-1000":
        raise ValueError("pose pairs require the declared capture profile")
    for name in ("navpy-pose.csv", "peer.json", "identity.json", "navpy-navigation.csv"):
        expected = manifest["evidence_sha256"].get(f"{case.name}/{name}")
        if hashlib.sha256((case / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"unbound pose experiment evidence: {name}")
    if peer != json.loads((case / "peer.json").read_text()):
        raise ValueError("pose pairs supplied a different peer")
    with (case / "navpy-pose.csv").open() as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != FIELDS:
            raise ValueError("pose CSV header mismatch")
        rows = list(reader)
    if len(rows) != WINDOW_SIZE or len(peer["records"]) != WINDOW_SIZE:
        raise ValueError("incomplete pose capture")
    snapshots = [Snapshot.decode(bytes.fromhex(record["snapshot"])) for record in peer["records"]]
    return rows, validate_pairs(rows, snapshots)
