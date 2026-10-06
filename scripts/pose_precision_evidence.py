"""Bind explicit rendering source to lossless producer values and each packet."""

import csv
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import struct

from pose_rounding_evidence import FIELDS as V1_FIELDS, PosePair, validate_pairs
from simtime_navigation_protocol import Snapshot, WINDOW_SIZE

PRECAST_FIELDS = ["precast_lat_hex", "precast_lng_hex", "precast_alt_hex"]
FIELDS = V1_FIELDS + ["pose_source"] + PRECAST_FIELDS


def bits(values: tuple[float, ...]) -> bytes:
    return struct.pack("<3d", *values)


def validate_precision(rows: list[dict], snapshots: list[Snapshot], source: str) -> list[PosePair]:
    if source not in ("rounded", "precast") or not rows or len(rows) != len(snapshots):
        raise ValueError("invalid/incomplete precision source capture")
    legacy_rows, legacy_snapshots = [], []
    for row, snapshot in zip(rows, snapshots):
        if (set(row) != set(FIELDS) or any(v is None for v in row.values()) or
                row["version"] != "2" or row["pose_source"] != source):
            raise ValueError("precision schema/source mismatch")
        legacy_rows.append({key: "1" if key == "version" else row[key] for key in V1_FIELDS})
        rounded = (int(row["rounded_lat"])*1e-7, int(row["rounded_lng"])*1e-7,
                   int(row["rounded_alt"])*1e-2)
        legacy_snapshots.append(replace(snapshot, truth=replace(snapshot.truth,
            latitude=rounded[0], longitude=rounded[1], altitude=rounded[2])))
    # Reuse the immutable v1 conversion, timestamp, branch and generation gates.
    pairs = validate_pairs(legacy_rows, legacy_snapshots)
    different = 0
    for row, snapshot, pair in zip(rows, snapshots, pairs):
        producer = tuple(float.fromhex(row[key]) for key in PRECAST_FIELDS)
        if bits(producer) != bits(pair.precast):
            raise ValueError("producer precast bits differ from conversion pair")
        actual = (snapshot.truth.latitude, snapshot.truth.longitude, snapshot.truth.altitude)
        expected = pair.precast if source == "precast" else pair.rounded
        if bits(actual) != bits(expected):
            raise ValueError("packet truth bits differ from declared pose source")
        different += bits(pair.rounded) != bits(pair.precast)
    if not different:
        raise ValueError("capture cannot distinguish rounded from precast source")
    return pairs


def read_precision(case: Path, peer: dict, source: str) -> tuple[list[dict], list[PosePair]]:
    manifest = json.loads((case.parent / "fleet.json").read_text())
    if (manifest.get("pose_capture") is not True or manifest.get("pose_source") != source or
            manifest.get("noise_profile") != "noise-off-1000"):
        raise ValueError("undeclared precision capture")
    for name in ("navpy-pose.csv", "peer.json", "identity.json", "navpy-navigation.csv"):
        expected = manifest["evidence_sha256"].get(f"{case.name}/{name}")
        if hashlib.sha256((case / name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"unbound precision evidence: {name}")
    if peer != json.loads((case / "peer.json").read_text()):
        raise ValueError("different precision peer")
    with (case / "navpy-pose.csv").open() as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != FIELDS:
            raise ValueError("precision CSV header mismatch")
        rows = list(reader)
    if len(rows) != WINDOW_SIZE or len(peer["records"]) != WINDOW_SIZE:
        raise ValueError("incomplete precision capture")
    snapshots = [Snapshot.decode(bytes.fromhex(record["snapshot"])) for record in peer["records"]]
    return rows, validate_precision(rows, snapshots, source)
