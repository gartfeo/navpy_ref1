"""NPYC header byte fixtures shared with aas-hitl (HITL step 6).

tests/fixtures/npyc/npyc_headers.json is written from navpy's codec; aas-hitl
keeps a byte-identical copy (tests/video/fixtures/npyc_headers.json) and
decodes the same cases, so both decoders read Unreal's frames alike. The
sha256 below is pinned in both repositories: a change here must be copied
there. Regenerate after a deliberate codec change with:

    python -m tests.modules.vision.test_npyc_fixtures
"""

from __future__ import annotations

import hashlib
import json
import struct
from dataclasses import asdict
from pathlib import Path

import pytest

from navpy.plugins.camera.unreal.npyc_codec import (
    NPYC_MAX_DIMENSION,
    NPYC_PIXEL_FORMAT_BGR8,
    NPYC_PIXEL_FORMAT_BGRA8,
    NPYC_PREFIX_SIZE,
    NPYC_VERSION_1,
    NPYC_VERSION_2,
    POSE_EXACT,
    POSE_INTERPOLATED,
    POSE_STAMP_VALID,
    NpycHeader,
    decode_header,
    decode_prefix,
    encode_header,
)

FIXTURES = Path(__file__).resolve().parents[2] / "fixtures" / "npyc" / "npyc_headers.json"
FIXTURES_SHA256 = "805f532f57c0bad9061c74b421561ebd933dced75c49dea2e9249fcc0a3a819e"
U64_MAX = 2 ** 64 - 1


def _header(version, width, height, pixel_format, *, index=0, time_s=0.0, board_time_us=0, sequence=0, flags=0):
    bpp = 3 if pixel_format == NPYC_PIXEL_FORMAT_BGR8 else 4
    return NpycHeader(version, width, height, pixel_format, index, time_s, width * height * bpp,
                      board_time_us, sequence, flags)


VALID = {
    "v1_bgr8_720p": _header(NPYC_VERSION_1, 1280, 720, NPYC_PIXEL_FORMAT_BGR8, index=9, time_s=1.5),
    "v1_bgra8": _header(NPYC_VERSION_1, 4, 3, NPYC_PIXEL_FORMAT_BGRA8, index=1, time_s=12.25),
    "v2_exact": _header(NPYC_VERSION_2, 1280, 720, NPYC_PIXEL_FORMAT_BGR8, index=300, time_s=4021.125,
                        board_time_us=7_000_000, sequence=42, flags=POSE_STAMP_VALID | POSE_EXACT),
    "v2_interpolated": _header(NPYC_VERSION_2, 1920, 1080, NPYC_PIXEL_FORMAT_BGR8, index=301, time_s=4021.175,
                               board_time_us=7_050_000, sequence=43, flags=POSE_STAMP_VALID | POSE_INTERPOLATED),
    "v2_no_stamp": _header(NPYC_VERSION_2, 1280, 720, NPYC_PIXEL_FORMAT_BGR8, index=2),
    "v2_bgra8": _header(NPYC_VERSION_2, 4, 3, NPYC_PIXEL_FORMAT_BGRA8, index=3, board_time_us=1,
                        sequence=1, flags=POSE_STAMP_VALID | POSE_EXACT),
    "v2_u64_extremes": _header(NPYC_VERSION_2, 16, 16, NPYC_PIXEL_FORMAT_BGR8, index=U64_MAX,
                               board_time_us=U64_MAX, sequence=U64_MAX, flags=POSE_STAMP_VALID | POSE_EXACT),
    "v2_max_dimensions": _header(NPYC_VERSION_2, NPYC_MAX_DIMENSION, NPYC_MAX_DIMENSION, NPYC_PIXEL_FORMAT_BGR8),
    "v1_one_pixel": _header(NPYC_VERSION_1, 1, 1, NPYC_PIXEL_FORMAT_BGR8),
}

_V1 = struct.Struct("<4sHHHHHHQdI")


def _patch_v1(raw: bytes, **fields) -> bytes:
    """*raw* with some of its v1 fields replaced (the tail, if any, kept)."""
    names = ("magic", "version", "header_size", "width", "height", "pixel_format", "reserved", "frame_index",
             "time_s", "payload_size")
    values = dict(zip(names, _V1.unpack_from(raw)))
    values.update(fields)
    return _V1.pack(*(values[name] for name in names)) + raw[_V1.size:]


def _malformed() -> dict[str, tuple[bytes, str]]:
    v1 = encode_header(VALID["v1_bgr8_720p"])
    v2 = encode_header(VALID["v2_exact"])
    return {
        "bad_magic": (b"NPYD" + v2[4:], "magic"),
        "version_0": (_patch_v1(v1, version=0), "version"),
        "version_3": (_patch_v1(v2, version=3), "version"),
        "v1_claims_v2_size": (_patch_v1(v1, header_size=56), "header size"),
        "v2_claims_v1_size": (_patch_v1(v2, header_size=36), "header size"),
        "width_0": (_patch_v1(v2, width=0, payload_size=0), "out of range"),
        "height_0": (_patch_v1(v1, height=0, payload_size=0), "out of range"),
        "width_over_max": (_patch_v1(v2, width=NPYC_MAX_DIMENSION + 1, height=1,
                                     payload_size=(NPYC_MAX_DIMENSION + 1) * 3), "out of range"),
        "height_over_max": (_patch_v1(v1, width=1, height=NPYC_MAX_DIMENSION + 1,
                                      payload_size=(NPYC_MAX_DIMENSION + 1) * 3), "out of range"),
        "pixel_format_0": (_patch_v1(v1, pixel_format=0), "format"),
        "pixel_format_3": (_patch_v1(v2, pixel_format=3), "format"),
        "v1_payload_short": (_patch_v1(v1, payload_size=1280 * 720 * 3 - 1), "payload"),
        "v2_payload_long": (_patch_v1(v2, payload_size=1280 * 720 * 3 + 1), "payload"),
        "v2_bgra_size_with_bgr_format": (_patch_v1(v2, payload_size=1280 * 720 * 4), "payload"),
        "v2_header_truncated": (v2[:NPYC_PREFIX_SIZE + 10], "header has"),
        "v1_header_long": (v1 + b"\0", "header has"),
        "prefix_truncated": (v1[:NPYC_PREFIX_SIZE - 1], "prefix"),
    }


def build_fixtures() -> str:
    """The fixture file's exact text, from navpy's encoder."""
    document = {
        "about": "NPYC (Unreal native camera) headers from navpy's npyc_codec; shared byte-identical with aas-hitl.",
        "fields": "version, width, height, pixel_format (1 BGRA8, 2 BGR8), frame_index, capture_time_s, payload_size, "
                  "board_time_us, pose_sequence, pose_flags (v2; 0 for v1). 'refused' is a text both decoders' messages contain.",
        "valid": [{"name": name, "hex": encode_header(h).hex(), "header": asdict(h)} for name, h in VALID.items()],
        "malformed": [{"name": name, "hex": raw.hex(), "refused": reason}
                      for name, (raw, reason) in _malformed().items()],
    }
    return json.dumps(document, indent=1) + "\n"


def _load() -> dict:
    return json.loads(FIXTURES.read_text(encoding="utf-8"))


def test_the_fixture_file_is_navpys_encoder_output_and_pinned():
    data = FIXTURES.read_bytes()
    assert data.decode("utf-8") == build_fixtures()
    assert hashlib.sha256(data).hexdigest() == FIXTURES_SHA256


def _case(kind: str, name: str) -> dict:
    return next(case for case in _load()[kind] if case["name"] == name)


@pytest.mark.parametrize("name", list(VALID))
def test_every_valid_fixture_decodes_to_its_fields(name):
    case = _case("valid", name)
    raw = bytes.fromhex(case["hex"])
    assert decode_prefix(raw[:NPYC_PREFIX_SIZE]) == (case["header"]["version"], len(raw))
    assert asdict(decode_header(raw)) == case["header"]


@pytest.mark.parametrize("name", list(_malformed()))
def test_every_malformed_fixture_is_refused(name):
    case = _case("malformed", name)
    with pytest.raises(ValueError, match=case["refused"]):
        decode_header(bytes.fromhex(case["hex"]))


if __name__ == "__main__":
    FIXTURES.parent.mkdir(parents=True, exist_ok=True)
    FIXTURES.write_bytes(build_fixtures().encode("utf-8"))
    print(FIXTURES, hashlib.sha256(FIXTURES.read_bytes()).hexdigest())
