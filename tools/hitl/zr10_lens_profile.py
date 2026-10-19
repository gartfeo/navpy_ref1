"""The siyi_zr10 camera.reading_intrinsics section of vision_profiles.json: its form and an in-place rewrite."""
from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from navpy.core.perception.camera.reading_intrinsics import ReadingIntrinsicsTable

PROFILE = Path(__file__).resolve().parents[2] / "src/navpy/core/perception/profile/vision_profiles.json"


def section(rows: Sequence[Mapping[str, Any]], size: tuple[int, int]) -> dict:
    return {"source": "tools/hitl/zr10_lens_table.py from the private measurements (see its docstring); "
                      "basis 'interpolated' = centre and distortion interpolated between measured settings",
            "streams": [{"mode": "main", "image_width": size[0], "image_height": size[1],
                         "crop": [0, 0, size[0], size[1]], "rows": list(rows)}]}


def section_text(sec: Mapping[str, Any], indent: str) -> str:
    """The section as JSON with one row per line, indented to sit under its key."""
    stream = sec["streams"][0]
    head = {k: v for k, v in stream.items() if k != "rows"}
    lines = ["{", f'  "source": {json.dumps(sec["source"])},', '  "streams": [', "    {"]
    lines += [f'      {json.dumps(k)}: {json.dumps(v)},' for k, v in head.items()]
    lines.append('      "rows": [')
    lines += [f"        {json.dumps(r)}{',' if i < len(stream['rows']) - 1 else ''}"
              for i, r in enumerate(stream["rows"])]
    lines += ["      ]", "    }", "  ]", "}"]
    return ("\n" + indent).join(lines)


def write_profile(path: Path, sec: Mapping[str, Any]) -> None:
    """Replace the siyi_zr10 camera's reading_intrinsics in place, keeping the rest of the file and its line ends."""
    raw = open(path, encoding="utf-8", newline="").read()
    newline = "\r\n" if "\r\n" in raw else "\n"
    text = raw.replace("\r\n", "\n")
    original = json.loads(text)
    key = '"reading_intrinsics": '
    at = text.index(key, text.index('"name": "siyi_zr10"'))
    start = at + len(key)
    _, length = json.JSONDecoder().raw_decode(text[start:])
    indent = " " * (at - text.rindex("\n", 0, at) - 1)
    new_text = text[:start] + section_text(sec, indent) + text[start + length:]
    camera = next(d for d in original["profiles"]["siyi_zr10"]["devices"] if d["name"] == "siyi_zr10")["camera"]
    camera["reading_intrinsics"] = json.loads(json.dumps(sec))
    if json.loads(new_text) != original:
        raise AssertionError("the rewritten profile differs from the intended one")
    ReadingIntrinsicsTable.from_profile(camera["reading_intrinsics"])
    open(path, "w", encoding="utf-8", newline="").write(new_text.replace("\n", newline))
